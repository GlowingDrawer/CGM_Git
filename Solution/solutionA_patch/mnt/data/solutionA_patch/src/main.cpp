#include "main.h"
#include "DACManager.h"
#include "ADCManager.h"
#include "BTCPP.h"
#include "SysTickTimer.h"
#include "IRQnManage.h"
#include "EchemConsole.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Non-blocking line reader: read bytes from USART ring buffer,
// assemble one line terminated by '\r' or '\n'.
static bool TryReadCommandLine(USART_Controller& usart, char* outLine, uint16_t outCap)
{
    static char lineBuf[96];
    static uint16_t lineLen = 0;
    static bool dropping = false;

    uint8_t rxTmp[64];
    uint16_t n = usart.Receiver(rxTmp, (uint16_t)sizeof(rxTmp));
    if (n == 0) return false;

    for (uint16_t i = 0; i < n; ++i) {
        char ch = (char)rxTmp[i];

        if (ch == '\r' || ch == '\n') {
            if (dropping) {
                dropping = false;
                lineLen = 0;
                continue;
            }
            if (lineLen == 0) continue;

            lineBuf[lineLen] = '\0';

            if (outCap > 0) {
                strncpy(outLine, lineBuf, outCap - 1);
                outLine[outCap - 1] = '\0';
            }
            lineLen = 0;
            return true;
        }

        if (dropping) continue;

        if (lineLen < (uint16_t)(sizeof(lineBuf) - 1)) {
            lineBuf[lineLen++] = ch;
        } else {
            dropping = true;
            lineLen = 0;
            usart.Printf("CMD buffer overflow, dropping until newline.\r\n");
        }
    }

    return false;
}

static void SendJsonLine(USART_Controller& usart,
                         uint32_t ms,
                         const char* modeStr,
                         uint16_t uric_raw,
                         uint16_t ascorbic_raw,
                         uint16_t glucose_raw,
                         uint16_t code12,
                         uint8_t mark)
{
    char outBuf[200];
    int n = snprintf(outBuf, sizeof(outBuf),
        "{\"Ms\":%lu,\"Mode\":\"%s\",\"Uric\":%u,\"Ascorbic\":%u,"
        "\"Glucose\":%u,\"Code12\":%u,\"Mark\":%u}\n",
        (unsigned long)ms,
        (modeStr ? modeStr : "?"),
        (unsigned)uric_raw,
        (unsigned)ascorbic_raw,
        (unsigned)glucose_raw,
        (unsigned)code12,
        (unsigned)mark);

    if (n <= 0) return;
    if (n >= (int)sizeof(outBuf)) n = (int)sizeof(outBuf) - 1;

    usart.Write((const uint8_t*)outBuf, (uint16_t)n);
}

int main(void)
{
    SysTickTimer::Init();
    NVIC_SetPriority(SysTick_IRQn, 0);

    OLED_Init();

    auto& bt = GetStaticBt();
    USART_IRQnManage::Add(bt.GetParams().USART, USART::IT::RXNE, BT_IRQHandler, 1, 3);
    bt.Start();

    // Command processor / cached config manager
    EchemConsole console;
    console.ApplyCachedToController();

    bt.Printf("Ready. Use HELP for commands.\r\n");
    bt.Printf("Waiting for START...\r\n");

    EchemConsole::State state = EchemConsole::State::UNKNOWN;
    uint32_t startTime = 0;

    // Wait for START
    while (state != EchemConsole::State::START) {
        char line[96];
        if (TryReadCommandLine(bt, line, (uint16_t)sizeof(line))) {
            bool resetTb = false;
            state = console.ProcessLine(bt, line, state, &resetTb);
            if (resetTb) startTime = SysTickTimer::GetTick();
        }
        SysTickTimer::DelayMs(20);
    }

    auto& adc = NS_ADC::GetStaticADC();
    const auto& adcBuf = adc.GetDmaBufferRef();
    const auto& codeRef = NS_DAC::GetCvValToSendRef();

    uint32_t lastReport = 0;
    const uint32_t REPORT_MS = 50;

    while (1) {
        // 1) handle commands
        char line[96];
        if (TryReadCommandLine(bt, line, (uint16_t)sizeof(line))) {
            bool resetTb = false;
            EchemConsole::State newState = console.ProcessLine(bt, line, state, &resetTb);
            if (resetTb && (state == EchemConsole::State::STOP || state == EchemConsole::State::UNKNOWN)) {
                startTime = SysTickTimer::GetTick();
            }
            state = newState;
        }

        // 2) OLED refresh must remain in main loop
        adc.Service();

        // 3) periodic report
        uint32_t now = SysTickTimer::GetTick();
        if ((uint32_t)(now - lastReport) >= REPORT_MS) {
            lastReport = now;

            if (state == EchemConsole::State::START || state == EchemConsole::State::RESUME) {
                uint32_t ms = now - startTime;

                uint16_t uric_raw     = adcBuf[0];
                uint16_t ascorbic_raw = adcBuf[1];
                uint16_t glucose_raw  = adcBuf[2];

                uint16_t code12 = ((uint16_t)codeRef) & 0x0FFF;
                uint8_t  mark   = NS_DAC::ConsumeDpvSampleFlags();

                SendJsonLine(bt, ms,
                             EchemConsole::ModeToString(console.GetMode()),
                             uric_raw, ascorbic_raw, glucose_raw,
                             code12, mark);
            }
        }

        SysTickTimer::DelayMs(10);
    }
}
