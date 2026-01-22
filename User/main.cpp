#include "DACManager.h"
#include "ADCManager.h"
#include "BTCPP.h"
#include "SysTickTimer.h"

#include "main.h"
#include "EchemConsole.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/**
 * 【关键修复】安全的行读取函数
 * 每次只从底层读取一个字节，确保不会因为一次读多而丢弃缓冲区里剩余的命令。
 */
static bool TryReadCommandLine(USART_Controller& usart, char* outLine, uint16_t outCap) {
    static char lineBuf[96];
    static uint16_t lineLen = 0;
    
    uint8_t ch;
    // 循环读取，直到硬件缓冲区空或读到完整的一行
    while (usart.Receiver(&ch, 1) > 0) {
        // 遇到换行符，说明一行结束
        if (ch == '\r' || ch == '\n') {
            if (lineLen > 0) {
                lineBuf[lineLen] = '\0';
                if (outCap > 0) {
                    strncpy(outLine, lineBuf, outCap - 1);
                    outLine[outCap - 1] = '\0';
                }
                lineLen = 0;
                return true; // 成功读取一行
            }
            // 如果 lineLen == 0，说明是空行（比如 \r 后的 \n），直接忽略，继续读下一个字符
            continue;
        }

        // 普通字符，存入缓冲区
        if (lineLen < sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = (char)ch;
        } else {
            // 缓冲区溢出，重置
            lineLen = 0;
            usart.Printf("Error: Line buffer overflow\r\n");
        }
    }
    return false; // 暂时没有读到完整的一行
}

static void SendJsonLine(USART_Controller& usart,
                         uint32_t ms,
                         uint16_t uric_raw,
                         uint16_t ascorbic_raw,
                         uint16_t glucose_raw,
                         uint16_t code12)
{
    char outBuf[160];
    int n = snprintf(outBuf, sizeof(outBuf),
        "{\"Ms\":%lu,\"Uric\":%u,\"Ascorbic\":%u,\"Glucose\":%u,\"Code12\":%u}\n",
        (unsigned long)ms,
        (unsigned)uric_raw,
        (unsigned)ascorbic_raw,
        (unsigned)glucose_raw,
        (unsigned)code12
    );
    if (n <= 0) return;
    if (n >= (int)sizeof(outBuf)) n = (int)sizeof(outBuf) - 1;
    usart.Write((const uint8_t*)outBuf, (uint16_t)n);
}

int main(void) {
    SysTickTimer::Init();
    NVIC_SetPriority(SysTick_IRQn, 0);

    OLED_Init();

    auto& bt = GetStaticBt();
    // 确保中断优先级配置正确，防止丢数据
    USART_IRQnManage::Add(bt.GetParams().USART, USART::IT::RXNE, BT_IRQHandler, 1, 3);
    bt.Start();

    ApplyDefaultParams();

    EchemConsole console;

    bt.Printf("System Ready.\r\n");

    EchemConsole::State state = EchemConsole::State::UNKNOWN;
    uint32_t startTime = 0;
    bool resetTimebase = false;

    // --- 第一阶段：等待 START ---
    while (state != EchemConsole::State::START) {
        char line[64];
        if (TryReadCommandLine(bt, line, sizeof(line))) {
            // 【调试】回显收到的命令，方便在手机端查看是否收到
            bt.Printf("ACK: %s\r\n", line);
            
            state = console.ProcessLine(bt, line, state, &resetTimebase);
            
            if (state == EchemConsole::State::START) {
                startTime = SysTickTimer::GetTick();
            }
        }
        // 降低延时，提高响应速度，防止数据积压
        SysTickTimer::DelayMs(5); 
    }

    auto& adc = NS_ADC::GetStaticADC();
    const auto& adcBuf = adc.GetDmaBufferRef();
    const auto& cvValRef = NS_DAC::GetCvValToSendRef();

    uint32_t lastReportTime = 0;
    constexpr uint32_t REPORT_INTERVAL_MS = 50;

    // --- 第二阶段：主循环 ---
    while (1) {
        // 1. 处理命令
        char line[64];
        // 这里使用 while 循环处理所有积压的命令，防止发送 JSON 阻塞导致命令处理不及时
        while (TryReadCommandLine(bt, line, sizeof(line))) {
            bt.Printf("ACK: %s\r\n", line); // 回显
            
            EchemConsole::State newState = console.ProcessLine(bt, line, state, &resetTimebase);
            
            if (resetTimebase) {
                startTime = SysTickTimer::GetTick();
            }
            state = newState;
        }

        // 2. 硬件服务
        adc.Service();

        // 3. 数据上报
        const uint32_t now = SysTickTimer::GetTick();
        if (now - lastReportTime >= REPORT_INTERVAL_MS) {
            lastReportTime = now;

            if (state == EchemConsole::State::START || state == EchemConsole::State::RESUME) {
                NS_DAC::SystemController::GetInstance().UpdateTick();

                const uint32_t ms = now - startTime;
                const uint16_t uric_raw     = adcBuf[0];
                const uint16_t ascorbic_raw = adcBuf[1];
                const uint16_t glucose_raw  = adcBuf[2];
                const uint16_t code12 = ((uint16_t)cvValRef) & 0x0FFF;

                SendJsonLine(bt, ms, uric_raw, ascorbic_raw, glucose_raw, code12);
            }
        }
        
        // 稍微减少延时，保证串口接收更流畅
        SysTickTimer::DelayMs(2);
    }
}