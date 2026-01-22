#include "main.h"
#include "DACManager.h"
#include "ADCManager.h"
#include "BTCPP.h"
#include "SysTickTimer.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ============================================================
// Route A (Defaults-only):
//   - Modify DEFAULT electrochemical parameters in this file.
//   - Re-flash the firmware.
//   - No runtime parameter update is supported in this main.
//
// Runtime commands supported:
//   HELP | SHOW
//   START | STOP | PAUSE | RESUME
// ============================================================

enum class CommandState : uint8_t {
    UNKNOWN = 0,
    START,
    PAUSE,
    RESUME,
    STOP
};

static int my_stricmp(const char* s1, const char* s2) {
    if (!s1 || !s2) return (s1 == s2) ? 0 : (s1 ? 1 : -1);
    while (*s1 && *s2) {
        char c1 = *s1;
        char c2 = *s2;
        if (c1 >= 'a' && c1 <= 'z') c1 = (char)(c1 - 'a' + 'A');
        if (c2 >= 'a' && c2 <= 'z') c2 = (char)(c2 - 'a' + 'A');
        if (c1 != c2) return (int)(unsigned char)c1 - (int)(unsigned char)c2;
        ++s1; ++s2;
    }
    return (int)(unsigned char)*s1 - (int)(unsigned char)*s2;
}

static void trim_inplace(char* s) {
    if (!s) return;
    char* p = s;
    while (*p == ' ' || *p == '\t') ++p;
    if (p != s) memmove(s, p, strlen(p) + 1);

    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) {
        s[n - 1] = '\0';
        --n;
    }
}

// Non-blocking line reader (terminated by \r or \n)
static bool TryReadCommandLine(USART_Controller& usart, char* outLine, uint16_t outCap) {
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

static const char* ModeToString(NS_DAC::RunMode m) {
    switch (m) {
        case NS_DAC::RunMode::CV:  return "CV";
        case NS_DAC::RunMode::DPV: return "DPV";
        case NS_DAC::RunMode::IT:  return "IT";
        default: return "?";
    }
}

// ============================================================
// DEFAULT electrochemical parameters (edit here)
// ============================================================

// Default mode after power-on
static constexpr NS_DAC::RunMode kDefaultMode = NS_DAC::RunMode::CV;

// CV: relative potentials (V) w.r.t. mid; OFF is absolute mid bias voltage (V)
static const NS_DAC::CV_VoltParams kDefaultCVVolt(0.8f, -0.8f, 1.65f);
// CV: duration(s) per step/timer period, rate(V/s), direction
static const NS_DAC::CV_Params kDefaultCVParams(0.05f, 0.05f, NS_DAC::ScanDIR::FORWARD);

// DPV: use DPV_Params default in DPVController.h, or override below
static DPV_Params MakeDefaultDPV() {
    DPV_Params p;
    // Example overrides (uncomment if needed):
    // p.startVolt = -0.5f;
    // p.endVolt   =  0.5f;
    // p.stepVolt  =  0.005f;
    // p.pulseAmp  =  0.05f;
    // p.pulsePeriodMs = 50;
    // p.pulseWidthMs  = 10;
    // p.sampleLeadMs  = 1;
    // p.midVolt       = 1.65f;
    return p;
}

// IT (Scan channel constant) and Bias (CH1 constant) are independent
static constexpr uint16_t kDefaultITScanCode = 2048; // 0..4095
static constexpr uint16_t kDefaultBiasCode   = 2048; // 0..4095

static void ApplyDefaultParams() {
    auto& sys = NS_DAC::SystemController::GetInstance();
    sys.SetMode(kDefaultMode);
    sys.SetCVParams(kDefaultCVVolt, kDefaultCVParams);
    sys.SetDPVParams(MakeDefaultDPV());

    // Requires the patched DACManager.* that supports split constants.
    sys.SetScanConstantVal(kDefaultITScanCode);
    sys.SetBiasConstantVal(kDefaultBiasCode);
}

static void PrintHelp(USART_Controller& usart) {
    usart.Printf(
        "Route-A firmware (defaults-only)\r\n"
        "Commands:\r\n"
        "  HELP | SHOW\r\n"
        "  START | STOP | PAUSE | RESUME\r\n"
        "Notes:\r\n"
        "  - CV/DPV/IT parameters are compiled defaults (edit main.cpp and re-flash).\r\n"
        "  - IT Scan constant and Bias constant are independent in this build.\r\n"
    );
}

static void PrintDefaults(USART_Controller& usart) {
    DPV_Params dpv = MakeDefaultDPV();
    usart.Printf("DefaultMode=%s\r\n", ModeToString(kDefaultMode));
    usart.Printf("CV  HIGH=%.3f LOW=%.3f OFF=%.3f DUR=%.4f RATE=%.4f DIR=%s\r\n",
        (double)kDefaultCVVolt.highVolt,
        (double)kDefaultCVVolt.lowVolt,
        (double)kDefaultCVVolt.voltOffset,
        (double)kDefaultCVParams.duration,
        (double)kDefaultCVParams.rate,
        (kDefaultCVParams.dir == NS_DAC::ScanDIR::FORWARD) ? "FWD" : "REV"
    );
    usart.Printf("DPV START=%.3f END=%.3f STEP=%.4f PULSE=%.4f PER=%u WIDTH=%u LEAD=%u OFF=%.3f\r\n",
        (double)dpv.startVolt,
        (double)dpv.endVolt,
        (double)dpv.stepVolt,
        (double)dpv.pulseAmp,
        (unsigned)dpv.pulsePeriodMs,
        (unsigned)dpv.pulseWidthMs,
        (unsigned)dpv.sampleLeadMs,
        (double)dpv.midVolt
    );
    usart.Printf("IT  ScanCode=%u  BiasCode=%u\r\n",
        (unsigned)kDefaultITScanCode,
        (unsigned)kDefaultBiasCode
    );
}

static CommandState ProcessCommandLine(USART_Controller& usart, const char* line, CommandState current) {
    if (!line || !line[0]) return current;

    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    trim_inplace(buf);
    if (!buf[0]) return current;

    char* cmd = strtok(buf, " \t,");
    if (!cmd) return current;

    if (my_stricmp(cmd, "HELP") == 0) {
        PrintHelp(usart);
        return current;
    }
    if (my_stricmp(cmd, "SHOW") == 0) {
        PrintDefaults(usart);
        usart.Printf("CurrentMode=%s\r\n", ModeToString(NS_DAC::SystemController::GetInstance().GetMode()));
        return current;
    }

    const bool isRunning = (current == CommandState::START || current == CommandState::PAUSE || current == CommandState::RESUME);

    if (my_stricmp(cmd, "START") == 0) {
        if (isRunning) {
            usart.Printf("Already running. Use STOP first.\r\n");
            return current;
        }
        usart.Printf("Starting...\r\n");
        NS_DAC::SystemController::GetInstance().Start();
        return CommandState::START;
    }

    if (my_stricmp(cmd, "STOP") == 0) {
        if (!isRunning && current != CommandState::STOP) {
            // allow STOP even in UNKNOWN
        }
        usart.Printf("Stopping...\r\n");
        NS_DAC::SystemController::GetInstance().Stop();
        return CommandState::STOP;
    }

    if (my_stricmp(cmd, "PAUSE") == 0) {
        if (current != CommandState::START && current != CommandState::RESUME) {
            usart.Printf("PAUSE ignored (not running).\r\n");
            return current;
        }
        usart.Printf("Paused.\r\n");
        NS_DAC::SystemController::GetInstance().Pause();
        return CommandState::PAUSE;
    }

    if (my_stricmp(cmd, "RESUME") == 0) {
        if (current != CommandState::PAUSE) {
            usart.Printf("RESUME ignored (not paused).\r\n");
            return current;
        }
        usart.Printf("Resumed.\r\n");
        NS_DAC::SystemController::GetInstance().Resume();
        return CommandState::RESUME;
    }

    usart.Printf("Unknown command. Use HELP.\r\n");
    return current;
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
        "{\"Ms\":%lu,\"Mode\":\"%s\",\"Uric\":%u,\"Ascorbic\":%u,\"Glucose\":%u,\"Code12\":%u,\"Mark\":%u}\n",
        (unsigned long)ms,
        modeStr,
        (unsigned)uric_raw,
        (unsigned)ascorbic_raw,
        (unsigned)glucose_raw,
        (unsigned)code12,
        (unsigned)mark
    );
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

    // Apply DEFAULT parameters before any START.
    ApplyDefaultParams();

    bt.Printf("Route-A ready. Use HELP. Waiting START...\r\n");

    CommandState state = CommandState::UNKNOWN;
    uint32_t startTime = 0;

    // Wait for START
    while (state != CommandState::START) {
        char line[96];
        if (TryReadCommandLine(bt, line, sizeof(line))) {
            CommandState newState = ProcessCommandLine(bt, line, state);
            if (newState == CommandState::START && (state == CommandState::UNKNOWN || state == CommandState::STOP)) {
                startTime = SysTickTimer::GetTick();
            }
            state = newState;
        }
        SysTickTimer::DelayMs(20);
    }

    auto& adc = NS_ADC::GetStaticADC();
    const auto& adcBuf = adc.GetDmaBufferRef();
    const auto& codeRef = NS_DAC::GetCvValToSendRef();

    uint32_t lastReport = 0;
    const uint32_t REPORT_MS = 50;

    while (1) {
        // Commands
        char line[96];
        if (TryReadCommandLine(bt, line, sizeof(line))) {
            CommandState newState = ProcessCommandLine(bt, line, state);
            if (newState == CommandState::START && (state == CommandState::UNKNOWN || state == CommandState::STOP)) {
                startTime = SysTickTimer::GetTick();
            }
            state = newState;
        }

        // OLED refresh & ADC UI in main loop
        adc.Service();

        // Periodic report
        const uint32_t now = SysTickTimer::GetTick();
        if ((now - lastReport) >= REPORT_MS) {
            lastReport = now;

            if (state == CommandState::START || state == CommandState::RESUME) {
                // Optional internal tick for legacy logic
                NS_DAC::SystemController::GetInstance().UpdateTick();

                const uint32_t ms = now - startTime;
                const uint16_t uric_raw     = adcBuf[0];
                const uint16_t ascorbic_raw = adcBuf[1];
                const uint16_t glucose_raw  = adcBuf[2];
                const uint16_t code12 = ((uint16_t)codeRef) & 0x0FFF;
                const uint8_t  mark   = NS_DAC::ConsumeDpvSampleFlags();

                SendJsonLine(bt, ms, ModeToString(NS_DAC::SystemController::GetInstance().GetMode()),
                             uric_raw, ascorbic_raw, glucose_raw, code12, mark);
            }
        }

        SysTickTimer::DelayMs(10);
    }
}
