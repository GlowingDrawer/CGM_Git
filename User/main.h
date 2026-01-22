#pragma once

// 这个头文件是为了让 main.cpp 更干净。
// 只建议在 C++ 源文件（.cpp）里 include，
// 不要让 .c 文件 include 到它。

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "DACManager.h"
#include "BTCPP.h"

#define WE_uA_Port GPIO_Pin_1       // GPIOA_Pin_1
#define WE_mA_Port GPIO_Pin_2       // GPIOA_Pin_2

// ============================================================
// Route-A (Defaults-only) command state
// ============================================================
enum class CommandState : uint8_t {
    UNKNOWN = 0,
    START,
    PAUSE,
    RESUME,
    STOP
};

// ============================================================
// Small utils
// ============================================================
static inline int my_stricmp(const char* s1, const char* s2) {
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

static inline void trim_inplace(char* s) {
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

static inline const char* ModeToString(NS_DAC::RunMode m) {
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
static inline DPV_Params MakeDefaultDPV() {
    DPV_Params p;
    // Example overrides:
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

static inline uint16_t VoltToCode12(float v, float vref = 3.3f) {
    if (v < 0.0f) v = 0.0f;
    if (v > vref) v = vref;
    int32_t code = (int32_t)(v / vref * 4095.0f + 0.5f);
    if (code < 0) code = 0;
    if (code > 4095) code = 4095;
    return (uint16_t)code;
}

static inline void ApplyDefaultParams() {
    auto& sys = NS_DAC::SystemController::GetInstance();
    sys.SetMode(kDefaultMode);
    sys.SetCVParams(kDefaultCVVolt, kDefaultCVParams);
    sys.SetDPVParams(MakeDefaultDPV());

    // 依赖你已打过补丁的 DACManager.*（split constants）
    sys.SetScanConstantVal(kDefaultITScanCode);
    sys.SetBiasConstantVal(kDefaultBiasCode);
}

// ============================================================
// CLI outputs
// ============================================================
static inline void PrintHelp(USART_Controller& usart) {
    usart.Printf(
        "Route-A firmware (defaults-only)\r\n"
        "Commands:\r\n"
        "  HELP | SHOW\r\n"
        "  START | STOP | PAUSE | RESUME\r\n"
        "Notes:\r\n"
        "  - CV/DPV/IT parameters are compiled defaults (edit main_app.h and re-flash).\r\n"
        "  - IT Scan constant and Bias constant are independent in this build.\r\n"
    );
}

static inline void PrintDefaults(USART_Controller& usart) {
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

// ============================================================
// Route-A command handler (runtime only START/STOP/PAUSE/RESUME)
// ============================================================
static inline CommandState ProcessCommandLine(USART_Controller& usart, const char* line, CommandState current) {
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
        usart.Printf("Stopping...\r\n");
        NS_DAC::SystemController::GetInstance().Stop();
        return CommandState::STOP;
    }

    if (my_stricmp(cmd, "PAUSE") == 0) {
        if (current != CommandState::START && current != CommandState::RESUME) {
            usart.Printf("Error: PAUSE only valid after START/RESUME.\r\n");
            return current;
        }
        usart.Printf("Paused.\r\n");
        NS_DAC::SystemController::GetInstance().Pause();
        return CommandState::PAUSE;
    }

    if (my_stricmp(cmd, "RESUME") == 0) {
        if (current != CommandState::PAUSE) {
            usart.Printf("Resume ignored. Device is not paused.\r\n");
            return current;
        }
        usart.Printf("Resumed.\r\n");
        NS_DAC::SystemController::GetInstance().Resume();
        return CommandState::RESUME;
    }

    usart.Printf("Unknown command. Use HELP.\r\n");
    return current;
}



