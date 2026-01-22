#pragma once

#include <cstdint>

#include "DACManager.h"
#include "DPVController.h"
#include "BTCPP.h"   // USART_Controller

// Command processor + cached configuration for CV/DPV/IT.
class EchemConsole {
public:
    enum class State : uint8_t {
        UNKNOWN = 0,
        START,
        PAUSE,
        RESUME,
        STOP
    };

    EchemConsole();

    // Apply cached parameters into NS_DAC::SystemController.
    void ApplyCachedToController();

    void PrintHelp(USART_Controller& usart) const;
    void PrintShow(USART_Controller& usart) const;

    // Process one command line. May call Start/Stop/Pause/Resume.
    // out_reset_timebase will be set to true if a fresh START should reset time base.
    State ProcessLine(USART_Controller& usart, const char* line, State last_state, bool* out_reset_timebase);

    NS_DAC::RunMode GetMode() const { return m_mode; }
    uint16_t GetBiasCode() const { return m_biasCode; }

    static const char* ModeToString(NS_DAC::RunMode mode);

private:
    NS_DAC::RunMode m_mode;
    NS_DAC::CV_VoltParams m_cvVolt;
    NS_DAC::CV_Params m_cvParams;
    DPV_Params m_dpvParams;
    uint16_t m_biasCode;

    static int StrIcmp(const char* s1, const char* s2);
    static void TrimInPlace(char* s);
    static bool TokenKeyEqualsI(const char* token, const char* key, const char** out_val_str);
    static bool ParseFloatKV(const char* token, const char* key, float* io_val);
    static bool ParseU32KV(const char* token, const char* key, uint32_t* io_val);
    static bool ParseDirKV(const char* token, const char* key, NS_DAC::ScanDIR* io_dir);
    static uint16_t Clamp12U16(int32_t v);
};
