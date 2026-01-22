#include "EchemConsole.h"

#include <cstring>
#include <cstdlib>

// ------------------------ helpers ------------------------

int EchemConsole::StrIcmp(const char* s1, const char* s2) {
    if (s1 == nullptr) s1 = "";
    if (s2 == nullptr) s2 = "";
    while (*s1 && *s2) {
        char c1 = *s1;
        char c2 = *s2;
        if (c1 >= 'a' && c1 <= 'z') c1 = (char)(c1 - 'a' + 'A');
        if (c2 >= 'a' && c2 <= 'z') c2 = (char)(c2 - 'a' + 'A');
        if (c1 != c2) return (unsigned char)c1 - (unsigned char)c2;
        ++s1;
        ++s2;
    }
    return (unsigned char)*s1 - (unsigned char)*s2;
}

void EchemConsole::TrimInPlace(char* s) {
    if (!s) return;
    char* p = s;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (p != s) ::memmove(s, p, ::strlen(p) + 1);
    size_t n = ::strlen(s);
    while (n > 0) {
        const char ch = s[n - 1];
        if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
            s[n - 1] = '\0';
            --n;
        } else {
            break;
        }
    }
}

bool EchemConsole::TokenKeyEqualsI(const char* token, const char* key, const char** out_val_str) {
    if (!token || !key) return false;
    const char* eq = ::strchr(token, '=');
    if (!eq) return false;
    const size_t klen = (size_t)(eq - token);
    const size_t keylen = ::strlen(key);
    if (klen != keylen) return false;
    for (size_t i = 0; i < klen; ++i) {
        char c1 = token[i];
        char c2 = key[i];
        if (c1 >= 'a' && c1 <= 'z') c1 = (char)(c1 - 'a' + 'A');
        if (c2 >= 'a' && c2 <= 'z') c2 = (char)(c2 - 'a' + 'A');
        if (c1 != c2) return false;
    }
    if (out_val_str) *out_val_str = eq + 1;
    return true;
}

bool EchemConsole::ParseFloatKV(const char* token, const char* key, float* io_val) {
    if (!token || !key || !io_val) return false;
    const char* vstr = nullptr;
    if (!TokenKeyEqualsI(token, key, &vstr) || !vstr || !vstr[0]) return false;
    char* endp = nullptr;
    const float v = (float)::strtof(vstr, &endp);
    if (endp == vstr) return false;
    *io_val = v;
    return true;
}

bool EchemConsole::ParseU32KV(const char* token, const char* key, uint32_t* io_val) {
    if (!token || !key || !io_val) return false;
    const char* vstr = nullptr;
    if (!TokenKeyEqualsI(token, key, &vstr) || !vstr || !vstr[0]) return false;
    char* endp = nullptr;
    const unsigned long v = ::strtoul(vstr, &endp, 10);
    if (endp == vstr) return false;
    *io_val = (uint32_t)v;
    return true;
}

bool EchemConsole::ParseDirKV(const char* token, const char* key, NS_DAC::ScanDIR* io_dir) {
    if (!token || !key || !io_dir) return false;
    const char* vstr = nullptr;
    if (!TokenKeyEqualsI(token, key, &vstr) || !vstr) return false;
    if (StrIcmp(vstr, "FWD") == 0 || StrIcmp(vstr, "FORWARD") == 0) {
        *io_dir = NS_DAC::ScanDIR::FORWARD;
        return true;
    }
    if (StrIcmp(vstr, "REV") == 0 || StrIcmp(vstr, "REVERSE") == 0) {
        *io_dir = NS_DAC::ScanDIR::REVERSE;
        return true;
    }
    return false;
}

uint16_t EchemConsole::Clamp12U16(int32_t v) {
    if (v < 0) return 0;
    if (v > 4095) return 4095;
    return (uint16_t)v;
}

// ------------------------ public APIs ------------------------

EchemConsole::EchemConsole()
    : m_mode(NS_DAC::RunMode::CV)
    , m_cvVolt(0.8f, -0.8f, 1.65f)
    , m_cvParams(0.05f, 0.05f, NS_DAC::ScanDIR::FORWARD)
    , m_dpvParams()
    , m_biasCode(2048) {
}

void EchemConsole::ApplyCachedToController() {
    auto& sys = NS_DAC::SystemController::GetInstance();
    sys.SetMode(m_mode);
    sys.SetCVParams(m_cvVolt, m_cvParams);
    sys.SetDPVParams(m_dpvParams);
    sys.SetConstantVal(m_biasCode);
}

const char* EchemConsole::ModeToString(NS_DAC::RunMode mode) {
    switch (mode) {
    case NS_DAC::RunMode::CV:  return "CV";
    case NS_DAC::RunMode::DPV: return "DPV";
    case NS_DAC::RunMode::IT:  return "IT";
    default: return "?";
    }
}

void EchemConsole::PrintHelp(USART_Controller& usart) const {
    usart.Printf("Commands:\r\n");
    usart.Printf("  START | STOP | PAUSE | RESUME\r\n");
    usart.Printf("  HELP  | SHOW\r\n");
    usart.Printf("  MODE CV|DPV|IT\r\n");
    usart.Printf("  CV  HIGH=.. LOW=.. OFF=.. DUR=.. RATE=.. DIR=FWD|REV\r\n");
    usart.Printf("  DPV START=.. END=.. STEP=.. PULSE=.. PER=.. WIDTH=.. LEAD=.. OFF=..\r\n");
    usart.Printf("  IT  CODE=0..4095   (or) IT VABS=0..3.3\r\n");
    usart.Printf("Notes:\r\n");
    usart.Printf("  - Incremental update: fields not provided stay unchanged.\r\n");
    usart.Printf("  - If modified while running, changes take effect after STOP then START.\r\n");
    usart.Printf("  - Voltage units are in V (e.g., PULSE=0.05 means 50mV).\r\n");
}

void EchemConsole::PrintShow(USART_Controller& usart) const {
    usart.Printf("Mode=%s\r\n", ModeToString(m_mode));
    usart.Printf("CV  HIGH=%.3f LOW=%.3f OFF=%.3f DUR=%.4f RATE=%.4f DIR=%s\r\n",
        (double)m_cvVolt.highVolt,
        (double)m_cvVolt.lowVolt,
        (double)m_cvVolt.voltOffset,
        (double)m_cvParams.duration,
        (double)m_cvParams.rate,
        (m_cvParams.dir == NS_DAC::ScanDIR::FORWARD) ? "FWD" : "REV");

    usart.Printf("DPV START=%.3f END=%.3f STEP=%.4f PULSE=%.4f PER=%u WIDTH=%u LEAD=%u OFF=%.3f\r\n",
        (double)m_dpvParams.startVolt,
        (double)m_dpvParams.endVolt,
        (double)m_dpvParams.stepVolt,
        (double)m_dpvParams.pulseAmp,
        (unsigned)m_dpvParams.pulsePeriodMs,
        (unsigned)m_dpvParams.pulseWidthMs,
        (unsigned)m_dpvParams.sampleLeadMs,
        (double)m_dpvParams.midVolt);

    usart.Printf("BIAS CODE=%u\r\n", (unsigned)m_biasCode);
}

EchemConsole::State EchemConsole::ProcessLine(USART_Controller& usart, const char* line, State last_state, bool* out_reset_timebase) {
    if (out_reset_timebase) *out_reset_timebase = false;
    if (!line || !line[0]) return last_state;

    const bool is_running = (last_state == State::START || last_state == State::PAUSE || last_state == State::RESUME);

    char buf[160];
    ::strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    TrimInPlace(buf);
    if (buf[0] == '\0') return last_state;

    // Tokenize: CMD [ARG...], separators: space/tab/comma
    char* cmd = ::strtok(buf, "\t ,");
    if (!cmd) return last_state;

    // HELP/SHOW
    if (StrIcmp(cmd, "HELP") == 0) {
        PrintHelp(usart);
        return last_state;
    }
    if (StrIcmp(cmd, "SHOW") == 0) {
        PrintShow(usart);
        return last_state;
    }

    // START/STOP/PAUSE/RESUME
    if (StrIcmp(cmd, "START") == 0) {
        if (is_running) {
            usart.Printf("START ignored: already running.\r\n");
            return last_state;
        }
        usart.Printf("Starting...\r\n");
        NS_DAC::SystemController::GetInstance().Start();
        if (out_reset_timebase) *out_reset_timebase = true;
        return State::START;
    }
    if (StrIcmp(cmd, "STOP") == 0) {
        usart.Printf("Stopping...\r\n");
        NS_DAC::SystemController::GetInstance().Stop();
        // Ensure cached parameters are pushed into SystemController for next run.
        ApplyCachedToController();
        return State::STOP;
    }
    if (StrIcmp(cmd, "PAUSE") == 0) {
        if (last_state != State::START && last_state != State::RESUME) {
            usart.Printf("Error: PAUSE only valid after START/RESUME.\r\n");
            return last_state;
        }
        usart.Printf("Paused.\r\n");
        NS_DAC::SystemController::GetInstance().Pause();
        return State::PAUSE;
    }
    if (StrIcmp(cmd, "RESUME") == 0) {
        if (last_state != State::PAUSE) {
            usart.Printf("RESUME ignored: device is not paused.\r\n");
            return last_state;
        }
        usart.Printf("Resumed.\r\n");
        NS_DAC::SystemController::GetInstance().Resume();
        return State::RESUME;
    }

    // MODE
    if (StrIcmp(cmd, "MODE") == 0) {
        char* m = ::strtok(nullptr, "\t ,");
        if (!m) {
            usart.Printf("Error: MODE requires CV|DPV|IT\r\n");
            return last_state;
        }
        if (StrIcmp(m, "CV") == 0)      m_mode = NS_DAC::RunMode::CV;
        else if (StrIcmp(m, "DPV") == 0) m_mode = NS_DAC::RunMode::DPV;
        else if (StrIcmp(m, "IT") == 0)  m_mode = NS_DAC::RunMode::IT;
        else {
            usart.Printf("Error: unknown MODE=%s\r\n", m);
            return last_state;
        }

        if (!is_running) {
            ApplyCachedToController();
            usart.Printf("MODE set to %s\r\n", m);
        } else {
            usart.Printf("MODE updated (will take effect after STOP then START).\r\n");
        }
        return last_state;
    }

    // CV params
    if (StrIcmp(cmd, "CV") == 0) {
        char* t = nullptr;
        while ((t = ::strtok(nullptr, "\t ,")) != nullptr) {
            ParseFloatKV(t, "HIGH", &m_cvVolt.highVolt);
            ParseFloatKV(t, "LOW",  &m_cvVolt.lowVolt);
            ParseFloatKV(t, "OFF",  &m_cvVolt.voltOffset);
            ParseFloatKV(t, "DUR",  &m_cvParams.duration);
            ParseFloatKV(t, "RATE", &m_cvParams.rate);
            ParseDirKV(t,   "DIR",  &m_cvParams.dir);
        }

        // basic guards
        if (m_cvVolt.highVolt < m_cvVolt.lowVolt) {
            const float tmp = m_cvVolt.highVolt;
            m_cvVolt.highVolt = m_cvVolt.lowVolt;
            m_cvVolt.lowVolt = tmp;
        }
        if (m_cvParams.duration <= 0.0f) m_cvParams.duration = 0.001f;
        if (m_cvParams.rate <= 0.0f) m_cvParams.rate = 0.001f;

        if (!is_running) ApplyCachedToController();
        usart.Printf("CV params updated%s\r\n", is_running ? " (apply after STOP/START)" : "");
        return last_state;
    }

    // DPV params
    if (StrIcmp(cmd, "DPV") == 0) {
        uint32_t tmp_u32 = 0;
        char* t = nullptr;
        while ((t = ::strtok(nullptr, "\t ,")) != nullptr) {
            ParseFloatKV(t, "START", &m_dpvParams.startVolt);
            ParseFloatKV(t, "END",   &m_dpvParams.endVolt);
            ParseFloatKV(t, "STEP",  &m_dpvParams.stepVolt);
            ParseFloatKV(t, "PULSE", &m_dpvParams.pulseAmp);
            if (ParseU32KV(t, "PER", &tmp_u32))   m_dpvParams.pulsePeriodMs = (uint16_t)tmp_u32;
            if (ParseU32KV(t, "WIDTH", &tmp_u32)) m_dpvParams.pulseWidthMs  = (uint16_t)tmp_u32;
            if (ParseU32KV(t, "LEAD", &tmp_u32))  m_dpvParams.sampleLeadMs  = (uint16_t)tmp_u32;
            ParseFloatKV(t, "OFF",   &m_dpvParams.midVolt);
        }

        // guards
        if (m_dpvParams.pulsePeriodMs == 0) m_dpvParams.pulsePeriodMs = 1;
        if (m_dpvParams.pulseWidthMs == 0)  m_dpvParams.pulseWidthMs = 1;
        if (m_dpvParams.pulseWidthMs >= m_dpvParams.pulsePeriodMs) {
            m_dpvParams.pulseWidthMs = (m_dpvParams.pulsePeriodMs > 1) ? (uint16_t)(m_dpvParams.pulsePeriodMs - 1) : 1;
        }

        // base hold time in ms
        const uint16_t base_ms = (m_dpvParams.pulsePeriodMs > m_dpvParams.pulseWidthMs)
            ? (uint16_t)(m_dpvParams.pulsePeriodMs - m_dpvParams.pulseWidthMs)
            : 1;
        if (m_dpvParams.sampleLeadMs == 0) m_dpvParams.sampleLeadMs = 1;
        if (m_dpvParams.sampleLeadMs >= base_ms) {
            m_dpvParams.sampleLeadMs = 1;
        }
        if (m_dpvParams.stepVolt == 0.0f) {
            m_dpvParams.stepVolt = 0.001f;
        }

        if (!is_running) ApplyCachedToController();
        usart.Printf("DPV params updated%s\r\n", is_running ? " (apply after STOP/START)" : "");
        return last_state;
    }

    // IT / BIAS
    if (StrIcmp(cmd, "IT") == 0 || StrIcmp(cmd, "BIAS") == 0) {
        uint32_t tmp_u32 = 0;
        float vabs = 0.0f;
        char* t = nullptr;
        while ((t = ::strtok(nullptr, "\t ,")) != nullptr) {
            if (ParseU32KV(t, "CODE", &tmp_u32)) {
                m_biasCode = Clamp12U16((int32_t)tmp_u32);
            }
            if (ParseFloatKV(t, "VABS", &vabs)) {
                if (vabs < 0.0f) vabs = 0.0f;
                if (vabs > 3.3f) vabs = 3.3f;
                const float code_f = (vabs / 3.3f) * 4095.0f;
                m_biasCode = Clamp12U16((int32_t)(code_f + 0.5f));
            }
        }

        if (!is_running) ApplyCachedToController();
        usart.Printf("BIAS updated%s\r\n", is_running ? " (apply after STOP/START)" : "");
        return last_state;
    }

    usart.Printf("Unknown command: %s. Use HELP.\r\n", cmd);
    return last_state;
}
