#include "DACManager.h"
#include "ADCManager.h" // 需引用外部 ADC
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_dac.h"
#include "GPIO.h"

namespace NS_DAC {

    // =========================================================
    // 0. 硬件映射表
    // =========================================================
    struct TIM_DMA_Mapping {
        TIM_TypeDef* TIMx;
        uint16_t DMA_Request;
        DMA_Channel_TypeDef* DMA_Channel;
    };
    struct TIM_DAC_Mapping {
        TIM_TypeDef* TIMx;
        uint32_t trigger;
    };

    const TIM_DMA_Mapping timDmaMap[] = {
        {TIM2, TIM_DMA_Update, DMA1_Channel2},
        {TIM3, TIM_DMA_Update, DMA1_Channel3},
        {TIM4, TIM_DMA_Update, DMA1_Channel7},
#ifdef TIM5
        {TIM5, TIM_DMA_Update, DMA2_Channel2},
#endif
#ifdef TIM6
        {TIM6, TIM_DMA_Update, DMA1_Channel3},
#endif
#ifdef TIM7
        {TIM7, TIM_DMA_Update, DMA2_Channel4}
#endif
    };

    const TIM_DAC_Mapping timDacMap[] = {
        {TIM2, DAC_Trigger_T2_TRGO},
        {TIM3, DAC_Trigger_T3_TRGO},
        {TIM4, DAC_Trigger_T4_TRGO},
#ifdef TIM5
        {TIM5, DAC_Trigger_T5_TRGO},
#endif
#ifdef TIM6
        {TIM6, DAC_Trigger_T6_TRGO},
#endif
#ifdef TIM7
        {TIM7, DAC_Trigger_T7_TRGO}
#endif
    };

    // =========================================================
    // DAC_ChanController
    // =========================================================
    DAC_ChanController::DAC_ChanController(const HW_Config& cfg) {
        ResolveHardware(cfg);
    }

    void DAC_ChanController::ResolveHardware(const HW_Config& cfg) {
        hw.dacChan = cfg.dacChan;
        hw.tim = cfg.tim;

        if (cfg.tim == nullptr) {
            hw.dacTrigger = DAC_Trigger_Software;
            hw.dmaChan = nullptr;
            hw.timDmaSrc = TIM_DMA_Update;
            return;
        }

        // 查找 DAC Trigger
        hw.dacTrigger = DAC_Trigger_Software;
        for (const auto& map : timDacMap) {
            if (map.TIMx == cfg.tim) { hw.dacTrigger = map.trigger; break; }
        }

        // 查找 DMA Channel
        hw.dmaChan = nullptr;
        hw.timDmaSrc = TIM_DMA_Update;
        for (const auto& map : timDmaMap) {
            if (map.TIMx == cfg.tim) { hw.dmaChan = map.DMA_Channel; break; }
        }
    }

    void DAC_ChanController::InitAsCV(const CV_VoltParams& v, const CV_Params& c) {
        dataMgr.SetupCV(v, c);
        dataMgr.SwitchMode(GenMode::CV_SCAN);
        useDMA = true;
    }

    void DAC_ChanController::InitAsDPV(const DPV_Params& d) {
        dataMgr.SetupDPV(d);
        dataMgr.SwitchMode(GenMode::DPV_PULSE);
        useDMA = false; // DPV：定时中断驱动，手写 DAC
    }

    void DAC_ChanController::InitAsConstant(uint16_t val) {
        dataMgr.SetupConstant(val);
        dataMgr.SwitchMode(GenMode::CONSTANT);
        useDMA = false; // 常量：直接一次写入
    }

    void DAC_ChanController::SetupGPIO() {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_InitTypeDef gpio;
        gpio.GPIO_Mode = GPIO_Mode_AIN;
        gpio.GPIO_Pin = (hw.dacChan == DAC_Channel::CH1) ? GPIO_Pin_4 : GPIO_Pin_5;
        GPIO_Init(GPIOA, &gpio);
    }

    void DAC_ChanController::SetupDAC() {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
        DAC_InitTypeDef dac;
        DAC_StructInit(&dac);

        // DMA 模式：硬件触发；非 DMA 模式：软件触发
        dac.DAC_Trigger = useDMA ? hw.dacTrigger : DAC_Trigger_Software;
        dac.DAC_OutputBuffer = DAC_OutputBuffer_Disable;

        DAC_Init((uint32_t)hw.dacChan, &dac);
        DAC_Cmd((uint32_t)hw.dacChan, ENABLE);

        if (useDMA) {
            DAC_DMACmd((uint32_t)hw.dacChan, ENABLE);
        } else {
            DAC_DMACmd((uint32_t)hw.dacChan, DISABLE);
        }
    }

    void DAC_ChanController::SetupDMA() {
        if (!useDMA || hw.dmaChan == nullptr) return;

        if (hw.dmaChan >= DMA2_Channel1) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
        else RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        DMA_DeInit(hw.dmaChan);
        DMA_InitTypeDef dma;
        dma.DMA_MemoryBaseAddr = (uint32_t)dataMgr.GetDMAAddr();
        dma.DMA_PeripheralBaseAddr = (hw.dacChan == DAC_Channel::CH1)
            ? (uint32_t)&DAC->DHR12R1
            : (uint32_t)&DAC->DHR12R2;

        dma.DMA_DIR = DMA_DIR_PeripheralDST;
        dma.DMA_BufferSize = 1;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Disable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_High;
        dma.DMA_M2M = DMA_M2M_Disable;

        DMA_Init(hw.dmaChan, &dma);
        DMA_Cmd(hw.dmaChan, ENABLE);
    }

    void DAC_ChanController::SetupTIM(float period) {
        if (hw.tim == nullptr) return;

        // Robust TIM init: reset registers and clear flags/counter each start.
        // This avoids edge cases where the first START outputs only mid-code (2048)
        // and the waveform begins only after STOP/START.
        if (hw.tim == TIM2) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
        if (hw.tim == TIM3) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        if (hw.tim == TIM4) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

        TIM_DeInit(hw.tim);

        TIM_TimeBaseInitTypeDef t;
        TIM_TimeBaseStructInit(&t);
        t.TIM_Prescaler = 7200 - 1; // 10kHz tick (0.1ms)
        // Clamp to [1, 65536] ticks to prevent underflow/overflow.
        unsigned int ticks = (unsigned int)(period * 10000.0f + 0.5f);
        if (ticks < 1u) ticks = 1u;
        if (ticks > 65536u) ticks = 65536u;
        t.TIM_Period = (uint16_t)(ticks - 1u);
        t.TIM_ClockDivision = TIM_CKD_DIV1;
        t.TIM_CounterMode = TIM_CounterMode_Up;

        TIM_TimeBaseInit(hw.tim, &t);
        TIM_SelectOutputTrigger(hw.tim, TIM_TRGOSource_Update);

        // Enable update interrupt (used for CV/DPV waveform stepping).
        TIM_ITConfig(hw.tim, TIM_IT_Update, ENABLE);

        TIM_SetCounter(hw.tim, 0);
        TIM_ClearITPendingBit(hw.tim, TIM_IT_Update);
        TIM_Cmd(hw.tim, DISABLE);
    }

    void DAC_ChanController::Start() {
        SetupGPIO();
        SetupDAC();
        SetupDMA();

        const GenMode mode = dataMgr.GetMode();

        // 1) 先把当前值写入 DAC（避免首次周期输出为旧值）
        const uint16_t initVal = dataMgr.GetCurrentData();
        if (hw.dacChan == DAC_Channel::CH1) {
            DAC_SetChannel1Data(DAC_Align_12b_R, initVal);
        } else {
            DAC_SetChannel2Data(DAC_Align_12b_R, initVal);
        }

        if (!useDMA) {
            // 软件触发立即更新输出
            DAC_SoftwareTriggerCmd((uint32_t)hw.dacChan, ENABLE);
        }

        // 2) 决定是否需要启用定时器
        const bool needTim = (hw.tim != nullptr) && (mode == GenMode::CV_SCAN || mode == GenMode::DPV_PULSE);

        if (needTim) {
            float period = 0.001f; // default 1ms (DPV)
            if (mode == GenMode::CV_SCAN) {
                period = dataMgr.GetCV().cvParams.duration;
                if (period <= 0.0f) period = 0.001f;
            }
            SetupTIM(period);

            // Clear any stale pending state BEFORE enabling.
            TIM_SetCounter(hw.tim, 0);
            TIM_ClearITPendingBit(hw.tim, TIM_IT_Update);
            NVIC_ClearPendingIRQ(TIM_IRQnManage::GetIRQn(hw.tim, TIM::IT::UP));

            // Enable timer, then force one UPDATE event to kick the trigger chain once.
            TIM_Cmd(hw.tim, ENABLE);
            TIM_GenerateEvent(hw.tim, TIM_EventSource_Update);
        }

        isPaused = false;
    }

    void DAC_ChanController::Stop() {
        if (hw.tim) TIM_Cmd(hw.tim, DISABLE);
        DAC_Cmd((uint32_t)hw.dacChan, DISABLE);
        if (useDMA && hw.dmaChan) DMA_Cmd(hw.dmaChan, DISABLE);
    }

    void DAC_ChanController::Pause() {
        if (isPaused) return;
        if (hw.tim) TIM_Cmd(hw.tim, DISABLE);
        isPaused = true;
    }

    void DAC_ChanController::Resume() {
        if (!isPaused) return;
        if (hw.tim) TIM_Cmd(hw.tim, ENABLE);
        isPaused = false;
    }

    void DAC_ChanController::TIM_IRQHandler() {
        // 注意：TIM_IRQnManage 已经完成“标志位判断 + 清除”
        const bool changed = dataMgr.UpdateNextStep();

        // 非 DMA：仅在值发生变化时写 DAC（减小 SPI/OLED 干扰与抖动）
        if (!useDMA && changed) {
            const uint16_t val = dataMgr.GetCurrentData();
            if (hw.dacChan == DAC_Channel::CH1)
                DAC_SetChannel1Data(DAC_Align_12b_R, val);
            else
                DAC_SetChannel2Data(DAC_Align_12b_R, val);

            DAC_SoftwareTriggerCmd((uint32_t)hw.dacChan, ENABLE);
        }
    }

    // =========================================================
    // 全局实例
    // =========================================================
    // Scan：TIM2；Constant：不占用定时器，避免与 ADC 显示定时器冲突
    DAC_ChanController DAC_Manager::Chan_Scan({DAC_Channel::CH2, TIM2});
    DAC_ChanController DAC_Manager::Chan_Constant({DAC_Channel::CH1, nullptr});

    void DAC_Manager::Init() {}

    // =========================================================
    // SystemController
    // =========================================================
    SystemController& SystemController::GetInstance() {
        static SystemController instance;
        return instance;
    }

    void SystemController::SetMode(RunMode mode) { if (!isRunning) currentMode = mode; }
    void SystemController::SetCVParams(const CV_VoltParams& v, const CV_Params& c) { cachedCV_Volt = v; cachedCV_Params = c; }
    void SystemController::SetDPVParams(const DPV_Params& d) { cachedDPV_Params = d; }
    void SystemController::SetConstantVal(uint16_t val) {
        // 兼容旧接口：同时设置 scan/bias
        cachedScanConstantVal = val;
        cachedBiasConstantVal = val;
    }

    void SystemController::SetScanConstantVal(uint16_t val) { cachedScanConstantVal = val; }
    void SystemController::SetBiasConstantVal(uint16_t val) { cachedBiasConstantVal = val; }

    void SystemController::Start() {
        // Start ADC first
        NS_ADC::GetStaticADC().StartConversion();

        // Init Scan channel based on cached params
        switch (currentMode) {
            case RunMode::CV:
                DAC_Manager::Chan_Scan.InitAsCV(cachedCV_Volt, cachedCV_Params);
                break;
            case RunMode::DPV:
                DAC_Manager::Chan_Scan.InitAsDPV(cachedDPV_Params);
                break;
            case RunMode::IT:
                DAC_Manager::Chan_Scan.InitAsConstant(cachedScanConstantVal);
                break;
        }

        // Bias channel: constant output
        DAC_Manager::Chan_Constant.InitAsConstant(cachedBiasConstantVal);

        // IMPORTANT: register/enable TIM2 update IRQ BEFORE starting the timer.
        // This avoids a first-run edge case where Code12 stays at mid-code (2048)
        // until the user performs STOP/START again.
        if (currentMode == RunMode::CV || currentMode == RunMode::DPV) {
            TIM_IRQnManage::Add(TIM2, TIM::IT::UP, [](){ DAC_Manager::Chan_Scan.TIM_IRQHandler(); }, 1, 1);
            NVIC_ClearPendingIRQ(TIM_IRQnManage::GetIRQn(TIM2, TIM::IT::UP));
        }

        DAC_Manager::Chan_Scan.Start();
        DAC_Manager::Chan_Constant.Start();

        // Kick one UPDATE event after everything is running (safe even if redundant).
        if (currentMode == RunMode::CV || currentMode == RunMode::DPV) {
            TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
        }

        isRunning = true;
        isPaused = false;
        ClearTick();
    }

    void SystemController::Stop() {
        DAC_Manager::Chan_Scan.Stop();
        DAC_Manager::Chan_Constant.Stop();
        isRunning = false;
        isPaused = false;
    }

    void SystemController::Pause() {
        DAC_Manager::Chan_Scan.Pause();
        NS_ADC::GetStaticADC().Pause();
        isPaused = true;
    }

    void SystemController::Resume() {
        DAC_Manager::Chan_Scan.Resume();
        NS_ADC::GetStaticADC().Resume();
        isPaused = false;
    }

    const uint16_t& GetCvValToSendRef() {
        const volatile uint16_t* ptr = DAC_Manager::Chan_Scan.GetDataMgr().GetDMAAddr();
        return const_cast<const uint16_t&>(*ptr);
    }

    uint8_t ConsumeDpvSampleFlags() {
        return DAC_Manager::Chan_Scan.GetDataMgr().ConsumeDpvSampleFlags();
    }

} // namespace NS_DAC
