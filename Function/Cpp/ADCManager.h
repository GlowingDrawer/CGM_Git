// ADC.hpp
#pragma once
#include "stm32f10x.h"
#include <array>
#include <InitArg.h>
#include "CustomWaveCPP.h"

namespace NS_ADC
{

    enum class Mode:uint32_t { INDEPENDENT = ADC_Mode_Independent, REG_INJEC_SIMULT = ADC_Mode_RegInjecSimult, REG_SIMULT_ALT_TRIG = ADC_Mode_RegSimult_AlterTrig, INJEC_SIMULT_FAST_INTERL = ADC_Mode_InjecSimult_FastInterl, INJEC_SIMULT_SLOW_INTERL = ADC_Mode_InjecSimult_SlowInterl, INJEC_SIMULT = ADC_Mode_InjecSimult, REG_SIMULT = ADC_Mode_RegSimult, FAST_INTERL = ADC_Mode_FastInterl, SLOW_INTERL = ADC_Mode_SlowInterl, ALTER_TRIG = ADC_Mode_AlterTrig };

    struct ShowParams
    {
        TIM_TypeDef* TIMx = nullptr;
        float period = 0.05;    // 显示周期，单位秒
        TIM::IT tim_it = TIM::IT::UP;

        ShowParams() = default;
        ShowParams(TIM_TypeDef* TIMx, float period = 0.05f, TIM::IT tim_it = TIM::IT::UP) : TIMx(TIMx), tim_it(tim_it) {
            this->period = MyCompare<float>(period, 1);
        }
    };

    // 初始化参数结构体
    struct InitParams {
        ADC_TypeDef* adc = ADC1;                        // ADC外设（ADC1/ADC2）
        Mode mode = Mode::INDEPENDENT;                  // 工作模式
        FunctionalState scan_mode = ENABLE;             // 扫描模式
        FunctionalState cont_mode = ENABLE;             // 连续模式
        uint8_t nbr_of_channels = 2;                    // 通道数量
        CGM::AD_ChanParams* channels = nullptr;         // 通道配置数组
        uint32_t clock_prescaler = RCC_PCLK2_Div6;      // 时钟分频

        InitParams() = default;
        InitParams(ADC_TypeDef* adc, Mode mode, FunctionalState scan_mode, FunctionalState cont_mode, uint8_t nbr_of_channels, CGM::AD_ChanParams* channels, uint32_t clock_prescaler) :
            adc(adc), mode(mode), scan_mode(scan_mode), cont_mode(cont_mode), nbr_of_channels(nbr_of_channels), channels(channels), clock_prescaler(clock_prescaler) {}
    };

    class ADC {
    public:
        static const float stepPerVolt;
        
        void ResetVoltRef(uint16_t ref_val) { this->staticRefVal = ref_val; }
        void ResetVoltRef(float volt_ref) { this->staticRefVal = volt_ref * ADC::stepPerVolt; }
        void ResetVoltRef(const NS_DAC::CV_Controller &cv_controller) { this->staticRefVal = cv_controller.GetCvParams().initVal; }
        void SetRefValBuf(const NS_DAC::CV_Controller &cv_controller) { this->dynRefValBuf = cv_controller.GetBufferPtr(); }

        // 构造函数
        explicit ADC(const InitParams& params, ShowParams show_params = ShowParams(), const uint16_t ref_val = 1249,const uint16_t * ref_val_buf = nullptr, CGM::VsMode vs_mode = CGM::VsMode::STATIC);
        explicit ADC(const InitParams& params, ShowParams show_params, const NS_DAC::CV_Controller &cv_controller) : ADC(params, show_params, cv_controller.GetCvParams().initVal, cv_controller.GetBufferPtr()) { }
        
        // 初始化ADC
        void Init();
        
        // 启动常规通道转换
        void StartConversion();
        void Pause();           // 新增：暂停功能
        void Resume();          // 新增：重新开始功能
        bool IsPaused() const { return isPaused; }  // 新增：获取暂停状态（可选）
        
        // 获取转换值（阻塞模式）
        uint16_t GetValue(uint8_t channel);
        
        // 获取DMA模式下的数据指针
        const uint16_t* GetDmaBufferHeader() const { return &dmaBuf[0]; }
        const std::array<uint16_t, 16> &GetDmaBufferRef() const { return dmaBuf; }
        const double * GetCurrentBufHeader() const { return &currentBuf[0]; }
        const std::array<double, 16> &GetCurrentBufRef() const { return currentBuf; }

        const InitParams & GetInitParams() const { return params; }
        DMA_Channel_TypeDef * GetDmaChannel() { return this->dmaChannel; }
        const ShowParams & GetShowParams() const { return showParams; }

        
        // 校准ADC
        void Calibrate();

        double ShowVoltage(uint8_t line, uint8_t col, uint16_t ad_value, uint16_t ref_val, uint32_t gain);
        void ShowBoardVal(uint8_t index);
        void ShowBoardVal();
        void Show();

        void TIM_IRQnHandler(void);
        void DMA_IRQnHandler(void);

        // 在主循环中调用：如果需要则刷新 OLED（建议 10Hz）
        void Service();

        void ResetVsMode(CGM::VsMode vs_mode) {
            if (this->vsMode != vs_mode) {
                this->vsMode = vs_mode;
                refValBuf = (vs_mode == CGM::VsMode::STATIC) ? &this->staticRefVal : this->dynRefValBuf; 
            }
        }
        
    private:
        volatile bool needOledRefresh = false;
        std::array<uint16_t, 16> snapBuf{};

        bool isPaused = false;  // 标记是否处于暂停状态
        TIM_TypeDef* showTim = nullptr;   // 已有的定时器记录变量（用于暂停/恢复）

        CGM::VsMode vsMode = CGM::VsMode::STATIC;
        const InitParams params;
        ShowParams showParams;
        ADC_TypeDef* adc;
        DMA_Channel_TypeDef  *dmaChannel;
        uint16_t staticRefVal = 1249;
        const uint16_t * dynRefValBuf = nullptr;
        const uint16_t * refValBuf = &staticRefVal;

        std::array<uint16_t, 16> maxVal, minVal;

        // 存储adc通道读取的12Bit值
        alignas(4) std::array<uint16_t, 16> dmaBuf; // DMA缓冲区（最多支持16通道）
        // 存储adc通道对应的电流值大小(单位：mA)
        std::array<double, 16> currentBuf;

        uint32_t lastOledRefreshTime = 0;
        
        void GpioConfig();
        void DmaConfig();
        void ShowConfig();
    };

    // class ADC_Controller{
    //     public:
    //     static ADC adc;
    // };

    // NS_ADC::InitParams& CGMParams2AdcInitParams(CGM::Params &params, NS_ADC::InitParams &init_params);

    // extern CGM::Params adcParams;
    // extern NS_ADC::InitParams adcInitParams;

    // inline ADC GetADC(){ 
    //     static NS_ADC::ADC adc(adcInitParams, NS_ADC::ShowParams(TIM3, 0.1), NS_DAC::DAC_Controller::dacChanCV.Get_CV_Controller());
    //     return adc;
    // }

} // namespace NS_ADC