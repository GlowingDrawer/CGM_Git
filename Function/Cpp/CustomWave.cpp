#include "stm32f10x.h"                  // Device header
#include "CustomWaveCPP.h"
#include "stm32f10x_tim.h"
#include "GPIO.h"
#include "Delay.h"
#include "misc.h"
#include "math.h"
#include "BTCPP.h"
#include <InitArg.h>
#include "ADCManager.h"
#include "stdio.h"

/* This code follows the following naming conventions.
Const:  UPPER_SNAKE_CASE
Enum:   PascalCase / ABBR_Case / Abbr_Case  {UPPER_SNAKE_CASE}
Class： PascalCase / ABBR_Case / Abbr_Case
Arg:    snake_name              
Var:    camelCase
Func:   PascalCase (bool Func:  isPascalCase / hasPascalCase)
namespace:  NS_PascalCase / NS_ABBR
Bool var:   is_snake_name
*/

const uint8_t DEBUG_LINE = 4;
const uint8_t DEBUG_COW = 1;


namespace NS_DMA {

    enum class AddrMode:uint8_t { MEMORY, PERIPH};

    void ChannelSetAddr(AddrMode addr_mode ,DMA_Channel_TypeDef* DMA_Channel, uint32_t mem_addr, uint32_t data_count) {
        // 禁用全局中断，防止在配置过程中被中断打断
        DMA_ITConfig(DMA_Channel, DMA_IT_TC | DMA_IT_HT, DISABLE);
        
        // 停止DMA通道
        DMA_Cmd(DMA_Channel, DISABLE);
        
        // 设置新的内存地址和传输数据数量
        if (addr_mode == AddrMode::MEMORY) {
            DMA_Channel->CMAR = mem_addr;
        } else if (addr_mode == AddrMode::PERIPH) {
            DMA_Channel->CPAR = mem_addr;
        }
        DMA_Channel->CNDTR = data_count;  // 重置传输数量
        
        // 重新启用DMA通道
        DMA_Cmd(DMA_Channel, ENABLE);
        
        // 恢复全局中断
        DMA_ITConfig(DMA_Channel, DMA_IT_TC | DMA_IT_HT, ENABLE);
    }

}

namespace NS_DAC {

    void DAC_Cmd(DAC_Channel channel, FunctionalState state){
        DAC_Cmd(static_cast<uint32_t>(channel), state);
    }

    void DAC_Controller::Init(DAC_Channel cv_channel){
        this->cv_channel = cv_channel;
        if (this->cv_channel == DAC_Channel::CH1) {
            this->constant_channel = DAC_Channel::CH2;
        };
    }
    DAC_Controller::DAC_Controller(DAC_Channel cv_channel) {
        this->Init(cv_channel);
    }

    DAC_Controller::~DAC_Controller()
    {
        
    }

    void DAC_Controller::SetCV_Channel(DAC_Channel cv_channel){
        if (cv_channel == DAC_Channel::CH1) {
            this->cv_channel = DAC_Channel::CH1;
            this->constant_channel = DAC_Channel::CH2;
        } else if (cv_channel == DAC_Channel::CH2) {
            this->cv_channel = DAC_Channel::CH2;
            this->constant_channel = DAC_Channel::CH1;
        }
    }

    // Class CV_Controller
    // // DAC 每伏特电压对应的步长值（1240.91 steps/V）
    const constexpr float CV_Controller::dacStepsPerVolt = 1240.91f;

    void CV_Controller::UpdateScanStep(void){
        this->cvParams.scanStep12Bit = this->dacStepsPerVolt * this->cvParams.voltScanRate * this->cvParams.scanDuration;
    }

    void CV_Controller::InitCVParams(const CV_VoltParams& volt_params, float scan_duration, float volt_scan_rate, ScanDIR scan_dir){
        float cvMaxVolt = volt_params.highVolt + volt_params.voltOffset;
        float cvMinVolt = volt_params.lowVolt + volt_params.voltOffset;
        float initVolt = volt_params.voltOffset;
        // DAC进行CV时的最大和最小值
        this->cvParams.maxVal = To_uint16(cvMaxVolt * this->dacStepsPerVolt);
        this->cvParams.minVal = To_uint16(cvMinVolt * this->dacStepsPerVolt);
        this->cvParams.initVal = To_uint16(initVolt * this->dacStepsPerVolt);
        this->currentVal = initVolt * this->dacStepsPerVolt;
        this->bufA[0] = To_uint16(this->currentVal);
        this->bufPtr = this->bufA;
    
        this->cvParams.scanDuration = scan_duration;
        this->cvParams.voltScanRate = volt_scan_rate;
        this->cvParams.scanDIR = scan_dir;
        // 初始化扫描步进值
        this->cvParams.scanStep12Bit = this->dacStepsPerVolt * this->cvParams.voltScanRate * this->cvParams.scanDuration;
        if (this->cvParams.scanDIR == ScanDIR::REVERSE) {
            this->cvParams.scanStep12Bit = -this->cvParams.scanStep12Bit;
        }
    }

    CV_Controller::CV_Controller(CV_VoltParams volt_params, float scan_duration, float volt_scan_rate, ScanDIR scan_dir, 
        WaveForm wave_form, DAC_UpdateMode dac_update_mode) : dacUpdateMode(dac_update_mode) {
        OLED_Init();
        this->waveForm = wave_form;
        InitCVParams(volt_params, scan_duration, volt_scan_rate, scan_dir);
    }

    CV_Controller::CV_Controller(const CV_VoltParams& volt_params, const CV_Params& cv_params, WaveForm wave_form, DAC_UpdateMode dac_update_mode) 
        : CV_Controller(volt_params, cv_params.scanDuration, cv_params.voltScanRate, cv_params.scanDIR, wave_form, dac_update_mode) {
    }

    uint16_t CV_Controller::GetValToSend(void){
        uint16_t result = 0;
        if (this->waveForm == WaveForm::CV_FLUCTUATE) {
            result = this->bufPtr[0];
        } else if (this->waveForm == WaveForm::CV_CONSTANT) {
            result = this->cvParams.initVal;
        }
        return result;
    }

    void CV_Controller::ResetParam(const CV_VoltParams& volt_params){
        float cvMaxVolt = volt_params.highVolt + volt_params.voltOffset;
        float cvMinVolt = volt_params.lowVolt + volt_params.voltOffset;
        float initVolt = volt_params.voltOffset;
        // DAC进行CV时的最大和最小值
        this->cvParams.maxVal = To_uint16(cvMaxVolt * this->dacStepsPerVolt);
        this->cvParams.minVal = To_uint16(cvMinVolt * this->dacStepsPerVolt);
        this->cvParams.initVal = To_uint16(initVolt * this->dacStepsPerVolt);
        this->currentVal = initVolt * this->dacStepsPerVolt;
        this->bufPtr[0] = To_uint16(this->currentVal);
    }

    void CV_Controller::ResetParam(const CV_VoltParams& volt_params, float scan_duration, float volt_scan_rate, 
        ScanDIR scan_dir){
        ResetParam(volt_params);

        this->cvParams.scanDuration = scan_duration;
        this->cvParams.voltScanRate = volt_scan_rate;
        this->cvParams.scanDIR = scan_dir;
        // DAC进行CV时的步进值
        this->cvParams.scanStep12Bit = this->dacStepsPerVolt * this->cvParams.voltScanRate * this->cvParams.scanDuration;
        if (this->cvParams.scanDIR == ScanDIR::REVERSE) {
            this->cvParams.scanStep12Bit = - this->cvParams.scanStep12Bit;
        }
        
    }

    void CV_Controller::UpdateCurrentVal(void){
        this->currentVal += this->cvParams.scanStep12Bit;
        if (this->currentVal >= this->cvParams.maxVal) {
            this->cvParams.scanStep12Bit = -fabs(this->cvParams.scanStep12Bit);
            this->currentVal = this->cvParams.maxVal;
        } else if (this->currentVal <= this->cvParams.minVal) {
            this->cvParams.scanStep12Bit = fabs(this->cvParams.scanStep12Bit);
            this->currentVal = this->cvParams.minVal;
        }

        this->bufPtr[0] = To_uint16(this->currentVal);
        // this->bufPtr[0] = dacStepsPerVolt * 0.6;
    }

    void CV_Controller::ReStoreDefault(void){
        this->currentVal = this->cvParams.initVal;
        this->bufPtr[0] = To_uint16(this->currentVal);
    }

    void CV_Controller::ShowCv(uint16_t delay_ms){
        OLED_ShowString(4, 1, "CV:");
        OLED_ShowNum(4, 4, To_uint16(currentVal), 4);
    }

    // Class WaveArr
    // WaveArr Init
    bool WaveArr::isInit = false;

    const constexpr std::array<uint16_t, To_uint16(BufSize::BUF_END) - 1>WaveStaticParams::s_TriangleArr = {0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480, 512, 544,   576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 960, 992, 1024, 1056, 1088, 1120, 1152, 1184, 1216, 1248, 1280, 1312, 1344, 1376, 1408, 1440, 1472, 1504, 1536, 1568, 1600, 1632, 1664, 1696, 1728, 1760, 1792, 1824, 1856, 1888, 1920, 1952, 1984, 2016, 2048, 2080, 2112, 2144, 2176, 2208, 2240, 2272, 2304, 2336, 2368, 2400, 2432, 2464, 2496, 2528, 2560, 2592, 2624, 2656, 2688, 2720, 2752, 2784, 2816, 2848, 2880, 2912, 2944, 2976, 3008, 3040, 3072, 3104, 3136, 3168, 3200, 3232, 3264, 3296, 3328, 3360, 3392, 3424, 3456, 3488, 3520, 3552, 3584, 3616, 3648, 3680, 3712, 3744, 3776, 3808, 3840, 3872, 3904, 3936, 3968, 4000, 4032, 4064, 4095, 4064, 4032, 4000, 3968, 3936, 3904, 3872, 3840, 3808, 3776, 3744, 3712, 3680, 3648, 3616, 3584, 3552, 3520, 3488, 3456, 3424, 3392, 3360, 3328, 3296, 3264, 3232, 3200, 3168, 3136, 3104, 3072, 3040, 3008, 2976, 2944, 2912, 2880, 2848, 2816, 2784, 2752, 2720, 2688, 2656, 2624, 2592, 2560, 2528, 2496, 2464, 2432, 2400, 2368, 2336, 2304, 2272, 2240, 2208, 2176, 2144, 2112, 2080, 2048, 2016, 1984, 1952, 1920, 1888, 1856, 1824, 1792, 1760, 1728, 1696, 1664, 1632, 1600, 1568, 1536, 1504, 1472, 1440, 1408, 1376, 1344, 1312, 1280, 1248, 1216, 1184, 1152, 1120, 1088, 1056, 1024, 992, 960, 928, 896, 864, 832, 800, 768, 736, 704, 672, 640, 608, 576, 544, 512, 480, 448, 416, 384, 352, 320, 288, 256, 224, 192, 160, 128, 96, 64, 32 };
    const constexpr std::array<uint16_t, To_uint16(BufSize::BUF_END) - 1>WaveStaticParams::s_SineArr = {2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784, 2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143, 3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459, 3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722, 3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919, 3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045, 4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094, 4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065, 4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958, 3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777, 3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530, 3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226, 3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877, 2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496, 2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098, 2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697, 1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311, 1264, 1218, 1172, 1127, 1082, 1038, 995, 952, 910, 869, 828, 788, 749, 710, 672, 636, 600, 565, 530, 497, 465, 433, 403, 373, 345, 318, 291, 266, 242, 219, 197, 176, 156, 137, 120, 103, 88, 74, 61, 50, 39, 30, 22, 15, 10, 6, 2, 1, 0, 1, 2, 6, 10, 15, 22, 30, 39, 50, 61, 74, 88, 103, 120, 137, 156, 176, 197, 219, 242, 266, 291, 318, 345, 373, 403, 433, 465, 497, 530, 565, 600, 636, 672, 710, 749, 788, 828, 869, 910, 952, 995, 1038, 1082, 1127, 1172, 1218, 1264, 1311, 1358, 1405, 1453, 1501, 1550, 1599, 1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997 };
    
    WaveStaticParams WaveArr::s_Params = WaveStaticParams();
    
    WaveArr::WaveArr(WaveForm wave_form, BufSize buf_size){
        this->dynParams.bufSize = To_uint16(buf_size);
        this->waveForm = wave_form;
        InitArr();
    }

    void WaveArr::InitBuffer(const uint16_t* sourceArr, float gain, int offset, uint16_t interval) {
        for (int i = 0, j = 0; i < To_uint16(BufSize::BUF_END) - 1; i += interval, j++) {
            this->dynParams.buffer[j] = gain * sourceArr[i] + offset;
        }
    }
    
    bool WaveArr::InitArr(WaveForm wave_form){
        bool result = true;
        
        float gain = 1.0f;
        auto den = int(dynParams.maxVal) - int(dynParams.minVal);
        if (den == 0) { gain = 1.0f; /*或返回false*/ }
        else gain = float(int(s_Params.maxVal)-int(s_Params.minVal)) / float(den);


        // float gain = (WaveArr::s_Params.maxVal - WaveArr::s_Params.minVal) / (this->dynParams.maxVal - this->dynParams.minVal);
        int offset = this->dynParams.minVal - WaveArr::s_Params.minVal;
        uint16_t interval = (To_uint16(BufSize::BUF_END) - 1 ) / this->dynParams.bufSize;
        switch (wave_form) {
        case WaveForm::SINE:
            InitBuffer(WaveStaticParams::s_SineArr.data(), gain, offset, interval);
            break;
        case WaveForm::TRIANGLE:
            InitBuffer(WaveStaticParams::s_TriangleArr.data(), gain, offset, interval);
            break;
        default: 
            InitBuffer(WaveStaticParams::s_SineArr.data(), gain, offset, interval);
            result = false;
            break;
        }
    
        return result;
    }

    void WaveArr::ResetStaticArr(const uint16_t max_value, const uint16_t min_value, const BufSize buf_max_size){
        WaveArr::s_Params.bufMaxSize = To_uint16(buf_max_size) + (buf_max_size == BufSize::BUF_END ? 0 : -1);
        WaveArr::s_Params.maxVal = max_value;
        WaveArr::s_Params.minVal = min_value;
        float phase = 0;
        const float phase_step = 2 * PI / (float)WaveArr::s_Params.bufMaxSize;
        uint16_t AMP = (WaveArr::s_Params.maxVal - WaveArr::s_Params.minVal) / 2;
        uint16_t AveVal = WaveArr::s_Params.minVal + AMP;
        float TriangleStep = 2 / (float)WaveArr::s_Params.bufMaxSize * AMP * 2;
        uint16_t minVal = WaveArr::s_Params.minVal;

    //     int i = 0;
    //     for (i = 0; i < (uint16_t)(WaveArr::s_Params.bufMaxSize / 2); i++) {
    //         WaveStaticParams::s_SineArr[i] = AMP * sin(phase) + AveVal;
    //         WaveStaticParams::s_SineArr[WaveArr::s_Params.bufMaxSize - i - 1] = AveVal - AMP * sin(phase);
    //         phase += phase_step;
    //         WaveStaticParams::s_TriangleArr[i] = minVal + (TriangleStep * i);
    //         WaveStaticParams::s_TriangleArr[WaveArr::s_Params.bufSize - i - 1] = minVal + (TriangleStep * (i + 1));
    //     }
    // }
    }
    

    void WaveArr::ShowStatic(uint8_t line, uint16_t delay_time){
        // Show
        for (int i = WaveArr::s_Params.bufMaxSize - 1; i > 0; i--)
        {
            OLED_ShowNum(line, 1, i, 3);
            OLED_ShowNum(line, 5, WaveStaticParams::s_SineArr[i], 4);
            OLED_ShowNum(line, 10, WaveStaticParams::s_TriangleArr[i], 4);
            Delay_ms(delay_time);
        }
    }

    void WaveArr::ShowDynamic(uint8_t line, uint16_t delay_time) const {
        for (int i = 0; i < WaveArr::dynParams.bufSize; i++)
        {
            OLED_ShowNum(line, 1, i, 3);
            OLED_ShowNum(line, 5, this->dynParams.buffer[i], 4);
            Delay_ms(delay_time);
        }
    }

    void WaveArr::ResetWaveForm(WaveForm wave_form){
        if (wave_form == this->waveForm) {
            return;
        }

        switch (wave_form)
        {
        case WaveForm::SINE:
        case WaveForm::TRIANGLE:
            this->waveForm = wave_form;
            break;
        default:
            // WaveArr仅支持查表波形（SINE/TRIANGLE），其它类型回退为SINE
            this->waveForm = WaveForm::SINE;
            break;
        }
        InitArr();
    }
    const uint16_t* WaveArr::GetBufferAddrPtr(uint16_t index){
        uint16_t * result = nullptr;
        if (this->waveForm == WaveForm::TRIANGLE) {
            result = const_cast<uint16_t *>(TriangleArr(index));
        } else if (this->waveForm == WaveForm::SINE){
            result = const_cast<uint16_t *>(SineArr(index));
        }
        return result;
    }

    const uint16_t* WaveArr::GetStaticBufAddrPtr(uint16_t index){
        uint16_t * result = nullptr;
        if (this->waveForm == WaveForm::TRIANGLE) {
            result = const_cast<uint16_t *>(TriangleArr(index));
        } else if (this->waveForm == WaveForm::SINE){
            result = const_cast<uint16_t *>(SineArr(index));
        }
        return result;
    }
    const uint16_t WaveArr::GetBufferVal(uint16_t index){
        uint16_t result = 0;
        const uint16_t* ptr = GetBufferAddrPtr(index);
        if (ptr && index < WaveArr::s_Params.bufMaxSize) {
            result = *ptr;
        }
        return result;
    }

    const uint16_t * WaveArr::TriangleArr(uint16_t index) { 
        uint16_t * result = nullptr;
        if (index < WaveArr::s_Params.bufMaxSize) {
            result = const_cast<uint16_t *>(&WaveStaticParams::s_TriangleArr[index]);
        } else { result = const_cast<uint16_t *>(&WaveStaticParams::s_TriangleArr[0]); }
        return result;
    }

    const uint16_t * WaveArr::SineArr(uint16_t index) {
        uint16_t * result = nullptr;
        if (index < WaveArr::s_Params.bufMaxSize) {
            result = const_cast<uint16_t *>(&WaveStaticParams::s_SineArr[index]);
        } else { result = const_cast<uint16_t *>(&WaveStaticParams::s_SineArr[0]); }
        return result;
    }

    DAC_ParamsTypedef::DAC_ParamsTypedef(DAC_Channel channel, TIM_TypeDef *TIM, DAC_UpdateMode update_mode) {
        AssignParams(channel, TIM);
        AssignUpdateMode(update_mode);
    }

    bool DAC_ParamsTypedef::AssignParams(DAC_Channel channel, TIM_TypeDef * tim){
        bool result = true;

        this->channel = channel;
        this->TIM = tim;
        
        this->pin = ((channel == DAC_Channel::CH1) ? DAC_GPIO_PIN::PIN4 : DAC_GPIO_PIN::PIN5);
        
        AssignDacTimIRQn();

        return result;
    }

    bool DAC_ParamsTypedef::AssignUpdateMode(DAC_UpdateMode update_mode){
        this->updateMode = update_mode;
        bool result = false;
        if (this->updateMode == DAC_UpdateMode::TIM_TRIG_DMA_SEND) {
            for (auto &tim_dac_map : timDacMap){
                if (tim_dac_map.TIMx == this->TIM) {
                    this->trigger = tim_dac_map.trigger;
                    result = true;
                    break;
                }
            }
        } else if (this->updateMode == DAC_UpdateMode::DIRECT_UPDATE || this->updateMode == DAC_UpdateMode::TIM_IT_UPDATE) {
            this->trigger = DAC_Trigger::SOFTWARE;
            result = true;
        } 
        return result;
    }
    // TimDacIRQn Up Params: TIM
    bool DAC_ParamsTypedef::AssignDacTimIRQn(void){
        bool result = true;
        this->timIRQn = TIM_IRQnManage::GetIRQn(this->TIM);
        if (this->timIRQn == IRQN_NONE) result = false; 
        return result;
    }


    // Class DAC_Chan_Controller
    DAC_ChanController::DAC_ChanController(WaveForm wave_form, DAC_Channel dac_channel, TIM_TypeDef * timx_dac, TIM_TypeDef * timx_dma, TIM_DMA_Source tim_dma_source, BufSize wave_buf_size, CV_VoltParams cv_volt_params, CV_Params cv_params, DAC_UpdateMode dac_update_mode) : waveArr(wave_form, wave_buf_size), cvController(cv_volt_params, cv_params, wave_form, dac_update_mode){
        this->waveForm = wave_form;

        AssignDacParams(dac_channel, timx_dac);
        AssignDmaParams(timx_dma, tim_dma_source);
        this->scanDuration = (this->waveForm == WaveForm::CV_FLUCTUATE ? this->cvController.GetCvParams().scanDuration : this->waveArr.GetDynParams().scanDuration);

        this->state = State::INITIALIZED;

    }
    DAC_ChanController::DAC_ChanController(WaveForm wave_form, DAC_ParamsTypedef dac_params, DMA_ParamsTypedef dma_params, BufSize wave_buf_size, CV_VoltParams volt_params, CV_Params cv_params, DAC_UpdateMode dac_update_mode) : DAC_ChanController(wave_form, dac_params.channel, dac_params.TIM, dma_params.TIM, dma_params.timDmaSource, wave_buf_size, volt_params, cv_params, dac_update_mode) {

    }


    DAC_ChanController::~DAC_ChanController()
    {
        FunctionalState state = DISABLE;
        
        DAC_Cmd(To_uint32(this->DAC_Params.channel), state);
        TIM_Cmd(this->DAC_Params.TIM, state);
        TIM::TIM_ITConfig(this->DAC_Params.TIM, TIM::IT::UP, state);
        DMA_Cmd(this->DMA_Params.channel, state);
        TIM_Cmd(this->DMA_Params.TIM, state);
        TIM_DMACmd(this->DMA_Params.TIM, To_uint16(this->DMA_Params.timDmaSource), state);
        
    }

    // Private Functions of DAC_Chan_Controller
    bool DAC_ChanController::AssignDacParams(DAC_Channel dac_channel, TIM_TypeDef * timx_dac){
        bool result = true;

        this->DAC_Params.channel = dac_channel;
        this->DAC_Params.TIM = timx_dac;

        // DAC Sub Setup
        this->DAC_Params.pin = (this->DAC_Params.channel == DAC_Channel::CH1 ? DAC_GPIO_PIN::PIN4 : DAC_GPIO_PIN::PIN5);

        // 根据波形/模式分配更新方式与触发源
        result &= AssignDacUpdateMode();

        // 根据TIM分配中断号
        result &= AssignDacTimIRQn();

        return result;
    }
    // DacUpdateMode Up Params: waveForm
    // Will Assign dacUpdateMode dacTrigger 
    bool DAC_ChanController::AssignDacUpdateMode(){
        bool result = true;
        switch (this->waveForm)
        {
        case WaveForm::SINE:
        case WaveForm::TRIANGLE:
            this->DAC_Params.updateMode = DAC_UpdateMode::TIM_TRIG_DMA_SEND;
            break;
        case WaveForm::CV_FLUCTUATE:
            this->DAC_Params.updateMode = this->cvController.GetUpdateMode();
            break;
        case WaveForm::CV_CONSTANT:
        case WaveForm::DC_CONSTANT:
            this->DAC_Params.updateMode = DAC_UpdateMode::DIRECT_UPDATE;
            break;
        default:
            result = false;
            break;
        }

        // 触发源依赖updateMode与TIM
        result &= this->AssignDacTrigger();

        return result;
    }
    // DacTrigger Up Params: TIM, updateMode
    bool DAC_ChanController::AssignDacTrigger(void) {
        bool result = false;
        if (this->DAC_Params.updateMode == DAC_UpdateMode::TIM_TRIG_DMA_SEND) {
            for (auto &tim_dac_map : timDacMap){
                if (tim_dac_map.TIMx == this->DAC_Params.TIM) {
                    this->DAC_Params.trigger = tim_dac_map.trigger;
                    result = true;
                    break;
                }
            }
        } else if (this->DAC_Params.updateMode == DAC_UpdateMode::DIRECT_UPDATE || this->DAC_Params.updateMode == DAC_UpdateMode::TIM_IT_UPDATE) {
            this->DAC_Params.trigger = DAC_Trigger::SOFTWARE;
            result = true;
        } 
        return result;
    }
    // TimDacIRQn Up Params: TIM
    bool DAC_ChanController::AssignDacTimIRQn(void){
        bool result = true;
        this->DAC_Params.timIRQn = TIM::GetIRQn(this->DAC_Params.TIM);
        if (this->DAC_Params.timIRQn == IRQN_NONE) {
            result = false;
        }
        return result;
    }

    bool DAC_ChanController::AssignDmaParams(TIM_TypeDef * timx_dma, TIM_DMA_Source tim_dma_source){
        bool result = true;

        this->DMA_Params.TIM = timx_dma;
        this->DMA_Params.timDmaSource = tim_dma_source;

        // 根据(TIM, DMA请求源)查表分配DMA通道
        result &= AssignDmaTrigger(tim_dma_source);

        return result;
    }
    // Up Params: TIM
    // Up Params: TIM, timDmaSource
    bool DAC_ChanController::AssignDmaTrigger(void) {
        return AssignDmaTrigger(this->DMA_Params.timDmaSource);
    }
    // DmaTrigger Up Params: TIM
    bool DAC_ChanController::AssignDmaTrigger(TIM_DMA_Source dma_tim_trigger) {
        bool result = false;
        this->DMA_Params.timDmaSource = dma_tim_trigger;
        for (auto& map : timDmaMap) {
            if (this->DMA_Params.TIM == map.TIMx && this->DMA_Params.timDmaSource == map.DMA_Request) {
                this->DMA_Params.channel = map.DMA_Channel;
                result = true;
                break;
            }
        }
        return result;
    }

    // Will Change dacUpdateMode, dacTrigger, scanDuration
    bool DAC_ChanController::AssignValAboutWaveForm(void) {
        bool result = true;
        switch (this->waveForm)
        {
        case WaveForm::SINE:
        case WaveForm::TRIANGLE: 
            this->DAC_Params.updateMode = DAC_UpdateMode::TIM_TRIG_DMA_SEND;
            this->scanDuration = this->waveArr.GetDynParams().scanDuration;
            break;
        case WaveForm::CV_FLUCTUATE:
            this->DAC_Params.updateMode = this->cvController.GetUpdateMode();
            this->scanDuration = this->cvController.GetCvParams().scanDuration;
            break;
        case WaveForm::CV_CONSTANT:
        case WaveForm::DC_CONSTANT:
            this->DAC_Params.updateMode = DAC_UpdateMode::DIRECT_UPDATE;
            break;
        default: result = false;
            break;
        }
        if (!AssignDacTrigger()) {
            OLED_ShowString(4, 1, "Error DAC Trigger");
        }

        return result;
    }

    void DAC_ChanController::SetDacChannelDate(void){
        if (this->DAC_Params.channel == DAC_Channel::CH1) {
            if (this->waveForm == WaveForm::CV_CONSTANT) {
                DAC_SetChannel1Data(DAC_Align_12b_R, this->cvController.GetCvParams().initVal);
            } else if (this->waveForm == WaveForm::DC_CONSTANT) {
                DAC_SetChannel1Data(DAC_Align_12b_R, waveArr.GetDynParams().constVal);
            }
        } else if (this->DAC_Params.channel == DAC_Channel::CH2) {
            if (this->waveForm == WaveForm::CV_CONSTANT) {
                DAC_SetChannel2Data(DAC_Align_12b_R, this->cvController.GetCvParams().initVal);
            } else if (this->waveForm == WaveForm::DC_CONSTANT) {
                DAC_SetChannel2Data(DAC_Align_12b_R, waveArr.GetDynParams().constVal);
            }
        } 
    }

    void DAC_ChanController::RCC_PeriphSetState(FunctionalState state){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, state);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, state);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, state);
    }
    void DAC_ChanController::AllPeriph_Cmd(FunctionalState state){
        if (DAC_Params.updateMode  == DAC_UpdateMode::TIM_IT_UPDATE) {
            TIM_ITConfig(this->DAC_Params.TIM, TIM_IT_Update, state);
        } else if (DAC_Params.updateMode == DAC_UpdateMode::TIM_TRIG_DMA_SEND) {
            TIM_DMACmd(this->DMA_Params.TIM, To_uint16(this->DMA_Params.timDmaSource), state);
        }
    }


    // Public Functions
    void DAC_ChanController::ResetWaveForm(WaveForm wave_form){
        this->waveForm = wave_form;
        this->AssignValAboutWaveForm();
    }
    void DAC_ChanController::ResetDacParams(DAC_Channel dac_channel){
        this->DAC_Params.channel = dac_channel;
        this->DAC_Params.pin = (this->DAC_Params.channel == DAC_Channel::CH1? DAC_GPIO_PIN::PIN4 : DAC_GPIO_PIN::PIN5);
    }
    void DAC_ChanController::ResetDacParams(TIM_TypeDef * timx_dac){
        this->DAC_Params.TIM = timx_dac;
        AssignDacTrigger();
        AssignDacTimIRQn();
    }
    void DAC_ChanController::ResetDacParams(DAC_Channel dac_channel, TIM_TypeDef * timx_dac){
        this->ResetDacParams(dac_channel);
        this->ResetDacParams(timx_dac);
    }

    void DAC_ChanController::ResetDmaTrigger(TIM_TypeDef * timx_dma, TIM_DMA_Source dma_tim_trigger){
        this->DMA_Params.TIM = timx_dma;
        this->AssignDmaTrigger(dma_tim_trigger);
    }
    void DAC_ChanController::ResetWaveArr(BufSize wave_buf_size){
        this->waveArr.ResetBufSize(wave_buf_size);
    }
    void DAC_ChanController::ResetCvUpdateMode(DAC_UpdateMode update_mode){
        if (this->cvController.GetUpdateMode() != update_mode) {
            this->cvController.ResetParam(update_mode);
            AssignDacUpdateMode();
        }
    }

    void DAC_ChanController::ResetCvController(CV_VoltParams volt_params, CV_Params cv_params, DAC_UpdateMode dac_update_mode){
        this->cvController.ResetParam(volt_params, cv_params.scanDuration, cv_params.voltScanRate, cv_params.scanDIR, this->waveForm, dac_update_mode);
        AssignDacUpdateMode();  
    }
    
    void DAC_ChanController::ReStore(void) { 
        RCC_PeriphSetState(ENABLE);
        AllPeriph_Cmd(ENABLE);
    }

    // Important Functions

    void DAC_ChanController::GPIO_Setup(void){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = To_uint16(this->DAC_Params.pin);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    void DAC_ChanController::DAC_Setup(void) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

        DAC_InitTypeDef DAC_InitStructure;
        // DAC_StructInit(&DAC_InitStructure);
        DAC_InitStructure.DAC_Trigger = To_uint32(this->DAC_Params.trigger);
        DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
        DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
        DAC_Init(To_uint32(this->DAC_Params.channel), &DAC_InitStructure);

        DAC_Cmd(To_uint32(this->DAC_Params.channel), ENABLE);
        
        if (this->DAC_Params.updateMode == DAC_UpdateMode::DIRECT_UPDATE) {
            this->SetDacChannelDate();
            DAC_SoftwareTriggerCmd(To_uint32(this->DAC_Params.channel), ENABLE);
        }

        // OLED_ShowString(4, 1, "DAC_Setup");
        
    }
    void DAC_ChanController::TIM_DAC_Setup(void) {
        // OLED_ShowString(1, 1, "TIM_DAC_Setup");
        // Delay_ms(500);
        
        this->DAC_Setup();     
        if (this->DAC_Params.updateMode == DAC_UpdateMode::TIM_IT_UPDATE || this->DAC_Params.updateMode == DAC_UpdateMode::TIM_TRIG_DMA_SEND) {
            // TIM_Clock ENABLE
            TIM::InitTIM(this->DAC_Params.TIM, this->scanDuration);
            TIM_SelectOutputTrigger(this->DAC_Params.TIM, TIM_TRGOSource_Update);
            TIM_Cmd(this->DAC_Params.TIM, ENABLE);
            TIM_ITConfig(this->DAC_Params.TIM, TIM_IT_Update, ENABLE);
        }
    }
    void DAC_ChanController::TIM_DAC_IRQHandler(void) {
        this->cvController.UpdateCurrentVal();
        if (this->DAC_Params.updateMode == DAC_UpdateMode::TIM_IT_UPDATE) {
            if (this->DAC_Params.channel == DAC_Channel::CH1) {
                DAC_SetChannel1Data(DAC_Align_12b_R, this->cvController.GetBufferVal());
            } else if (this->DAC_Params.channel == DAC_Channel::CH2) {
                DAC_SetChannel2Data(DAC_Align_12b_R, this->cvController.GetBufferVal());
            }
            DAC_SoftwareTriggerCmd(To_uint32(this->DAC_Params.channel), ENABLE);
        }
    }

    void DAC_ChanController::DMA_Setup(void) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        DMA_InitTypeDef DMA_InitStructure;
        uint32_t arrToSendAddr;
        uint16_t bufSize;

        if (this->DAC_Params.updateMode == DAC_UpdateMode::TIM_TRIG_DMA_SEND) {
            if (this->waveForm == WaveForm::CV_FLUCTUATE) {
                arrToSendAddr = reinterpret_cast<uintptr_t>(this->cvController.GetBufferAPtr());
                bufSize = 1;
            } else {
                arrToSendAddr = (uint32_t)this->waveArr.GetBufferAddrPtr();
                bufSize = this->waveArr.GetDynParams().bufSize;
            }
            if (this->DAC_Params.channel == DAC_Channel::CH1) {
                DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (DAC->DHR12R1);
            } else if (this->DAC_Params.channel == DAC_Channel::CH2) {
                DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (DAC->DHR12R2);
            }
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_MemoryBaseAddr = arrToSendAddr;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_Priority = DMA_Priority_High;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
            DMA_InitStructure.DMA_BufferSize = bufSize;
            DMA_Init(this->DMA_Params.channel, &DMA_InitStructure);
            DMA_Cmd(this->DMA_Params.channel, ENABLE);
        } 
    }
    
    void DAC_ChanController::TIM_DMA_Setup(void) {
        this->DMA_Setup();
        // OLED_ShowString(1, 1, "TIM_DMA_Setup");
        // Delay_ms(500);
        
        if (this->DMA_Params.TIM != this->DAC_Params.TIM) {
            TIM::InitTIM(this->DMA_Params.TIM, this->scanDuration);
            TIM_Cmd(this->DMA_Params.TIM, ENABLE);
        }
        if (this->DAC_Params.updateMode == DAC_UpdateMode::TIM_TRIG_DMA_SEND ) {
            TIM_DMACmd(this->DMA_Params.TIM, To_uint16(this->DMA_Params.timDmaSource), ENABLE);
        }

    }

    void DAC_ChanController::SetState(FunctionalState state) {
        DAC_Cmd(To_uint32(this->DAC_Params.channel), state);
        TIM_Cmd(this->DAC_Params.TIM, state);
        TIM_ITConfig(this->DAC_Params.TIM, TIM_IT_Update, state);
        TIM_Cmd(this->DMA_Params.TIM, state);
        TIM_DMACmd(this->DAC_Params.TIM, To_uint16(this->DMA_Params.timDmaSource), state);
        TIM_DMACmd(this->DMA_Params.TIM, To_uint16(this->DMA_Params.timDmaSource), state);
        DMA_Cmd(this->DMA_Params.channel, state);
        DMA::DMA_ITConfig(this->DMA_Params.channel, DMA::IT::TC, state);
    }

    void DAC_ChanController::Start(){
        if (state == State::INITIALIZED) {
            // 启动操作
            state = State::RUNNING;
            this->GPIO_Setup();
            this->TIM_DAC_Setup();
            this->TIM_DMA_Setup();
        } else if (state == State::STOPPED) {
            // 
            state = State::RUNNING;
            this->SetState(ENABLE);
        }
    }
    
    void NS_DAC::DAC_ChanController::Pause() {
        // 避免重复暂停
        if (pauseState.isPaused || state != State::RUNNING) return;

        // 1. 保存暂停前的硬件使能状态到结构体
        pauseState.preDACState = STM32PeriphFunc::DAC_GetCmdStatus(To_uint32(this->DAC_Params.channel));
        pauseState.preTIMDACState = STM32PeriphFunc::TIM_GetCmdStatus(this->DAC_Params.TIM);
        pauseState.preTIMDMAState = STM32PeriphFunc::TIM_GetCmdStatus(this->DMA_Params.TIM);
        pauseState.preDMAState = STM32PeriphFunc::DMA_GetCmdStatus(this->DMA_Params.channel);
        pauseState.preTIMITState = TIM_GetITStatus(this->DAC_Params.TIM, TIM_IT_Update) == RESET? ENABLE: DISABLE;
        pauseState.preTIMDMACmdState = STM32PeriphFunc::TIM_GetDMACmdStatus(this->DMA_Params.TIM, To_uint16(this->DMA_Params.timDmaSource)) ? ENABLE : DISABLE;

        // 2. 禁用硬件
        DAC_Cmd(To_uint32(this->DAC_Params.channel), DISABLE);
        TIM_Cmd(this->DAC_Params.TIM, DISABLE);
        TIM_Cmd(this->DMA_Params.TIM, DISABLE);
        TIM_ITConfig(this->DAC_Params.TIM, TIM_IT_Update, DISABLE);
        DMA_Cmd(this->DMA_Params.channel, DISABLE);
        TIM_DMACmd(this->DMA_Params.TIM, To_uint16(this->DMA_Params.timDmaSource), DISABLE);
        NVIC_DisableIRQ(DAC_Params.timIRQn);  // 禁用定时器中断的 NVIC 通道

        // 3. 更新状态标记
        pauseState.isPaused = true;
        state = State::PAUSED;
    }

    void NS_DAC::DAC_ChanController::Resume() {
        // 仅在暂停状态下执行恢复
        if (!pauseState.isPaused || state != State::PAUSED) return;

        // 1. 根据结构体保存的状态恢复硬件
        if (pauseState.preDMAState == ENABLE) {
            DMA_Cmd(this->DMA_Params.channel, ENABLE);
        }
        if (pauseState.preTIMDMACmdState == ENABLE) {
            TIM_DMACmd(this->DMA_Params.TIM, To_uint16(this->DMA_Params.timDmaSource), ENABLE);
        }
        if (pauseState.preTIMDACState == ENABLE) {
            TIM_SetCounter(this->DAC_Params.TIM, 0); // 重置计数器
            TIM_Cmd(this->DAC_Params.TIM, ENABLE);
        }
        if (pauseState.preTIMDMAState == ENABLE) {
            TIM_Cmd(this->DMA_Params.TIM, ENABLE);
        }
        if (pauseState.preTIMITState == ENABLE) {
            // 关键：清除残留的中断标志位（必须在启用中断前）
            TIM_ClearITPendingBit(this->DAC_Params.TIM, TIM_IT_Update);
            TIM_ITConfig(this->DAC_Params.TIM, TIM_IT_Update, ENABLE);

            NVIC_EnableIRQ(DAC_Params.timIRQn);  
        }
        if (pauseState.preDACState == ENABLE) {
            DAC_Cmd(To_uint32(this->DAC_Params.channel), ENABLE);
            // 值初始化
            this->Get_CV_Controller().ReStoreDefault();
            // 方案2：若所有波形模式均需恢复输出，直接移除判断（简化逻辑）
            this->SetDacChannelDate(); // 加载最新数据
            DAC_SoftwareTriggerCmd(To_uint32(this->DAC_Params.channel), ENABLE); // 触发输出
        }
        
        // 2. 清除暂停标记
        pauseState.isPaused = false;
        state = State::RUNNING;
    }

    // CvDrive::CvDrive(DAC_Channel cv_channel, TIM_TypeDef * timx_dac, TIM_TypeDef * timx_dma, TIM_DMA_Source tim_dma_source, BufSize buf_size, CV_VoltParams volt_params, CV_Params cv_params, DAC_UpdateMode dac_update_mode) : cvChanController(WaveForm::CV_FLUCTUATE, cv_channel, timx_dac, timx_dma, tim_dma_source, buf_size, volt_params, cv_params, dac_update_mode) {
    //     this->cvChannel = cv_channel;
    //     this->constChannel = (this->cvChannel == DAC_Channel::CH1? DAC_Channel::CH2: DAC_Channel::CH1);
    //     constChanController = DAC_ChanController(WaveForm::CV_CONSTANT, this->constChannel);
    // }

    // Class CustomWave
    
    // DAC_WaveDrive::DAC_WaveDrive(WaveForm wave_form, DAC_Channel dac_channel, DMA_Channel_TypeDef *dma_channel, TIM_TypeDef * timx) 
    //     : waveForm(wave_form), dmaChannel(dma_channel), TIMx(timx)
    // {
    //     // Arg_Init();
    //     if (this->waveForm == WaveForm::SINE) {
    //         DAC_Controller::SetCV_Channel(this->cv_channel);
    //     }  
    // }
    // DAC_WaveDrive::~DAC_WaveDrive() { };
    // int DAC_WaveDrive::Get_DAC_Val(void) {
    //     return CV_Controller::GetBufferVal();
    // }
    // void DAC_WaveDrive::FillDualChannel12bit(void){
    //     uint32_t Idx = 0;    
    //     /* Fill Sine32bit table */
    //     if (this->waveForm == WaveForm::SINE || this->waveForm == WaveForm::TRIANGLE) {
    //         for (Idx = 0; Idx < WaveArr::GetDynParams().bufSize; Idx++) {
    //             dualChannel12BitArr[Idx] = (WaveArr::GetBufferVal(Idx) << 16) + (WaveArr::GetBufferVal(Idx));
    //         }
    //     } 
    // }

    // 定义并初始化静态成员
    NS_DAC::DAC_ChanController NS_DAC::DAC_Controller::dacChanCV(
        NS_DAC::WaveForm::CV_FLUCTUATE, 
        DAC_Channel::CH2, 
        TIM2, TIM2, 
        TIM_DMA_Source::UP, 
        BufSize::BUF_32, 
        CV_VoltParams(0.4f, -0.8f, 1.5f), 
        CV_Params(0.05f, 0.05f, NS_DAC::ScanDIR::FORWARD), 
        DAC_UpdateMode::TIM_IT_UPDATE
    );

    NS_DAC::DAC_ChanController NS_DAC::DAC_Controller::dacChanConstant(
        NS_DAC::WaveForm::CV_CONSTANT,
        DAC_Channel::CH1,
        TIM3, TIM3,
        TIM_DMA_Source::UP,
        BufSize::BUF_32,
        CV_VoltParams(0.4f, -0.8f, 1.5f), 
        CV_Params(0.05f, 0.05f, NS_DAC::ScanDIR::FORWARD),
        DAC_UpdateMode::TIM_IT_UPDATE
    );

    // 获取DAC即将发送的CV值（12Bit）
    const uint16_t GetCvValToSend(void) {
        return NS_DAC::DAC_Controller::dacChanCV.Get_CV_Controller().GetValToSend();
    }

    // 获取DAC即将发送的CV值指针（12Bit）
    const uint16_t * GetCvValToSendBuf(void) {
        return NS_DAC::DAC_Controller::dacChanCV.Get_CV_Controller().GetValToSendPtr();
    }

    // 获取DAC即将发送的CV值引用（12Bit）
    const uint16_t & GetCvValToSendRef(void) {
        return NS_DAC::DAC_Controller::dacChanCV.Get_CV_Controller().GetValToSendRef();
    }

    NS_ADC::InitParams& CGMParams2AdcInitParams(CGM::Params &params, NS_ADC::InitParams &init_params) {
        init_params.adc = ADC1;
        init_params.mode = NS_ADC::Mode::INDEPENDENT;
        init_params.nbr_of_channels = params.nbr_of_channels;
        init_params.channels = &(params.adChanConfig[0]);
        init_params.scan_mode = ENABLE;
        init_params.cont_mode = ENABLE;
        init_params.clock_prescaler = RCC_PCLK2_Div6;
        return init_params;
    }

    CGM::Params adcParams = CGM::CGM_250901::params;
    NS_ADC::InitParams adcInitParams = CGMParams2AdcInitParams(adcParams, adcInitParams);

    NS_ADC::ADC adc(adcInitParams, NS_ADC::ShowParams(TIM3, 0.05), NS_DAC::DAC_Controller::dacChanCV.Get_CV_Controller());

    // 获取ADC DMA缓冲区头指针
    const uint16_t * GetADCDmaBufHeader(void) {
        return adc.GetDmaBufferHeader();
    }
    // 获取ADC DMA缓冲区引用
    const std::array<uint16_t, 16> & GetADCDmaBufRef(void) {
        return adc.GetDmaBufferRef();
    }

    // 获取ADC DMA缓冲区对应计算所得电流大小缓冲区头指针
    const double * GetADCCurrentBufHeader(void) {
        return adc.GetCurrentBufHeader();
    }
    // 获取ADC DMA缓冲区对应计算所得电流大小缓冲区引用
    const std::array<double, 16> & GetADCCurrentBufRef(void) {
        return adc.GetCurrentBufRef();
    }

    // 直接返回adc引用
    const NS_ADC::ADC & GetADCRef() {
        return adc;
    }

    NS_ADC::ADC & GetADC(void) {
    return adc;
    }


    static void TIM_IRQn_DAC() {
        NS_DAC::DAC_Controller::dacChanCV.TIM_DAC_IRQHandler();
    }

    static void TIM_IRQn_AD() {
        adc.TIM_IRQnHandler();
    }

    static void DMA_IRQn_AD() {
        adc.DMA_IRQnHandler();
    }

    // 定义单例实例的获取方法
    SystemController& SystemController::GetInstance() {
        static SystemController instance;  // 局部静态变量，确保唯一实例且线程安全（C++11及以上）
        return instance;
    }

    void SystemController::Start() { 
        bool isAdd = TIM_IRQnManage::Add(DAC_Controller::dacChanCV.Get_DAC_Params().TIM, TIM::IT::UP, &TIM_IRQn_DAC, 1, 1);
        if (!isAdd) { JustShow("Add 1 Failed"); }
        isAdd = TIM_IRQnManage::Add(adc.GetShowParams().TIMx, adc.GetShowParams().tim_it,  &TIM_IRQn_AD, 1, 2);
        if (!isAdd) { JustShow("Add 2 Failed"); }
        // isAdd = DMA_IRQnManage::Add(adc.GetDmaChannel(), &DMA_IRQn_AD, DMA::DMA_IT::TC);
        // if (!isAdd) { JustShow("Add 3 Failed"); }
        DAC_Controller::dacChanCV.Start();
        DAC_Controller::dacChanConstant.Start();
        adc.StartConversion();
        this->systemState = SystemState::RUNNING;
    }

    // 系统级暂停实现
    void SystemController::Pause() {
        // 暂停ADC（检查状态避免重复操作）
        if (!adc.IsPaused()) {
            adc.Pause();
        }

        // 暂停CV通道DAC（检查状态避免重复操作）
        if (!NS_DAC::DAC_Controller::dacChanCV.IsPaused()) {
            NS_DAC::DAC_Controller::dacChanCV.Pause();
        }

        // 暂停恒定电压通道DAC（检查状态避免重复操作）
        if (!NS_DAC::DAC_Controller::dacChanConstant.IsPaused()) {
            NS_DAC::DAC_Controller::dacChanConstant.Pause();
        }

        this->ClearUpdataTimes();
        this->systemState = SystemState::PAUSED;

        OLED_ShowString(4, 1, "Syetem Paused");

        // 可选：添加其他模块的暂停逻辑
        // 例如：按键扫描暂停、OLED刷新暂停等
        // KeyScanner::GetInstance().Pause();
        // OLED::GetInstance().PauseRefresh();
    }

    // 系统级恢复实现
    void SystemController::Resume() {
        // 恢复ADC（检查状态避免重复操作）
        if (adc.IsPaused()) {
            adc.Resume();
        }

        // 恢复CV通道DAC（检查状态避免重复操作）
        if (NS_DAC::DAC_Controller::dacChanCV.IsPaused()) {
            NS_DAC::DAC_Controller::dacChanCV.Resume();
        }

        // 恢复恒定电压通道DAC（检查状态避免重复操作）
        if (NS_DAC::DAC_Controller::dacChanConstant.IsPaused()) {
            NS_DAC::DAC_Controller::dacChanConstant.Resume();
        }

        OLED_ShowString(4, 1, "System Resumed");
        this->systemState = SystemState::RUNNING;

        // 可选：添加其他模块的恢复逻辑
        // 例如：恢复按键扫描、OLED刷新等
        // KeyScanner::GetInstance().Resume();
        // OLED::GetInstance().ResumeRefresh();
    }

}
