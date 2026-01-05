#include "ADCManager.h"
#include "OLED.h"
#include "GPIO.h"
#include "Delay.h"
#include "math.h"
// #include <IRQnManage.h>

namespace NS_ADC_DISPOSIT
{
    uint16_t AD_Buf[4];

    float max_current = 999.0f, min_current = -999.0f;

    enum class AD_Chan:uint8_t{
        CH0 = ADC_Channel_0, CH1 = ADC_Channel_1, CH2 = ADC_Channel_2, CH3 = ADC_Channel_3, CH4 = ADC_Channel_4, CH5 = ADC_Channel_5, CH6 = ADC_Channel_6, CH7 = ADC_Channel_7, CH8 = ADC_Channel_8, CH9 = ADC_Channel_9, CH10 = ADC_Channel_10, CH11 = ADC_Channel_11, CH12 = ADC_Channel_12, CH13 = ADC_Channel_13, CH14 = ADC_Channel_14, CH15 = ADC_Channel_15
    };

    enum class GPIO:uint16_t { PIN0 = GPIO_Pin_0, PIN1 = GPIO_Pin_1, PIN2 = GPIO_Pin_2, PIN3 = GPIO_Pin_3, PIN4 = GPIO_Pin_4, PIN5 = GPIO_Pin_5, PIN6 = GPIO_Pin_6, PIN7 = GPIO_Pin_7, PIN8 = GPIO_Pin_8,PIN9 = GPIO_Pin_9, PIN10 = GPIO_Pin_10, PIN11 = GPIO_Pin_11, PIN12 = GPIO_Pin_12, PIN13 = GPIO_Pin_13, PIN14 = GPIO_Pin_14, PIN15 = GPIO_Pin_15
    };

    struct  AD_ChanTypeDef {
        AD_Chan Channelx;
        GPIO_TypeDef * GPIOx;
        GPIO GPIO_Pin;
    };

    AD_ChanTypeDef AD_ChanMap[4] = {
        {AD_Chan::CH0, GPIOA, GPIO::PIN0},
        {AD_Chan::CH2, GPIOA, GPIO::PIN2},
        {AD_Chan::CH3, GPIOA, GPIO::PIN3},
        {AD_Chan::CH3, GPIOA, GPIO::PIN4},
    };



    void Fill_AD_ChannelStruct(AD_ChanTypeDef * ADC_ChannelStruct)
    {
        uint8_t channel = To_uint8(ADC_ChannelStruct->Channelx);
        uint16_t gpioPin = GPIO_Pin_0;
        if (channel <= ADC_Channel_7) {
            ADC_ChannelStruct->GPIOx = GPIOA;
            gpioPin = GPIO_Pin_0 << channel;
        } else if (channel <= ADC_Channel_9) {
            ADC_ChannelStruct->GPIOx = GPIOB;
            gpioPin = GPIO_Pin_0 << (channel - 8);
        } else if (channel <= ADC_Channel_15) {
            ADC_ChannelStruct->GPIOx = GPIOC;
            gpioPin = (GPIO_Pin_0 << (channel - 10));
        }
        ADC_ChannelStruct->GPIO_Pin = static_cast<GPIO>(gpioPin);
        
    }
    void AD_Init(void)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        
        RCC_ADCCLKConfig(RCC_PCLK2_Div6);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        uint16_t Pin = GPIO_Pin_1 | GPIO_Pin_2;
        GPIOA_Init(Pin, GPIO_Mode_AIN, GPIO_Speed_2MHz);
        
        // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, 
        //     ADC_SampleTime_55Cycles5);
        ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, 
            ADC_SampleTime_55Cycles5);
        ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, 
            ADC_SampleTime_55Cycles5);
        // ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, 
        //     ADC_SampleTime_55Cycles5);

        
        
        ADC_InitTypeDef ADC_InitStructure;
        ADC_StructInit(&ADC_InitStructure);
        ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_NbrOfChannel = 2;
        ADC_InitStructure.ADC_ScanConvMode = ENABLE;
        ADC_Init(ADC1, &ADC_InitStructure);
        
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        
        DMA_InitTypeDef DMA_InitStructure;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Buf;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = 2;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
        DMA_Init(DMA1_Channel1, &DMA_InitStructure);
        
        DMA_Cmd(DMA1_Channel1,ENABLE);
        ADC_DMACmd(ADC1, ENABLE);
        ADC_Cmd(ADC1, ENABLE);
            
        ADC_ResetCalibration(ADC1);
        while (ADC_GetResetCalibrationStatus(ADC1) == SET);
        ADC_StartCalibration(ADC1);
        while (ADC_GetCalibrationStatus(ADC1) == SET)
            Delay_ms(1);
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    }

} // namespace NS_ADC_DISPOSIT

namespace NS_ADC
{
    const float ADC::stepPerVolt = 1240.9091f;
    void ADC::ShowBoardVal(uint8_t i) {
        if (this->maxVal[i] < snapBuf[i]) { this->maxVal[i] = snapBuf[i]; }
        if (this->minVal[i] > snapBuf[i]) { this->minVal[i] = snapBuf[i]; }
        auto difVal = MyCompare<uint16_t>(maxVal[i] - minVal[i], 4095, 0);
        this->ShowVoltage(3 + i, 6, difVal, 0, this->params.channels[i].gain);
    }
    void ADC::ShowBoardVal() {
        for (uint16_t i = 0; i < this->params.nbr_of_channels; i++) {
            ShowBoardVal(i);
        }
    }

    // Return the Value of the Current(mA)
    double ADC::ShowVoltage(uint8_t line, uint8_t col, uint16_t ad_value, uint16_t ref_value, uint32_t gain) {
        double difVoltage = (double)(ad_value - ref_value) / ADC::stepPerVolt;
        double current = difVoltage * 1000.0 / gain;   // mA
        uint8_t current_left;
        uint16_t current_right;
        const uint16_t str_size = 12;
        char str_result[str_size] = "0.000mA   ";
        double temp_current = current;
        short signOfCurrent = 1;
        if (ad_value < ref_value) {
            signOfCurrent = -1;
            current = -current;
        }
        if (current >  1) {
            temp_current = current * signOfCurrent;
            snprintf(str_result, str_size, "%3.3fmA ", temp_current);
        } else if (current > 1e-3f) {
            temp_current = current * 1e3 * signOfCurrent;
            snprintf(str_result, str_size, "%3.3fuA ", temp_current);
        } else if (current > 1e-6f) {
            temp_current = current * 1e6 * signOfCurrent;
            snprintf(str_result, str_size, "%3.3fnA ", temp_current);
        }
        OLED_ShowString(line, col, str_result);

        return current;
    }

    void ADC::Show(){
        OLED_ShowString(1, 1, "CH1:");
        OLED_ShowString(2, 1, "CH2:");
        OLED_ShowString(3, 1, "Dif1:");
        OLED_ShowString(4, 1, "Dif2:");
        // TIM_IRQnHandler();
    }

    void ADC::TIM_IRQnHandler(void){
        // 仅做“触发 + 快照”，避免在 ISR 内做 OLED I/O 与 snprintf/double 运算
        for (uint16_t i = 0; i < this->params.nbr_of_channels; i++) {
            snapBuf[i] = dmaBuf[i];
        }
        needOledRefresh = true;
    }

    void ADC::DMA_IRQnHandler(void){
        ShowBoardVal();
    }

    void ADC::Service(){
        if (!needOledRefresh) return;
        needOledRefresh = false;

        //2. 限制 OLED 刷新频率 (例如每 100ms 刷新一次)
        if (SysTickTimer::GetTick() - lastOledRefreshTime > 100) {
            lastOledRefreshTime = SysTickTimer::GetTick();
            
            // 执行耗时的 OLED 操作
             for (uint16_t i = 0; i < this->params.nbr_of_channels; i++) {
            OLED_ShowNum(i + 1, 1, snapBuf[i], 4);
            this->currentBuf[i] = this->ShowVoltage(i + 1, 6, snapBuf[i], this->refValBuf[0], this->params.channels[i].gain);
            ShowBoardVal(i);
            }
        }
    }


    ADC::ADC(const InitParams& params, ShowParams show_params, const uint16_t ref_val,const uint16_t * ref_val_buf, CGM::VsMode vs_mode) : params(params), showParams(show_params), staticRefVal(ref_val), dynRefValBuf(ref_val_buf), vsMode(vs_mode) {
        this->adc = params.adc;
        this->dmaChannel = (this->adc == ADC1) ? DMA1_Channel1 : DMA1_Channel3;
        if (this->vsMode == CGM::VsMode::DYNAMIC) {
            this->refValBuf = this->dynRefValBuf;
        } else if (this->vsMode == CGM::VsMode::STATIC) {
            this->refValBuf = &(this->staticRefVal);
        }
        this->maxVal.fill(0);
        this->minVal.fill(4095);
        this->showTim = show_params.TIMx;

        this->Init();
    }

    void ADC::Init() {
        // 1. 使能时钟
        if(adc == ADC1) RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
        if(adc == ADC2) RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
        
        RCC_ADCCLKConfig(params.clock_prescaler);
        // 2. 配置GPIO
        GpioConfig();
        
        // 3. ADC参数配置
        ADC_InitTypeDef adc_init;
        ADC_StructInit(&adc_init);
        adc_init.ADC_Mode = To_uint32(params.mode);
        adc_init.ADC_ScanConvMode = params.scan_mode;
        adc_init.ADC_ContinuousConvMode = params.cont_mode;
        adc_init.ADC_DataAlign = ADC_DataAlign_Right;
        adc_init.ADC_NbrOfChannel = params.nbr_of_channels;
        adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
        ADC_Init(adc, &adc_init);
        
        // 4. 配置通道
        for(uint8_t i=0; i<params.nbr_of_channels; ++i) {
            ADC_RegularChannelConfig(adc, 
                params.channels[i].channel,
                i+1,  // 转换顺序
                params.channels[i].sampleTime);
        }
        
        // 5. 配置DMA
        if(params.scan_mode == ENABLE) {
            DmaConfig();
        }

        // 6. 使能ADC
        ADC_Cmd(adc, ENABLE);

        // 7. 校准
        Calibrate();
    }

    static void AdcChannelToGpio(uint8_t ch, GPIO_TypeDef*& port, uint16_t& pin) {

        if (ch <= 7) {                 // PA0..PA7
            port = GPIOA;
            pin  = static_cast<uint16_t>(GPIO_Pin_0 << ch);
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        } else if (ch <= 9) {          // PB0..PB1
            port = GPIOB;
            pin  = static_cast<uint16_t>(GPIO_Pin_0 << (ch - 8));
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        } else if (ch <= 15) {         // PC0..PC5
            port = GPIOC;
            pin  = static_cast<uint16_t>(GPIO_Pin_0 << (ch - 10));
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        } else {
            port = nullptr;
            pin  = 0;
        }
    }

    void ADC::GpioConfig()
    {
        GPIO_InitTypeDef gpio;
        gpio.GPIO_Mode  = GPIO_Mode_AIN;
        gpio.GPIO_Speed = GPIO_Speed_50MHz; // AIN模式速度字段通常无关，但补上不吃亏

        for (uint8_t i = 0; i < params.nbr_of_channels; ++i) {
            const uint8_t ch = params.channels[i].channel;

            GPIO_TypeDef* port;
            uint16_t pin;
            AdcChannelToGpio(ch, port, pin);
            if (!port || pin == 0) {
                // 可选：记录错误/断言
                continue;
            }

            gpio.GPIO_Pin = pin;
            GPIO_Init(port, &gpio);
        }
    }


    void ADC::DmaConfig() {
        DMA_InitTypeDef dma;
        DMA_Channel_TypeDef * dma_channel = (adc == ADC1) ? DMA1_Channel1 : DMA1_Channel3;
        
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        
        DMA_DeInit(dma_channel);
        dma.DMA_PeripheralBaseAddr = (uint32_t)&(adc->DR);
        dma.DMA_MemoryBaseAddr = (uint32_t)dmaBuf.data();
        dma.DMA_DIR = DMA_DIR_PeripheralSRC;
        dma.DMA_BufferSize = params.nbr_of_channels;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_High;
        dma.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(dma_channel, &dma);
        DMA_Cmd(dma_channel, ENABLE);
        
        ADC_DMACmd(adc, ENABLE);
        // auto irqn = DMA_IRQnManage::GetIRQn(dma_channel);
        // if (irqn != IRQN_NONE) {
        //     DMA_ITConfig(dma_channel, DMA_IT_TC, ENABLE);
        //     NVIC_InitTypeDef NVIC_InitStructure;
        //     NVIC_InitStructure.NVIC_IRQChannel = DMA_IRQnManage::GetIRQn(dma_channel);
        //     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        //     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
        //     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        //     NVIC_Init(&NVIC_InitStructure);
        // }
        
    }

    void ADC::ShowConfig(){
        if (showParams.TIMx == nullptr) return;
        auto tim = showParams.TIMx;

        TIM::InitTIM(tim, showParams.period);

        TIM_Cmd(tim, ENABLE);
        TIM::TIM_ITConfig(tim, showParams.tim_it, ENABLE);
        
        // Show Once for Some necessary Show
        Show();
    }

    void ADC::Calibrate() {
        ADC_ResetCalibration(adc);
        while(ADC_GetResetCalibrationStatus(adc) == SET);
        ADC_StartCalibration(adc);
        while(ADC_GetCalibrationStatus(adc) == SET) {
            Delay_ms(1);
        }
    }

    void ADC::StartConversion() {
        ADC_SoftwareStartConvCmd(adc, ENABLE);

        // 开始转换后进行显示
        ShowConfig();
    }

    void ADC::Pause() {
        if (isPaused) return;  // 避免重复暂停

        // 1. 停止ADC连续转换（不禁用ADC外设，保留校准状态）
        ADC_SoftwareStartConvCmd(adc, DISABLE);  // 停止软件触发转换
        // （关键）不调用 ADC_Cmd(adc, DISABLE)，避免ADC外设重启后需重新校准

        // 2. 暂停DMA传输（不重置DMA配置，保留当前缓存和传输计数）
        if (params.scan_mode == ENABLE) {
            DMA_Cmd(this->dmaChannel, DISABLE);  // 禁用DMA通道（暂停传输）
            // （关键）不调用 DMA_DeInit，保留DMA的缓冲区地址、循环模式等配置
        }

        // 3. 暂停定时器及其中断（避免触发无效显示）
        if (this->showTim != nullptr) {
            TIM_Cmd(this->showTim, DISABLE);                          // 停止定时器计数
            TIM::TIM_ITConfig(this->showTim, showParams.tim_it, DISABLE);  // 禁用定时器中断
            NVIC_DisableIRQ(TIM::GetIRQn(this->showTim));              // 禁用NVIC中断通道
        }

        // 4. 标记暂停状态
        isPaused = true;
    }

    void ADC::Resume() {
        if (!isPaused) return;  // 避免重复恢复

        // 1. 恢复DMA传输（直接启用已配置的DMA通道）
        if (params.scan_mode == ENABLE) {
            DMA_Cmd(this->dmaChannel, ENABLE);  // 启用DMA通道（恢复传输）
            ADC_DMACmd(adc, ENABLE);            // 确保ADC->DMA映射已启用（预防意外关闭）
        }

        // 2. 恢复定时器及其中断（延续之前的定时周期）
        if (this->showTim != nullptr) {
            NVIC_EnableIRQ(TIM::GetIRQn(this->showTim));              // 启用NVIC中断通道
            TIM::TIM_ITConfig(this->showTim, showParams.tim_it, ENABLE);  // 启用定时器中断
            TIM_Cmd(this->showTim, ENABLE);                          // 启动定时器计数
        }

        // 3. 恢复ADC转换（直接触发软件转换，无需重新校准）
        ADC_SoftwareStartConvCmd(adc, ENABLE);  // 恢复软件触发转换
        // （关键）ADC外设未被禁用，校准状态保留，无需重新执行 Calibrate()

        // 4. 清除暂停状态标记
        isPaused = false;

        
    }

    uint16_t ADC::GetValue(uint8_t channel) {
        ADC_RegularChannelConfig(adc, params.channels[channel].channel, 1, 
            params.channels[channel].sampleTime);
        ADC_SoftwareStartConvCmd(adc, ENABLE);
        while(!ADC_GetFlagStatus(adc, ADC_FLAG_EOC));
        return ADC_GetConversionValue(adc);
    }


    // NS_ADC::InitParams& CGMParams2AdcInitParams(CGM::Params &params, NS_ADC::InitParams &init_params) {
    //     init_params.adc = ADC1;
    //     init_params.mode = NS_ADC::Mode::INDEPENDENT;
    //     init_params.nbr_of_channels = params.nbr_of_channels;
    //     init_params.channels = &(params.adChanConfig[0]);
    //     init_params.scan_mode = ENABLE;
    //     init_params.cont_mode = ENABLE;
    //     init_params.clock_prescaler = RCC_PCLK2_Div6;
    //     return init_params;
    // }

    // CGM::Params adcParams = CGM::CGM_250305::params;
    // NS_ADC::InitParams adcInitParams = CGMParams2AdcInitParams(adcParams, adcInitParams);

} // namespace NS_ADC