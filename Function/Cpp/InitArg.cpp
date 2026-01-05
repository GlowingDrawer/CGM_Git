#include <InitArg.h>
#include <unordered_map>


void JustShow(const char * str, uint8_t row, uint8_t col) {
    OLED_ShowString(row, col, str);
}

// 1. 初始化静态成员变量
volatile uint32_t SysTickTimer::msTicks = 0;

// 2. 方法实现
void SysTickTimer::Init() {
    // SystemCoreClock 在 system_stm32f10x.c 中定义，通常为 72000000
    // 配置 SysTick 为 1ms 中断一次
    if (SysTick_Config(SystemCoreClock / 1000)) {
        // 初始化失败处理（通常是时钟配置错误）
        while (1);
    }
    // 默认 SysTick 优先级最低，如果需要提高优先级可在此配置
    // NVIC_SetPriority(SysTick_IRQn, 0); 
}

uint32_t SysTickTimer::GetTick() {
    return msTicks;
}

void SysTickTimer::DelayMs(uint32_t ms) {
    uint32_t startTick = GetTick();
    // 利用无符号整数溢出回绕特性，直接相减也是安全的
    while ((GetTick() - startTick) < ms) {
        // 空转等待
    }
}

void SysTickTimer::IncTick() {
    msTicks++;
}

// 3. 中断服务函数连接
// 必须使用 extern "C" 防止 C++ 名称修饰，否则启动文件找不到该函数
extern "C" {
    void SysTick_Handler(void) {
        SysTickTimer::IncTick();
    }
}


namespace TIM {
    const std::array<TIM_TypeDef*, To_uint8(Index::END)> timArr = {TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8};

    const std::array<IT, To_uint8(IT_Index::END)> ITMap = {IT::UP, IT::CC1, IT::CC2, IT::CC3, IT::CC4, IT::COM};
    const std::array<TimStruct, 8> timMap = {{
        // index | 定时器基地址 | 中断号            | 外设时钟使能位
        {0,      TIM1,          TIM1_UP_IRQn,      RCC_APB2Periph_TIM1},
        {1,      TIM2,          TIM2_IRQn,         RCC_APB1Periph_TIM2},
        {2,      TIM3,          TIM3_IRQn,         RCC_APB1Periph_TIM3},
        {3,      TIM4,          TIM4_IRQn,         RCC_APB1Periph_TIM4},
        {4,      TIM5,          TIM5_IRQn,         RCC_APB1Periph_TIM5},
        {5,      TIM6,          TIM6_IRQn,         RCC_APB1Periph_TIM6},
        {6,      TIM7,          TIM7_IRQn,         RCC_APB1Periph_TIM7},
        {7,      TIM8,          TIM8_UP_IRQn,      RCC_APB2Periph_TIM8}
    }};

    IRQn GetIRQn(TIM_TypeDef * timx){
        auto irqn = static_cast<IRQn>(IRQN_NONE);
        for(auto &tim : timMap){
            if (tim.timx == timx) {
                irqn = tim.irqn;
            } 
        }
        return irqn;
    }

    uint8_t GetIndex(TIM_TypeDef * timx){
        for (uint8_t i = 0; i < timArr.size(); i++) {
            if (timArr[i] == timx) {
                return i;
            }
        }
        return 0;
    }
    void RCCPeriphTim(TIM_TypeDef * timx, FunctionalState state){
        if(timx == TIM1 || timx == TIM8) {
            // 高级定时器使用APB2总线时钟使能函数
            RCC_APB2PeriphClockCmd(timMap[timx == TIM1 ? 0 : 7].periph, state);
        } else {
            // 通用定时器使用APB1总线时钟使能函数
            for (size_t i = 1; i < timMap.size(); i++) {
                if(timMap[i].timx == timx) {
                    RCC_APB1PeriphClockCmd(timMap[i].periph, state);
                    break;
                }
            }
        }
    }

    void InitTIM(TIM_TypeDef * timx, float period, uint8_t repeatCounter){
        RCCPeriphTim(timx);
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        // 10k/s stepDuration ms
        TIM_TimeBaseStructure.TIM_Period = 10000 * (MyCompare<float>(period, 6)) - 1;     
        // 72MHz/7200 = 10kHz
        TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;      
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = repeatCounter;
        TIM_TimeBaseInit(timx, &TIM_TimeBaseStructure);
    }

    void TIM_ITConfig(TIM_TypeDef * timx, IT tim_it, FunctionalState state){
        TIM_ITConfig(timx, static_cast<uint16_t>(tim_it), state);
    }
}

namespace DMA
{
    /**
     * @brief   DMA映射表，包含常用DMA通道配置信息
     */
    const std::array<DmaStruct, static_cast<uint16_t>(ChanIndex::END)> dmaMap = {
        {
            {DMA1_Channel1, IRQn::DMA1_Channel1_IRQn},
            {DMA1_Channel2, IRQn::DMA1_Channel2_IRQn},
            {DMA1_Channel3, IRQn::DMA1_Channel3_IRQn},
            {DMA1_Channel4, IRQn::DMA1_Channel4_IRQn},
            {DMA1_Channel5, IRQn::DMA1_Channel5_IRQn},
            {DMA1_Channel6, IRQn::DMA1_Channel6_IRQn},
            {DMA1_Channel7, IRQn::DMA1_Channel7_IRQn},
            {DMA2_Channel1, IRQn::DMA2_Channel1_IRQn},
            {DMA2_Channel2, IRQn::DMA2_Channel2_IRQn},
            {DMA2_Channel3, IRQn::DMA2_Channel3_IRQn},
            {DMA2_Channel4, IRQn::DMA2_Channel4_5_IRQn},
            {DMA2_Channel5, IRQn::DMA2_Channel4_5_IRQn},
        }
    };

    uint8_t GetIndexFromType(DMA_Channel_TypeDef * dmay_chanx){
        uint8_t index = 0;  
        for (auto dma_chan : dmaMap) {
            if (dmay_chanx == dma_chan.dmayChanx) {
                break;
            }
            ++index;
        }
        return index;
    }
    uint8_t GetItIndexFromType(IT dma_it){
        uint8_t index = 0;
        for (auto dma_it_ : ITMap) {
            if (dma_it == dma_it_) {
                break;
            }
            ++index;
        }
        return index;
    }

    /**
     * @brief   获取指定DMA数据流的中断号
     * @param   dmax    DMA控制器指针
     * @param   stream  数据流编号
     * @return  对应的中断号，如果未找到则返回(IRQn_Type)0
     */
    IRQn_Type GetDmaIRQn(DMA_Channel_TypeDef * dmay_chanx)
    {
        for (const auto &dma : dmaMap) {
            if(dma.dmayChanx == dmay_chanx) {
                return dma.irqn;
            }
        }
        return IRQN_NONE;
    }

    /**
     * @brief   使能或禁用指定DMA控制器时钟
     * @param   dmax    DMA控制器指针
     * @param   state   使能状态(ENABLE/DISABLE)
     */
    void RCCDmaClockCmd(DMA_TypeDef * dmax, FunctionalState state)
    {
        if (dmax == DMA1) {
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, state);
        } else if (dmax == DMA2) {
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, state);
        }
    }

    void DMA_ITConfig(DMA_Channel_TypeDef * dmay_chanx, IT dma_it, FunctionalState state) {
        DMA_ITConfig(dmay_chanx, static_cast<uint32_t>(dma_it), state);
    }

    uint8_t GetChanIndex(DMA_Channel_TypeDef * dmay_chanx) {
        uint8_t index = 0;
        for (auto &dma : dmaMap) {
            if (dmay_chanx == dma.dmayChanx) {
                break;
            }
            index++;
        }
        for (auto &chan : chanMap) {
            if (chan == dmay_chanx) {
                break;
            }
            ++index;
        }
        return index;
    }

    uint8_t  GetIT_Index(IT dma_it) {
        uint8_t  index = 0;
        for (auto &it : ITMap) {
            if (dma_it == it) {
                break;
            }
            index++;
        }
        return index;
    }


} // namespace DMA

namespace USART
{
    constexpr const std::array<IT, static_cast<int>(IT_Index::END)> ITMap = {
        IT::PE, IT::TXE, IT::TC, IT::RXNE, IT::IDLE, IT::LBD, IT::CTS, IT::ERR, IT::ORE, IT::NE, IT::FE
    };

    void RCCPeriphUsart(USART_TypeDef * usartx, FunctionalState state = ENABLE)
    {
        if (usartx == USART1) {
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, state);
        } else if (usartx == USART2) {
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, state);
        } else if (usartx == USART3) {
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, state);
        }
    }

    //  USART参数结构体
    //  USART_TypeDef * USART;          // Serial of BT
    //     uint32_t RCC_Periph_USART;      // 串口外设时钟
    //     uint32_t GPIO_Periph;           // GPIO外设
    //     GPIO_TypeDef * GPIOx;          
    //     uint16_t USART_TX;              // USART_TX使用的GPIO_Pin
    //     uint16_t USART_RX;              // USART_RX使用的GPIO_Pin
    //     uint32_t baudrate;              // 波特率
    //     IRQn USARTx_IRQn;
    //     DMA_Channel_TypeDef * TX_DMA_Channel;
    //     DMA_Channel_TypeDef * RX_DMA_Channel;

    const std::array<USART_Params, 3> USART_ConfigMap = {
        {
            { USART1, RCC_APB2Periph_USART1, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_9, GPIO_Pin_10, 115200, USART1_IRQn, DMA1_Channel4, DMA1_Channel5 },
            { USART2, RCC_APB1Periph_USART2, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_2, GPIO_Pin_3, 115200, USART2_IRQn, DMA1_Channel6, DMA1_Channel7 },
            { USART3, RCC_APB1Periph_USART3, RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_10, GPIO_Pin_11, 115200, USART3_IRQn, DMA1_Channel2, DMA1_Channel3 }
        }
    };

    void MyUSART_ITConfig(USART_TypeDef * usartx, IT usart_it, FunctionalState state) {
        USART_ITConfig(usartx, static_cast<uint16_t>(usart_it), state);
    }

    uint8_t GetIndexFromType(USART_TypeDef * usartx) {
        for (uint8_t i = 0; i < USART_ConfigMap.size(); i++) {
            if (USART_ConfigMap[i].USART == usartx) {
                return i;
            }
        }
        return 0;
    }

    uint8_t GetIT_IndexFromType(IT usart_it) {
        uint8_t index = 0;
        for (index = 0; index < ITMap.size(); index++) {
            if (usart_it == ITMap[index]) {
                break;
            }
        }
        return index;
    }

    IRQn GetUSART_IRQnFromType(USART_TypeDef * usartx) {
        uint8_t index = 0;
        for (index = 0; index < USART_ConfigMap.size(); index++) {
            if (usartx == USART_ConfigMap[index].USART) {
                break;
            }
        }
        return USART_ConfigMap[index].USARTx_IRQn;
    }
} // namespace USART

namespace MyNVIC {
    bool SetPriority(IRQn_Type irqn, uint8_t pre_priority, uint8_t sub_priority, FunctionalState state) {
        bool result = true;
        if (irqn != IRQN_NONE) {
            NVIC_InitTypeDef NVIC_InitStructure;
            NVIC_InitStructure.NVIC_IRQChannelCmd = state;
            NVIC_InitStructure.NVIC_IRQChannel = irqn;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = pre_priority;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub_priority;
            NVIC_Init(&NVIC_InitStructure);
        } else {
            result = false;
        }
        return result;
    }
} // namespace NVIC

namespace STM32PeriphFunc
{
    // 获取DAC通道的使能状态
    FunctionalState DAC_GetCmdStatus(uint32_t DAC_Channel) {
        // 检查DAC通道1的使能位
        if (DAC_Channel == DAC_Channel_1) {
            return (DAC->CR & DAC_CR_EN1) ? ENABLE : DISABLE;
        }
        // 检查DAC通道2的使能位
        else if (DAC_Channel == DAC_Channel_2) {
            return (DAC->CR & DAC_CR_EN2) ? ENABLE : DISABLE;
        }
        return DISABLE;
    }

    /**
     * @brief 获取定时器的使能状态
     * @param TIMx: 定时器外设 (TIM1-TIM8等)
     * @return 使能状态: ENABLE 或 DISABLE
     */
    FunctionalState TIM_GetCmdStatus(TIM_TypeDef* TIMx) {
        // 检查定时器的计数器使能位 (CEN位)
        return (TIMx->CR1 & TIM_CR1_CEN) ? ENABLE : DISABLE;
    }

    /**
     * @brief 获取DMA通道的使能状态
     * @param DMA_Channel: DMA通道 (DMA1_Channel1-DMA1_Channel7等)
     * @return 使能状态: ENABLE 或 DISABLE
     */
    FunctionalState DMA_GetCmdStatus(const DMA_Channel_TypeDef* DMA_Channel) {
        // DMA通道使能位位于CCR寄存器的第0位，直接使用位位置判断
        // 标准库中无统一的DMA_CCR_EN宏，不同通道为DMA_CCR1_EN、DMA_CCR2_EN等
        return (DMA_Channel->CCR & (1 << 0)) ? ENABLE : DISABLE;
    }

    /**
     * @brief 获取定时器DMA请求的使能状态
     * @param TIMx: 定时器外设
     * @param TIM_DMASource: 定时器DMA请求源 (如TIM_DMA_Update等)
     * @return 使能状态: ENABLE 或 DISABLE
     */
    FunctionalState TIM_GetDMACmdStatus(TIM_TypeDef* TIMx, uint16_t TIM_DMASource) {
        // 检查对应的DMA请求使能位
        return (TIMx->DIER & TIM_DMASource) ? ENABLE : DISABLE;
    }

} // namespace STM32PeriphFunc


    

