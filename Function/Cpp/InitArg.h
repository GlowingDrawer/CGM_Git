#pragma once

#include "stm32f10x.h"
#include "OLED.h"
#include "SysTickTimer.h"
#include <array>

template <typename Enum_uint8> 
constexpr auto To_uint8(Enum_uint8 e) ->uint8_t { return static_cast<uint8_t>(e); }

template <typename Enum_uint16>
constexpr auto To_uint16(Enum_uint16 e) ->uint16_t { return static_cast<uint16_t>(e); }

template <typename Enum_uint32>
constexpr auto To_uint32(Enum_uint32 e) ->uint32_t { return static_cast<uint32_t>(e); }

// return min_val -- max_val (now_val)
template <typename T>
auto MyCompare(T now_val, T max_value, T min_value = static_cast<T>(0)) -> T { 
    if (now_val > max_value) return max_value;
    else if (now_val < min_value) return min_value;
    else return now_val;
}

void JustShow(const char * str = "Just Show", uint8_t row = 4, uint8_t col = 1);

const float PI = 3.1415926535f;

enum class BaudRate:uint32_t {BAUD_9600 = 9600, BAUD_19200 = 19200, BAUD_38400 = 38400, BAUD_57600 = 57600, BAUD_115200 = 115200};
enum class BufSize:uint16_t {BUF_32 = 32, BUF_64 = 64, BUF_128 = 128, BUF_256 = 256, BUF_END};

namespace CGM
{   
    enum class ADC_GPIO:uint8_t{
        PA0 = ADC_Channel_0, PA1 = ADC_Channel_1, PA2 = ADC_Channel_2, PA3 = ADC_Channel_3, PA4 = ADC_Channel_4, PA5 = ADC_Channel_5, PA6 = ADC_Channel_6, PA7 = ADC_Channel_7, PB0 = ADC_Channel_8, PB1 = ADC_Channel_9, PC0 = ADC_Channel_10, PC1 = ADC_Channel_11, PC2 = ADC_Channel_12, PC3 = ADC_Channel_13, PC4 = ADC_Channel_14, PC5 = ADC_Channel_15
    };
    // 增益使用固定电阻实现还是LMP91000实现
    enum class GainMode{
        RES_VALUE, LMP91000
    };

    // 三电极传感器中Vs（Vworking）对应的电压值变化是动态的还是静态的
    enum class VsMode{
        STATIC, DYNAMIC
    };

    enum class ADC_SampleTime:uint8_t
    {
        Time_1Cycles5 = ADC_SampleTime_1Cycles5,
        Time_7Cycles5 = ADC_SampleTime_7Cycles5,
        Time_13Cycles5 = ADC_SampleTime_13Cycles5,
        Time_28Cycles5 = ADC_SampleTime_28Cycles5,
        Time_41Cycles5 = ADC_SampleTime_41Cycles5,
        Time_55Cycles5 = ADC_SampleTime_55Cycles5,
        Time_71Cycles5 = ADC_SampleTime_71Cycles5,
        Time_239Cycles5 = ADC_SampleTime_239Cycles5,
    };

    struct AD_ChanParams
    {
        uint8_t channel = ADC_Channel_1;
        uint32_t gain = 1;
        GainMode gainMode = GainMode::RES_VALUE;
        uint8_t sampleTime = ADC_SampleTime_55Cycles5;
        VsMode vsMode = VsMode::STATIC;

        AD_ChanParams() = default;
        AD_ChanParams(uint8_t channel, uint32_t gain, uint8_t sample_time = ADC_SampleTime_55Cycles5, GainMode gain_mode = GainMode::RES_VALUE, VsMode vs_mode = VsMode::STATIC) : channel(channel), gain(gain), sampleTime(sample_time), gainMode(gain_mode), vsMode(vs_mode) {}
    };
    
    struct Params
    {
        uint32_t DAC_Chan_RE;
        uint32_t DAC_Chan_WE;
        uint8_t nbr_of_channels;
        std::array<CGM::AD_ChanParams, 16> adChanConfig;
    };
    
    namespace CGM_250305
    {
        const Params params {
            DAC_Channel_2,          // PA5/VR   DAC_Chan_RE
            DAC_Channel_1,          // PA4/VS   DAC_Chan_WE
            2,                      // nbr of channels
            {
                // PA1 20k - 40k    default: 40k ***not connected***
                AD_ChanParams(To_uint8(ADC_GPIO::PA1), 40000, ADC_SampleTime_55Cycles5, GainMode::RES_VALUE, VsMode::STATIC),
                // PA2 300
                AD_ChanParams(To_uint8(ADC_GPIO::PA2), 300, ADC_SampleTime_55Cycles5, GainMode::RES_VALUE, VsMode::STATIC),
            }
        };
        
    } // namespace CGM_250305

    namespace CGM_250409{
        const Params params = {
            DAC_Channel_2,          // PA5/VR   DAC_Chan_RE
            DAC_Channel_1,          // PA4/VS   DAC_Chan_WE
            2,                      // nbr of channels
            {
                AD_ChanParams(To_uint8(ADC_GPIO::PA1), 10000, ADC_SampleTime_55Cycles5, GainMode::RES_VALUE, VsMode::STATIC),    // PA1 10k 15k 20k    default: 10k  
                AD_ChanParams(To_uint8(ADC_GPIO::PA2), 200, ADC_SampleTime_55Cycles5, GainMode::RES_VALUE, VsMode::STATIC),      // PA2 200                          
            }
        };
    }

    namespace CGM_250410{
        const Params params = {
            DAC_Channel_1,          // PA4/VR   DAC_Chan_RE
            DAC_Channel_2,          // PA5/VS   DAC_Chan_WE
            3,                      // nbr of channels  
            {
                // PA1 200 
                AD_ChanParams(To_uint8(ADC_GPIO::PA1), 200, ADC_SampleTime_55Cycles5, GainMode::RES_VALUE, VsMode::STATIC),                        
                // PC0 Designed by LMP91000
                AD_ChanParams(To_uint8(ADC_GPIO::PC0), 300, ADC_SampleTime_55Cycles5, GainMode::LMP91000, VsMode::STATIC),   
                // PC1 Designed by LMP91000 
                AD_ChanParams(To_uint8(ADC_GPIO::PC1), 400, ADC_SampleTime_55Cycles5, GainMode::LMP91000, VsMode::STATIC),    
            }
        };
    }

    namespace CGM_250901{
        const Params params = {
            DAC_Channel_2,          // PA5/VR   DAC_Chan_RE
            DAC_Channel_1,          // PA4/VS   DAC_Chan_WE
            3,                      // nbr of channels  
            {
                // PA1 20400        uric
                AD_ChanParams(To_uint8(ADC_GPIO::PA1), 20400, ADC_SampleTime_55Cycles5, GainMode::RES_VALUE, VsMode::STATIC),                        
                // PA6 4700         ascorbic
                AD_ChanParams(To_uint8(ADC_GPIO::PA6), 4700, ADC_SampleTime_55Cycles5, GainMode::RES_VALUE, VsMode::STATIC),   
                // PA7 200          glucose
                AD_ChanParams(To_uint8(ADC_GPIO::PA7), 200, ADC_SampleTime_55Cycles5, GainMode::RES_VALUE, VsMode::STATIC),    
            }
        };
    }

} // namespace CGM

const IRQn IRQN_NONE = static_cast<IRQn>(0xFF);

namespace TIM
{
    enum class Index:uint8_t { T1 = 0, T2 = 1, T3 = 2, T4 = 3, T5 = 4, T6 = 5, T7 = 6, T8 = 7, END};
    extern const std::array<TIM_TypeDef*, To_uint8(Index::END)> timArr;

    enum class IT_Index:uint8_t { UP = 0, CC1 = 1, CC2 = 2, CC3 = 3, CC4 = 4, COM = 5, END};
    enum class IT:uint16_t {
        UP = TIM_IT_Update, CC1 = TIM_IT_CC1, CC2 = TIM_IT_CC2, CC3 = TIM_IT_CC3, CC4 = TIM_IT_CC4, 
        COM = TIM_IT_COM, END
    };
    extern const std::array<IT, To_uint8(IT_Index::END)> ITMap;
    


    /**
     * @struct  TimStruct
     * @brief   定时器映射结构体，包含定时器寄存器地址、中断号和时钟使能位
     */
    struct TimStruct {
        uint8_t index;        ///< 定时器的索引（0~7）
        TIM_TypeDef *timx;    ///< 定时器寄存器基地址
        IRQn_Type irqn;       ///< 定时器中断号
        uint32_t periph;      ///< 定时器外设时钟使能位
    };

    /**
     * @brief   定时器映射表，包含8个常用定时器的配置信息
     */
    extern const std::array<TimStruct, 8> timMap;

    /**
     * @brief   获取指定定时器的中断号
     * @param   timx    定时器寄存器指针
     * @return  对应的中断号，如果未找到则返回(IRQn_Type)0
     */
    IRQn_Type GetIRQn(TIM_TypeDef * timx);

    /**
     * @brief   控制定时器外设时钟使能状态
     * @param   timx    定时器寄存器指针
     * @param   state   使能状态(ENABLE/DISABLE)
     */
    void RCCPeriphTim(TIM_TypeDef * timx, FunctionalState state = ENABLE);

    /**
     * @brief   初始化定时器为基本定时模式
     * @param   timx        定时器寄存器指针
     * @param   period      定时周期(单位：秒)
     * @param   repeatCounter    重复次数(高级定时器使用)(定时器溢出repeatCounter + 1)后触发更新事件
     */
    void InitTIM(TIM_TypeDef * timx, float period, uint8_t repeatCounter = 0);

    // 重构
    void TIM_ITConfig(TIM_TypeDef * timx, IT tim_it, FunctionalState state);
    
} // namespace TIM

namespace DMA
{
    // DMA_Chan
    enum class ChanIndex : uint8_t { D1CH1 = 0, D1CH2 = 1, D1CH3 = 2, D1CH4 = 3, D1CH5 = 4, D1CH6 = 5, D1CH7 = 6, D2CH1 = 7, D2CH2 = 8, D2CH3 = 9, D2CH4 = 10, D2CH5 = 11, END };
    static const std::array<DMA_Channel_TypeDef*, To_uint8(ChanIndex::END)> chanMap = {DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6, DMA1_Channel7, DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, DMA2_Channel5};

    enum class IT_Index : uint8_t { TC = 0, HT = 1, TE = 2, END };
    enum class IT : uint32_t {
        TC = DMA_IT_TC,
        HT = DMA_IT_HT,
        TE = DMA_IT_TE,
        END
    };
    static const std::array<IT, To_uint8(IT_Index::END)> ITMap = {IT::TC, IT::HT, IT::TE};

    
    /**
     * @struct  DmaStruct
     * @brief   DMA通道映射结构体，包含DMA控制器、通道和中断号
     */
    struct DmaStruct
    {
        DMA_Channel_TypeDef * dmayChanx;    ///< DMA控制器通道
        IRQn_Type irqn;                    ///< 对应的中断号
    };

    /**
     * @brief   DMA通道映射表
     */
    extern const std::array<DmaStruct, static_cast<uint16_t>(ChanIndex::END)> dmaMap;

    uint8_t GetIndexFromType(DMA_Channel_TypeDef * dmay_chanx);
    uint8_t GetItIndexFromType(IT dma_it);
    /**
     * @brief   获取指定DMA通道的中断号
     * @param   dmay_chanx    DMA通道指针
     * @return  对应的中断号，如果未找到则返回0
     */
    IRQn_Type GetDmaIRQn(DMA_Channel_TypeDef * dmay_chanx);

    /**
     * @brief   使能或禁用指定DMA控制器时钟
     * @param   dmax    DMA控制器指针
     * @param   state   使能状态(ENABLE/DISABLE)
     */
    void RCCDmaClockCmd(DMA_TypeDef * dmax, FunctionalState state);

    void DMA_ITConfig(DMA_Channel_TypeDef * dmay_chanx, IT dma_it, FunctionalState state);
} // namespace DMA

namespace USART
{   
    enum class Index:uint8_t { US1 = 0, US2 = 1, US3 = 2, END };
    const std::array<USART_TypeDef*, To_uint8(Index::END)> usartArr = {USART1, USART2, USART3};
    const std::array<IRQn_Type, To_uint8(Index::END)> usartIrqnArr = {USART1_IRQn, USART2_IRQn, USART3_IRQn};
    

    enum class IT_Index:uint8_t { CTS = 0, LIN = 1, ERR = 2, FE = 3, RXNE = 4, IDLE = 5, TXE = 6, TC = 7, PE = 8, ORE = 9, NE = 10, END };
    enum class IT : uint16_t {
        PE    = USART_IT_PE,    // 奇偶校验错误中断：接收数据奇偶校验失败时触发
        TXE   = USART_IT_TXE,   // 发送缓冲区空中断：发送缓冲区为空，可写入新数据
        TC    = USART_IT_TC,    // 传输完成中断：数据帧发送完成（包括停止位）
        RXNE  = USART_IT_RXNE,  // 接收缓冲区非空中断：接收缓冲区有新数据可读
        IDLE  = USART_IT_IDLE,  // 总线空闲中断：检测到总线空闲状态时触发
        LBD   = USART_IT_LBD,   // LIN断开检测中断：LIN模式下检测到断开帧
        CTS   = USART_IT_CTS,   // CTS中断：CTS信号变化（硬件流控制）
        ERR   = USART_IT_ERR,   // 错误汇总中断：任意错误发生（ORE、NE、FE）
        ORE   = USART_IT_ORE,   // 溢出错误中断：新数据覆盖未读取的旧数据
        NE    = USART_IT_NE,    // 噪声错误中断：接收数据被噪声干扰
        FE    = USART_IT_FE,    // 帧错误中断：接收数据帧格式错误（如停止位缺失）
    };
    extern const std::array<IT, To_uint8(IT_Index::END)> ITMap;



    struct USART_Params {
        USART_TypeDef * USART;          // Serial of BT
        uint32_t RCC_Periph_USART;      // 串口外设时钟
        uint32_t GPIO_Periph;           // GPIO外设
        GPIO_TypeDef * GPIOx;          
        uint16_t USART_TX;              // USART_TX使用的GPIO_Pin
        uint16_t USART_RX;              // USART_RX使用的GPIO_Pin
        uint32_t baudrate;              // 波特率
        IRQn USARTx_IRQn;
        DMA_Channel_TypeDef * TX_DMA_Channel;
        DMA_Channel_TypeDef * RX_DMA_Channel;
    };
    extern const std::array<USART_Params, 3> USART_ConfigMap;

    uint8_t GetIndexFromType(USART_TypeDef * usartx);
    uint8_t GetIT_IndexFromType(IT usart_it);
    IRQn GetUSART_IRQnFromType(USART_TypeDef * usartx);
} // namespace USART


namespace MyNVIC {
    bool SetPriority(IRQn_Type irqn, uint8_t pre_priority, uint8_t sub_priority, FunctionalState state = ENABLE);
} // namespace NVIC

namespace STM32PeriphFunc
{
    // 获取DAC通道的使能状态
    FunctionalState DAC_GetCmdStatus(uint32_t DAC_Channel);

    // 获取定时器的使能状态
    FunctionalState TIM_GetCmdStatus(TIM_TypeDef* TIMx);

    // 获取DMA通道的使能状态
    FunctionalState DMA_GetCmdStatus(const DMA_Channel_TypeDef* DMA_Channel);

    // 获取定时器DMA请求的使能状态
    FunctionalState TIM_GetDMACmdStatus(TIM_TypeDef* TIMx, uint16_t TIM_DMASource);
} // namespace STM32PeriphFunc



