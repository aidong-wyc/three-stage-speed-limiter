/*!
    \file    bsp_adc.h
    \brief   9 路模拟量采集（ADC0 + DMA 循环扫描 + 滑动平均）

    \note    通道映射依据原理图推导，锚点是 DAC_OUT0/DAC_OUT1：
             DAC0_OUT 硬件固定在 PA4、DAC1_OUT 固定在 PA5，
             原理图上这两个网络标号正好落在引脚 20/21，
             由此确定"网络标号所在行 = 引脚号"，再往下推出 AIN5/6/7。

             ！！注意：网络名 AIN5/AIN6/AIN7 与 ADC 通道号并不一致 ！！
             AIN1/2/3 -> PA1/PA2/PA3 = 通道 1/2/3   （名字与通道号一致）
             AIN10/11/12 -> PC0/PC1/PC2 = 通道 10/11/12（一致）
             AIN5 -> PA6  = 通道 6    （名字 5，实际通道 6）
             AIN6 -> PA7  = 通道 7    （名字 6，实际通道 7）
             AIN7 -> PC4  = 通道 14   （名字 7，实际通道 14）
             推测原因：设计时本想用 PA5/PA6/PA7（通道5/6/7），
             后来 PA5 被 DAC1 占用，引脚整体下移一位但网络名没改。

             分压比（J4 侧串阻 + 3.3K 下拉）：
               100K  -> 满量程 103.3V   （预留100V/前信号/参考/后信号）
               22K   -> 满量程 25.3V    （两路加速踏板）
               9.1K  -> 满量程 12.4V    （12V输入/两路输出反馈）

             ADC 时钟：APB2=120MHz，GD32F303 的 ADC 上限 14MHz，
             因此预分频取 /12 -> 10MHz，不能用常见的 /8（会是 15MHz 超标）。
*/

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "gd32f30x.h"

/*!
    \brief  采集通道索引（按扫描顺序排列）
*/
typedef enum {
    ADC_CH_RSVD100 = 0,     /* AIN1  PA1  通道1   100K  预留 100V 采样      */
    ADC_CH_FRONT,           /* AIN2  PA2  通道2   100K  车辆前进信号        */
    ADC_CH_VREF,            /* AIN3  PA3  通道3   100K  参考电压正极        */
    ADC_CH_BACK,            /* AIN5  PA6  通道6   100K  车辆后退信号        */
    ADC_CH_PEDAL1,          /* AIN6  PA7  通道7   22K   加速踏板信号1       */
    ADC_CH_PEDAL2,          /* AIN7  PC4  通道14  22K   加速踏板信号2       */
    ADC_CH_SUPPLY12,        /* AIN10 PC0  通道10  9.1K  外部 12V 输入检测   */
    ADC_CH_OUT1_FB,         /* AIN11 PC1  通道11  9.1K  OUT1 输出电压反馈   */
    ADC_CH_OUT2_FB,         /* AIN12 PC2  通道12  9.1K  OUT2 输出电压反馈   */
    ADC_CH_COUNT
} adc_ch_t;

/* 12bit ADC 满码 */
#define ADC_FULL_SCALE          4095U

/* 采集异常判定阈值：接近 0 码或满码认为信号超范围（协议故障码 0x02） */
#define ADC_RAW_LOW_LIMIT       8U
#define ADC_RAW_HIGH_LIMIT      4088U

/*!
    \brief      初始化 ADC0 + DMA，启动连续扫描
    \note       启动后 DMA 会不停地把 9 个通道的转换结果刷进内部缓冲，
                无需 CPU 干预。调用 bsp_adc_poll() 做滤波即可。
*/
void bsp_adc_init(void);

/*!
    \brief      滤波任务，建议每 5~10ms 调用一次
    \note       从 DMA 缓冲取最新一轮扫描结果，喂进各通道的滑动平均。
*/
void bsp_adc_poll(void);

/*!
    \brief      取滤波后的原始码值（0~4095）
*/
uint16_t bsp_adc_get_raw(adc_ch_t ch);

/*!
    \brief      取换算后的信号电压，单位 0.01V（协议规定的单位）
    \note       返回的是分压前的真实信号电压，已按各通道的分压比还原。
*/
uint16_t bsp_adc_get_cv(adc_ch_t ch);

/*!
    \brief      滤波器是否已经填满（上电初期数据还不可信）
    \retval     1 已就绪
*/
uint8_t bsp_adc_ready(void);

/*!
    \brief      是否有通道的原始码值超出合理范围
    \retval     1 有通道超范围，对应协议故障码 0x02
*/
uint8_t bsp_adc_out_of_range(void);

#endif /* BSP_ADC_H */
