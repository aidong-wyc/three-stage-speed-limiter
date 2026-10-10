/*!
    \file    bsp_dac.h
    \brief   两路限速电压输出（DAC0/DAC1 + LM321X 同相放大）

    \note    通道1: DAC0 -> PA4 -> R17(100) -> U2(LM321X) -> R18(20) -> ANALOG_O0
             通道2: DAC1 -> PA5 -> R23(100) -> U4(LM321X) -> R24(20) -> ANALOG_O1

             运放增益 = 1 + R22/R21 = 1 + 26.4K/10K = 3.64
             校核：DAC 满量程 3.3V x 3.64 = 12.01V，
                   与反馈分压（9.1K/3.3K，满量程 12.4V）的量程吻合，
                   说明这一级就是设计成 0~12V 输出的。

             ！增益是从原理图阻值推出来的，没有实测。
              上板后建议用 bsp_dac_set_raw() 给一个已知码值，
              万用表量 ANALOG_O0 实测一次，必要时修正下面的宏。
              另外程序里 DAC 与 ADC 反馈的偏差自检（故障码 0x01）
              也会自动暴露增益不准的问题。
*/

#ifndef BSP_DAC_H
#define BSP_DAC_H

#include "gd32f30x.h"

/* 运放增益，用分数表示避免浮点：gain = NUM/DEN = 3.64 */
#define DAC_GAIN_NUM            364U
#define DAC_GAIN_DEN            100U

/* DAC 参考电压，单位 mV（等于 VDDA） */
#define DAC_VREF_MV             3300U

/* 12bit DAC 满码 */
#define DAC_FULL_SCALE          4095U

/* 输出电压上限，单位 0.01V。3.3V x 3.64 = 12.01V */
#define DAC_OUT_MAX_CV          1201U

typedef enum {
    DAC_CH_OUT1 = 0,        /* ANALOG_O0，对应限速通道 1 */
    DAC_CH_OUT2,            /* ANALOG_O1，对应限速通道 2 */
    DAC_CH_COUNT
} dac_ch_t;

/*!
    \brief      初始化两路 DAC，上电输出 0V
*/
void bsp_dac_init(void);

/*!
    \brief      按目标输出电压设置 DAC
    \param[in]  ch: 通道
    \param[in]  cv: ANALOG_Ox 上的目标电压，单位 0.01V，超过上限自动钳位
*/
void bsp_dac_set_cv(dac_ch_t ch, uint16_t cv);

/*!
    \brief      直接写 DAC 码值，调试/标定用
*/
void bsp_dac_set_raw(dac_ch_t ch, uint16_t raw);

/*!
    \brief      读回当前设定的目标电压，单位 0.01V
*/
uint16_t bsp_dac_get_cv(dac_ch_t ch);

/*!
    \brief      电压(0.01V) 转 DAC 码值，供上层做偏差自检时复用
*/
uint16_t bsp_dac_cv_to_raw(uint16_t cv);

#endif /* BSP_DAC_H */
