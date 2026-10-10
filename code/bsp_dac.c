/*!
    \file    bsp_dac.c
    \brief   两路限速电压输出实现
*/

#include "bsp_dac.h"
#include "bsp_board.h"

/* 记录当前设定值，供读回 */
static uint16_t s_set_cv[DAC_CH_COUNT];

uint16_t bsp_dac_cv_to_raw(uint16_t cv)
{
    uint32_t vdac_mv;
    uint32_t raw;

    if(cv > DAC_OUT_MAX_CV) {
        cv = DAC_OUT_MAX_CV;
    }

    /* 目标输出电压 -> 运放输入（DAC 输出）电压
       cv 单位 0.01V，先换成 mV：cv * 10
       再除以增益：/ (364/100) 等价于 * 100 / 364 */
    vdac_mv = ((uint32_t)cv * 10UL * DAC_GAIN_DEN) / DAC_GAIN_NUM;

    if(vdac_mv > DAC_VREF_MV) {
        vdac_mv = DAC_VREF_MV;
    }

    /* mV -> 12bit 码值 */
    raw = (vdac_mv * DAC_FULL_SCALE) / DAC_VREF_MV;

    if(raw > DAC_FULL_SCALE) {
        raw = DAC_FULL_SCALE;
    }

    return (uint16_t)raw;
}

void bsp_dac_init(void)
{
    s_set_cv[DAC_CH_OUT1] = 0U;
    s_set_cv[DAC_CH_OUT2] = 0U;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_DAC);

    /* DAC 输出引脚必须配成模拟输入模式，
       让 GPIO 的数字部分彻底断开，否则会影响输出精度并增加漏电 */
    gpio_init(BSP_DAC_PORT, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ,
              BSP_DAC0_PIN | BSP_DAC1_PIN);

    dac_deinit(DAC0);

    /* 不用触发，写入数据保持寄存器后立即生效 */
    dac_trigger_disable(DAC0, DAC_OUT0);
    dac_trigger_disable(DAC0, DAC_OUT1);

    /* 关掉输出缓冲。本板 DAC 后面直接接运放的高阻输入，
       不需要缓冲的驱动能力；关掉缓冲能让输出摆幅下探到接近 0V，
       开着缓冲时下限约 0.2V，小电压设定会不准。 */
    dac_output_buffer_disable(DAC0, DAC_OUT0);
    dac_output_buffer_disable(DAC0, DAC_OUT1);

    dac_wave_mode_config(DAC0, DAC_OUT0, DAC_WAVE_DISABLE);
    dac_wave_mode_config(DAC0, DAC_OUT1, DAC_WAVE_DISABLE);

    dac_enable(DAC0, DAC_OUT0);
    dac_enable(DAC0, DAC_OUT1);

    /* 上电输出 0V，保证继电器 NO 点没有电压 */
    dac_data_set(DAC0, DAC_OUT0, DAC_ALIGN_12B_R, 0U);
    dac_data_set(DAC0, DAC_OUT1, DAC_ALIGN_12B_R, 0U);
}

void bsp_dac_set_cv(dac_ch_t ch, uint16_t cv)
{
    uint16_t raw;

    if(ch >= DAC_CH_COUNT) {
        return;
    }

    if(cv > DAC_OUT_MAX_CV) {
        cv = DAC_OUT_MAX_CV;
    }

    s_set_cv[ch] = cv;
    raw = bsp_dac_cv_to_raw(cv);

    dac_data_set(DAC0, (DAC_CH_OUT1 == ch) ? DAC_OUT0 : DAC_OUT1,
                 DAC_ALIGN_12B_R, raw);
}

void bsp_dac_set_raw(dac_ch_t ch, uint16_t raw)
{
    if(ch >= DAC_CH_COUNT) {
        return;
    }

    if(raw > DAC_FULL_SCALE) {
        raw = DAC_FULL_SCALE;
    }

    /* 反算出等效目标电压，保证读回值和实际输出一致 */
    s_set_cv[ch] = (uint16_t)(((uint32_t)raw * DAC_VREF_MV * DAC_GAIN_NUM)
                              / ((uint32_t)DAC_FULL_SCALE * DAC_GAIN_DEN * 10UL));

    dac_data_set(DAC0, (DAC_CH_OUT1 == ch) ? DAC_OUT0 : DAC_OUT1,
                 DAC_ALIGN_12B_R, raw);
}

uint16_t bsp_dac_get_cv(dac_ch_t ch)
{
    if(ch >= DAC_CH_COUNT) {
        return 0U;
    }

    return s_set_cv[ch];
}
