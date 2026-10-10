/*!
    \file    bsp_adc.c
    \brief   9 路模拟量采集实现
*/

#include "bsp_adc.h"
#include "systick.h"

/* 滑动平均深度，必须是 2 的幂（用移位代替除法） */
#define ADC_FILTER_LEN          16U
#define ADC_FILTER_SHIFT        4U

/* 各通道对应的 ADC 硬件通道号，顺序必须与 adc_ch_t 一致 */
static const uint8_t s_adc_channel[ADC_CH_COUNT] = {
    ADC_CHANNEL_1,      /* AIN1  PA1  预留100V   */
    ADC_CHANNEL_2,      /* AIN2  PA2  前进信号   */
    ADC_CHANNEL_3,      /* AIN3  PA3  参考电压   */
    ADC_CHANNEL_6,      /* AIN5  PA6  后退信号   */
    ADC_CHANNEL_7,      /* AIN6  PA7  踏板信号1  */
    ADC_CHANNEL_14,     /* AIN7  PC4  踏板信号2  */
    ADC_CHANNEL_10,     /* AIN10 PC0  12V 输入   */
    ADC_CHANNEL_11,     /* AIN11 PC1  OUT1 反馈  */
    ADC_CHANNEL_12      /* AIN12 PC2  OUT2 反馈  */
};

/* 各通道的电压换算系数：cV = raw * NUM / DEN
   NUM = 满量程电压 x 100（单位 0.01V），DEN = 4095 x 10
   100K 串阻 -> 满量程 (100000+3300)/3300 x 3.3V = 103.3V -> NUM=10330
   22K  串阻 -> 满量程 (22000+3300)/3300  x 3.3V = 25.3V  -> NUM=2530
   9.1K 串阻 -> 满量程 (9100+3300)/3300   x 3.3V = 12.4V  -> NUM=1240
   为保精度这里把 NUM 放大 10 倍、DEN 用 40950 */
#define ADC_DEN                 40950UL
#define ADC_NUM_100K            103300UL
#define ADC_NUM_22K             25300UL
#define ADC_NUM_9K1             12400UL

static const uint32_t s_adc_num[ADC_CH_COUNT] = {
    ADC_NUM_100K,       /* 预留100V  */
    ADC_NUM_100K,       /* 前进信号  */
    ADC_NUM_100K,       /* 参考电压  */
    ADC_NUM_100K,       /* 后退信号  */
    ADC_NUM_22K,        /* 踏板1     */
    ADC_NUM_22K,        /* 踏板2     */
    ADC_NUM_9K1,        /* 12V 输入  */
    ADC_NUM_9K1,        /* OUT1 反馈 */
    ADC_NUM_9K1         /* OUT2 反馈 */
};

/* DMA 目标缓冲，硬件直接写这里 */
static volatile uint16_t s_dma_buf[ADC_CH_COUNT];

/* 滑动平均 */
static uint16_t s_hist[ADC_CH_COUNT][ADC_FILTER_LEN];
static uint32_t s_sum[ADC_CH_COUNT];
static uint8_t  s_hist_idx;
static uint8_t  s_filled;
static uint8_t  s_out_of_range;

/*!
    \brief      配置 9 个模拟输入引脚为模拟输入模式
*/
static void adc_gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* PA1 PA2 PA3 PA6 PA7 */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ,
              GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);

    /* PC0 PC1 PC2 PC4 */
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_10MHZ,
              GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4);
}

/*!
    \brief      配置 DMA0 通道0，把 ADC0 结果循环搬进 s_dma_buf
*/
static void adc_dma_config(void)
{
    dma_parameter_struct dma_cfg;

    rcu_periph_clock_enable(RCU_DMA0);
    dma_deinit(DMA0, DMA_CH0);

    dma_struct_para_init(&dma_cfg);
    dma_cfg.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_cfg.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_cfg.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_cfg.memory_addr  = (uint32_t)s_dma_buf;
    dma_cfg.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_cfg.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_cfg.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_cfg.number       = (uint32_t)ADC_CH_COUNT;
    dma_cfg.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_cfg);

    /* 循环模式：一轮扫描结束后自动回到缓冲头，持续刷新 */
    dma_circulation_enable(DMA0, DMA_CH0);
    dma_channel_enable(DMA0, DMA_CH0);
}

void bsp_adc_init(void)
{
    uint8_t i;

    s_hist_idx     = 0U;
    s_filled       = 0U;
    s_out_of_range = 0U;
    for(i = 0U; i < (uint8_t)ADC_CH_COUNT; i++) {
        s_sum[i] = 0U;
    }

    adc_gpio_config();

    /* ADC 时钟 = APB2/12 = 10MHz。
       GD32F303 的 ADC 上限是 14MHz，APB2 为 120MHz 时
       只能选 /12(10MHz) 或 /16(7.5MHz)，常见的 /8 会得到 15MHz 超标。 */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV12);
    rcu_periph_clock_enable(RCU_ADC0);

    adc_dma_config();

    adc_deinit(ADC0);
    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, (uint32_t)ADC_CH_COUNT);

    /* 采样时间取最长的 239.5 周期。
       前端是 1K 串阻 + 0.1uF 到地，源阻抗偏高，
       长采样时间保证采样保持电容充分充电，宁慢不错。
       单通道转换时间 = (239.5+12.5)/10MHz = 25.2us，
       9 通道一轮约 227us，对本应用完全够快。 */
    for(i = 0U; i < (uint8_t)ADC_CH_COUNT; i++) {
        adc_routine_channel_config(ADC0, i, s_adc_channel[i], ADC_SAMPLETIME_239POINT5);
    }

    /* 软件触发，配合连续模式只需触发一次 */
    adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC0_1_2_EXTTRIG_ROUTINE_NONE);
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, ENABLE);

    adc_dma_mode_enable(ADC0);
    adc_enable(ADC0);

    /* 使能到校准之间必须留稳定时间（tSTAB）。
       官方例程 ADC0_routine_channel_with_DMA 也是 adc_enable 后 delay_1ms(1)
       再校准；少了这个延时校准结果可能不生效，表现为各通道有固定偏移。 */
    delay_1ms(1U);
    adc_calibration_enable(ADC0);

    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
}

void bsp_adc_poll(void)
{
    uint8_t  i;
    uint8_t  oor = 0U;
    uint16_t raw;

    for(i = 0U; i < (uint8_t)ADC_CH_COUNT; i++) {
        raw = s_dma_buf[i];
        if(raw > ADC_FULL_SCALE) {
            raw = ADC_FULL_SCALE;
        }

        /* 滑动平均：减去将被覆盖的旧值，加上新值 */
        s_sum[i] -= s_hist[i][s_hist_idx];
        s_hist[i][s_hist_idx] = raw;
        s_sum[i] += raw;

        if((raw < ADC_RAW_LOW_LIMIT) || (raw > ADC_RAW_HIGH_LIMIT)) {
            oor = 1U;
        }
    }

    s_hist_idx++;
    if(s_hist_idx >= ADC_FILTER_LEN) {
        s_hist_idx = 0U;
        s_filled   = 1U;
    }

    s_out_of_range = oor;
}

uint16_t bsp_adc_get_raw(adc_ch_t ch)
{
    if(ch >= ADC_CH_COUNT) {
        return 0U;
    }

    return (uint16_t)(s_sum[ch] >> ADC_FILTER_SHIFT);
}

uint16_t bsp_adc_get_cv(adc_ch_t ch)
{
    uint32_t raw;
    uint32_t cv;

    if(ch >= ADC_CH_COUNT) {
        return 0U;
    }

    raw = (uint32_t)(s_sum[ch] >> ADC_FILTER_SHIFT);
    cv  = (raw * s_adc_num[ch]) / ADC_DEN;

    if(cv > 0xFFFFU) {
        cv = 0xFFFFU;
    }

    return (uint16_t)cv;
}

uint8_t bsp_adc_ready(void)
{
    return s_filled;
}

uint8_t bsp_adc_out_of_range(void)
{
    return s_out_of_range;
}
