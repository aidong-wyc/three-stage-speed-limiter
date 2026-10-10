/*!
    \file    app_limiter.c
    \brief   限速业务逻辑实现
*/

#include "app_limiter.h"
#include "bsp_board.h"
#include "bsp_adc.h"
#include "bsp_dac.h"
#include "param_store.h"
#include "systick.h"
#include "modbus_slave.h"

#define LIM_CH_COUNT            2U

/* 上位机下发的量 */
static uint16_t s_limit_state;       /* 0/1/2 */
static uint16_t s_alarm_cmd;         /* 0/1 */
static uint16_t s_work_mode;         /* 0 正常 / 1 学习 */

/* 运行状态 */
static uint16_t s_target_cv[LIM_CH_COUNT];   /* 当前生效的限速电压设定 */
static uint8_t  s_engaged[LIM_CH_COUNT];     /* 继电器是否吸合 */
static uint32_t s_relay_tick[LIM_CH_COUNT];  /* 上次状态变化时刻 */
static uint32_t s_dac_set_tick;              /* DAC 上次改变设定值的时刻 */
static uint16_t s_dac_last_cv[LIM_CH_COUNT]; /* 上次写入 DAC 的值 */

/* 整组吸合状态。参与切换的通道共用这一个决策，保证同步动作 */
static uint8_t  s_group_engaged;
static uint32_t s_group_tick;                /* 组状态上次变化时刻 */
static uint8_t  s_pedal_sw_last;             /* 上一次的踏板信号开关值 */

/* 按键解除 */
static uint8_t  s_btn_stable;        /* 消抖后的按键电平 */
static uint8_t  s_btn_raw_last;
static uint32_t s_btn_tick;
static uint8_t  s_release_active;
static uint32_t s_release_tick;

/* 座椅 / 安全带数字输入消抖 */
static uint8_t  s_seat_stable;
static uint8_t  s_seat_raw_last;
static uint32_t s_seat_tick;
static uint8_t  s_belt_stable;
static uint8_t  s_belt_raw_last;
static uint32_t s_belt_tick;

/* 故障 */
static uint16_t s_fault_code;
static uint32_t s_engage_tick[LIM_CH_COUNT];  /* 本次吸合起始时刻，用于反馈判定延时 */

/*!
    \brief      按限速等级取出该通道应输出的限速电压
*/
static uint16_t limit_cv_for(uint8_t ch, uint16_t level)
{
    const param_t *p = param_get();

    if(1U == level) {
        return (0U == ch) ? p->out1_l2_cv : p->out2_l2_cv;
    }
    if(2U == level) {
        return (0U == ch) ? p->out1_l3_cv : p->out2_l3_cv;
    }

    return 0U;
}

/*!
    \brief      读该通道的加速踏板电压
*/
static uint16_t pedal_cv(uint8_t ch)
{
    return bsp_adc_get_cv((0U == ch) ? ADC_CH_PEDAL1 : ADC_CH_PEDAL2);
}

/*!
    \brief      读该通道的输出反馈电压
*/
static uint16_t feedback_cv(uint8_t ch)
{
    return bsp_adc_get_cv((0U == ch) ? ADC_CH_OUT1_FB : ADC_CH_OUT2_FB);
}

static void relay_apply(uint8_t ch, uint8_t on)
{
    if(0U == ch) {
        bsp_relay0_set(on);
    } else {
        bsp_relay1_set(on);
    }
}

void app_limiter_init(void)
{
    uint8_t i;

    s_limit_state = 0U;
    s_alarm_cmd   = 0U;
    s_work_mode   = 0U;
    s_fault_code  = 0U;

    s_release_active = 0U;
    s_btn_stable     = 0U;
    s_btn_raw_last   = 0U;
    s_btn_tick       = systick_get_ms();
    s_dac_set_tick   = systick_get_ms();

    /* 组状态从释放开始。踏板信号开关的初值直接取参数里的当前值，
       避免第一次 poll 就被当成"开关刚变动"而多做一次整组释放。
       （param_load 已在本函数之前调用，这里能读到落盘值） */
    s_group_engaged = 0U;
    s_group_tick    = systick_get_ms();
    s_pedal_sw_last = (0U != param_get()->limit_mode) ? 1U : 0U;

    /* 座椅/安全带的消抖初值直接取当前电平，
       避免上电瞬间产生一次假跳变 */
    s_seat_stable   = bsp_seat_raw();
    s_seat_raw_last = s_seat_stable;
    s_seat_tick     = systick_get_ms();
    s_belt_stable   = bsp_belt_raw();
    s_belt_raw_last = s_belt_stable;
    s_belt_tick     = systick_get_ms();

    for(i = 0U; i < LIM_CH_COUNT; i++) {
        s_target_cv[i]   = 0U;
        s_engaged[i]     = 0U;
        s_relay_tick[i]  = systick_get_ms();
        s_engage_tick[i] = systick_get_ms();
        s_dac_last_cv[i] = 0U;

        relay_apply(i, 0U);
        bsp_dac_set_cv((dac_ch_t)i, 0U);
    }

    bsp_alarm_set(0U);
}

/*!
    \brief      按键处理：消抖 + 上升沿触发解除限速窗口
*/
static void button_task(void)
{
    uint8_t         raw = bsp_button_get();
    const param_t  *p   = param_get();
    uint32_t        dur_ms;

    /* 电平变化就重新计时，稳定超过消抖时间才认账 */
    if(raw != s_btn_raw_last) {
        s_btn_raw_last = raw;
        s_btn_tick     = systick_get_ms();
    } else if(systick_elapsed(s_btn_tick, LIM_BUTTON_DEBOUNCE_MS)) {
        if(raw != s_btn_stable) {
            s_btn_stable = raw;

            /* EXT_BT+ 与 EXT_BT- 短接 -> 缓冲器输出高 -> 这里看到上升沿 */
            if(0U != s_btn_stable) {
                s_release_active = 1U;
                s_release_tick   = systick_get_ms();
            }
        }
    }

    /* 解除窗口计时。持续时间来自协议 0x11，默认 10s */
    if(s_release_active) {
        dur_ms = (uint32_t)p->release_dur_s * 1000UL;
        if(0U == dur_ms) {
            dur_ms = 1000UL;
        }

        if(systick_elapsed(s_release_tick, dur_ms)) {
            s_release_active = 0U;
        }
    }
}

/*!
    \brief      座椅 / 安全带数字输入消抖
    \note       这两路是 12V 开关量，经 36.5K/10K 分压 + SN74LV1T34
                同相缓冲进 MCU，高电平代表外部有 12V。
                车上电平抖动比按键厉害，消抖时间取得比按键长。
*/
static void din_task(void)
{
    uint8_t raw;

    raw = bsp_seat_raw();
    if(raw != s_seat_raw_last) {
        s_seat_raw_last = raw;
        s_seat_tick     = systick_get_ms();
    } else if(systick_elapsed(s_seat_tick, LIM_DIN_DEBOUNCE_MS)) {
        s_seat_stable = raw;
    }

    raw = bsp_belt_raw();
    if(raw != s_belt_raw_last) {
        s_belt_raw_last = raw;
        s_belt_tick     = systick_get_ms();
    } else if(systick_elapsed(s_belt_tick, LIM_DIN_DEBOUNCE_MS)) {
        s_belt_stable = raw;
    }
}

/*!
    \brief      继电器与 DAC 控制
    \note       踏板信号开关（协议 0x05）决定哪几路继电器参与切换：
                  开(1) -> 只有 RELAY1 动作，RELAY0 保持释放，踏板信号1 直通
                  关(0) -> RELAY0 与 RELAY1 同步动作
                参与的通道共用一个吸合决策（s_group_engaged），
                这样两路要么一起吸、要么一起放，不会有中间态。

                限速电压设定为 0 的通道跳过踏板比较，直接放行：
                输出 0V 不可能高于踏板，没有窜车风险。
                全为 0 时整组无条件吸合，不依赖 ADC。
*/
static void limit_task(void)
{
    uint8_t  i;
    uint8_t  limiting;
    uint8_t  pedal_sw;
    uint8_t  ch_active[LIM_CH_COUNT];
    uint8_t  want;
    uint8_t  desired;
    uint16_t target;
    uint16_t pedal;

    /* 解除窗口内强制不限速 */
    limiting = ((0U != s_limit_state) && (0U == s_release_active)) ? 1U : 0U;

    /* 踏板信号开关来自掉电保存的参数，上位机改完立即生效 */
    pedal_sw = (0U != param_get()->limit_mode) ? 1U : 0U;

    /* 开关为"开"时通道1 不参与切换，加速踏板信号1 原样直通 */
    ch_active[0] = (0U != pedal_sw) ? 0U : 1U;
    ch_active[1] = 1U;

    /* 开关刚被改动：先整组释放，下个周期按新的参与关系重新判断。
       不这样做的话，从"开"切到"关"的瞬间通道1 会在从未校验过
       踏板条件的情况下直接切进去，若踏板1 电压低于限速电压
       就会把信号抬高，造成窜车。 */
    if(pedal_sw != s_pedal_sw_last) {
        s_pedal_sw_last = pedal_sw;
        s_group_engaged = 0U;
        s_group_tick    = systick_get_ms();
    }

    /* ---- 先把限速电压送到 DAC，让继电器 NO 点提前带上正确电压 ----
       两路都照常输出，即使某路当前不参与切换：这样开关一旦拨过来
       不需要再等 DAC 建立时间，0x0F/0x10 的反馈读数也始终有意义。 */
    for(i = 0U; i < LIM_CH_COUNT; i++) {
        target = limiting ? limit_cv_for(i, s_limit_state) : 0U;
        s_target_cv[i] = target;

        if(target != s_dac_last_cv[i]) {
            s_dac_last_cv[i] = target;
            s_dac_set_tick   = systick_get_ms();
            bsp_dac_set_cv((dac_ch_t)i, target);
        }
    }

    /* ---- 整组统一决策 ---- */
    want = 0U;

    if(limiting) {
        want = 1U;

        for(i = 0U; i < LIM_CH_COUNT; i++) {
            if(0U == ch_active[i]) {
                continue;       /* 不参与的通道不影响决策 */
            }

            target = s_target_cv[i];

            /* 设定电压为 0：不做踏板比较，收到限速指令就直接动作。
               这一档 DAC 输出 0V，切过去信号只会比踏板低、不可能更高，
               本来就没有窜车风险，所以防窜车比较可以跳过。
               ！实车效果等同急停：车辆控制器会收到 0V。 */
            if(0U == target) {
                continue;
            }

            /* 下面要用踏板读数。ADC 没就绪时不敢吸合，
               否则会拿着全 0 的假读数去比较，误判成"踏板低于限速电压"。 */
            if(!bsp_adc_ready()) {
                want = 0U;
                continue;
            }

            pedal = pedal_cv(i);

            if(0U != s_group_engaged) {
                /* 已吸合：任一参与通道的踏板跌到限速电压以下就整组放开，
                   否则送出去的信号比司机踩的还高，会窜车 */
                if(pedal <= target) {
                    want = 0U;
                }
            } else {
                /* 未吸合：每个参与通道都要高出一个回差才允许吸合 */
                if(pedal <= (uint32_t)(target + LIM_PEDAL_HYST_CV)) {
                    want = 0U;
                }
            }
        }

        /* 吸合前还要等 DAC 输出建立稳定 */
        if((0U == s_group_engaged) && (0U != want) &&
           !systick_elapsed(s_dac_set_tick, LIM_DAC_SETTLE_MS)) {
            want = 0U;
        }
    }

    /* ---- 组状态变化，带最小保持时间 ---- */
    if(want != s_group_engaged) {
        /* 释放方向不受最小保持时间限制：安全优先，要能立刻松开 */
        if((0U == want) || systick_elapsed(s_group_tick, LIM_RELAY_MIN_DWELL_MS)) {
            s_group_engaged = want;
            s_group_tick    = systick_get_ms();
        }
    }

    /* ---- 落到各路继电器：参与的跟随组状态，不参与的保持释放 ----
       同一次循环里一起写，两路动作间隔只有几条指令。 */
    for(i = 0U; i < LIM_CH_COUNT; i++) {
        desired = ((0U != s_group_engaged) && (0U != ch_active[i])) ? 1U : 0U;

        if(desired != s_engaged[i]) {
            s_engaged[i]    = desired;
            s_relay_tick[i] = systick_get_ms();
            relay_apply(i, desired);

            if(0U != desired) {
                s_engage_tick[i] = systick_get_ms();
            }
        }
    }

    /* 报警输出：限速动作时自动拉高，或上位机单独置位 */
    bsp_alarm_set((limiting || (0U != s_alarm_cmd)) ? 1U : 0U);
}

/*!
    \brief      故障判定
*/
static void fault_task(void)
{
    uint8_t  i;
    uint16_t fault = 0U;
    uint16_t cmd;
    uint16_t fb;
    uint32_t diff;
    uint32_t tol;

    if(!bsp_adc_ready()) {
        return;
    }

    /* 0x02：任一通道原始码值贴 0 或贴满量程，说明信号超出采集范围 */
    if(bsp_adc_out_of_range()) {
        fault = MB_FAULT_ADC_OUT_OF_RANGE;
    }

    /* 0x01：吸合且稳定之后，比较 DAC 设定值与 ADC 反馈值 */
    for(i = 0U; i < LIM_CH_COUNT; i++) {
        if(!s_engaged[i]) {
            continue;
        }
        if(!systick_elapsed(s_engage_tick[i], LIM_FB_CHECK_DELAY_MS)) {
            continue;
        }

        cmd = s_target_cv[i];
        if(cmd < LIM_FB_CHECK_MIN_CV) {
            continue;   /* 设定值太小，按百分比判没意义 */
        }

        fb   = feedback_cv(i);
        diff = (fb > cmd) ? ((uint32_t)fb - cmd) : ((uint32_t)cmd - fb);
        tol  = ((uint32_t)cmd * LIM_FB_TOLERANCE_PCT) / 100UL;

        if(diff > tol) {
            fault = MB_FAULT_DAC_DEVIATION;
        }
    }

    s_fault_code = fault;
}

void app_limiter_poll(void)
{
    button_task();
    din_task();
    limit_task();
    fault_task();
}

uint8_t app_limiter_set_limit_state(uint16_t state)
{
    if(state > 2U) {
        return 0U;
    }

    s_limit_state = state;

    return 1U;
}

uint8_t app_limiter_set_alarm(uint16_t on)
{
    if(on > 1U) {
        return 0U;
    }

    s_alarm_cmd = on;

    return 1U;
}

uint8_t app_limiter_set_work_mode(uint16_t mode)
{
    if(mode > 1U) {
        return 0U;
    }

    s_work_mode = mode;

    return 1U;
}

uint16_t app_limiter_get_limit_state(void)
{
    return s_limit_state;
}

uint16_t app_limiter_get_alarm(void)
{
    return s_alarm_cmd;
}

uint16_t app_limiter_get_work_mode(void)
{
    return s_work_mode;
}

uint16_t app_limiter_get_vehicle_state(void)
{
    uint16_t front = bsp_adc_get_cv(ADC_CH_FRONT);
    uint16_t back  = bsp_adc_get_cv(ADC_CH_BACK);

    if(front >= LIM_DIR_ACTIVE_CV) {
        return LIM_VEHICLE_FORWARD;
    }
    if(back >= LIM_DIR_ACTIVE_CV) {
        return LIM_VEHICLE_BACKWARD;
    }

    return LIM_VEHICLE_IDLE;
}

uint16_t app_limiter_get_release_state(void)
{
    return s_release_active;
}

uint16_t app_limiter_get_seat_present(void)
{
    return (uint16_t)s_seat_stable;
}

uint16_t app_limiter_get_belt_state(void)
{
    return (uint16_t)s_belt_stable;
}

uint16_t app_limiter_get_seat_cv(void)
{
    return s_seat_stable ? LIM_DIN_NOMINAL_CV : 0U;
}

uint16_t app_limiter_get_belt_cv(void)
{
    return s_belt_stable ? LIM_DIN_NOMINAL_CV : 0U;
}

uint16_t app_limiter_get_fault_code(void)
{
    return s_fault_code;
}

uint16_t app_limiter_get_pedal_switch(void)
{
    return (0U != param_get()->limit_mode) ? 1U : 0U;
}

uint8_t app_limiter_relay_engaged(uint8_t ch)
{
    return (ch < LIM_CH_COUNT) ? s_engaged[ch] : 0U;
}

uint16_t app_limiter_target_cv(uint8_t ch)
{
    return (ch < LIM_CH_COUNT) ? s_target_cv[ch] : 0U;
}
