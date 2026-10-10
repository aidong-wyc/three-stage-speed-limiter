/*!
    \file    app_limiter.h
    \brief   限速业务逻辑

    \note    机械接法（来自原理图 Relay 部分，TQ2SA-12V-Z 双刀双掷）：
               LS1: COM=AOUT0(去车辆控制器)  NC=YM0(加速踏板信号1)  NO=ANALOG_O0(本机DAC输出)
               LS2: COM=AOUT1               NC=YM1(加速踏板信号2)  NO=ANALOG_O1
             继电器不吸合时踏板信号原样通过；吸合后踏板被切断，
             改由本机 DAC 经运放放大后的限速电压送给车辆控制器。

             限速等级只决定用哪一组电压：
               限速状态 0 -> 释放，踏板直通
               限速状态 1 -> 二级限速，通道1/2 分别用 0x0B / 0x0C
               限速状态 2 -> 三级限速，通道1/2 分别用 0x0D / 0x0E

             ===== 踏板信号开关（协议 0x05）=====
             决定哪几路继电器参与切换：
               开(1) -> 只有 LS2/RELAY1 动作，LS1/RELAY0 始终释放，
                        加速踏板信号1 原样直通到车辆控制器
               关(0) -> LS1/RELAY0 与 LS2/RELAY1 同步动作
             参与切换的通道共用一个吸合决策，保证"同时跳动"，
             不会出现一路已切成限速电压、另一路还是踏板原信号的中间态
             （车辆控制器一般会对双路踏板信号做一致性校验）。
             开关值掉电保存，改动后立即生效。

             ===== 防突然加速的关键逻辑 =====
             继电器吸合会把车辆控制器看到的信号从"踏板电压"切成"限速电压"。
             若此时踏板电压低于限速电压，切换会让信号变高，车辆反而窜出去。
             所以吸合条件不是一次性判断，而是持续成立才保持吸合。
             又因为参与切换的通道要同步动作，判断按"整组"来做：
               所有参与通道都 踏板电压 > 限速电压 + 回差  ->  整组吸合
               任一参与通道   踏板电压 <= 限速电压        ->  整组立即释放
             回差用于避免踏板在阈值附近时继电器抖动，
             另外加最小保持时间进一步保护继电器触点。
             踏板信号开关被改动时也会先整组释放再重新判断，
             避免把没校验过踏板条件的通道直接切进去。

             ===== 设定电压为 0 的例外 =====
             某通道的限速电压设定为 0 时，该通道不做上面的踏板比较，
             收到限速指令(0x03 非 0)就直接动作：
               这一档 DAC 输出 0V，切过去信号只会比踏板低、不可能更高，
               本来就不存在窜车风险，比较没有意义。
             若参与切换的通道设定值全为 0，则整组无条件吸合，
             此时也不依赖 ADC 读数。
             ！注意实车效果等同急停：车辆控制器收到 0V，车辆不能行驶。
               这一档主要用于台架验证继电器动作，实车投用前
               请确认 0x0B~0x0E 已设成预期的限速电压。
*/

#ifndef APP_LIMITER_H
#define APP_LIMITER_H

#include <stdint.h>

/* ---------------- 可调参数 ---------------- */

/* 踏板与限速电压比较的回差，单位 0.01V */
#define LIM_PEDAL_HYST_CV           15U     /* 0.15V */

/* 继电器两次状态变化之间的最小间隔，ms，防触点抖动 */
#define LIM_RELAY_MIN_DWELL_MS      200U

/* DAC 设定后到允许继电器吸合的建立时间，ms。
   保证 NO 点上的电压已经稳定，切过去不会有跳变 */
#define LIM_DAC_SETTLE_MS           30U

/* 前进/后退信号的有效判定阈值，单位 0.01V（分压后满量程 103.3V） */
#define LIM_DIR_ACTIVE_CV           500U    /* 5.00V */

/* DAC 输出与 ADC 反馈的允许偏差，百分比。超出报故障码 0x01 */
#define LIM_FB_TOLERANCE_PCT        10U

/* 偏差判定前的稳定等待时间，ms */
#define LIM_FB_CHECK_DELAY_MS       300U

/* 做偏差判定的最小限速电压，单位 0.01V。
   设定值太小时按百分比判定没有意义（分辨率不够） */
#define LIM_FB_CHECK_MIN_CV         100U    /* 1.00V */

/* 按键消抖时间，ms */
#define LIM_BUTTON_DEBOUNCE_MS      50U

/* 座椅/安全带数字输入的消抖时间，ms。
   车上抖动比按键厉害，取长一些 */
#define LIM_DIN_DEBOUNCE_MS         100U

/* 座椅、安全带走的是 12V 数字输入（GPIO_IN0/GPIO_IN1），不是模拟采集。
   但协议 0x06/0x07/0x09 要求返回"电压"，这里按有效时折算成标称 12.00V、
   无效时 0V 上报，让上位机拿到一个合理的数值。 */
#define LIM_DIN_NOMINAL_CV          1200U   /* 12.00V */

/* ---------------- 车辆状态（协议 0x13） ---------------- */
#define LIM_VEHICLE_BACKWARD        0U
#define LIM_VEHICLE_FORWARD         1U
#define LIM_VEHICLE_IDLE            2U

/* ---------------- 生命周期 ---------------- */

/*!
    \brief      初始化。必须在 bsp_adc_init / bsp_dac_init / param_load 之后调用
*/
void app_limiter_init(void);

/*!
    \brief      主任务，建议每 10ms 调用一次
*/
void app_limiter_poll(void);

/* ---------------- 上位机写入的量 ---------------- */

/*!
    \brief      设置限速状态
    \param[in]  state: 0 无效 / 1 二级限速 / 2 三级限速
    \retval     1 接受，0 参数非法
*/
uint8_t app_limiter_set_limit_state(uint16_t state);

/*!
    \brief      设置报警信号（协议 0x04）
    \note       限速动作时本模块也会自动拉高 ALARM 输出，
                这里是上位机的独立控制，两者是"或"的关系
*/
uint8_t app_limiter_set_alarm(uint16_t on);

/*!
    \brief      设置工作模式（协议 0x02，0 正常 / 1 学习）
*/
uint8_t app_limiter_set_work_mode(uint16_t mode);

/* ---------------- 上位机读取的量 ---------------- */

uint16_t app_limiter_get_limit_state(void);
uint16_t app_limiter_get_alarm(void);
uint16_t app_limiter_get_work_mode(void);

/*!
    \brief      车辆状态（协议 0x13）：0 后退 / 1 前进 / 2 静止
*/
uint16_t app_limiter_get_vehicle_state(void);

/*!
    \brief      解除限速状态（协议 0x12）：1 表示当前处于按键解除窗口内
*/
uint16_t app_limiter_get_release_state(void);

/*!
    \brief      故障码（协议 0x17）：0 无 / 0x01 DAC偏差过大 / 0x02 ADC超范围
*/
uint16_t app_limiter_get_fault_code(void);

/*!
    \brief      座椅在位状态（协议 0x0A），消抖后的结果
    \retval     1 在位
*/
uint16_t app_limiter_get_seat_present(void);

/*!
    \brief      安全带状态，消抖后的结果
    \retval     1 已扣合
*/
uint16_t app_limiter_get_belt_state(void);

/*!
    \brief      座椅采集"电压"（协议 0x07/0x09），单位 0.01V
    \note       实际是数字输入，按标称 12V 折算
*/
uint16_t app_limiter_get_seat_cv(void);

/*!
    \brief      安全带采集"电压"（协议 0x06），单位 0.01V
*/
uint16_t app_limiter_get_belt_cv(void);

/*!
    \brief      踏板信号开关当前值（协议 0x05）
    \retval     1 开（只有 RELAY1 动作）/ 0 关（两路同步动作）
*/
uint16_t app_limiter_get_pedal_switch(void);

/*!
    \brief      两路继电器当前是否吸合，调试用
*/
uint8_t app_limiter_relay_engaged(uint8_t ch);

/*!
    \brief      当前生效的限速电压设定值，单位 0.01V，调试用
*/
uint16_t app_limiter_target_cv(uint8_t ch);

#endif /* APP_LIMITER_H */
