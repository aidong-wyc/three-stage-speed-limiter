/*!
    \file    main.c
    \brief   电子限速器主控板 —— 应用主程序

    \note    MCU  : GD32F303RBT6, 外部晶振 24MHz, 系统时钟 120MHz
             RS485: USART2 (PB10/PB11) + PB2 方向控制, Modbus RTU 9600 8N1, 从站 0x05
             调试 : USART0 (PA9/PA10) 115200, printf 输出

             整机功能：
             - 9 路模拟量采集（踏板信号、方向信号、12V 供电、输出反馈等）
             - 2 路限速电压输出（DAC + 运放 -> 继电器 NO 点）
             - 继电器串在加速踏板信号线上，限速时切断踏板改送限速电压
             - 限速动作时 ALARM 输出 12V
             - EXT_BT 短接触发解除限速
             - 二/三级限速电压等参数掉电保存在片内 Flash
*/

#include <stdio.h>
#include <stddef.h>

#include "gd32f30x.h"
#include "systick.h"
#include "bsp_board.h"
#include "bsp_debug_uart.h"
#include "bsp_adc.h"
#include "bsp_dac.h"
#include "rs485.h"
#include "modbus_crc.h"
#include "modbus_slave.h"
#include "param_store.h"
#include "app_limiter.h"

/* 固件版本：格式 YYMMDDVV，与协议 0x30 寄存器说明一致。
   0x26091701 = 26年09月17日第01版：新增踏板信号开关(0x05)逻辑、
   两路继电器同步动作、参数落盘日志区分真写与跳过。 */
#define FW_VERSION              0x26091701UL

/* RS485 波特率，协议规定 9600 */
#define RS485_BAUDRATE          9600U

/* 任务周期 */
#define TASK_ADC_PERIOD_MS      5U      /* ADC 滤波 */
#define TASK_APP_PERIOD_MS      10U     /* 限速逻辑 */
#define TASK_STAT_PERIOD_MS     5000U   /* 状态打印 */

/* 参数延迟落盘：最后一次修改后再等这么久才写 Flash。
   上位机用 0x10 一次写多个寄存器时，这样只擦写一次。 */
#define PARAM_COMMIT_DELAY_MS   1000U

/* 收到 0x16 复位命令的标志 */
static volatile uint8_t s_reset_pending = 0U;

/* 参数待落盘标志与最后修改时刻 */
static uint8_t  s_param_pending = 0U;
static uint32_t s_param_tick    = 0U;

/*!
    \brief      标记参数有改动，等主循环延迟落盘
*/
static void param_touch(void)
{
    s_param_pending = 1U;
    s_param_tick    = systick_get_ms();
}

/* ==================== Modbus 寄存器读回调 ==================== */

/*!
    \brief      读寄存器
    \note       传感器ID(0x00/0x01) 的字节序按协议原文
                "从低到高依次 00 aa bb cc" 实现，即低字节先上线；
                其余寄存器一律高位在前。这一条与常规 Modbus 大端约定不同，
                联调时需与上位机确认。
*/
static uint8_t app_reg_read(uint16_t addr, uint16_t *value)
{
    uint32_t       uid = bsp_mcu_uid32();
    const param_t *p   = param_get();

    switch(addr) {
    case MB_REG_SENSOR_ID:              /* 0x00 ID 低 2 字节，低字节先上线 */
        *value = (uint16_t)(((uid & 0x000000FFUL) << 8) | ((uid & 0x0000FF00UL) >> 8));
        break;

    case (MB_REG_SENSOR_ID + 1U):       /* 0x01 ID 高 2 字节 */
        *value = (uint16_t)(((uid & 0x00FF0000UL) >> 8) | ((uid & 0xFF000000UL) >> 24));
        break;

    case MB_REG_WORK_MODE:              /* 0x02 */
        *value = app_limiter_get_work_mode();
        break;

    case MB_REG_LIMIT_STATE:            /* 0x03 */
        *value = app_limiter_get_limit_state();
        break;

    case MB_REG_ALARM:                  /* 0x04 */
        *value = app_limiter_get_alarm();
        break;

    case MB_REG_LIMIT_MODE:             /* 0x05 */
        *value = p->limit_mode;
        break;

    /* 座椅与安全带走 12V 数字输入（GPIO_IN0/GPIO_IN1），不是模拟采集。
       协议这几个寄存器要求返回"电压"，所以按有效=标称12.00V、
       无效=0V 折算上报。 */
    case MB_REG_BELT_VOLT:              /* 0x06 安全带采集电压 */
        *value = app_limiter_get_belt_cv();
        break;

    case MB_REG_SEAT_VOLT_RAW:          /* 0x07 座椅采集电压 */
    case MB_REG_SEAT_VOLT:              /* 0x09 座椅采集电压 */
        *value = app_limiter_get_seat_cv();
        break;

    case MB_REG_SEAT_PRESENT:           /* 0x0A 座椅在位状态 */
        *value = app_limiter_get_seat_present();
        break;

    /* 0x08 座椅在位电压：原本是模拟采集时的判定阈值。
       现在座椅改成数字输入，这个阈值已经不参与任何判断，
       仅为兼容协议保留可读写并掉电保存。 */
    case MB_REG_SEAT_PRESENT_VOLT:      /* 0x08 */
        *value = p->seat_present_cv;
        break;

    case MB_REG_OUT1_L2_VOLT:           /* 0x0B */
        *value = p->out1_l2_cv;
        break;

    case MB_REG_OUT2_L2_VOLT:           /* 0x0C */
        *value = p->out2_l2_cv;
        break;

    case MB_REG_OUT1_L3_VOLT:           /* 0x0D */
        *value = p->out1_l3_cv;
        break;

    case MB_REG_OUT2_L3_VOLT:           /* 0x0E */
        *value = p->out2_l3_cv;
        break;

    case MB_REG_OUT1_VOLT:              /* 0x0F 通道1输出电压反馈 AIN11 */
        *value = bsp_adc_get_cv(ADC_CH_OUT1_FB);
        break;

    case MB_REG_OUT2_VOLT:              /* 0x10 通道2输出电压反馈 AIN12 */
        *value = bsp_adc_get_cv(ADC_CH_OUT2_FB);
        break;

    case MB_REG_RELEASE_DURATION:       /* 0x11 */
        *value = p->release_dur_s;
        break;

    case MB_REG_RELEASE_STATE:          /* 0x12 */
        *value = app_limiter_get_release_state();
        break;

    case MB_REG_VEHICLE_STATE:          /* 0x13 */
        *value = app_limiter_get_vehicle_state();
        break;

    case MB_REG_SUPPLY_VOLT:            /* 0x15 供电电压 AIN10 */
        *value = bsp_adc_get_cv(ADC_CH_SUPPLY12);
        break;

    case MB_REG_FAULT_CODE:             /* 0x17 */
        *value = app_limiter_get_fault_code();
        break;

    case MB_REG_IN1_VOLT:               /* 0x18 加速踏板信号1 AIN6 */
        *value = bsp_adc_get_cv(ADC_CH_PEDAL1);
        break;

    case MB_REG_IN2_VOLT:               /* 0x19 加速踏板信号2 AIN7 */
        *value = bsp_adc_get_cv(ADC_CH_PEDAL2);
        break;

    case MB_REG_VERSION:                /* 0x30 版本高 16bit */
        *value = (uint16_t)((FW_VERSION >> 16) & 0xFFFFUL);
        break;

    case (MB_REG_VERSION + 1U):         /* 0x31 版本低 16bit */
        *value = (uint16_t)(FW_VERSION & 0xFFFFUL);
        break;

    default:
        /* 协议表里未定义的地址（0x14、0x1A~0x2F 等） */
        return MB_EX_ILLEGAL_ADDRESS;
    }

    return 0U;
}

/* ==================== Modbus 寄存器写回调 ==================== */

/*!
    \brief      写寄存器
    \note       落到 param_t 里的项只改内存并打脏标记，
                由主循环延迟统一落盘，避免一次多寄存器写触发多次擦写。
*/
static uint8_t app_reg_write(uint16_t addr, uint16_t value)
{
    param_t np = *param_get();

    switch(addr) {
    case MB_REG_WORK_MODE:              /* 0x02 */
        if(!app_limiter_set_work_mode(value)) {
            return MB_EX_ILLEGAL_VALUE;
        }
        break;

    case MB_REG_LIMIT_STATE:            /* 0x03 限速指令 */
        if(!app_limiter_set_limit_state(value)) {
            return MB_EX_ILLEGAL_VALUE;
        }
        break;

    case MB_REG_ALARM:                  /* 0x04 */
        if(!app_limiter_set_alarm(value)) {
            return MB_EX_ILLEGAL_VALUE;
        }
        break;

    case MB_REG_LIMIT_MODE:             /* 0x05 踏板信号开关 */
        /* 只认 0/1：0 两路继电器同步动作，1 只有 RELAY1 动作。
           取值一旦超出范围就回非法数据值，避免上位机把
           界面上的脏数据写进来、又因为掉电保存一直留着。 */
        if(value > 1U) {
            return MB_EX_ILLEGAL_VALUE;
        }
        np.limit_mode = value;
        param_set(&np);
        param_touch();
        break;

    case MB_REG_SEAT_PRESENT_VOLT:      /* 0x08 */
        np.seat_present_cv = value;
        param_set(&np);
        param_touch();
        break;

    /* 0x0B~0x0E 二/三级限速电压，掉电保存 */
    case MB_REG_OUT1_L2_VOLT:           /* 0x0B */
        if(value > DAC_OUT_MAX_CV) {
            return MB_EX_ILLEGAL_VALUE;
        }
        np.out1_l2_cv = value;
        param_set(&np);
        param_touch();
        break;

    case MB_REG_OUT2_L2_VOLT:           /* 0x0C */
        if(value > DAC_OUT_MAX_CV) {
            return MB_EX_ILLEGAL_VALUE;
        }
        np.out2_l2_cv = value;
        param_set(&np);
        param_touch();
        break;

    case MB_REG_OUT1_L3_VOLT:           /* 0x0D */
        if(value > DAC_OUT_MAX_CV) {
            return MB_EX_ILLEGAL_VALUE;
        }
        np.out1_l3_cv = value;
        param_set(&np);
        param_touch();
        break;

    case MB_REG_OUT2_L3_VOLT:           /* 0x0E */
        if(value > DAC_OUT_MAX_CV) {
            return MB_EX_ILLEGAL_VALUE;
        }
        np.out2_l3_cv = value;
        param_set(&np);
        param_touch();
        break;

    case MB_REG_RELEASE_DURATION:       /* 0x11 解除限速持续时间，秒 */
        if((0U == value) || (value > 600U)) {
            return MB_EX_ILLEGAL_VALUE;
        }
        np.release_dur_s = value;
        param_set(&np);
        param_touch();
        break;

    case MB_REG_DEVICE_RESET:           /* 0x16 */
        if(0U != value) {
            s_reset_pending = 1U;
        }
        break;

    default:
        /* 只读寄存器或未定义地址 */
        return MB_EX_ILLEGAL_ADDRESS;
    }

    return 0U;
}

static const modbus_reg_ops_t s_reg_ops = {
    app_reg_read,
    app_reg_write
};

/* ==================== 打印 ==================== */

static void print_banner(uint8_t param_loaded)
{
    const param_t *p = param_get();

    printf("\r\n\r\n==== Speed Limiter Control Board ====\r\n");
    printf("MCU        : GD32F303RBT6\r\n");
    printf("CK_SYS     : %u Hz\r\n", (unsigned)rcu_clock_freq_get(CK_SYS));
    printf("CK_APB1/2  : %u / %u Hz\r\n",
           (unsigned)rcu_clock_freq_get(CK_APB1),
           (unsigned)rcu_clock_freq_get(CK_APB2));
    printf("MCU UID32  : 0x%08X\r\n", (unsigned)bsp_mcu_uid32());
    printf("FW Version : 0x%08X\r\n", (unsigned)FW_VERSION);
    printf("RS485      : USART2 PB10/PB11, DE=PB2, %u 8N1\r\n", (unsigned)RS485_BAUDRATE);
    printf("Modbus addr: 0x%02X\r\n", (unsigned)MB_SLAVE_ADDR);
    printf("Frame dump : %s\r\n", MODBUS_DEBUG_DUMP ? "ON" : "OFF");
    printf("-- params (%s) --\r\n", param_loaded ? "from flash" : "DEFAULT, flash empty");
    printf("  L2 out1/out2 : %u / %u (0.01V)\r\n",
           (unsigned)p->out1_l2_cv, (unsigned)p->out2_l2_cv);
    printf("  L3 out1/out2 : %u / %u (0.01V)\r\n",
           (unsigned)p->out1_l3_cv, (unsigned)p->out2_l3_cv);
    printf("  release dur  : %u s\r\n", (unsigned)p->release_dur_s);
    printf("  seat present : %u (0.01V)\r\n", (unsigned)p->seat_present_cv);
    printf("  pedal switch : %u (%s)\r\n", (unsigned)p->limit_mode,
           (0U != p->limit_mode) ? "ON: RELAY1 only" : "OFF: RELAY0+RELAY1");
    printf("=====================================\r\n\r\n");
}

static void print_status(void)
{
    const rs485_stat_t  *rs = rs485_stat();
    const param_stat_t  *ps = param_stat();

    printf("[AD] pedal=%u/%u  out_fb=%u/%u  12V=%u  front=%u  back=%u  ref=%u  r100=%u (0.01V)\r\n",
           (unsigned)bsp_adc_get_cv(ADC_CH_PEDAL1),
           (unsigned)bsp_adc_get_cv(ADC_CH_PEDAL2),
           (unsigned)bsp_adc_get_cv(ADC_CH_OUT1_FB),
           (unsigned)bsp_adc_get_cv(ADC_CH_OUT2_FB),
           (unsigned)bsp_adc_get_cv(ADC_CH_SUPPLY12),
           (unsigned)bsp_adc_get_cv(ADC_CH_FRONT),
           (unsigned)bsp_adc_get_cv(ADC_CH_BACK),
           (unsigned)bsp_adc_get_cv(ADC_CH_VREF),
           (unsigned)bsp_adc_get_cv(ADC_CH_RSVD100));

    printf("[AP] limit=%u pedalsw=%u relay=%u/%u target=%u/%u veh=%u release=%u fault=0x%02X\r\n",
           (unsigned)app_limiter_get_limit_state(),
           (unsigned)app_limiter_get_pedal_switch(),
           (unsigned)app_limiter_relay_engaged(0U),
           (unsigned)app_limiter_relay_engaged(1U),
           (unsigned)app_limiter_target_cv(0U),
           (unsigned)app_limiter_target_cv(1U),
           (unsigned)app_limiter_get_vehicle_state(),
           (unsigned)app_limiter_get_release_state(),
           (unsigned)app_limiter_get_fault_code());

    printf("[DI] seat=%u belt=%u button=%u\r\n",
           (unsigned)app_limiter_get_seat_present(),
           (unsigned)app_limiter_get_belt_state(),
           (unsigned)bsp_button_get());

    printf("[485] rx=%u tx=%u drop=%u guard=%u ovr=%u ferr=%u nerr=%u | flash ok=%u fail=%u slot=%u\r\n",
           (unsigned)rs->rx_frames, (unsigned)rs->tx_frames,
           (unsigned)rs->rx_dropped, (unsigned)rs->rx_guarded,
           (unsigned)rs->err_overrun, (unsigned)rs->err_frame, (unsigned)rs->err_noise,
           (unsigned)ps->commit_ok, (unsigned)ps->commit_fail, (unsigned)ps->slot_used);
}

/* ==================== 主函数 ==================== */

int main(void)
{
    uint32_t wdi_tick;
    uint32_t adc_tick;
    uint32_t app_tick;
    uint32_t stat_tick;
    uint8_t  param_loaded;

    /* SystemInit() 已在启动文件里把时钟拉到 120MHz，这里按实际寄存器刷新一次 */
    SystemCoreClockUpdate();

    /* 板级初始化最先做：内部会立刻把 RS485 的 DE 拉低、继电器与输出全部置为安全态 */
    bsp_board_init();

    systick_config();

    bsp_debug_uart_init(BSP_DBG_BAUDRATE);

    /* 参数要在 app_limiter_init 之前加载，否则限速电压取不到 */
    param_loaded = param_load();

    print_banner(param_loaded);

    bsp_adc_init();
    bsp_dac_init();
    app_limiter_init();

    rs485_init(RS485_BAUDRATE);
    modbus_slave_init(MB_SLAVE_ADDR, &s_reg_ops);

    printf("System ready, Modbus RTU slave running.\r\n\r\n");

    wdi_tick  = systick_get_ms();
    adc_tick  = systick_get_ms();
    app_tick  = systick_get_ms();
    stat_tick = systick_get_ms();

    while(1) {
        /* ---- Modbus 从站处理 ---- */
        (void)modbus_slave_poll();

        /* ---- 收到设备复位命令：等回复发完再复位 ---- */
        if(s_reset_pending && (0U == rs485_tx_busy())) {
            printf("[SYS] reset by Modbus command\r\n");
            /* 复位前把待保存的参数落盘，别丢了上位机刚下发的设定 */
            if(s_param_pending) {
                (void)param_commit();
                s_param_pending = 0U;
            }
            delay_1ms(20U);
            NVIC_SystemReset();
        }

        /* ---- ADC 滤波 ---- */
        if(systick_elapsed(adc_tick, TASK_ADC_PERIOD_MS)) {
            adc_tick = systick_get_ms();
            bsp_adc_poll();
        }

        /* ---- 限速业务逻辑 ---- */
        if(systick_elapsed(app_tick, TASK_APP_PERIOD_MS)) {
            app_tick = systick_get_ms();
            app_limiter_poll();
        }

        /* ---- 参数延迟落盘 ---- */
        if(s_param_pending && systick_elapsed(s_param_tick, PARAM_COMMIT_DELAY_MS)) {
            s_param_pending = 0U;

            /* 必须先问 param_dirty()：param_commit() 对"内容与 Flash 一致"
               这种情况也返回成功（它内部直接跳过擦写以保护寿命）。
               只看返回值就打印 saved，会让上位机反复下发同一组值时
               每次都显示"已保存"，实际一个字节都没写。 */
            if(!param_dirty()) {
                printf("[PRM] unchanged, no write (slot=%u)\r\n",
                       (unsigned)param_stat()->slot_used);
            } else if(param_commit()) {
                printf("[PRM] saved to flash (slot=%u)\r\n",
                       (unsigned)param_stat()->slot_used);
            } else {
                printf("[PRM] flash save FAILED\r\n");
            }
        }

        /* ---- 喂外部看门狗 TPV6823S，不喂会被拉复位 ---- */
        if(systick_elapsed(wdi_tick, BSP_WDI_FEED_INTERVAL_MS)) {
            wdi_tick = systick_get_ms();
            bsp_wdi_feed();
        }

        /* ---- 周期状态打印 ---- */
        if(systick_elapsed(stat_tick, TASK_STAT_PERIOD_MS)) {
            stat_tick = systick_get_ms();
            print_status();
        }
    }
}
