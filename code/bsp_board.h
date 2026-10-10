/*!
    \file    bsp_board.h
    \brief   三级限速器主控板 板级引脚定义（MCU_Control_UnIT Rev1.0）

    \note    MCU: GD32F303RBT6 (LQFP64, 128KB Flash / 32KB SRAM)
             原理图封装标注为 STM32F103R8T6，与 GD32F303RBT6 引脚兼容。
             外部晶振 Y1 = 24MHz。
*/

#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include "gd32f30x.h"

/* ---- 编译期护栏：这两个宏必须由工程 Define 提供，缺一个就直接编译报错 ----
   少了 GD32F30X_HD：器件寄存器映射和中断向量会按错误的子系列展开。
   少了 HXTAL_VALUE：gd32f30x.h 默认按 8MHz 算，串口波特率、定时器时基全部会错，
   而且这种错编译期不报、上板才发现，所以在这里拦住。 */
#if !defined(GD32F30X_HD)
#error "GD32F303RBT6 属 HD 系列，请在工程 Define 中加 GD32F30X_HD"
#endif

#if !defined(HXTAL_VALUE)
#error "请在工程 Define 中加 HXTAL_VALUE=24000000（本板 Y1 为 24MHz 晶振）"
#elif (HXTAL_VALUE != 24000000)
#error "HXTAL_VALUE 与本板 24MHz 晶振不符，请检查工程 Define"
#endif

/* ======================== 时钟 ======================== */
#define BSP_HXTAL_HZ                24000000U
#define BSP_SYSCLK_HZ               120000000U

/* ======================== RS485 (原理图网络名 UART1_xxx) ======================== */
/* U5 = SN65HVD72DGK / SIT3485E
   pin1 RO -> PB11 (USART2_RX)
   pin2 /RE 与 pin3 DE 并接 -> PB2
   pin4 DI <- PB10 (USART2_TX)
   因 /RE 与 DE 并接：PB2 = 1 -> 发送（收发器驱动使能、接收禁止）
                      PB2 = 0 -> 接收
   PB10/PB11 是 USART2 的默认引脚，无需 AFIO 重映射。 */
#define BSP_RS485_USART             USART2
#define BSP_RS485_USART_RCU         RCU_USART2
#define BSP_RS485_IRQN              USART2_IRQn

#define BSP_RS485_GPIO_RCU          RCU_GPIOB
#define BSP_RS485_PORT              GPIOB
#define BSP_RS485_TX_PIN            GPIO_PIN_10
#define BSP_RS485_RX_PIN            GPIO_PIN_11

#define BSP_RS485_DE_PORT           GPIOB
#define BSP_RS485_DE_PIN            GPIO_PIN_2

/* ======================== 调试串口 (原理图网络名 UART0_xxx，J1 Debug Port) ======================== */
#define BSP_DBG_USART               USART0
#define BSP_DBG_USART_RCU           RCU_USART0
#define BSP_DBG_GPIO_RCU            RCU_GPIOA
#define BSP_DBG_PORT                GPIOA
#define BSP_DBG_TX_PIN              GPIO_PIN_9
#define BSP_DBG_RX_PIN              GPIO_PIN_10
#define BSP_DBG_BAUDRATE            115200U

/* ======================== 外部看门狗 TPV6823S (U3) ======================== */
/* WDI_IN -> PA0。必须周期性翻转，否则 U3 拉低 RESET# 复位 MCU。
   TPS382x 系列看门狗超时典型 1.6s，这里按 200ms 喂一次留足余量。 */
#define BSP_WDI_PORT                GPIOA
#define BSP_WDI_PIN                 GPIO_PIN_0
#define BSP_WDI_FEED_INTERVAL_MS    200U

/* ======================== 继电器 (LS1/LS2, TQ2SA-12V-Z, BC817W 低边驱动) ======================== */
/* 高电平 -> 三极管导通 -> 继电器吸合 */
#define BSP_RELAY_PORT              GPIOC
#define BSP_RELAY0_PIN              GPIO_PIN_11
#define BSP_RELAY1_PIN              GPIO_PIN_12

/* ======================== 开关量输出 (BC817W / FDN304PW 低边驱动，高有效) ======================== */
#define BSP_GPO_PORT                GPIOB
#define BSP_GPO0_PIN                GPIO_PIN_13     /* GPIO_OUT0 -> GPO0  */
#define BSP_GPO1_PIN                GPIO_PIN_14     /* GPIO_OUT1 -> GPO1  */
#define BSP_ALARM_PIN               GPIO_PIN_15     /* ALARM_OUT -> ALARM 12V 输出 */

/* ======================== 开关量输入 (经 SN74LV1T34 同相缓冲，高有效) ======================== */
/* 注意 PB3=JTDO、PB4=NJTRST，默认被 JTAG 占用，
   bsp_board_init() 里会关掉 JTAG 只留 SWD 才能当普通 GPIO 用。

   外部进来的是 12V 电平，经 36.5K/10K 分压后送 SN74LV1T34 同相缓冲器，
   缓冲器输出 3.3V 逻辑给 MCU，所以这里读到高电平就代表外部有 12V。 */
#define BSP_GPI_PORT                GPIOB
#define BSP_GPI0_PIN                GPIO_PIN_5      /* GPIO_IN0 <- GPI0    */
#define BSP_GPI1_PIN                GPIO_PIN_4      /* GPIO_IN1 <- GPI1    */
#define BSP_BUTTON_PIN              GPIO_PIN_3      /* BUTTON   <- EXT_BT- */

/* 语义别名：座椅与安全带检测走这两路数字输入，不走 ADC。
   ！！GPIO_IN0 对应座椅、GPIO_IN1 对应安全带，这个对应关系是按
     需求描述"座椅和安全带 ... GPIO_in0 和 GPIO_in1"的顺序定的，
     实车上如果发现反了，把下面两行对调即可。 */
#define BSP_SEAT_PIN                BSP_GPI0_PIN    /* 座椅在位检测   */
#define BSP_BELT_PIN                BSP_GPI1_PIN    /* 安全带扣合检测 */

/* ======================== DAC 输出 ======================== */
/* DAC_OUT0 = PA4 -> U2(LM321X) -> ANALOG_O0
   DAC_OUT1 = PA5 -> U4(LM321X) -> ANALOG_O1 */
#define BSP_DAC_PORT                GPIOA
#define BSP_DAC0_PIN                GPIO_PIN_4
#define BSP_DAC1_PIN                GPIO_PIN_5

/* ======================== 板级 API ======================== */

/* 初始化时钟外的板级基础资源：GPIO 时钟、AFIO、方向与安全初值。
   必须在任何外设初始化之前调用。 */
void bsp_board_init(void);

/* 喂外部看门狗（翻转 WDI_IN）。需在主循环里按 BSP_WDI_FEED_INTERVAL_MS 周期调用。 */
void bsp_wdi_feed(void);

/* 继电器 / 开关量输出控制，on 非 0 表示动作 */
void bsp_relay0_set(uint8_t on);
void bsp_relay1_set(uint8_t on);
void bsp_gpo0_set(uint8_t on);
void bsp_gpo1_set(uint8_t on);
void bsp_alarm_set(uint8_t on);

/* 开关量输入读取，返回 1 表示高电平（即外部有 12V） */
uint8_t bsp_gpi0_get(void);
uint8_t bsp_gpi1_get(void);
uint8_t bsp_button_get(void);

/* 座椅 / 安全带原始电平，未消抖。消抖后的结果请用 app_limiter 的接口 */
uint8_t bsp_seat_raw(void);
uint8_t bsp_belt_raw(void);

/* 读 MCU 96bit 唯一 ID 的低 32bit，用于协议里的“传感器ID”寄存器 */
uint32_t bsp_mcu_uid32(void);

#endif /* BSP_BOARD_H */
