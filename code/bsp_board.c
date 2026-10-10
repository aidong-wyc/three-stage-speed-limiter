/*!
    \file    bsp_board.c
    \brief   三级限速器主控板 板级初始化
*/

#include "bsp_board.h"

/* GD32F303 96bit 唯一 ID 基地址 */
#define MCU_UID_BASE            0x1FFFF7E8U

/*!
    \brief      板级基础初始化
    \note       顺序很重要：先把 RS485 的 DE 拉低，再做其它事情。
                复位后 PB2 是浮空输入，SIT3485E 的 DE 没有下拉电阻，
                收发器可能被误使能而占用总线，所以要尽早驱动为低（接收态）。
*/
void bsp_board_init(void)
{
    /* 先定好中断优先级分组：2 位抢占 + 2 位子优先级。
       复位默认是 PRE0_SUB4（0 位抢占），那种情况下所有中断都无法互相抢占，
       后面给 USART2 设的抢占优先级会被静默忽略。 */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

    /* 打开需要用到的 GPIO 端口与 AFIO 时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);

    /* --- 第一优先：RS485 方向脚立即置为接收，避免上电瞬间占用 485 总线 --- */
    gpio_bit_reset(BSP_RS485_DE_PORT, BSP_RS485_DE_PIN);
    gpio_init(BSP_RS485_DE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BSP_RS485_DE_PIN);
    gpio_bit_reset(BSP_RS485_DE_PORT, BSP_RS485_DE_PIN);

    /* 关闭 JTAG 只保留 SWD，释放 PB3(JTDO) / PB4(NJTRST) / PA15(JTDI)。
       本板 BUTTON=PB3、GPIO_IN1=PB4，不做这一步这两个输入读不到正确电平。
       SWDIO(PA13)/SWCLK(PA14) 不受影响，调试口仍可用。 */
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

    /* --- 外部看门狗喂狗脚 --- */
    gpio_init(BSP_WDI_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, BSP_WDI_PIN);
    gpio_bit_reset(BSP_WDI_PORT, BSP_WDI_PIN);

    /* --- 继电器：上电默认释放 --- */
    gpio_bit_reset(BSP_RELAY_PORT, BSP_RELAY0_PIN | BSP_RELAY1_PIN);
    gpio_init(BSP_RELAY_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ,
              BSP_RELAY0_PIN | BSP_RELAY1_PIN);
    gpio_bit_reset(BSP_RELAY_PORT, BSP_RELAY0_PIN | BSP_RELAY1_PIN);

    /* --- 开关量输出 / 报警输出：上电默认关断 --- */
    gpio_bit_reset(BSP_GPO_PORT, BSP_GPO0_PIN | BSP_GPO1_PIN | BSP_ALARM_PIN);
    gpio_init(BSP_GPO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ,
              BSP_GPO0_PIN | BSP_GPO1_PIN | BSP_ALARM_PIN);
    gpio_bit_reset(BSP_GPO_PORT, BSP_GPO0_PIN | BSP_GPO1_PIN | BSP_ALARM_PIN);

    /* --- 开关量输入：外部缓冲器已有确定电平输出，用浮空输入即可 --- */
    gpio_init(BSP_GPI_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_10MHZ,
              BSP_GPI0_PIN | BSP_GPI1_PIN | BSP_BUTTON_PIN);
}

/*!
    \brief      喂外部看门狗，翻转 WDI_IN
*/
void bsp_wdi_feed(void)
{
    if(RESET == gpio_output_bit_get(BSP_WDI_PORT, BSP_WDI_PIN)) {
        gpio_bit_set(BSP_WDI_PORT, BSP_WDI_PIN);
    } else {
        gpio_bit_reset(BSP_WDI_PORT, BSP_WDI_PIN);
    }
}

void bsp_relay0_set(uint8_t on)
{
    if(on) {
        gpio_bit_set(BSP_RELAY_PORT, BSP_RELAY0_PIN);
    } else {
        gpio_bit_reset(BSP_RELAY_PORT, BSP_RELAY0_PIN);
    }
}

void bsp_relay1_set(uint8_t on)
{
    if(on) {
        gpio_bit_set(BSP_RELAY_PORT, BSP_RELAY1_PIN);
    } else {
        gpio_bit_reset(BSP_RELAY_PORT, BSP_RELAY1_PIN);
    }
}

void bsp_gpo0_set(uint8_t on)
{
    if(on) {
        gpio_bit_set(BSP_GPO_PORT, BSP_GPO0_PIN);
    } else {
        gpio_bit_reset(BSP_GPO_PORT, BSP_GPO0_PIN);
    }
}

void bsp_gpo1_set(uint8_t on)
{
    if(on) {
        gpio_bit_set(BSP_GPO_PORT, BSP_GPO1_PIN);
    } else {
        gpio_bit_reset(BSP_GPO_PORT, BSP_GPO1_PIN);
    }
}

void bsp_alarm_set(uint8_t on)
{
    if(on) {
        gpio_bit_set(BSP_GPO_PORT, BSP_ALARM_PIN);
    } else {
        gpio_bit_reset(BSP_GPO_PORT, BSP_ALARM_PIN);
    }
}

uint8_t bsp_gpi0_get(void)
{
    return (RESET != gpio_input_bit_get(BSP_GPI_PORT, BSP_GPI0_PIN)) ? 1U : 0U;
}

uint8_t bsp_gpi1_get(void)
{
    return (RESET != gpio_input_bit_get(BSP_GPI_PORT, BSP_GPI1_PIN)) ? 1U : 0U;
}

uint8_t bsp_button_get(void)
{
    return (RESET != gpio_input_bit_get(BSP_GPI_PORT, BSP_BUTTON_PIN)) ? 1U : 0U;
}

uint8_t bsp_seat_raw(void)
{
    return (RESET != gpio_input_bit_get(BSP_GPI_PORT, BSP_SEAT_PIN)) ? 1U : 0U;
}

uint8_t bsp_belt_raw(void)
{
    return (RESET != gpio_input_bit_get(BSP_GPI_PORT, BSP_BELT_PIN)) ? 1U : 0U;
}

/*!
    \brief      读 MCU 唯一 ID 低 32bit
*/
uint32_t bsp_mcu_uid32(void)
{
    return *(volatile uint32_t *)MCU_UID_BASE;
}
