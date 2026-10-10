/*!
    \file    bsp_debug_uart.c
    \brief   调试串口（USART0）与 printf 重定向
*/

#include <stdio.h>
#include "bsp_debug_uart.h"
#include "bsp_board.h"

void bsp_debug_uart_init(uint32_t baudrate)
{
    rcu_periph_clock_enable(BSP_DBG_GPIO_RCU);
    rcu_periph_clock_enable(BSP_DBG_USART_RCU);

    /* PA9 = USART0_TX 复用推挽（USART0 默认引脚） */
    gpio_init(BSP_DBG_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BSP_DBG_TX_PIN);

    /* PA10 = USART0_RX 用上拉输入。调试线没插时引脚是悬空的，
       浮空会拾取噪声产生假起始位；目前没用到接收所以无害，
       但上拉是更稳的默认，以后要用 USART0 收数据也不会踩坑。 */
    gpio_init(BSP_DBG_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, BSP_DBG_RX_PIN);

    usart_deinit(BSP_DBG_USART);
    usart_baudrate_set(BSP_DBG_USART, baudrate);
    usart_word_length_set(BSP_DBG_USART, USART_WL_8BIT);
    usart_stop_bit_set(BSP_DBG_USART, USART_STB_1BIT);
    usart_parity_config(BSP_DBG_USART, USART_PM_NONE);
    usart_hardware_flow_rts_config(BSP_DBG_USART, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(BSP_DBG_USART, USART_CTS_DISABLE);
    usart_receive_config(BSP_DBG_USART, USART_RECEIVE_ENABLE);
    usart_transmit_config(BSP_DBG_USART, USART_TRANSMIT_ENABLE);
    usart_enable(BSP_DBG_USART);
}

void bsp_debug_putc(uint8_t ch)
{
    usart_data_transmit(BSP_DBG_USART, (uint32_t)ch);

    while(RESET == usart_flag_get(BSP_DBG_USART, USART_FLAG_TBE)) {
    }
}

void bsp_debug_dump_hex(const char *tag, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    printf("%s[%u]:", tag, (unsigned)len);
    for(i = 0U; i < len; i++) {
        printf(" %02X", data[i]);
    }
    printf("\r\n");
}

/* ---------------- printf 重定向 ----------------
   Arm Compiler 6 用 MicroLIB 时重写 fputc 即可；
   不用 MicroLIB 时也能通过 fputc 走到这里（需要下面的 stdout 桩）。 */
#if defined(__ARMCC_VERSION)

#if !defined(__MICROLIB)
/* 非 MicroLIB：屏蔽半主机，避免链接时报 __use_no_semihosting 相关错误 */
__asm(".global __use_no_semihosting\n\t");

FILE __stdout;
FILE __stderr;

void _sys_exit(int x)
{
    (void)x;
    while(1) {
    }
}

char *_sys_command_string(char *cmd, int len)
{
    (void)len;
    return cmd;
}

void _ttywrch(int ch)
{
    bsp_debug_putc((uint8_t)ch);
}
#endif /* !__MICROLIB */

int fputc(int ch, FILE *f)
{
    (void)f;
    bsp_debug_putc((uint8_t)ch);
    return ch;
}

#endif /* __ARMCC_VERSION */
