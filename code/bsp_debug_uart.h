/*!
    \file    bsp_debug_uart.h
    \brief   调试串口（USART0 / PA9-PA10 / J1 Debug Port），printf 重定向

    \note    和 RS485 完全分开，调试打印不会污染 485 总线。
             默认 115200 8N1。
*/

#ifndef BSP_DEBUG_UART_H
#define BSP_DEBUG_UART_H

#include "gd32f30x.h"

/* 初始化调试串口 */
void bsp_debug_uart_init(uint32_t baudrate);

/* 阻塞发送一个字节，printf 最终走这里 */
void bsp_debug_putc(uint8_t ch);

/* 以十六进制打印一段数据，用于观察 RS485 报文 */
void bsp_debug_dump_hex(const char *tag, const uint8_t *data, uint16_t len);

#endif /* BSP_DEBUG_UART_H */
