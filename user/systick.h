/*!
    \file    systick.h
    \brief   SysTick 1ms 时基
*/

#ifndef SYSTICK_H
#define SYSTICK_H

#include "gd32f30x.h"

/* 配置 SysTick 为 1ms 中断 */
void systick_config(void);
/* 毫秒级阻塞延时 */
void delay_1ms(uint32_t count);
/* 供 SysTick_Handler 调用，递减阻塞延时计数 */
void delay_decrement(void);
/* 取上电以来的毫秒计数（32bit 溢出约 49.7 天） */
uint32_t systick_get_ms(void);
/* 判断自 start 起是否已过去 ms 毫秒（可安全处理溢出） */
uint8_t systick_elapsed(uint32_t start, uint32_t ms);

#endif /* SYSTICK_H */
