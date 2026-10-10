/*!
    \file    gd32f30x_it.h
    \brief   中断服务程序声明（GD32F303RBT6 / HD）
*/

#ifndef GD32F30X_IT_H
#define GD32F30X_IT_H

#include "gd32f30x.h"

/* Cortex-M4 内核异常 */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* 外设中断 */
void USART2_IRQHandler(void);        /* RS485 收发 */
void TIMER5_IRQHandler(void);        /* RS485 Modbus t3.5 帧定界 */

#endif /* GD32F30X_IT_H */
