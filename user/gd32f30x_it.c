/*!
    \file    gd32f30x_it.c
    \brief   中断服务程序（GD32F303RBT6 / HD）
*/

#include "gd32f30x_it.h"
#include "systick.h"
#include "rs485.h"

/*!
    \brief      NMI 异常
*/
void NMI_Handler(void)
{
}

/*!
    \brief      硬件错误
    \note       调试阶段停在这里，方便用 SWD 回溯栈找出错点。
                量产固件应改为记录故障码后由看门狗复位。
*/
void HardFault_Handler(void)
{
    while(1) {
    }
}

void MemManage_Handler(void)
{
    while(1) {
    }
}

void BusFault_Handler(void)
{
    while(1) {
    }
}

void UsageFault_Handler(void)
{
    while(1) {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/*!
    \brief      SysTick 1ms
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

/*!
    \brief      USART2 中断：RS485 收发
*/
void USART2_IRQHandler(void)
{
    rs485_usart_irq_handler();
}

/*!
    \brief      TIMER5 中断：Modbus RTU t3.5 帧定界超时
*/
void TIMER5_IRQHandler(void)
{
    rs485_t35_irq_handler();
}
