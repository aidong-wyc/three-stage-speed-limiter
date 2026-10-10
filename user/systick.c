/*!
    \file    systick.c
    \brief   SysTick 1ms 时基
*/

#include "systick.h"

static volatile uint32_t s_delay_cnt = 0U;
static volatile uint32_t s_tick_ms   = 0U;

/*!
    \brief      配置 SysTick 为 1000Hz（1ms）中断
*/
void systick_config(void)
{
    /* SystemCoreClock 由 SystemInit 中的时钟配置决定，本板为 120MHz */
    if(SysTick_Config(SystemCoreClock / 1000U)) {
        /* 重载值超范围，配置失败 */
        while(1) {
        }
    }
    /* SysTick 优先级设为最低一档，避免抢占 RS485 收发中断的时序 */
    NVIC_SetPriority(SysTick_IRQn, 0x0FU);
}

/*!
    \brief      毫秒级阻塞延时
    \param[in]  count: 延时毫秒数
*/
void delay_1ms(uint32_t count)
{
    s_delay_cnt = count;

    while(0U != s_delay_cnt) {
    }
}

/*!
    \brief      SysTick 中断中调用，递减延时计数并累加系统时基
*/
void delay_decrement(void)
{
    s_tick_ms++;

    if(0U != s_delay_cnt) {
        s_delay_cnt--;
    }
}

/*!
    \brief      取上电以来的毫秒计数
*/
uint32_t systick_get_ms(void)
{
    return s_tick_ms;
}

/*!
    \brief      判断自 start 起是否已过去 ms 毫秒
    \note       用无符号减法比较，32bit 回绕时结果依然正确
*/
uint8_t systick_elapsed(uint32_t start, uint32_t ms)
{
    return ((uint32_t)(s_tick_ms - start) >= ms) ? 1U : 0U;
}
