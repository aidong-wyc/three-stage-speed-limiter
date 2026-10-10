/*!
    \file    system_gd32f30x.c
    \brief   本板专用 CMSIS 系统初始化（GD32F303RBT6，外部晶振 24MHz）

    \note    为什么不用固件库自带的 system_gd32f30x.c：
             库里 HD 系列的 120MHz 配置写死为 (HXTAL/2) * 30，只适用于 8MHz 晶振。
             本板 Y1 = 24MHz，若照抄会得到 (24/2)*30 = 360MHz，远超器件上限。
             这里改为 (HXTAL/2) * 10 = (24/2)*10 = 120MHz。

             时钟树：
               HXTAL   = 24 MHz
               PREDV0  = /2      -> PLL 输入 12 MHz
               PLLMF   = x10     -> CK_PLL  = 120 MHz
               CK_SYS  = CK_PLL  = 120 MHz
               AHB     = /1      = 120 MHz
               APB2    = /1      = 120 MHz  (上限 120MHz，USART0 在此总线)
               APB1    = /2      =  60 MHz  (上限  60MHz，USART2/TIMER5 在此总线)

             注意：ADC 时钟来自 APB2，GD32F303 的 ADC 上限为 14MHz。
             APB2 = 120MHz 时，ADC 预分频必须取 /12(10MHz) 或 /16(7.5MHz)。
*/

#include "gd32f30x.h"

/* CK_SYS 选择状态 */
#define SEL_IRC8M       0x00U
#define SEL_HXTAL       0x01U
#define SEL_PLL         0x02U

/* 目标系统频率。SystemInit 在 RAM 初始化之前运行，
   因此该变量必须是编译期常量，由 C 库启动代码从 .data 装载。 */
uint32_t SystemCoreClock = 120000000U;

static void system_clock_config(void);
static void soft_delay(uint32_t time);

/* 降频时逐级切换 AHB 分频，避免 Vcore 跳变。等价于官方库的 RCU_MODIFY_4 宏。 */
static void rcu_ahb_stepdown(uint32_t delay)
{
    volatile uint32_t i;
    uint32_t reg;

    if(0U == delay) {
        return;
    }

    for(i = 0U; i < delay; i++) {
    }
    reg = RCU_CFG0;
    reg &= ~RCU_CFG0_AHBPSC;
    reg |= RCU_AHB_CKSYS_DIV2;
    RCU_CFG0 = reg;

    for(i = 0U; i < delay; i++) {
    }
    reg = RCU_CFG0;
    reg &= ~RCU_CFG0_AHBPSC;
    reg |= RCU_AHB_CKSYS_DIV4;
    RCU_CFG0 = reg;

    for(i = 0U; i < delay; i++) {
    }
    reg = RCU_CFG0;
    reg &= ~RCU_CFG0_AHBPSC;
    reg |= RCU_AHB_CKSYS_DIV8;
    RCU_CFG0 = reg;

    for(i = 0U; i < delay; i++) {
    }
    reg = RCU_CFG0;
    reg &= ~RCU_CFG0_AHBPSC;
    reg |= RCU_AHB_CKSYS_DIV16;
    RCU_CFG0 = reg;

    for(i = 0U; i < delay; i++) {
    }
}

static void soft_delay(uint32_t time)
{
    __IO uint32_t i;
    for(i = 0U; i < time * 10U; i++) {
    }
}

/*!
    \brief      系统初始化，由 startup_gd32f30x_hd.s 的 Reset_Handler 在进入 main 之前调用
*/
void SystemInit(void)
{
    /* 使能 FPU（Cortex-M4F） */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#endif

    /* 把 RCU 恢复到复位默认状态 */
    RCU_CTL |= RCU_CTL_IRC8MEN;
    while(0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
    }

    if((RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_PLL) {
        rcu_ahb_stepdown(0x50U);
    }

    RCU_CFG0 &= ~RCU_CFG0_SCS;
    soft_delay(200U);

    /* 关 PLL / 时钟监视 / HXTAL，并屏蔽全部 RCU 中断 */
    RCU_CTL &= ~(RCU_CTL_PLLEN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN);
    RCU_INT = 0x009F0000U;

    RCU_CTL &= ~RCU_CTL_HXTALBPS;

    RCU_CFG0 = 0x00000000U;
    RCU_CFG1 = 0x00000000U;

    system_clock_config();
}

/*!
    \brief      24MHz 外部晶振 PLL 到 120MHz
*/
static void system_clock_config(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;
    __IO uint32_t reg_temp;

    /* 起振 HXTAL(24MHz) */
    RCU_CTL |= RCU_CTL_HXTALEN;

    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    } while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
        /* 晶振没起来：停在这里，便于用调试器定位硬件问题 */
        while(1) {
        }
    }

    /* 120MHz 需要 LDO 输出最高档 */
    RCU_APB1EN |= RCU_APB1EN_PMUEN;
    PMU_CTL |= PMU_CTL_LDOVS;

    /* 提频之前先把 Flash 等待周期加到 2（>60MHz 必须） */
    FMC_WS = (FMC_WS & ~FMC_WS_WSCNT) | WS_WSCNT_2;

    /* 总线分频：AHB = CK_SYS，APB2 = AHB，APB1 = AHB/2 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* PLL 源选 HXTAL，PREDV0 = /2  ->  24MHz / 2 = 12MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
    RCU_CFG0 |= (RCU_PLLSRC_HXTAL_IRC48M | RCU_CFG0_PREDV0);

    /* CK_PLL = 12MHz * 10 = 120MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
    RCU_CFG0 |= RCU_PLL_MUL10;

    /* 使能 PLL 并等待锁定 */
    RCU_CTL |= RCU_CTL_PLLEN;
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* 打开 high-drive，才允许跑到 120MHz */
    PMU_CTL |= PMU_CTL_HDEN;
    while(0U == (PMU_CS & PMU_CS_HDRF)) {
    }
    PMU_CTL |= PMU_CTL_HDS;
    while(0U == (PMU_CS & PMU_CS_HDSRF)) {
    }

    /* 切 CK_SYS 到 PLL */
    reg_temp = RCU_CFG0;
    reg_temp &= ~RCU_CFG0_SCS;
    reg_temp |= RCU_CKSYSSRC_PLL;
    RCU_CFG0 = reg_temp;

    while(0U == (RCU_CFG0 & RCU_SCSS_PLL)) {
    }
}

/*!
    \brief      按当前寄存器实际值刷新 SystemCoreClock
*/
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = rcu_clock_freq_get(CK_SYS);
}
