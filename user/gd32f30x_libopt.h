/*!
    \file    gd32f30x_libopt.h
    \brief   固件库模块裁剪（GD32F303RBT6 / HD 系列 / 三级限速器主控板）

    \note    仅包含本板实际用到的外设，减少编译体积。
             ENET / EXMC / SDIO / CTC 未使用，故不引入。
*/

#ifndef GD32F30X_LIBOPT_H
#define GD32F30X_LIBOPT_H

#include "gd32f30x_rcu.h"
#include "gd32f30x_misc.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_usart.h"
#include "gd32f30x_timer.h"
#include "gd32f30x_adc.h"
#include "gd32f30x_dac.h"
#include "gd32f30x_dma.h"
#include "gd32f30x_exti.h"
#include "gd32f30x_fmc.h"
#include "gd32f30x_fwdgt.h"
#include "gd32f30x_wwdgt.h"
#include "gd32f30x_pmu.h"
#include "gd32f30x_bkp.h"
#include "gd32f30x_rtc.h"
#include "gd32f30x_crc.h"
#include "gd32f30x_dbg.h"

#endif /* GD32F30X_LIBOPT_H */
