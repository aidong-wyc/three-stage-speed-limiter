/*!
    \file    param_store.h
    \brief   参数掉电存储（片内 Flash 最后一页）

    \note    GD32F303RB：128KB Flash，页大小 2KB，最后一页起始 0x0801F800。
             整页专用于参数，不放代码（工程 IROM 只用到约 14KB，离得很远）。

             为什么用"追加记录"而不是"每次擦一次写一次"：
             1) 掉电安全：新记录写坏了 CRC 校验不过，自动回退用上一条有效记录，
                不会因为一次掉电把参数全丢；
             2) 寿命：一页能放 64 条记录，写满才擦一次，
                擦除次数是直接改写方式的 1/64。

             记录 32 字节定长，页内顺序追加，上电时扫描最后一条有效记录。
*/

#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdint.h>

/*!
    \brief  需要掉电保存的参数
    \note   本结构体会被整体写进 Flash，改动字段请同时把 PARAM_VERSION 加一，
            旧版本记录会被识别为无效并回落到默认值。
*/
typedef struct {
    uint16_t out1_l2_cv;        /* 通道1 二级限速电压，0.01V（协议 0x0B） */
    uint16_t out2_l2_cv;        /* 通道2 二级限速电压，0.01V（协议 0x0C） */
    uint16_t out1_l3_cv;        /* 通道1 三级限速电压，0.01V（协议 0x0D） */
    uint16_t out2_l3_cv;        /* 通道2 三级限速电压，0.01V（协议 0x0E） */
    uint16_t seat_present_cv;   /* 座椅在位电压，0.01V（协议 0x08），默认 350 */
    uint16_t release_dur_s;     /* 解除限速持续时间，秒（协议 0x11），默认 10 */
    uint16_t limit_mode;        /* 限速方式（协议 0x05） */
} param_t;

/*!
    \brief      上电加载参数
    \note       找不到有效记录时（首次上电或版本变更）自动填默认值，
                此时不会立即写 Flash，等第一次真正修改参数时才写。
    \retval     1 从 Flash 成功加载，0 用了默认值
*/
uint8_t param_load(void);

/*!
    \brief      取参数（只读指针，始终有效）
*/
const param_t *param_get(void);

/*!
    \brief      修改单个参数（只改内存，不写 Flash）
    \note       改完请调用 param_commit()。分开是为了让上位机一次写多个
                寄存器时只落盘一次，减少 Flash 擦写。
*/
void param_set(const param_t *p);

/*!
    \brief      把内存里的参数落盘
    \retval     1 成功，0 失败
    \note       内部会先比较，内容没变就直接返回成功、不动 Flash。
*/
uint8_t param_commit(void);

/*!
    \brief      内存中的参数与 Flash 里是否已不一致（有待保存的修改）
*/
uint8_t param_dirty(void);

/*!
    \brief      恢复出厂默认值并落盘
*/
uint8_t param_restore_default(void);

/*!
    \brief      统计信息，便于排查 Flash 问题
*/
typedef struct {
    uint32_t load_ok;           /* 上电加载成功次数（0 或 1） */
    uint32_t commit_ok;         /* 落盘成功次数 */
    uint32_t commit_fail;       /* 落盘失败次数 */
    uint32_t page_erase;        /* 整页擦除次数 */
    uint16_t slot_used;         /* 当前页内已用记录数 */
} param_stat_t;

const param_stat_t *param_stat(void);

#endif /* PARAM_STORE_H */
