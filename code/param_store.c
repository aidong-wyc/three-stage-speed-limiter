/*!
    \file    param_store.c
    \brief   参数掉电存储实现
*/

#include <stddef.h>

#include "param_store.h"
#include "modbus_crc.h"
#include "gd32f30x.h"

/* Flash 参数区：128KB 的最后一页 */
#define PARAM_PAGE_ADDR         0x0801F800UL
#define PARAM_PAGE_SIZE         0x800UL         /* GD32F30x 页大小 2KB，已核对固件库例程 */

/* 记录定长 32 字节，页内可放 64 条 */
#define PARAM_REC_SIZE          32UL
#define PARAM_REC_COUNT         (PARAM_PAGE_SIZE / PARAM_REC_SIZE)

#define PARAM_MAGIC             0x5A4C4D31UL    /* "ZLM1" 限速器参数标识 */
#define PARAM_VERSION           1U

/* CRC 覆盖记录的前 30 字节，crc 自己放在最后 2 字节，
   这样校验的是一段连续内存，比分段算再异或更可靠 */
#define PARAM_CRC_LEN           30U

/* 落盘记录布局：4 字节对齐、总长 32 字节 */
typedef struct {
    uint32_t magic;                                     /* +0            */
    uint16_t version;                                   /* +4            */
    param_t  data;                                      /* +6  (14 字节) */
    uint8_t  pad[PARAM_CRC_LEN - 6U - sizeof(param_t)]; /* +20           */
    uint16_t crc;                                       /* +30           */
} param_rec_t;

/* 编译期断言：记录必须正好 PARAM_REC_SIZE 字节。
   以后往 param_t 里加字段时，如果加到让整条记录不再是 32 字节，
   这里会直接编译报错，而不是让 Flash 里的记录悄悄错位、
   上电读出一堆乱参数。加字段的正确做法是同时缩小 pad，
   pad 不够时就把 PARAM_REC_SIZE 改成 64 并把 PARAM_CRC_LEN 改成 62。 */
_Static_assert(sizeof(param_rec_t) == PARAM_REC_SIZE,
               "param_rec_t must be exactly PARAM_REC_SIZE bytes");
_Static_assert(PARAM_REC_SIZE % 4U == 0U,
               "PARAM_REC_SIZE must be a multiple of 4 (flash is word-programmed)");

/* 默认值 */
static const param_t s_default = {
    0U,         /* out1_l2_cv      上电默认 0V，必须由上位机下发 */
    0U,         /* out2_l2_cv      */
    0U,         /* out1_l3_cv      */
    0U,         /* out2_l3_cv      */
    350U,       /* seat_present_cv 协议默认 3.5V */
    10U,        /* release_dur_s   协议默认 10s */
    0U          /* limit_mode      */
};

static param_t       s_ram;             /* 工作副本 */
static param_t       s_flash;           /* 最后一次落盘的内容 */
static uint8_t       s_valid_in_flash;  /* Flash 里是否已有本版本的有效记录 */
static uint16_t      s_next_slot;       /* 下一条记录写在第几个槽位 */
static param_stat_t  s_stat;

static uint16_t rec_crc(const param_rec_t *r)
{
    return modbus_crc16((const uint8_t *)r, PARAM_CRC_LEN);
}

static uint8_t rec_valid(const param_rec_t *r)
{
    if(PARAM_MAGIC != r->magic) {
        return 0U;
    }
    if(PARAM_VERSION != r->version) {
        return 0U;
    }

    return (rec_crc(r) == r->crc) ? 1U : 0U;
}

static uint8_t param_page_erase(void)
{
    fmc_state_enum st;

    fmc_unlock();
    st = fmc_page_erase(PARAM_PAGE_ADDR);
    fmc_lock();

    if(FMC_READY != st) {
        return 0U;
    }

    s_stat.page_erase++;
    s_next_slot = 0U;

    return 1U;
}

static uint8_t param_rec_write(uint16_t slot, const param_rec_t *r)
{
    uint32_t        addr = PARAM_PAGE_ADDR + ((uint32_t)slot * PARAM_REC_SIZE);
    const uint32_t *src  = (const uint32_t *)(const void *)r;
    uint32_t        i;
    fmc_state_enum  st;

    fmc_unlock();
    for(i = 0U; i < (PARAM_REC_SIZE / 4UL); i++) {
        st = fmc_word_program(addr + (i * 4UL), src[i]);
        if(FMC_READY != st) {
            fmc_lock();
            return 0U;
        }
    }
    fmc_lock();

    /* 回读比对，确认真的落盘了 */
    for(i = 0U; i < (PARAM_REC_SIZE / 4UL); i++) {
        if(*(volatile uint32_t *)(addr + (i * 4UL)) != src[i]) {
            return 0U;
        }
    }

    return 1U;
}

uint8_t param_load(void)
{
    uint16_t            i;
    const param_rec_t  *rec;
    const param_rec_t  *last = NULL;

    s_next_slot      = 0U;
    s_valid_in_flash = 0U;

    /* 从头扫到第一个空槽，记住途中最后一条有效记录 */
    for(i = 0U; i < (uint16_t)PARAM_REC_COUNT; i++) {
        rec = (const param_rec_t *)(PARAM_PAGE_ADDR + ((uint32_t)i * PARAM_REC_SIZE));

        if(0xFFFFFFFFUL == rec->magic) {
            break;      /* 空槽，后面都是空的 */
        }

        if(rec_valid(rec)) {
            last = rec;
        }

        s_next_slot = (uint16_t)(i + 1U);
    }

    s_stat.slot_used = s_next_slot;

    if(NULL != last) {
        s_ram            = last->data;
        s_flash          = last->data;
        s_valid_in_flash = 1U;
        s_stat.load_ok   = 1U;
        return 1U;
    }

    /* 没有有效记录（首次上电、或结构版本变更）：用默认值。
       s_valid_in_flash 保持 0，param_dirty() 会一直报"有待保存"，
       等上层第一次 commit 时才真正写 Flash。 */
    s_ram          = s_default;
    s_flash        = s_default;
    s_stat.load_ok = 0U;

    return 0U;
}

const param_t *param_get(void)
{
    return &s_ram;
}

void param_set(const param_t *p)
{
    if(NULL != p) {
        s_ram = *p;
    }
}

uint8_t param_dirty(void)
{
    const uint8_t *a = (const uint8_t *)&s_ram;
    const uint8_t *b = (const uint8_t *)&s_flash;
    uint32_t       i;

    /* Flash 里还没有本版本的有效记录，视为有待保存 */
    if(!s_valid_in_flash) {
        return 1U;
    }

    for(i = 0U; i < sizeof(param_t); i++) {
        if(a[i] != b[i]) {
            return 1U;
        }
    }

    return 0U;
}

uint8_t param_commit(void)
{
    param_rec_t rec;
    uint32_t    i;

    if(!param_dirty()) {
        return 1U;
    }

    /* 页写满了先擦 */
    if(s_next_slot >= (uint16_t)PARAM_REC_COUNT) {
        if(!param_page_erase()) {
            s_stat.commit_fail++;
            return 0U;
        }
    }

    rec.magic   = PARAM_MAGIC;
    rec.version = PARAM_VERSION;
    rec.data    = s_ram;
    for(i = 0U; i < sizeof(rec.pad); i++) {
        rec.pad[i] = 0xFFU;
    }
    rec.crc = rec_crc(&rec);

    if(!param_rec_write(s_next_slot, &rec)) {
        /* 该槽位写失败（可能有残留数据），擦页后重试一次 */
        if(!param_page_erase()) {
            s_stat.commit_fail++;
            return 0U;
        }
        if(!param_rec_write(s_next_slot, &rec)) {
            s_stat.commit_fail++;
            return 0U;
        }
    }

    s_next_slot++;
    s_stat.slot_used  = s_next_slot;
    s_flash           = s_ram;
    s_valid_in_flash  = 1U;
    s_stat.commit_ok++;

    return 1U;
}

uint8_t param_restore_default(void)
{
    s_ram = s_default;

    return param_commit();
}

const param_stat_t *param_stat(void)
{
    return &s_stat;
}
