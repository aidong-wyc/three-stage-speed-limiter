/*!
    \file    modbus_slave.c
    \brief   Modbus RTU 从站实现
*/

#include <stddef.h>

#include "modbus_slave.h"
#include "modbus_crc.h"
#include "rs485.h"

#if MODBUS_DEBUG_DUMP
#include "bsp_debug_uart.h"
#endif

/* 单次读写的寄存器个数上限（受 256 字节帧长限制） */
#define MB_MAX_READ_REGS        125U
#define MB_MAX_WRITE_REGS       123U

static uint8_t                  s_addr = MB_SLAVE_ADDR;
static const modbus_reg_ops_t  *s_ops  = NULL;
static modbus_stat_t            s_stat;

static uint8_t s_req[RS485_BUF_SIZE];
static uint8_t s_rsp[RS485_BUF_SIZE];

void modbus_slave_init(uint8_t addr, const modbus_reg_ops_t *ops)
{
    s_addr = addr;
    s_ops  = ops;
}

const modbus_stat_t *modbus_slave_stat(void)
{
    return &s_stat;
}

/* 在帧尾追加 CRC（低字节先）并发送 */
static void mb_reply(uint8_t *frame, uint16_t len)
{
    uint16_t crc = modbus_crc16(frame, len);

    frame[len]      = (uint8_t)(crc & 0xFFU);
    frame[len + 1U] = (uint8_t)((crc >> 8) & 0xFFU);

    /* 先启动发送再打印：rs485_send 是中断驱动的，
       报文已经开始往外走，printf 与之并行，不会延后响应 */
    (void)rs485_send(frame, len + 2U);

#if MODBUS_DEBUG_DUMP
    bsp_debug_dump_hex("  TX", frame, (uint16_t)(len + 2U));
#endif
}

/* 回异常帧 */
static void mb_reply_exception(uint8_t fc, uint8_t ex_code)
{
    s_rsp[0] = s_addr;
    s_rsp[1] = (uint8_t)(fc | 0x80U);
    s_rsp[2] = ex_code;

    s_stat.exception++;
    mb_reply(s_rsp, 3U);
}

/* 功能码 0x03：读多个寄存器 */
static void mb_handle_read(const uint8_t *req, uint16_t len, uint8_t broadcast)
{
    uint16_t start, count, i;
    uint8_t  byte_cnt;

    /* 固定长度：地址1 + 功能码1 + 起始地址2 + 数量2 + CRC2 = 8 */
    if(8U != len) {
        mb_reply_exception(MB_FC_READ_REGS, MB_EX_ILLEGAL_VALUE);
        return;
    }

    start = ((uint16_t)req[2] << 8) | req[3];
    count = ((uint16_t)req[4] << 8) | req[5];

    if((0U == count) || (count > MB_MAX_READ_REGS)) {
        mb_reply_exception(MB_FC_READ_REGS, MB_EX_ILLEGAL_VALUE);
        return;
    }

    if(((uint32_t)start + count) > MB_REG_MAX) {
        mb_reply_exception(MB_FC_READ_REGS, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    byte_cnt = (uint8_t)(count * 2U);

    s_rsp[0] = s_addr;
    s_rsp[1] = MB_FC_READ_REGS;
    s_rsp[2] = byte_cnt;

    for(i = 0U; i < count; i++) {
        uint16_t v  = 0U;
        uint8_t  rc = MB_EX_ILLEGAL_ADDRESS;

        if((NULL != s_ops) && (NULL != s_ops->reg_read)) {
            rc = s_ops->reg_read((uint16_t)(start + i), &v);
        }

        if(0U != rc) {
            mb_reply_exception(MB_FC_READ_REGS, rc);
            return;
        }

        /* 数据高位在前，低位在后 */
        s_rsp[3U + i * 2U]      = (uint8_t)((v >> 8) & 0xFFU);
        s_rsp[3U + i * 2U + 1U] = (uint8_t)(v & 0xFFU);
    }

    /* 广播帧不回复 */
    if(broadcast) {
        return;
    }

    s_stat.rsp_sent++;
    mb_reply(s_rsp, (uint16_t)(3U + byte_cnt));
}

/* 功能码 0x10：写多个寄存器 */
static void mb_handle_write(const uint8_t *req, uint16_t len, uint8_t broadcast)
{
    uint16_t start, count, i;
    uint8_t  byte_cnt;

    /* 最短：地址1 + 功能码1 + 起始2 + 数量2 + 字节数1 + 数据2 + CRC2 = 11 */
    if(len < 11U) {
        mb_reply_exception(MB_FC_WRITE_REGS, MB_EX_ILLEGAL_VALUE);
        return;
    }

    start    = ((uint16_t)req[2] << 8) | req[3];
    count    = ((uint16_t)req[4] << 8) | req[5];
    byte_cnt = req[6];

    if((0U == count) || (count > MB_MAX_WRITE_REGS) || (byte_cnt != (count * 2U))) {
        mb_reply_exception(MB_FC_WRITE_REGS, MB_EX_ILLEGAL_VALUE);
        return;
    }

    /* 帧总长必须等于 7 字节头 + 数据 + 2 字节 CRC */
    if(len != (uint16_t)(7U + byte_cnt + 2U)) {
        mb_reply_exception(MB_FC_WRITE_REGS, MB_EX_ILLEGAL_VALUE);
        return;
    }

    if(((uint32_t)start + count) > MB_REG_MAX) {
        mb_reply_exception(MB_FC_WRITE_REGS, MB_EX_ILLEGAL_ADDRESS);
        return;
    }

    for(i = 0U; i < count; i++) {
        uint16_t v  = ((uint16_t)req[7U + i * 2U] << 8) | req[7U + i * 2U + 1U];
        uint8_t  rc = MB_EX_ILLEGAL_ADDRESS;

        if((NULL != s_ops) && (NULL != s_ops->reg_write)) {
            rc = s_ops->reg_write((uint16_t)(start + i), v);
        }

        if(0U != rc) {
            mb_reply_exception(MB_FC_WRITE_REGS, rc);
            return;
        }
    }

    if(broadcast) {
        return;
    }

    /* 正常回复：回显起始地址与写入个数 */
    s_rsp[0] = s_addr;
    s_rsp[1] = MB_FC_WRITE_REGS;
    s_rsp[2] = (uint8_t)((start >> 8) & 0xFFU);
    s_rsp[3] = (uint8_t)(start & 0xFFU);
    s_rsp[4] = (uint8_t)((count >> 8) & 0xFFU);
    s_rsp[5] = (uint8_t)(count & 0xFFU);

    s_stat.rsp_sent++;
    mb_reply(s_rsp, 6U);
}

uint8_t modbus_slave_poll(void)
{
    uint16_t len;
    uint16_t crc_calc, crc_recv;
    uint8_t  broadcast;

    len = rs485_frame_get(s_req, sizeof(s_req));
    if(0U == len) {
        return 0U;
    }

    s_stat.req_total++;

#if MODBUS_DEBUG_DUMP
    bsp_debug_dump_hex("  RX", s_req, len);
#endif

    /* 最短合法帧：地址1 + 功能码1 + CRC2 */
    if(len < 4U) {
        return 1U;
    }

    /* 地址过滤：只处理本机地址和广播地址 0 */
    if((s_req[0] != s_addr) && (0x00U != s_req[0])) {
        return 1U;
    }

    broadcast = (0x00U == s_req[0]) ? 1U : 0U;
    s_stat.req_for_me++;

    /* CRC 校验，低字节在前 */
    crc_calc = modbus_crc16(s_req, (uint16_t)(len - 2U));
    crc_recv = (uint16_t)s_req[len - 2U] | ((uint16_t)s_req[len - 1U] << 8);

    if(crc_calc != crc_recv) {
        /* CRC 错按规范不回复，避免总线上互相干扰 */
        s_stat.crc_error++;
        return 1U;
    }

    switch(s_req[1]) {
    case MB_FC_READ_REGS:
        /* 读操作对广播无意义，直接忽略 */
        if(!broadcast) {
            mb_handle_read(s_req, len, broadcast);
        }
        break;

    case MB_FC_WRITE_REGS:
        mb_handle_write(s_req, len, broadcast);
        break;

    default:
        if(!broadcast) {
            mb_reply_exception(s_req[1], MB_EX_ILLEGAL_FUNCTION);
        }
        break;
    }

    return 1U;
}
