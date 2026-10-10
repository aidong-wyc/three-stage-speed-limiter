/*!
    \file    rs485.h
    \brief   RS485 半双工收发驱动（USART2 + PB2 方向控制，Modbus RTU 帧定界）

    \note    硬件：U5 = SN65HVD72DGK / SIT3485E
             /RE 与 DE 并接到 PB2，所以方向只需一根线：
               PB2 = 1 -> 发送
               PB2 = 0 -> 接收
             协议：Modbus RTU，9600 8N1，从站地址 0x05（见《电子限速器通信协议》）

             帧定界按 Modbus RTU 规范用 t3.5 静默间隔判定，
             由 TIMER5 做一次性定时器实现；不用 USART 的 IDLE 标志，
             因为 IDLE 只有 1 个字符时间，主机字节间稍有间隙就会把一帧拆开。
*/

#ifndef RS485_H
#define RS485_H

#include "gd32f30x.h"

/* Modbus RTU 单帧最大长度（地址1 + 功能码1 + 数据252 + CRC2） */
#define RS485_BUF_SIZE          256U

/* 收发统计，便于现场排查线路问题 */
typedef struct {
    uint32_t rx_frames;         /* 收到的完整帧数（未做地址/CRC 过滤） */
    uint32_t tx_frames;         /* 发出的帧数 */
    uint32_t rx_dropped;        /* 因超长被丢弃的帧数 */
    uint32_t rx_guarded;        /* 发送后静默保护期内丢掉的字节数 */
    uint32_t err_overrun;       /* 溢出错误次数 */
    uint32_t err_frame;         /* 帧错误次数（波特率不匹配时会涨） */
    uint32_t err_noise;         /* 噪声错误次数 */
} rs485_stat_t;

/*!
    \brief      初始化 RS485
    \param[in]  baudrate: 波特率，协议规定 9600
*/
void rs485_init(uint32_t baudrate);

/*!
    \brief      取一帧已接收数据
    \param[out] buf: 输出缓冲
    \param[in]  bufsize: 缓冲大小
    \retval     帧长度，0 表示当前没有完整帧
    \note       非阻塞。取走后内部帧标志自动清除，接收器立即可以收下一帧。
*/
uint16_t rs485_frame_get(uint8_t *buf, uint16_t bufsize);

/*!
    \brief      发送一帧（中断驱动，非阻塞）
    \param[in]  data: 数据指针
    \param[in]  len:  长度，最大 RS485_BUF_SIZE
    \retval     1 已受理，0 被拒绝（正在发送或参数非法）
    \note       内部自动拉高 DE，最后一个字节移位完成（TC）后再拉低，
                不会截断尾字节。
*/
uint8_t rs485_send(const uint8_t *data, uint16_t len);

/*!
    \brief      是否正在发送
*/
uint8_t rs485_tx_busy(void);

/*!
    \brief      取统计计数（只读指针）
*/
const rs485_stat_t *rs485_stat(void);

/* ---- 中断入口，由 gd32f30x_it.c 转调 ---- */
void rs485_usart_irq_handler(void);
void rs485_t35_irq_handler(void);

#endif /* RS485_H */
