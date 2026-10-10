/*!
    \file    modbus_crc.h
    \brief   Modbus RTU CRC-16 校验
*/

#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stdint.h>

/*!
    \brief      计算 Modbus RTU CRC-16
    \param[in]  data: 数据指针
    \param[in]  len:  长度
    \retval     CRC 值。上线时低字节先发、高字节后发。
    \note       多项式 0xA001（0x8005 反转），初值 0xFFFF
*/
uint16_t modbus_crc16(const uint8_t *data, uint16_t len);

#endif /* MODBUS_CRC_H */
