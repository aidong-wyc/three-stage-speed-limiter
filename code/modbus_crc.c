/*!
    \file    modbus_crc.c
    \brief   Modbus RTU CRC-16 校验
*/

#include "modbus_crc.h"

uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  bit;

    for(i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i];

        for(bit = 0U; bit < 8U; bit++) {
            if(0U != (crc & 0x0001U)) {
                crc >>= 1;
                crc ^= 0xA001U;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}
