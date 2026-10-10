/*!
    \file    modbus_slave.h
    \brief   Modbus RTU 从站（跑在 RS485 上）

    \note    依据《电子限速器通信协议》：
               波特率 9600，CRC-16，从站地址 0x05
               功能码 0x03 读多个寄存器、0x10 写多个寄存器
               异常回复 = 请求功能码 | 0x80，后跟异常码
*/

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>

/* 协议规定的本机地址 */
#define MB_SLAVE_ADDR               0x05U

/* ---- 调试开关：把收发的原始报文以十六进制打到调试串口 ----
   1 = 打开（联调期用），0 = 关闭（量产固件请关掉）。
   代价：每帧多占用调试串口的发送时间，115200 下大约
   3 字符/0.26ms，一帧 8 字节约 3ms。Modbus 主机超时通常
   100ms 以上，联调期开着不影响响应。 */
#ifndef MODBUS_DEBUG_DUMP
#define MODBUS_DEBUG_DUMP           1
#endif

/* 支持的功能码 */
#define MB_FC_READ_REGS             0x03U
#define MB_FC_WRITE_REGS            0x10U

/* 异常码（协议“错误代码”一节） */
#define MB_EX_ILLEGAL_FUNCTION      0x01U   /* 非法功能 */
#define MB_EX_ILLEGAL_ADDRESS       0x02U   /* 非法数据地址 */
#define MB_EX_ILLEGAL_VALUE         0x03U   /* 非法数据值 */
#define MB_EX_SLAVE_FAILURE         0x04U   /* 从站设备故障 */
#define MB_EX_ACK                   0x05U   /* 已收到，需要时间处理 */
#define MB_EX_SLAVE_BUSY            0x06U   /* 从站忙 */

/* ---------------- 寄存器偏移地址（协议“表一”） ---------------- */
#define MB_REG_SENSOR_ID            0x00U   /* R  4字节，MCU 芯片ID，占 0x00/0x01 */
#define MB_REG_WORK_MODE            0x02U   /* RW 0正常 1学习 */
#define MB_REG_LIMIT_STATE          0x03U   /* RW 0无效 1二级限速 2三级限速 */
#define MB_REG_ALARM                0x04U   /* RW 控制12V输出 0无效 1有效 */
#define MB_REG_LIMIT_MODE           0x05U   /* RW 限速方式（踏板信号开关） */
#define MB_REG_BELT_VOLT            0x06U   /* R  安全带采集电压 */
#define MB_REG_SEAT_VOLT_RAW        0x07U   /* R  座椅采集电压 */
#define MB_REG_SEAT_PRESENT_VOLT    0x08U   /* RW 座椅在位电压，默认3.5V */
#define MB_REG_SEAT_VOLT            0x09U   /* R  座椅采集电压，0.01V */
#define MB_REG_SEAT_PRESENT         0x0AU   /* R  座椅在位状态 */
#define MB_REG_OUT1_L2_VOLT         0x0BU   /* RW 通道1二级限速电压(DAC1) */
#define MB_REG_OUT2_L2_VOLT         0x0CU   /* RW 通道2二级限速电压(DAC2) */
#define MB_REG_OUT1_L3_VOLT         0x0DU   /* RW 通道1三级限速电压(DAC1) */
#define MB_REG_OUT2_L3_VOLT         0x0EU   /* RW 通道2三级限速电压(DAC2) */
#define MB_REG_OUT1_VOLT            0x0FU   /* R  通道1输出电压(ADC0) */
#define MB_REG_OUT2_VOLT            0x10U   /* R  通道2输出电压(ADC1) */
#define MB_REG_RELEASE_DURATION     0x11U   /* RW 解除限速持续时间，默认10s */
#define MB_REG_RELEASE_STATE        0x12U   /* R  解除限速状态 */
#define MB_REG_VEHICLE_STATE        0x13U   /* R  0后退 1前进 2静止 */
#define MB_REG_SUPPLY_VOLT          0x15U   /* R  供电电压，0.01V */
#define MB_REG_DEVICE_RESET         0x16U   /* W  设备复位 */
#define MB_REG_FAULT_CODE           0x17U   /* R  故障码 */
#define MB_REG_IN1_VOLT             0x18U   /* R  通道1输入电压(ADC2) */
#define MB_REG_IN2_VOLT             0x19U   /* R  通道2输入电压(ADC3) */
#define MB_REG_VERSION              0x30U   /* R  4字节固件版本，占 0x30/0x31 */

/* 寄存器空间上界（含 0x31） */
#define MB_REG_MAX                  0x32U

/* 故障码 */
#define MB_FAULT_DAC_DEVIATION      0x01U   /* DAC 输出偏差过大 */
#define MB_FAULT_ADC_OUT_OF_RANGE   0x02U   /* ADC 信号超范围 */

/*!
    \brief  寄存器访问回调，由应用层实现
    \note   返回 0 表示成功，返回非 0 值即作为 Modbus 异常码回给主机
*/
typedef struct {
    uint8_t (*reg_read)(uint16_t addr, uint16_t *value);
    uint8_t (*reg_write)(uint16_t addr, uint16_t value);
} modbus_reg_ops_t;

/*!
    \brief      初始化从站
    \param[in]  addr: 本机地址，协议为 0x05
    \param[in]  ops:  寄存器读写回调
*/
void modbus_slave_init(uint8_t addr, const modbus_reg_ops_t *ops);

/*!
    \brief      轮询处理：取一帧、校验、执行、回复
    \retval     1 本次处理了一帧，0 没有新帧
    \note       放在主循环里反复调用即可，内部不阻塞
*/
uint8_t modbus_slave_poll(void);

/* 统计信息 */
typedef struct {
    uint32_t req_total;         /* 收到的帧总数 */
    uint32_t req_for_me;        /* 地址匹配的帧数 */
    uint32_t crc_error;         /* CRC 错误帧数 */
    uint32_t exception;         /* 回了异常码的次数 */
    uint32_t rsp_sent;          /* 正常回复次数 */
} modbus_stat_t;

const modbus_stat_t *modbus_slave_stat(void);

#endif /* MODBUS_SLAVE_H */
