/*!
    \file    rs485.c
    \brief   RS485 半双工收发驱动（USART2 + PB2 方向控制，Modbus RTU 帧定界）
*/

#include <stddef.h>

#include "rs485.h"
#include "bsp_board.h"

/* t3.5 定界用的定时器：TIMER5 是 APB1 上的基本定时器，不占用任何引脚 */
#define RS485_T35_TIMER         TIMER5
#define RS485_T35_TIMER_RCU     RCU_TIMER5
#define RS485_T35_IRQN          TIMER5_IRQn

/* 方向控制：DE 与 /RE 并接，高=发送，低=接收 */
#define RS485_DE_TX()           gpio_bit_set(BSP_RS485_DE_PORT, BSP_RS485_DE_PIN)
#define RS485_DE_RX()           gpio_bit_reset(BSP_RS485_DE_PORT, BSP_RS485_DE_PIN)

/* 接收状态机 */
#define RX_IDLE                 0U      /* 空闲，等第一个字节 */
#define RX_RECEIVING            1U      /* 正在收，t3.5 定时器在跑 */
#define RX_FRAME_READY          2U      /* 一帧收完，等上层取走 */
#define RX_GUARD                3U      /* 刚发完，静默保护期，丢弃一切收到的东西 */

static uint8_t           s_rx_buf[RS485_BUF_SIZE];
static volatile uint16_t s_rx_len;
static volatile uint8_t  s_rx_state;
static volatile uint8_t  s_rx_bad;          /* 本帧出现过错误或超长 */

static uint8_t           s_tx_buf[RS485_BUF_SIZE];
static volatile uint16_t s_tx_len;
static volatile uint16_t s_tx_idx;
static volatile uint8_t  s_tx_busy;

static rs485_stat_t      s_stat;

/*!
    \brief      算出 APB1 上定时器的实际时钟
    \note       GD32F30x 沿用 STM32F1 规则：APB 预分频为 1 时定时器时钟 = APB，
                否则 = APB x 2。本板 APB1 = AHB/2，所以定时器时钟 = 60M x 2 = 120MHz。
                这里在运行期算，改了分频也不会算错。
*/
static uint32_t apb1_timer_clock(void)
{
    uint32_t apb1 = rcu_clock_freq_get(CK_APB1);
    uint32_t ahb  = rcu_clock_freq_get(CK_AHB);

    return (apb1 == ahb) ? apb1 : (apb1 * 2U);
}

/*!
    \brief      按波特率算 t3.5 静默时间（微秒）
    \note       Modbus RTU 规范：一个字符按 11 位算（起始1+数据8+校验1+停止1）。
                波特率高于 19200 时规范固定取 1.75ms。
                9600 时 t3.5 = 3.5 x 11 / 9600 = 4010us。
*/
static uint16_t t35_us_calc(uint32_t baudrate)
{
    uint32_t us;

    if(baudrate > 19200U) {
        return 1750U;
    }

    us = (35U * 11U * 1000000U) / (baudrate * 10U);

    if(us > 60000U) {
        us = 60000U;
    }

    return (uint16_t)us;
}

/*!
    \brief      配置 t3.5 定时器为 1us 计数、到点产生一次更新中断
*/
static void t35_timer_config(uint32_t baudrate)
{
    timer_parameter_struct tp;
    uint32_t tclk = apb1_timer_clock();

    rcu_periph_clock_enable(RS485_T35_TIMER_RCU);
    timer_deinit(RS485_T35_TIMER);

    timer_struct_para_init(&tp);
    tp.prescaler        = (uint16_t)((tclk / 1000000U) - 1U);   /* 分频到 1MHz，1 计数 = 1us */
    tp.alignedmode      = TIMER_COUNTER_EDGE;
    tp.counterdirection = TIMER_COUNTER_UP;
    tp.period           = (uint16_t)(t35_us_calc(baudrate) - 1U);
    tp.clockdivision    = TIMER_CKDIV_DIV1;
    tp.repetitioncounter = 0U;
    timer_init(RS485_T35_TIMER, &tp);

    /* 清掉 timer_init 产生的那次更新标志，否则一使能就立刻进中断 */
    timer_interrupt_flag_clear(RS485_T35_TIMER, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(RS485_T35_TIMER, TIMER_INT_UP);

    /* 先不启动，收到第一个字节才开始计时 */
    timer_disable(RS485_T35_TIMER);
}

/*!
    \brief      重启 t3.5 计时（每收到一个字节调用一次）
*/
static void t35_restart(void)
{
    timer_disable(RS485_T35_TIMER);
    timer_counter_value_config(RS485_T35_TIMER, 0U);
    timer_interrupt_flag_clear(RS485_T35_TIMER, TIMER_INT_FLAG_UP);
    timer_enable(RS485_T35_TIMER);
}

/*!
    \brief      初始化 RS485
*/
void rs485_init(uint32_t baudrate)
{
    s_rx_len   = 0U;
    s_rx_state = RX_IDLE;
    s_rx_bad   = 0U;
    s_tx_len   = 0U;
    s_tx_idx   = 0U;
    s_tx_busy  = 0U;

    /* DE 已在 bsp_board_init() 里配好并置为接收态，这里只兜底再置一次 */
    RS485_DE_RX();

    rcu_periph_clock_enable(BSP_RS485_GPIO_RCU);
    rcu_periph_clock_enable(BSP_RS485_USART_RCU);

    /* PB10 = USART2_TX 复用推挽。
       PB10/PB11 是 USART2 默认引脚，不需要 AFIO 重映射。 */
    gpio_init(BSP_RS485_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BSP_RS485_TX_PIN);

    /* PB11 = USART2_RX，必须用【上拉】输入，不能用浮空。
       SIT3485E/SN65HVD72 在 /RE 拉高（也就是我们发送）时 RO 是高阻态，
       PB11 若浮空就会在发送期间悬空，被旁边翻转的 PB10 耦合出假起始位，
       表现为每发一帧就多一次 framing error。
       上拉住之后 RO 高阻期间 PB11 保持高电平（UART 空闲态），不会误触发。 */
    gpio_init(BSP_RS485_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, BSP_RS485_RX_PIN);

    usart_deinit(BSP_RS485_USART);
    usart_baudrate_set(BSP_RS485_USART, baudrate);
    usart_word_length_set(BSP_RS485_USART, USART_WL_8BIT);
    usart_stop_bit_set(BSP_RS485_USART, USART_STB_1BIT);
    usart_parity_config(BSP_RS485_USART, USART_PM_NONE);
    usart_hardware_flow_rts_config(BSP_RS485_USART, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(BSP_RS485_USART, USART_CTS_DISABLE);
    usart_receive_config(BSP_RS485_USART, USART_RECEIVE_ENABLE);
    usart_transmit_config(BSP_RS485_USART, USART_TRANSMIT_ENABLE);
    usart_enable(BSP_RS485_USART);

    t35_timer_config(baudrate);

    /* RS485 收发时序比 SysTick 关键，优先级设得比 SysTick 高（数值更小）。
       t3.5 定时器优先级要低于串口，保证串口先把字节收进来。 */
    nvic_irq_enable(BSP_RS485_IRQN, 1U, 0U);
    nvic_irq_enable(RS485_T35_IRQN, 2U, 0U);

    usart_interrupt_enable(BSP_RS485_USART, USART_INT_RBNE);
}

/*!
    \brief      取一帧已接收数据
*/
uint16_t rs485_frame_get(uint8_t *buf, uint16_t bufsize)
{
    uint16_t len = 0U;

    if((NULL == buf) || (0U == bufsize)) {
        return 0U;
    }

    if(RX_FRAME_READY != s_rx_state) {
        return 0U;
    }

    /* 本帧收过错误/超长，直接丢弃并复位接收 */
    if(s_rx_bad) {
        s_rx_bad   = 0U;
        s_rx_len   = 0U;
        s_rx_state = RX_IDLE;
        return 0U;
    }

    len = s_rx_len;
    if(len > bufsize) {
        len = bufsize;
    }

    /* 拷贝期间关中断，避免新字节改写正在读的缓冲 */
    __disable_irq();
    {
        uint16_t i;
        for(i = 0U; i < len; i++) {
            buf[i] = s_rx_buf[i];
        }
        s_rx_len   = 0U;
        s_rx_state = RX_IDLE;
    }
    __enable_irq();

    return len;
}

/*!
    \brief      发送一帧（中断驱动）
*/
uint8_t rs485_send(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if((NULL == data) || (0U == len) || (len > RS485_BUF_SIZE)) {
        return 0U;
    }

    if(s_tx_busy) {
        return 0U;
    }

    for(i = 0U; i < len; i++) {
        s_tx_buf[i] = data[i];
    }

    s_tx_len  = len;
    s_tx_idx  = 0U;
    s_tx_busy = 1U;

    /* 先切到发送方向，再开 TBE 中断 */
    RS485_DE_TX();

    /* 清一次 TC，避免上一帧残留的 TC 立刻触发收尾逻辑 */
    usart_flag_clear(BSP_RS485_USART, USART_FLAG_TC);
    usart_interrupt_enable(BSP_RS485_USART, USART_INT_TBE);

    return 1U;
}

uint8_t rs485_tx_busy(void)
{
    return s_tx_busy;
}

const rs485_stat_t *rs485_stat(void)
{
    return &s_stat;
}

/*!
    \brief      USART2 中断服务
    \note       一次性读出 STAT0，同一次进中断里同时处理数据和错误标志。
                GD32/STM32F1 的 ORERR/FERR/NERR 都是"读 STAT0 再读 DATA"清除。
*/
void rs485_usart_irq_handler(void)
{
    uint32_t stat = USART_STAT0(BSP_RS485_USART);
    uint32_t ctl0 = USART_CTL0(BSP_RS485_USART);

    /* ---------------- 接收 ---------------- */
    if(0U != (stat & (USART_STAT0_RBNE | USART_STAT0_ORERR |
                      USART_STAT0_FERR | USART_STAT0_NERR))) {

        /* 读 DATA：既取走数据，也清掉上面几个错误标志 */
        uint8_t ch = (uint8_t)(USART_DATA(BSP_RS485_USART) & 0xFFU);

        if(0U != (stat & USART_STAT0_ORERR)) {
            s_stat.err_overrun++;
            s_rx_bad = 1U;
        }
        if(0U != (stat & USART_STAT0_FERR)) {
            s_stat.err_frame++;
            s_rx_bad = 1U;
        }
        if(0U != (stat & USART_STAT0_NERR)) {
            s_stat.err_noise++;
            s_rx_bad = 1U;
        }

        /* 发送刚结束的静默保护期内，收到的一律丢掉。
           本板 A/B 上的失效保护偏置是 51K（配 120Ω 终端只有约 4mV 差分），
           DE 撒手后总线要爬一段时间才回到空闲电平，
           这段时间接收器会把拖尾当成一个起始位，产生幻影字节（实测为 0xE0）。
           不拦住的话：回复 -> 幻影字节 -> 又当成请求 -> 再回复，会自激。 */
        if(RX_GUARD == s_rx_state) {
            s_stat.rx_guarded++;
        }
        /* 上一帧还没被取走时，新来的字节只能丢，否则会踩坏缓冲 */
        else if(RX_FRAME_READY == s_rx_state) {
            s_stat.rx_dropped++;
        } else {
            if(s_rx_len < RS485_BUF_SIZE) {
                s_rx_buf[s_rx_len] = ch;
                s_rx_len++;
            } else {
                /* 超长帧：标记为坏帧，继续等 t3.5 结束再整帧丢掉 */
                s_rx_bad = 1U;
            }

            s_rx_state = RX_RECEIVING;
            t35_restart();
        }
    }

    /* ---------------- 发送：数据寄存器空 ---------------- */
    if((0U != (ctl0 & USART_CTL0_TBEIE)) && (0U != (stat & USART_STAT0_TBE))) {
        if(s_tx_idx < s_tx_len) {
            USART_DATA(BSP_RS485_USART) = (uint32_t)s_tx_buf[s_tx_idx];
            s_tx_idx++;
        } else {
            /* 最后一个字节已经写进移位器，改等 TC 再放开 DE */
            usart_interrupt_disable(BSP_RS485_USART, USART_INT_TBE);
            usart_interrupt_enable(BSP_RS485_USART, USART_INT_TC);
        }
    }

    /* ---------------- 发送：整帧移位完成 ---------------- */
    if((0U != (ctl0 & USART_CTL0_TCIE)) && (0U != (stat & USART_STAT0_TC))) {
        usart_interrupt_disable(BSP_RS485_USART, USART_INT_TC);
        usart_flag_clear(BSP_RS485_USART, USART_FLAG_TC);

        /* TC 置位说明最后一位已经移出去了，此刻放开 DE 不会截断尾字节 */
        RS485_DE_RX();

        s_tx_busy = 0U;
        s_tx_len  = 0U;
        s_tx_idx  = 0U;
        s_stat.tx_frames++;

        /* 进入静默保护期：丢弃 t3.5 时间内收到的一切。
           一是吃掉 DE 释放瞬间总线爬升产生的幻影字节，
           二是 Modbus 规范本来就要求主机在收到回复后等 t3.5 才能再发，
           所以这段时间丢东西不会丢掉真正的请求。 */
        s_rx_len   = 0U;
        s_rx_bad   = 0U;
        s_rx_state = RX_GUARD;
        t35_restart();
    }
}

/*!
    \brief      t3.5 定时器中断：静默超时即认为一帧结束
*/
void rs485_t35_irq_handler(void)
{
    if(RESET == timer_interrupt_flag_get(RS485_T35_TIMER, TIMER_INT_FLAG_UP)) {
        return;
    }

    timer_interrupt_flag_clear(RS485_T35_TIMER, TIMER_INT_FLAG_UP);
    timer_disable(RS485_T35_TIMER);

    if(RX_RECEIVING == s_rx_state) {
        if(0U == s_rx_len) {
            s_rx_state = RX_IDLE;
        } else {
            s_rx_state = RX_FRAME_READY;
            s_stat.rx_frames++;
        }
    } else if(RX_GUARD == s_rx_state) {
        /* 静默保护期结束，恢复正常接收。
           注意这里是定长保护，不会被期间的噪声无限延长，
           否则总线有持续干扰时会永远收不到数据。 */
        s_rx_len   = 0U;
        s_rx_bad   = 0U;
        s_rx_state = RX_IDLE;
    }
}
