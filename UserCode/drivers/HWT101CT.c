/**
 * @file    HWT101CT.c
 * @author  syhanjin
 * @date    2025-11-01
 */
#include "HWT101CT.h"
#ifdef USE_RTOS
#    include "cmsis_os2.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// TODO: 把这个东西放到合适的地方安放
static void delay_us(const uint32_t us)
{
    uint32_t cycles = SystemCoreClock / 4000000 * us;

    __asm volatile("1: \n"                     // 标签 1
                   "   subs %[n], %[n], #1 \n" // n--
                   "   bne 1b \n"              // 如果 n != 0，跳回标签 1
                   : [n] "+r"(cycles)          // 输出操作数（读写）
                   :                           // 无输入
                   : "cc"                      // 声明修改了条件码寄存器
    );
}

static void write_reg(const HWT101CT_t* hwt101ct,
                      const uint8_t     addr,
                      const uint8_t     data_l,
                      const uint8_t     data_h)
{
    static const uint8_t unlock_cmd[5] = { 0xFF, 0xAA, 0x69, 0x88, 0xB5 };
    static const uint8_t save_cmd[5]   = { 0xFF, 0xAA, 0x00, 0x00, 0x00 };
    const uint8_t        cmd[5]        = { 0xFF, 0xAA, addr, data_l, data_h };
    HAL_UART_Transmit(hwt101ct->huart, unlock_cmd, 5, 20);
    delay_us(250);
    HAL_UART_Transmit(hwt101ct->huart, cmd, 5, 20);
    delay_us(250);
    HAL_UART_Transmit(hwt101ct->huart, save_cmd, 5, 20);
}

/**
 * 初始化
 *
 * @attention 请确保串口回调注册在本函数调用前进行
 * @param hwt101ct HWT101CT对象
 * @param huart 所用串口
 */
void HWT101CT_Init(HWT101CT_t* hwt101ct, UART_HandleTypeDef* huart)
{
    hwt101ct->huart        = huart;
    hwt101ct->yaw          = 0.0f;
    hwt101ct->round_count  = 0;
    hwt101ct->feedback_yaw = 0.0f;
    hwt101ct->wz           = 0.0f;
    hwt101ct->sync_state   = HWT101CT_WAIT_HEAD;
    UART_Start_Receive_IT(huart, hwt101ct->rx_buffer, 1);
}

/**
 * 数据解码
 * @param hwt101ct HWT101CT对象
 * @param data 解码的数据
 */
void HWT101CT_DecodeData(HWT101CT_t* hwt101ct, const uint8_t* data)
{
    uint8_t sum = 0x55;
    for (uint8_t i = 1; i < HWT101CT_CMD_LEN - 1; i++)
        sum += data[i];
    if (sum != data[HWT101CT_CMD_LEN - 1])
    {
        // 校验和失败不判定为需要重新同步
        HWT101CT_DEBUG_FRAME_ERROR(hwt101ct);
        return;
    }
    switch (data[1])
    {
    case 0x53:
        const float new_yaw = (float) ((int16_t) (data[7] << 8) | data[6]) / 32768.0f * 180.0f;
        if (hwt101ct->feedback_yaw > 60.0f && new_yaw < -60.0f)
            hwt101ct->round_count++;
        else if (hwt101ct->feedback_yaw < -60.0f && new_yaw > 60.0f)
            hwt101ct->round_count--;
        hwt101ct->yaw          = new_yaw + (float) hwt101ct->round_count * 360.0f;
        hwt101ct->feedback_yaw = new_yaw;
        break;
    case 0x52:
        hwt101ct->wz = (float) ((int16_t) (data[7] << 8) | data[6]) / 32768.0f * 2000.0f;
        break;
    default:
        HWT101CT_DEBUG_FRAME_ERROR(hwt101ct);
        return;
    }
}

/**
 * 串口 Rx 接收回调
 *
 * 请用户自行维护回调调用关系
 * @param hwt101ct HWT101CT对象
 */
void HWT101CT_RxCallback(HWT101CT_t* hwt101ct)
{
    if (hwt101ct->sync_state == HWT101CT_DMA_ACTIVE)
    {
        HWT101CT_DEBUG_FRAME_COUNT(hwt101ct);
        if (hwt101ct->rx_buffer[0] != HWT101CT_HEAD)
        {
            HAL_UART_DMAStop(hwt101ct->huart);
            hwt101ct->sync_state = HWT101CT_WAIT_HEAD;
            HAL_UART_Receive_IT(hwt101ct->huart, hwt101ct->rx_buffer, 1);
        }
        HWT101CT_DecodeData(hwt101ct, hwt101ct->rx_buffer);
    }
    else if (hwt101ct->sync_state == HWT101CT_RECEIVING)
    {
        HWT101CT_DEBUG_FRAME_COUNT(hwt101ct);
        HWT101CT_DEBUG_SYNC_COUNT(hwt101ct);
        HWT101CT_DecodeData(hwt101ct, hwt101ct->rx_buffer);
        HAL_UART_Receive_DMA(hwt101ct->huart, hwt101ct->rx_buffer, HWT101CT_CMD_LEN);
        hwt101ct->sync_state = HWT101CT_DMA_ACTIVE;
    }
    else if (hwt101ct->sync_state == HWT101CT_WAIT_HEAD)
    {
        if (hwt101ct->rx_buffer[0] == HWT101CT_HEAD)
        {
            HAL_UART_Receive_IT(hwt101ct->huart, hwt101ct->rx_buffer + 1, HWT101CT_CMD_LEN - 1);
            hwt101ct->sync_state = HWT101CT_RECEIVING;
        }
        else
        {
            HAL_UART_Receive_IT(hwt101ct->huart, hwt101ct->rx_buffer, 1);
        }
    }
}

/**
 * 重置 yaw
 *
 * @attention 本函数并不会立即设置 yaw 为 0，而是写入传感器寄存器，下一次接收到数据时才会重置。
 *            同时本函数带有大约 2ms 的阻塞
 * @param hwt101ct
 */
void HWT101CT_ResetYaw(HWT101CT_t* hwt101ct)
{
    // hwt101ct->yaw = 0.0f;
    hwt101ct->round_count  = 0;
    hwt101ct->feedback_yaw = 0.0f;
    write_reg(hwt101ct, 0x76, 0x00, 0x00);
}

/**
 * 校准陀螺仪零偏
 *
 * @attention 校准过程中，请确保传感器处于静止状态。故该函数为阻塞函数
 * @param hwt101ct HWT101CT对象
 * @param duration_ms 零偏校准持续时间
 */
void HWT101CT_Calibrate(const HWT101CT_t* hwt101ct, const uint32_t duration_ms)
{
    write_reg(hwt101ct, 0xA6, 0x01, 0x01);
#ifdef USE_RTOS
    osDelay(duration_ms);
#else
    HAL_Delay(duration_ms);
#endif

    write_reg(hwt101ct, 0xA6, 0x04, 0x00);
}

/**
 * 设置陀螺仪输出频率
 *
 * @note 实际上因为反馈有角度和角速度，所以串口接收数据的频率是设置值的两倍
 * @param hwt101ct HWT101CT对象
 * @param rate 输出频率
 */
void HWT101CT_SetOutputRate(const HWT101CT_t* hwt101ct, const HWT101CT_RRate_t rate)
{
    write_reg(hwt101ct, 0x03, rate, 0x00);
}

#ifdef __cplusplus
}
#endif
