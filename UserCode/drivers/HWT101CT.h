/**
 * @file    HWT101CT.h
 * @author  syhanjin
 * @date    2025-11-01
 * @brief   维特智能 HWT101CT 单轴高精度角度传感器驱动
 */
#ifndef HWT101CT_H
#define HWT101CT_H
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define HWT101CT_HEAD    (0x55)
#define HWT101CT_CMD_LEN (11U)

#ifdef DEBUG
#    define HWT101CT_DEBUG_FRAME_COUNT(__HWT101CT__) ((__HWT101CT__)->frame_count++)
#    define HWT101CT_DEBUG_FRAME_ERROR(__HWT101CT__) ((__HWT101CT__)->error_count++)
#    define HWT101CT_DEBUG_SYNC_COUNT(__HWT101CT__)  ((__HWT101CT__)->sync_count++)
#else
#    define HWT101CT_DEBUG_FRAME_COUNT(__HWT101CT__) ((void*) (__HWT101CT__))
#    define HWT101CT_DEBUG_FRAME_ERROR(__HWT101CT__) ((void*) (__HWT101CT__))
#    define HWT101CT_DEBUG_SYNC_COUNT(__HWT101CT__)  ((void*) (__HWT101CT__))
#endif

/**
 * 模块输出速率枚举
 */
typedef enum
{
    HWT101CT_RRATE_0_2HZ  = 0x01, ///< 0.2Hz
    HWT101CT_RRATE_0_5HZ  = 0x02, ///< 0.5Hz
    HWT101CT_RRATE_1HZ    = 0x03, ///< 1Hz
    HWT101CT_RRATE_2HZ    = 0x04, ///< 2Hz
    HWT101CT_RRATE_5HZ    = 0x05, ///< 5Hz
    HWT101CT_RRATE_10HZ   = 0x06, ///< 10Hz
    HWT101CT_RRATE_20HZ   = 0x07, ///< 20Hz
    HWT101CT_RRATE_50HZ   = 0x08, ///< 50Hz
    HWT101CT_RRATE_100HZ  = 0x09, ///< 100Hz
    HWT101CT_RRATE_200HZ  = 0x0B, ///< 200Hz
    HWT101CT_RRATE_500HZ  = 0x0C, ///< 500Hz
    HWT101CT_RRATE_1000Hz = 0x0D, ///< 1000Hz
} HWT101CT_RRate_t;

typedef enum
{
    HWT101CT_WAIT_HEAD = 0U,
    HWT101CT_RECEIVING,
    HWT101CT_DMA_ACTIVE,
} HWT101CT_SyncState_t;

typedef struct
{
    UART_HandleTypeDef*  huart;
    HWT101CT_SyncState_t sync_state;   ///< 同步状态
    float                feedback_yaw; ///< 反馈偏航角（-180~180）
    int32_t              round_count;  ///< 圈数统计
    float                yaw;          ///< 偏航角
    float                wz;           ///< 角速度

    uint8_t rx_buffer[HWT101CT_CMD_LEN];

#ifdef DEBUG
    uint32_t sync_count;  ///< 同步计数
    uint32_t frame_count; ///< 帧数
    uint32_t error_count; ///< 错误帧数
#endif
} HWT101CT_t;

static float HWT101CT_GetYaw(const HWT101CT_t* hwt101ct)
{
    return hwt101ct->yaw;
}
static float HWT101CT_GetWz(const HWT101CT_t* hwt101ct)
{
    return hwt101ct->wz;
}

void HWT101CT_Init(HWT101CT_t* hwt101ct, UART_HandleTypeDef* huart);
void HWT101CT_RxCallback(HWT101CT_t* hwt101ct);
void HWT101CT_ResetYaw(HWT101CT_t* hwt101ct);
void HWT101CT_Calibrate(const HWT101CT_t* hwt101ct, uint32_t duration_ms);
void HWT101CT_SetOutputRate(const HWT101CT_t* hwt101ct, HWT101CT_RRate_t rate);

#ifdef __cplusplus
}
#endif

#endif // HWT101CT_H
