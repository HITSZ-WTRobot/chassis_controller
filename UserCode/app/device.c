/**
 * @file    device.c
 * @author  syhanjin
 * @date    2025-12-02
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "device.h"
#ifdef __cplusplus
extern "C"
{
#endif
DJI_t motor_wheel[4];

HWT101CT_t sensor_gyro_yaw;

void APP_Device_Init()
{
    // 陀螺仪初始化
    // HAL_UART_RegisterCallback(&huart3, HAL_UART_RX_COMPLETE_CB_ID, APP_Device_RxCpltCallback);
    // HWT101CT_Init(&sensor_gyro_yaw, &huart3);
    // HWT101CT_ResetYaw(&sensor_gyro_yaw);

    // 初始化 CAN
    DJI_CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, 0, DJI_CAN_BaseReceiveCallback);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    static const DJI_Config_t motor_wheel_config[4] = {
        {
                .hcan       = &hcan1,
                .id1        = 1,
                .motor_type = M3508_C620,
                .reverse    = true,
        },
        {
                .hcan       = &hcan1,
                .id1        = 2,
                .motor_type = M3508_C620,
                .reverse    = false,
        },
        {
                .hcan       = &hcan1,
                .id1        = 3,
                .motor_type = M3508_C620,
                .reverse    = false,
        },
        {
                .hcan       = &hcan1,
                .id1        = 4,
                .motor_type = M3508_C620,
                .reverse    = true,
        },
    };

    // 初始化电机
    for (size_t i = 0; i < 4; i++)
        DJI_Init(&motor_wheel[i], &motor_wheel_config[i]);
}
#ifdef __cplusplus
}
#endif