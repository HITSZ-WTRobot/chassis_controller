/**
 * @file    device.h
 * @author  syhanjin
 * @date    2025-12-02
 * @brief   定义整机使用的所有设备
 */
#ifndef DEVICE_H
#define DEVICE_H

#include "can.h"
#include "usart.h"
#include "bsp/can_driver.h"
#include "drivers/DJI.h"
#include "drivers/HWT101CT.h"
#include "libs/pid_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * 底盘使用的轮子
 */
extern DJI_t motor_wheel[4];

/**
 * 底盘使用的陀螺仪
 */
extern HWT101CT_t sensor_gyro_yaw;

static void APP_Device_Update()
{
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
}

static void APP_Device_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == sensor_gyro_yaw.huart)
    {
        HWT101CT_RxCallback(&sensor_gyro_yaw);
    }
}

void APP_Device_Init();

#ifdef __cplusplus
}
#endif

#endif // DEVICE_H
