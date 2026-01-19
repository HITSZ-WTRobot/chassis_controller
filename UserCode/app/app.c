/**
 * @file    app.h
 * @author  syhanjin
 * @date    2025-11-08
 */
#include "app.h"
#include "can.h"
#include "chassis.h"
#include "cmsis_os2.h"
#include "system.h"
#include "device.h"
#include "tim.h"
#include "usart.h"
#include "bsp/can_driver.h"
#include "drivers/DJI.h"
#include "drivers/HWT101CT.h"
#include "interfaces/chassis_if.h"
#include "interfaces/motor_if.h"

void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    APP_Chassis_Update_1kHz();

    APP_Device_Update();
}

void TIM_Callback_200Hz(TIM_HandleTypeDef* htim)
{
    APP_Chassis_Update_200Hz();
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
void Init(void* argument)
{
    /* 初始化代码 */
    APP_Device_Init();

    APP_Chassis_InitBeforeUpdate();

    // 注册定时器
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_200Hz);
    HAL_TIM_Base_Start_IT(&htim13);

    // 一秒钟时间等待各种东西更新
    osDelay(1000);

    APP_Chassis_Init();

    osEventFlagsSet(systemEventHandle, SYSTEM_INITIALIZED);

    /* 初始化完成后退出线程 */
    osThreadExit();
}

void StartTestRun(void* argument)
{
    System_WaitInitialize();
    osDelay(2000);

    Chassis_SetTargetPostureInWorld(&chassis, &(Chassis_PostureTarget_t) {
        .posture = {
            .x = 1.0f,
            .y = 0,
            .yaw = 0,
        },
        .limit_x = {
            .max_spd = 5,
            .max_acc = 1.2f,
            .max_jerk = 2,
        },
        .limit_y = {
            .max_spd = 5,
            .max_acc = 1.2f,
            .max_jerk = 2,
        },
        .limit_yaw = {
            .max_spd = 180,
            .max_acc = 60,
            .max_jerk = 360,
        },
    });

    while (!Chassis_TrajectoryIsFinished(&chassis))
    {
        osDelay(1);
    }

    Chassis_SetTargetPostureInBody(&chassis, &(Chassis_PostureTarget_t) {
        .posture = {
            .x = 0,
            .y = 0,
            .yaw = 90.0f,
        },
        .limit_x = {
            .max_spd = 5,
            .max_acc = 1,
            .max_jerk = 2,
        },
        .limit_y = {
            .max_spd = 5,
            .max_acc = 1,
            .max_jerk = 2,
        },
        .limit_yaw = {
            .max_spd = 180,
            .max_acc = 45,
            .max_jerk = 90,
        },
    });

    while (!Chassis_TrajectoryIsFinished(&chassis))
    {
        osDelay(1);
    }

    Chassis_SetTargetPostureInBody(&chassis, &(Chassis_PostureTarget_t) {
        .posture = {
            .x = 0,
            .y = -0.5f,
            .yaw = 0,
        },
        .limit_x = {
            .max_spd = 5,
            .max_acc = 1,
            .max_jerk = 2,
        },
        .limit_y = {
            .max_spd = 5,
            .max_acc = 1,
            .max_jerk = 2,
        },
        .limit_yaw = {
            .max_spd = 180,
            .max_acc = 45,
            .max_jerk = 90,
        },
    });

    while (!Chassis_TrajectoryIsFinished(&chassis))
    {
        osDelay(1);
    }

    Chassis_SetTargetPostureInBody(&chassis, &(Chassis_PostureTarget_t) {
        .posture = {
            .x = 3.0f,
            .y = 1.0f,
            .yaw = 180.0f,
        },
        .limit_x = {
            .max_spd = 5,
            .max_acc = 0.5f,
            .max_jerk = 2,
        },
        .limit_y = {
            .max_spd = 5,
            .max_acc = 0.5f,
            .max_jerk = 2,
        },
        .limit_yaw = {
            .max_spd = 180,
            .max_acc = 15.0f,
            .max_jerk = 90,
        },
    });

    while (!Chassis_TrajectoryIsFinished(&chassis))
    {
        osDelay(1);
    }

    Chassis_SetVelBodyFrame(&chassis, &(Chassis_Velocity_t) { .vx = 0, .vy = 0, .wz = 0 }, false);
}