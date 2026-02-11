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
#include "drivers/steering_wheel.h"

SteeringWheel_t wheel;
Motor_VelCtrl_t driver_motor_ctrl;
DJI_t           steerer_motor;
VESC_t          driver_motor;
Motor_VelCtrl_t steerer_motor_vel_ctrl;
Motor_PosCtrl_t steerer_motor_pos_ctrl;
GPIO_t          photogate = { .port = GPIOE, .pin = GPIO_PIN_12 };
GPIO_PinState   gate;

void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    // APP_Chassis_Update_1kHz();
    //
    // APP_Device_Update();
    SteeringWheel_ControlUpdate(&wheel);

    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
}

void TIM_Callback_200Hz(TIM_HandleTypeDef* htim)
{
    // APP_Chassis_Update_200Hz();
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    GPIO_EXTI_Callback(pin);
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
void Init(void* argument)
{
    /* 初始化代码 */
    DJI_CAN_FilterInit(&hcan1, 0);
    VESC_CAN_FilterInit(&hcan1, 1);
    CAN_RegisterCallback(&hcan1, DJI_CAN_BaseReceiveCallback);
    CAN_RegisterCallback(&hcan1, VESC_CAN_BaseReceiveCallback);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    DJI_Init(&steerer_motor,
             &(DJI_Config_t) { .auto_zero      = true,
                               .hcan           = &hcan1,
                               .id1            = 4,
                               .motor_type     = M2006_C610,
                               .reduction_rate = 2.0f,
                               .reverse        = true });
    VESC_Init(&driver_motor,
              &(VESC_Config_t) {
                      .auto_zero  = true,
                      .hcan       = &hcan1,
                      .id         = 23,
                      .electrodes = 14,
              });

    Motor_VelCtrl_Init(
            &steerer_motor_vel_ctrl,
            &(Motor_VelCtrlConfig_t) {
                    .motor      = &steerer_motor,
                    .motor_type = MOTOR_TYPE_DJI,
                    .pid = { .Kp = 60.0f, .Ki = 0.2f, .Kd = 0.0f, .abs_output_max = 4000.0f },
            });

    Motor_PosCtrl_Init(&steerer_motor_pos_ctrl,
                       &(Motor_PosCtrlConfig_t) {
                               .motor        = &steerer_motor,
                               .motor_type   = MOTOR_TYPE_DJI,
                               .velocity_pid = { .Kp             = 60.0f,
                                                 .Ki             = 0.2f,
                                                 .Kd             = 0.0f,
                                                 .abs_output_max = 4000.0f },
                           .position_pid = {
                                   .Kp = 2.8f,
                                   .Ki = 0.06f,
                                   .Kd = 0.0f,
                                   .abs_output_max = 250.0f,
                           },
                           .pos_vel_freq_ratio = 10,
                       });
    Motor_VelCtrl_Init(&driver_motor_ctrl,
                       &(Motor_VelCtrlConfig_t) {
                               .motor_type = MOTOR_TYPE_VESC,
                               .motor      = &driver_motor,
                       });

    SteeringWheel_Init(&wheel,
                       &(SteeringWheel_Config_t) {
                               .drive_motor  = &driver_motor_ctrl,
                               .steer_motor  = &steerer_motor_pos_ctrl,
                               .steer_offset = 10.0f,
                               .calib        = {
                               .photogate = {
                                   .port = GPIOE,
                                   .pin = GPIO_PIN_12,
                               },
                               .photogate_triggered_state = GPIO_PIN_RESET,
                               .steer_motor = &steerer_motor_vel_ctrl},
                       });

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);

    __MOTOR_CTRL_ENABLE(&driver_motor_ctrl);

    osDelay(1000);

    SteeringWheel_StartCalibration(&wheel);

    while (wheel.calib.state != STEER_CALIB_DONE)
        osDelay(1);
    osDelay(2000);
    SteeringWheel_SetVelocity(&wheel,
                              &(SteeringWheel_Velocity_t) {
                                      .angle = 90.0f,
                                      .speed = 1.0f,
                              });
    osDelay(5000);
    SteeringWheel_SetVelocity(&wheel,
                              &(SteeringWheel_Velocity_t) { .angle = -45.0f, .speed = 1.0f });
    osDelay(5000);
    SteeringWheel_SetVelocity(&wheel,
                              &(SteeringWheel_Velocity_t) { .angle = -90.0f, .speed = -1.0f });
    osDelay(5000);
    SteeringWheel_SetVelocity(&wheel, &(SteeringWheel_Velocity_t) { .angle = 0.0f, .speed = 2.0f });

    for (;;)
    {
        osDelay(1);
    }

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