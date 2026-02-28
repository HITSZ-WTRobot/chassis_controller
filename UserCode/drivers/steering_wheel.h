/**
 * @file    steering_wheel.h
 * @author  syhanjin
 * @date    2025-10-03
 * @brief   Steering wheel drive
 *
 * 舵轮驱动
 */
#ifndef STEERING_WHEEL_H
#define STEERING_WHEEL_H

#include <math.h>
#include "bsp/gpio_driver.h"
#include "interfaces/motor_if.h"

/**
 * Rad/s to Round/min
 */
#define RPS2RPM(__RPS__) ((__RPS__) / (2 * 3.14159265358979323846f) * 60)

typedef struct
{
    float angle; ///< 舵轮角度，正前方为零点，逆时针为正 (unit: deg)
    float speed; ///< 舵轮速度，angle = 0 是向正前方运动为正 (unit: m/s)
} SteeringWheel_Velocity_t;

typedef enum
{
    STEER_CALIB_IDLE = 0U,    ///< 校准未开始
    STEER_CALIB_SEEK_GATE,    ///< 正转寻找光电门
    STEER_CALIB_FINE_CAPTURE, ///< 进入光电门后减速移出
    STEER_CALIB_DONE,         ///< 移出瞬间捕获角度设为零点，校准完成
} SteeringWheel_Calib_State_t;

typedef struct
{
    Motor_VelCtrl_t* drive_motor; ///< 驱动电机句柄
    Motor_PosCtrl_t* steer_motor; ///< 转向电机句柄

    float steer_offset; ///< 转向电机偏移量，即光电门所在角度

    struct
    {
        SteeringWheel_Calib_State_t state;                     ///< 校准状态
        Motor_VelCtrl_t*            steer_motor;               ///< 转向电机速度控制
        GPIO_t                      photogate;                 ///< 光电门引脚
        GPIO_PinState               photogate_triggered_state; ///< 光电门被触发状态
    } calib;                                                   ///< 校准

    SteeringWheel_Velocity_t velocity; ///< 速度
} SteeringWheel_t;

typedef struct
{
    Motor_VelCtrl_t* drive_motor;  ///< 驱动电机句柄
    Motor_PosCtrl_t* steer_motor;  ///< 转向电机句柄
    float            steer_offset; ///< 转向电机偏移量，即光电门所在角度

    struct
    {
        Motor_VelCtrl_t* steer_motor;               ///< 转向电机速度控制
        GPIO_t           photogate;                 ///< 光电门引脚
        GPIO_PinState    photogate_triggered_state; ///< 光电门被触发状态
    } calib;                                        ///< 校准设备
} SteeringWheel_Config_t;

void SteeringWheel_Init(SteeringWheel_t* wheel, const SteeringWheel_Config_t* config);
void SteeringWheel_StartCalibration(SteeringWheel_t* wheel);
void SteeringWheel_SetVelocity(SteeringWheel_t* wheel, const SteeringWheel_Velocity_t* velocity);
void SteeringWheel_ControlUpdate(const SteeringWheel_t* wheel);

static bool SteeringWheel_isCalibrated(const SteeringWheel_t* wheel)
{
    return wheel->calib.state == STEER_CALIB_DONE;
}

static float SteeringWheel_GetSteerAngle(const SteeringWheel_t* wheel)
{
    return MotorCtrl_GetAngle(wheel->steer_motor);
}

static float SteeringWheel_GetDriveAngle(const SteeringWheel_t* wheel)
{
    return MotorCtrl_GetAngle(wheel->drive_motor);
}

static float SteeringWheel_GetDriveSpeed(const SteeringWheel_t* wheel)
{
    return MotorCtrl_GetVelocity(wheel->drive_motor);
}

#endif // STEERING_WHEEL_H
