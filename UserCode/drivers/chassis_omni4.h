/**
 * @file    chassis_omni4.h
 * @author  modified from syhanjin's code
 * @date    2025-10-17
 * @brief   4 wheels Omni chassis driver
 */
#ifndef CHASSIS_OMNI4_H
#define CHASSIS_OMNI4_H

#include "interfaces/motor_if.h"
#include <math.h>

#define RPS2RPM(__RPS__) ((__RPS__) * 60.0f / (2 * 3.14159265358979323846f))
#define DEG2RAD(__DEG__) ((__DEG__) * (float) 3.14159265358979323846f / 180.0f)
#define RAD2DEG(__RAD__) ((__RAD__) * 180.0f / (float) 3.14159265358979323846f)

typedef enum
{
    OMNI4_WHEEL_FR = 0U, ///< 右前轮
    OMNI4_WHEEL_FL,      ///< 左前轮
    OMNI4_WHEEL_RL,      ///< 左后轮
    OMNI4_WHEEL_RR,      ///< 右后轮
    OMNI4_WHEEL_MAX
} Omni4_WheelType_t;

typedef struct
{
    float            wheel_radius; ///< 轮子半径 (unit: m)
    float            half_diag;    ///< 轮子到中心的半对角线距离 (unit: m)
    Motor_VelCtrl_t* wheel[OMNI4_WHEEL_MAX];
} Omni4_t;

typedef struct
{
    float            wheel_radius;     ///< 轮子半径 (unit: mm)
    float            wheel_distance_x; ///< 左右轮距 (unit: mm)
    float            wheel_distance_y; ///< 前后轮距 (unit: mm)
    Motor_VelCtrl_t* wheel_front_right;
    Motor_VelCtrl_t* wheel_front_left;
    Motor_VelCtrl_t* wheel_rear_left;
    Motor_VelCtrl_t* wheel_rear_right;
} Omni4_Config_t;

void Omni4_Init(Omni4_t* chassis, const Omni4_Config_t* config);
void Omni4_ApplyVelocity(Omni4_t* chassis, float vx, float vy, float wz);
void Omni4_Update(const Omni4_t* chassis);

// 正运动学解算
float Omni4Forward_GetYaw(const Omni4_t* chassis);
float Omni4Forward_GetX(const Omni4_t* chassis);
float Omni4Forward_GetY(const Omni4_t* chassis);

#endif // CHASSIS_OMNI4_H