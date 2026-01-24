/**
 * @file    chassis_steering4.h
 * @author  syhanjin
 * @date    2025-10-03
 * @brief   4轮舵轮底盘驱动
 */
#ifndef CHASSIS_STEERING4_H
#define CHASSIS_STEERING4_H

#include <math.h>
#include "drivers/steering_wheel.h"

#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180.0f)
#define RAD2DEG(__RAD__) ((__RAD__) * 180.0f / 3.14159265358979323846f)

typedef enum
{
    STEERING4_WHEEL_FR = 0U,
    STEERING4_WHEEL_FL,
    STEERING4_WHEEL_RL,
    STEERING4_WHEEL_RR,
    STEERING4_WHEEL_MAX
} Steering4_WheelType_t;

typedef struct
{
    bool heading_lock;  ///< 是否锁定航向
    bool is_calibrated; ///< 是否校准

    SteeringWheel_t wheel[STEERING4_WHEEL_MAX];
    float           wheel_last_steer_angle[STEERING4_WHEEL_MAX];
    float           wheel_last_drive_angle[STEERING4_WHEEL_MAX];

    float radius; ///< 驱动轮半径 (unit: m)
    float k_rpm;

    float half_width, half_height; ///< 半宽，半高
    float inv_l2;                  ///< 1 / (半宽^2 + 半高^2)
} Steering4_t;

typedef struct
{
    SteeringWheel_Config_t wheel_front_right; ///< 右前方
    SteeringWheel_Config_t wheel_front_left;  ///< 左前方
    SteeringWheel_Config_t wheel_rear_left;   ///< 左后方
    SteeringWheel_Config_t wheel_rear_right;  ///< 右后方

    float radius; ///< 驱动轮半径 (unit: mm)

    float width;  ///< 底盘宽度
    float height; ///< 底盘高度

} Steering4_Config_t;

void Steering4_ApplyVelocity(Steering4_t* chassis, float vx, float vy, float wz);
void Steering4_Update(Steering4_t* chassis);

// forward kinematics solutions
float Steering4Forward_GetYaw(const Steering4_t* chassis);
float Steering4Forward_GetX(const Steering4_t* chassis);
float Steering4Forward_GetY(const Steering4_t* chassis);
float Steering4Forward_GetWz(const Steering4_t* chassis);
float Steering4Forward_GetVx(const Steering4_t* chassis);
float Steering4Forward_GetVy(const Steering4_t* chassis);

#endif // CHASSIS_STEERING4_H
