/**
 * @file    chassis_omni4.h
 * @author  modified from syhanjin's code
 * @date    2025-10-17
 * @brief   4 wheels Omni chassis driver
 *
 * --------------------------------------------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Project repository: https://github.com/HITSZ-WTRobot/chassises_controller
 */
#ifndef CHASSIS_OMNI4_H
#define CHASSIS_OMNI4_H

#include "interfaces/motor_if.h"
#include <math.h>

#define RPS2RPM(__RPS__) ((__RPS__) * 60.0f / (2 * 3.14159265358979323846f))
#define DEG2RAD(__DEG__) ((__DEG__) * (float) 3.14159265358979323846f / 180.0f)
#define RAD2DEG(__RAD__) ((__RAD__) * 180.0f / (float) 3.14159265358979323846f)
#define RPM2DPS(__RPM__) ((__RPM__) / 60.0f * 360.0f)

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
float Omni4Forward_GetWz(const Omni4_t* chassis);
float Omni4Forward_GetVx(const Omni4_t* chassis);
float Omni4Forward_GetVy(const Omni4_t* chassis);

#endif // CHASSIS_OMNI4_H