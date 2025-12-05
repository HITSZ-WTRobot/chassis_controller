/**
 * @file    chassis_mecanum4.h
 * @author  syhanjin
 * @date    2025-10-17
 * @brief   the 4 wheels Mecanum chassis driver
 *
 * @note    降级到 drivers，因为此时本驱动已经不包括控制的功能，仅负责正逆运动学解算
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
#ifndef CHASSIS_MECANUM4_H
#define CHASSIS_MECANUM4_H

/**
 * Dependence: https://github.com/HITSZ-WTR2026/motor_drivers
 */
#include "interfaces/motor_if.h"

#include <math.h>
/**
 * rad/s to round/min
 * @param __RPS__ rad/s
 */
#define RPS2RPM(__RPS__) ((__RPS__) * 60.0f / (2 * 3.14159265358979323846f))

#define DEG2RAD(__DEG__) ((__DEG__) * (float) 3.14159265358979323846f / 180.0f)

typedef enum
{
    MECANUM4_WHEEL_FR = 0U, ///< 右前轮
    MECANUM4_WHEEL_FL,      ///< 左前轮
    MECANUM4_WHEEL_RL,      ///< 左后轮
    MECANUM4_WHEEL_RR,      ///< 右后轮
    MECANUM4_WHEEL_MAX
} Mecanum4_WheelType_t;

/**
 * @enum    Mecanum4_ChassisType_t
 * @brief   the 4 wheels Mecanum chassis layout type
 *
 * @attention 从地面侧向上看（即与地面接触侧的滚轮的构型）
 */
typedef enum
{
    MECANUM4_X_TYPE = 0U, ///< X 型布局
    MECANUM4_O_TYPE,      ///< O 型布局
} Mecanum4_ChassisType_t;

typedef struct
{
    float                  wheel_radius; ///< 轮子半径 (unit: m)
    float                  k_omega;      ///< O 型：半宽 + 半高；X 型：半宽 - 半高 (unit: m)
    Mecanum4_ChassisType_t chassis_type; ///< 底盘构型
    Motor_VelCtrl_t*       wheel[MECANUM4_WHEEL_MAX];
} Mecanum4_t;

typedef struct
{
    float                  wheel_radius;     ///< 轮子半径 (unit: mm)
    float                  wheel_distance_x; ///< 左右轮距 (unit: mm)
    float                  wheel_distance_y; ///< 前后轮距 (unit: mm)
    Mecanum4_ChassisType_t chassis_type;     ///< 底盘构型

    Motor_VelCtrl_t* wheel_front_right; ///< 右前方
    Motor_VelCtrl_t* wheel_front_left;  ///< 左前方
    Motor_VelCtrl_t* wheel_rear_left;   ///< 左后方
    Motor_VelCtrl_t* wheel_rear_right;  ///< 右后方
} Mecanum4_Config_t;

void Mecanum4_Init(Mecanum4_t* chassis, const Mecanum4_Config_t* config);
void Mecanum4_ApplyVelocity(Mecanum4_t* chassis, float vx, float vy, float wz);
void Mecanum4_Update(const Mecanum4_t* chassis);

// forward kinematics solutions
float Mecanum4Forward_GetYaw(const Mecanum4_t* chassis);
float Mecanum4Forward_GetX(const Mecanum4_t* chassis);
float Mecanum4Forward_GetY(const Mecanum4_t* chassis);

#endif // CHASSIS_MECANUM4_H
