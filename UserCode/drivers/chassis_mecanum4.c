/**
 * @file    chassis_mecanum4.c
 * @author  syhanjin
 * @date    2025-10-17
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
#include "chassis_mecanum4.h"
#include <string.h>

/**
 * 初始化四轮麦轮底盘
 *
 * @note 轮子旋转的正方向为：使用普通轮子时让车向前移动的旋转方向
 * @param chassis 底盘
 * @param config 底盘配置
 */
void Mecanum4_Init(Mecanum4_t* chassis, const Mecanum4_Config_t* config)
{
    memset(chassis, 0, sizeof(Mecanum4_t));

    chassis->chassis_type = config->chassis_type;

    if (config->chassis_type == MECANUM4_O_TYPE)
        chassis->k_omega = config->wheel_distance_x * 1e-3f * 0.5f +
                           config->wheel_distance_y * 1e-3f * 0.5f;
    else if (config->chassis_type == MECANUM4_X_TYPE)
        chassis->k_omega = config->wheel_distance_x * 1e-3f * 0.5f -
                           config->wheel_distance_y * 1e-3f * 0.5f;

    chassis->wheel_radius = config->wheel_radius * 1e-3f;

    chassis->wheel[MECANUM4_WHEEL_FR] = config->wheel_front_right;
    chassis->wheel[MECANUM4_WHEEL_FL] = config->wheel_front_left;
    chassis->wheel[MECANUM4_WHEEL_RL] = config->wheel_rear_left;
    chassis->wheel[MECANUM4_WHEEL_RR] = config->wheel_rear_right;

    // 进行一次角度归零，用于计算正解
    for (size_t i = 0; i < MECANUM4_WHEEL_MAX; i++)
        Motor_ResetAngle(chassis->wheel[i]->motor_type, chassis->wheel[i]->motor);
}

/**
 * 设置底盘速度
 * @param chassis 底盘
 * @param vx x轴速度 指向车体前方 (unit: m/s)
 * @param vy y轴速度 指向车体左侧 (unit: m/s)
 * @param wz z轴速度 向上（逆时针）为正 (unit: deg/s)
 */
void Mecanum4_ApplyVelocity(Mecanum4_t* chassis, const float vx, const float vy, const float wz)
{
    if (chassis->chassis_type == MECANUM4_O_TYPE)
    {
        /** Mecanum4 O 型运动学解算
         * w_fr = (+ vx + vy + (w + h) * ω) / r
         * w_fl = (+ vx - vy - (w + h) * ω) / r
         * w_rl = (+ vx + vy - (w + h) * ω) / r
         * w_rr = (+ vx - vy + (w + h) * ω) / r
         */
        Motor_VelCtrl_SetRef(chassis->wheel[MECANUM4_WHEEL_FR],
                             RPS2RPM((vx + vy + chassis->k_omega * DEG2RAD(wz)) /
                                     chassis->wheel_radius));
        Motor_VelCtrl_SetRef(chassis->wheel[MECANUM4_WHEEL_FL],
                             RPS2RPM((vx - vy - chassis->k_omega * DEG2RAD(wz)) /
                                     chassis->wheel_radius));
        Motor_VelCtrl_SetRef(chassis->wheel[MECANUM4_WHEEL_RL],
                             RPS2RPM((vx + vy - chassis->k_omega * DEG2RAD(wz)) /
                                     chassis->wheel_radius));
        Motor_VelCtrl_SetRef(chassis->wheel[MECANUM4_WHEEL_RR],
                             RPS2RPM((vx - vy + chassis->k_omega * DEG2RAD(wz)) /
                                     chassis->wheel_radius));
    }
    else if (chassis->chassis_type == MECANUM4_X_TYPE)
    {
        /** Mecanum4 X 型运动学解算
         * w_fr = (+ vx - vy + (w - h) * ω) / r
         * w_fl = (+ vx + vy - (w - h) * ω) / r
         * w_rl = (+ vx - vy - (w - h) * ω) / r
         * w_rr = (+ vx + vy + (w - h) * ω) / r
         */
        Motor_VelCtrl_SetRef(chassis->wheel[MECANUM4_WHEEL_FR],
                             RPS2RPM((vx - vy + chassis->k_omega * DEG2RAD(wz)) /
                                     chassis->wheel_radius));
        Motor_VelCtrl_SetRef(chassis->wheel[MECANUM4_WHEEL_FL],
                             RPS2RPM((vx + vy - chassis->k_omega * DEG2RAD(wz)) /
                                     chassis->wheel_radius));
        Motor_VelCtrl_SetRef(chassis->wheel[MECANUM4_WHEEL_RL],
                             RPS2RPM((vx - vy - chassis->k_omega * DEG2RAD(wz)) /
                                     chassis->wheel_radius));
        Motor_VelCtrl_SetRef(chassis->wheel[MECANUM4_WHEEL_RR],
                             RPS2RPM((vx + vy + chassis->k_omega * DEG2RAD(wz)) /
                                     chassis->wheel_radius));
    }
}

/**
 * 底盘控制更新函数
 *
 * 本函数自动处理控制逻辑，并依序调用每个轮子的 PID 更新函数
 *
 * @note 推荐控制调用频率 1kHz，调用频率将会影响轮子的 PID 参数
 * @param chassis 底盘
 */
void Mecanum4_Update(const Mecanum4_t* chassis)
{
    for (size_t i = 0; i < MECANUM4_WHEEL_MAX; i++)
    {
        Motor_VelCtrlUpdate(chassis->wheel[i]);
    }
}

/* Mecanum4 forward kinematics solutions */
float Mecanum4Forward_GetYaw(const Mecanum4_t* chassis)
{
    // TODO: 优化 motor_if 的 GetAngle 函数
    if (chassis->chassis_type == MECANUM4_O_TYPE)
        return chassis->wheel_radius / (4 * chassis->k_omega) *
               (Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FR]->motor_type,
                               chassis->wheel[MECANUM4_WHEEL_FR]->motor) -
                Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FL]->motor_type,
                               chassis->wheel[MECANUM4_WHEEL_FL]->motor) +
                Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RR]->motor_type,
                               chassis->wheel[MECANUM4_WHEEL_RR]->motor) -
                Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RL]->motor_type,
                               chassis->wheel[MECANUM4_WHEEL_RL]->motor));
    if (chassis->chassis_type == MECANUM4_X_TYPE)
        return chassis->wheel_radius / (4 * chassis->k_omega) *
               (Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FR]->motor_type,
                               chassis->wheel[MECANUM4_WHEEL_FR]->motor) -
                Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FL]->motor_type,
                               chassis->wheel[MECANUM4_WHEEL_FL]->motor) -
                Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RR]->motor_type,
                               chassis->wheel[MECANUM4_WHEEL_RR]->motor) +
                Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RL]->motor_type,
                               chassis->wheel[MECANUM4_WHEEL_RL]->motor));

    return 0;
}

float Mecanum4Forward_GetX(const Mecanum4_t* chassis)
{
    return chassis->wheel_radius * 0.25f *
           DEG2RAD(Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FR]->motor_type,
                                  chassis->wheel[MECANUM4_WHEEL_FR]->motor) +
                   Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FL]->motor_type,
                                  chassis->wheel[MECANUM4_WHEEL_FL]->motor) +
                   Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RR]->motor_type,
                                  chassis->wheel[MECANUM4_WHEEL_RR]->motor) +
                   Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RL]->motor_type,
                                  chassis->wheel[MECANUM4_WHEEL_RL]->motor));
}

float Mecanum4Forward_GetY(const Mecanum4_t* chassis)
{
    if (chassis->chassis_type == MECANUM4_O_TYPE)
        return chassis->wheel_radius * 0.25f *
               DEG2RAD(Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FR]->motor_type,
                                      chassis->wheel[MECANUM4_WHEEL_FR]->motor) -
                       Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FL]->motor_type,
                                      chassis->wheel[MECANUM4_WHEEL_FL]->motor) -
                       Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RR]->motor_type,
                                      chassis->wheel[MECANUM4_WHEEL_RR]->motor) +
                       Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RL]->motor_type,
                                      chassis->wheel[MECANUM4_WHEEL_RL]->motor));
    if (chassis->chassis_type == MECANUM4_X_TYPE)
        return chassis->wheel_radius / (4 * chassis->k_omega) *
               DEG2RAD(-Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FR]->motor_type,
                                       chassis->wheel[MECANUM4_WHEEL_FR]->motor) +
                       Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_FL]->motor_type,
                                      chassis->wheel[MECANUM4_WHEEL_FL]->motor) +
                       Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RR]->motor_type,
                                      chassis->wheel[MECANUM4_WHEEL_RR]->motor) -
                       Motor_GetAngle(chassis->wheel[MECANUM4_WHEEL_RL]->motor_type,
                                      chassis->wheel[MECANUM4_WHEEL_RL]->motor));

    return 0;
}