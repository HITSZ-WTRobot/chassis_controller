#include "chassis_omni4.h"
#include <string.h>

/**
 * 初始化四全向轮底盘
 * @param chassis 底盘实例
 * @param config 配置参数
 */
void Omni4_Init(Omni4_t* chassis, const Omni4_Config_t* config)
{
    memset(chassis, 0, sizeof(Omni4_t));

    // 转换单位：mm -> m
    chassis->wheel_radius = config->wheel_radius * 1e-3f;

    // 计算轮子到中心的半对角线距离 和 计算角度
    const float half_x = config->wheel_distance_x * 1e-3f * 0.5f;
    const float half_y = config->wheel_distance_y * 1e-3f * 0.5f;
    chassis->half_diag = sqrtf(half_x * half_x + half_y * half_y);
    chassis->angle     = atan(config->wheel_distance_y / config->wheel_distance_x);

    // 绑定电机
    chassis->wheel[OMNI4_WHEEL_FR] = config->wheel_front_right;
    chassis->wheel[OMNI4_WHEEL_FL] = config->wheel_front_left;
    chassis->wheel[OMNI4_WHEEL_RL] = config->wheel_rear_left;
    chassis->wheel[OMNI4_WHEEL_RR] = config->wheel_rear_right;

    // 角度归零
    for (size_t i = 0; i < OMNI4_WHEEL_MAX; i++)
    {
        Motor_ResetAngle(chassis->wheel[i]->motor_type, chassis->wheel[i]->motor);
    }
}

/**
 * 设置底盘速度（逆运动学解算）
 * @param vx x轴速度（车体前方为正）(m/s)
 * @param vy y轴速度（车体左侧为正）(m/s)
 * @param wz z轴角速度（逆时针为正）(deg/s)
 */
void Omni4_ApplyVelocity(Omni4_t* chassis, const float vx, const float vy, const float wz)
{
    // 转换角速度单位：deg/s -> rad/s
    const float wz_rad = DEG2RAD(wz);

    // 四个轮子的线速度计算（全向轮核心公式）
    const float v_fr = vx * cos(chassis->angle) + vy * sin(chassis->angle) +
                       chassis->half_diag * wz_rad; // 右前轮
    const float v_fl = -vx * cos(chassis->angle) + vy * sin(chassis->angle) +
                       chassis->half_diag * wz_rad; // 左前轮
    const float v_rl = -vx * cos(chassis->angle) - vy * sin(chassis->angle) +
                       chassis->half_diag * wz_rad; // 左后轮
    const float v_rr = vx * cos(chassis->angle) - vy * sin(chassis->angle) +
                       chassis->half_diag * wz_rad; // 右后轮

    // 转换为电机转速 (RPM)：v = ω*r → ω = v/r (rad/s)，再转RPM
    Motor_VelCtrl_SetRef(chassis->wheel[OMNI4_WHEEL_FR], RPS2RPM(v_fr / chassis->wheel_radius));
    Motor_VelCtrl_SetRef(chassis->wheel[OMNI4_WHEEL_FL], RPS2RPM(v_fl / chassis->wheel_radius));
    Motor_VelCtrl_SetRef(chassis->wheel[OMNI4_WHEEL_RL], RPS2RPM(v_rl / chassis->wheel_radius));
    Motor_VelCtrl_SetRef(chassis->wheel[OMNI4_WHEEL_RR], RPS2RPM(v_rr / chassis->wheel_radius));
}

/**
 * 底盘控制更新
 */
void Omni4_Update(const Omni4_t* chassis)
{
    for (size_t i = 0; i < OMNI4_WHEEL_MAX; i++)
    {
        Motor_VelCtrlUpdate(chassis->wheel[i]);
    }
}

/**
 * 正运动学解算 - 旋转角度
 */
float Omni4Forward_GetYaw(const Omni4_t* chassis)
{
    const float angle_fr = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_FR]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_FR]->motor);
    const float angle_fl = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_FL]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_FL]->motor);
    const float angle_rl = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_RL]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_RL]->motor);
    const float angle_rr = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_RR]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_RR]->motor);

    return -(chassis->wheel_radius / (4 * chassis->half_diag)) *
           DEG2RAD(angle_fr + angle_fl + angle_rl + angle_rr);
}

/**
 * 正运动学解算 - X方向位移
 */
float Omni4Forward_GetX(const Omni4_t* chassis)
{
    const float angle_fr = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_FR]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_FR]->motor);
    const float angle_fl = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_FL]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_FL]->motor);
    const float angle_rl = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_RL]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_RL]->motor);
    const float angle_rr = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_RR]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_RR]->motor);

    return (chassis->wheel_radius / 4) *
           DEG2RAD(angle_fr / cos(chassis->angle) - angle_fl / cos(chassis->angle) -
                   angle_rl / cos(chassis->angle) + angle_rr / cos(chassis->angle));
}

/**
 * 正运动学解算 - Y方向位移
 */
float Omni4Forward_GetY(const Omni4_t* chassis)
{
    const float angle_fr = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_FR]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_FR]->motor);
    const float angle_fl = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_FL]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_FL]->motor);
    const float angle_rl = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_RL]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_RL]->motor);
    const float angle_rr = Motor_GetAngle(chassis->wheel[OMNI4_WHEEL_RR]->motor_type,
                                          chassis->wheel[OMNI4_WHEEL_RR]->motor);

    return (chassis->wheel_radius / 4) *
           DEG2RAD(angle_fr / sin(chassis->angle) + angle_fl / sin(chassis->angle) -
                   angle_rl / sin(chassis->angle) - angle_rr / sin(chassis->angle));
}