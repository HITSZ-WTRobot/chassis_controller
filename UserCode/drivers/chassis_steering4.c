/**
 * @file    chassis_steering4.c
 * @author  syhanjin
 * @date    2025-10-03
 */
#include "chassis_steering4.h"

#include <string.h>

struct
{
    float kx, ky;
} wheel_position[STEERING4_WHEEL_MAX] = { { 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 } };

static void get_wheel_position(const Steering4_t*          chassis,
                               const Steering4_WheelType_t wheel_type,
                               float*                      xi,
                               float*                      yi)
{
    *xi = wheel_position[wheel_type].kx * chassis->half_width;
    *yi = wheel_position[wheel_type].ky * chassis->half_height;
}

/**
 * 初始化四轮舵轮底盘
 * @param chassis 底盘
 * @param config 底盘配置
 */
void Steering4_Init(Steering4_t* chassis, const Steering4_Config_t* config)
{
    memset(chassis, 0, sizeof(Steering4_t));

    chassis->half_width  = config->width * 0.5f;
    chassis->half_height = config->height * 0.5f;
    chassis->inv_l2      = 1.0f / (chassis->half_width * chassis->half_width +
                              chassis->half_height * chassis->half_height);

    chassis->radius = config->radius * 1e-3f; // mm to m
    chassis->k_rpm  = 1.0f / (chassis->radius * 3.14159265358979323846f * 2) * 60.0f;

    SteeringWheel_Init(&chassis->wheel[STEERING4_WHEEL_FR], &config->wheel_front_right);
    SteeringWheel_Init(&chassis->wheel[STEERING4_WHEEL_FL], &config->wheel_front_left);
    SteeringWheel_Init(&chassis->wheel[STEERING4_WHEEL_RL], &config->wheel_rear_left);
    SteeringWheel_Init(&chassis->wheel[STEERING4_WHEEL_RR], &config->wheel_rear_right);
}

/**
 * 底盘控制更新函数
 *
 * 本函数自动处理控制逻辑，并依序调用每个轮子的 PID 更新函数
 *
 * @note 推荐控制调用频率 1kHz，调用频率将会影响轮子的 PID 参数
 * @param chassis 底盘
 */
void Steering4_Update(Steering4_t* chassis)
{
    if (!chassis->is_calibrated)
    { // 检查校准状态
        bool is_calibrated = true;
        for (size_t i = 0; i < STEERING4_WHEEL_MAX; i++)
            if (chassis->wheel[i].calib.state != STEER_CALIB_DONE)
            {
                is_calibrated = false;
                break;
            }
        chassis->is_calibrated = is_calibrated;
    }

    // 更新控制
    for (size_t i = 0; i < STEERING4_WHEEL_MAX; i++)
    {
        SteeringWheel_ControlUpdate(&chassis->wheel[i]);
    }
}

/**
 * 开启底盘舵轮校准
 *
 * @note 校准过程中舵轮会正转寻找光电门，定位零点
 * @param chassis 底盘
 */
void Steering4_StartCalibration(Steering4_t* chassis)
{
    for (size_t i = 0; i < STEERING4_WHEEL_MAX; i++)
    {
        SteeringWheel_StartCalibration(&chassis->wheel[i]);
    }
}

/**
 * 设置底盘速度
 *
 * @param chassis 底盘
 * @param vx
 * @param vy
 * @param wz
 */
void Steering4_ApplyVelocity(Steering4_t* chassis, const float vx, const float vy, const float wz)
{
    if (!chassis->is_calibrated)
        // 必须先校准才能设置速度
        return;

    for (size_t i = 0; i < STEERING4_WHEEL_MAX; i++)
    {
        float xi = 0, yi = 0;
        get_wheel_position(chassis, i, &xi, &yi);
        const float vxi       = vx - yi * wz;
        const float vyi       = vy + xi * wz;
        const float speed_rpm = chassis->k_rpm * sqrtf(vxi * vxi + vyi * vyi);
        if (fabsf(speed_rpm) < 1e-6f)
        { // 速度为零
            SteeringWheel_SetVelocity(&chassis->wheel[i],
                                      &(SteeringWheel_Velocity_t) {
                                              SteeringWheel_GetSteerAngle(&chassis->wheel[i]), 0 });
        }
        else
        {
            const float angle = RAD2DEG(atan2f(vyi, vxi));
            SteeringWheel_SetVelocity(&chassis->wheel[i],
                                      &(SteeringWheel_Velocity_t) { angle, speed_rpm });
        }
    }
}

static void steering4_velocity_forward(const Steering4_t* chassis, float* vx, float* vy, float* wz)
{
    *vx = 0, *vy = 0, *wz = 0;
    for (size_t i = 0; i < STEERING4_WHEEL_MAX; i++)
    {
        float xi = 0, yi = 0;
        get_wheel_position(chassis, i, &xi, &yi);
        const float steer_angle  = SteeringWheel_GetSteerAngle(&chassis->wheel[i]);
        const float driver_speed = SteeringWheel_GetDriveSpeed(&chassis->wheel[i]) / chassis->k_rpm;
        const float steer_angle_rad = DEG2RAD(steer_angle);
        const float sin_theta       = sinf(steer_angle_rad);
        const float cos_theta       = cosf(steer_angle_rad);
        *vx += driver_speed * cos_theta;
        *vy += driver_speed * sin_theta;
        *wz += -yi * cos_theta + xi * sin_theta;
    }
    *vx *= 0.25f;
    *vy *= 0.25f;
    *wz *= chassis->inv_l2;
}

static void steering4_position_forward(const Steering4_t* chassis, float* x, float* y, float* z)
{
    *x = 0, *y = 0, *z = 0;
    for (size_t i = 0; i < STEERING4_WHEEL_MAX; i++)
    {
        float xi = 0, yi = 0;
        get_wheel_position(chassis, i, &xi, &yi);
        const float sin_theta = sinf(chassis->wheel[i].velocity.angle);
        const float cos_theta = cosf(chassis->wheel[i].velocity.angle);
        *x += chassis->wheel[i].velocity.speed * cos_theta;
        *y += chassis->wheel[i].velocity.speed * sin_theta;
        *z += -yi * cos_theta + xi * sin_theta;
    }
    *x *= 0.25f;
    *y *= 0.25f;
    *z *= chassis->inv_l2;
}

// TODO: 优化此处函数调用的性能

float Steering4Forward_GetYaw(const Steering4_t* chassis) {}
float Steering4Forward_GetX(const Steering4_t* chassis) {}
float Steering4Forward_GetY(const Steering4_t* chassis) {}
float Steering4Forward_GetWz(const Steering4_t* chassis) {}
float Steering4Forward_GetVx(const Steering4_t* chassis) {}
float Steering4Forward_GetVy(const Steering4_t* chassis) {}
