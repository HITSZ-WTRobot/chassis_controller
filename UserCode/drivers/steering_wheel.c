/**
 * @file    steering_wheel.c
 * @author  syhanjin
 * @date    2025-10-03
 */
#include "steering_wheel.h"
#include <string.h>
#include "bsp/gpio_driver.h"

/**
 * 将轮子角度转化为电机角度
 *
 * @param wheel 轮子
 * @param input_angle 轮子相对于正前方的角度
 * @return 电机相对于零点（光电门）的角度
 */
static float get_steer_motor_angle(const SteeringWheel_t* wheel, const float input_angle)
{
    return input_angle + wheel->steer_offset;
}

/**
 * 光电门 EXTI 回调
 * @param gpio GPIO 引脚
 * @param counter 触发计数器
 * @param data 自定义数据
 */
void SteeringWheel_Photogate_Callback(const GPIO_t* gpio, uint32_t counter, void* data)
{
    SteeringWheel_t* wheel = data;
    switch (wheel->calib.state)
    {
    case STEER_CALIB_SEEK_GATE: // 第一次触发，降低速度
        Motor_VelCtrl_SetRef(wheel->calib.steer_motor, 5);
        wheel->calib.state = STEER_CALIB_FINE_CAPTURE;
        break;
    case STEER_CALIB_FINE_CAPTURE: // 第二次触发，记录角度并锁定
        __MOTOR_CTRL_DISABLE(wheel->calib.steer_motor);
        MotorCtrl_ResetAngle(wheel->steer_motor);
        Motor_PosCtrl_SetRef(wheel->steer_motor, get_steer_motor_angle(wheel, 0));
        // 使能驱动电机速度环和转向电机位置环
        __MOTOR_CTRL_ENABLE(wheel->steer_motor);
        __MOTOR_CTRL_ENABLE(wheel->drive_motor);
        wheel->calib.state = STEER_CALIB_DONE;
        break;
    default:;
    }
}

/**
 * 舵轮初始化
 * @param wheel 轮子
 * @param config 配置
 */
void SteeringWheel_Init(SteeringWheel_t* wheel, const SteeringWheel_Config_t* config)
{
    memset(wheel, 0, sizeof(SteeringWheel_t));

    wheel->drive_motor  = config->drive_motor;
    wheel->steer_motor  = config->steer_motor;
    wheel->steer_offset = config->steer_offset;

    wheel->calib.state                     = STEER_CALIB_IDLE;
    wheel->calib.steer_motor               = config->calib.steer_motor;
    wheel->calib.photogate                 = config->calib.photogate;
    wheel->calib.photogate_triggered_state = config->calib.photogate_triggered_state;

    __MOTOR_CTRL_DISABLE(wheel->calib.steer_motor);
    __MOTOR_CTRL_DISABLE(wheel->steer_motor);
    __MOTOR_CTRL_DISABLE(wheel->drive_motor);
}

/**
 * 舵轮控制更新函数
 *
 * 本函数在内部维护 motor 控制器
 * @param wheel 轮子
 */
void SteeringWheel_ControlUpdate(const SteeringWheel_t* wheel)
{
    Motor_PosCtrlUpdate(wheel->steer_motor);
    Motor_VelCtrlUpdate(wheel->drive_motor);
    Motor_VelCtrlUpdate(wheel->calib.steer_motor);
}

/**
 * 舵轮校准函数
 * @param wheel 轮子
 */
void SteeringWheel_StartCalibration(SteeringWheel_t* wheel)
{
    // 进入校准模式
    __MOTOR_CTRL_DISABLE(wheel->steer_motor);
    __MOTOR_CTRL_DISABLE(wheel->drive_motor);
    // 注册光电门回调
    GPIO_EXTI_RegisterCallback(&wheel->calib.photogate, SteeringWheel_Photogate_Callback, wheel);
    if (GPIO_ReadPin(&wheel->calib.photogate) == wheel->calib.photogate_triggered_state)
    {
        // 一开始就在光电门内，低速正转
        wheel->calib.state = STEER_CALIB_FINE_CAPTURE;
        Motor_VelCtrl_SetRef(wheel->calib.steer_motor, 5);
    }
    else
    {
        // 一开始不在光电门内，快速正转寻找光电门
        wheel->calib.state = STEER_CALIB_SEEK_GATE;
        Motor_VelCtrl_SetRef(wheel->calib.steer_motor, 30);
    }
    __MOTOR_CTRL_ENABLE(wheel->calib.steer_motor);
}

/**
 * 获得最短旋转路径速度
 * @param wheel 轮子
 * @param velocity 输入速度
 * @return 最短路径速度
 */
static void to_best_velocity(const SteeringWheel_t* wheel, SteeringWheel_Velocity_t* velocity)
{
    uint32_t round         = 0;                     // 当前角度对应的圈数（整圈计数）
    float    current_angle = wheel->velocity.angle; // 当前角度（可能大于360°或小于0°）

    /* 角度归一化，将当前角度调整到 [0, 360) 范围内，同时记录整圈数量 */
    while (current_angle > 360.0f)
        current_angle -= 360.0f, round++;
    while (current_angle < 0.0f)
        current_angle += 360.0f, round--;

    /* 将目标角度也归一化到 [0, 360) */
    while (velocity->angle > 360.0f)
        velocity->angle -= 360.0f;
    while (velocity->angle < 0.0f)
        velocity->angle += 360.0f;

    /* 计算目标角度相对于当前角度的差值 */
    const float delta = velocity->angle - current_angle;

    /*
     * 角度差分区间说明：
     * -360° ≤ delta < -270° : 当前角度比目标角度多约一圈 → 加一圈
     * -270° ≤ delta < -90°  : 反向驱动更短（加180°并反向速度）
     * -90° < delta ≤ 90°    : 最短路径，不需调整
     *  90° < delta ≤ 270°   : 反向驱动更短（减180°并反向速度）
     * 270° < delta ≤ 360°   : 当前角度比目标角度少约一圈 → 减一圈
     */
    if (-360.0f <= delta && delta < -270.0f)
    {
        velocity->angle += 360.0f;
    }
    else if (-270.0f <= delta && delta < -90.0f)
    {
        velocity->angle += 180.0f;
        velocity->speed = -velocity->speed;
    }
    // else if (-90.0f < delta && delta <= 90.0f) // do nothing
    else if (90.0f < delta && delta <= 270.0f)
    {
        velocity->angle -= 180.0f;
        velocity->speed = -velocity->speed;
    }
    else if (270.0f < delta && delta <= 360.0f)
    {
        velocity->angle -= 360.0f;
    }
    velocity->angle += (float) round * 360.0f;
}

/**
 * 设置舵轮速度
 *
 * 该函数根据当前舵轮角度，计算出输入速度向量的最优旋转路径（即转向角变化最小的路径）
 *
 * @param wheel 轮子
 * @param velocity 速度向量
 */
void SteeringWheel_SetVelocity(SteeringWheel_t* wheel, const SteeringWheel_Velocity_t* velocity)
{
    SteeringWheel_Velocity_t best_velocity = *velocity;
    to_best_velocity(wheel, &best_velocity);
    wheel->velocity = best_velocity;
    Motor_PosCtrl_SetRef(wheel->steer_motor, get_steer_motor_angle(wheel, wheel->velocity.angle));
    Motor_VelCtrl_SetRef(wheel->drive_motor, wheel->velocity.speed);
}
