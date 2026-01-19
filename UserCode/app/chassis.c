/**
 * @file    chassis.c
 * @author  syhanjin
 * @date    2025-12-02
 */
#include "chassis.h"

#include "device.h"
#include "interfaces/chassis_if.h"

#ifdef __cplusplus
extern "C"
{
#endif
static MotorPID_Config_t motor_wheel_vel_pid = { //
    .Kp             = 45.0f,
    .Ki             = 0.07f,
    .Kd             = 5.00f,
    .abs_output_max = 8000.0f
};
Motor_VelCtrl_t motor_vel_ctrl[4];
const Chassis_Config_t chassis_config = {
        .driver = {
            .chassis_type      = MECANUM4_O_TYPE,    ///< 底盘构型
            .wheel_radius      = 77.0f,              ///< 轮子半径 (unit: mm)
            .wheel_distance_x  = 504.60f,            ///< 左右轮距 (unit: mm)
            .wheel_distance_y  = 579.78f,            ///< 前后轮距 (unit: mm)
            .wheel_front_right = &motor_vel_ctrl[0], ///< 右前方
            .wheel_front_left  = &motor_vel_ctrl[1], ///< 左前方
            .wheel_rear_left   = &motor_vel_ctrl[2], ///< 左后方
            .wheel_rear_right  = &motor_vel_ctrl[3], ///< 右后方
    },
    .traj_update_interval = 0.005f,
    .posture = {
        .error_pd = {
            .vx = {.Kp = 2, .Kd = 1, .abs_output_max = 0.1f},
            .vy = {.Kp = 2, .Kd = 1, .abs_output_max = 0.1f},
            .wz = {.Kp = 2, .Kd = 1, .abs_output_max = 25.0f},
        }
    },
    .feedback_source = {
            // .yaw = &sensor_gyro_yaw.yaw,
        },
    };
Chassis_t chassis;

void APP_Chassis_InitBeforeUpdate()
{
    // 初始化控制器
    for (size_t i = 0; i < 4; i++)
        Motor_VelCtrl_Init(&motor_vel_ctrl[i],
                           &(Motor_VelCtrlConfig_t) { .motor_type = MOTOR_TYPE_DJI,
                                                      .motor      = &motor_wheel[i],
                                                      .pid        = motor_wheel_vel_pid });
    // 初始化底盘
    Chassis_Init(&chassis, &chassis_config);
}
void APP_Chassis_Init()
{
    Chassis_SetWorldFromCurrent(&chassis);
}

#ifdef __cplusplus
}
#endif