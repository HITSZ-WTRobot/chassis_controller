/**
 * @file    chassis_if.h
 * @author  syhanjin
 * @date    2025-11-03
 * @brief   底盘统一接口
 */
#ifndef CHASSIS_IF_H
#define CHASSIS_IF_H

#define __CHASSIS_IF_VERSION__ "0.0.1"

#include "cmsis_os2.h"
#include "libs/pid_pd.h"

#include <stdbool.h>

#define CHASSIS_MECANUM4

#ifdef CHASSIS_MECANUM4
#    include "drivers/chassis_mecanum4.h"
#    define ChassisDriver_t        Mecanum4_t
#    define ChassisDriver_Config_t Mecanum4_Config_t
#endif

#if defined(CHASSIS_MECANUM4) != 1
#    error "There must be one and only one chassis type enabled at a time."
#endif

#define DEG2RAD(__DEG__) ((__DEG__) * (float) M_PI / 180.0f)

typedef struct
{
    float vx; ///< 指向车体前方 (unit: m/s)
    float vy; ///< 指向车体左侧 (unit: m/s)
    float wz; ///< 向上（逆时针）为正 (unit: deg/s)
} Chassis_Velocity_t;

typedef struct
{
    float x;   ///< 指向车体前方 (unit: m)
    float y;   ///< 指向车体左侧 (unit: m)
    float yaw; ///< 向上（逆时针）为正 (unit: deg)
} Chassis_Posture_t;

typedef struct
{
    Chassis_Posture_t posture;
    float             speed;
    float             omega;
} Chassis_PostureTarget_t;

typedef enum
{
    CHASSIS_VEL = 0U,
    CHASSIS_POS
} Chassis_CtrlMode_t;

typedef struct
{
    osMutexId_t lock;

    Chassis_CtrlMode_t ctrl_mode; ///< 控制模式
    struct
    {
        bool               target_in_world; ///< 速度是否相对于世界坐标系不变
        Chassis_Velocity_t in_world;        ///< 世界坐标系下速度
        Chassis_Velocity_t in_body;         ///< 车体坐标系下速度
    } velocity;

    struct
    {
        Chassis_PostureTarget_t target; ///< 目标位置
        /**
         * feedback_posture [x, y, yaw] - world.posture [x, y, yaw]
         * = posture.in_world [x, y, yaw]
         */
        Chassis_Posture_t in_world; ///< 车身位置（世界坐标系与车身坐标系的变换关系）

        struct
        {
            PD_t vx; ///< x 速度 PD 控制器
            PD_t vy; ///< y 速度 PD 控制器
            PD_t wz; ///< 角速度 PD 控制器
        } pd;
    } posture;

    struct
    {
        Chassis_Posture_t posture; ///< 世界坐标系位置（相对于反馈）
    } world;

    ChassisDriver_t driver; ///< 底盘驱动器

    /**
     * 车体运动反馈，用于闭环控制
     *
     * 指针指向反馈来源。为 NULL 表示没有数据来源，在必要情况下会通过运动学解算开环控制
     */
    struct
    {
        // 相对于车体
        float* vx; ///< 指向车体前方 (unit: m/s)
        float* vy; ///< 指向车体左侧 (unit: m/s)
        float* wz; ///< 车体角速度 (unit: deg/s)

        float* sx;  ///< x 方向里程计读数 (unit: m)
        float* sy;  ///< y 方向里程计读数 (unit: m)
        float* yaw; ///< 车体角度 (unit: deg)
    } feedback;

    struct
    {
        float sx;
        float sy;
        float yaw;
    } last_feedback;
} Chassis_t;

typedef struct
{
    ChassisDriver_Config_t driver; ///< 底盘驱动配置

    struct
    {
        struct
        {
            PD_Config_t vx; ///< x 速度 PD 控制器
            PD_Config_t vy; ///< y 速度 PD 控制器
            PD_Config_t wz; ///< 角速度 PD 控制器
        } pd;
    } posture;

    /**
     * 车体运动反馈，用于闭环控制
     *
     * 指针指向反馈来源。为 NULL 表示没有数据来源，在必要情况下会通过运动学解算开环控制
     */
    struct
    {
        // 相对于车体
        float* vx; ///< 指向车体前方 (unit: m/s)
        float* vy; ///< 指向车体左侧 (unit: m/s)
        float* wz; ///< 车体角速度 (unit: deg/s)

        float* sx;  ///< x 方向里程计读数 (unit: m)
        float* sy;  ///< y 方向里程计读数 (unit: m)
        float* yaw; ///< 车体角度 (unit: deg)
    } feedback_source;
} Chassis_Config_t;

void Chassis_Init(Chassis_t* chassis, const Chassis_Config_t* config);
void Chassis_Update(Chassis_t* chassis);

void Chassis_WorldVelocity2BodyVelocity(const Chassis_t*          chassis,
                                        const Chassis_Velocity_t* velocity_in_world,
                                        Chassis_Velocity_t*       velocity_in_body);
void Chassis_BodyVelocity2WorldVelocity(const Chassis_t*          chassis,
                                        const Chassis_Velocity_t* velocity_in_body,
                                        Chassis_Velocity_t*       velocity_in_world);

void Chassis_SetVelWorldFrame(Chassis_t*                chassis,
                              const Chassis_Velocity_t* world_velocity,
                              const bool                target_in_world);
void Chassis_SetVelBodyFrame(Chassis_t*                chassis,
                             const Chassis_Velocity_t* body_velocity,
                             const bool                target_in_world);
void Chassis_SetTargetPostureInBody(Chassis_t*                     chassis,
                                    const Chassis_PostureTarget_t* relative_target);

void Chassis_SetTargetPostureInWorld(Chassis_t*                     chassis,
                                     const Chassis_PostureTarget_t* absolute_target);
void Chassis_SetWorldFromCurrent(Chassis_t* chassis);

#endif // CHASSIS_IF_H
