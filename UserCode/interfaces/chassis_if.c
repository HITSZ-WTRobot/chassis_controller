/**
 * @file    chassis_if.c
 * @author  syhanjin
 * @date    2025-11-03
 */
#include "chassis_if.h"
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CHASSIS_MECANUM4
#    define ChassisDriver_Init          Mecanum4_Init
#    define ChassisDriver_ApplyVelocity Mecanum4_ApplyVelocity
#    define ChassisDriver_Update        Mecanum4_Update
#    define ChassisForward_GetYaw       Mecanum4Forward_GetYaw
#    define ChassisForward_GetX         Mecanum4Forward_GetX
#    define ChassisForward_GetY         Mecanum4Forward_GetY
#endif

/**
 * 将世界坐标系下的速度变换为车身坐标系下的速度
 *
 * @param chassis 底盘
 * @param velocity_in_world 世界坐标系下的速度
 * @param velocity_in_body 变换后的车身坐标系下的速度
 */
void Chassis_WorldVelocity2BodyVelocity(const Chassis_t*          chassis,
                                        const Chassis_Velocity_t* velocity_in_world,
                                        Chassis_Velocity_t*       velocity_in_body)
{
    const float _sin_yaw = sinf(DEG2RAD(-chassis->posture.in_world.yaw)),
                _cos_yaw = cosf(DEG2RAD(-chassis->posture.in_world.yaw));

    velocity_in_body->vx = velocity_in_world->vx * _cos_yaw - velocity_in_world->vy * _sin_yaw;
    velocity_in_body->vy = velocity_in_world->vx * _sin_yaw + velocity_in_world->vy * _cos_yaw;
    velocity_in_body->wz = velocity_in_world->wz;
}

/**
 * 将世界坐标系下的位姿变换到车身坐标系
 *
 * @param chassis 底盘
 * @param posture_in_world 世界坐标系下的位姿
 * @param posture_in_body 变换后的车身坐标系下的位姿
 */
void Chassis_WorldPosture2BodyPosture(const Chassis_t*         chassis,
                                      const Chassis_Posture_t* posture_in_world,
                                      Chassis_Posture_t*       posture_in_body)
{
    const float _sin_yaw = sinf(DEG2RAD(-chassis->posture.in_world.yaw)),
                _cos_yaw = cosf(DEG2RAD(-chassis->posture.in_world.yaw));

    const float tx = posture_in_world->x - chassis->posture.in_world.x;
    const float ty = posture_in_world->y - chassis->posture.in_world.y;

    posture_in_body->x   = tx * _cos_yaw - ty * _sin_yaw;
    posture_in_body->y   = tx * _sin_yaw + ty * _cos_yaw;
    posture_in_body->yaw = posture_in_world->yaw - chassis->posture.in_world.yaw;
}

/**
 * 将车身坐标系下的速度变换为世界坐标系下的速度
 *
 * @param chassis 底盘
 * @param velocity_in_body 车身坐标系下的速度
 * @param velocity_in_world 变换后的世界坐标系下的速度
 */
void Chassis_BodyVelocity2WorldVelocity(const Chassis_t*          chassis,
                                        const Chassis_Velocity_t* velocity_in_body,
                                        Chassis_Velocity_t*       velocity_in_world)
{
    const float sin_yaw = sinf(DEG2RAD(chassis->posture.in_world.yaw)),
                cos_yaw = cosf(DEG2RAD(chassis->posture.in_world.yaw));

    velocity_in_world->vx = velocity_in_body->vx * cos_yaw - velocity_in_body->vy * sin_yaw;
    velocity_in_world->vy = velocity_in_body->vx * sin_yaw + velocity_in_body->vy * cos_yaw;
    velocity_in_world->wz = velocity_in_body->wz;
}

/**
 * 将车身坐标系下的位姿变换到世界坐标系
 *
 * @param chassis 底盘
 * @param posture_in_body 车身坐标系下的位姿
 * @param posture_in_world 变换后的世界坐标系下的位姿
 */
void Chassis_BodyPosture2WorldPosture(const Chassis_t*         chassis,
                                      const Chassis_Posture_t* posture_in_body,
                                      Chassis_Posture_t*       posture_in_world)
{
    const float sin_yaw = sinf(DEG2RAD(chassis->posture.in_world.yaw)),
                cos_yaw = cosf(DEG2RAD(chassis->posture.in_world.yaw));

    const float tx = posture_in_body->x + chassis->posture.in_world.x;
    const float ty = posture_in_body->y + chassis->posture.in_world.y;

    posture_in_world->x   = tx * cos_yaw - ty * sin_yaw;
    posture_in_world->y   = tx * sin_yaw + ty * cos_yaw;
    posture_in_world->yaw = posture_in_body->yaw + chassis->posture.in_world.yaw;
}

/**
 * 底盘初始化
 *
 * @note 初始化之后会自动保持静止
 * @param chassis 底盘
 * @param config 配置
 */
void Chassis_Init(Chassis_t* chassis, const Chassis_Config_t* config)
{
    ChassisDriver_Init(&chassis->driver, &config->driver);

    chassis->feedback.vx = config->feedback_source.vx;
    chassis->feedback.vy = config->feedback_source.vy;
    chassis->feedback.wz = config->feedback_source.wz;
    chassis->feedback.sx = config->feedback_source.sx;
    chassis->feedback.sy = config->feedback_source.sy;
    chassis->feedback.yaw = config->feedback_source.yaw;

    chassis->last_feedback.sx = 0;
    chassis->last_feedback.sy = 0;
    chassis->last_feedback.yaw = 0;

    // 初始化位置 PD 控制器
    PD_Init(&chassis->posture.pd.vx, &config->posture.pd.vx);
    PD_Init(&chassis->posture.pd.vy, &config->posture.pd.vy);
    PD_Init(&chassis->posture.pd.wz, &config->posture.pd.wz);

    chassis->ctrl_mode = CHASSIS_VEL;
    ChassisDriver_ApplyVelocity(&chassis->driver, 0, 0, 0);

    chassis->lock = osMutexNew(NULL);
}

static void update_chassis_posture(Chassis_t* chassis)
{
    const float sx  = chassis->feedback.sx != NULL ? *chassis->feedback.sx
                                                   : ChassisForward_GetX(&chassis->driver);
    const float sy  = chassis->feedback.sy != NULL ? *chassis->feedback.sy
                                                   : ChassisForward_GetY(&chassis->driver);
    const float yaw = chassis->feedback.yaw != NULL ? *chassis->feedback.yaw
                                                    : ChassisForward_GetYaw(&chassis->driver);

    const float dx               = sx - chassis->last_feedback.sx;
    const float dy               = sy - chassis->last_feedback.sy;
    const float ave_yaw          = (yaw + chassis->last_feedback.yaw) * 0.5f;
    const float ave_yaw_in_world = ave_yaw - chassis->world.posture.yaw;

    chassis->last_feedback.sx  = sx;
    chassis->last_feedback.sy  = sy;
    chassis->last_feedback.yaw = yaw;

    chassis->posture.in_world.x += dx * cosf(DEG2RAD(ave_yaw_in_world)) -
                                   dy * sinf(DEG2RAD(ave_yaw_in_world));
    chassis->posture.in_world.y += dx * sinf(DEG2RAD(ave_yaw_in_world)) +
                                   dy * cosf(DEG2RAD(ave_yaw_in_world));
    chassis->posture.in_world.yaw = yaw - chassis->world.posture.yaw;
}

static void update_chassis_velocity_control(Chassis_t* chassis)
{
    if (chassis->velocity.target_in_world)
    { // 如果基于世界坐标计算速度，则需要转为车身坐标系，并应用到底盘驱动器
        Chassis_WorldVelocity2BodyVelocity(chassis,
                                           &chassis->velocity.in_world,
                                           &chassis->velocity.in_body);
        // 进行修正
        const float              beta     = DEG2RAD(0.5f * chassis->velocity.in_body.wz * 1e-3f);
        const float              cot_beta = 1.0f / tanf(beta);
        const Chassis_Velocity_t temp_velocity = {
            .vx = beta * (chassis->velocity.in_body.vx * cot_beta + chassis->velocity.in_body.vy),
            .vy = beta * (chassis->velocity.in_body.vy * cot_beta - chassis->velocity.in_body.vx),
            .wz = chassis->velocity.in_body.wz
        };
        ChassisDriver_ApplyVelocity(&chassis->driver,
                                    temp_velocity.vx,
                                    temp_velocity.vy,
                                    temp_velocity.wz);
    }
    else
    {
        // 否则基于机体坐标计算速度，则不需要变动
    }
}

void update_chassis_position_control(Chassis_t* chassis)
{
    // 计算与目标的相对位置
Chassis_Posture_t relative_posture;
    Chassis_WorldPosture2BodyPosture(chassis, &chassis->posture.target.posture, &relative_posture);
    // 计算速度
    chassis->posture.pd.vx.ref = relative_posture.x;
    chassis->posture.pd.vy.ref = relative_posture.y;
    chassis->posture.pd.wz.ref = relative_posture.yaw;
    chassis->posture.pd.vx.fdb = 0;
    chassis->posture.pd.vy.fdb = 0;
    chassis->posture.pd.wz.fdb = 0;
    PD_Calculate(&chassis->posture.pd.vx);
    PD_Calculate(&chassis->posture.pd.vy);
    PD_Calculate(&chassis->posture.pd.wz);
    Chassis_Velocity_t body_velocity = {
        .vx = chassis->posture.pd.vx.output,
        .vy = chassis->posture.pd.vy.output,
        .wz = chassis->posture.pd.wz.output
    };

    const float speed = sqrtf(body_velocity.vx * body_velocity.vx +
                              body_velocity.vy * body_velocity.vy);
    if (speed > chassis->posture.target.speed)
    {
        const float ratio = chassis->posture.target.speed / speed;
        body_velocity.vx *= ratio;
        body_velocity.vy *= ratio;
    }
    // if (fabsf(body_velocity.wz) > chassis->posture.target.omega)
    // {
    //     const float ratio = chassis->posture.target.omega / fabsf(body_velocity.wz);
    //     body_velocity.wz *= ratio;
    // }
    chassis->velocity.in_body.vx = body_velocity.vx;
    chassis->velocity.in_body.vy = body_velocity.vy;
    chassis->velocity.in_body.wz = body_velocity.wz;
    ChassisDriver_ApplyVelocity(&chassis->driver,
                                body_velocity.vx,
                                body_velocity.vy,
                                body_velocity.wz);
}

void Chassis_Update(Chassis_t* chassis)
{
    update_chassis_posture(chassis);
    if (chassis->ctrl_mode == CHASSIS_VEL)
        update_chassis_velocity_control(chassis);
    else if (chassis->ctrl_mode == CHASSIS_POS)
        update_chassis_position_control(chassis);

    // 更新底盘驱动器
    ChassisDriver_Update(&chassis->driver);
}


    void Chassis_SetTargetPostureInWorld(Chassis_t*                     chassis,
                                         const Chassis_PostureTarget_t* absolute_target)
{
    chassis->posture.target.posture.x   = absolute_target->posture.x;
    chassis->posture.target.posture.y   = absolute_target->posture.y;
    chassis->posture.target.posture.yaw = absolute_target->posture.yaw;
    chassis->posture.target.speed       = absolute_target->speed;
    chassis->posture.target.omega       = absolute_target->omega;
    chassis->ctrl_mode                  = CHASSIS_POS;
    chassis->posture.pd.vx.abs_output_max = absolute_target->speed;
    chassis->posture.pd.vy.abs_output_max = absolute_target->speed;
    chassis->posture.pd.wz.abs_output_max = absolute_target->omega;
}

void Chassis_SetTargetPostureInBody(Chassis_t*                     chassis,
                                    const Chassis_PostureTarget_t* relative_target)
{
    Chassis_PostureTarget_t absolute_target;
    Chassis_BodyPosture2WorldPosture(chassis, &relative_target->posture, (Chassis_Posture_t*)&absolute_target);
    absolute_target.speed = relative_target->speed;
    absolute_target.omega = relative_target->omega;
    Chassis_SetTargetPostureInWorld(chassis, &absolute_target);
}

/**
 * 基于世界坐标系设置速度
 *
 * @param chassis 底盘对象
 * @param world_velocity 在世界坐标系下的速度
 * @param target_in_world 是否以世界坐标系为参考系保持速度不变.
 *                        为 true 则车体会相对于世界坐标系保持速度不变;
 *                        为 false 则相对于车身坐标系保持不变.
 *                        没做出来，暂时去掉本参数
 */
void Chassis_SetVelWorldFrame(Chassis_t*                chassis,
                              const Chassis_Velocity_t* world_velocity,
                              const bool                target_in_world)
{
    chassis->velocity.target_in_world = target_in_world;
    chassis->velocity.in_world.vx     = world_velocity->vx;
    chassis->velocity.in_world.vy     = world_velocity->vy;
    chassis->velocity.in_world.wz     = world_velocity->wz;
    Chassis_WorldVelocity2BodyVelocity(chassis, world_velocity, &chassis->velocity.in_body);
    ChassisDriver_ApplyVelocity(&chassis->driver,
                                chassis->velocity.in_body.vx,
                                chassis->velocity.in_body.vy,
                                chassis->velocity.in_body.wz);
    chassis->ctrl_mode = CHASSIS_VEL;
}

/**
 * 基于车身坐标系设置速度
 *
 * @param chassis 底盘对象
 * @param body_velocity 在世界坐标系下的速度
 * @param target_in_world 是否以世界坐标系为参考系保持速度不变.
 *                        为 true 则车体会相对于世界坐标系保持速度不变;
 *                        为 false 则相对于车身坐标系保持不变.
 *                        没做出来，暂时去掉本参数
 */
void Chassis_SetVelBodyFrame(Chassis_t*                chassis,
                             const Chassis_Velocity_t* body_velocity,
                             const bool                target_in_world)
{
    chassis->velocity.target_in_world = target_in_world;
    chassis->velocity.in_body.vx      = body_velocity->vx;
    chassis->velocity.in_body.vy      = body_velocity->vy;
    chassis->velocity.in_body.wz      = body_velocity->wz;
    Chassis_BodyVelocity2WorldVelocity(chassis, body_velocity, &chassis->velocity.in_world);
    ChassisDriver_ApplyVelocity(&chassis->driver,
                                chassis->velocity.in_body.vx,
                                chassis->velocity.in_body.vy,
                                chassis->velocity.in_body.wz);
    chassis->ctrl_mode = CHASSIS_VEL;
}

/**
 * 将当前车身坐标系设置为世界坐标系
 * @param chassis 底盘对象
 */
void Chassis_SetWorldFromCurrent(Chassis_t* chassis)
{
    chassis->world.posture.x += chassis->posture.in_world.x;
    chassis->world.posture.y += chassis->posture.in_world.y;
    chassis->world.posture.yaw += chassis->posture.in_world.yaw;
    chassis->posture.in_world.x   = 0.0f;
    chassis->posture.in_world.y   = 0.0f;
    chassis->posture.in_world.yaw = 0.0f;
    chassis->velocity.in_world    = chassis->velocity.in_body;
}

#ifdef __cplusplus
}
#endif