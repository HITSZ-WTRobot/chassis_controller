/**
 * @file    chassis_if.c
 * @author  syhanjin
 * @date    2025-11-03
 */
#include "chassis_if.h"
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef CHASSIS_MECANUM4
#    define ChassisDriver_Init          Mecanum4_Init
#    define ChassisDriver_ApplyVelocity Mecanum4_ApplyVelocity
#    define ChassisDriver_Update        Mecanum4_Update
#    define ChassisForward_GetYaw       Mecanum4Forward_GetYaw
#    define ChassisForward_GetX         Mecanum4Forward_GetX
#    define ChassisForward_GetY         Mecanum4Forward_GetY
#endif
#ifdef CHASSIS_OMNI4
#    define ChassisDriver_Init          Omni4_Init
#    define ChassisDriver_ApplyVelocity Omni4_ApplyVelocity
#    define ChassisDriver_Update        Omni4_Update
#    define ChassisForward_GetYaw       Omni4Forward_GetYaw
#    define ChassisForward_GetX         Omni4Forward_GetX
#    define ChassisForward_GetY         Omni4Forward_GetY
#endif

static uint32_t isr_lock()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static void isr_unlock(uint32_t primask)
{
    __DSB();
    __ISB();
    __set_PRIMASK(primask);
}

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

    posture_in_world->x = posture_in_body->x * cos_yaw - posture_in_body->y * sin_yaw +
                          chassis->posture.in_world.x;
    posture_in_world->y = posture_in_body->x * sin_yaw + posture_in_body->y * cos_yaw +
                          chassis->posture.in_world.y;
    posture_in_world->yaw = posture_in_body->yaw + chassis->posture.in_world.yaw;
}

/**
 * 底盘初始化
 *
 * @attention config 对象不要在栈上初始化，可能会存在随机值，导致野指针
 * @note 初始化之后会自动保持静止
 * @param chassis 底盘
 * @param config 配置
 */
void Chassis_Init(Chassis_t* chassis, const Chassis_Config_t* config)
{
    ChassisDriver_Init(&chassis->driver, &config->driver);

    chassis->feedback.vx  = config->feedback_source.vx;
    chassis->feedback.vy  = config->feedback_source.vy;
    chassis->feedback.wz  = config->feedback_source.wz;
    chassis->feedback.sx  = config->feedback_source.sx;
    chassis->feedback.sy  = config->feedback_source.sy;
    chassis->feedback.yaw = config->feedback_source.yaw;

    chassis->last_feedback.sx  = 0;
    chassis->last_feedback.sy  = 0;
    chassis->last_feedback.yaw = 0;

    chassis->chassis_update_interval = config->chassis_update_interval * 1e-3f; // ms -> s

    // 初始化位置 PD 控制器
    PD_Init(&chassis->posture.trajectory.pd.vx, &config->posture.error_pd.vx);
    PD_Init(&chassis->posture.trajectory.pd.vy, &config->posture.error_pd.vy);
    PD_Init(&chassis->posture.trajectory.pd.wz, &config->posture.error_pd.wz);

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
        // 进行修正 1e-3f 为更新间隔，此处发现前馈并没有什么精度优化，暂时不做前馈
        // const float              beta     = DEG2RAD(0.5f * chassis->velocity.in_body.wz * 1e-3f);
        // const float              cot_beta = 1.0f / tanf(beta);
        // const Chassis_Velocity_t temp_velocity = {
        //     .vx = beta * (chassis->velocity.in_body.vx * cot_beta +
        //     chassis->velocity.in_body.vy), .vy = beta * (chassis->velocity.in_body.vy * cot_beta
        //     - chassis->velocity.in_body.vx), .wz = chassis->velocity.in_body.wz
        // };
        // ChassisDriver_ApplyVelocity(&chassis->driver,
        //                             temp_velocity.vx,
        //                             temp_velocity.vy,
        //                             temp_velocity.wz);
    }
    else
    {
        // 直接应用速度
    }
    ChassisDriver_ApplyVelocity(&chassis->driver,
                                chassis->velocity.in_body.vx,
                                chassis->velocity.in_body.vy,
                                chassis->velocity.in_body.wz);
}

void update_chassis_position_control(Chassis_t* chassis)
{
    const float now = chassis->posture.trajectory.now + chassis->chassis_update_interval;
    chassis->posture.trajectory.now = now;

    // 计算前馈速度
    const Chassis_Velocity_t ff_velocity = {
        .vx = SCurve_CalcV(&chassis->posture.trajectory.curve.x, now),
        .vy = SCurve_CalcV(&chassis->posture.trajectory.curve.y, now),
        .wz = SCurve_CalcV(&chassis->posture.trajectory.curve.yaw, now)
    };

    // 计算当前目标
    const Chassis_Posture_t target_now = {
        .x   = SCurve_CalcX(&chassis->posture.trajectory.curve.x, now),
        .y   = SCurve_CalcX(&chassis->posture.trajectory.curve.y, now),
        .yaw = SCurve_CalcX(&chassis->posture.trajectory.curve.yaw, now)
    };

    // 使用 pd 控制器跟随当前目标
    chassis->posture.trajectory.pd.vx.ref = target_now.x;
    chassis->posture.trajectory.pd.vx.fdb = chassis->posture.in_world.x;
    PD_Calculate(&chassis->posture.trajectory.pd.vx);

    chassis->posture.trajectory.pd.vy.ref = target_now.y;
    chassis->posture.trajectory.pd.vy.fdb = chassis->posture.in_world.y;
    PD_Calculate(&chassis->posture.trajectory.pd.vy);

    chassis->posture.trajectory.pd.wz.ref = target_now.yaw;
    chassis->posture.trajectory.pd.wz.fdb = chassis->posture.in_world.yaw;
    PD_Calculate(&chassis->posture.trajectory.pd.wz);

    // 叠加前馈和 pd 输出
    const Chassis_Velocity_t velocity_in_world = {
        ff_velocity.vx + chassis->posture.trajectory.pd.vx.output,
        ff_velocity.vy + chassis->posture.trajectory.pd.vy.output,
        ff_velocity.wz + chassis->posture.trajectory.pd.wz.output,
    };

    // 将世界坐标系速度转换为底盘坐标系速度
    Chassis_Velocity_t body_velocity;
    Chassis_WorldVelocity2BodyVelocity(chassis, &velocity_in_world, &body_velocity);
    // 应用速度
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

/**
 * 相对于世界坐标系设置目标位姿
 * @param chassis 底盘
 * @param absolute_target 绝对目标位姿
 * @return 生成移动曲线的结果
 */
SCurve_Result_t Chassis_SetTargetPostureInWorld(Chassis_t*                     chassis,
                                                const Chassis_PostureTarget_t* absolute_target)
{
    osMutexAcquire(chassis->lock, osWaitForever);

    // copy 当前位置和速度
    const Chassis_Posture_t  start    = chassis->posture.in_world;
    const Chassis_Velocity_t velocity = chassis->velocity.in_world;

    SCurve_t curve_x, curve_y, curve_yaw;
    float    ax = 0, ay = 0, ayaw = 0;
    if (chassis->ctrl_mode == CHASSIS_POS)
    {
        ax   = SCurve_CalcA(&chassis->posture.trajectory.curve.x, chassis->posture.trajectory.now);
        ay   = SCurve_CalcA(&chassis->posture.trajectory.curve.y, chassis->posture.trajectory.now);
        ayaw = SCurve_CalcA(&chassis->posture.trajectory.curve.yaw,
                            chassis->posture.trajectory.now);
    }
    // 初始化 S 型曲线
    // 衔接当前位置，速度，如果之前是位置控制还会衔接加速度
    const SCurve_Result_t res_x   = SCurve_Init(&curve_x,
                                              start.x,
                                              absolute_target->posture.x,
                                              velocity.vx,
                                              ax,
                                              absolute_target->limit_x.max_spd,
                                              absolute_target->limit_x.max_acc,
                                              absolute_target->limit_x.max_jerk);
    const SCurve_Result_t res_y   = SCurve_Init(&curve_y,
                                              start.y,
                                              absolute_target->posture.y,
                                              velocity.vy,
                                              ay,
                                              absolute_target->limit_y.max_spd,
                                              absolute_target->limit_y.max_acc,
                                              absolute_target->limit_y.max_jerk);
    const SCurve_Result_t res_yaw = SCurve_Init(&curve_yaw,
                                                start.yaw,
                                                absolute_target->posture.yaw,
                                                velocity.wz,
                                                ayaw,
                                                absolute_target->limit_yaw.max_spd,
                                                absolute_target->limit_yaw.max_acc,
                                                absolute_target->limit_yaw.max_jerk);

    if (res_x == S_CURVE_FAILED || res_y == S_CURVE_FAILED || res_yaw == S_CURVE_FAILED)
        return S_CURVE_FAILED;

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    chassis->posture.trajectory.now = 0;

    chassis->posture.trajectory.curve.x   = curve_x;
    chassis->posture.trajectory.curve.y   = curve_y;
    chassis->posture.trajectory.curve.yaw = curve_yaw;

    chassis->ctrl_mode = CHASSIS_POS;

    isr_unlock(saved);

    osMutexRelease(chassis->lock);
    return S_CURVE_SUCCESS;
}

/**
 * 相对于机身坐标系设置目标位姿
 * @param chassis 底盘
 * @param relative_target 相对目标位姿
 * @return 生成移动曲线的结果
 */
SCurve_Result_t Chassis_SetTargetPostureInBody(Chassis_t*                     chassis,
                                               const Chassis_PostureTarget_t* relative_target)
{
    Chassis_PostureTarget_t absolute_target = *relative_target;

    osMutexAcquire(chassis->lock, osWaitForever);
    Chassis_BodyPosture2WorldPosture(chassis, &relative_target->posture, &absolute_target.posture);
    osMutexRelease(chassis->lock);

    return Chassis_SetTargetPostureInWorld(chassis, &absolute_target);
}

/**
 * 基于世界坐标系设置速度
 *
 * @param chassis 底盘对象
 * @param world_velocity 在世界坐标系下的速度
 * @param target_in_world 是否以世界坐标系为参考系保持速度不变.
 *                        为 true 则车体会相对于世界坐标系保持速度不变;
 *                        为 false 则相对于车身坐标系保持不变.
 */
void Chassis_SetVelWorldFrame(Chassis_t*                chassis,
                              const Chassis_Velocity_t* world_velocity,
                              const bool                target_in_world)
{
    osMutexAcquire(chassis->lock, osWaitForever);
    Chassis_Velocity_t shadow_body_velocity;
    Chassis_WorldVelocity2BodyVelocity(chassis, world_velocity, &shadow_body_velocity);

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    chassis->velocity.target_in_world = target_in_world;
    chassis->velocity.in_world.vx     = world_velocity->vx;
    chassis->velocity.in_world.vy     = world_velocity->vy;
    chassis->velocity.in_world.wz     = world_velocity->wz;
    chassis->velocity.in_body.vx      = shadow_body_velocity.vx;
    chassis->velocity.in_body.vy      = shadow_body_velocity.vy;
    chassis->velocity.in_body.wz      = shadow_body_velocity.wz;

    chassis->ctrl_mode = CHASSIS_VEL;

    isr_unlock(saved);

    osMutexRelease(chassis->lock);
}

/**
 * 基于车身坐标系设置速度
 *
 * @param chassis 底盘对象
 * @param body_velocity 在世界坐标系下的速度
 * @param target_in_world 是否以世界坐标系为参考系保持速度不变.
 *                        为 true 则车体会相对于世界坐标系保持速度不变;
 *                        为 false 则相对于车身坐标系保持不变.
 */
void Chassis_SetVelBodyFrame(Chassis_t*                chassis,
                             const Chassis_Velocity_t* body_velocity,
                             const bool                target_in_world)
{
    osMutexAcquire(chassis->lock, osWaitForever);
    Chassis_Velocity_t shadow_world_velocity;
    Chassis_BodyVelocity2WorldVelocity(chassis, body_velocity, &shadow_world_velocity);

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    chassis->velocity.target_in_world = target_in_world;
    chassis->velocity.in_body.vx      = body_velocity->vx;
    chassis->velocity.in_body.vy      = body_velocity->vy;
    chassis->velocity.in_body.wz      = body_velocity->wz;
    chassis->velocity.in_world.vx     = shadow_world_velocity.vx;
    chassis->velocity.in_world.vy     = shadow_world_velocity.vy;
    chassis->velocity.in_world.wz     = shadow_world_velocity.wz;

    chassis->ctrl_mode = CHASSIS_VEL;

    isr_unlock(saved);
    osMutexRelease(chassis->lock);
}

/**
 * 将当前车身坐标系设置为世界坐标系
 * @param chassis 底盘对象
 */
void Chassis_SetWorldFromCurrent(Chassis_t* chassis)
{
    osMutexAcquire(chassis->lock, osWaitForever);

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    chassis->world.posture.x += chassis->posture.in_world.x;
    chassis->world.posture.y += chassis->posture.in_world.y;
    chassis->world.posture.yaw += chassis->posture.in_world.yaw;
    chassis->posture.in_world.x   = 0.0f;
    chassis->posture.in_world.y   = 0.0f;
    chassis->posture.in_world.yaw = 0.0f;
    chassis->velocity.in_world    = chassis->velocity.in_body;

    isr_unlock(saved);

    osMutexRelease(chassis->lock);
}

#ifdef __cplusplus
}
#endif