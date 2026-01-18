/**
 * @file    chassis.h
 * @author  syhanjin
 * @date    2025-12-02
 * @brief   底盘机构的管理
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "interfaces/chassis_if.h"
#ifdef __cplusplus
extern "C"
{
#endif

extern Chassis_t chassis;

static void APP_Chassis_Update_200Hz()
{
    Chassis_TrajUpdate(&chassis);
}

static uint32_t prescaler_500Hz = 0;

static void APP_Chassis_Update_1kHz()
{
    Chassis_FeedbackUpdate(&chassis);
    prescaler_500Hz++;
    if (prescaler_500Hz >= 2)
    {
        Chassis_TrajPDUpdate(&chassis);
        prescaler_500Hz = 0;
    }
    Chassis_VelUpdate(&chassis);
}

void APP_Chassis_InitBeforeUpdate();
void APP_Chassis_Init();

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_H
