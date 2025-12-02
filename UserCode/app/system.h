/**
 * @file    system.h
 * @author  syhanjin
 * @date    2025-12-02
 * @brief   system defines
 */
#ifndef SYSTEM_H
#define SYSTEM_H

#include "cmsis_os2.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif
extern osEventFlagsId_t systemEventHandle;

#define SYSTEM_INITIALIZED (0x00000001U)

static uint32_t System_WaitInitialize(void)
{
    return osEventFlagsWait(systemEventHandle,
                            SYSTEM_INITIALIZED,
                            osFlagsWaitAny | osFlagsNoClear,
                            osWaitForever);
}
#ifdef __cplusplus
}
#endif

#endif // SYSTEM_H
