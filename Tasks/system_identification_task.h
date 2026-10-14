#ifndef SYSTEM_IDENTIFICATION_TASK_H
#define SYSTEM_IDENTIFICATION_TASK_H

#include <stdint.h>

typedef enum
{
    SYSID_STATE_IDLE = 0,
    SYSID_STATE_ARMED = 1,
    SYSID_STATE_RUNNING = 2,
    SYSID_STATE_COMPLETE = 3,
    SYSID_STATE_ABORTED = 4
} SysIdState_t;

typedef enum
{
    SYSID_EXCITATION_STEP = 1,
    SYSID_EXCITATION_PRBS = 2
} SysIdExcitation_t;

void SystemIdentificationTask_Init(void);

#endif /* SYSTEM_IDENTIFICATION_TASK_H */
