#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdbool.h>

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float quaternion[4]; /* w, x, y, z */
    float gyro[3];
    float gyro_offset[3];
    float temperature;
    float calibration_temperature;
    unsigned int warmup_timed_out;
    unsigned int valid;
} ImuAttitude_t;

void ImuTask_Init(void);
void ImuTask_GetAttitude(ImuAttitude_t *attitude);
void ImuTask_SetReportingEnabled(bool enabled);

/* 0: starting, 1: sensor ready/warming/calibrating,
 * 2: gyro bias calibrated, 3: running.
 * 0x100 + BMI088 error code means sensor initialization failed. */
extern volatile unsigned int g_imu_task_state;

#endif /* IMU_TASK_H */
