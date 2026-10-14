#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include <stdint.h>

typedef struct
{
    uint16_t pitch_ecd;
    int16_t pitch_speed;
    int16_t pitch_current;
    uint8_t pitch_temperature;
    uint8_t pitch_online;
    int16_t pitch_output;
    float pitch_target_rad;
    float pitch_reference_rad;
    float pitch_feedforward;
    uint16_t yaw_ecd;
    int16_t yaw_speed;
    int16_t yaw_current;
    uint8_t yaw_temperature;
    uint8_t yaw_online;
    int16_t yaw_output;
    float yaw_target_rad;
    float yaw_reference_rad;
    float yaw_feedforward;
    uint8_t references_ready;
    uint8_t control_enabled;
    uint8_t link_alive;
    uint8_t fire_requested;
    uint32_t protocol_crc_errors;
    uint32_t loop_count;
} GimbalDebug_t;

void GimbalTask_Init(void);
void GimbalTask_GetDebug(GimbalDebug_t *debug);

#endif /* GIMBAL_TASK_H */
