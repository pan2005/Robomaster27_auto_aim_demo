#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include "bsp_can.h"
#include "pid.h"
#include <stdint.h>

#define CAN_MOTOR_ENCODER_COUNTS 8192
#define CAN_MOTOR_PI             3.14159265358979323846f
#define CAN_MOTOR_TWO_PI         (2.0f * CAN_MOTOR_PI)
#define CAN_MOTOR_OFFLINE_MS     100U

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    uint8_t feedback_initialized;
    int32_t round_count;
    uint32_t last_rx_tick;
} CanMotorMeasure_t;

typedef enum
{
    CAN_MOTOR_M3508_SPEED = 0,
    CAN_MOTOR_M3508_POSITION,
    CAN_MOTOR_GM6020_POSITION
} CanMotorMode_t;

typedef struct CanMotor
{
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t rx_id;
    CanMotorMode_t mode;
    CanMotorMeasure_t measure;
    PID_t position_pid;
    PID_t speed_pid;
    float gear_ratio;
    float target;
    float target_speed;
    float current_angle;
    float feedforward;
    float control_period_s;
    int16_t output;
    uint8_t enabled;
} CanMotor_t;

int CanMotor_Init(CanMotor_t *motor, FDCAN_HandleTypeDef *hfdcan,
                  uint32_t rx_id, CanMotorMode_t mode, float gear_ratio);
void CanMotor_Enable(CanMotor_t *motor);
void CanMotor_Disable(CanMotor_t *motor);
void CanMotor_SetTarget(CanMotor_t *motor, float target);
void CanMotor_Update(CanMotor_t *motor);
uint8_t CanMotor_IsOnline(const CanMotor_t *motor);

HAL_StatusTypeDef DJI_Motor_SendGroup(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t command_id,
                                      int16_t value1, int16_t value2,
                                      int16_t value3, int16_t value4);

float CanMotor_NormalizeAngle(float angle);

#endif /* CAN_MOTOR_H */
