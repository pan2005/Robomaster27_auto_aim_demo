#include "can_motor.h"

#include "stm32h7xx_hal.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static void CanMotor_Decode(void *device, const uint8_t *data, uint8_t length)
{
    CanMotor_t *motor = (CanMotor_t *)device;
    if ((motor == NULL) || (data == NULL) || (length < 7U)) return;

    uint16_t new_ecd = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    if (motor->measure.feedback_initialized != 0U)
    {
        int32_t difference = (int32_t)new_ecd - (int32_t)motor->measure.ecd;
        if (difference < -(CAN_MOTOR_ENCODER_COUNTS / 2)) ++motor->measure.round_count;
        else if (difference > (CAN_MOTOR_ENCODER_COUNTS / 2)) --motor->measure.round_count;
    }
    else
    {
        motor->measure.feedback_initialized = 1U;
        motor->measure.round_count = 0;
    }

    motor->measure.last_ecd = motor->measure.ecd;
    motor->measure.ecd = new_ecd;
    motor->measure.speed_rpm = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    motor->measure.given_current = (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
    motor->measure.temperature = data[6];
    motor->measure.last_rx_tick = HAL_GetTick();

    float motor_shaft_angle = ((float)(motor->measure.round_count * CAN_MOTOR_ENCODER_COUNTS
                                      + motor->measure.ecd)
                              / (float)CAN_MOTOR_ENCODER_COUNTS) * CAN_MOTOR_TWO_PI;
    motor->current_angle = motor_shaft_angle / motor->gear_ratio;
}

float CanMotor_NormalizeAngle(float angle)
{
    angle = fmodf(angle, CAN_MOTOR_TWO_PI);
    if (angle > CAN_MOTOR_PI) angle -= CAN_MOTOR_TWO_PI;
    if (angle < -CAN_MOTOR_PI) angle += CAN_MOTOR_TWO_PI;
    return angle;
}

int CanMotor_Init(CanMotor_t *motor, FDCAN_HandleTypeDef *hfdcan,
                  uint32_t rx_id, CanMotorMode_t mode, float gear_ratio)
{
    if ((motor == NULL) || (hfdcan == NULL) || (rx_id > 0x7FFU) ||
        (gear_ratio <= 0.0f)) return -1;

    memset(motor, 0, sizeof(*motor));
    motor->hfdcan = hfdcan;
    motor->rx_id = rx_id;
    motor->mode = mode;
    motor->gear_ratio = gear_ratio;
    motor->control_period_s = 0.001f;
    return BSP_CAN_RegisterRxCallback(hfdcan, rx_id, FDCAN_STANDARD_ID,
                                      CanMotor_Decode, motor);
}

void CanMotor_Enable(CanMotor_t *motor)
{
    if (motor == NULL) return;
    PID_Reset(&motor->position_pid);
    PID_Reset(&motor->speed_pid);
    motor->target = motor->current_angle;
    motor->target_speed = 0.0f;
    motor->output = 0;
    motor->enabled = 1U;
}

void CanMotor_Disable(CanMotor_t *motor)
{
    if (motor == NULL) return;
    motor->enabled = 0U;
    motor->output = 0;
    PID_Reset(&motor->position_pid);
    PID_Reset(&motor->speed_pid);
}

void CanMotor_SetTarget(CanMotor_t *motor, float target)
{
    if (motor != NULL) motor->target = target;
}

uint8_t CanMotor_IsOnline(const CanMotor_t *motor)
{
    return (motor != NULL) && (motor->measure.feedback_initialized != 0U) &&
           ((HAL_GetTick() - motor->measure.last_rx_tick) <= CAN_MOTOR_OFFLINE_MS);
}

void CanMotor_Update(CanMotor_t *motor)
{
    if ((motor == NULL) || (motor->enabled == 0U) || !CanMotor_IsOnline(motor))
    {
        if (motor != NULL) motor->output = 0;
        return;
    }

    float speed_target;
    if (motor->mode == CAN_MOTOR_M3508_SPEED)
    {
        speed_target = motor->target;
    }
    else
    {
        float position_target = motor->target;
        if (motor->mode == CAN_MOTOR_GM6020_POSITION)
        {
            float error = CanMotor_NormalizeAngle(motor->target
                                                  - CanMotor_NormalizeAngle(motor->current_angle));
            position_target = motor->current_angle + error;
        }
        speed_target = PID_CalculateDt(&motor->position_pid, position_target,
                                       motor->current_angle,
                                       motor->control_period_s);
    }

    motor->target_speed = speed_target;
    float output = PID_CalculateDt(&motor->speed_pid, speed_target,
                                   (float)motor->measure.speed_rpm,
                                   motor->control_period_s) + motor->feedforward;
    if (output > 30000.0f) output = 30000.0f;
    if (output < -30000.0f) output = -30000.0f;
    motor->output = (int16_t)output;
}

HAL_StatusTypeDef DJI_Motor_SendGroup(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t command_id,
                                      int16_t value1, int16_t value2,
                                      int16_t value3, int16_t value4)
{
    if ((command_id != 0x200U) && (command_id != 0x1FFU) &&
        (command_id != 0x2FFU) && (command_id != 0x1FEU)) return HAL_ERROR;

    const int16_t values[4] = {value1, value2, value3, value4};
    uint8_t data[8];
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        data[i * 2U] = (uint8_t)((uint16_t)values[i] >> 8U);
        data[i * 2U + 1U] = (uint8_t)values[i];
    }
    return BSP_CAN_Send(hfdcan, command_id, FDCAN_STANDARD_ID, data, 8U);
}
