#include "gimbal_task.h"

#include "bsp_usb_cdc.h"
#include "can_motor.h"
#include "fdcan.h"
#include "gimbal_protocol.h"
#include "imu_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define GIMBAL_CONTROL_PERIOD_TICKS     1U
#define GIMBAL_CONTROL_DT_S             0.001f
#define GIMBAL_COMMAND_STEP_RAD         (3.0f * CAN_MOTOR_PI / 180.0f)
#define GIMBAL_COMMAND_TIMEOUT_MS       200U
#define GIMBAL_STATUS_PERIOD_MS         20U
#define GIMBAL_ATTITUDE_PERIOD_MS       10U
#define GIMBAL_MDEG_TO_RAD              (CAN_MOTOR_PI / 180000.0f)
#define GIMBAL_RAD_TO_MDEG              (180000.0f / CAN_MOTOR_PI)
#define PITCH_ENCODER_LIMIT_RAD         (CAN_MOTOR_PI / 2.0f)
#define YAW_ENCODER_LIMIT_RAD           (CAN_MOTOR_PI / 2.0f)
#define GIMBAL_SPEED_LIMIT_RPM          60.0f

/* Pitch parameters identified on the 2026-10-01 mechanical configuration.
 * The dynamic feedforward scale of 0.75 is the best value actually validated
 * by the A/B trajectory test; the full identified inverse is not used yet. */
#define PITCH_POSITION_KP               250.0f
#define PITCH_SPEED_KP                  250.0f
#define PITCH_CURRENT_LIMIT             16000.0f
#define PITCH_GRAVITY_KG                (-2298.8448f)
#define PITCH_GRAVITY_BIAS              (-99.3683f)
#define PITCH_DYNAMIC_KV                1830.4092f
#define PITCH_DYNAMIC_KA                50.6137f
#define PITCH_DYNAMIC_FF_SCALE          0.75f
#define PITCH_REFERENCE_MAX_VEL_RAD_S   3.2f
#define PITCH_REFERENCE_MAX_ACC_RAD_S2  10.0f

/* Final saved Yaw velocity-loop candidate.  Friction and viscous terms are
 * direction dependent because the two measured directions were asymmetric. */
#define YAW_POSITION_KP                 150.0f
#define YAW_SPEED_KP                    120.0f
#define YAW_SPEED_KI                    300.0f
#define YAW_SPEED_INTEGRAL_LIMIT        5.0f
#define YAW_PID_CURRENT_LIMIT           8000.0f
#define YAW_TOTAL_CURRENT_LIMIT         16000.0f
#define YAW_POS_FRICTION_CURRENT        734.3f
#define YAW_POS_VELOCITY_CURRENT_RPM    102.1f
#define YAW_NEG_FRICTION_CURRENT        486.4f
#define YAW_NEG_VELOCITY_CURRENT_RPM    98.9f
#define YAW_FRICTION_BLEND_RPM          1.5f
#define YAW_FRICTION_SLEW_PER_S         5000.0f
#define YAW_ZERO_TARGET_RPM             0.5f
#define YAW_INTEGRAL_LEAK_TAU_S         0.10f
#define YAW_REFERENCE_MAX_VEL_RAD_S     (16.0f * 2.0f * CAN_MOTOR_PI / 60.0f)
#define YAW_REFERENCE_MAX_ACC_RAD_S2    10.0f

#define RAD_S_TO_RPM                    (60.0f / (2.0f * CAN_MOTOR_PI))

/* Both signs were confirmed on the assembled gimbal. */
#define PITCH_OUTPUT_SIGN               1.0f
#define YAW_OUTPUT_SIGN                 1.0f

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float acceleration_rad_s2;
} MotionReference_t;

static void GimbalTask_Entry(void *argument);

static const osThreadAttr_t gimbal_task_attributes = {
    .name = "GimbalTask",
    .stack_size = 256U * 4U,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

static CanMotor_t pitch_motor;
static CanMotor_t yaw_motor;
static GimbalDebug_t g_gimbal_debug;

static int32_t RadToMdeg(float value)
{
    return (int32_t)lroundf(value * GIMBAL_RAD_TO_MDEG);
}

static int32_t RadPerSecToMdeg(float value)
{
    return (int32_t)lroundf(value * GIMBAL_RAD_TO_MDEG);
}

static int32_t RadPerSecToMicro(float value)
{
    return (int32_t)lroundf(value * 1000000.0f);
}

static int16_t QuaternionToI16(float value)
{
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    return (int16_t)lroundf(value * 32767.0f);
}

static uint8_t SequenceIsNewer(uint16_t sequence, uint16_t previous,
                               uint8_t initialized)
{
    return (initialized == 0U) || ((int16_t)(sequence - previous) > 0);
}

static void SendStatus(struct usb_device *usb, uint16_t sequence,
                       uint16_t flags, const ImuAttitude_t *imu,
                       float yaw_unwrapped, float pitch_zero, float yaw_zero,
                       float pitch_target, float yaw_target,
                       const GimbalProtocolParser_t *parser,
                       uint16_t last_rx_sequence)
{
    enum { STATUS_PAYLOAD_SIZE = 29 };
    static uint8_t frame_buffer[GIMBAL_PROTOCOL_MAX_FRAME];
    uint8_t payload[STATUS_PAYLOAD_SIZE] = {0};
    GimbalProtocol_WriteU16(&payload[0], flags);
    GimbalProtocol_WriteI32(&payload[2], RadToMdeg(imu->pitch - pitch_zero));
    GimbalProtocol_WriteI32(&payload[6], RadToMdeg(yaw_unwrapped - yaw_zero));
    GimbalProtocol_WriteI32(&payload[10], RadToMdeg(pitch_target - pitch_zero));
    GimbalProtocol_WriteI32(&payload[14], RadToMdeg(yaw_target - yaw_zero));
    GimbalProtocol_WriteI16(&payload[18], pitch_motor.measure.speed_rpm);
    GimbalProtocol_WriteI16(&payload[20], yaw_motor.measure.speed_rpm);
    payload[22] = pitch_motor.measure.temperature;
    payload[23] = yaw_motor.measure.temperature;
    uint32_t errors = parser->crc_errors + parser->length_errors +
                      parser->version_errors;
    if (errors > 0xFFFFU) errors = 0xFFFFU;
    GimbalProtocol_WriteU16(&payload[24], (uint16_t)errors);
    GimbalProtocol_WriteU16(&payload[26], last_rx_sequence);
    payload[28] = GIMBAL_PROTOCOL_VERSION;

    size_t length = GimbalProtocol_Encode(
        GIMBAL_MSG_STATUS, sequence, payload, sizeof(payload),
        frame_buffer, sizeof(frame_buffer));
    if (length > 0U)
    {
        /* Non-blocking: a busy CDC endpoint simply drops this status sample. */
        (void)usb->Send(usb, (char *)frame_buffer, (int)length);
    }
}

static void SendAttitude(struct usb_device *usb, uint16_t sequence,
                         uint16_t flags, uint32_t timestamp_ms,
                         const ImuAttitude_t *imu, float yaw_unwrapped,
                         float pitch_zero, float yaw_zero)
{
    enum { ATTITUDE_PAYLOAD_SIZE = 46 };
    static uint8_t frame_buffer[GIMBAL_PROTOCOL_MAX_FRAME];
    uint8_t payload[ATTITUDE_PAYLOAD_SIZE] = {0};
    GimbalProtocol_WriteU32(&payload[0], timestamp_ms);
    GimbalProtocol_WriteU16(&payload[4], flags);
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        GimbalProtocol_WriteI16(&payload[6U + i * 2U],
                                QuaternionToI16(imu->quaternion[i]));
    }
    GimbalProtocol_WriteI32(&payload[14], RadToMdeg(imu->pitch - pitch_zero));
    GimbalProtocol_WriteI32(&payload[18], RadToMdeg(yaw_unwrapped - yaw_zero));
    GimbalProtocol_WriteI32(&payload[22], RadPerSecToMdeg(imu->gyro[1]));
    GimbalProtocol_WriteI32(&payload[26], RadPerSecToMdeg(imu->gyro[2]));
    GimbalProtocol_WriteI16(&payload[30],
                            (int16_t)lroundf(imu->temperature * 100.0f));
    GimbalProtocol_WriteI16(
        &payload[32],
        (int16_t)lroundf(imu->calibration_temperature * 100.0f));
    GimbalProtocol_WriteI32(&payload[34],
                            RadPerSecToMicro(imu->gyro_offset[0]));
    GimbalProtocol_WriteI32(&payload[38],
                            RadPerSecToMicro(imu->gyro_offset[1]));
    GimbalProtocol_WriteI32(&payload[42],
                            RadPerSecToMicro(imu->gyro_offset[2]));
    size_t length = GimbalProtocol_Encode(
        GIMBAL_MSG_ATTITUDE, sequence, payload, sizeof(payload),
        frame_buffer, sizeof(frame_buffer));
    if (length > 0U) (void)usb->Send(usb, (char *)frame_buffer, (int)length);
}

static void SendTimeSync(struct usb_device *usb, uint16_t sequence,
                         const uint8_t *request, uint32_t timestamp_ms)
{
    static uint8_t frame_buffer[GIMBAL_PROTOCOL_MAX_FRAME];
    uint8_t payload[12];
    memcpy(payload, request, 8U);
    GimbalProtocol_WriteU32(&payload[8], timestamp_ms);
    size_t length = GimbalProtocol_Encode(
        GIMBAL_MSG_TIME_SYNC_RESP, sequence, payload, sizeof(payload),
        frame_buffer, sizeof(frame_buffer));
    if (length > 0U) (void)usb->Send(usb, (char *)frame_buffer, (int)length);
}

void GimbalTask_GetDebug(GimbalDebug_t *debug)
{
    if (debug == NULL) return;
    taskENTER_CRITICAL();
    *debug = g_gimbal_debug;
    taskEXIT_CRITICAL();
}

static float ClampFloat(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static int16_t ClampMotorOutput(float output, float limit)
{
    return (int16_t)ClampFloat(output, limit);
}

static void MotionReferenceReset(MotionReference_t *reference,
                                 float position_rad)
{
    reference->position_rad = position_rad;
    reference->velocity_rad_s = 0.0f;
    reference->acceleration_rad_s2 = 0.0f;
}

/* Acceleration-limited reference generator.  Besides avoiding a 3 degree
 * target discontinuity, it supplies physically meaningful velocity and
 * acceleration inputs to the identified feedforward model. */
static void MotionReferenceUpdate(MotionReference_t *reference,
                                  float target_rad, float max_velocity_rad_s,
                                  float max_acceleration_rad_s2)
{
    float error = target_rad - reference->position_rad;
    if ((fabsf(error) < 1.0e-6f) &&
        (fabsf(reference->velocity_rad_s) < 1.0e-4f))
    {
        MotionReferenceReset(reference, target_rad);
        return;
    }

    float braking_velocity = sqrtf(2.0f * max_acceleration_rad_s2 *
                                   fabsf(error));
    float desired_velocity = fminf(max_velocity_rad_s, braking_velocity);
    if (error < 0.0f) desired_velocity = -desired_velocity;

    float old_velocity = reference->velocity_rad_s;
    float velocity_step = ClampFloat(desired_velocity - old_velocity,
                                     max_acceleration_rad_s2 *
                                     GIMBAL_CONTROL_DT_S);
    float new_velocity = old_velocity + velocity_step;
    float new_position = reference->position_rad +
                         new_velocity * GIMBAL_CONTROL_DT_S;

    if ((error != 0.0f) &&
        ((target_rad - new_position) * error <= 0.0f))
    {
        MotionReferenceReset(reference, target_rad);
    }
    else
    {
        reference->position_rad = new_position;
        reference->velocity_rad_s = new_velocity;
        reference->acceleration_rad_s2 = velocity_step /
                                         GIMBAL_CONTROL_DT_S;
    }
}

static float PitchFeedforward(const ImuAttitude_t *imu,
                              const MotionReference_t *reference)
{
    float gravity = PITCH_GRAVITY_KG * sinf(imu->pitch) +
                    PITCH_GRAVITY_BIAS;
    float dynamic = PITCH_DYNAMIC_FF_SCALE *
                    (PITCH_DYNAMIC_KV * reference->velocity_rad_s +
                     PITCH_DYNAMIC_KA * reference->acceleration_rad_s2);
    return gravity + dynamic;
}

static float YawFeedforward(float target_rpm, float *friction_command)
{
    float ff_target_rpm = (fabsf(target_rpm) >= YAW_ZERO_TARGET_RPM)
                        ? target_rpm : 0.0f;
    float blend = tanhf(fabsf(ff_target_rpm) / YAW_FRICTION_BLEND_RPM);
    float friction_target = 0.0f;
    float velocity = 0.0f;

    if (ff_target_rpm > 0.0f)
    {
        friction_target = YAW_POS_FRICTION_CURRENT * blend;
        velocity = YAW_POS_VELOCITY_CURRENT_RPM * ff_target_rpm;
    }
    else if (ff_target_rpm < 0.0f)
    {
        friction_target = -YAW_NEG_FRICTION_CURRENT * blend;
        velocity = YAW_NEG_VELOCITY_CURRENT_RPM * ff_target_rpm;
    }

    *friction_command += ClampFloat(friction_target - *friction_command,
                                    YAW_FRICTION_SLEW_PER_S *
                                    GIMBAL_CONTROL_DT_S);
    return *friction_command + velocity;
}

void GimbalTask_Init(void)
{
    (void)osThreadNew(GimbalTask_Entry, NULL, &gimbal_task_attributes);
}

static void GimbalTask_Entry(void *argument)
{
    (void)argument;
    struct usb_device *usb = usb_get_device();
    uint8_t command_buffer[64];
    GimbalProtocolParser_t protocol_parser;
    GimbalProtocolFrame_t protocol_frame;
    ImuAttitude_t imu = {0};
    MotionReference_t pitch_reference = {0};
    MotionReference_t yaw_reference = {0};
    float pitch_target = 0.0f;
    float yaw_target = 0.0f;
    float pitch_imu_zero = 0.0f;
    float yaw_imu_zero = 0.0f;
    float yaw_unwrapped = 0.0f;
    float yaw_last_wrapped = 0.0f;
    float pitch_encoder_zero = 0.0f;
    float yaw_encoder_zero = 0.0f;
    float yaw_friction_command = 0.0f;
    uint8_t yaw_unwrap_ready = 0U;
    uint8_t references_ready = 0U;
    uint8_t control_enabled = 0U;
    uint8_t fire_requested = 0U;
    uint8_t aim_mode = 0U;
    uint32_t aim_execute_at_ms = 0U;
    float aim_pitch_rad = 0.0f;
    float aim_yaw_rad = 0.0f;
    float aim_pitch_rate_rad_s = 0.0f;
    float aim_yaw_rate_rad_s = 0.0f;
    float aim_pitch_accel_rad_s2 = 0.0f;
    float aim_yaw_accel_rad_s2 = 0.0f;
    uint8_t hello_received = 0U;
    uint8_t control_sequence_ready = 0U;
    uint16_t last_control_sequence = 0U;
    uint16_t status_sequence = 0U;
    uint16_t attitude_sequence = 0U;
    uint16_t time_sync_sequence = 0U;
    uint32_t last_control_tick = 0U;
    uint32_t last_status_tick = 0U;
    uint32_t last_attitude_tick = 0U;

    GimbalProtocolParser_Init(&protocol_parser);

    (void)CanMotor_Init(&pitch_motor, &hfdcan1, 0x205U,
                        CAN_MOTOR_GM6020_POSITION, 1.0f);
    (void)CanMotor_Init(&yaw_motor, &hfdcan1, 0x206U,
                        CAN_MOTOR_GM6020_POSITION, 1.0f);

    PID_Init(&pitch_motor.position_pid, PITCH_POSITION_KP, 0.0f, 0.0f, 0.0f,
             0.0f, GIMBAL_SPEED_LIMIT_RPM);
    PID_Init(&pitch_motor.speed_pid, PITCH_SPEED_KP, 0.0f, 0.0f, 0.0f,
             0.0f, PITCH_CURRENT_LIMIT);
    PID_Init(&yaw_motor.position_pid, YAW_POSITION_KP, 0.0f, 0.0f, 0.0f,
             0.0f, GIMBAL_SPEED_LIMIT_RPM);
    PID_Init(&yaw_motor.speed_pid, YAW_SPEED_KP, YAW_SPEED_KI, 0.0f, 0.0f,
             YAW_SPEED_INTEGRAL_LIMIT, YAW_PID_CURRENT_LIMIT);

    /* Let defaultTask initialize the USB receive queue first. */
    osDelay(200U);
    (void)osThreadSetPriority(osThreadGetId(), osPriorityHigh);
    uint32_t wake_tick = osKernelGetTickCount();

    for (;;)
    {
        ImuTask_GetAttitude(&imu);

        if (imu.valid != 0U)
        {
            if (yaw_unwrap_ready == 0U)
            {
                yaw_unwrapped = imu.yaw;
                yaw_last_wrapped = imu.yaw;
                yaw_unwrap_ready = 1U;
            }
            else
            {
                yaw_unwrapped += CanMotor_NormalizeAngle(
                    imu.yaw - yaw_last_wrapped);
                yaw_last_wrapped = imu.yaw;
            }
        }

        if ((references_ready == 0U) && (imu.valid != 0U) &&
            CanMotor_IsOnline(&pitch_motor) && CanMotor_IsOnline(&yaw_motor))
        {
            /* The redesigned IMU installation uses AHRS pitch as physical
             * gimbal Pitch; the previous mechanism used AHRS roll. */
            pitch_imu_zero = imu.pitch;
            yaw_imu_zero = yaw_unwrapped;
            pitch_encoder_zero = pitch_motor.current_angle;
            yaw_encoder_zero = yaw_motor.current_angle;
            pitch_target = pitch_imu_zero;
            yaw_target = yaw_imu_zero;
            MotionReferenceReset(&pitch_reference, pitch_target);
            MotionReferenceReset(&yaw_reference, yaw_target);
            references_ready = 1U;
        }

        uint32_t now = osKernelGetTickCount();
        int command_count = usb->Recv(usb, (char *)command_buffer,
                                      sizeof(command_buffer), 0);
        for (int i = 0; i < command_count; ++i)
        {
#if GIMBAL_LEGACY_ASCII_COMMANDS
            switch (command_buffer[i])
            {
                case 'a': case 'A':
                    yaw_target += GIMBAL_COMMAND_STEP_RAD;
                    control_enabled = 1U;
                    break;
                case 'd': case 'D':
                    yaw_target -= GIMBAL_COMMAND_STEP_RAD;
                    control_enabled = 1U;
                    break;
                case 'w': case 'W':
                    pitch_target += GIMBAL_COMMAND_STEP_RAD;
                    control_enabled = 1U;
                    break;
                case 's': case 'S':
                    pitch_target -= GIMBAL_COMMAND_STEP_RAD;
                    control_enabled = 1U;
                    break;
                case 'x': case 'X': case ' ':
                    control_enabled = 0U;
                    break;
                default:
                    break;
            }
#else
            if (GimbalProtocolParser_Push(&protocol_parser, command_buffer[i],
                                          &protocol_frame) == 0) continue;

            if (protocol_frame.type == GIMBAL_MSG_HELLO)
            {
                hello_received = 1U;
                control_sequence_ready = 0U;
                control_enabled = 0U;
                fire_requested = 0U;
                aim_mode = 0U;
                last_control_tick = now;
            }
            else if ((protocol_frame.type == GIMBAL_MSG_TIME_SYNC_REQ) &&
                     (protocol_frame.length == 8U))
            {
                SendTimeSync(usb, time_sync_sequence++, protocol_frame.payload, now);
            }
            else if ((protocol_frame.type == GIMBAL_MSG_DISARM) ||
                     (protocol_frame.type == GIMBAL_MSG_ESTOP))
            {
                control_enabled = 0U;
                fire_requested = 0U;
                aim_mode = 0U;
            }
            else if ((protocol_frame.type == GIMBAL_MSG_CONTROL) &&
                     (protocol_frame.length == 5U) &&
                     (hello_received != 0U) &&
                     SequenceIsNewer(protocol_frame.sequence,
                                     last_control_sequence,
                                     control_sequence_ready))
            {
                uint8_t flags = protocol_frame.payload[0];
                int16_t yaw_delta_mdeg = GimbalProtocol_ReadI16(
                    &protocol_frame.payload[1]);
                int16_t pitch_delta_mdeg = GimbalProtocol_ReadI16(
                    &protocol_frame.payload[3]);
                last_control_sequence = protocol_frame.sequence;
                control_sequence_ready = 1U;
                last_control_tick = now;
                fire_requested = ((flags & GIMBAL_CONTROL_FIRE) != 0U);
                control_enabled = ((flags & GIMBAL_CONTROL_ARMED) != 0U);
                aim_mode = 0U;
                if (control_enabled != 0U)
                {
                    /* Positive yaw means camera-right; this mechanism's yaw
                     * target decreases in that physical direction. */
                    yaw_target -= (float)yaw_delta_mdeg * GIMBAL_MDEG_TO_RAD;
                    pitch_target += (float)pitch_delta_mdeg * GIMBAL_MDEG_TO_RAD;
                }
            }
            else if ((protocol_frame.type == GIMBAL_MSG_AIM_TARGET) &&
                     (protocol_frame.length == 30U) &&
                     (hello_received != 0U) &&
                     SequenceIsNewer(protocol_frame.sequence,
                                     last_control_sequence,
                                     control_sequence_ready))
            {
                uint8_t flags = protocol_frame.payload[0];
                uint8_t mode = protocol_frame.payload[1];
                uint32_t execute_at = GimbalProtocol_ReadU32(
                    &protocol_frame.payload[2]);
                int32_t time_ahead = (int32_t)(execute_at - now);
                uint8_t time_valid = (time_ahead >= -100) &&
                                     (time_ahead <= 1000);
                last_control_sequence = protocol_frame.sequence;
                control_sequence_ready = 1U;
                last_control_tick = now;
                control_enabled = ((flags & GIMBAL_AIM_VALID) != 0U) &&
                                  ((flags & GIMBAL_AIM_ARMED) != 0U) &&
                                  (mode == 1U) && time_valid;
                fire_requested = control_enabled &&
                                 ((flags & GIMBAL_AIM_FIRE) != 0U);
                aim_mode = control_enabled;
                if (control_enabled != 0U)
                {
                    aim_execute_at_ms = execute_at;
                    aim_yaw_rad = (float)GimbalProtocol_ReadI32(
                        &protocol_frame.payload[6]) * GIMBAL_MDEG_TO_RAD;
                    aim_pitch_rad = (float)GimbalProtocol_ReadI32(
                        &protocol_frame.payload[10]) * GIMBAL_MDEG_TO_RAD;
                    aim_yaw_rate_rad_s = (float)GimbalProtocol_ReadI32(
                        &protocol_frame.payload[14]) * GIMBAL_MDEG_TO_RAD;
                    aim_pitch_rate_rad_s = (float)GimbalProtocol_ReadI32(
                        &protocol_frame.payload[18]) * GIMBAL_MDEG_TO_RAD;
                    aim_yaw_accel_rad_s2 = (float)GimbalProtocol_ReadI32(
                        &protocol_frame.payload[22]) * GIMBAL_MDEG_TO_RAD;
                    aim_pitch_accel_rad_s2 = (float)GimbalProtocol_ReadI32(
                        &protocol_frame.payload[26]) * GIMBAL_MDEG_TO_RAD;
                }
            }
#endif
        }

#if !GIMBAL_LEGACY_ASCII_COMMANDS
        uint8_t link_alive = (hello_received != 0U) &&
                             ((now - last_control_tick) <=
                              GIMBAL_COMMAND_TIMEOUT_MS);
        if ((hello_received != 0U) && (link_alive == 0U))
        {
            control_enabled = 0U;
            fire_requested = 0U;
            hello_received = 0U;
            control_sequence_ready = 0U;
            aim_mode = 0U;
        }
#else
        uint8_t link_alive = 1U;
#endif

        if ((aim_mode != 0U) && (control_enabled != 0U))
        {
            float dt = (float)(int32_t)(now - aim_execute_at_ms) * 0.001f;
            if (dt > 0.2f) dt = 0.2f;
            if (dt < -0.05f) dt = -0.05f;
            yaw_reference.position_rad = yaw_imu_zero + aim_yaw_rad +
                aim_yaw_rate_rad_s * dt + 0.5f * aim_yaw_accel_rad_s2 * dt * dt;
            pitch_reference.position_rad = pitch_imu_zero + aim_pitch_rad +
                aim_pitch_rate_rad_s * dt + 0.5f * aim_pitch_accel_rad_s2 * dt * dt;
            yaw_reference.velocity_rad_s = aim_yaw_rate_rad_s +
                                           aim_yaw_accel_rad_s2 * dt;
            pitch_reference.velocity_rad_s = aim_pitch_rate_rad_s +
                                             aim_pitch_accel_rad_s2 * dt;
            yaw_reference.acceleration_rad_s2 = aim_yaw_accel_rad_s2;
            pitch_reference.acceleration_rad_s2 = aim_pitch_accel_rad_s2;
            yaw_target = yaw_reference.position_rad;
            pitch_target = pitch_reference.position_rad;
            if ((fabsf(yaw_target - yaw_imu_zero) > YAW_ENCODER_LIMIT_RAD) ||
                (fabsf(pitch_target - pitch_imu_zero) > PITCH_ENCODER_LIMIT_RAD))
            {
                control_enabled = 0U;
                fire_requested = 0U;
                aim_mode = 0U;
            }
        }

        pitch_target = fminf(pitch_target,
                             pitch_imu_zero + PITCH_ENCODER_LIMIT_RAD);
        pitch_target = fmaxf(pitch_target,
                             pitch_imu_zero - PITCH_ENCODER_LIMIT_RAD);
        yaw_target = fminf(yaw_target, yaw_imu_zero + YAW_ENCODER_LIMIT_RAD);
        yaw_target = fmaxf(yaw_target, yaw_imu_zero - YAW_ENCODER_LIMIT_RAD);

        float pitch_encoder_delta = pitch_motor.current_angle - pitch_encoder_zero;
        float yaw_encoder_delta = yaw_motor.current_angle - yaw_encoder_zero;
        uint8_t safe = (references_ready != 0U) && (imu.valid != 0U) &&
                       CanMotor_IsOnline(&pitch_motor) && CanMotor_IsOnline(&yaw_motor) &&
                       (pitch_encoder_delta <= PITCH_ENCODER_LIMIT_RAD) &&
                       (pitch_encoder_delta >= -PITCH_ENCODER_LIMIT_RAD) &&
                       (yaw_encoder_delta <= YAW_ENCODER_LIMIT_RAD) &&
                       (yaw_encoder_delta >= -YAW_ENCODER_LIMIT_RAD);

        if ((hello_received != 0U) &&
            ((now - last_status_tick) >= GIMBAL_STATUS_PERIOD_MS))
        {
            uint16_t status_flags = 0U;
            if (imu.valid != 0U) status_flags |= GIMBAL_STATUS_IMU_VALID;
            if (CanMotor_IsOnline(&pitch_motor))
                status_flags |= GIMBAL_STATUS_PITCH_ONLINE;
            if (CanMotor_IsOnline(&yaw_motor))
                status_flags |= GIMBAL_STATUS_YAW_ONLINE;
            if (references_ready != 0U)
                status_flags |= GIMBAL_STATUS_REFERENCES_READY;
            if (control_enabled != 0U)
                status_flags |= GIMBAL_STATUS_CONTROL_ENABLED;
            if (link_alive != 0U) status_flags |= GIMBAL_STATUS_LINK_ALIVE;
            if (fire_requested != 0U)
                status_flags |= GIMBAL_STATUS_FIRE_REQUESTED;
            if (safe != 0U) status_flags |= GIMBAL_STATUS_SAFE;
            SendStatus(usb, status_sequence++, status_flags, &imu,
                       yaw_unwrapped, pitch_imu_zero, yaw_imu_zero,
                       pitch_target, yaw_target, &protocol_parser,
                       last_control_sequence);
            last_status_tick = now;
        }

        if ((hello_received != 0U) &&
            ((now - last_attitude_tick) >= GIMBAL_ATTITUDE_PERIOD_MS))
        {
            uint16_t attitude_flags = 0U;
            if (imu.valid != 0U) attitude_flags |= GIMBAL_STATUS_IMU_VALID;
            if (CanMotor_IsOnline(&pitch_motor))
                attitude_flags |= GIMBAL_STATUS_PITCH_ONLINE;
            if (CanMotor_IsOnline(&yaw_motor))
                attitude_flags |= GIMBAL_STATUS_YAW_ONLINE;
            if (control_enabled != 0U)
                attitude_flags |= GIMBAL_STATUS_CONTROL_ENABLED;
            if (safe != 0U) attitude_flags |= GIMBAL_STATUS_SAFE;
            if (imu.warmup_timed_out != 0U)
                attitude_flags |= GIMBAL_STATUS_IMU_WARMUP_TIMEOUT;
            SendAttitude(usb, attitude_sequence++, attitude_flags, now, &imu,
                         yaw_unwrapped, pitch_imu_zero, yaw_imu_zero);
            last_attitude_tick = now;
        }

        int16_t pitch_output = 0;
        int16_t yaw_output = 0;
        float pitch_ff = 0.0f;
        float yaw_ff = 0.0f;
        if ((control_enabled != 0U) && (safe != 0U))
        {
            if (aim_mode == 0U)
            {
                MotionReferenceUpdate(&pitch_reference, pitch_target,
                                      PITCH_REFERENCE_MAX_VEL_RAD_S,
                                      PITCH_REFERENCE_MAX_ACC_RAD_S2);
                MotionReferenceUpdate(&yaw_reference, yaw_target,
                                      YAW_REFERENCE_MAX_VEL_RAD_S,
                                      YAW_REFERENCE_MAX_ACC_RAD_S2);
            }

            float pitch_position_correction = PID_CalculateDt(
                &pitch_motor.position_pid, pitch_reference.position_rad,
                imu.pitch, GIMBAL_CONTROL_DT_S);
            float yaw_position_correction = PID_CalculateDt(
                &yaw_motor.position_pid, yaw_reference.position_rad,
                yaw_unwrapped, GIMBAL_CONTROL_DT_S);
            float pitch_speed_target = ClampFloat(
                pitch_reference.velocity_rad_s * RAD_S_TO_RPM +
                pitch_position_correction, GIMBAL_SPEED_LIMIT_RPM);
            float yaw_speed_target = ClampFloat(
                yaw_reference.velocity_rad_s * RAD_S_TO_RPM +
                yaw_position_correction, GIMBAL_SPEED_LIMIT_RPM);

            if (fabsf(yaw_speed_target) < YAW_ZERO_TARGET_RPM)
            {
                float decay = expf(-GIMBAL_CONTROL_DT_S /
                                   YAW_INTEGRAL_LEAK_TAU_S);
                yaw_motor.speed_pid.integral *= decay;
            }

            float pitch_pid = PID_CalculateDt(
                &pitch_motor.speed_pid, pitch_speed_target,
                (float)pitch_motor.measure.speed_rpm, GIMBAL_CONTROL_DT_S);
            float yaw_pid = PID_CalculateDt(
                &yaw_motor.speed_pid, yaw_speed_target,
                (float)yaw_motor.measure.speed_rpm, GIMBAL_CONTROL_DT_S);
            pitch_ff = PitchFeedforward(&imu, &pitch_reference);
            yaw_ff = YawFeedforward(yaw_speed_target,
                                    &yaw_friction_command);

            pitch_output = ClampMotorOutput(
                PITCH_OUTPUT_SIGN * (pitch_pid + pitch_ff),
                PITCH_CURRENT_LIMIT);
            yaw_output = ClampMotorOutput(
                YAW_OUTPUT_SIGN * (yaw_pid + yaw_ff),
                YAW_TOTAL_CURRENT_LIMIT);
        }
        else
        {
            PID_Reset(&pitch_motor.position_pid);
            PID_Reset(&pitch_motor.speed_pid);
            PID_Reset(&yaw_motor.position_pid);
            PID_Reset(&yaw_motor.speed_pid);
            yaw_friction_command = 0.0f;
            aim_mode = 0U;
            if (references_ready != 0U)
            {
                pitch_target = imu.pitch;
                yaw_target = yaw_unwrapped;
                MotionReferenceReset(&pitch_reference, pitch_target);
                MotionReferenceReset(&yaw_reference, yaw_target);
            }
            if (safe == 0U) control_enabled = 0U;
        }

        /* 0x205 is slot 1 (Pitch), 0x206 is slot 2 (Yaw). */
        (void)DJI_Motor_SendGroup(&hfdcan1, 0x1FFU,
                                  pitch_output, yaw_output, 0, 0);

        taskENTER_CRITICAL();
        g_gimbal_debug.pitch_ecd = pitch_motor.measure.ecd;
        g_gimbal_debug.pitch_speed = pitch_motor.measure.speed_rpm;
        g_gimbal_debug.pitch_current = pitch_motor.measure.given_current;
        g_gimbal_debug.pitch_temperature = pitch_motor.measure.temperature;
        g_gimbal_debug.pitch_online = CanMotor_IsOnline(&pitch_motor);
        g_gimbal_debug.pitch_output = pitch_output;
        g_gimbal_debug.pitch_target_rad = pitch_target;
        g_gimbal_debug.pitch_reference_rad = pitch_reference.position_rad;
        g_gimbal_debug.pitch_feedforward = pitch_ff;
        g_gimbal_debug.yaw_ecd = yaw_motor.measure.ecd;
        g_gimbal_debug.yaw_speed = yaw_motor.measure.speed_rpm;
        g_gimbal_debug.yaw_current = yaw_motor.measure.given_current;
        g_gimbal_debug.yaw_temperature = yaw_motor.measure.temperature;
        g_gimbal_debug.yaw_online = CanMotor_IsOnline(&yaw_motor);
        g_gimbal_debug.yaw_output = yaw_output;
        g_gimbal_debug.yaw_target_rad = yaw_target;
        g_gimbal_debug.yaw_reference_rad = yaw_reference.position_rad;
        g_gimbal_debug.yaw_feedforward = yaw_ff;
        g_gimbal_debug.references_ready = references_ready;
        g_gimbal_debug.control_enabled = control_enabled;
        g_gimbal_debug.link_alive = link_alive;
        g_gimbal_debug.fire_requested = fire_requested;
        g_gimbal_debug.protocol_crc_errors = protocol_parser.crc_errors;
        ++g_gimbal_debug.loop_count;
        taskEXIT_CRITICAL();

        wake_tick += GIMBAL_CONTROL_PERIOD_TICKS;
        (void)osDelayUntil(wake_tick);
    }
}
