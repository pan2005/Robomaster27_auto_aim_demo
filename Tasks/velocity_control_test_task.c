#include "velocity_control_test_task.h"

#include "bsp_can.h"
#include "bsp_usb_cdc.h"
#include "can_motor.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "imu_task.h"
#include "pid.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define VTEST_MAGIC                  0xA65AU
#define VTEST_VERSION                3U
#define VTEST_PERIOD_MS              1U
#define VTEST_PID_CURRENT_LIMIT      8000.0f
#define VTEST_TOTAL_CURRENT_LIMIT    16000.0f
#define VTEST_MAX_SPEED_RPM          60
#define VTEST_MAX_TARGET_RPM         30
#define VTEST_MAX_ANGLE_RAD          (CAN_MOTOR_PI / 2.0f)
#define VTEST_MAX_DURATION_MS        60000U
#define VTEST_INTER_RUN_HOLD_MS      5000U
#define VTEST_TX_QUEUE_LENGTH        128U
#define VTEST_MAX_FRAME_SIZE         64U
#define VTEST_RX_BUFFER_SIZE         128U

/* The feedback controller is intentionally identical in all three modes. */
#define VTEST_PID_KP                 80.0f
#define VTEST_PID_KI                 300.0f
#define VTEST_PID_KD                 0.0f
#define VTEST_PID_INTEGRAL_LIMIT     5.0f

/* New mechanical configuration: direction-dependent static feedforward. */
#define VTEST_LINEAR_FF_VELOCITY     100.5f
#define VTEST_POS_FF_FRICTION        734.3f
#define VTEST_POS_FF_VELOCITY        102.1f
#define VTEST_NEG_FF_FRICTION        486.4f
#define VTEST_NEG_FF_VELOCITY        98.9f
#define VTEST_FRICTION_BLEND_RPM     2.5f
#define VTEST_FRICTION_SLEW_PER_S    5000.0f
#define VTEST_ZERO_TARGET_RPM        0.5f
#define VTEST_INTEGRAL_LEAK_TAU_S    0.10f

#define VTEST_PITCH_MAX_DELTA_RAD    (5.0f * CAN_MOTOR_PI / 180.0f)
#define VTEST_PITCH_MAX_SPEED_RPM    60
#define VTEST_PITCH_CURRENT_LIMIT    5000.0f
#define VTEST_PITCH_POSITION_KP      150.0f
#define VTEST_PITCH_SPEED_KP         250.0f
#define VTEST_PITCH_OUTPUT_SIGN      1.0f

#define VTEST_MSG_CONFIG             0x01U
#define VTEST_MSG_START              0x02U
#define VTEST_MSG_STOP               0x03U
#define VTEST_MSG_QUERY              0x04U
#define VTEST_MSG_SET_PARAMS         0x05U
#define VTEST_MSG_GET_PARAMS         0x06U
#define VTEST_MSG_ACK                0x81U
#define VTEST_MSG_STATUS             0x82U
#define VTEST_MSG_SAMPLE             0x83U
#define VTEST_MSG_PARAMS             0x84U

#define VTEST_FAULT_MOTOR_OFFLINE    (1U << 0)
#define VTEST_FAULT_IMU_INVALID      (1U << 1)
#define VTEST_FAULT_CAN              (1U << 2)
#define VTEST_FAULT_OVERSPEED        (1U << 3)
#define VTEST_FAULT_ANGLE_LIMIT      (1U << 4)
#define VTEST_FAULT_HOST_STOP        (1U << 6)
#define VTEST_FAULT_BAD_CONFIG       (1U << 7)
#define VTEST_FAULT_PITCH_OFFLINE    (1U << 8)
#define VTEST_FAULT_PITCH_DEVIATION  (1U << 9)
#define VTEST_FAULT_PITCH_OVERSPEED  (1U << 10)

#if defined(__GNUC__)
#define VTEST_PACKED __attribute__((packed))
#else
#define VTEST_PACKED
#endif

typedef enum
{
    VTEST_STATE_IDLE = 0,
    VTEST_STATE_ARMED = 1,
    VTEST_STATE_RUNNING = 2,
    VTEST_STATE_COMPLETE = 3,
    VTEST_STATE_ABORTED = 4
} VTestState_t;

typedef enum
{
    VTEST_MODE_PID_ONLY = 1,
    VTEST_MODE_VELOCITY_FF = 2,
    VTEST_MODE_FULL_FF = 3
} VTestMode_t;

typedef struct VTEST_PACKED
{
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t payload_length;
    uint32_t sequence;
} VTestHeader_t;

typedef struct VTEST_PACKED
{
    uint8_t mode;
    uint8_t profile;
    int16_t target_rpm;
    uint16_t ramp_ms;
    uint16_t hold_ms;
    uint16_t repetitions;
} VTestConfig_t;

typedef struct VTEST_PACKED
{
    uint8_t command;
    uint8_t accepted;
    uint8_t state;
    uint16_t reason;
} VTestAck_t;

typedef struct VTEST_PACKED
{
    uint8_t state;
    uint8_t mode;
    int16_t target_rpm;
    uint16_t ramp_ms;
    uint16_t hold_ms;
    uint16_t repetitions;
    uint32_t dropped_samples;
    uint16_t reason;
} VTestStatus_t;

typedef struct VTEST_PACKED
{
    float pid_kp;
    float pid_ki;
    float pid_integral_limit;
    float positive_friction;
    float positive_velocity;
    float negative_friction;
    float negative_velocity;
    float friction_blend_rpm;
    float friction_slew_per_s;
    float zero_target_rpm;
    float integral_leak_tau_s;
} VTestParams_t;

typedef struct VTEST_PACKED
{
    uint32_t timestamp_ms;
    uint32_t sample_index;
    float target_rpm;
    float target_accel_rpm_s;
    int16_t speed_rpm;
    int16_t feedback_current;
    float motor_angle_rad;
    int16_t pid_output;
    int16_t friction_ff;
    int16_t velocity_ff;
    int16_t acceleration_ff;
    int16_t output_current;
    int16_t pitch_hold_current;
    float imu_yaw_rad;
    float imu_pitch_rad;
    float gyro_z_rad_s;
    uint8_t state;
    uint16_t safety_flags;
} VTestSample_t;

typedef struct
{
    uint16_t length;
    uint8_t bytes[VTEST_MAX_FRAME_SIZE];
} VTestTxItem_t;

typedef struct
{
    VTestState_t state;
    VTestConfig_t config;
    uint16_t abort_reason;
    uint32_t frame_sequence;
    uint32_t sample_index;
    uint32_t start_tick;
    uint32_t duration_ms;
    uint32_t dropped_samples;
    float motor_zero_rad;
    float pitch_motor_zero_rad;
    float pitch_target_rad;
    float friction_command;
    uint32_t pitch_hold_deadline;
    uint8_t pitch_reference_valid;
    VTestParams_t params;
} VTestContext_t;

_Static_assert(sizeof(VTestHeader_t) == 10U, "VTest header layout changed");
_Static_assert(sizeof(VTestConfig_t) == 10U, "VTest config layout changed");
_Static_assert(sizeof(VTestAck_t) == 5U, "VTest ACK layout changed");
_Static_assert(sizeof(VTestStatus_t) == 16U, "VTest status layout changed");
_Static_assert(sizeof(VTestParams_t) == 44U, "VTest params layout changed");
_Static_assert(sizeof(VTestSample_t) == 51U, "VTest sample layout changed");

static CanMotor_t pitch_motor;
static CanMotor_t yaw_motor;
static PID_t speed_pid;
static VTestContext_t context;
static QueueHandle_t tx_queue;

static void VTestControlTask(void *argument);
static void VTestTxTask(void *argument);

static const osThreadAttr_t control_attributes = {
    .name = "VelocityTest",
    .stack_size = 768U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

static const osThreadAttr_t tx_attributes = {
    .name = "VelocityUsbTx",
    .stack_size = 384U * 4U,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

static float VTestClamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float VTestSlew(float current, float target, float max_step)
{
    return current + VTestClamp(target - current, max_step);
}

static VTestParams_t VTestDefaultParams(void)
{
    const VTestParams_t params = {
        .pid_kp = VTEST_PID_KP,
        .pid_ki = VTEST_PID_KI,
        .pid_integral_limit = VTEST_PID_INTEGRAL_LIMIT,
        .positive_friction = VTEST_POS_FF_FRICTION,
        .positive_velocity = VTEST_POS_FF_VELOCITY,
        .negative_friction = VTEST_NEG_FF_FRICTION,
        .negative_velocity = VTEST_NEG_FF_VELOCITY,
        .friction_blend_rpm = VTEST_FRICTION_BLEND_RPM,
        .friction_slew_per_s = VTEST_FRICTION_SLEW_PER_S,
        .zero_target_rpm = VTEST_ZERO_TARGET_RPM,
        .integral_leak_tau_s = VTEST_INTEGRAL_LEAK_TAU_S,
    };
    return params;
}

static uint8_t VTestParamsValid(const VTestParams_t *params)
{
    if (params == NULL) return 0U;
    if (!isfinite(params->pid_kp) || (params->pid_kp < 0.0f) ||
        (params->pid_kp > 500.0f)) return 0U;
    if (!isfinite(params->pid_ki) || (params->pid_ki < 0.0f) ||
        (params->pid_ki > 2000.0f)) return 0U;
    if (!isfinite(params->pid_integral_limit) ||
        (params->pid_integral_limit < 0.0f) ||
        (params->pid_integral_limit > 20.0f)) return 0U;
    if (!isfinite(params->positive_friction) ||
        (params->positive_friction < 0.0f) ||
        (params->positive_friction > 3000.0f)) return 0U;
    if (!isfinite(params->negative_friction) ||
        (params->negative_friction < 0.0f) ||
        (params->negative_friction > 3000.0f)) return 0U;
    if (!isfinite(params->positive_velocity) ||
        (params->positive_velocity < 0.0f) ||
        (params->positive_velocity > 300.0f)) return 0U;
    if (!isfinite(params->negative_velocity) ||
        (params->negative_velocity < 0.0f) ||
        (params->negative_velocity > 300.0f)) return 0U;
    if (!isfinite(params->friction_blend_rpm) ||
        (params->friction_blend_rpm < 0.1f) ||
        (params->friction_blend_rpm > 10.0f)) return 0U;
    if (!isfinite(params->friction_slew_per_s) ||
        (params->friction_slew_per_s < 100.0f) ||
        (params->friction_slew_per_s > 30000.0f)) return 0U;
    if (!isfinite(params->zero_target_rpm) ||
        (params->zero_target_rpm < 0.0f) ||
        (params->zero_target_rpm > 3.0f)) return 0U;
    if (!isfinite(params->integral_leak_tau_s) ||
        (params->integral_leak_tau_s < 0.01f) ||
        (params->integral_leak_tau_s > 2.0f)) return 0U;
    return 1U;
}

static void VTestApplyParams(const VTestParams_t *params)
{
    context.params = *params;
    speed_pid.Kp = params->pid_kp;
    speed_pid.Ki = params->pid_ki;
    speed_pid.integral_limit = params->pid_integral_limit;
    PID_Reset(&speed_pid);
    context.friction_command = 0.0f;
}

static uint16_t VTestCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                  : (uint16_t)(crc << 1U);
    }
    return crc;
}

static uint8_t VTestQueueFrame(uint8_t type, const void *payload, uint16_t length)
{
    if ((tx_queue == NULL) ||
        ((uint32_t)sizeof(VTestHeader_t) + length + 2U > VTEST_MAX_FRAME_SIZE))
        return 0U;

    VTestTxItem_t item = {0};
    VTestHeader_t header = {
        VTEST_MAGIC, VTEST_VERSION, type, length, context.frame_sequence++
    };
    memcpy(item.bytes, &header, sizeof(header));
    if ((payload != NULL) && (length != 0U))
        memcpy(item.bytes + sizeof(header), payload, length);
    uint16_t body_length = (uint16_t)(sizeof(header) + length);
    uint16_t crc = VTestCrc16(item.bytes, body_length);
    memcpy(item.bytes + body_length, &crc, sizeof(crc));
    item.length = (uint16_t)(body_length + 2U);
    return (xQueueSend(tx_queue, &item, 0U) == pdPASS) ? 1U : 0U;
}

static void VTestSendAck(uint8_t command, uint8_t accepted, uint16_t reason)
{
    VTestAck_t ack = {command, accepted, (uint8_t)context.state, reason};
    (void)VTestQueueFrame(VTEST_MSG_ACK, &ack, sizeof(ack));
}

static void VTestSendStatus(void)
{
    VTestStatus_t status = {
        (uint8_t)context.state, context.config.mode, context.config.target_rpm,
        context.config.ramp_ms, context.config.hold_ms,
        context.config.repetitions, context.dropped_samples,
        context.abort_reason
    };
    (void)VTestQueueFrame(VTEST_MSG_STATUS, &status, sizeof(status));
}

static uint8_t VTestConfigValid(const VTestConfig_t *config,
                                uint32_t *duration_ms)
{
    if ((config == NULL) || (duration_ms == NULL)) return 0U;
    if ((config->mode < VTEST_MODE_PID_ONLY) ||
        (config->mode > VTEST_MODE_FULL_FF) || (config->profile != 1U)) return 0U;
    if ((config->target_rpm <= 0) ||
        (config->target_rpm > VTEST_MAX_TARGET_RPM)) return 0U;
    if ((config->ramp_ms < 50U) || (config->ramp_ms > 2000U) ||
        (config->hold_ms < 100U) || (config->hold_ms > 5000U) ||
        (config->repetitions == 0U) || (config->repetitions > 20U)) return 0U;
    uint32_t one_cycle = 4U * ((uint32_t)config->ramp_ms + config->hold_ms);
    *duration_ms = one_cycle * config->repetitions;
    return (*duration_ms <= VTEST_MAX_DURATION_MS) ? 1U : 0U;
}

static uint16_t VTestSafetyFlags(const ImuAttitude_t *imu)
{
    uint16_t flags = 0U;
    BspCanDebug_t can = {0};
    BSP_CAN_GetDebug(&hfdcan1, &can);
    if (!CanMotor_IsOnline(&yaw_motor)) flags |= VTEST_FAULT_MOTOR_OFFLINE;
    if (!CanMotor_IsOnline(&pitch_motor)) flags |= VTEST_FAULT_PITCH_OFFLINE;
    if ((imu == NULL) || (imu->valid == 0U)) flags |= VTEST_FAULT_IMU_INVALID;
    if ((can.bus_off != 0U) || (can.error_passive != 0U)) flags |= VTEST_FAULT_CAN;
    if (abs(yaw_motor.measure.speed_rpm) > VTEST_MAX_SPEED_RPM)
        flags |= VTEST_FAULT_OVERSPEED;
    if (fabsf(yaw_motor.current_angle - context.motor_zero_rad) > VTEST_MAX_ANGLE_RAD)
        flags |= VTEST_FAULT_ANGLE_LIMIT;
    if (abs(pitch_motor.measure.speed_rpm) > VTEST_PITCH_MAX_SPEED_RPM)
        flags |= VTEST_FAULT_PITCH_OVERSPEED;
    if ((imu != NULL) && (imu->valid != 0U) &&
        ((fabsf(imu->pitch - context.pitch_target_rad) >
          VTEST_PITCH_MAX_DELTA_RAD) ||
         (fabsf(pitch_motor.current_angle - context.pitch_motor_zero_rad) >
          VTEST_PITCH_MAX_DELTA_RAD)))
        flags |= VTEST_FAULT_PITCH_DEVIATION;
    return flags;
}

static void VTestStopMotor(void)
{
    (void)DJI_Motor_SendGroup(&hfdcan1, 0x1FFU, 0, 0, 0, 0);
    PID_Reset(&pitch_motor.position_pid);
    PID_Reset(&pitch_motor.speed_pid);
    context.friction_command = 0.0f;
}

static int16_t VTestPitchHoldCurrent(const ImuAttitude_t *imu)
{
    float speed_target = PID_CalculateDt(&pitch_motor.position_pid,
                                         context.pitch_target_rad,
                                         imu->pitch, 0.001f);
    float output = VTEST_PITCH_OUTPUT_SIGN *
                   PID_CalculateDt(&pitch_motor.speed_pid, speed_target,
                                   (float)pitch_motor.measure.speed_rpm,
                                   0.001f);
    return (int16_t)VTestClamp(output, VTEST_PITCH_CURRENT_LIMIT);
}

static void VTestAbort(uint16_t reason)
{
    context.abort_reason = reason;
    context.state = VTEST_STATE_ABORTED;
    PID_Reset(&speed_pid);
    context.pitch_reference_valid = 0U;
    VTestStopMotor();
    VTestSendStatus();
}

/* One cycle is: ramp +V, hold +V, ramp to 0, hold 0,
 * ramp -V, hold -V, ramp to 0, hold 0. */
static void VTestTarget(uint32_t elapsed_ms, float *target_rpm,
                        float *target_accel_rpm_s)
{
    uint32_t ramp = context.config.ramp_ms;
    uint32_t hold = context.config.hold_ms;
    uint32_t cycle = 4U * (ramp + hold);
    uint32_t t = elapsed_ms % cycle;
    float velocity = (float)context.config.target_rpm;
    float accel = velocity * 1000.0f / (float)ramp;
    float target;
    float target_accel;

    if (t < ramp)
    {
        target = velocity * (float)t / (float)ramp;
        target_accel = accel;
    }
    else if ((t -= ramp) < hold)
    {
        target = velocity;
        target_accel = 0.0f;
    }
    else if ((t -= hold) < ramp)
    {
        target = velocity * (1.0f - (float)t / (float)ramp);
        target_accel = -accel;
    }
    else if ((t -= ramp) < hold)
    {
        target = 0.0f;
        target_accel = 0.0f;
    }
    else if ((t -= hold) < ramp)
    {
        target = -velocity * (float)t / (float)ramp;
        target_accel = -accel;
    }
    else if ((t -= ramp) < hold)
    {
        target = -velocity;
        target_accel = 0.0f;
    }
    else if ((t -= hold) < ramp)
    {
        target = -velocity * (1.0f - (float)t / (float)ramp);
        target_accel = accel;
    }
    else
    {
        target = 0.0f;
        target_accel = 0.0f;
    }

    *target_rpm = target;
    *target_accel_rpm_s = target_accel;
}

/* Feedforward is deliberately kept outside PID. Each physical contribution
 * is visible and logged independently, so its benefit can be measured rather
 * than hidden inside a single opaque gain. */
static float VTestFeedforward(VTestMode_t mode, float target_rpm,
                              float target_accel_rpm_s,
                              float *friction, float *velocity,
                              float *acceleration)
{
    *friction = 0.0f;
    *velocity = 0.0f;
    *acceleration = 0.0f;

    if (mode == VTEST_MODE_VELOCITY_FF)
    {
        *velocity = VTEST_LINEAR_FF_VELOCITY * target_rpm;
    }
    else if (mode == VTEST_MODE_FULL_FF)
    {
        float magnitude = fabsf(target_rpm);
        float blend = tanhf(magnitude / context.params.friction_blend_rpm);
        float friction_target = 0.0f;
        if (target_rpm > 0.0f)
        {
            friction_target = context.params.positive_friction * blend;
            *velocity = context.params.positive_velocity * target_rpm;
        }
        else if (target_rpm < 0.0f)
        {
            friction_target = -context.params.negative_friction * blend;
            *velocity = context.params.negative_velocity * target_rpm;
        }
        context.friction_command = VTestSlew(
            context.friction_command, friction_target,
            context.params.friction_slew_per_s * 0.001f);
        *friction = context.friction_command;
        /* Keep acceleration FF disabled in this comparison so that the
         * measured difference is solely the direction-dependent static map. */
        (void)target_accel_rpm_s;
    }
    else
    {
        context.friction_command = 0.0f;
    }

    return *friction + *velocity + *acceleration;
}

static void VTestHandleCommand(uint8_t type, const uint8_t *payload,
                               uint16_t length)
{
    if (type == VTEST_MSG_CONFIG)
    {
        VTestConfig_t config = {0};
        uint32_t duration = 0U;
        if (length == sizeof(config)) memcpy(&config, payload, sizeof(config));
        if ((length != sizeof(config)) ||
            !VTestConfigValid(&config, &duration) ||
            (context.state == VTEST_STATE_RUNNING))
        {
            VTestSendAck(type, 0U, VTEST_FAULT_BAD_CONFIG);
            return;
        }
        context.config = config;
        context.duration_ms = duration;
        context.abort_reason = 0U;
        context.state = VTEST_STATE_ARMED;
        VTestSendAck(type, 1U, 0U);
    }
    else if (type == VTEST_MSG_START)
    {
        ImuAttitude_t imu = {0};
        ImuTask_GetAttitude(&imu);
        if ((length != 0U) || (context.state != VTEST_STATE_ARMED))
        {
            VTestSendAck(type, 0U, VTEST_FAULT_BAD_CONFIG);
            return;
        }
        context.motor_zero_rad = yaw_motor.current_angle;
        context.pitch_motor_zero_rad = pitch_motor.current_angle;
        context.pitch_target_rad = imu.pitch;
        context.pitch_reference_valid = 1U;
        context.pitch_hold_deadline = HAL_GetTick() + VTEST_INTER_RUN_HOLD_MS;
        PID_Reset(&pitch_motor.position_pid);
        PID_Reset(&pitch_motor.speed_pid);
        uint16_t flags = VTestSafetyFlags(&imu);
        if (flags != 0U)
        {
            VTestSendAck(type, 0U, flags);
            return;
        }
        context.sample_index = 0U;
        context.dropped_samples = 0U;
        context.start_tick = HAL_GetTick();
        context.abort_reason = 0U;
        PID_Reset(&speed_pid);
        context.friction_command = 0.0f;
        context.state = VTEST_STATE_RUNNING;
        VTestSendAck(type, 1U, 0U);
    }
    else if (type == VTEST_MSG_STOP)
    {
        if (context.state == VTEST_STATE_RUNNING)
            VTestAbort(VTEST_FAULT_HOST_STOP);
        else
        {
            context.state = VTEST_STATE_IDLE;
            context.abort_reason = VTEST_FAULT_HOST_STOP;
            context.pitch_reference_valid = 0U;
            VTestStopMotor();
        }
        VTestSendAck(type, 1U, VTEST_FAULT_HOST_STOP);
    }
    else if (type == VTEST_MSG_SET_PARAMS)
    {
        VTestParams_t params = {0};
        if (length == sizeof(params)) memcpy(&params, payload, sizeof(params));
        if ((length != sizeof(params)) || !VTestParamsValid(&params) ||
            (context.state == VTEST_STATE_RUNNING))
        {
            VTestSendAck(type, 0U, VTEST_FAULT_BAD_CONFIG);
            return;
        }
        VTestApplyParams(&params);
        VTestSendAck(type, 1U, 0U);
        (void)VTestQueueFrame(VTEST_MSG_PARAMS, &context.params,
                              sizeof(context.params));
    }
    else if (type == VTEST_MSG_GET_PARAMS)
    {
        if (length != 0U)
        {
            VTestSendAck(type, 0U, VTEST_FAULT_BAD_CONFIG);
            return;
        }
        VTestSendAck(type, 1U, 0U);
        (void)VTestQueueFrame(VTEST_MSG_PARAMS, &context.params,
                              sizeof(context.params));
    }
    else if (type == VTEST_MSG_QUERY)
    {
        VTestSendStatus();
    }
}

static void VTestParseUsb(void)
{
    static uint8_t buffer[VTEST_RX_BUFFER_SIZE];
    static uint16_t used = 0U;
    uint8_t incoming[32];
    struct usb_device *usb = usb_get_device();
    int received = usb->Recv(usb, (char *)incoming, sizeof(incoming), 0);
    if (received <= 0) return;

    if ((uint32_t)used + (uint32_t)received > sizeof(buffer)) used = 0U;
    memcpy(buffer + used, incoming, (size_t)received);
    used = (uint16_t)(used + received);

    while (used >= sizeof(VTestHeader_t) + 2U)
    {
        VTestHeader_t header;
        memcpy(&header, buffer, sizeof(header));
        if ((header.magic != VTEST_MAGIC) || (header.version != VTEST_VERSION))
        {
            memmove(buffer, buffer + 1U, --used);
            continue;
        }
        uint32_t frame_length = sizeof(header) + header.payload_length + 2U;
        if (frame_length > sizeof(buffer))
        {
            memmove(buffer, buffer + 1U, --used);
            continue;
        }
        if (used < frame_length) break;
        uint16_t received_crc;
        memcpy(&received_crc, buffer + frame_length - 2U, 2U);
        if (received_crc == VTestCrc16(buffer, (uint16_t)(frame_length - 2U)))
            VTestHandleCommand(header.type, buffer + sizeof(header),
                               header.payload_length);
        used = (uint16_t)(used - frame_length);
        memmove(buffer, buffer + frame_length, used);
    }
}

void VelocityControlTestTask_Init(void)
{
    ImuTask_SetReportingEnabled(false);
    memset(&context, 0, sizeof(context));
    context.params = VTestDefaultParams();
    context.state = VTEST_STATE_IDLE;
    tx_queue = xQueueCreate(VTEST_TX_QUEUE_LENGTH, sizeof(VTestTxItem_t));
    if (tx_queue == NULL) return;
    (void)osThreadNew(VTestTxTask, NULL, &tx_attributes);
    (void)osThreadNew(VTestControlTask, NULL, &control_attributes);
}

static void VTestControlTask(void *argument)
{
    (void)argument;
    (void)CanMotor_Init(&pitch_motor, &hfdcan1, 0x205U,
                        CAN_MOTOR_GM6020_POSITION, 1.0f);
    (void)CanMotor_Init(&yaw_motor, &hfdcan1, 0x206U,
                        CAN_MOTOR_GM6020_POSITION, 1.0f);
    PID_Init(&pitch_motor.position_pid, VTEST_PITCH_POSITION_KP,
             0.0f, 0.0f, 0.0f, 0.0f, VTEST_PITCH_MAX_SPEED_RPM);
    PID_Init(&pitch_motor.speed_pid, VTEST_PITCH_SPEED_KP,
             0.0f, 0.0f, 0.0f, 0.0f, VTEST_PITCH_CURRENT_LIMIT);
    PID_Init(&speed_pid, context.params.pid_kp, context.params.pid_ki,
             VTEST_PID_KD, 0.0f, context.params.pid_integral_limit,
             VTEST_PID_CURRENT_LIMIT);
    osDelay(200U);
    uint32_t wake_tick = osKernelGetTickCount();

    for (;;)
    {
        VTestParseUsb();
        ImuAttitude_t imu = {0};
        ImuTask_GetAttitude(&imu);

        if (context.state == VTEST_STATE_RUNNING)
        {
            uint32_t elapsed = HAL_GetTick() - context.start_tick;
            uint16_t flags = VTestSafetyFlags(&imu);
            if (elapsed >= context.duration_ms)
            {
                PID_Reset(&speed_pid);
                context.friction_command = 0.0f;
                context.pitch_hold_deadline = HAL_GetTick() +
                                              VTEST_INTER_RUN_HOLD_MS;
                context.state = VTEST_STATE_COMPLETE;
                VTestSendStatus();
            }
            else if (flags != 0U)
            {
                VTestAbort(flags);
            }
            else
            {
                float target_rpm;
                float target_accel;
                VTestTarget(elapsed, &target_rpm, &target_accel);
                if (fabsf(target_rpm) < context.params.zero_target_rpm)
                {
                    float decay = expf(-0.001f /
                                       context.params.integral_leak_tau_s);
                    speed_pid.integral *= decay;
                }
                float pid_output = PID_CalculateDt(
                    &speed_pid, target_rpm,
                    (float)yaw_motor.measure.speed_rpm, 0.001f);
                float friction_ff;
                float velocity_ff;
                float acceleration_ff;
                float feedforward = VTestFeedforward(
                    (VTestMode_t)context.config.mode, target_rpm, target_accel,
                    &friction_ff, &velocity_ff, &acceleration_ff);
                int16_t output = (int16_t)VTestClamp(
                    pid_output + feedforward, VTEST_TOTAL_CURRENT_LIMIT);
                int16_t pitch_hold_current = VTestPitchHoldCurrent(&imu);

                if (DJI_Motor_SendGroup(&hfdcan1, 0x1FFU,
                                        pitch_hold_current, output, 0, 0)
                    != HAL_OK)
                {
                    VTestAbort(VTEST_FAULT_CAN);
                }
                else
                {
                    VTestSample_t sample = {
                        .timestamp_ms = HAL_GetTick(),
                        .sample_index = context.sample_index++,
                        .target_rpm = target_rpm,
                        .target_accel_rpm_s = target_accel,
                        .speed_rpm = yaw_motor.measure.speed_rpm,
                        .feedback_current = yaw_motor.measure.given_current,
                        .motor_angle_rad = yaw_motor.current_angle,
                        .pid_output = (int16_t)pid_output,
                        .friction_ff = (int16_t)friction_ff,
                        .velocity_ff = (int16_t)velocity_ff,
                        .acceleration_ff = (int16_t)acceleration_ff,
                        .output_current = output,
                        .pitch_hold_current = pitch_hold_current,
                        .imu_yaw_rad = imu.yaw,
                        .imu_pitch_rad = imu.pitch,
                        .gyro_z_rad_s = imu.gyro[2],
                        .state = (uint8_t)context.state,
                        .safety_flags = 0U,
                    };
                    if (!VTestQueueFrame(VTEST_MSG_SAMPLE, &sample, sizeof(sample)))
                        ++context.dropped_samples;
                }
            }
        }
        else
        {
            uint32_t now = HAL_GetTick();
            uint8_t inter_run_hold =
                ((context.state == VTEST_STATE_COMPLETE) ||
                 (context.state == VTEST_STATE_ARMED)) &&
                (context.pitch_reference_valid != 0U) &&
                ((int32_t)(context.pitch_hold_deadline - now) > 0) &&
                (imu.valid != 0U) && CanMotor_IsOnline(&pitch_motor) &&
                (fabsf(imu.pitch - context.pitch_target_rad) <=
                 VTEST_PITCH_MAX_DELTA_RAD) &&
                (fabsf(pitch_motor.current_angle -
                        context.pitch_motor_zero_rad) <=
                 VTEST_PITCH_MAX_DELTA_RAD);
            if (inter_run_hold != 0U)
            {
                int16_t pitch_hold_current = VTestPitchHoldCurrent(&imu);
                (void)DJI_Motor_SendGroup(&hfdcan1, 0x1FFU,
                                          pitch_hold_current, 0, 0, 0);
            }
            else
            {
                context.pitch_reference_valid = 0U;
                VTestStopMotor();
            }
        }

        wake_tick += VTEST_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}

static void VTestTxTask(void *argument)
{
    (void)argument;
    struct usb_device *usb = usb_get_device();
    VTestTxItem_t item;
    osDelay(250U);
    for (;;)
    {
        if (xQueueReceive(tx_queue, &item, osWaitForever) == pdPASS)
        {
            while (usb->Send(usb, (char *)item.bytes, item.length) == 0)
                osDelay(1U);
        }
    }
}
