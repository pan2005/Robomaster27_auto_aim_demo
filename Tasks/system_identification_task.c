#include "system_identification_task.h"

#include "bsp_can.h"
#include "bsp_usb_cdc.h"
#include "can_motor.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "imu_task.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SYSID_MAGIC                 0xA55AU
#define SYSID_PROTOCOL_VERSION      4U
#define SYSID_CONTROL_PERIOD_MS     1U
#define SYSID_MAX_CURRENT           2000
#define SYSID_TOTAL_CURRENT_LIMIT   5000.0f
#define SYSID_MAX_SPEED_RPM         60
#define SYSID_YAW_MAX_ANGLE_RAD     (CAN_MOTOR_PI / 2.0f)
#define SYSID_PITCH_MAX_ANGLE_RAD   (30.0f * CAN_MOTOR_PI / 180.0f)
#define SYSID_GRAVITY_MAX_TRAVEL_RAD (40.0f * CAN_MOTOR_PI / 180.0f)
#define SYSID_SUPPORT_MAX_DELTA_RAD (5.0f * CAN_MOTOR_PI / 180.0f)
#define SYSID_SUPPORT_MAX_SPEED_RPM 60
#define SYSID_SUPPORT_CURRENT_LIMIT 5000.0f
#define SYSID_PITCH_POSITION_KP     150.0f
#define SYSID_PITCH_SPEED_KP        250.0f
#define SYSID_PITCH_OUTPUT_SIGN     1.0f
#define SYSID_YAW_POSITION_KP       50.0f
#define SYSID_YAW_SPEED_KP          100.0f
#define SYSID_YAW_OUTPUT_SIGN       1.0f
#define SYSID_PITCH_GRAVITY_KG      (-2298.8448f)
#define SYSID_PITCH_GRAVITY_BIAS    (-99.3683f)
#define SYSID_PITCH_DYNAMIC_KV      1830.4092f
#define SYSID_PITCH_DYNAMIC_KA      50.6137f
#define SYSID_MAX_DURATION_MS       60000U
#define SYSID_MIN_DURATION_MS       100U
#define SYSID_MIN_SWITCH_MS         5U
#define SYSID_MAX_SWITCH_MS         1000U
#define SYSID_TX_QUEUE_LENGTH       128U
#define SYSID_MAX_FRAME_SIZE        64U
#define SYSID_RX_BUFFER_SIZE        128U

#define SYSID_MSG_CONFIG            0x01U
#define SYSID_MSG_START             0x02U
#define SYSID_MSG_STOP              0x03U
#define SYSID_MSG_QUERY             0x04U
#define SYSID_MSG_GRAVITY_CONFIG    0x05U
#define SYSID_MSG_PITCH_CONTROL_CONFIG 0x06U
#define SYSID_MSG_PITCH_PRBS_CONFIG  0x07U
#define SYSID_MSG_ACK               0x81U
#define SYSID_MSG_STATUS            0x82U
#define SYSID_MSG_SAMPLE            0x83U
#define SYSID_MSG_GRAVITY_SAMPLE    0x84U
#define SYSID_MSG_PITCH_CONTROL_SAMPLE 0x85U

#define SYSID_FAULT_MOTOR_OFFLINE   (1U << 0)
#define SYSID_FAULT_IMU_INVALID     (1U << 1)
#define SYSID_FAULT_CAN             (1U << 2)
#define SYSID_FAULT_OVERSPEED       (1U << 3)
#define SYSID_FAULT_ANGLE_LIMIT     (1U << 4)
#define SYSID_FAULT_TIMEOUT         (1U << 5)
#define SYSID_FAULT_HOST_STOP       (1U << 6)
#define SYSID_FAULT_BAD_CONFIG      (1U << 7)
#define SYSID_FAULT_SUPPORT_OFFLINE (1U << 8)
#define SYSID_FAULT_SUPPORT_DEVIATION (1U << 9)
#define SYSID_FAULT_SUPPORT_OVERSPEED (1U << 10)

#if defined(__GNUC__)
#define SYSID_PACKED __attribute__((packed))
#else
#define SYSID_PACKED
#endif

typedef struct SYSID_PACKED
{
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t payload_length;
    uint32_t sequence;
} SysIdFrameHeader_t;

typedef struct SYSID_PACKED
{
    uint8_t axis;
    uint8_t excitation;
    int16_t amplitude;
    uint16_t switch_ms;
    uint32_t duration_ms;
    uint32_t seed;
} SysIdConfigPayload_t;

typedef struct SYSID_PACKED
{
    uint8_t command;
    uint8_t accepted;
    uint8_t state;
    uint16_t reason;
} SysIdAckPayload_t;

typedef struct SYSID_PACKED
{
    uint8_t state;
    uint8_t excitation;
    int16_t amplitude;
    uint16_t switch_ms;
    uint32_t duration_ms;
    uint32_t seed;
    uint32_t dropped_samples;
    uint16_t reason;
} SysIdStatusPayload_t;

typedef struct SYSID_PACKED
{
    int16_t minimum_cdeg;
    int16_t maximum_cdeg;
    int16_t step_cdeg;
    uint16_t ramp_ms;
    uint16_t settle_ms;
    uint8_t repetitions;
} SysIdGravityConfig_t;

typedef struct SYSID_PACKED
{
    uint32_t timestamp_ms;
    uint32_t sample_index;
    int16_t excitation_current;
    int16_t output_current;
    int16_t support_hold_current;
    uint16_t encoder;
    float motor_angle_rad;
    int16_t speed_rpm;
    int16_t feedback_current;
    float imu_yaw_rad;
    float imu_pitch_rad;
    float imu_roll_rad;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
    uint8_t state;
    uint16_t safety_flags;
} SysIdSamplePayload_t;

typedef struct SYSID_PACKED
{
    uint32_t timestamp_ms;
    uint32_t sample_index;
    float target_pitch_rad;
    float imu_pitch_rad;
    float gyro_y_rad_s;
    int16_t output_current;
    int16_t feedback_current;
    uint16_t encoder;
    float motor_angle_rad;
    int16_t yaw_hold_current;
    float imu_yaw_rad;
    uint8_t phase;
    int8_t direction;
    uint8_t state;
    uint16_t safety_flags;
} SysIdGravitySample_t;

typedef struct SYSID_PACKED
{
    uint8_t mode;
    int16_t center_cdeg;
    int16_t amplitude_cdeg;
    uint16_t ramp_ms;
    uint16_t hold_ms;
    uint8_t repetitions;
    uint16_t dynamic_ff_permille;
} SysIdPitchControlConfig_t;

typedef struct SYSID_PACKED
{
    uint32_t timestamp_ms;
    uint32_t sample_index;
    float target_pitch_rad;
    float imu_pitch_rad;
    float gyro_y_rad_s;
    int16_t pid_current;
    int16_t feedforward_current;
    int16_t output_current;
    int16_t feedback_current;
    uint16_t encoder;
    float motor_angle_rad;
    int16_t yaw_hold_current;
    float imu_yaw_rad;
    uint8_t phase;
    uint8_t mode;
    uint8_t state;
    uint16_t safety_flags;
} SysIdPitchControlSample_t;

typedef struct SYSID_PACKED
{
    int16_t operating_cdeg;
    int16_t amplitude;
    uint16_t switch_ms;
    uint32_t excitation_duration_ms;
    uint32_t seed;
    uint16_t move_ms;
    uint16_t settle_ms;
} SysIdPitchPrbsConfig_t;

_Static_assert(sizeof(SysIdFrameHeader_t) == 10U, "Protocol header layout changed");
_Static_assert(sizeof(SysIdConfigPayload_t) == 14U, "Config payload layout changed");
_Static_assert(sizeof(SysIdAckPayload_t) == 5U, "ACK payload layout changed");
_Static_assert(sizeof(SysIdStatusPayload_t) == 20U, "Status payload layout changed");
_Static_assert(sizeof(SysIdGravityConfig_t) == 11U, "Gravity config layout changed");
_Static_assert(sizeof(SysIdSamplePayload_t) == 51U, "Sample payload layout changed");
_Static_assert(sizeof(SysIdGravitySample_t) == 41U, "Gravity sample layout changed");
_Static_assert(sizeof(SysIdPitchControlConfig_t) == 12U, "Pitch control config layout changed");
_Static_assert(sizeof(SysIdPitchControlSample_t) == 45U, "Pitch control sample layout changed");
_Static_assert(sizeof(SysIdPitchPrbsConfig_t) == 18U, "Pitch PRBS config layout changed");

typedef enum
{
    SYSID_MODE_DYNAMIC = 0,
    SYSID_MODE_GRAVITY = 1,
    SYSID_MODE_PITCH_CONTROL = 2,
    SYSID_MODE_PITCH_PRBS_POSITIONED = 3
} SysIdExperimentMode_t;

typedef struct
{
    uint16_t length;
    uint8_t bytes[SYSID_MAX_FRAME_SIZE];
} SysIdTxItem_t;

typedef struct
{
    SysIdState_t state;
    SysIdConfigPayload_t config;
    uint16_t abort_reason;
    uint32_t frame_sequence;
    uint32_t sample_index;
    uint32_t start_tick;
    uint32_t dropped_samples;
    uint32_t lfsr;
    float excitation_motor_zero_rad;
    float support_motor_zero_rad;
    float pitch_target_rad;
    float yaw_target_rad;
    float gravity_start_pitch_rad;
    SysIdGravityConfig_t gravity_config;
    SysIdPitchControlConfig_t pitch_control_config;
    SysIdPitchPrbsConfig_t pitch_prbs_config;
    SysIdExperimentMode_t experiment_mode;
} SysIdContext_t;

static CanMotor_t pitch_motor;
static CanMotor_t yaw_motor;
static SysIdContext_t sysid;
static QueueHandle_t tx_queue;

static void SysIdControlTask(void *argument);
static void SysIdTxTask(void *argument);

static const osThreadAttr_t control_attributes = {
    .name = "SysIdControl",
    .stack_size = 768U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

static const osThreadAttr_t tx_attributes = {
    .name = "SysIdUsbTx",
    .stack_size = 384U * 4U,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

static uint16_t SysIdCrc16(const uint8_t *data, uint16_t length)
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

static uint8_t SysIdQueueFrame(uint8_t type, const void *payload,
                               uint16_t payload_length)
{
    if ((tx_queue == NULL) ||
        ((uint32_t)sizeof(SysIdFrameHeader_t) + payload_length + 2U >
         SYSID_MAX_FRAME_SIZE)) return 0U;

    SysIdTxItem_t item = {0};
    SysIdFrameHeader_t header = {
        .magic = SYSID_MAGIC,
        .version = SYSID_PROTOCOL_VERSION,
        .type = type,
        .payload_length = payload_length,
        .sequence = sysid.frame_sequence++,
    };
    memcpy(item.bytes, &header, sizeof(header));
    if ((payload != NULL) && (payload_length != 0U))
        memcpy(item.bytes + sizeof(header), payload, payload_length);
    uint16_t body_length = (uint16_t)(sizeof(header) + payload_length);
    uint16_t crc = SysIdCrc16(item.bytes, body_length);
    memcpy(item.bytes + body_length, &crc, sizeof(crc));
    item.length = (uint16_t)(body_length + sizeof(crc));

    return (xQueueSend(tx_queue, &item, 0U) == pdPASS) ? 1U : 0U;
}

static void SysIdSendAck(uint8_t command, uint8_t accepted, uint16_t reason)
{
    SysIdAckPayload_t ack = {command, accepted, (uint8_t)sysid.state, reason};
    (void)SysIdQueueFrame(SYSID_MSG_ACK, &ack, sizeof(ack));
}

static void SysIdSendStatus(void)
{
    SysIdStatusPayload_t status = {
        .state = (uint8_t)sysid.state,
        .excitation = sysid.config.excitation,
        .amplitude = sysid.config.amplitude,
        .switch_ms = sysid.config.switch_ms,
        .duration_ms = sysid.config.duration_ms,
        .seed = sysid.config.seed,
        .dropped_samples = sysid.dropped_samples,
        .reason = sysid.abort_reason,
    };
    (void)SysIdQueueFrame(SYSID_MSG_STATUS, &status, sizeof(status));
}

static uint8_t SysIdConfigValid(const SysIdConfigPayload_t *config)
{
    if (config == NULL) return 0U;
    if ((config->axis != 1U) && (config->axis != 2U)) return 0U;
    if ((config->excitation != SYSID_EXCITATION_STEP) &&
        (config->excitation != SYSID_EXCITATION_PRBS)) return 0U;
    if ((config->amplitude == 0) ||
        (config->amplitude > SYSID_MAX_CURRENT) ||
        (config->amplitude < -SYSID_MAX_CURRENT)) return 0U;
    if ((config->duration_ms < SYSID_MIN_DURATION_MS) ||
        (config->duration_ms > SYSID_MAX_DURATION_MS)) return 0U;
    if ((config->excitation == SYSID_EXCITATION_PRBS) &&
        ((config->switch_ms < SYSID_MIN_SWITCH_MS) ||
         (config->switch_ms > SYSID_MAX_SWITCH_MS))) return 0U;
    return 1U;
}

static void SysIdAbort(uint16_t reason)
{
    sysid.abort_reason = reason;
    sysid.state = SYSID_STATE_ABORTED;
    (void)DJI_Motor_SendGroup(&hfdcan1, 0x1FFU, 0, 0, 0, 0);
    PID_Reset(&pitch_motor.position_pid);
    PID_Reset(&pitch_motor.speed_pid);
    PID_Reset(&yaw_motor.position_pid);
    PID_Reset(&yaw_motor.speed_pid);
    SysIdSendStatus();
}

static uint16_t SysIdSafetyFlags(const ImuAttitude_t *imu)
{
    uint16_t flags = 0U;
    BspCanDebug_t can = {0};
    const uint8_t pitch_excitation = (sysid.config.axis == 2U);
    const CanMotor_t *excited_motor = pitch_excitation ? &pitch_motor : &yaw_motor;
    const CanMotor_t *support_motor = pitch_excitation ? &yaw_motor : &pitch_motor;
    const float excitation_limit =
        ((sysid.experiment_mode == SYSID_MODE_GRAVITY) ||
         (sysid.experiment_mode == SYSID_MODE_PITCH_CONTROL) ||
         (sysid.experiment_mode == SYSID_MODE_PITCH_PRBS_POSITIONED))
        ? SYSID_GRAVITY_MAX_TRAVEL_RAD
        : (pitch_excitation ? SYSID_PITCH_MAX_ANGLE_RAD
                            : SYSID_YAW_MAX_ANGLE_RAD);
    BSP_CAN_GetDebug(&hfdcan1, &can);
    if (!CanMotor_IsOnline(excited_motor)) flags |= SYSID_FAULT_MOTOR_OFFLINE;
    if (!CanMotor_IsOnline(support_motor)) flags |= SYSID_FAULT_SUPPORT_OFFLINE;
    if ((imu == NULL) || (imu->valid == 0U)) flags |= SYSID_FAULT_IMU_INVALID;
    if ((can.bus_off != 0U) || (can.error_passive != 0U)) flags |= SYSID_FAULT_CAN;
    if (abs(excited_motor->measure.speed_rpm) > SYSID_MAX_SPEED_RPM)
        flags |= SYSID_FAULT_OVERSPEED;
    if (fabsf(excited_motor->current_angle - sysid.excitation_motor_zero_rad) >
        excitation_limit)
        flags |= SYSID_FAULT_ANGLE_LIMIT;
    if (abs(support_motor->measure.speed_rpm) > SYSID_SUPPORT_MAX_SPEED_RPM)
        flags |= SYSID_FAULT_SUPPORT_OVERSPEED;
    if ((imu != NULL) && (imu->valid != 0U))
    {
        float support_imu_error = pitch_excitation
            ? CanMotor_NormalizeAngle(imu->yaw - sysid.yaw_target_rad)
            : (imu->pitch - sysid.pitch_target_rad);
        if ((fabsf(support_imu_error) > SYSID_SUPPORT_MAX_DELTA_RAD) ||
            (fabsf(support_motor->current_angle - sysid.support_motor_zero_rad) >
             SYSID_SUPPORT_MAX_DELTA_RAD))
            flags |= SYSID_FAULT_SUPPORT_DEVIATION;
    }
    return flags;
}

static int16_t SysIdPitchHoldCurrent(const ImuAttitude_t *imu)
{
    float speed_target = PID_CalculateDt(&pitch_motor.position_pid,
                                         sysid.pitch_target_rad,
                                         imu->pitch, 0.001f);
    float output = SYSID_PITCH_OUTPUT_SIGN *
                   PID_CalculateDt(&pitch_motor.speed_pid, speed_target,
                                   (float)pitch_motor.measure.speed_rpm,
                                   0.001f);
    if (output > SYSID_SUPPORT_CURRENT_LIMIT) output = SYSID_SUPPORT_CURRENT_LIMIT;
    if (output < -SYSID_SUPPORT_CURRENT_LIMIT) output = -SYSID_SUPPORT_CURRENT_LIMIT;
    return (int16_t)output;
}

static int16_t SysIdYawHoldCurrent(const ImuAttitude_t *imu)
{
    float yaw_target = imu->yaw +
        CanMotor_NormalizeAngle(sysid.yaw_target_rad - imu->yaw);
    float speed_target = PID_CalculateDt(&yaw_motor.position_pid,
                                         yaw_target,
                                         imu->yaw, 0.001f);
    float output = SYSID_YAW_OUTPUT_SIGN *
                   PID_CalculateDt(&yaw_motor.speed_pid, speed_target,
                                   (float)yaw_motor.measure.speed_rpm,
                                   0.001f);
    if (output > SYSID_SUPPORT_CURRENT_LIMIT) output = SYSID_SUPPORT_CURRENT_LIMIT;
    if (output < -SYSID_SUPPORT_CURRENT_LIMIT) output = -SYSID_SUPPORT_CURRENT_LIMIT;
    return (int16_t)output;
}

static int16_t SysIdClampTotalCurrent(float output)
{
    if (output > SYSID_TOTAL_CURRENT_LIMIT) output = SYSID_TOTAL_CURRENT_LIMIT;
    if (output < -SYSID_TOTAL_CURRENT_LIMIT) output = -SYSID_TOTAL_CURRENT_LIMIT;
    return (int16_t)output;
}

static uint32_t SysIdLfsrNext(uint32_t value)
{
    uint32_t lsb = value & 1U;
    value >>= 1U;
    if (lsb != 0U) value ^= 0x80200003U;
    return (value != 0U) ? value : 1U;
}

static int16_t SysIdExcitation(uint32_t elapsed_ms)
{
    int16_t amplitude = (sysid.config.amplitude < 0)
                      ? (int16_t)-sysid.config.amplitude
                      : sysid.config.amplitude;
    if (sysid.config.excitation == SYSID_EXCITATION_STEP)
    {
        uint32_t quarter = sysid.config.duration_ms / 4U;
        if (elapsed_ms < quarter) return amplitude;
        if (elapsed_ms < quarter * 2U) return 0;
        if (elapsed_ms < quarter * 3U) return (int16_t)-amplitude;
        return 0;
    }

    if ((sysid.sample_index == 0U) ||
        ((elapsed_ms % sysid.config.switch_ms) == 0U))
        sysid.lfsr = SysIdLfsrNext(sysid.lfsr);
    return ((sysid.lfsr & 1U) != 0U) ? amplitude : (int16_t)-amplitude;
}

static uint8_t SysIdGravityConfigValid(const SysIdGravityConfig_t *config,
                                       uint32_t *duration_ms)
{
    if ((config == NULL) || (duration_ms == NULL)) return 0U;
    if ((config->minimum_cdeg < -4500) || (config->maximum_cdeg > 4500) ||
        (config->minimum_cdeg >= config->maximum_cdeg) ||
        (config->step_cdeg <= 0)) return 0U;
    int32_t span = (int32_t)config->maximum_cdeg - config->minimum_cdeg;
    if ((span > 6000) || ((span % config->step_cdeg) != 0)) return 0U;
    if ((config->ramp_ms < 200U) || (config->ramp_ms > 5000U) ||
        (config->settle_ms < 200U) || (config->settle_ms > 5000U) ||
        (config->repetitions == 0U) || (config->repetitions > 5U)) return 0U;
    uint32_t points = (uint32_t)(span / config->step_cdeg) + 1U;
    /* min -> max -> min -> experiment-start angle */
    uint32_t path_points = 2U * points;
    *duration_ms = path_points * config->repetitions *
                   ((uint32_t)config->ramp_ms + config->settle_ms);
    return (*duration_ms <= SYSID_MAX_DURATION_MS) ? 1U : 0U;
}

static void SysIdGravityTarget(uint32_t elapsed_ms, float *target_rad,
                               uint8_t *phase, int8_t *direction)
{
    const SysIdGravityConfig_t *config = &sysid.gravity_config;
    uint32_t interval = (uint32_t)config->ramp_ms + config->settle_ms;
    uint32_t span = (uint32_t)((int32_t)config->maximum_cdeg -
                               config->minimum_cdeg);
    uint32_t points = span / (uint32_t)config->step_cdeg + 1U;
    uint32_t path_points = 2U * points;
    uint32_t segment = elapsed_ms / interval;
    uint32_t local = segment % path_points;
    float start_cdeg = sysid.gravity_start_pitch_rad *
                       (18000.0f / CAN_MOTOR_PI);
    float next_cdeg;
    if (local < points)
        next_cdeg = (float)config->minimum_cdeg +
                    (float)local * config->step_cdeg;
    else if (local < path_points - 1U)
        next_cdeg = (float)config->minimum_cdeg +
                    (float)(2U * points - 2U - local) * config->step_cdeg;
    else
        next_cdeg = start_cdeg;
    float previous_cdeg;
    if (segment == 0U)
    {
        previous_cdeg = start_cdeg;
    }
    else
    {
        uint32_t previous_local = (segment - 1U) % path_points;
        if (previous_local < points)
            previous_cdeg = (float)config->minimum_cdeg +
                            (float)previous_local * config->step_cdeg;
        else if (previous_local < path_points - 1U)
            previous_cdeg = (float)config->minimum_cdeg +
                            (float)(2U * points - 2U - previous_local) *
                            config->step_cdeg;
        else
            previous_cdeg = start_cdeg;
    }
    int32_t segment_time = (int32_t)(elapsed_ms % interval);
    if ((uint32_t)segment_time < config->ramp_ms)
    {
        float alpha = (float)segment_time / (float)config->ramp_ms;
        *target_rad = (previous_cdeg + (next_cdeg - previous_cdeg) * alpha) *
                      (CAN_MOTOR_PI / 18000.0f);
        *phase = 1U;
    }
    else
    {
        *target_rad = next_cdeg * (CAN_MOTOR_PI / 18000.0f);
        *phase = 2U;
    }
    *direction = (next_cdeg > previous_cdeg) ? 1 :
                 ((next_cdeg < previous_cdeg) ? -1 : 0);
}

static uint8_t SysIdPitchControlConfigValid(
    const SysIdPitchControlConfig_t *config, uint32_t *duration_ms)
{
    if ((config == NULL) || (duration_ms == NULL)) return 0U;
    if ((config->mode < 1U) || (config->mode > 3U) ||
        (config->center_cdeg < -3000) || (config->center_cdeg > 3000) ||
        (config->amplitude_cdeg <= 0) ||
        (config->amplitude_cdeg > 2000) ||
        ((int32_t)config->center_cdeg + config->amplitude_cdeg > 3000) ||
        ((int32_t)config->center_cdeg - config->amplitude_cdeg < -3000) ||
        (config->dynamic_ff_permille > 1200U))
        return 0U;
    if ((config->ramp_ms < 200U) || (config->ramp_ms > 5000U) ||
        (config->hold_ms < 200U) || (config->hold_ms > 5000U) ||
        (config->repetitions == 0U) || (config->repetitions > 5U))
        return 0U;
    *duration_ms = 3U * config->repetitions *
                   ((uint32_t)config->ramp_ms + config->hold_ms);
    return (*duration_ms <= SYSID_MAX_DURATION_MS) ? 1U : 0U;
}

static void SysIdPitchControlTarget(uint32_t elapsed_ms, float *target_rad,
                                    float *target_velocity_rad_s,
                                    float *target_acceleration_rad_s2,
                                    uint8_t *phase)
{
    const SysIdPitchControlConfig_t *config = &sysid.pitch_control_config;
    uint32_t interval = (uint32_t)config->ramp_ms + config->hold_ms;
    uint32_t segment = elapsed_ms / interval;
    uint32_t local = segment % 3U;
    float center = (float)config->center_cdeg;
    float amplitude = (float)config->amplitude_cdeg;
    float next_cdeg = (local == 0U) ? center + amplitude
                     : ((local == 1U) ? center - amplitude : center);
    float previous_cdeg;
    if (segment == 0U)
        previous_cdeg = sysid.gravity_start_pitch_rad *
                        (18000.0f / CAN_MOTOR_PI);
    else
    {
        uint32_t previous_local = (segment - 1U) % 3U;
        previous_cdeg = (previous_local == 0U) ? center + amplitude
                        : ((previous_local == 1U) ? center - amplitude : center);
    }
    uint32_t segment_time = elapsed_ms % interval;
    if (segment_time < config->ramp_ms)
    {
        float ramp_s = (float)config->ramp_ms / 1000.0f;
        float normalized_time = (float)segment_time / (float)config->ramp_ms;
        float phase_rad = CAN_MOTOR_PI * normalized_time;
        float previous_rad = previous_cdeg * (CAN_MOTOR_PI / 18000.0f);
        float delta_rad = (next_cdeg - previous_cdeg) *
                          (CAN_MOTOR_PI / 18000.0f);
        *target_rad = previous_rad +
                      0.5f * delta_rad * (1.0f - cosf(phase_rad));
        *target_velocity_rad_s = 0.5f * delta_rad * CAN_MOTOR_PI /
                                 ramp_s * sinf(phase_rad);
        *target_acceleration_rad_s2 = 0.5f * delta_rad * CAN_MOTOR_PI *
                                      CAN_MOTOR_PI / (ramp_s * ramp_s) *
                                      cosf(phase_rad);
        *phase = 1U;
    }
    else
    {
        *target_rad = next_cdeg * (CAN_MOTOR_PI / 18000.0f);
        *target_velocity_rad_s = 0.0f;
        *target_acceleration_rad_s2 = 0.0f;
        *phase = 2U;
    }
}

static int16_t SysIdPitchGravityFeedforward(const ImuAttitude_t *imu)
{
    float output = SYSID_PITCH_GRAVITY_KG * sinf(imu->pitch) +
                   SYSID_PITCH_GRAVITY_BIAS;
    return SysIdClampTotalCurrent(output);
}

static float SysIdPitchDynamicFeedforward(float target_velocity_rad_s,
                                          float target_acceleration_rad_s2)
{
    float scale = (float)sysid.pitch_control_config.dynamic_ff_permille /
                  1000.0f;
    return scale * (SYSID_PITCH_DYNAMIC_KV * target_velocity_rad_s +
                    SYSID_PITCH_DYNAMIC_KA * target_acceleration_rad_s2);
}

static uint8_t SysIdPitchPrbsConfigValid(const SysIdPitchPrbsConfig_t *config,
                                         uint32_t *total_duration_ms)
{
    if ((config == NULL) || (total_duration_ms == NULL)) return 0U;
    if ((config->operating_cdeg < -2000) ||
        (config->operating_cdeg > 2000) ||
        (config->amplitude <= 0) ||
        (config->amplitude > SYSID_MAX_CURRENT) ||
        (config->switch_ms < SYSID_MIN_SWITCH_MS) ||
        (config->switch_ms > SYSID_MAX_SWITCH_MS) ||
        (config->excitation_duration_ms < SYSID_MIN_DURATION_MS) ||
        (config->move_ms < 500U) || (config->move_ms > 10000U) ||
        (config->settle_ms < 200U) || (config->settle_ms > 5000U))
        return 0U;
    *total_duration_ms = config->excitation_duration_ms +
                         2U * (uint32_t)config->move_ms +
                         2U * (uint32_t)config->settle_ms;
    return (*total_duration_ms <= SYSID_MAX_DURATION_MS) ? 1U : 0U;
}

static uint8_t SysIdPitchPrbsPhase(uint32_t elapsed_ms,
                                   uint32_t *excitation_elapsed_ms)
{
    const SysIdPitchPrbsConfig_t *config = &sysid.pitch_prbs_config;
    uint32_t move_end = config->move_ms;
    uint32_t settle_end = move_end + config->settle_ms;
    uint32_t excitation_end = settle_end + config->excitation_duration_ms;
    uint32_t return_end = excitation_end + config->move_ms;
    float start = sysid.gravity_start_pitch_rad;
    float operating = (float)config->operating_cdeg *
                      (CAN_MOTOR_PI / 18000.0f);
    if (elapsed_ms < move_end)
    {
        float alpha = (float)elapsed_ms / (float)config->move_ms;
        sysid.pitch_target_rad = start + (operating - start) * alpha;
        return 0U;
    }
    if (elapsed_ms < settle_end)
    {
        sysid.pitch_target_rad = operating;
        return 0U;
    }
    if (elapsed_ms < excitation_end)
    {
        sysid.pitch_target_rad = operating;
        *excitation_elapsed_ms = elapsed_ms - settle_end;
        return 1U;
    }
    if (elapsed_ms < return_end)
    {
        float alpha = (float)(elapsed_ms - excitation_end) /
                      (float)config->move_ms;
        sysid.pitch_target_rad = operating + (start - operating) * alpha;
        return 0U;
    }
    sysid.pitch_target_rad = start;
    return 0U;
}

static void SysIdHandleCommand(uint8_t type, const uint8_t *payload,
                               uint16_t length)
{
    if (type == SYSID_MSG_CONFIG)
    {
        SysIdConfigPayload_t config;
        if (length == sizeof(config)) memcpy(&config, payload, sizeof(config));
        if ((length != sizeof(config)) || !SysIdConfigValid(&config) ||
            (sysid.state == SYSID_STATE_RUNNING))
        {
            SysIdSendAck(type, 0U, SYSID_FAULT_BAD_CONFIG);
            return;
        }
        sysid.config = config;
        sysid.experiment_mode = SYSID_MODE_DYNAMIC;
        if (sysid.config.seed == 0U) sysid.config.seed = 1U;
        sysid.abort_reason = 0U;
        sysid.state = SYSID_STATE_ARMED;
        SysIdSendAck(type, 1U, 0U);
    }
    else if (type == SYSID_MSG_GRAVITY_CONFIG)
    {
        SysIdGravityConfig_t config = {0};
        uint32_t duration_ms = 0U;
        if (length == sizeof(config)) memcpy(&config, payload, sizeof(config));
        if ((length != sizeof(config)) ||
            !SysIdGravityConfigValid(&config, &duration_ms) ||
            (sysid.state == SYSID_STATE_RUNNING))
        {
            SysIdSendAck(type, 0U, SYSID_FAULT_BAD_CONFIG);
            return;
        }
        sysid.gravity_config = config;
        sysid.config.axis = 2U;
        sysid.config.excitation = 0U;
        sysid.config.amplitude = 0;
        sysid.config.switch_ms = 0U;
        sysid.config.duration_ms = duration_ms;
        sysid.config.seed = 0U;
        sysid.experiment_mode = SYSID_MODE_GRAVITY;
        sysid.abort_reason = 0U;
        sysid.state = SYSID_STATE_ARMED;
        SysIdSendAck(type, 1U, 0U);
    }
    else if (type == SYSID_MSG_PITCH_CONTROL_CONFIG)
    {
        SysIdPitchControlConfig_t config = {0};
        uint32_t duration_ms = 0U;
        if (length == sizeof(config)) memcpy(&config, payload, sizeof(config));
        if ((length != sizeof(config)) ||
            !SysIdPitchControlConfigValid(&config, &duration_ms) ||
            (sysid.state == SYSID_STATE_RUNNING))
        {
            SysIdSendAck(type, 0U, SYSID_FAULT_BAD_CONFIG);
            return;
        }
        sysid.pitch_control_config = config;
        sysid.config.axis = 2U;
        sysid.config.excitation = config.mode;
        sysid.config.amplitude = config.amplitude_cdeg;
        sysid.config.switch_ms = config.ramp_ms;
        sysid.config.duration_ms = duration_ms;
        sysid.config.seed = 0U;
        sysid.experiment_mode = SYSID_MODE_PITCH_CONTROL;
        sysid.abort_reason = 0U;
        sysid.state = SYSID_STATE_ARMED;
        SysIdSendAck(type, 1U, 0U);
    }
    else if (type == SYSID_MSG_PITCH_PRBS_CONFIG)
    {
        SysIdPitchPrbsConfig_t config = {0};
        uint32_t total_duration_ms = 0U;
        if (length == sizeof(config)) memcpy(&config, payload, sizeof(config));
        if ((length != sizeof(config)) ||
            !SysIdPitchPrbsConfigValid(&config, &total_duration_ms) ||
            (sysid.state == SYSID_STATE_RUNNING))
        {
            SysIdSendAck(type, 0U, SYSID_FAULT_BAD_CONFIG);
            return;
        }
        sysid.pitch_prbs_config = config;
        sysid.config.axis = 2U;
        sysid.config.excitation = SYSID_EXCITATION_PRBS;
        sysid.config.amplitude = config.amplitude;
        sysid.config.switch_ms = config.switch_ms;
        sysid.config.duration_ms = total_duration_ms;
        sysid.config.seed = (config.seed != 0U) ? config.seed : 1U;
        sysid.experiment_mode = SYSID_MODE_PITCH_PRBS_POSITIONED;
        sysid.abort_reason = 0U;
        sysid.state = SYSID_STATE_ARMED;
        SysIdSendAck(type, 1U, 0U);
    }
    else if (type == SYSID_MSG_START)
    {
        ImuAttitude_t imu = {0};
        ImuTask_GetAttitude(&imu);
        if ((length != 0U) || (sysid.state != SYSID_STATE_ARMED))
        {
            SysIdSendAck(type, 0U, SYSID_FAULT_BAD_CONFIG);
            return;
        }
        if (sysid.experiment_mode == SYSID_MODE_PITCH_CONTROL)
        {
            float center_rad = (float)sysid.pitch_control_config.center_cdeg *
                               (CAN_MOTOR_PI / 18000.0f);
            if ((imu.valid == 0U) ||
                (fabsf(imu.pitch - center_rad) >
                 (10.0f * CAN_MOTOR_PI / 180.0f)))
            {
                SysIdSendAck(type, 0U, SYSID_FAULT_ANGLE_LIMIT);
                return;
            }
        }
        if (sysid.experiment_mode == SYSID_MODE_PITCH_PRBS_POSITIONED)
        {
            float operating_rad =
                (float)sysid.pitch_prbs_config.operating_cdeg *
                (CAN_MOTOR_PI / 18000.0f);
            if ((imu.valid == 0U) ||
                (fabsf(imu.pitch - operating_rad) >
                 (25.0f * CAN_MOTOR_PI / 180.0f)))
            {
                SysIdSendAck(type, 0U, SYSID_FAULT_ANGLE_LIMIT);
                return;
            }
        }
        if (sysid.config.axis == 2U)
        {
            sysid.excitation_motor_zero_rad = pitch_motor.current_angle;
            sysid.support_motor_zero_rad = yaw_motor.current_angle;
        }
        else
        {
            sysid.excitation_motor_zero_rad = yaw_motor.current_angle;
            sysid.support_motor_zero_rad = pitch_motor.current_angle;
        }
        sysid.pitch_target_rad = imu.pitch;
        sysid.yaw_target_rad = imu.yaw;
        sysid.gravity_start_pitch_rad = imu.pitch;
        PID_Reset(&pitch_motor.position_pid);
        PID_Reset(&pitch_motor.speed_pid);
        PID_Reset(&yaw_motor.position_pid);
        PID_Reset(&yaw_motor.speed_pid);
        uint16_t flags = SysIdSafetyFlags(&imu);
        if (flags != 0U)
        {
            SysIdSendAck(type, 0U, flags);
            return;
        }
        sysid.sample_index = 0U;
        sysid.dropped_samples = 0U;
        sysid.lfsr = sysid.config.seed;
        sysid.start_tick = HAL_GetTick();
        sysid.abort_reason = 0U;
        sysid.state = SYSID_STATE_RUNNING;
        SysIdSendAck(type, 1U, 0U);
    }
    else if (type == SYSID_MSG_STOP)
    {
        if (sysid.state == SYSID_STATE_RUNNING)
            SysIdAbort(SYSID_FAULT_HOST_STOP);
        else
        {
            sysid.state = SYSID_STATE_IDLE;
            sysid.abort_reason = SYSID_FAULT_HOST_STOP;
            (void)DJI_Motor_SendGroup(&hfdcan1, 0x1FFU, 0, 0, 0, 0);
        }
        SysIdSendAck(type, 1U, SYSID_FAULT_HOST_STOP);
    }
    else if (type == SYSID_MSG_QUERY)
    {
        SysIdSendStatus();
    }
}

static void SysIdParseUsb(void)
{
    static uint8_t buffer[SYSID_RX_BUFFER_SIZE];
    static uint16_t used = 0U;
    struct usb_device *usb = usb_get_device();
    uint8_t incoming[32];
    int received = usb->Recv(usb, (char *)incoming, sizeof(incoming), 0);
    if (received <= 0) return;

    if ((uint32_t)used + (uint32_t)received > sizeof(buffer)) used = 0U;
    memcpy(buffer + used, incoming, (size_t)received);
    used = (uint16_t)(used + received);

    while (used >= sizeof(SysIdFrameHeader_t) + 2U)
    {
        SysIdFrameHeader_t header;
        memcpy(&header, buffer, sizeof(header));
        if ((header.magic != SYSID_MAGIC) ||
            (header.version != SYSID_PROTOCOL_VERSION))
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
        if (received_crc == SysIdCrc16(buffer, (uint16_t)(frame_length - 2U)))
            SysIdHandleCommand(header.type, buffer + sizeof(header),
                               header.payload_length);
        used = (uint16_t)(used - frame_length);
        memmove(buffer, buffer + frame_length, used);
    }
}

void SystemIdentificationTask_Init(void)
{
    ImuTask_SetReportingEnabled(false);
    memset(&sysid, 0, sizeof(sysid));
    sysid.state = SYSID_STATE_IDLE;
    tx_queue = xQueueCreate(SYSID_TX_QUEUE_LENGTH, sizeof(SysIdTxItem_t));
    if (tx_queue == NULL) return;
    (void)osThreadNew(SysIdTxTask, NULL, &tx_attributes);
    (void)osThreadNew(SysIdControlTask, NULL, &control_attributes);
}

static void SysIdControlTask(void *argument)
{
    (void)argument;
    (void)CanMotor_Init(&pitch_motor, &hfdcan1, 0x205U,
                        CAN_MOTOR_GM6020_POSITION, 1.0f);
    (void)CanMotor_Init(&yaw_motor, &hfdcan1, 0x206U,
                        CAN_MOTOR_GM6020_POSITION, 1.0f);
    PID_Init(&pitch_motor.position_pid, SYSID_PITCH_POSITION_KP,
             0.0f, 0.0f, 0.0f, 0.0f, SYSID_SUPPORT_MAX_SPEED_RPM);
    PID_Init(&pitch_motor.speed_pid, SYSID_PITCH_SPEED_KP,
             0.0f, 0.0f, 0.0f, 0.0f, SYSID_SUPPORT_CURRENT_LIMIT);
    PID_Init(&yaw_motor.position_pid, SYSID_YAW_POSITION_KP,
             0.0f, 0.0f, 0.0f, 0.0f, SYSID_SUPPORT_MAX_SPEED_RPM);
    PID_Init(&yaw_motor.speed_pid, SYSID_YAW_SPEED_KP,
             0.0f, 0.0f, 0.0f, 0.0f, SYSID_SUPPORT_CURRENT_LIMIT);
    osDelay(200U);
    uint32_t wake_tick = osKernelGetTickCount();

    for (;;)
    {
        SysIdParseUsb();
        ImuAttitude_t imu = {0};
        ImuTask_GetAttitude(&imu);

        if (sysid.state == SYSID_STATE_RUNNING)
        {
            uint32_t elapsed = HAL_GetTick() - sysid.start_tick;
            uint16_t flags = SysIdSafetyFlags(&imu);
            if (elapsed >= sysid.config.duration_ms)
            {
                (void)DJI_Motor_SendGroup(&hfdcan1, 0x1FFU, 0, 0, 0, 0);
                PID_Reset(&pitch_motor.position_pid);
                PID_Reset(&pitch_motor.speed_pid);
                PID_Reset(&yaw_motor.position_pid);
                PID_Reset(&yaw_motor.speed_pid);
                sysid.state = SYSID_STATE_COMPLETE;
                SysIdSendStatus();
            }
            else if (flags != 0U)
            {
                SysIdAbort(flags);
            }
            else
            {
                if (sysid.experiment_mode == SYSID_MODE_PITCH_CONTROL)
                {
                    uint8_t phase;
                    float target_velocity_rad_s;
                    float target_acceleration_rad_s2;
                    SysIdPitchControlTarget(elapsed, &sysid.pitch_target_rad,
                                            &target_velocity_rad_s,
                                            &target_acceleration_rad_s2,
                                            &phase);
                    int16_t pid_current = SysIdPitchHoldCurrent(&imu);
                    float feedforward = 0.0f;
                    if (sysid.pitch_control_config.mode >= 2U)
                        feedforward += SysIdPitchGravityFeedforward(&imu);
                    if (sysid.pitch_control_config.mode == 3U)
                        feedforward += SysIdPitchDynamicFeedforward(
                            target_velocity_rad_s,
                            target_acceleration_rad_s2);
                    int16_t feedforward_current =
                        SysIdClampTotalCurrent(feedforward);
                    int16_t pitch_output = SysIdClampTotalCurrent(
                        (float)pid_current + (float)feedforward_current);
                    int16_t yaw_output = SysIdYawHoldCurrent(&imu);
                    HAL_StatusTypeDef send_status = DJI_Motor_SendGroup(
                        &hfdcan1, 0x1FFU, pitch_output, yaw_output, 0, 0);
                    if (send_status != HAL_OK)
                    {
                        SysIdAbort(SYSID_FAULT_CAN);
                    }
                    else
                    {
                        SysIdPitchControlSample_t sample = {
                            .timestamp_ms = HAL_GetTick(),
                            .sample_index = sysid.sample_index++,
                            .target_pitch_rad = sysid.pitch_target_rad,
                            .imu_pitch_rad = imu.pitch,
                            .gyro_y_rad_s = imu.gyro[1],
                            .pid_current = pid_current,
                            .feedforward_current = feedforward_current,
                            .output_current = pitch_output,
                            .feedback_current = pitch_motor.measure.given_current,
                            .encoder = pitch_motor.measure.ecd,
                            .motor_angle_rad = pitch_motor.current_angle,
                            .yaw_hold_current = yaw_output,
                            .imu_yaw_rad = imu.yaw,
                            .phase = phase,
                            .mode = sysid.pitch_control_config.mode,
                            .state = (uint8_t)sysid.state,
                            .safety_flags = 0U,
                        };
                        if (!SysIdQueueFrame(SYSID_MSG_PITCH_CONTROL_SAMPLE,
                                             &sample, sizeof(sample)))
                            ++sysid.dropped_samples;
                    }
                }
                else if (sysid.experiment_mode == SYSID_MODE_GRAVITY)
                {
                    uint8_t phase;
                    int8_t direction;
                    SysIdGravityTarget(elapsed, &sysid.pitch_target_rad,
                                       &phase, &direction);
                    int16_t pitch_output = SysIdPitchHoldCurrent(&imu);
                    int16_t yaw_output = SysIdYawHoldCurrent(&imu);
                    HAL_StatusTypeDef send_status = DJI_Motor_SendGroup(
                        &hfdcan1, 0x1FFU, pitch_output, yaw_output, 0, 0);
                    if (send_status != HAL_OK)
                    {
                        SysIdAbort(SYSID_FAULT_CAN);
                    }
                    else
                    {
                        SysIdGravitySample_t sample = {
                            .timestamp_ms = HAL_GetTick(),
                            .sample_index = sysid.sample_index++,
                            .target_pitch_rad = sysid.pitch_target_rad,
                            .imu_pitch_rad = imu.pitch,
                            .gyro_y_rad_s = imu.gyro[1],
                            .output_current = pitch_output,
                            .feedback_current = pitch_motor.measure.given_current,
                            .encoder = pitch_motor.measure.ecd,
                            .motor_angle_rad = pitch_motor.current_angle,
                            .yaw_hold_current = yaw_output,
                            .imu_yaw_rad = imu.yaw,
                            .phase = phase,
                            .direction = direction,
                            .state = (uint8_t)sysid.state,
                            .safety_flags = 0U,
                        };
                        if (!SysIdQueueFrame(SYSID_MSG_GRAVITY_SAMPLE,
                                             &sample, sizeof(sample)))
                            ++sysid.dropped_samples;
                    }
                }
                else
                {
                uint32_t excitation_elapsed = elapsed;
                uint8_t record_sample = 1U;
                if (sysid.experiment_mode == SYSID_MODE_PITCH_PRBS_POSITIONED)
                    record_sample = SysIdPitchPrbsPhase(
                        elapsed, &excitation_elapsed);
                int16_t excitation = (record_sample != 0U)
                                   ? SysIdExcitation(excitation_elapsed) : 0;
                const uint8_t pitch_excitation = (sysid.config.axis == 2U);
                const CanMotor_t *excited_motor = pitch_excitation
                                                ? &pitch_motor : &yaw_motor;
                int16_t support_hold_current;
                int16_t output_current;
                int16_t pitch_output;
                int16_t yaw_output;
                if (pitch_excitation != 0U)
                {
                    int16_t pitch_feedback = SysIdPitchHoldCurrent(&imu);
                    int16_t pitch_gravity = SysIdPitchGravityFeedforward(&imu);
                    support_hold_current = SysIdYawHoldCurrent(&imu);
                    output_current = SysIdClampTotalCurrent(
                        (float)pitch_feedback + (float)pitch_gravity +
                        (float)excitation);
                    pitch_output = output_current;
                    yaw_output = support_hold_current;
                }
                else
                {
                    support_hold_current = SysIdPitchHoldCurrent(&imu);
                    output_current = excitation;
                    pitch_output = support_hold_current;
                    yaw_output = output_current;
                }
                HAL_StatusTypeDef send_status = DJI_Motor_SendGroup(
                    &hfdcan1, 0x1FFU, pitch_output, yaw_output, 0, 0);
                if (send_status != HAL_OK)
                {
                    SysIdAbort(SYSID_FAULT_CAN);
                }
                else
                {
                    if (record_sample != 0U)
                    {
                        SysIdSamplePayload_t sample = {
                        .timestamp_ms = HAL_GetTick(),
                        .sample_index = sysid.sample_index++,
                        .excitation_current = excitation,
                        .output_current = output_current,
                        .support_hold_current = support_hold_current,
                        .encoder = excited_motor->measure.ecd,
                        .motor_angle_rad = excited_motor->current_angle,
                        .speed_rpm = excited_motor->measure.speed_rpm,
                        .feedback_current = excited_motor->measure.given_current,
                        .imu_yaw_rad = imu.yaw,
                        .imu_pitch_rad = imu.pitch,
                        .imu_roll_rad = imu.roll,
                        .gyro_x_rad_s = imu.gyro[0],
                        .gyro_y_rad_s = imu.gyro[1],
                        .gyro_z_rad_s = imu.gyro[2],
                        .state = (uint8_t)sysid.state,
                        .safety_flags = 0U,
                        };
                        if (!SysIdQueueFrame(SYSID_MSG_SAMPLE, &sample,
                                             sizeof(sample)))
                            ++sysid.dropped_samples;
                    }
                }
                }
            }
        }
        else
        {
            (void)DJI_Motor_SendGroup(&hfdcan1, 0x1FFU, 0, 0, 0, 0);
            PID_Reset(&pitch_motor.position_pid);
            PID_Reset(&pitch_motor.speed_pid);
            PID_Reset(&yaw_motor.position_pid);
            PID_Reset(&yaw_motor.speed_pid);
        }

        wake_tick += SYSID_CONTROL_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}

static void SysIdTxTask(void *argument)
{
    (void)argument;
    struct usb_device *usb = usb_get_device();
    SysIdTxItem_t item;
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
