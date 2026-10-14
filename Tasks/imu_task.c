#include "imu_task.h"

#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "bsp_can.h"
#include "bsp_usb_cdc.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>

#define IMU_UPDATE_PERIOD_TICKS  1U
#define IMU_REPORT_DIVIDER       10U
#define IMU_TX_BUFFER_SIZE       320U
#define RAD_TO_DEG               57.29577951308232f
#define GYRO_CALIBRATION_SAMPLES       2000U
#define IMU_WARMUP_TOLERANCE_C         2.0f
#define IMU_WARMUP_STABLE_SAMPLES      100U
#define IMU_WARMUP_TIMEOUT_TICKS       15000U

#define IMU_TARGET_TEMPERATURE   40.0f
#define HEATER_KP                100.0f
#define HEATER_KI                50.0f
#define HEATER_KD                10.0f
#define HEATER_MAX_OUTPUT        500.0f

static void ImuTask_Entry(void *argument);

volatile unsigned int g_imu_task_state = 0U;
static ImuAttitude_t g_imu_attitude;
#if defined(SYSTEM_IDENTIFICATION_MODE) && (SYSTEM_IDENTIFICATION_MODE != 0)
static volatile uint8_t g_imu_reporting_enabled = 0U;
#else
static volatile uint8_t g_imu_reporting_enabled = 1U;
#endif

void ImuTask_SetReportingEnabled(bool enabled)
{
    g_imu_reporting_enabled = enabled ? 1U : 0U;
}

void ImuTask_GetAttitude(ImuAttitude_t *attitude)
{
    if (attitude == NULL) return;
    taskENTER_CRITICAL();
    *attitude = g_imu_attitude;
    taskEXIT_CRITICAL();
}

static float Heater_Update(float temperature)
{
    static float previous_error = 0.0f;
    static float previous_previous_error = 0.0f;
    float error = IMU_TARGET_TEMPERATURE - temperature;
    float output = HEATER_KP * error
                 + HEATER_KI * (error + previous_error + previous_previous_error)
                 + HEATER_KD * (error - previous_error);

    previous_previous_error = previous_error;
    previous_error = error;

    if (output > HEATER_MAX_OUTPUT) output = HEATER_MAX_OUTPUT;
    if (output < 0.0f) output = 0.0f;
    return output;
}

static const osThreadAttr_t imu_task_attributes = {
    .name = "ImuTask",
    /* Official project uses 1024 32-bit stack entries (4096 bytes). */
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityNormal,
};

void ImuTask_Init(void)
{
    (void)osThreadNew(ImuTask_Entry, NULL, &imu_task_attributes);
}

static void ImuTask_Entry(void *argument)
{
    (void)argument;

    struct usb_device *usb = usb_get_device();
    float gyro[3] = {0.0f};
    float gyro_offset[3] = {0.0f};
    float accel[3] = {0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float temperature = 0.0f;
    float calibration_temperature_sum = 0.0f;
    char tx_buffer[2][IMU_TX_BUFFER_SIZE];
    uint32_t tx_index = 0U;
    uint32_t report_counter = 0U;
    uint32_t wake_tick = osKernelGetTickCount();
    float last_yaw = 0.0f;
    float continuous_yaw = 0.0f;
    uint8_t yaw_initialized = 0U;
    GimbalDebug_t gimbal_debug = {0};
    BspCanDebug_t can1_debug = {0};

    uint8_t init_error;
    do
    {
        init_error = BMI088_init();
        if (init_error != BMI088_NO_ERROR)
        {
            g_imu_task_state = 0x100U + init_error;
            osDelay(100U);
        }
    } while (init_error != BMI088_NO_ERROR);

    g_imu_task_state = 1U;

    /* Heat the IMU to its operating point before measuring gyro bias.  A
     * cold-start bias is not valid after the BMI088 reaches 40 degrees C. */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    uint32_t warmup_stable_samples = 0U;
    uint32_t warmup_start_tick = osKernelGetTickCount();
    while ((warmup_stable_samples < IMU_WARMUP_STABLE_SAMPLES) &&
           ((osKernelGetTickCount() - warmup_start_tick) <
            IMU_WARMUP_TIMEOUT_TICKS))
    {
        BMI088_read(gyro, accel, &temperature);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,
                              (uint32_t)Heater_Update(temperature));
        if (fabsf(temperature - IMU_TARGET_TEMPERATURE) <=
            IMU_WARMUP_TOLERANCE_C)
        {
            ++warmup_stable_samples;
        }
        else
        {
            warmup_stable_samples = 0U;
        }
        osDelay(1U);
    }
    uint8_t warmup_timed_out =
        (warmup_stable_samples < IMU_WARMUP_STABLE_SAMPLES) ? 1U : 0U;

    /* Keep the gimbal still while averaging two seconds of gyro samples. */
    for (uint32_t sample = 0U; sample < GYRO_CALIBRATION_SAMPLES; ++sample)
    {
        BMI088_read(gyro, accel, &temperature);
        gyro_offset[0] += gyro[0];
        gyro_offset[1] += gyro[1];
        gyro_offset[2] += gyro[2];
        calibration_temperature_sum += temperature;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,
                              (uint32_t)Heater_Update(temperature));
        osDelay(1U);
    }
    gyro_offset[0] /= (float)GYRO_CALIBRATION_SAMPLES;
    gyro_offset[1] /= (float)GYRO_CALIBRATION_SAMPLES;
    gyro_offset[2] /= (float)GYRO_CALIBRATION_SAMPLES;
    float calibration_temperature = calibration_temperature_sum /
                                    (float)GYRO_CALIBRATION_SAMPLES;
    g_imu_task_state = 2U;

    g_imu_task_state = 3U;
    wake_tick = osKernelGetTickCount();

    for (;;)
    {
        BMI088_read(gyro, accel, &temperature);
        gyro[0] -= gyro_offset[0];
        gyro[1] -= gyro_offset[1];
        gyro[2] -= gyro_offset[2];

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,
                              (uint32_t)Heater_Update(temperature));
        MahonyAHRSupdateIMU(quaternion,
                           gyro[0], gyro[1], gyro[2],
                           accel[0], accel[1], accel[2]);

        float yaw = atan2f(2.0f * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
                           2.0f * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1]) - 1.0f);
        float pitch_arg = -2.0f * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
        if (pitch_arg > 1.0f) pitch_arg = 1.0f;
        if (pitch_arg < -1.0f) pitch_arg = -1.0f;
        float pitch = asinf(pitch_arg);
        float roll = atan2f(2.0f * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]),
                            2.0f * (quaternion[0] * quaternion[0] + quaternion[3] * quaternion[3]) - 1.0f);

        if (yaw_initialized == 0U)
        {
            last_yaw = yaw;
            continuous_yaw = yaw;
            yaw_initialized = 1U;
        }
        else
        {
            float yaw_delta = yaw - last_yaw;
            if (yaw_delta > (float)M_PI) yaw_delta -= 2.0f * (float)M_PI;
            if (yaw_delta < -((float)M_PI)) yaw_delta += 2.0f * (float)M_PI;
            continuous_yaw += yaw_delta;
            last_yaw = yaw;
        }

        taskENTER_CRITICAL();
        g_imu_attitude.yaw = continuous_yaw;
        g_imu_attitude.pitch = pitch;
        g_imu_attitude.roll = roll;
        g_imu_attitude.quaternion[0] = quaternion[0];
        g_imu_attitude.quaternion[1] = quaternion[1];
        g_imu_attitude.quaternion[2] = quaternion[2];
        g_imu_attitude.quaternion[3] = quaternion[3];
        g_imu_attitude.gyro[0] = gyro[0];
        g_imu_attitude.gyro[1] = gyro[1];
        g_imu_attitude.gyro[2] = gyro[2];
        g_imu_attitude.gyro_offset[0] = gyro_offset[0];
        g_imu_attitude.gyro_offset[1] = gyro_offset[1];
        g_imu_attitude.gyro_offset[2] = gyro_offset[2];
        g_imu_attitude.temperature = temperature;
        g_imu_attitude.calibration_temperature = calibration_temperature;
        g_imu_attitude.warmup_timed_out = warmup_timed_out;
        g_imu_attitude.valid = 1U;
        taskEXIT_CRITICAL();

        if (++report_counter >= IMU_REPORT_DIVIDER)
        {
            if (g_imu_reporting_enabled != 0U)
            {
                GimbalTask_GetDebug(&gimbal_debug);
                BSP_CAN_GetDebug(&hfdcan1, &can1_debug);
                char *frame = tx_buffer[tx_index];
                int length = snprintf(frame, IMU_TX_BUFFER_SIZE,
                                  "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,"
                                  "%u,%u,%d,%d,%u,%d,%u,%u,%d,%d,%u,%d,%u,%u,"
                                  "%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%lu\r\n",
                                  continuous_yaw * RAD_TO_DEG, pitch * RAD_TO_DEG, roll * RAD_TO_DEG,
                                  gyro[0], gyro[1], gyro[2],
                                  accel[0], accel[1], accel[2], temperature,
                                  gimbal_debug.pitch_online, gimbal_debug.pitch_ecd,
                                  gimbal_debug.pitch_speed, gimbal_debug.pitch_current,
                                  gimbal_debug.pitch_temperature, gimbal_debug.pitch_output,
                                  gimbal_debug.yaw_online, gimbal_debug.yaw_ecd,
                                  gimbal_debug.yaw_speed, gimbal_debug.yaw_current,
                                  gimbal_debug.yaw_temperature, gimbal_debug.yaw_output,
                                  gimbal_debug.references_ready,
                                  gimbal_debug.control_enabled,
                                  (unsigned long)can1_debug.rx_count,
                                  (unsigned long)can1_debug.last_rx_id,
                                  (unsigned long)can1_debug.tx_ok_count,
                                  (unsigned long)can1_debug.tx_fail_count,
                                  can1_debug.tx_error_count,
                                  can1_debug.rx_error_count,
                                  can1_debug.bus_off,
                                  can1_debug.error_passive,
                                  can1_debug.warning,
                                  can1_debug.activity,
                                  can1_debug.last_error_code,
                                  (unsigned long)gimbal_debug.loop_count);

                if ((length > 0) && (length < (int)IMU_TX_BUFFER_SIZE))
                {
                    while (usb->Send(usb, frame, length) == 0)
                    {
                        osDelay(1U);
                    }
                    tx_index ^= 1U;
                }
            }

            report_counter = 0U;
        }

        wake_tick += IMU_UPDATE_PERIOD_TICKS;
        (void)osDelayUntil(wake_tick);
    }
}
