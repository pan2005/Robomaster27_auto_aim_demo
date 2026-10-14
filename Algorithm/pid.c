#include "pid.h"

#include <stddef.h>

static float Clamp(float value, float limit)
{
    if (limit <= 0.0f) return value;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float kf,
              float integral_limit, float output_limit)
{
    if (pid == NULL) return;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Kf = kf;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) return;
    pid->target = 0.0f;
    pid->current_measure = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->iout = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0U;
}

float PID_CalculateDt(PID_t *pid, float target, float measure, float dt_s)
{
    if ((pid == NULL) || (dt_s <= 0.0f)) return 0.0f;

    pid->target = target;
    pid->current_measure = measure;
    pid->error = target - measure;

    float derivative = 0.0f;
    if (pid->initialized != 0U)
        derivative = (pid->error - pid->last_error) / dt_s;

    float candidate_integral = Clamp(pid->integral + pid->error * dt_s,
                                     pid->integral_limit);
    float candidate_output = pid->Kp * pid->error
                           + pid->Ki * candidate_integral
                           + pid->Kd * derivative
                           + pid->Kf * target;

    /* Conditional integration: do not integrate farther into saturation. */
    if ((pid->output_limit <= 0.0f) ||
        ((candidate_output < pid->output_limit) &&
         (candidate_output > -pid->output_limit)) ||
        ((candidate_output >= pid->output_limit) && (pid->error < 0.0f)) ||
        ((candidate_output <= -pid->output_limit) && (pid->error > 0.0f)))
    {
        pid->integral = candidate_integral;
    }

    pid->iout = pid->Ki * pid->integral;
    pid->output = Clamp(pid->Kp * pid->error + pid->iout
                        + pid->Kd * derivative + pid->Kf * target,
                        pid->output_limit);
    pid->last_error = pid->error;
    pid->initialized = 1U;
    return pid->output;
}

float PID_Calculate(PID_t *pid, float target, float measure)
{
    return PID_CalculateDt(pid, target, measure, 1.0f);
}
