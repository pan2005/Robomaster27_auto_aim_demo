#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float integral_limit;
    float output_limit;
    float target;
    float current_measure;
    float error;
    float last_error;
    float integral;
    float iout;
    float output;
    unsigned char initialized;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float kf,
              float integral_limit, float output_limit);
void PID_Reset(PID_t *pid);
float PID_CalculateDt(PID_t *pid, float target, float measure, float dt_s);

/* Compatibility helper for old gains that were tuned per update call. */
float PID_Calculate(PID_t *pid, float target, float measure);

#endif /* PID_H */
