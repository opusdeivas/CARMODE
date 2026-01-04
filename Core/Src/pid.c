/**
 * @file    pid.c
 * @brief   PID controller implementation
 */

#include "pid.h"

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

void PID_Init(PID_Handle_t *pid, float Kp, float Ki, float Kd, 
              float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    pid->output_min = out_min;
    pid->output_max = out_max;
    
    /* Set integral limit to prevent windup */
    pid->integral_max = (out_max - out_min) / 2.0f;
    if (Ki > 0.001f) {
        pid->integral_max = pid->integral_max / Ki;
    }
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    pid->enabled = true;
}

void PID_SetGains(PID_Handle_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    /* Recalculate integral limit */
    if (Ki > 0.001f) {
        pid->integral_max = (pid->output_max - pid->output_min) / (2.0f * Ki);
    }
}

void PID_SetSetpoint(PID_Handle_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

float PID_Compute(PID_Handle_t *pid, float measurement, float dt)
{
    if (!pid->enabled || dt <= 0.0f) {
        return pid->output;
    }
    
    /* Calculate error */
    float error = pid->setpoint - measurement;
    
    /* Proportional term */
    float P = pid->Kp * error;
    
    /* Integral term with anti-windup */
    pid->integral += error * dt;
    
    /* Clamp integral */
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    
    float I = pid->Ki * pid->integral;
    
    /* Derivative term (on error) */
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    
    /* Save error for next iteration */
    pid->prev_error = error;
    
    /* Calculate output */
    pid->output = P + I + D;
    
    /* Clamp output */
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    return pid->output;
}

void PID_Reset(PID_Handle_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

void PID_SetEnabled(PID_Handle_t *pid, bool enabled)
{
    pid->enabled = enabled;
    if (! enabled) {
        PID_Reset(pid);
    }
}

float PID_GetOutput(PID_Handle_t *pid)
{
    return pid->output;
}