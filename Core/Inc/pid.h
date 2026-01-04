/**
 * @file    pid.h
 * @brief   Generic PID controller
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * TYPES
 * ============================================================================ */
typedef struct {
    /* Gains */
    float Kp;
    float Ki;
    float Kd;
    
    /* Output limits */
    float output_min;
    float output_max;
    
    /* Anti-windup integral limit */
    float integral_max;
    
    /* State */
    float integral;
    float prev_error;
    float output;
    
    /* Setpoint */
    float setpoint;
    
    /* Configuration */
    bool enabled;
} PID_Handle_t;

/* ============================================================================
 * FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize PID controller
 * @param pid Pointer to PID handle
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param out_min Minimum output value
 * @param out_max Maximum output value
 */
void PID_Init(PID_Handle_t *pid, float Kp, float Ki, float Kd, 
              float out_min, float out_max);

/**
 * @brief Set PID gains
 * @param pid Pointer to PID handle
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 */
void PID_SetGains(PID_Handle_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief Set setpoint
 * @param pid Pointer to PID handle
 * @param setpoint Target value
 */
void PID_SetSetpoint(PID_Handle_t *pid, float setpoint);

/**
 * @brief Compute PID output
 * @param pid Pointer to PID handle
 * @param measurement Current measured value
 * @param dt Time step in seconds
 * @return PID output (clamped to limits)
 */
float PID_Compute(PID_Handle_t *pid, float measurement, float dt);

/**
 * @brief Reset PID state (integral and derivative)
 * @param pid Pointer to PID handle
 */
void PID_Reset(PID_Handle_t *pid);

/**
 * @brief Enable/disable PID controller
 * @param pid Pointer to PID handle
 * @param enabled true to enable, false to disable
 */
void PID_SetEnabled(PID_Handle_t *pid, bool enabled);

/**
 * @brief Get last computed output
 * @param pid Pointer to PID handle
 * @return Last output value
 */
float PID_GetOutput(PID_Handle_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */