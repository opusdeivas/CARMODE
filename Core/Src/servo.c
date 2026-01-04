/**
 * @file    servo.c
 * @brief   Servo control with Ackermann geometry implementation
 */

#include "servo.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================================
 * SERVO CONTROL
 * ============================================================================ */

void Servo_Init(Servo_Handle_t *servo, TIM_HandleTypeDef *htim, uint32_t channel)
{
    servo->htim = htim;
    servo->channel = channel;
    
    /* Set calibration values from config */
    servo->center_ccr = SERVO_CENTER_CCR;
    servo->ccr_per_degree = SERVO_CCR_PER_DEGREE;
    servo->max_angle_sw = SERVO_MAX_ANGLE_SW;
    servo->linkage_ratio = SERVO_LINKAGE_RATIO;
    
    servo->current_angle_sw = 0;
    servo->current_angle_wheel = 0.0f;
    servo->current_ccr = servo->center_ccr;
    
    /* Start PWM and center servo */
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
    Servo_Center(servo);
}

void Servo_SetAngle(Servo_Handle_t *servo, int8_t angle_sw)
{
    /* Clamp SOFTWARE angle to limits (±5°) */
    if (angle_sw > servo->max_angle_sw) {
        angle_sw = servo->max_angle_sw;
    } else if (angle_sw < -servo->max_angle_sw) {
        angle_sw = -servo->max_angle_sw;
    }
    
    servo->current_angle_sw = angle_sw;
    
    /* Calculate actual WHEEL angle */
    servo->current_angle_wheel = (float)angle_sw * servo->linkage_ratio;
    
    /* Calculate CCR value */
    /* Positive angle = right turn = higher CCR */
    /* Negative angle = left turn = lower CCR */
    servo->current_ccr = servo->center_ccr + (angle_sw * servo->ccr_per_degree);
    
    /* Clamp CCR to safe range */
    if (servo->current_ccr < SERVO_MIN_CCR) {
        servo->current_ccr = SERVO_MIN_CCR;
    } else if (servo->current_ccr > SERVO_MAX_CCR) {
        servo->current_ccr = SERVO_MAX_CCR;
    }
    
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->current_ccr);
}

void Servo_SetWheelAngle(Servo_Handle_t *servo, float angle_wheel)
{
    /* Convert WHEEL angle to SOFTWARE angle */
    float angle_sw_f = angle_wheel / servo->linkage_ratio;
    
    /* Round and clamp */
    int8_t angle_sw = (int8_t)angle_sw_f;
    
    Servo_SetAngle(servo, angle_sw);
}

void Servo_Center(Servo_Handle_t *servo)
{
    Servo_SetAngle(servo, 0);
}

void Servo_SetCCR(Servo_Handle_t *servo, uint16_t ccr)
{
    /* For calibration - bypass angle calculation */
    servo->current_ccr = ccr;
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, ccr);
    
    /* Back-calculate angles */
    servo->current_angle_sw = (int8_t)((int16_t)ccr - (int16_t)servo->center_ccr) 
                              / (int16_t)servo->ccr_per_degree;
    servo->current_angle_wheel = (float)servo->current_angle_sw * servo->linkage_ratio;
}

int8_t Servo_GetAngle(Servo_Handle_t *servo)
{
    return servo->current_angle_sw;
}

float Servo_GetWheelAngle(Servo_Handle_t *servo)
{
    return servo->current_angle_wheel;
}

/* ============================================================================
 * ACKERMANN GEOMETRY (uses actual WHEEL angles)
 * ============================================================================ */

float Ackermann_GetTurnRadius(float angle_wheel)
{
    if (fabsf(angle_wheel) < 0.1f) {
        return 100000.0f;  /* Effectively straight - very large radius */
    }
    
    /* Convert to radians */
    float angle_rad = angle_wheel * M_PI / 180.0f;
    
    /* Turn radius from center of rear axle */
    /* R = wheelbase / tan(steering_angle) */
    float radius = CAR_WHEELBASE_MM / tanf(angle_rad);
    
    return radius;  /* Positive = right turn, Negative = left turn */
}

float Ackermann_GetAngleForRadius(float radius)
{
    if (fabsf(radius) < 10.0f || fabsf(radius) > 50000.0f) {
        return 0.0f;  /* Straight or invalid */
    }
    
    /* angle = atan(wheelbase / radius) */
    float angle_rad = atanf(CAR_WHEELBASE_MM / radius);
    float angle_deg = angle_rad * 180.0f / M_PI;
    
    /* Clamp to max wheel angle */
    if (angle_deg > SERVO_MAX_ANGLE_WHEEL) angle_deg = SERVO_MAX_ANGLE_WHEEL;
    if (angle_deg < -SERVO_MAX_ANGLE_WHEEL) angle_deg = -SERVO_MAX_ANGLE_WHEEL;
    
    return angle_deg;
}

int8_t Ackermann_GetSWAngleForRadius(float radius)
{
    float wheel_angle = Ackermann_GetAngleForRadius(radius);
    float sw_angle = wheel_angle / SERVO_LINKAGE_RATIO;
    
    /* Round and clamp */
    int8_t result = (int8_t)sw_angle;
    if (result > SERVO_MAX_ANGLE_SW) result = SERVO_MAX_ANGLE_SW;
    if (result < -SERVO_MAX_ANGLE_SW) result = -SERVO_MAX_ANGLE_SW;
    
    return result;
}

float Ackermann_GetArcLength(float radius, float arc_degrees)
{
    if (fabsf(radius) < 10.0f) {
        return 0.0f;  /* Invalid radius */
    }
    
    /* Arc length = radius * angle_in_radians */
    float arc_rad = arc_degrees * M_PI / 180.0f;
    return fabsf(radius * arc_rad);
}