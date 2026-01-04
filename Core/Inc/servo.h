/**
 * @file    servo.h
 * @brief   Servo motor control with Ackermann geometry calculations
 * 
 * NOTE: This car has a 5:1 mechanical linkage ratio
 *   - Software angle input: ±5° max
 *   - Actual wheel angle:  ±25° max
 *   - All Servo_SetAngle() calls use SOFTWARE angle
 *   - Ackermann calculations use actual WHEEL angle
 */

#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include "car_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * TYPES
 * ============================================================================ */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    
    /* Calibration */
    uint16_t center_ccr;
    uint16_t ccr_per_degree;
    int8_t max_angle_sw;          // Max SOFTWARE angle (±5°)
    float linkage_ratio;          // Mechanical ratio (5.0)
    
    /* State */
    int8_t current_angle_sw;      // Current SOFTWARE angle
    float current_angle_wheel;    // Current WHEEL angle (for Ackermann)
    uint16_t current_ccr;
} Servo_Handle_t;

/* ============================================================================
 * FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize servo
 * @param servo Pointer to servo handle
 * @param htim Timer handle (TIM21)
 * @param channel PWM channel (TIM_CHANNEL_1)
 */
void Servo_Init(Servo_Handle_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief Set servo angle (SOFTWARE angle, will be amplified by linkage)
 * @param servo Pointer to servo handle
 * @param angle_sw Software angle in degrees (-5 to +5)
 *                 Negative = left turn, Positive = right turn
 */
void Servo_SetAngle(Servo_Handle_t *servo, int8_t angle_sw);

/**
 * @brief Set servo angle using target WHEEL angle (auto-converts to SW angle)
 * @param servo Pointer to servo handle
 * @param angle_wheel Desired wheel angle in degrees (-25 to +25)
 */
void Servo_SetWheelAngle(Servo_Handle_t *servo, float angle_wheel);

/**
 * @brief Set servo to center position
 * @param servo Pointer to servo handle
 */
void Servo_Center(Servo_Handle_t *servo);

/**
 * @brief Set servo by raw CCR value (for calibration)
 * @param servo Pointer to servo handle
 * @param ccr CCR value to set
 */
void Servo_SetCCR(Servo_Handle_t *servo, uint16_t ccr);

/**
 * @brief Get current SOFTWARE steering angle
 * @param servo Pointer to servo handle
 * @return Current SW angle in degrees (-5 to +5)
 */
int8_t Servo_GetAngle(Servo_Handle_t *servo);

/**
 * @brief Get current WHEEL steering angle (actual physical angle)
 * @param servo Pointer to servo handle
 * @return Current wheel angle in degrees (-25 to +25)
 */
float Servo_GetWheelAngle(Servo_Handle_t *servo);

/* ============================================================================
 * ACKERMANN GEOMETRY FUNCTIONS (all use WHEEL angles)
 * ============================================================================ */

/**
 * @brief Calculate turn radius from WHEEL steering angle
 * @param angle_wheel Wheel steering angle in degrees
 * @return Turn radius in mm (positive = right turn, negative = left turn)
 *         Returns very large number for straight (angle ≈ 0)
 */
float Ackermann_GetTurnRadius(float angle_wheel);

/**
 * @brief Calculate WHEEL steering angle for desired turn radius
 * @param radius Desired turn radius in mm (positive = right, negative = left)
 * @return Required wheel angle in degrees
 */
float Ackermann_GetAngleForRadius(float radius);

/**
 * @brief Calculate SOFTWARE angle for desired turn radius
 * @param radius Desired turn radius in mm
 * @return Required software angle in degrees (-5 to +5)
 */
int8_t Ackermann_GetSWAngleForRadius(float radius);

/**
 * @brief Calculate arc length for given radius and arc angle
 * @param radius Turn radius in mm
 * @param arc_degrees Arc angle in degrees
 * @return Arc length in mm
 */
float Ackermann_GetArcLength(float radius, float arc_degrees);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */