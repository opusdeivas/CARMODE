/**
 * @file    navigation.h
 * @brief   Navigation logic for obstacle avoidance and wall following
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include "car_config.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "pid.h"
#include "buttons.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * TYPES
 * ============================================================================ */

/* Mode 1 (Obstacle Avoidance) States */
typedef enum {
    NAV1_STATE_IDLE = 0,
    NAV1_STATE_DRIVE_STRAIGHT,
    NAV1_STATE_OBSTACLE_DETECTED,
    NAV1_STATE_ARC_OUT,             /* Turning away from obstacle */
    NAV1_STATE_ARC_BACK,            /* Turning back to centerline */
    NAV1_STATE_APPROACH,            /* Approaching target */
    NAV1_STATE_STOPPING,            /* Decelerating to stop */
    NAV1_STATE_COMPLETE,            /* Reached target */
    NAV1_STATE_ERROR
} Nav1_State_t;

/* Mode 2 (Wall Following) States */
typedef enum {
    NAV2_STATE_IDLE = 0,
    NAV2_STATE_RUNNING,
    NAV2_STATE_OBSTACLE_AHEAD,      /* Car or obstacle detected ahead */
    NAV2_STATE_EMERGENCY_STOP,
    NAV2_STATE_ERROR
} Nav2_State_t;

/* Avoidance direction */
typedef enum {
    AVOID_NONE = 0,
    AVOID_LEFT,
    AVOID_RIGHT
} Avoid_Direction_t;

typedef struct {
    /* Hardware handles */
    Motor_Handle_t *motor;
    Servo_Handle_t *servo;
    US_Handle_t *ultrasonic;
    Encoder_Handle_t *encoder;
    Buttons_Handle_t *buttons;
    
    /* PID controllers */
    PID_Handle_t pid_speed;
    PID_Handle_t pid_steering;
    PID_Handle_t pid_heading;
    
    /* Mode 1 state */
    Nav1_State_t nav1_state;
    Avoid_Direction_t avoid_direction;
    float arc_start_distance;
    float arc_target_distance;
    int8_t arc_steering_angle;      /* SOFTWARE angle (±5°) */
    float obstacle_distance_at_detect;
    
    /* Mode 2 state */
    Nav2_State_t nav2_state;
    
    /* Common state */
    uint16_t target_distance_mm;
    float target_speed_mm_s;
    uint16_t current_pwm;
    
    /* Sensor data (cached) */
    uint16_t dist_front;
    uint16_t dist_left;
    uint16_t dist_right;
    uint16_t dist_rear;
    
    /* Timing */
    uint32_t last_update_time;
    uint32_t state_entry_time;
    
    /* Flags */
    bool is_initialized;
    bool emergency_stop;
} Navigation_Handle_t;

/* ============================================================================
 * FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize navigation system
 * @param nav Pointer to navigation handle
 * @param motor Motor handle
 * @param servo Servo handle
 * @param ultrasonic Ultrasonic handle
 * @param encoder Encoder handle
 * @param buttons Buttons handle
 */
void Navigation_Init(Navigation_Handle_t *nav,
                     Motor_Handle_t *motor,
                     Servo_Handle_t *servo,
                     US_Handle_t *ultrasonic,
                     Encoder_Handle_t *encoder,
                     Buttons_Handle_t *buttons);

/**
 * @brief Start navigation for selected mode
 * @param nav Pointer to navigation handle
 * @param mode Operating mode
 * @param target_distance Target distance for Mode 1 (ignored for Mode 2)
 */
void Navigation_Start(Navigation_Handle_t *nav, Car_Mode_t mode, uint16_t target_distance);

/**
 * @brief Stop navigation immediately
 * @param nav Pointer to navigation handle
 */
void Navigation_Stop(Navigation_Handle_t *nav);

/**
 * @brief Update navigation (call from main loop at control rate)
 * @param nav Pointer to navigation handle
 */
void Navigation_Update(Navigation_Handle_t *nav);

/**
 * @brief Check if navigation task is complete
 * @param nav Pointer to navigation handle
 * @return true if task completed successfully
 */
bool Navigation_IsComplete(Navigation_Handle_t *nav);

/**
 * @brief Emergency stop
 * @param nav Pointer to navigation handle
 */
void Navigation_EmergencyStop(Navigation_Handle_t *nav);

/**
 * @brief Get current Mode 1 state (for debugging)
 * @param nav Pointer to navigation handle
 * @return Current Nav1 state
 */
Nav1_State_t Navigation_GetNav1State(Navigation_Handle_t *nav);

/**
 * @brief Get current Mode 2 state (for debugging)
 * @param nav Pointer to navigation handle
 * @return Current Nav2 state
 */
Nav2_State_t Navigation_GetNav2State(Navigation_Handle_t *nav);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_H */