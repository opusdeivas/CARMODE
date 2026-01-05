/**
 * @file    ultrasonic.h
 * @brief   Ultrasonic distance sensor driver (HC-SR04 / SRF05)
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

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
typedef enum {
    US_SENSOR_FRONT = 0,
    US_SENSOR_RIGHT,
    US_SENSOR_LEFT,
    US_SENSOR_REAR,
    US_SENSOR_COUNT
} US_Sensor_ID_t;

typedef enum {
    US_STATE_IDLE = 0,
    US_STATE_TRIGGER,
    US_STATE_WAIT_ECHO_START,
    US_STATE_WAIT_ECHO_END,
    US_STATE_COMPLETE,
    US_STATE_TIMEOUT
} US_State_t;

typedef struct {
    /* Pin configuration */
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
    
    /* Measurement parameters */
    uint16_t max_range_mm;
    uint16_t timeout_us;
    
    /* State */
    US_State_t state;
    uint32_t echo_start_time;
    uint32_t echo_end_time;
    uint16_t distance_mm;
    bool new_data_available;
} US_Sensor_t;

typedef struct {
    TIM_HandleTypeDef *htim;    /* Timer for echo measurement (TIM2) */
    US_Sensor_t sensors[US_SENSOR_COUNT];
    US_Sensor_ID_t current_sensor;
    uint32_t sequence_start_time;
    bool sequence_running;
    uint8_t current_mode;       /* 1 = Mode1, 2 = Mode2 (affects timeouts) */
} US_Handle_t;

/* ============================================================================
 * FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize ultrasonic sensor system
 * @param us Pointer to ultrasonic handle
 * @param htim Timer handle for echo timing (TIM2)
 */
void US_Init(US_Handle_t *us, TIM_HandleTypeDef *htim);

/**
 * @brief Set operating mode (affects timeout values)
 * @param us Pointer to ultrasonic handle
 * @param mode 1 = obstacle avoidance, 2 = wall following
 */
void US_SetMode(US_Handle_t *us, uint8_t mode);

/**
 * @brief Start measurement sequence (all 4 sensors)
 * @param us Pointer to ultrasonic handle
 */
void US_StartSequence(US_Handle_t *us);

/**
 * @brief Update sensor state machine (call from main loop)
 * @param us Pointer to ultrasonic handle
 */
void US_Update(US_Handle_t *us);

/**
 * @brief Handle EXTI callback for echo pins
 * @param us Pointer to ultrasonic handle
 * @param GPIO_Pin Pin that triggered the interrupt
 */
void US_EXTI_Callback(US_Handle_t *us, uint16_t GPIO_Pin);

/**
 * @brief Check if measurement sequence is complete
 * @param us Pointer to ultrasonic handle
 * @return true if all sensors have been read
 */
bool US_SequenceComplete(US_Handle_t *us);

/**
 * @brief Get distance from specific sensor
 * @param us Pointer to ultrasonic handle
 * @param sensor Sensor ID
 * @return Distance in mm, or 0xFFFF if invalid/timeout
 */
uint16_t US_GetDistance(US_Handle_t *us, US_Sensor_ID_t sensor);

/**
 * @brief Get all distances
 * @param us Pointer to ultrasonic handle
 * @param front Pointer to store front distance
 * @param left Pointer to store left distance
 * @param right Pointer to store right distance
 * @param rear Pointer to store rear distance
 */
void US_GetAllDistances(US_Handle_t *us, uint16_t *front, uint16_t *left, 
                        uint16_t *right, uint16_t *rear);

/**
 * @brief Trigger single sensor (internal use)
 * @param us Pointer to ultrasonic handle
 * @param sensor Sensor to trigger
 */
void US_TriggerSensor(US_Handle_t *us, US_Sensor_ID_t sensor);

void US_EXTI_CallbackWithState(US_Handle_t *us, uint16_t GPIO_Pin, uint8_t pin_state);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */