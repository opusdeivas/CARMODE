/**
 * @file    buttons.h
 * @brief   Button handling with debounce and state machine
 */

#ifndef BUTTONS_H
#define BUTTONS_H

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
    BTN_GREEN = 0,      /* Start */
    BTN_RED,            /* Stop */
    BTN_BLUE,           /* Mode 1 (obstacle avoidance) */
    BTN_WHITE,          /* Mode 2 (wall following) */
    BTN_COUNT
} Button_ID_t;

typedef enum {
    CAR_STATE_IDLE = 0,     /* Waiting for mode selection and start */
    CAR_STATE_RUNNING,      /* Actively running selected mode */
    CAR_STATE_STOPPED,      /* Stopped (can restart or change mode) */
    CAR_STATE_ERROR         /* Error state */
} Car_State_t;

typedef enum {
    CAR_MODE_NONE = 0,
    CAR_MODE_1_OBSTACLE,    /* Task 1: Obstacle avoidance */
    CAR_MODE_2_WALLFOLLOW   /* Task 2: Wall following race */
} Car_Mode_t;

typedef enum {
    TASK1_DIST_SHORT = 0,   /* 3005mm */
    TASK1_DIST_LONG         /* 7515mm */
} Task1_Distance_t;

typedef struct {
    /* Button state */
    uint32_t last_press_time[BTN_COUNT];
    bool pending_action[BTN_COUNT];
    
    /* Car state */
    Car_State_t state;
    Car_Mode_t mode;
    Task1_Distance_t task1_distance;
    
    /* Target distance for Mode 1 */
    uint16_t target_distance_mm;
    
    /* Callbacks */
    void (*on_start)(void);
    void (*on_stop)(void);
    void (*on_mode_change)(Car_Mode_t mode);
} Buttons_Handle_t;

/* ============================================================================
 * FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize button handler
 * @param btn Pointer to button handle
 */
void Buttons_Init(Buttons_Handle_t *btn);

/**
 * @brief Set callback functions
 * @param btn Pointer to button handle
 * @param on_start Called when start button pressed
 * @param on_stop Called when stop button pressed
 * @param on_mode_change Called when mode changes
 */
void Buttons_SetCallbacks(Buttons_Handle_t *btn,
                          void (*on_start)(void),
                          void (*on_stop)(void),
                          void (*on_mode_change)(Car_Mode_t mode));

/**
 * @brief Handle EXTI callback (call from HAL_GPIO_EXTI_Callback)
 * @param btn Pointer to button handle
 * @param GPIO_Pin Pin that triggered interrupt
 */
void Buttons_EXTI_Callback(Buttons_Handle_t *btn, uint16_t GPIO_Pin);

/**
 * @brief Process pending button actions (call from main loop)
 * @param btn Pointer to button handle
 */
void Buttons_Update(Buttons_Handle_t *btn);

/**
 * @brief Get current car state
 * @param btn Pointer to button handle
 * @return Current state
 */
Car_State_t Buttons_GetState(Buttons_Handle_t *btn);

/**
 * @brief Get current mode
 * @param btn Pointer to button handle
 * @return Current mode
 */
Car_Mode_t Buttons_GetMode(Buttons_Handle_t *btn);

/**
 * @brief Get target distance for Mode 1
 * @param btn Pointer to button handle
 * @return Target distance in mm
 */
uint16_t Buttons_GetTargetDistance(Buttons_Handle_t *btn);

/**
 * @brief Set car state (for external control)
 * @param btn Pointer to button handle
 * @param state New state
 */
void Buttons_SetState(Buttons_Handle_t *btn, Car_State_t state);

/**
 * @brief Check if car is running
 * @param btn Pointer to button handle
 * @return true if in RUNNING state
 */
bool Buttons_IsRunning(Buttons_Handle_t *btn);

#ifdef __cplusplus
}
#endif

#endif /* BUTTONS_H */