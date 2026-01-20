/**
 * @file    buttons.c
 * @brief   Button handling and car state machine implementation
 */

#include "buttons.h"
#include "utils.h"
#include "main.h"

/* ============================================================================
 * PRIVATE FUNCTIONS
 * ============================================================================ */

static Button_ID_t Buttons_GetIDFromPin(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GREEN_BUTTON_Pin:   return BTN_GREEN;
        case RED_BUTTON_Pin:    return BTN_RED;
        case BLUE_BUTTON_Pin:   return BTN_BLUE;
        case WHITE_BUTTON_Pin:  return BTN_WHITE;
        default:                return BTN_COUNT;
    }
}

static void Buttons_ProcessGreen(Buttons_Handle_t *btn)
{
    /* GREEN = Start (only if mode selected) */
    if (btn->state == CAR_STATE_IDLE || btn->state == CAR_STATE_STOPPED) {
        if (btn->mode != CAR_MODE_NONE) {
            btn->state = CAR_STATE_RUNNING;
            Utils_SetLEDPattern(LED_PATTERN_BLINK_FAST);
            
            if (btn->on_start) {
                btn->on_start();
            }
        }
    }
}

static void Buttons_ProcessRed(Buttons_Handle_t *btn)
{
    /* RED = Stop / Return to Idle */
    if (btn->state == CAR_STATE_RUNNING) {
        /* Stop the car */
        btn->state = CAR_STATE_STOPPED;
        Utils_SetLEDPattern(LED_PATTERN_OFF);
        
        if (btn->on_stop) {
            btn->on_stop();
        }
    } else if (btn->state == CAR_STATE_STOPPED) {
        /* Return to idle, clear mode */
        btn->state = CAR_STATE_IDLE;
        btn->mode = CAR_MODE_NONE;
        btn->target_distance_mm = 0;
        Utils_SetLEDPattern(LED_PATTERN_BLINK_SLOW);
        
        if (btn->on_mode_change) {
            btn->on_mode_change(btn->mode);
        }
    }
}

static void Buttons_ProcessBlue(Buttons_Handle_t *btn)
{
    /* BLUE = Mode 1 (Obstacle Avoidance) / Toggle Distance */
    if (btn->state == CAR_STATE_IDLE || btn->state == CAR_STATE_STOPPED) {
        if (btn->mode == CAR_MODE_1_OBSTACLE) {
            /* Already in Mode 1 - toggle distance */
            if (btn->task1_distance == TASK1_DIST_SHORT) {
                btn->task1_distance = TASK1_DIST_LONG;
                btn->target_distance_mm = TASK1_DISTANCE_LONG;
                Utils_SetLEDPattern(LED_PATTERN_BLINK_2);
            } else {
                btn->task1_distance = TASK1_DIST_SHORT;
                btn->target_distance_mm = TASK1_DISTANCE_SHORT;
                Utils_SetLEDPattern(LED_PATTERN_BLINK_1);
            }
        } else {
            /* Switch to Mode 1 with short distance */
            btn->mode = CAR_MODE_1_OBSTACLE;
            btn->task1_distance = TASK1_DIST_SHORT;
            btn->target_distance_mm = TASK1_DISTANCE_SHORT;
            Utils_SetLEDPattern(LED_PATTERN_BLINK_1);
        }
        
        if (btn->on_mode_change) {
            btn->on_mode_change(btn->mode);
        }
    }
}

static void Buttons_ProcessWhite(Buttons_Handle_t *btn)
{
    /* WHITE = Mode 2 (Wall Following) */
    if (btn->state == CAR_STATE_IDLE || btn->state == CAR_STATE_STOPPED) {
        btn->mode = CAR_MODE_2_WALLFOLLOW;
        btn->target_distance_mm = 0;
        Utils_SetLEDPattern(LED_PATTERN_SOLID);
        
        if (btn->on_mode_change) {
            btn->on_mode_change(btn->mode);
        }
    }
}

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

void Buttons_Init(Buttons_Handle_t *btn)
{
    btn->state = CAR_STATE_IDLE;
    btn->mode = CAR_MODE_NONE;
    btn->task1_distance = TASK1_DIST_SHORT;
    btn->target_distance_mm = 0;
    
    for (int i = 0; i < BTN_COUNT; i++) {
        btn->last_press_time[i] = 0;
        btn->pending_action[i] = false;
    }
    
    btn->on_start = NULL;
    btn->on_stop = NULL;
    btn->on_mode_change = NULL;
    
    /* Initial LED pattern - slow blink indicates idle/ready */
    Utils_SetLEDPattern(LED_PATTERN_BLINK_SLOW);
}

void Buttons_SetCallbacks(Buttons_Handle_t *btn,
                          void (*on_start)(void),
                          void (*on_stop)(void),
                          void (*on_mode_change)(Car_Mode_t mode))
{
    btn->on_start = on_start;
    btn->on_stop = on_stop;
    btn->on_mode_change = on_mode_change;
}

void Buttons_EXTI_Callback(Buttons_Handle_t *btn, uint16_t GPIO_Pin)
{
    Button_ID_t id = Buttons_GetIDFromPin(GPIO_Pin);
    if (id >= BTN_COUNT) return;
    
    uint32_t now = HAL_GetTick();
    
    /* Debounce check */
    if ((now - btn->last_press_time[id]) >= BUTTON_DEBOUNCE_MS) {
        btn->last_press_time[id] = now;
        btn->pending_action[id] = true;
    }
}

void Buttons_Update(Buttons_Handle_t *btn)
{
    /* Process pending button actions */
    if (btn->pending_action[BTN_GREEN]) {
        btn->pending_action[BTN_GREEN] = false;
        Buttons_ProcessGreen(btn);
    }
    
    if (btn->pending_action[BTN_RED]) {
        btn->pending_action[BTN_RED] = false;
        Buttons_ProcessRed(btn);
    }
    
    if (btn->pending_action[BTN_BLUE]) {
        btn->pending_action[BTN_BLUE] = false;
        Buttons_ProcessBlue(btn);
    }
    
    if (btn->pending_action[BTN_WHITE]) {
        btn->pending_action[BTN_WHITE] = false;
        Buttons_ProcessWhite(btn);
    }
}

Car_State_t Buttons_GetState(Buttons_Handle_t *btn)
{
    return btn->state;
}

Car_Mode_t Buttons_GetMode(Buttons_Handle_t *btn)
{
    return btn->mode;
}

uint16_t Buttons_GetTargetDistance(Buttons_Handle_t *btn)
{
    return btn->target_distance_mm;
}

bool Buttons_IsRunning(Buttons_Handle_t *btn)
{
    return (btn->state == CAR_STATE_RUNNING);
}

void Buttons_SetState(Buttons_Handle_t *btn, Car_State_t state)
{
    btn->state = state;
    
    /* Update LED pattern based on new state */
    switch (state) {
        case CAR_STATE_IDLE: 
            Utils_SetLEDPattern(LED_PATTERN_BLINK_SLOW);
            break;
        case CAR_STATE_RUNNING:
            Utils_SetLEDPattern(LED_PATTERN_BLINK_FAST);
            break;
        case CAR_STATE_STOPPED:
            Utils_SetLEDPattern(LED_PATTERN_OFF);
            break;
        case CAR_STATE_ERROR:
            Utils_SetLEDPattern(LED_PATTERN_BLINK_FAST);
            break;
        default:
            break;
    }
}

void Buttons_SetComplete(Buttons_Handle_t *btn)
{
    /* Called when task completes successfully */
    btn->state = CAR_STATE_STOPPED;
    Utils_SetLEDPattern(LED_PATTERN_SOLID);
}

void Buttons_SetError(Buttons_Handle_t *btn)
{
    /* Called on error */
    btn->state = CAR_STATE_ERROR;
    Utils_SetLEDPattern(LED_PATTERN_BLINK_FAST);  /* Fast blink = error */
}