/**
 * @file    utils.c
 * @brief   Utility functions implementation
 */

#include "utils.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ============================================================================
 * PRIVATE VARIABLES
 * ============================================================================ */
static TIM_HandleTypeDef *htim_delay = NULL;
static UART_HandleTypeDef *huart_debug = NULL;

static LED_Pattern_t current_pattern = LED_PATTERN_OFF;
static uint32_t led_last_toggle = 0;
static uint8_t led_blink_count = 0;
static uint8_t led_blink_phase = 0;

static char debug_buffer[UART_TX_BUFFER_SIZE];

/* ============================================================================
 * DELAY FUNCTIONS
 * ============================================================================ */
void Utils_Init(TIM_HandleTypeDef *htim)
{
    htim_delay = htim;
    HAL_TIM_Base_Start(htim_delay);
}

void Utils_DelayUs(uint32_t us)
{
    if (htim_delay == NULL) return;
    
    /* Handle delays longer than timer period */
    while (us > 65535) {
        __HAL_TIM_SET_COUNTER(htim_delay, 0);
        while (__HAL_TIM_GET_COUNTER(htim_delay) < 65535);
        us -= 65535;
    }
    
    /* Handle remaining delay */
    __HAL_TIM_SET_COUNTER(htim_delay, 0);
    while (__HAL_TIM_GET_COUNTER(htim_delay) < us);
}

uint32_t Utils_GetMicros(void)
{
    if (htim_delay == NULL) return 0;
    /* Note: This is a simple implementation. For longer timing, 
       combine with HAL_GetTick() for milliseconds */
    return __HAL_TIM_GET_COUNTER(htim_delay);
}

/* ============================================================================
 * LED FUNCTIONS
 * ============================================================================ */
void Utils_SetLEDPattern(LED_Pattern_t pattern)
{
    if (current_pattern != pattern) {
        current_pattern = pattern;
        led_blink_count = 0;
        led_blink_phase = 0;
        led_last_toggle = HAL_GetTick();
        
        /* Immediate state for static patterns */
        if (pattern == LED_PATTERN_OFF) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        } else if (pattern == LED_PATTERN_ON || pattern == LED_PATTERN_SOLID) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        }
    }
}

void Utils_UpdateLED(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - led_last_toggle;
    
    switch (current_pattern) {
        case LED_PATTERN_OFF:
        case LED_PATTERN_ON:
        case LED_PATTERN_SOLID:
            /* Static states, nothing to update */
            break;
            
        case LED_PATTERN_BLINK_SLOW: 
            if (elapsed >= LED_BLINK_SLOW_MS) {
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                led_last_toggle = now;
            }
            break;
            
        case LED_PATTERN_BLINK_FAST:
            if (elapsed >= LED_BLINK_FAST_MS) {
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                led_last_toggle = now;
            }
            break;
            
        case LED_PATTERN_BLINK_1:
            /* Pattern:  ON-OFF-pause (1 blink) */
            if (led_blink_phase == 0 && elapsed >= LED_BLINK_FAST_MS) {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                led_blink_phase = 1;
                led_last_toggle = now;
            } else if (led_blink_phase == 1 && elapsed >= LED_BLINK_FAST_MS) {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                led_blink_phase = 2;
                led_last_toggle = now;
            } else if (led_blink_phase == 2 && elapsed >= LED_BLINK_SLOW_MS) {
                led_blink_phase = 0;
                led_last_toggle = now;
            }
            break;
            
        case LED_PATTERN_BLINK_2:
            /* Pattern: ON-OFF-ON-OFF-pause (2 blinks) */
            if (led_blink_phase < 4) {
                if (elapsed >= LED_BLINK_FAST_MS) {
                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                    led_blink_phase++;
                    led_last_toggle = now;
                }
            } else if (elapsed >= LED_BLINK_SLOW_MS) {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                led_blink_phase = 0;
                led_last_toggle = now;
            }
            break;
    }
}

/* ============================================================================
 * DEBUG OUTPUT
 * ============================================================================ */
void Debug_Init(UART_HandleTypeDef *huart)
{
    huart_debug = huart;
}

void Debug_Print(const char *format, ...)
{
    if (huart_debug == NULL) return;
    
    va_list args;
    va_start(args, format);
    int len = vsnprintf(debug_buffer, UART_TX_BUFFER_SIZE, format, args);
    va_end(args);
    
    if (len > 0) {
        HAL_UART_Transmit(huart_debug, (uint8_t*)debug_buffer, len, 10);
    }
}

void Debug_PrintDistances(uint16_t front, uint16_t left, uint16_t right, uint16_t rear)
{
    Debug_Print("DIST F:%4d L:%4d R:%4d B:%4d\r\n", front, left, right, rear);
}

void Debug_PrintOdometry(float distance, float speed, float offset)
{
    Debug_Print("ODO D: %.1f S:%.1f O: %.1f\r\n", distance, speed, offset);
}