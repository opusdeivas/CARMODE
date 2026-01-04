/**
 * @file    utils.h
 * @brief   Utility functions - delays, LED patterns, debug output
 */

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include "car_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * DELAY FUNCTIONS
 * ============================================================================ */
/**
 * @brief Initialize the microsecond delay timer (TIM6)
 * @param htim Pointer to TIM6 handle
 */
void Utils_Init(TIM_HandleTypeDef *htim);

/**
 * @brief Microsecond delay (non-blocking friendly for short delays)
 * @param us Microseconds to delay
 */
void Utils_DelayUs(uint32_t us);

/**
 * @brief Get current microsecond timestamp
 * @return Microsecond counter value
 */
uint32_t Utils_GetMicros(void);

/* ============================================================================
 * LED FUNCTIONS
 * ============================================================================ */
typedef enum {
    LED_PATTERN_OFF,          // LED off
    LED_PATTERN_ON,           // LED on solid
    LED_PATTERN_BLINK_SLOW,   // Slow blink (IDLE)
    LED_PATTERN_BLINK_FAST,   // Fast blink (RUNNING)
    LED_PATTERN_BLINK_1,      // 1 blink pattern (Mode1 short)
    LED_PATTERN_BLINK_2,      // 2 blink pattern (Mode1 long)
    LED_PATTERN_SOLID         // Solid on (Mode2)
} LED_Pattern_t;

/**
 * @brief Set LED blink pattern
 * @param pattern Pattern to display
 */
void Utils_SetLEDPattern(LED_Pattern_t pattern);

/**
 * @brief Update LED state (call from main loop)
 */
void Utils_UpdateLED(void);

/* ============================================================================
 * DEBUG OUTPUT
 * ============================================================================ */
/**
 * @brief Initialize debug UART
 * @param huart Pointer to USART2 handle
 */
void Debug_Init(UART_HandleTypeDef *huart);

/**
 * @brief Print debug message (printf-style)
 * @param format Format string
 */
void Debug_Print(const char *format, ...);

/**
 * @brief Print sensor distances
 * @param front Front distance in mm
 * @param left Left distance in mm
 * @param right Right distance in mm
 * @param rear Rear distance in mm
 */
void Debug_PrintDistances(uint16_t front, uint16_t left, uint16_t right, uint16_t rear);

/**
 * @brief Print odometry data
 * @param distance Total distance in mm
 * @param speed Current speed in mm/s
 * @param offset Lateral offset in mm
 */
void Debug_PrintOdometry(float distance, float speed, float offset);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */