/**
 * @file    encoder.h
 * @brief   Encoder data reception from ESP32 and odometry calculations
 * 
 * ESP32 Protocol:
 *   - Format: "D:dL,dR,vL,vR\n"
 *   - dL,dR = encoder deltas (counts) since last send
 *   - vL,vR = filtered speed in mm/s (calculated by ESP32)
 *   - Distance calculated by STM32 from deltas
 *   - Speed received directly from ESP32 (EMA filtered)
 */

#ifndef ENCODER_H
#define ENCODER_H

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
    UART_HandleTypeDef *huart;
    
    /* Receive buffer */
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t rx_index;
    volatile bool packet_ready;
    
    /* Latest encoder deltas (from ESP32) */
    int16_t delta_left;
    int16_t delta_right;
    
    /* Speed from ESP32 (pre-filtered with EMA) */
    int16_t speed_left_mm_s;      /* Left wheel speed from ESP32 */
    int16_t speed_right_mm_s;     /* Right wheel speed from ESP32 */
    
    /* Accumulated odometry (calculated by STM32) */
    float total_distance_mm;        /* Total distance traveled */
    float left_distance_mm;         /* Left wheel total distance */
    float right_distance_mm;        /* Right wheel total distance */
    float lateral_offset_mm;        /* Estimated lateral offset from start */
    float heading_rad;              /* Estimated heading in radians */
    
    /* Combined speed (average of both wheels) */
    float speed_mm_s;               /* Current speed in mm/s */
    
    /* Timing */
    uint32_t last_packet_time;
    uint32_t packet_interval_ms;
    
    /* Statistics */
    uint32_t packet_count;
    uint32_t error_count;
} Encoder_Handle_t;

/* ============================================================================
 * FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize encoder interface
 * @param enc Pointer to encoder handle
 * @param huart UART handle (USART1)
 */
void Encoder_Init(Encoder_Handle_t *enc, UART_HandleTypeDef *huart);

/**
 * @brief Start receiving encoder data
 * @param enc Pointer to encoder handle
 */
void Encoder_StartReceive(Encoder_Handle_t *enc);

/**
 * @brief Process received UART data (call from UART RX interrupt)
 * @param enc Pointer to encoder handle
 * @param data Received byte
 */
void Encoder_ProcessByte(Encoder_Handle_t *enc, uint8_t data);

/**
 * @brief Update odometry calculations (call from main loop)
 * @param enc Pointer to encoder handle
 * @param steering_angle_sw Current SOFTWARE steering angle for Ackermann compensation
 */
void Encoder_Update(Encoder_Handle_t *enc, int8_t steering_angle_sw);

/**
 * @brief Reset odometry to zero
 * @param enc Pointer to encoder handle
 */
void Encoder_ResetOdometry(Encoder_Handle_t *enc);

/**
 * @brief Get current speed in mm/s (from ESP32)
 * @param enc Pointer to encoder handle
 * @return Speed in mm/s (average of both wheels)
 */
float Encoder_GetSpeed(Encoder_Handle_t *enc);

/**
 * @brief Get left wheel speed in mm/s
 * @param enc Pointer to encoder handle
 * @return Left wheel speed in mm/s
 */
int16_t Encoder_GetLeftSpeed(Encoder_Handle_t *enc);

/**
 * @brief Get right wheel speed in mm/s
 * @param enc Pointer to encoder handle
 * @return Right wheel speed in mm/s
 */
int16_t Encoder_GetRightSpeed(Encoder_Handle_t *enc);

/**
 * @brief Get total distance traveled in mm
 * @param enc Pointer to encoder handle
 * @return Total distance in mm
 */
float Encoder_GetDistance(Encoder_Handle_t *enc);

/**
 * @brief Get estimated lateral offset in mm
 * @param enc Pointer to encoder handle
 * @return Lateral offset in mm (positive = right of start)
 */
float Encoder_GetLateralOffset(Encoder_Handle_t *enc);

/**
 * @brief Get estimated heading in degrees
 * @param enc Pointer to encoder handle
 * @return Heading in degrees (positive = turned left)
 */
float Encoder_GetHeading(Encoder_Handle_t *enc);

/**
 * @brief Check if new encoder data is available
 * @param enc Pointer to encoder handle
 * @return true if new data since last update
 */
bool Encoder_HasNewData(Encoder_Handle_t *enc);

/**
 * @brief Send command to ESP32 (for bidirectional comms)
 * @param enc Pointer to encoder handle
 * @param cmd Command string (without newline)
 */
void Encoder_SendCommand(Encoder_Handle_t *enc, const char *cmd);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */