/**
 * @file    encoder.c
 * @brief   Encoder and odometry implementation
 * 
 * Receives data from ESP32 in format:  "D:dL,dR,vL,vR\n"
 *   - dL,dR = encoder count deltas
 *   - vL,vR = wheel speeds in mm/s (EMA filtered by ESP32)
 * 
 * STM32 calculates:
 *   - Total distance from deltas
 *   - Heading from wheel differential + Ackermann geometry
 *   - Lateral offset estimation
 */

#include "encoder.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================================
 * PRIVATE FUNCTIONS
 * ============================================================================ */

/**
 * @brief Parse packet from ESP32
 * @param enc Pointer to encoder handle
 * @return true if parsing successful
 * 
 * Expected format: "D:dL,dR,vL,vR" e.g., "D:5,6,150,148" or "D:-3,4,-50,60"
 */
static bool Encoder_ParsePacket(Encoder_Handle_t *enc)
{
    char *buffer = (char*)enc->rx_buffer;
    
    /* Check header */
    if (buffer[0] != 'D' || buffer[1] != ':') {
        return false;
    }
    
    /* Parse using strtok to handle the 4 values */
    char *ptr = &buffer[2];
    char *token;
    int values[4];
    int count = 0;
    
    /* Make a copy since strtok modifies the string */
    char temp[UART_RX_BUFFER_SIZE];
    strncpy(temp, ptr, UART_RX_BUFFER_SIZE - 1);
    temp[UART_RX_BUFFER_SIZE - 1] = '\0';
    
    token = strtok(temp, ",");
    while (token != NULL && count < 4) {
        values[count++] = atoi(token);
        token = strtok(NULL, ",");
    }
    
    /* Need at least 2 values (deltas), 4 is full packet with speeds */
    if (count < 2) {
        return false;
    }
    
    /* Store parsed values */
    enc->delta_left = (int16_t)values[0];
    enc->delta_right = (int16_t)values[1];
    
    if (count >= 4) {
        /* Full packet with speeds */
        enc->speed_left_mm_s = (int16_t)values[2];
        enc->speed_right_mm_s = (int16_t)values[3];
        enc->speed_mm_s = (float)(enc->speed_left_mm_s + enc->speed_right_mm_s) / 2.0f;
    }
    /* If only 2 values, keep previous speed (ESP sends 0,0 speeds when stopped) */
    
    return true;
}

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

void Encoder_Init(Encoder_Handle_t *enc, UART_HandleTypeDef *huart)
{
    enc->huart = huart;
    enc->rx_index = 0;
    enc->packet_ready = false;
    
    enc->delta_left = 0;
    enc->delta_right = 0;
    
    enc->speed_left_mm_s = 0;
    enc->speed_right_mm_s = 0;
    
    enc->last_packet_time = 0;
    enc->packet_interval_ms = 0;
    
    enc->total_distance_mm = 0.0f;
    enc->left_distance_mm = 0.0f;
    enc->right_distance_mm = 0.0f;
    enc->lateral_offset_mm = 0.0f;
    enc->heading_rad = 0.0f;
    
    enc->speed_mm_s = 0.0f;
    
    enc->packet_count = 0;
    enc->error_count = 0;
    
    memset(enc->rx_buffer, 0, UART_RX_BUFFER_SIZE);
}

void Encoder_StartReceive(Encoder_Handle_t *enc)
{
    /* Enable UART receive interrupt */
    __HAL_UART_ENABLE_IT(enc->huart, UART_IT_RXNE);
}

void Encoder_ProcessByte(Encoder_Handle_t *enc, uint8_t data)
{
    /* Check for end of packet */
    if (data == '\n' || data == '\r') {
        if (enc->rx_index > 0) {
            enc->rx_buffer[enc->rx_index] = '\0';
            enc->packet_ready = true;
        }
        enc->rx_index = 0;
        return;
    }
    
    /* Add byte to buffer */
    if (enc->rx_index < UART_RX_BUFFER_SIZE - 1) {
        enc->rx_buffer[enc->rx_index++] = data;
    } else {
        /* Buffer overflow - reset */
        enc->rx_index = 0;
        enc->error_count++;
    }
}

void Encoder_Update(Encoder_Handle_t *enc, int8_t steering_angle_sw)
{
    if (!enc->packet_ready) return;
    
    enc->packet_ready = false;
    
    /* Calculate time since last packet */
    uint32_t now = HAL_GetTick();
    if (enc->last_packet_time > 0) {
        enc->packet_interval_ms = now - enc->last_packet_time;
    }
    enc->last_packet_time = now;
    
    /* Parse the packet */
    if (! Encoder_ParsePacket(enc)) {
        enc->error_count++;
        return;
    }
    
    enc->packet_count++;
    
    /* Convert encoder counts to mm */
    float left_delta_mm = (float)enc->delta_left * ENCODER_MM_PER_COUNT;
    float right_delta_mm = (float)enc->delta_right * ENCODER_MM_PER_COUNT;
    
    /* Accumulate wheel distances */
    enc->left_distance_mm += left_delta_mm;
    enc->right_distance_mm += right_delta_mm;
    
    /* Calculate center distance (average of both wheels) */
    float center_delta_mm = (left_delta_mm + right_delta_mm) / 2.0f;
    enc->total_distance_mm += fabsf(center_delta_mm);
    
    /* Differential odometry for heading and lateral offset */
    float wheel_diff_mm = right_delta_mm - left_delta_mm;
    
    /* Calculate actual WHEEL angle from SOFTWARE angle */
    float wheel_angle_deg = (float)steering_angle_sw * SERVO_LINKAGE_RATIO;
    
    if (fabsf(wheel_angle_deg) > 0.5f) {
        /* During turns, use Ackermann geometry */
        float angle_rad = wheel_angle_deg * M_PI / 180.0f;
        float turn_radius = CAR_WHEELBASE_MM / tanf(angle_rad);
        
        /* Angular change based on arc traveled */
        if (fabsf(turn_radius) > 10.0f) {
            float delta_heading = center_delta_mm / turn_radius;
            enc->heading_rad += delta_heading;
        }
    } else {
        /* Straight line - use differential for small corrections */
        float delta_heading = wheel_diff_mm / CAR_REAR_TRACK_MM;
        enc->heading_rad += delta_heading;
    }
    
    /* Update lateral offset based on heading */
    enc->lateral_offset_mm += center_delta_mm * sinf(enc->heading_rad);
}

void Encoder_ResetOdometry(Encoder_Handle_t *enc)
{
    enc->total_distance_mm = 0.0f;
    enc->left_distance_mm = 0.0f;
    enc->right_distance_mm = 0.0f;
    enc->lateral_offset_mm = 0.0f;
    enc->heading_rad = 0.0f;
    
    /* Don't reset speed - keep current reading */
}

float Encoder_GetSpeed(Encoder_Handle_t *enc)
{
    return enc->speed_mm_s;
}

int16_t Encoder_GetLeftSpeed(Encoder_Handle_t *enc)
{
    return enc->speed_left_mm_s;
}

int16_t Encoder_GetRightSpeed(Encoder_Handle_t *enc)
{
    return enc->speed_right_mm_s;
}

float Encoder_GetDistance(Encoder_Handle_t *enc)
{
    return enc->total_distance_mm;
}

float Encoder_GetLateralOffset(Encoder_Handle_t *enc)
{
    return enc->lateral_offset_mm;
}

float Encoder_GetHeading(Encoder_Handle_t *enc)
{
    return enc->heading_rad * 180.0f / M_PI;
}

bool Encoder_HasNewData(Encoder_Handle_t *enc)
{
    return enc->packet_ready;
}

void Encoder_SendCommand(Encoder_Handle_t *enc, const char *cmd)
{
    if (enc->huart == NULL) return;
    
    /* Send command with newline */
    HAL_UART_Transmit(enc->huart, (uint8_t*)cmd, strlen(cmd), 10);
    HAL_UART_Transmit(enc->huart, (uint8_t*)"\n", 1, 10);
}