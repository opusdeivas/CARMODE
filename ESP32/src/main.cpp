/**
 * @file    main.cpp
 * @brief   ESP32 Encoder Counter for CARMODE project
 * 
 * Hardware:
 *   - ESP32 WROOM DevKit1
 *   - 2x DIY magnetic quadrature encoders (left and right rear wheels)
 *   - Each encoder:  2x SS41 Hall effect sensors (open-drain with external pullups)
 *   - 6 magnets per wheel (3 pole pairs)
 *   - Resolution: 12 counts per revolution
 * 
 * Communication:
 *   - UART2 to STM32 (bidirectional)
 *   - TX Format: "D:dL,dR,vL,vR\n" 
 *       dL,dR = encoder deltas since last send
 *       vL,vR = filtered speed in mm/s
 *   - RX:  Commands from STM32
 * 
 * Pin assignments:
 *   - Left encoder A:   GPIO 25
 *   - Left encoder B:  GPIO 26
 *   - Right encoder A: GPIO 34 (input-only, external pullup required)
 *   - Right encoder B: GPIO 35 (input-only, external pullup required)
 *   - UART TX to STM32: GPIO 17 (UART2 TX) -> STM32 PA10 (USART1 RX)
 *   - UART RX from STM32: GPIO 16 (UART2 RX) <- STM32 PA9 (USART1 TX)
 */

#include <Arduino.h>

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */

/* Encoder pins */
#define LEFT_ENC_A_PIN      25
#define LEFT_ENC_B_PIN      26
#define RIGHT_ENC_A_PIN     34
#define RIGHT_ENC_B_PIN     35

/* UART to STM32 (bidirectional) */
#define STM32_SERIAL        Serial2
#define STM32_BAUD          115200
#define STM32_TX_PIN        17
#define STM32_RX_PIN        16

/* UART receive buffer */
#define RX_BUFFER_SIZE      64

/* Timing - STM32 communication */
#define STM32_SEND_INTERVAL_MS      40      /* 25Hz when moving */
#define STM32_HEARTBEAT_INTERVAL_MS 200     /* 5Hz when stopped */

/* Speed calculation */
#define DEBOUNCE_US             50          /* Debounce time for hall sensors */
#define SPEED_TIMEOUT_US        500000      /* 500ms - if no tick, speed = 0 */
#define MM_PER_COUNT            17.08f      /* 205mm circumference / 12 counts */
#define EMA_ALPHA               0.2f        /* EMA filter:  0.2 = moderate smoothing */

/* Debug output on USB Serial */
#define DEBUG_ENABLED               1
#define DEBUG_SERIAL                Serial
#define DEBUG_BAUD                  115200
#define DEBUG_HEARTBEAT_INTERVAL_MS 5000    /* 5s heartbeat when stopped */

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

/* Encoder counts - volatile because modified in ISR */
volatile int32_t left_count = 0;
volatile int32_t right_count = 0;

/* Timing for speed calculation - volatile because modified in ISR */
volatile uint32_t left_last_tick_us = 0;
volatile uint32_t right_last_tick_us = 0;
volatile uint32_t left_interval_us = 0;
volatile uint32_t right_interval_us = 0;
volatile int8_t left_direction = 1;
volatile int8_t right_direction = 1;

/* Debounce timestamps */
volatile uint32_t left_a_last_isr = 0;
volatile uint32_t left_b_last_isr = 0;
volatile uint32_t right_a_last_isr = 0;
volatile uint32_t right_b_last_isr = 0;

/* Previous counts for delta calculation */
int32_t left_count_prev = 0;
int32_t right_count_prev = 0;

/* Filtered speed (EMA) */
float left_speed_filtered = 0.0f;
float right_speed_filtered = 0.0f;

/* Timing */
uint32_t last_stm32_send_time = 0;
uint32_t last_debug_event_time = 0;

/* Debug - track last values for event detection */
#if DEBUG_ENABLED
int32_t debug_last_left = 0;
int32_t debug_last_right = 0;
#endif

/* UART receive buffer */
char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

/* ============================================================================
 * INTERRUPT SERVICE ROUTINES
 * 
 * Quadrature decoding:
 *   - On A rising:  if B is LOW, forward; if B is HIGH, backward
 *   - On A falling: if B is HIGH, forward; if B is LOW, backward
 *   - On B rising:  if A is HIGH, forward; if A is LOW, backward
 *   - On B falling: if A is LOW, forward; if A is HIGH, backward
 * ============================================================================ */

void IRAM_ATTR leftEncoderA_ISR() 
{
    uint32_t now = micros();
    if ((now - left_a_last_isr) < DEBOUNCE_US) return;
    left_a_last_isr = now;
    
    /* Speed calculation - interval between ticks */
    if (left_last_tick_us != 0) {
        left_interval_us = now - left_last_tick_us;
    }
    left_last_tick_us = now;
    
    int a_state = digitalRead(LEFT_ENC_A_PIN);
    int b_state = digitalRead(LEFT_ENC_B_PIN);
    
    if (a_state == HIGH) {
        if (b_state == LOW) {
            left_count++;
            left_direction = 1;
        } else {
            left_count--;
            left_direction = -1;
        }
    } else {
        if (b_state == HIGH) {
            left_count++;
            left_direction = 1;
        } else {
            left_count--;
            left_direction = -1;
        }
    }
}

void IRAM_ATTR leftEncoderB_ISR() 
{
    uint32_t now = micros();
    if ((now - left_b_last_isr) < DEBOUNCE_US) return;
    left_b_last_isr = now;
    
    if (left_last_tick_us != 0) {
        left_interval_us = now - left_last_tick_us;
    }
    left_last_tick_us = now;
    
    int a_state = digitalRead(LEFT_ENC_A_PIN);
    int b_state = digitalRead(LEFT_ENC_B_PIN);
    
    if (b_state == HIGH) {
        if (a_state == HIGH) {
            left_count++;
            left_direction = 1;
        } else {
            left_count--;
            left_direction = -1;
        }
    } else {
        if (a_state == LOW) {
            left_count++;
            left_direction = 1;
        } else {
            left_count--;
            left_direction = -1;
        }
    }
}

void IRAM_ATTR rightEncoderA_ISR() 
{
    uint32_t now = micros();
    if ((now - right_a_last_isr) < DEBOUNCE_US) return;
    right_a_last_isr = now;
    
    if (right_last_tick_us != 0) {
        right_interval_us = now - right_last_tick_us;
    }
    right_last_tick_us = now;
    
    int a_state = digitalRead(RIGHT_ENC_A_PIN);
    int b_state = digitalRead(RIGHT_ENC_B_PIN);
    
    if (a_state == HIGH) {
        if (b_state == LOW) {
            right_count++;
            right_direction = 1;
        } else {
            right_count--;
            right_direction = -1;
        }
    } else {
        if (b_state == HIGH) {
            right_count++;
            right_direction = 1;
        } else {
            right_count--;
            right_direction = -1;
        }
    }
}

void IRAM_ATTR rightEncoderB_ISR() 
{
    uint32_t now = micros();
    if ((now - right_b_last_isr) < DEBOUNCE_US) return;
    right_b_last_isr = now;
    
    if (right_last_tick_us != 0) {
        right_interval_us = now - right_last_tick_us;
    }
    right_last_tick_us = now;
    
    int a_state = digitalRead(RIGHT_ENC_A_PIN);
    int b_state = digitalRead(RIGHT_ENC_B_PIN);
    
    if (b_state == HIGH) {
        if (a_state == HIGH) {
            right_count++;
            right_direction = 1;
        } else {
            right_count--;
            right_direction = -1;
        }
    } else {
        if (a_state == LOW) {
            right_count++;
            right_direction = 1;
        } else {
            right_count--;
            right_direction = -1;
        }
    }
}

/* ============================================================================
 * SPEED CALCULATION
 * ============================================================================ */

/**
 * @brief Calculate raw speed from tick interval with decay
 * @param interval_us Time between ticks in microseconds
 * @param direction 1 for forward, -1 for backward
 * @param last_tick_us Timestamp of last tick
 * @return Speed in mm/s (signed), 0 if timed out
 */
float calculateRawSpeed(uint32_t interval_us, int8_t direction, uint32_t last_tick_us)
{
    uint32_t now = micros();
    uint32_t time_since_tick;
    
    /* Handle micros() overflow */
    if (now >= last_tick_us) {
        time_since_tick = now - last_tick_us;
    } else {
        time_since_tick = (0xFFFFFFFF - last_tick_us) + now;
    }
    
    /* If no tick for too long, speed is 0 */
    if (last_tick_us == 0 || time_since_tick > SPEED_TIMEOUT_US || interval_us == 0) {
        return 0.0f;
    }
    
    /* Use the larger of:  last interval OR time since last tick
       This makes speed decay when slowing down */
    uint32_t effective_interval = (time_since_tick > interval_us) ? time_since_tick :  interval_us;
    
    /* Speed = distance / time = MM_PER_COUNT / (interval_us / 1000000) */
    float speed = (MM_PER_COUNT * 1000000.0f) / effective_interval;
    
    return speed * direction;
}

/**
 * @brief Apply EMA filter to speed
 * @param new_value New raw speed value
 * @param filtered Current filtered value
 * @return New filtered value
 */
float applyEMAFilter(float new_value, float filtered)
{
    return (EMA_ALPHA * new_value) + ((1.0f - EMA_ALPHA) * filtered);
}

/* ============================================================================
 * UART COMMAND PROCESSING
 * ============================================================================ */

/**
 * @brief Process received command from STM32
 * @param cmd Null-terminated command string
 */
void processCommand(const char* cmd)
{
    #if DEBUG_ENABLED
    DEBUG_SERIAL.printf("[RX] %s\n", cmd);
    #endif
    
    if (strncmp(cmd, "RST", 3) == 0) {
        /* Reset encoder counts and speed */
        noInterrupts();
        left_count = 0;
        right_count = 0;
        left_interval_us = 0;
        right_interval_us = 0;
        left_last_tick_us = 0;
        right_last_tick_us = 0;
        interrupts();
        
        left_count_prev = 0;
        right_count_prev = 0;
        left_speed_filtered = 0.0f;
        right_speed_filtered = 0.0f;
        
        #if DEBUG_ENABLED
        debug_last_left = 0;
        debug_last_right = 0;
        #endif
        
        STM32_SERIAL.println("OK: RST");
        
        #if DEBUG_ENABLED
        DEBUG_SERIAL.println("[EVENT] Reset");
        #endif
    }
    else if (strncmp(cmd, "PING", 4) == 0) {
        STM32_SERIAL.println("OK:PONG");
        
        #if DEBUG_ENABLED
        DEBUG_SERIAL.println("[EVENT] PING -> PONG");
        #endif
    }
    else if (strncmp(cmd, "GET", 3) == 0) {
        noInterrupts();
        int32_t l = left_count;
        int32_t r = right_count;
        interrupts();
        STM32_SERIAL.printf("ABS:%ld,%ld\n", l, r);
        
        #if DEBUG_ENABLED
        DEBUG_SERIAL.printf("[EVENT] GET -> ABS:%ld,%ld\n", l, r);
        #endif
    }
    else if (strncmp(cmd, "DBG:", 4) == 0) {
        #if DEBUG_ENABLED
        DEBUG_SERIAL.printf("[STM32] %s\n", cmd + 4);
        #endif
    }
    else {
        STM32_SERIAL.printf("ERR: UNKNOWN:%s\n", cmd);
        
        #if DEBUG_ENABLED
        DEBUG_SERIAL.printf("[EVENT] Unknown:  %s\n", cmd);
        #endif
    }
}

/**
 * @brief Check for and process incoming UART data from STM32
 */
void handleUARTReceive()
{
    while (STM32_SERIAL.available()) {
        char c = STM32_SERIAL.read();
        
        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                processCommand(rx_buffer);
                rx_index = 0;
            }
        } else {
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = c;
            } else {
                rx_index = 0;
            }
        }
    }
}

/* ============================================================================
 * SETUP
 * ============================================================================ */

void setup() 
{
    #if DEBUG_ENABLED
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    delay(100);
    DEBUG_SERIAL.println("\n[BOOT] ESP32 Encoder - CARMODE");
    DEBUG_SERIAL.println("[BOOT] Format: D:dL,dR,vL,vR");
    DEBUG_SERIAL.printf("[BOOT] STM32: 25Hz moving, 5Hz stopped\n");
    DEBUG_SERIAL.printf("[BOOT] Debug: event-based, HB every %ds\n", DEBUG_HEARTBEAT_INTERVAL_MS / 1000);
    DEBUG_SERIAL.printf("[BOOT] EMA alpha: %.2f\n", EMA_ALPHA);
    DEBUG_SERIAL.println("[BOOT] Ready\n");
    #endif
    
    STM32_SERIAL.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
    
    pinMode(LEFT_ENC_A_PIN, INPUT);
    pinMode(LEFT_ENC_B_PIN, INPUT);
    pinMode(RIGHT_ENC_A_PIN, INPUT);
    pinMode(RIGHT_ENC_B_PIN, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PIN), leftEncoderB_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PIN), rightEncoderB_ISR, CHANGE);
    
    last_stm32_send_time = millis();
    last_debug_event_time = millis();
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
}

/* ============================================================================
 * MAIN LOOP
 * ============================================================================ */

void loop() 
{
    uint32_t now = millis();
    
    handleUARTReceive();
    
    /* Read encoder state atomically */
    noInterrupts();
    int32_t left_now = left_count;
    int32_t right_now = right_count;
    uint32_t l_interval = left_interval_us;
    uint32_t r_interval = right_interval_us;
    uint32_t l_last_tick = left_last_tick_us;
    uint32_t r_last_tick = right_last_tick_us;
    int8_t l_dir = left_direction;
    int8_t r_dir = right_direction;
    interrupts();
    
    /* Calculate raw speeds with decay */
    float left_speed_raw = calculateRawSpeed(l_interval, l_dir, l_last_tick);
    float right_speed_raw = calculateRawSpeed(r_interval, r_dir, r_last_tick);
    
    /* Apply EMA filter */
    left_speed_filtered = applyEMAFilter(left_speed_raw, left_speed_filtered);
    right_speed_filtered = applyEMAFilter(right_speed_raw, right_speed_filtered);
    
    /* Snap to zero if very small (avoids floating point drift) */
    if (fabsf(left_speed_filtered) < 1.0f) left_speed_filtered = 0.0f;
    if (fabsf(right_speed_filtered) < 1.0f) right_speed_filtered = 0.0f;
    
    /* Determine if moving */
    bool is_moving = (left_speed_filtered != 0.0f) || (right_speed_filtered != 0.0f);
    
    /* Determine send interval based on movement */
    uint32_t send_interval = is_moving ? STM32_SEND_INTERVAL_MS : STM32_HEARTBEAT_INTERVAL_MS;
    
    /* ---- STM32 Communication ---- */
    if ((now - last_stm32_send_time) >= send_interval) {
        
        /* Calculate deltas since last send */
        int32_t delta_left = left_now - left_count_prev;
        int32_t delta_right = right_now - right_count_prev;
        
        /* Send to STM32:  D:dL,dR,vL,vR */
        STM32_SERIAL.printf("D:%ld,%ld,%d,%d\n", 
                           delta_left, delta_right,
                           (int16_t)left_speed_filtered, 
                           (int16_t)right_speed_filtered);
        
        /* Update state */
        left_count_prev = left_now;
        right_count_prev = right_now;
        last_stm32_send_time = now;
    }
    
    /* ---- Debug Output (event-based) ---- */
    #if DEBUG_ENABLED
    {
        bool has_encoder_change = (left_now != debug_last_left) || (right_now != debug_last_right);
        bool debug_heartbeat_due = (now - last_debug_event_time) >= DEBUG_HEARTBEAT_INTERVAL_MS;
        
        if (has_encoder_change) {
            /* Event:  encoder tick */
            int32_t dL = left_now - debug_last_left;
            int32_t dR = right_now - debug_last_right;
            
            DEBUG_SERIAL.printf("[ENC] d:%ld,%ld | abs:%ld,%ld | v:%d,%d mm/s\r\n",
                               dL, dR, left_now, right_now,
                               (int16_t)left_speed_filtered,
                               (int16_t)right_speed_filtered);
            
            debug_last_left = left_now;
            debug_last_right = right_now;
            last_debug_event_time = now;
        }
        else if (debug_heartbeat_due) {
            /* Heartbeat when stopped */
            DEBUG_SERIAL.println("[HB]");
            last_debug_event_time = now;
        }
    }
    #endif
    
    delayMicroseconds(100);
}

// /**
//  * @file    main.cpp
//  * @brief   ESP32 Encoder Counter for CARMODE project
//  * 
//  * Hardware:
//  *   - ESP32 WROOM DevKit1
//  *   - 2x DIY magnetic quadrature encoders (left and right rear wheels)
//  *   - Each encoder:  2x SS41 Hall effect sensors (open-drain with external pullups)
//  *   - 6 magnets per wheel (3 pole pairs)
//  *   - Resolution: 12 counts per revolution
//  * 
//  * Communication:
//  *   - UART2 to STM32 (bidirectional)
//  *   - TX Format: "D:dL,dR\n" where dL/dR are encoder deltas since last send
//  *   - RX:  Reserved for future commands from STM32
//  * 
//  * Pin assignments:
//  *   - Left encoder A:  GPIO 25
//  *   - Left encoder B:  GPIO 26
//  *   - Right encoder A: GPIO 34 (input-only, external pullup required)
//  *   - Right encoder B: GPIO 35 (input-only, external pullup required)
//  *   - UART TX to STM32: GPIO 17 (UART2 TX) -> STM32 PA10 (USART1 RX)
//  *   - UART RX from STM32: GPIO 16 (UART2 RX) <- STM32 PA9 (USART1 TX)
//  */

// #include <Arduino.h>

// /* ============================================================================
//  * CONFIGURATION
//  * ============================================================================ */

// /* Encoder pins */
// #define LEFT_ENC_A_PIN      25
// #define LEFT_ENC_B_PIN      26
// #define RIGHT_ENC_A_PIN     34
// #define RIGHT_ENC_B_PIN     35

// /* UART to STM32 (bidirectional) */
// #define STM32_SERIAL        Serial2
// #define STM32_BAUD          115200
// #define STM32_TX_PIN        17
// #define STM32_RX_PIN        16

// /* UART receive buffer */
// #define RX_BUFFER_SIZE      64

// /* Timing */

// #define DEBOUNCE_US             50      /* Debounce time for hall sensors in microseconds */
// #define HEARTBEAT_INTERVAL_MS   2000     /* Heartbeat interval when no movement */

// /* Debug output on USB Serial */
// #define DEBUG_ENABLED           1
// #define DEBUG_SERIAL            Serial
// #define DEBUG_BAUD              115200
// #define DEBUG_PRINT_INTERVAL_MS 500

// /* ============================================================================
//  * GLOBAL VARIABLES
//  * ============================================================================ */

// /* Encoder counts - volatile because modified in ISR */
// volatile int32_t left_count = 0;
// volatile int32_t right_count = 0;

// /* Previous counts for delta calculation */
// int32_t left_count_prev = 0;
// int32_t right_count_prev = 0;

// /* Timing */
// uint32_t last_send_time = 0;
// uint32_t last_debug_time = 0;

// /* Last interrupt time for debouncing */
// volatile uint32_t left_a_last_isr = 0;
// volatile uint32_t left_b_last_isr = 0;
// volatile uint32_t right_a_last_isr = 0;
// volatile uint32_t right_b_last_isr = 0;

// /* UART receive buffer */
// char rx_buffer[RX_BUFFER_SIZE];
// uint8_t rx_index = 0;

// /* ============================================================================
//  * INTERRUPT SERVICE ROUTINES
//  * 
//  * Quadrature decoding:
//  *   - On A rising:   if B is LOW, forward; if B is HIGH, backward
//  *   - On A falling:  if B is HIGH, forward; if B is LOW, backward
//  *   - On B rising:  if A is HIGH, forward; if A is LOW, backward
//  *   - On B falling: if A is LOW, forward; if A is HIGH, backward
//  * ============================================================================ */

// void IRAM_ATTR leftEncoderA_ISR() 
// {
//     uint32_t now = micros();
//     if ((now - left_a_last_isr) < DEBOUNCE_US) return;
//     left_a_last_isr = now;
    
//     int a_state = digitalRead(LEFT_ENC_A_PIN);
//     int b_state = digitalRead(LEFT_ENC_B_PIN);
    
//     if (a_state == HIGH) {
//         if (b_state == LOW) {
//             left_count++;
//         } else {
//             left_count--;
//         }
//     } else {
//         if (b_state == HIGH) {
//             left_count++;
//         } else {
//             left_count--;
//         }
//     }
// }

// void IRAM_ATTR leftEncoderB_ISR() 
// {
//     uint32_t now = micros();
//     if ((now - left_b_last_isr) < DEBOUNCE_US) return;
//     left_b_last_isr = now;
    
//     int a_state = digitalRead(LEFT_ENC_A_PIN);
//     int b_state = digitalRead(LEFT_ENC_B_PIN);
    
//     if (b_state == HIGH) {
//         if (a_state == HIGH) {
//             left_count++;
//         } else {
//             left_count--;
//         }
//     } else {
//         if (a_state == LOW) {
//             left_count++;
//         } else {
//             left_count--;
//         }
//     }
// }

// void IRAM_ATTR rightEncoderA_ISR() 
// {
//     uint32_t now = micros();
//     if ((now - right_a_last_isr) < DEBOUNCE_US) return;
//     right_a_last_isr = now;
    
//     int a_state = digitalRead(RIGHT_ENC_A_PIN);
//     int b_state = digitalRead(RIGHT_ENC_B_PIN);
    
//     if (a_state == HIGH) {
//         if (b_state == LOW) {
//             right_count++;
//         } else {
//             right_count--;
//         }
//     } else {
//         if (b_state == HIGH) {
//             right_count++;
//         } else {
//             right_count--;
//         }
//     }
// }

// void IRAM_ATTR rightEncoderB_ISR() 
// {
//     uint32_t now = micros();
//     if ((now - right_b_last_isr) < DEBOUNCE_US) return;
//     right_b_last_isr = now;
    
//     int a_state = digitalRead(RIGHT_ENC_A_PIN);
//     int b_state = digitalRead(RIGHT_ENC_B_PIN);
    
//     if (b_state == HIGH) {
//         if (a_state == HIGH) {
//             right_count++;
//         } else {
//             right_count--;
//         }
//     } else {
//         if (a_state == LOW) {
//             right_count++;
//         } else {
//             right_count--;
//         }
//     }
// }

// /* ============================================================================
//  * UART COMMAND PROCESSING
//  * ============================================================================ */

// /**
//  * @brief Process received command from STM32
//  * @param cmd Null-terminated command string
//  */
// void processCommand(const char* cmd)
// {
//     #if DEBUG_ENABLED
//     DEBUG_SERIAL.printf("RX from STM32: %s\n", cmd);
//     #endif
    
//     /* Command parsing - add your commands here */
//     if (strncmp(cmd, "RST", 3) == 0) {
//         /* Reset encoder counts */
//         noInterrupts();
//         left_count = 0;
//         right_count = 0;
//         interrupts();
//         left_count_prev = 0;
//         right_count_prev = 0;
        
//         STM32_SERIAL.println("OK: RST");
        
//         #if DEBUG_ENABLED
//         DEBUG_SERIAL.println("Encoder counts reset");
//         #endif
//     }
//     else if (strncmp(cmd, "PING", 4) == 0) {
//         /* Ping/pong for connection test */
//         STM32_SERIAL.println("OK:PONG");
//     }
//     else if (strncmp(cmd, "GET", 3) == 0) {
//         /* Get absolute counts (not deltas) */
//         noInterrupts();
//         int32_t l = left_count;
//         int32_t r = right_count;
//         interrupts();
//         STM32_SERIAL.printf("ABS:%ld,%ld\n", l, r);
//     }
//     else if (strncmp(cmd, "DBG:", 4) == 0) {
//         /* Debug message pass-through */
//         #if DEBUG_ENABLED
//         DEBUG_SERIAL.printf("STM32 DBG: %s\n", cmd + 4);
//         #endif
//     }
//     else {
//         /* Unknown command */
//         STM32_SERIAL.printf("ERR: UNKNOWN:%s\n", cmd);
        
//         #if DEBUG_ENABLED
//         DEBUG_SERIAL.printf("Unknown command: %s\n", cmd);
//         #endif
//     }
// }

// /**
//  * @brief Check for and process incoming UART data from STM32
//  */
// void handleUARTReceive()
// {
//     while (STM32_SERIAL. available()) {
//         char c = STM32_SERIAL.read();
        
//         if (c == '\n' || c == '\r') {
//             if (rx_index > 0) {
//                 rx_buffer[rx_index] = '\0';
//                 processCommand(rx_buffer);
//                 rx_index = 0;
//             }
//         } else {
//             if (rx_index < RX_BUFFER_SIZE - 1) {
//                 rx_buffer[rx_index++] = c;
//             } else {
//                 /* Buffer overflow - reset */
//                 rx_index = 0;
//             }
//         }
//     }
// }

// /* ============================================================================
//  * SETUP
//  * ============================================================================ */

// void setup() 
// {
//     #if DEBUG_ENABLED
//     DEBUG_SERIAL.begin(DEBUG_BAUD);
//     delay(100);
//     DEBUG_SERIAL.println("\n\n========================================");
//     DEBUG_SERIAL.println("   ESP32 Encoder Counter - CARMODE");
//     DEBUG_SERIAL.println("========================================");
//     DEBUG_SERIAL.println("Initializing...\n");
//     #endif
    
//     /* Initialize bidirectional UART to STM32 */
//     STM32_SERIAL.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
    
//     /* Configure encoder pins as inputs
//      * GPIO 25, 26: Have internal pullup available (but using external)
//      * GPIO 34, 35: Input-only, NO internal pullup (using external)
//      * All SS41 sensors have external pullups installed
//      */
//     pinMode(LEFT_ENC_A_PIN, INPUT);
//     pinMode(LEFT_ENC_B_PIN, INPUT);
//     pinMode(RIGHT_ENC_A_PIN, INPUT);
//     pinMode(RIGHT_ENC_B_PIN, INPUT);
    
//     /* Attach interrupts for quadrature decoding */
//     attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncoderA_ISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PIN), leftEncoderB_ISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncoderA_ISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PIN), rightEncoderB_ISR, CHANGE);
    
//     #if DEBUG_ENABLED
//     DEBUG_SERIAL.println("Pin configuration:");
//     DEBUG_SERIAL.printf("  Left encoder A:   GPIO %d\n", LEFT_ENC_A_PIN);
//     DEBUG_SERIAL.printf("  Left encoder B:  GPIO %d\n", LEFT_ENC_B_PIN);
//     DEBUG_SERIAL.printf("  Right encoder A: GPIO %d\n", RIGHT_ENC_A_PIN);
//     DEBUG_SERIAL.printf("  Right encoder B: GPIO %d\n", RIGHT_ENC_B_PIN);
//     DEBUG_SERIAL.println("  (All sensors have external pullups)");
//     DEBUG_SERIAL.println();
//     DEBUG_SERIAL.printf("UART to STM32: %d baud\n", STM32_BAUD);
//     DEBUG_SERIAL.printf("  TX: GPIO %d -> STM32 PA10 (USART1 RX)\n", STM32_TX_PIN);
//     DEBUG_SERIAL.printf("  RX: GPIO %d <- STM32 PA9  (USART1 TX)\n", STM32_RX_PIN);
//     DEBUG_SERIAL.println();
//     DEBUG_SERIAL.println("TX format: \"D:dL,dR\\n\"");
//     DEBUG_SERIAL.println("  dL = left wheel delta (counts)");
//     DEBUG_SERIAL.println("  dR = right wheel delta (counts)");
//     DEBUG_SERIAL.println("  12 counts = 1 revolution = 205mm");
//     DEBUG_SERIAL.println();
//     DEBUG_SERIAL.println("RX commands from STM32:");
//     DEBUG_SERIAL.println("  RST       - Reset encoder counts");
//     DEBUG_SERIAL.println("  PING      - Connection test (replies PONG)");
//     DEBUG_SERIAL.println("  GET       - Get absolute counts");
//     DEBUG_SERIAL.println("  DBG:xxx   - Debug message pass-through");
//     DEBUG_SERIAL.println();
//     DEBUG_SERIAL.println("----------------------------------------");
//     DEBUG_SERIAL.println("Running...\n");
//     #endif
    
//     /* Initialize timing */
//     last_send_time = millis();
//     last_debug_time = millis();
    
//     /* Clear receive buffer */
//     rx_index = 0;
//     memset(rx_buffer, 0, RX_BUFFER_SIZE);
// }

// /* ============================================================================
//  * MAIN LOOP
//  * ============================================================================ */

// void loop() 
// {
//     uint32_t now = millis();
    
//     handleUARTReceive();
    
//     /* Read encoder counts atomically */
//     noInterrupts();
//     int32_t left_now = left_count;
//     int32_t right_now = right_count;
//     interrupts();
    
//     /* Calculate deltas */
//     int32_t delta_left = left_now - left_count_prev;
//     int32_t delta_right = right_now - right_count_prev;
    
//     /* Check conditions */
//     bool has_change = (delta_left != 0) || (delta_right != 0);
//     bool heartbeat_due = (now - last_send_time) >= HEARTBEAT_INTERVAL_MS;
    
//     /* Send immediately on change, or heartbeat if stopped */
//     if (has_change || heartbeat_due) {
        
//         STM32_SERIAL.printf("D:%ld,%ld\n", delta_left, delta_right);
        
//         #if DEBUG_ENABLED
//         if (has_change) {
//             DEBUG_SERIAL.printf("[ENC] dL:%ld dR:%ld | L:%ld R:%ld\n", 
//                                delta_left, delta_right, left_now, right_now);
//         } else {
//             DEBUG_SERIAL. println("[HB]");
//         }
//         #endif
        
//         left_count_prev = left_now;
//         right_count_prev = right_now;
//         last_send_time = now;
//     }
    
//     delayMicroseconds(100);
// }