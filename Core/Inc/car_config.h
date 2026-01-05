/**
 * @file    car_config.h
 * @brief   Central configuration file for CARMODE project
 *          All tuneable parameters in one place
 */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * PHYSICAL PARAMETERS
 * ============================================================================ */
#define CAR_WHEELBASE_MM        170.0f    // Distance between front and rear axles
#define CAR_REAR_TRACK_MM       150.0f    // Distance between rear wheels
#define CAR_FRONT_TRACK_MM      155.0f    // Distance between front wheels
#define CAR_WHEEL_CIRCUMFERENCE 205.0f    // Wheel circumference in mm
#define CAR_WEIGHT_KG           1.25f     // Approximate car weight

/* ============================================================================
 * ENCODER PARAMETERS
 * ============================================================================ */
#define ENCODER_COUNTS_PER_REV  12        // 6 magnets * 2 sensors
#define ENCODER_MM_PER_COUNT    17.08f    // 205mm / 12 counts

/* ============================================================================
 * SERVO PARAMETERS (TIM21, 50Hz, ARR=7999)
 * 
 * IMPORTANT: Mechanical linkage ratio is 5:1
 *   - Software angle: ±5° max
 *   - Actual wheel angle: ±25° max
 * ============================================================================ */
#define SERVO_CENTER_CCR        650       // CCR value for 0 degrees
#define SERVO_CCR_PER_DEGREE    25        // CCR change per SOFTWARE degree
#define SERVO_MAX_ANGLE_SW      5         // Maximum SOFTWARE steering angle (degrees)
#define SERVO_LINKAGE_RATIO     5.0f      // Mechanical ratio:  wheel_angle = sw_angle * ratio
#define SERVO_MAX_ANGLE_WHEEL   25        // Actual maximum WHEEL angle (degrees)
#define SERVO_MIN_CCR           (SERVO_CENTER_CCR - (SERVO_MAX_ANGLE_SW * SERVO_CCR_PER_DEGREE))  // 525
#define SERVO_MAX_CCR           (SERVO_CENTER_CCR + (SERVO_MAX_ANGLE_SW * SERVO_CCR_PER_DEGREE))  // 775

/* ============================================================================
 * MOTOR PARAMETERS (TIM22, 25kHz, ARR=1279)
 * ============================================================================ */
#define MOTOR_PWM_MAX           1279      // Maximum PWM value (100% duty)
#define MOTOR_PWM_MIN           0         // Minimum PWM value
#define MOTOR_PWM_LIMIT         300      // Software limit (~78% duty) for safety

/* Speed settings in mm/s */
#define SPEED_CRUISE_MIN        150.0f    // Minimum cruise speed
#define SPEED_CRUISE_MAX        300.0f    // Maximum cruise speed
#define SPEED_APPROACH          100.0f    // Approach/careful speed
#define SPEED_MANEUVER          150.0f    // Speed during obstacle avoidance
#define SPEED_CRAWL             75.0f    // Very slow speed for precision

/* PWM estimates (will be refined by PID) */
#define PWM_CRUISE              200       // ~27% duty for cruise
#define PWM_APPROACH            100      // ~16% duty for approach
#define PWM_MANEUVER            150       // ~20% duty for maneuver

/* ============================================================================
 * ULTRASONIC PARAMETERS
 * ============================================================================ */
/* Trigger pulse duration in microseconds */
#define US_TRIGGER_PULSE_US     12        // 10us minimum, use 12 for safety

/* Speed of sound:  343 m/s = 0.343 mm/us, round trip = 0.1715 mm/us */
#define US_MM_PER_US            0.1715f   // mm per microsecond (round trip)

/* Maximum ranges in mm */
//#define US_FRONT_MAX_RANGE      4000      // 1.5m front sensor range
//#define US_SIDE_MAX_RANGE_MODE1 1000      // 1.0m side range for obstacle avoidance
//#define US_SIDE_MAX_RANGE_MODE2 1500      // 1.5m side range for wall following
//#define US_SIDE_MAX_RANGE 			4000
//#define US_REAR_MAX_RANGE       4000       // 0.5m rear sensor range

/* Timeout values in microseconds (range / 0.1715 + margin) */
//#define US_FRONT_TIMEOUT_US     40000      // ~1.5m range
//#define US_SIDE_TIMEOUT_US     	40000      // ~1.5m range
//#define US_SIDE_TIMEOUT_MODE1   6000      // ~1.0m range
//#define US_SIDE_TIMEOUT_MODE2   9000      // ~1.5m range
//#define US_REAR_TIMEOUT_US      40000      // ~0.5m range

/* Measurement interval */
#define US_SEQUENCE_INTERVAL_MS 10        // Time between sensor triggers

/* ============================================================================
 * TASK 1: OBSTACLE AVOIDANCE PARAMETERS
 * ============================================================================ */
#define TASK1_DISTANCE_SHORT    3005      // Short distance in mm
#define TASK1_DISTANCE_LONG     7515      // Long distance in mm
#define TASK1_ACCURACY          100       // Target accuracy ±100mm
#define TASK1_TRACK_WIDTH       1200      // Track width in mm
#define TASK1_SAFE_ZONE         600       // Safe zone on each side (no obstacles)
#define TASK1_OBSTACLE_MAX_DIA  400       // Max obstacle diameter
#define TASK1_OBSTACLE_CLEARANCE_MIN 100  // Min clearance from obstacle
#define TASK1_OBSTACLE_CLEARANCE_MAX 300  // Max clearance from obstacle
#define TASK1_OBSTACLE_CLEARANCE_TARGET 200 // Target clearance

/* Detection thresholds */
#define TASK1_OBSTACLE_DETECT_DIST  500   // Start avoidance when obstacle at this distance
#define TASK1_APPROACH_ZONE         500   // Slow down this far before goal
#define TASK1_STOP_ZONE             50    // Start stopping this far before goal

/* ============================================================================
 * TASK 2: WALL FOLLOWING PARAMETERS
 * ============================================================================ */
#define TASK2_TRACK_WIDTH_MIN   500       // Minimum track width
#define TASK2_TRACK_WIDTH_MAX   2500      // Maximum track width
#define TASK2_WALL_HEIGHT_MIN   150       // Minimum wall height

/* Wall following thresholds */
#define TASK2_MIN_WALL_DIST     150       // Minimum safe distance from wall
#define TASK2_TARGET_CENTER     0         // Target error (0 = centered)
#define TASK2_FRONT_SLOW_DIST   800       // Slow down if front obstacle at this distance
#define TASK2_FRONT_STOP_DIST   200       // Emergency stop distance
#define TASK2_CAR_DETECT_DIST   600       // Distance to detect car ahead (vs wall)

/* ============================================================================
 * PID PARAMETERS
 * ============================================================================ */
/* Speed PID (controls motor PWM based on encoder feedback) */
#define PID_SPEED_KP            1.0f
#define PID_SPEED_KI            0.0f
#define PID_SPEED_KD            0.0f
#define PID_SPEED_MAX_OUTPUT    MOTOR_PWM_LIMIT
#define PID_SPEED_MIN_OUTPUT    0

/* Steering PID for wall following (output is SOFTWARE angle ±5°) */
#define PID_STEER_KP            0.05f     // SW degrees per mm of error
#define PID_STEER_KI            0.001f
#define PID_STEER_KD            0.02f
#define PID_STEER_MAX_OUTPUT    ((float)SERVO_MAX_ANGLE_SW)   // 5.0
#define PID_STEER_MIN_OUTPUT    (-(float)SERVO_MAX_ANGLE_SW)  // -5.0

/* Heading PID for straight line driving (output is SOFTWARE angle) */
#define PID_HEADING_KP          0.1f
#define PID_HEADING_KI          0.01f
#define PID_HEADING_KD          0.05f

/* ============================================================================
 * BUTTON DEBOUNCE
 * ============================================================================ */
#define BUTTON_DEBOUNCE_MS      300       // Debounce time in milliseconds

/* ============================================================================
 * CONTROL LOOP TIMING
 * ============================================================================ */
#define CONTROL_LOOP_PERIOD_MS  20        // 50Hz control loop
#define LED_BLINK_FAST_MS       100       // Fast blink period
#define LED_BLINK_SLOW_MS       500       // Slow blink period

/* ============================================================================
 * UART BUFFER SIZES
 * ============================================================================ */
#define UART_RX_BUFFER_SIZE     32        // ESP32 receive buffer
#define UART_TX_BUFFER_SIZE     128       // Debug transmit buffer

/* ============================================================================
 * ACKERMANN GEOMETRY CALCULATIONS
 * ============================================================================ */
/* Minimum turn radius at max WHEEL angle (mm) */
/* R = wheelbase / tan(wheel_angle) = 170 / tan(25°) ≈ 365mm */
#define CAR_MIN_TURN_RADIUS     365.0f

#ifdef __cplusplus
}
#endif

#endif /* CAR_CONFIG_H */