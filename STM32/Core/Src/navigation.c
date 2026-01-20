/**
 * @file    navigation.c
 * @brief   Navigation logic implementation
 * 
 * NOTE: All steering commands use SOFTWARE angles (±5°)
 *       Actual wheel angles are ±25° due to 5:  1 linkage
 *       Ackermann calculations use wheel angles internally
 */

#include "navigation.h"
#include "utils.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================================
 * PRIVATE FUNCTIONS - MODE 1 (OBSTACLE AVOIDANCE)
 * ============================================================================ */

static void Nav1_EnterState(Navigation_Handle_t *nav, Nav1_State_t new_state)
{
    nav->nav1_state = new_state;
    nav->state_entry_time = HAL_GetTick();
    
    switch (new_state) {
        case NAV1_STATE_DRIVE_STRAIGHT:
            nav->target_speed_mm_s = SPEED_CRUISE_MIN;
            Servo_SetAngle(nav->servo, 0);
            PID_SetSetpoint(&nav->pid_heading, 0.0f);
            break;
            
        case NAV1_STATE_ARC_OUT: 
            nav->arc_distance_compensation = 0;
            nav->target_speed_mm_s = SPEED_MANEUVER;
            nav->arc_start_distance = Encoder_GetDistance(nav->encoder);
            /* arc_steering_angle is SOFTWARE angle (±5) */
            Servo_SetAngle(nav->servo, nav->arc_steering_angle);
            break;
				
				case NAV1_STATE_DRIVE_PAST:
						nav->target_speed_mm_s = SPEED_MANEUVER;
						nav->arc_start_distance = Encoder_GetDistance(nav->encoder);
						Servo_SetAngle(nav->servo, 0);
						break;
            
        case NAV1_STATE_ARC_BACK: 
            nav->target_speed_mm_s = SPEED_MANEUVER;
            nav->arc_start_distance = Encoder_GetDistance(nav->encoder);
            /* Reverse steering to return to centerline */
            Servo_SetAngle(nav->servo, -nav->arc_steering_angle);
            break;
            
        case NAV1_STATE_APPROACH: 
            nav->target_speed_mm_s = SPEED_APPROACH;
            Servo_SetAngle(nav->servo, 0);
            break;
            
        case NAV1_STATE_STOPPING:  
            nav->target_speed_mm_s = 0.0f;
            Motor_Brake(nav->motor);
            Servo_SetAngle(nav->servo, 0);
            break;
            
        case NAV1_STATE_COMPLETE:  
            Motor_Stop(nav->motor);
            Servo_SetAngle(nav->servo, 0);
            break;
            
        default:
            break;
    }
}

static void Nav1_AddArcCompensation(Navigation_Handle_t *nav, float arc_traveled)
{
    float wheel_angle = fabsf((float)nav->arc_steering_angle * SERVO_LINKAGE_RATIO);
    float turn_radius = fabsf(Ackermann_GetTurnRadius(wheel_angle));
    
    if (turn_radius > 10.0f) {
        float arc_angle_rad = arc_traveled / turn_radius;
        float forward_progress = turn_radius * sinf(arc_angle_rad);
        nav->arc_distance_compensation += (arc_traveled - forward_progress);
    }
}

static void Nav1_UpdateDriveStraight(Navigation_Handle_t *nav)
{
    float distance = Encoder_GetDistance(nav->encoder);
    float effective_distance = distance - nav->arc_distance_compensation;
    float remaining = nav->target_distance_mm - effective_distance;
    
    uint32_t time_in_state = HAL_GetTick() - nav->state_entry_time;
    if (time_in_state < STARTUP_GRACE_PERIOD_MS) {
        Servo_SetAngle(nav->servo, 0);  /* Keep straight */
        nav->obstacle_confirm_count = 0;
        return;
    }
    
    /* Check for obstacle */
    bool front_valid = (nav->dist_front > 30 && nav->dist_front < 4000);
    
    if (front_valid && nav->dist_front < TASK1_OBSTACLE_DETECT_DIST) {
        nav->obstacle_confirm_count++;
        if (nav->obstacle_confirm_count >= OBSTACLE_CONFIRM_THRESHOLD) {
            nav->obstacle_distance_at_detect = distance;
            nav->nav1_state = NAV1_STATE_OBSTACLE_DETECTED;
            nav->obstacle_confirm_count = 0;
            return;
        }
    } else {
        nav->obstacle_confirm_count = 0;
    }
    
    /* Check if approaching target */
    if (remaining < TASK1_APPROACH_ZONE) {
        Nav1_EnterState(nav, NAV1_STATE_APPROACH);
        return;
    }
    
    /* Use heading PID for straight line correction */
    float heading_error = Encoder_GetHeading(nav->encoder);
    float dt = CONTROL_LOOP_PERIOD_MS / 1000.0f;
    
    PID_SetSetpoint(&nav->pid_heading, 0.0f);
    float steering_correction = PID_Compute(&nav->pid_heading, heading_error, dt);
    
    /* Limit to max SOFTWARE steering angle (±5°) */
    if (steering_correction > SERVO_MAX_ANGLE_SW) steering_correction = SERVO_MAX_ANGLE_SW;
    if (steering_correction < -SERVO_MAX_ANGLE_SW) steering_correction = -SERVO_MAX_ANGLE_SW;
    
    Servo_SetAngle(nav->servo, (int8_t)steering_correction);
}

static void Nav1_UpdateObstacleDetected(Navigation_Handle_t *nav)
{
    /* Decide which way to go around - prefer left */
    if (nav->dist_left >= nav->dist_right || 
        nav->dist_left > (TASK1_OBSTACLE_CLEARANCE_TARGET + TASK1_OBSTACLE_MAX_DIA / 2)) {
        nav->avoid_direction = AVOID_LEFT;
        nav->arc_steering_angle = -SERVO_MAX_ANGLE_SW;  /* Full left = -5° SW = -25° wheel */
    } else {
        nav->avoid_direction = AVOID_RIGHT;
        nav->arc_steering_angle = SERVO_MAX_ANGLE_SW;   /* Full right = +5° SW = +25° wheel */
    }
    
    /* Store the detection distance for exit condition */
    nav->obstacle_detect_front_dist = nav->dist_front;
    
    /* Reset arc tracking */
    nav->arc_out_distance = 0;
    
    Nav1_EnterState(nav, NAV1_STATE_ARC_OUT);
}

static void Nav1_UpdateArcOut(Navigation_Handle_t *nav)
{
    float arc_traveled = Encoder_GetDistance(nav->encoder) - nav->arc_start_distance;
    
    /* Get the side sensor that should see the obstacle (opposite of turn direction) */
    uint16_t side_dist = (nav->avoid_direction == AVOID_LEFT) ? 
                         nav->dist_right : nav->dist_left;
    
    /* Exit condition: front is clear AND obstacle is now beside/behind us */
    bool front_clear = (nav->dist_front > nav->obstacle_detect_front_dist) || 
                       (nav->dist_front > TASK1_OBSTACLE_DETECT_DIST);
    bool obstacle_passed = (side_dist > nav->obstacle_detect_front_dist) ||
                           (side_dist > TASK1_OBSTACLE_DETECT_DIST);
    
    /* Minimum arc distance to prevent premature exit */
    bool min_arc_done = (arc_traveled > 200.0f);
    
    if (front_clear && obstacle_passed && min_arc_done) {
        /* Store how far we traveled for ARC_BACK */
        nav->arc_out_distance = arc_traveled;
        
        Nav1_AddArcCompensation(nav, arc_traveled);
        Nav1_EnterState(nav, NAV1_STATE_DRIVE_PAST);
        return;
    }
    
    /* Safety:  don't arc forever */
    if (arc_traveled > 1500.0f) {
        nav->arc_out_distance = arc_traveled;
        Nav1_AddArcCompensation(nav, arc_traveled);
        Nav1_EnterState(nav, NAV1_STATE_ARC_BACK);
        return;
    }
    
    /* Maintain steering */
    Servo_SetAngle(nav->servo, nav->arc_steering_angle);
}

static void Nav1_UpdateDrivePast(Navigation_Handle_t *nav)
{
    float distance_in_state = Encoder_GetDistance(nav->encoder) - nav->arc_start_distance;
    
    /* Drive straight for 200mm (20cm) to clear the obstacle */
    #define DRIVE_PAST_DISTANCE 200.0f
    
    if (distance_in_state >= DRIVE_PAST_DISTANCE) {
        Nav1_EnterState(nav, NAV1_STATE_ARC_BACK);
        return;
    }
    
    /* Keep driving straight */
    Servo_SetAngle(nav->servo, 0);
}

static void Nav1_UpdateArcBack(Navigation_Handle_t *nav)
{
    float arc_traveled = Encoder_GetDistance(nav->encoder) - nav->arc_start_distance;
    
    /* Turn back for HALF the distance we turned out */
    float target_arc = nav->arc_out_distance * 0.5f;
    
    if (arc_traveled >= target_arc) {
        Nav1_AddArcCompensation(nav, arc_traveled);
        Nav1_EnterState(nav, NAV1_STATE_DRIVE_STRAIGHT);
        return;
    }
    
    /* Safety: don't arc forever */
    if (arc_traveled > 1000.0f) {
        Nav1_AddArcCompensation(nav, arc_traveled);
        Nav1_EnterState(nav, NAV1_STATE_DRIVE_STRAIGHT);
        return;
    }
    
    /* Maintain opposite steering (already set in Nav1_EnterState) */
}

static void Nav1_UpdateApproach(Navigation_Handle_t *nav)
{
    float distance = Encoder_GetDistance(nav->encoder);
    float effective_distance = distance - nav->arc_distance_compensation;  
    float remaining = nav->target_distance_mm - effective_distance;  
    
    /* Check for obstacle even in approach phase */
    if (nav->dist_front < TASK1_OBSTACLE_DETECT_DIST / 2 && nav->dist_front > 0) {
        nav->obstacle_distance_at_detect = distance;
        nav->nav1_state = NAV1_STATE_OBSTACLE_DETECTED;
        return;
    }
    
    /* Check if close enough to stop */
    if (remaining < TASK1_STOP_ZONE) {
        Nav1_EnterState(nav, NAV1_STATE_STOPPING);
        return;
    }
    
    /* Gradual slowdown */
    float speed_factor = remaining / TASK1_APPROACH_ZONE;
    if (speed_factor < 0.3f) speed_factor = 0.3f;
    nav->target_speed_mm_s = SPEED_APPROACH * speed_factor + SPEED_CRAWL * (1.0f - speed_factor);
    
    /* Maintain straight heading */
    float heading_error = Encoder_GetHeading(nav->encoder);
    float dt = CONTROL_LOOP_PERIOD_MS / 1000.0f;
    float steering_correction = PID_Compute(&nav->pid_heading, heading_error, dt);
    
    if (steering_correction > SERVO_MAX_ANGLE_SW) steering_correction = SERVO_MAX_ANGLE_SW;
    if (steering_correction < -SERVO_MAX_ANGLE_SW) steering_correction = -SERVO_MAX_ANGLE_SW;
    
    Servo_SetAngle(nav->servo, (int8_t)steering_correction);
}

static void Nav1_UpdateStopping(Navigation_Handle_t *nav)
{
    float distance = Encoder_GetDistance(nav->encoder);
    float remaining = nav->target_distance_mm - distance;
    float speed = Encoder_GetSpeed(nav->encoder);
    
    if (remaining <= 0 || fabsf(speed) < 10.0f) {
        Nav1_EnterState(nav, NAV1_STATE_COMPLETE);
    }
}

static void Nav1_Update(Navigation_Handle_t *nav)
{
    switch (nav->nav1_state) {
        case NAV1_STATE_IDLE:
            break;
        case NAV1_STATE_DRIVE_STRAIGHT:
            Nav1_UpdateDriveStraight(nav);
            break;
        case NAV1_STATE_OBSTACLE_DETECTED:
            Nav1_UpdateObstacleDetected(nav);
            break;
        case NAV1_STATE_ARC_OUT:
            Nav1_UpdateArcOut(nav);
            break;
				case NAV1_STATE_DRIVE_PAST:
						Nav1_UpdateDrivePast(nav);
						break;
        case NAV1_STATE_ARC_BACK:
            Nav1_UpdateArcBack(nav);
            break;
        case NAV1_STATE_APPROACH:
            Nav1_UpdateApproach(nav);
            break;
        case NAV1_STATE_STOPPING:
            Nav1_UpdateStopping(nav);
            break;
        case NAV1_STATE_COMPLETE:
            break;
        default:
            break;
    }
    
    /* Update motor speed using PID */
    if (nav->nav1_state != NAV1_STATE_STOPPING && 
        nav->nav1_state != NAV1_STATE_COMPLETE &&
        nav->nav1_state != NAV1_STATE_IDLE) {
        
        float current_speed = Encoder_GetSpeed(nav->encoder);
        float dt = CONTROL_LOOP_PERIOD_MS / 1000.0f;
        
        PID_SetSetpoint(&nav->pid_speed, nav->target_speed_mm_s);
        float pwm_output = PID_Compute(&nav->pid_speed, current_speed, dt);
            
        /* Add minimum PWM offset - motor can't move below 250 */
        if (nav->target_speed_mm_s > 0) {
            uint16_t pwm = MOTOR_PWM_MIN + (uint16_t)fmaxf(pwm_output, 0);
            if (pwm > MOTOR_PWM_LIMIT) pwm = MOTOR_PWM_LIMIT;
            Motor_Set(nav->motor, MOTOR_DIR_FORWARD, pwm);
        } else {
            Motor_Stop(nav->motor);
        }
    }
}

/* ============================================================================
 * PRIVATE FUNCTIONS - MODE 2 (WALL FOLLOWING)
 * ============================================================================ */

static void Nav2_Update(Navigation_Handle_t *nav)
{
    float dt = CONTROL_LOOP_PERIOD_MS / 1000.0f;
    
    switch (nav->nav2_state) {
        case NAV2_STATE_IDLE:
            break;
            
        case NAV2_STATE_RUNNING:
        {
            static float filtered_error = 0;
            float raw_error = (float)nav->dist_right - (float)nav->dist_left;
            filtered_error = 0.15f * raw_error + 0.85f * filtered_error;  /* Heavy filtering */
            
            /* Large dead zone for straight driving */
            float steering_angle = 0.0f;
            
            #define DEADZONE 80.0f    /* Big dead zone - 80mm */
            #define GAIN     0.01f    /* Gentle response */
            
            if (filtered_error > DEADZONE) {
                steering_angle = GAIN * (filtered_error - DEADZONE);
            } else if (filtered_error < -DEADZONE) {
                steering_angle = GAIN * (filtered_error + DEADZONE);
            }
            /* else: go straight */
            
            /* Clamp */
            if (steering_angle > 5.0f) steering_angle = 5.0f;
            if (steering_angle < -5.0f) steering_angle = -5.0f;
            
            /* Apply with inverted sign */
            Servo_SetAngle(nav->servo, (int8_t)steering_angle);
            
            if (nav->dist_front < TASK2_FRONT_STOP_DIST && nav->dist_front > 0) {
                nav->nav2_state = NAV2_STATE_EMERGENCY_STOP;
                Motor_Brake(nav->motor);
                return;
            }
            
            if (nav->dist_front < TASK2_FRONT_SLOW_DIST && nav->dist_front > 0) {
                uint16_t avg_side = (nav->dist_left + nav->dist_right) / 2;
                
                if (nav->dist_front < avg_side) {
                    nav->target_speed_mm_s = (nav->dist_front < TASK2_CAR_DETECT_DIST) ? 
                                             SPEED_CRAWL : SPEED_APPROACH;
                } else {
                    nav->target_speed_mm_s = SPEED_MANEUVER;
                }
            } else {
                uint16_t track_width = nav->dist_left + nav->dist_right;
                if (track_width > 1500) {
                    nav->target_speed_mm_s = SPEED_CRUISE_MAX;
                } else if (track_width > 800) {
                    nav->target_speed_mm_s = SPEED_CRUISE_MIN;
                } else {
                    nav->target_speed_mm_s = SPEED_MANEUVER;
                }
            }
            
            /* Update motor speed */
            float current_speed = Encoder_GetSpeed(nav->encoder);
            PID_SetSetpoint(&nav->pid_speed, nav->target_speed_mm_s);
            float pwm_output = PID_Compute(&nav->pid_speed, current_speed, dt);
            
            /* Add minimum PWM offset - motor can't move below 250 */
            if (nav->target_speed_mm_s > 0) {
                uint16_t pwm = MOTOR_PWM_MIN + (uint16_t)fmaxf(pwm_output, 0);
                if (pwm > MOTOR_PWM_LIMIT) pwm = MOTOR_PWM_LIMIT;
                Motor_Set(nav->motor, MOTOR_DIR_FORWARD, pwm);
            } else {
                Motor_Stop(nav->motor);
            }
            break;
        }
            
        case NAV2_STATE_OBSTACLE_AHEAD:
            if (nav->dist_front > TASK2_CAR_DETECT_DIST) {
                nav->nav2_state = NAV2_STATE_RUNNING;
            }
            break;
            
        case NAV2_STATE_EMERGENCY_STOP:
            Motor_Brake(nav->motor);
            if (nav->dist_front > TASK2_FRONT_STOP_DIST * 2) {
                nav->nav2_state = NAV2_STATE_RUNNING;
            }
            break;
            
        default:
            break;
    }
}

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

void Navigation_Init(Navigation_Handle_t *nav,
                     Motor_Handle_t *motor,
                     Servo_Handle_t *servo,
                     US_Handle_t *ultrasonic,
                     Encoder_Handle_t *encoder,
                     Buttons_Handle_t *buttons)
{
    nav->motor = motor;
    nav->servo = servo;
    nav->ultrasonic = ultrasonic;
    nav->encoder = encoder;
    nav->buttons = buttons;
    
    /* Initialize PIDs with outputs scaled for SOFTWARE angles (±5°) */
    PID_Init(&nav->pid_speed, PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD,
             PID_SPEED_MIN_OUTPUT, PID_SPEED_MAX_OUTPUT);
    
    PID_Init(&nav->pid_steering, PID_STEER_KP, PID_STEER_KI, PID_STEER_KD,
             PID_STEER_MIN_OUTPUT, PID_STEER_MAX_OUTPUT);  /* ±5° */
    
    PID_Init(&nav->pid_heading, PID_HEADING_KP, PID_HEADING_KI, PID_HEADING_KD,
             -(float)SERVO_MAX_ANGLE_SW, (float)SERVO_MAX_ANGLE_SW);  /* ±5° */
    
    nav->nav1_state = NAV1_STATE_IDLE;
    nav->nav2_state = NAV2_STATE_IDLE;
    nav->avoid_direction = AVOID_NONE;
    
    nav->target_distance_mm = 0;
    nav->target_speed_mm_s = 0.0f;
    nav->current_pwm = 0;
    
    nav->dist_front = 0xFFFF;
    nav->dist_left = 0xFFFF;
    nav->dist_right = 0xFFFF;
    nav->dist_rear = 0xFFFF;
    
    nav->last_update_time = 0;
    nav->state_entry_time = 0;
    
    /* Initialize new fields */
    nav->arc_distance_compensation = 0.0f;
    nav->arc_out_distance = 0.0f;
    nav->obstacle_detect_front_dist = 0;
    
    nav->is_initialized = true;
    nav->emergency_stop = false;
}

void Navigation_Start(Navigation_Handle_t *nav, Car_Mode_t mode, uint16_t target_distance)
{
    Encoder_ResetOdometry(nav->encoder);
    
    PID_Reset(&nav->pid_speed);
    PID_Reset(&nav->pid_steering);
    PID_Reset(&nav->pid_heading);
    
    nav->target_distance_mm = target_distance;
    nav->emergency_stop = false;
    
    /* Reset arc compensation for new run */
    nav->arc_distance_compensation = 0.0f;
    nav->arc_out_distance = 0.0f;
    nav->obstacle_detect_front_dist = 0;
    
    if (mode == CAR_MODE_1_OBSTACLE) {
        US_SetMode(nav->ultrasonic, 1);
        Nav1_EnterState(nav, NAV1_STATE_DRIVE_STRAIGHT);
    } else if (mode == CAR_MODE_2_WALLFOLLOW) {
        US_SetMode(nav->ultrasonic, 2);
        nav->nav2_state = NAV2_STATE_RUNNING;
        nav->target_speed_mm_s = SPEED_CRUISE_MIN;
        Servo_SetAngle(nav->servo, 0);
    }
}

void Navigation_Stop(Navigation_Handle_t *nav)
{
    Motor_Stop(nav->motor);
    Servo_SetAngle(nav->servo, 0);
    
    nav->nav1_state = NAV1_STATE_IDLE;
    nav->nav2_state = NAV2_STATE_IDLE;
    nav->target_speed_mm_s = 0.0f;
}

void Navigation_Update(Navigation_Handle_t *nav)
{
    if (! nav->is_initialized || nav->emergency_stop) return;
    
    /* Get latest sensor data */
    US_GetAllDistances(nav->ultrasonic, 
                       &nav->dist_front, &nav->dist_left,
                       &nav->dist_right, &nav->dist_rear);
    
    /* Update encoder/odometry with current SOFTWARE steering angle */
    int8_t current_sw_angle = Servo_GetAngle(nav->servo);
    Encoder_Update(nav->encoder, current_sw_angle);
    
    /* Run mode-specific logic */
    Car_Mode_t mode = Buttons_GetMode(nav->buttons);
    
    if (mode == CAR_MODE_1_OBSTACLE) {
        Nav1_Update(nav);
    } else if (mode == CAR_MODE_2_WALLFOLLOW) {
        Nav2_Update(nav);
    }
    
    nav->last_update_time = HAL_GetTick();
}

bool Navigation_IsComplete(Navigation_Handle_t *nav)
{
    return nav->nav1_state == NAV1_STATE_COMPLETE;
}

void Navigation_EmergencyStop(Navigation_Handle_t *nav)
{
    nav->emergency_stop = true;
    Motor_Brake(nav->motor);
    Servo_SetAngle(nav->servo, 0);
}

Nav1_State_t Navigation_GetNav1State(Navigation_Handle_t *nav)
{
    return nav->nav1_state;
}

Nav2_State_t Navigation_GetNav2State(Navigation_Handle_t *nav)
{
    return nav->nav2_state;
}