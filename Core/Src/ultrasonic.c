/**
 * @file    ultrasonic.c
 * @brief   Ultrasonic sensor implementation
 */

#include "ultrasonic.h"
#include "utils.h"
#include "main.h"

/* ============================================================================
 * PRIVATE FUNCTIONS
 * ============================================================================ */

static void US_ConfigureSensor(US_Sensor_t *sensor, 
                               GPIO_TypeDef *trig_port, uint16_t trig_pin,
                               GPIO_TypeDef *echo_port, uint16_t echo_pin,
                               uint16_t max_range, uint16_t timeout)
{
    sensor->trig_port = trig_port;
    sensor->trig_pin = trig_pin;
    sensor->echo_port = echo_port;
    sensor->echo_pin = echo_pin;
    sensor->max_range_mm = max_range;
    sensor->timeout_us = timeout;
    sensor->state = US_STATE_IDLE;
    sensor->distance_mm = 0xFFFF;
    sensor->new_data_available = false;
}

static uint32_t US_GetTimerCount(US_Handle_t *us)
{
    return __HAL_TIM_GET_COUNTER(us->htim);
}

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

void US_Init(US_Handle_t *us, TIM_HandleTypeDef *htim)
{
    us->htim = htim;
    us->current_sensor = US_SENSOR_FRONT;
    us->sequence_running = false;
    us->current_mode = 1;
    
    /* Configure each sensor with default Mode 1 timeouts */
    US_ConfigureSensor(&us->sensors[US_SENSOR_FRONT],
                       FRONT_TRIG_GPIO_Port, FRONT_TRIG_Pin,
                       FRONT_ECHO_GPIO_Port, FRONT_ECHO_Pin,
                       US_FRONT_MAX_RANGE, US_FRONT_TIMEOUT_US);
    
    US_ConfigureSensor(&us->sensors[US_SENSOR_RIGHT],
                       RIGHT_TRIG_GPIO_Port, RIGHT_TRIG_Pin,
                       RIGHT_ECHO_GPIO_Port, RIGHT_ECHO_Pin,
                       US_SIDE_MAX_RANGE, US_SIDE_TIMEOUT_US);
    
    US_ConfigureSensor(&us->sensors[US_SENSOR_LEFT],
                       LEFT_TRIG_GPIO_Port, LEFT_TRIG_Pin,
                       LEFT_ECHO_GPIO_Port, LEFT_ECHO_Pin,
                       US_SIDE_MAX_RANGE, US_SIDE_TIMEOUT_US);
    
    US_ConfigureSensor(&us->sensors[US_SENSOR_REAR],
                       REAR_TRIG_GPIO_Port, REAR_TRIG_Pin,
                       REAR_ECHO_GPIO_Port, REAR_ECHO_Pin,
                       US_REAR_MAX_RANGE, US_REAR_TIMEOUT_US);
    
    /* Ensure all triggers are LOW */
    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        HAL_GPIO_WritePin(us->sensors[i].trig_port, 
                          us->sensors[i].trig_pin, GPIO_PIN_RESET);
    }
    
    /* Start timer */
    HAL_TIM_Base_Start(us->htim);
}

void US_SetMode(US_Handle_t *us, uint8_t mode)
{
	
		us->current_mode = mode;
		/* Wait for any active sequence to complete */
		/*
    while (us->sequence_running) {
        US_Update(us);  // Let it finish
    }
		
    us->current_mode = mode;
    
    if (mode == 2) {
        
        us->sensors[US_SENSOR_LEFT].max_range_mm = US_SIDE_MAX_RANGE_MODE2;
        us->sensors[US_SENSOR_LEFT].timeout_us = US_SIDE_TIMEOUT_MODE2;
        us->sensors[US_SENSOR_RIGHT].max_range_mm = US_SIDE_MAX_RANGE_MODE2;
        us->sensors[US_SENSOR_RIGHT].timeout_us = US_SIDE_TIMEOUT_MODE2;
    } else {
        
        us->sensors[US_SENSOR_LEFT].max_range_mm = US_SIDE_MAX_RANGE_MODE1;
        us->sensors[US_SENSOR_LEFT].timeout_us = US_SIDE_TIMEOUT_MODE1;
        us->sensors[US_SENSOR_RIGHT].max_range_mm = US_SIDE_MAX_RANGE_MODE1;
        us->sensors[US_SENSOR_RIGHT].timeout_us = US_SIDE_TIMEOUT_MODE1;
    }*/
}

void US_TriggerSensor(US_Handle_t *us, US_Sensor_ID_t sensor)
{
    US_Sensor_t *s = &us->sensors[sensor];
    
    /* Reset state */
    s->state = US_STATE_TRIGGER;
    s->new_data_available = false;
    
    /* Reset timer counter */
    __HAL_TIM_SET_COUNTER(us->htim, 0);
    
    /* Send trigger pulse (10-12us HIGH) */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    Utils_DelayUs(US_TRIGGER_PULSE_US);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
    
    /* Move to waiting for echo */
    s->state = US_STATE_WAIT_ECHO_START;
    s->echo_start_time = US_GetTimerCount(us);
}

void US_StartSequence(US_Handle_t *us)
{
    us->current_sensor = US_SENSOR_FRONT;
    us->sequence_running = true;
    us->sequence_start_time = HAL_GetTick();
    
    /* Reset all sensors */
    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        us->sensors[i].state = US_STATE_IDLE;
        us->sensors[i].new_data_available = false;
    }
    
    /* Start with front sensor */
    US_TriggerSensor(us, US_SENSOR_FRONT);
}

void US_Update(US_Handle_t *us)
{
    if (! us->sequence_running) return;
    
    US_Sensor_t *current = &us->sensors[us->current_sensor];
    uint32_t current_time = US_GetTimerCount(us);
    
    /* Check for timeout */
    if (current->state == US_STATE_WAIT_ECHO_START || 
        current->state == US_STATE_WAIT_ECHO_END) {
        
        uint32_t elapsed;
        if (current->state == US_STATE_WAIT_ECHO_START) {
            elapsed = current_time - current->echo_start_time;
        } else {
            elapsed = current_time - current->echo_start_time;
        }
        
        /* Handle timer overflow */
        if (current_time < current->echo_start_time) {
            elapsed = (65535 - current->echo_start_time) + current_time;
        }
        
        if (elapsed > current->timeout_us) {
            /* Timeout - no obstacle within range */
            current->state = US_STATE_TIMEOUT;
            current->distance_mm = current->max_range_mm;  /* Report max range */
            current->new_data_available = true;
        }
    }
    
    /* Move to next sensor if current is done */
    if (current->state == US_STATE_COMPLETE || 
        current->state == US_STATE_TIMEOUT) {
        
        /* Move to next sensor in sequence:  FRONT -> RIGHT -> LEFT -> REAR */
        us->current_sensor++;
        
        if (us->current_sensor >= US_SENSOR_COUNT) {
            /* Sequence complete */
            us->sequence_running = false;
        } else {
            /* Trigger next sensor */
            US_TriggerSensor(us, us->current_sensor);
        }
    }
}

void US_EXTI_Callback(US_Handle_t *us, uint16_t GPIO_Pin)
{
    /* Find which sensor this pin belongs to */
    US_Sensor_t *sensor = NULL;
    
    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        if (us->sensors[i].echo_pin == GPIO_Pin) {
            sensor = &us->sensors[i];
            break;
        }
    }
    
    if (sensor == NULL) return;
    
    uint32_t current_time = US_GetTimerCount(us);
    
    /* Check pin state to determine rising or falling edge */
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(sensor->echo_port, sensor->echo_pin);
    
    if (pin_state == GPIO_PIN_SET) {
        /* Rising edge - echo started */
        if (sensor->state == US_STATE_WAIT_ECHO_START) {
            sensor->echo_start_time = current_time;
            sensor->state = US_STATE_WAIT_ECHO_END;
        }
    } else {
        /* Falling edge - echo ended */
        if (sensor->state == US_STATE_WAIT_ECHO_END) {
            sensor->echo_end_time = current_time;
            
            /* Calculate pulse width */
            uint32_t pulse_width;
            if (sensor->echo_end_time >= sensor->echo_start_time) {
                pulse_width = sensor->echo_end_time - sensor->echo_start_time;
            } else {
                /* Timer overflow */
                pulse_width = (65535 - sensor->echo_start_time) + sensor->echo_end_time;
            }
            
            /* Convert to distance:  distance = time * 0.1715 mm/us */
            sensor->distance_mm = (uint16_t)(pulse_width * US_MM_PER_US);
            
            /* Clamp to max range */
            if (sensor->distance_mm > sensor->max_range_mm) {
                sensor->distance_mm = sensor->max_range_mm;
            }
            
            sensor->state = US_STATE_COMPLETE;
            sensor->new_data_available = true;
        }
    }
}

bool US_SequenceComplete(US_Handle_t *us)
{
    return ! us->sequence_running;
}

uint16_t US_GetDistance(US_Handle_t *us, US_Sensor_ID_t sensor)
{
    if (sensor >= US_SENSOR_COUNT) return 0xFFFF;
    return us->sensors[sensor].distance_mm;
}

void US_GetAllDistances(US_Handle_t *us, uint16_t *front, uint16_t *left, 
                        uint16_t *right, uint16_t *rear)
{
    if (front) *front = us->sensors[US_SENSOR_FRONT].distance_mm;
    if (left) *left = us->sensors[US_SENSOR_LEFT].distance_mm;
    if (right) *right = us->sensors[US_SENSOR_RIGHT].distance_mm;
    if (rear) *rear = us->sensors[US_SENSOR_REAR].distance_mm;
}