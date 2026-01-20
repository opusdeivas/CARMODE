/**
 * @file    ultrasonic.c
 * @brief   Ultrasonic sensor implementation - SIMPLIFIED
 * 
 * Operation:
 *   1. Trigger sensor (12µs pulse)
 *   2. Wait for rising edge (echo start)
 *   3. Wait for falling edge (echo end)
 *   4. Calculate distance
 *   5. Move to next sensor
 * 
 * Only ONE sensor active at a time.  Hardware timeout is ~30ms,
 * software timeout is 40ms to ensure we don't interfere. 
 */

#include "ultrasonic.h"
#include "utils.h"
#include "main.h"

/* Software timeout - must be > hardware timeout (30ms) */
#define US_TIMEOUT_US           40000    /* 40ms software timeout */

/* ============================================================================
 * PRIVATE FUNCTIONS
 * ============================================================================ */

static void US_ConfigureSensor(US_Sensor_t *sensor, 
                               GPIO_TypeDef *trig_port, uint16_t trig_pin,
                               GPIO_TypeDef *echo_port, uint16_t echo_pin)
{
    sensor->trig_port = trig_port;
    sensor->trig_pin = trig_pin;
    sensor->echo_port = echo_port;
    sensor->echo_pin = echo_pin;
    sensor->state = US_STATE_IDLE;
    sensor->distance_mm = 0xFFFF;
    sensor->new_data_available = false;
    sensor->echo_start_time = 0;
    sensor->echo_end_time = 0;
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
    us->trigger_time = 0;
    
    US_ConfigureSensor(&us->sensors[US_SENSOR_FRONT],
                       FRONT_TRIG_GPIO_Port, FRONT_TRIG_Pin,
                       FRONT_ECHO_GPIO_Port, FRONT_ECHO_Pin);
    
    US_ConfigureSensor(&us->sensors[US_SENSOR_RIGHT],
                       RIGHT_TRIG_GPIO_Port, RIGHT_TRIG_Pin,
                       RIGHT_ECHO_GPIO_Port, RIGHT_ECHO_Pin);
    
    US_ConfigureSensor(&us->sensors[US_SENSOR_LEFT],
                       LEFT_TRIG_GPIO_Port, LEFT_TRIG_Pin,
                       LEFT_ECHO_GPIO_Port, LEFT_ECHO_Pin);
    
    US_ConfigureSensor(&us->sensors[US_SENSOR_REAR],
                       REAR_TRIG_GPIO_Port, REAR_TRIG_Pin,
                       REAR_ECHO_GPIO_Port, REAR_ECHO_Pin);
    
    /* Ensure all triggers are LOW */
    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        HAL_GPIO_WritePin(us->sensors[i]. trig_port, 
                          us->sensors[i].trig_pin, GPIO_PIN_RESET);
    }
    
    /* Start timer once - never reset it */
    __HAL_TIM_SET_COUNTER(us->htim, 0);
    HAL_TIM_Base_Start(us->htim);
}

void US_SetMode(US_Handle_t *us, uint8_t mode)
{
    us->current_mode = mode;
}

void US_TriggerSensor(US_Handle_t *us, US_Sensor_ID_t sensor_id)
{
    US_Sensor_t *s = &us->sensors[sensor_id];
    
    /* Reset state for this sensor */
    s->state = US_STATE_TRIGGER;
    s->new_data_available = false;
    s->echo_start_time = 0;
    s->echo_end_time = 0;
    
    /* Record trigger time for timeout calculation */
    us->trigger_time = __HAL_TIM_GET_COUNTER(us->htim);
    
    /* Send trigger pulse (12µs HIGH) */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    Utils_DelayUs(US_TRIGGER_PULSE_US);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
    
    /* Now waiting for echo rising edge */
    s->state = US_STATE_WAIT_ECHO_START;
}

void US_StartSequence(US_Handle_t *us)
{
    /* Reset sequence */
    us->current_sensor = US_SENSOR_FRONT;
    us->sequence_running = true;
    us->sequence_start_time = HAL_GetTick();
    
    /* Reset all sensor states */
    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        us->sensors[i]. state = US_STATE_IDLE;
        us->sensors[i].new_data_available = false;
    }
    
    /* Start with front sensor */
    US_TriggerSensor(us, US_SENSOR_FRONT);
}

void US_Update(US_Handle_t *us)
{
    if (! us->sequence_running) return;
    
    US_Sensor_t *current = &us->sensors[us->current_sensor];
    
    /* Check for timeout */
    if (current->state == US_STATE_WAIT_ECHO_START || 
        current->state == US_STATE_WAIT_ECHO_END) {
        
        uint32_t now = __HAL_TIM_GET_COUNTER(us->htim);
        uint32_t elapsed;
        
        if (now >= us->trigger_time) {
            elapsed = now - us->trigger_time;
        } else {
            elapsed = (65535 - us->trigger_time) + now + 1;
        }
        
        if (elapsed > US_TIMEOUT_US) {
            current->state = US_STATE_TIMEOUT;
            current->distance_mm = 0xFFFF;
            current->new_data_available = true;
        }
    }
    
    /* Move to next sensor if current is done */
    if (current->state == US_STATE_COMPLETE || 
        current->state == US_STATE_TIMEOUT) {
        
        us->current_sensor++;
        
        if (us->current_sensor >= US_SENSOR_COUNT) {
            us->sequence_running = false;
        } else {
            /* Reset next sensor's state before triggering */
            us->sensors[us->current_sensor].state = US_STATE_IDLE;
            US_TriggerSensor(us, us->current_sensor);
        }
    }
}

void US_EXTI_Callback(US_Handle_t *us, uint16_t GPIO_Pin)
{
    /* Find which sensor this pin ACTUALLY belongs to */
    int sensor_index = -1;
    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        if (us->sensors[i].echo_pin == GPIO_Pin) {
            sensor_index = i;
            break;
        }
    }
    
    /* DEBUG: Store info about every callback */
    static volatile uint16_t last_pin = 0;
    static volatile int last_sensor = -1;
    static volatile int last_current = -1;
    static volatile uint8_t last_pin_state = 0;
    
    last_pin = GPIO_Pin;
    last_sensor = sensor_index;
    last_current = us->current_sensor;
    
    if (sensor_index >= 0) {
        last_pin_state = HAL_GPIO_ReadPin(us->sensors[sensor_index].echo_port, 
                                          us->sensors[sensor_index]. echo_pin);
    }
    
    /* Only process if sequence is running */
    if (! us->sequence_running) return;
    
    /* REMOVED: The "only current sensor" check - let's see what happens */
    // if (sensor_index != us->current_sensor) return;
    
    if (sensor_index < 0) return;  // Unknown pin
    
    US_Sensor_t *sensor = &us->sensors[sensor_index];
    
    /* Get timestamp FIRST */
    uint32_t current_time = __HAL_TIM_GET_COUNTER(us->htim);
    
    /* Read current pin state */
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(sensor->echo_port, sensor->echo_pin);
    
    if (pin_state == GPIO_PIN_SET) {
        /* Rising edge */
        if (sensor->state == US_STATE_WAIT_ECHO_START) {
            sensor->echo_start_time = current_time;
            sensor->state = US_STATE_WAIT_ECHO_END;
        }
    } else {
        /* Falling edge */
        if (sensor->state == US_STATE_WAIT_ECHO_END) {
            sensor->echo_end_time = current_time;
            
            uint32_t pulse_width;
            if (sensor->echo_end_time >= sensor->echo_start_time) {
                pulse_width = sensor->echo_end_time - sensor->echo_start_time;
            } else {
                pulse_width = (65535 - sensor->echo_start_time) + sensor->echo_end_time + 1;
            }
            
            sensor->distance_mm = (uint16_t)(pulse_width * US_MM_PER_US);
            sensor->state = US_STATE_COMPLETE;
            sensor->new_data_available = true;
            
            /* If this sensor completed AND it's the current one, great! 
               If not, we'll let US_Update handle the timeout for current sensor */
        }
    }
}

void US_EXTI_CallbackWithState(US_Handle_t *us, uint16_t GPIO_Pin, uint8_t pin_state)
{
    /* Only process if sequence is running */
    if (!us->sequence_running) return;
    
    /* Only process interrupts for the CURRENT sensor */
    US_Sensor_t *sensor = &us->sensors[us->current_sensor];
    
    /* Verify this interrupt is for the current sensor's echo pin */
    if (sensor->echo_pin != GPIO_Pin) return;
    
    /* Get timestamp FIRST */
    uint32_t current_time = __HAL_TIM_GET_COUNTER(us->htim);
    
    if (pin_state == GPIO_PIN_SET) {
        /* Rising edge - echo pulse started */
        if (sensor->state == US_STATE_WAIT_ECHO_START) {
            sensor->echo_start_time = current_time;
            sensor->state = US_STATE_WAIT_ECHO_END;
        }
    } else {
        /* Falling edge - echo pulse ended */
        if (sensor->state == US_STATE_WAIT_ECHO_END) {
            sensor->echo_end_time = current_time;
            
            /* Calculate pulse width with overflow handling */
            uint32_t pulse_width;
            if (sensor->echo_end_time >= sensor->echo_start_time) {
                pulse_width = sensor->echo_end_time - sensor->echo_start_time;
            } else {
                pulse_width = (65535 - sensor->echo_start_time) + sensor->echo_end_time + 1;
            }
            
            /* Convert to distance */
            sensor->distance_mm = (uint16_t)(pulse_width * US_MM_PER_US);
            
            sensor->state = US_STATE_COMPLETE;
            sensor->new_data_available = true;
        }
    }
}

bool US_SequenceComplete(US_Handle_t *us)
{
    return !us->sequence_running;
}

uint16_t US_GetDistance(US_Handle_t *us, US_Sensor_ID_t sensor)
{
    if (sensor >= US_SENSOR_COUNT) return 0xFFFF;
    return us->sensors[sensor]. distance_mm;
}

void US_GetAllDistances(US_Handle_t *us, uint16_t *front, uint16_t *left, 
                        uint16_t *right, uint16_t *rear)
{
    if (front) *front = us->sensors[US_SENSOR_FRONT].distance_mm;
    if (left) *left = us->sensors[US_SENSOR_LEFT].distance_mm;
    if (right) *right = us->sensors[US_SENSOR_RIGHT].distance_mm;
    if (rear) *rear = us->sensors[US_SENSOR_REAR].distance_mm;
}