#include "ir_sensors.h"

/* get ports and masks for faster digital reads so that it will be OK to read in the isr */
const uint32_t IR_SENSOR_LEFT_PIN_MASK      = digitalPinToBitMask(IR_SENSOR_LEFT_PIN);
const uint32_t IR_SENSOR_RIGHT_PIN_MASK     = digitalPinToBitMask(IR_SENSOR_RIGHT_PIN);

volatile uint32_t* IR_SENSOR_LEFT_PIN_PORT  = portInputRegister(digitalPinToPort(IR_SENSOR_LEFT_PIN));
volatile uint32_t* IR_SENSOR_RIGHT_PIN_PORT = portInputRegister(digitalPinToPort(IR_SENSOR_RIGHT_PIN));

/* time variables */
DRAM_ATTR unsigned long ir_left_trigger_timestamp   = 0; 
DRAM_ATTR unsigned long ir_right_trigger_timestamp  = 0;
DRAM_ATTR unsigned long ir_left_trigger_timestamp_previous   = 0; 
DRAM_ATTR unsigned long ir_right_trigger_timestamp_previous  = 0; 

DRAM_ATTR unsigned long ir_left_passing_time        = 0; /* time the pin was low in us */
DRAM_ATTR unsigned long ir_right_passing_time       = 0; /* time the pin was low in us */
DRAM_ATTR unsigned long ir_left_history_time        = 0; /* time between two tape passings */
DRAM_ATTR unsigned long ir_right_history_time       = 0; /* time between two tape passings */

DRAM_ATTR signed long ir_left_right_time_difference = 0; /* time (in us) it takes the right ir sensor to trigger after the left one triggered. can be used to detect curves. */

/* logic */
DRAM_ATTR bool left_done = false;   /* data is only ready when both left and right sensors are done reading */
DRAM_ATTR bool right_done = false;  /* data is only ready when both left and right sensors are done reading */

DRAM_ATTR Ticker left_debounce_timer;
DRAM_ATTR Ticker right_debounce_timer;

void init_ir_sensors()
{
    pinMode(IR_SENSOR_LEFT_PIN, INPUT_PULLUP);
    pinMode(IR_SENSOR_RIGHT_PIN, INPUT_PULLUP);
    attachInterrupt(IR_SENSOR_LEFT_PIN, ir_sensor_left_trigger,     CHANGE);
    attachInterrupt(IR_SENSOR_RIGHT_PIN, ir_sensor_right_trigger,   CHANGE);
}

/* reads true for a pin == '0' and false for a pin == '1'. Inverts the logic so it's normal again, since the IR sensors give a LOW signal when they detect the tape.*/
inline bool inverted_fast_digital_read(uint32_t mask, volatile uint32_t* port)
{
    return (*port & mask) == 0;
}

IRAM_ATTR void ir_sensor_left_trigger()
{
    left_debounce_timer.once_ms(DEBOUNCE_TIMEOUT_MS, ir_sensor_left_isr);
}

IRAM_ATTR void ir_sensor_right_trigger()
{
    right_debounce_timer.once_ms(DEBOUNCE_TIMEOUT_MS, ir_sensor_right_isr);
}

IRAM_ATTR void ir_sensor_left_isr()
{
    if ( inverted_fast_digital_read(IR_SENSOR_LEFT_PIN_MASK, IR_SENSOR_LEFT_PIN_PORT) )
    {
        /* driving onto reflective surface */
        ir_left_trigger_timestamp_previous = ir_left_trigger_timestamp;
        ir_left_trigger_timestamp = micros();
        ir_left_history_time = ir_left_trigger_timestamp - ir_left_trigger_timestamp_previous;
        left_done = false;
    }
    else
    {
        /* driving off of reflective surface */
        ir_left_passing_time = micros() - ir_left_trigger_timestamp;
        
        left_done = ir_left_passing_time > SHORTEST_PASSING_TIME; /* check value for validity. If it is longer than the SHORTEST_PASSING_TIME, we are done reading. */
        if (left_done && right_done)
        {
            ir_left_right_time_difference = ir_right_trigger_timestamp - ir_left_trigger_timestamp;
            #if DEBUG
                Serial.printf("Infrared sensor left was high for %ld us.\n", ir_left_passing_time);
                Serial.printf("Infrared sensor right was high for %ld us.\n", ir_right_passing_time);
                Serial.printf("Infrared sensor right triggered %ld us after the left one.\n", ir_left_right_time_difference);            
            #endif            
            xSemaphoreGiveFromISR(ir_data_semaphore,NULL);
            left_done = false;
            right_done = false;
        }
    }
}

IRAM_ATTR void ir_sensor_right_isr()
{
    if ( inverted_fast_digital_read(IR_SENSOR_RIGHT_PIN_MASK, IR_SENSOR_RIGHT_PIN_PORT) )
    {
        /* driving onto reflective surface */
        ir_right_trigger_timestamp_previous = ir_right_trigger_timestamp;
        ir_right_trigger_timestamp = micros();
        ir_right_history_time = ir_right_trigger_timestamp - ir_right_trigger_timestamp_previous;
        right_done = false;
    }
    else
    {
        /* driving off of reflective surface */
        ir_right_passing_time = micros() - ir_right_trigger_timestamp;
        
        right_done = ir_right_passing_time > SHORTEST_PASSING_TIME;
        if (left_done && right_done)
        {
            ir_left_right_time_difference = ir_right_trigger_timestamp - ir_left_trigger_timestamp;
            #if DEBUG
                Serial.printf("Infrared sensor left was high for %ld us.\n", ir_left_passing_time);
                Serial.printf("Infrared sensor right was high for %ld us.\n", ir_right_passing_time);
                Serial.printf("Infrared sensor right triggered %ld us after the left one.\n", ir_left_right_time_difference);
            #endif
            xSemaphoreGiveFromISR(ir_data_semaphore,NULL);
            left_done = false;
            right_done = false;
        }
    }
}