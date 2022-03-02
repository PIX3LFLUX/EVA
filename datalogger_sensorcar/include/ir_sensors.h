#include "globals.h"
#include <Ticker.h>

#define IR_SENSOR_LEFT_PIN      16      /* pin for the digital signal */
#define IR_SENSOR_RIGHT_PIN     27

#define DEBOUNCE_TIMEOUT_MS     1

#define TAPE_WIDTH              20e-3   /* tape width in meters */
#if CALIBRATE_IR_SPEED
    const double_t CAL_LEFT[] =   {2.47510363770948, -1.03939098433124}; /* calibration parameter that accounts for detection of tape without being at it's edges (usually being close to the tape already triggers detection, effectively making it bigger), slope and offset */
    const double_t CAL_RIGHT[] =  {2.27465000160550, -0.928610854146550};
#endif
#define MAXIMUM_POSSIBLE_SPEED  20      /* in m/s, needed for calculation of SHORTEST_PASSING_TIME*/
#define SHORTEST_PASSING_TIME   TAPE_WIDTH/MAXIMUM_POSSIBLE_SPEED*1e6 /* in us. If a time shorter than this is measured, it is likely a measurement error. */

extern unsigned long ir_left_passing_time;
extern unsigned long ir_right_passing_time;
extern unsigned long ir_left_history_time;
extern unsigned long ir_right_history_time;
extern signed long ir_left_right_time_difference;

void init_ir_sensors();
inline bool inverted_fast_digital_read(uint32_t mask, volatile uint32_t* port);
IRAM_ATTR void ir_sensor_left_trigger();    /* starts a debouncing timer */
IRAM_ATTR void ir_sensor_right_trigger();   /* starts a debouncing timer */
IRAM_ATTR void ir_sensor_left_isr();
IRAM_ATTR void ir_sensor_right_isr();