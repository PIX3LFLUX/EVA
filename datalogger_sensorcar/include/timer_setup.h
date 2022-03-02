#include <Arduino.h>
#include "globals.h"

#if DEBUG
    #define SAMPLING_INTERVAL 1000000 /* sampling period T (1/f) in microseconds (us). Resolution is only one ms, the ESP will change between intervals to get the required interval on average. For consistency, only use ms based periods. */
#else
    #define SAMPLING_INTERVAL 10000 /* 100 Hz */
#endif
#define CONTROLLER_INTERVAL         75000                  /* 75ms, equal to Carrera CU sampling clock */
#define MEASUREMENT_TIMER_INTERVAL  CONTROLLER_INTERVAL*1  /* multiple of Carrera CU sampling clock. Even multiples are recommended since logging is done at the sampling interval */

inline void init_timer_generic();
void init_timers();
inline void halt_timer_generic();
IRAM_ATTR void halt_timers();
IRAM_ATTR void restart_timers();

IRAM_ATTR void on_sample_timer();
IRAM_ATTR void on_controller_timer();
IRAM_ATTR void on_measurement_timer();