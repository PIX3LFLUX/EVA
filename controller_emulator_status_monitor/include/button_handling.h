#include "globals.h"
#include <Arduino.h>
#include "Ticker.h" /* timer library */

/* those pins are available as input with internal pullup. Additionally, they are not part of the SPI pins, which are thus free for use */
#define START_BUTTON_PIN                4
#define INCREASE_RACE_LAPS_BUTTON_PIN   13
#define DECREASE_RACE_LAPS_BUTTON_PIN   27

#define DEBOUNCE_TIMEOUT_MS             10 /* time to wait after an interrupt was triggered from a button to read it */

extern DRAM_ATTR bool start_button_state;
extern DRAM_ATTR bool increase_laps_button_state;
extern DRAM_ATTR bool decrease_laps_button_state;

void init_buttons();
inline bool fast_digital_read(uint32_t mask, volatile uint32_t* port);
IRAM_ATTR void button_isr();
IRAM_ATTR void timer_isr();