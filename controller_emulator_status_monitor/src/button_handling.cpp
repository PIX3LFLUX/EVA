#include "button_handling.h"

DRAM_ATTR bool start_button_state           = false;
DRAM_ATTR bool increase_laps_button_state   = false;
DRAM_ATTR bool decrease_laps_button_state   = false;

/* get ports and masks for faster digital reads so that it will be OK to read the buttons in the timer isr */
const uint32_t START_BUTTON_PIN_MASK                 = digitalPinToBitMask(START_BUTTON_PIN);
const uint32_t INCREASE_RACE_LAPS_BUTTON_PIN_MASK    = digitalPinToBitMask(INCREASE_RACE_LAPS_BUTTON_PIN);
const uint32_t DECREASE_RACE_LAPS_BUTTON_PIN_MASK    = digitalPinToBitMask(DECREASE_RACE_LAPS_BUTTON_PIN);

volatile uint32_t* START_BUTTON_PIN_PORT                = portInputRegister(digitalPinToPort(START_BUTTON_PIN));
volatile uint32_t* INCREASE_RACE_LAPS_BUTTON_PIN_PORT   = portInputRegister(digitalPinToPort(INCREASE_RACE_LAPS_BUTTON_PIN));
volatile uint32_t* DECREASE_RACE_LAPS_BUTTON_PIN_PORT   = portInputRegister(digitalPinToPort(DECREASE_RACE_LAPS_BUTTON_PIN));

DRAM_ATTR hw_timer_t* timer = NULL;
DRAM_ATTR Ticker debounce_timer;
DRAM_ATTR portMUX_TYPE timer_mutex = portMUX_INITIALIZER_UNLOCKED;

void init_buttons()
{
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(INCREASE_RACE_LAPS_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DECREASE_RACE_LAPS_BUTTON_PIN, INPUT_PULLUP);

    attachInterrupt(START_BUTTON_PIN, button_isr, RISING);
    attachInterrupt(INCREASE_RACE_LAPS_BUTTON_PIN, button_isr, RISING);
    attachInterrupt(DECREASE_RACE_LAPS_BUTTON_PIN, button_isr, RISING);
}

inline bool fast_digital_read(uint32_t mask, volatile uint32_t* port)
{
    return (*port & mask) != 0;
}

IRAM_ATTR void button_isr()
{
    /* when any button is pressed or bouncing, reload a timer to wait for the switches to stop bouncing */
    debounce_timer.once_ms(DEBOUNCE_TIMEOUT_MS, timer_isr);
}

IRAM_ATTR void timer_isr()
{
    /* no edges for DEBOUNCE_TIMEOUT_MS milliseconds, so the bouncing should have stopped */
    /* read button states, but only if the most recent press has already been processed */
    if (xSemaphoreTakeFromISR(button_pressed_semaphore,0) != pdTRUE)
    {
        portENTER_CRITICAL_ISR(&timer_mutex);
        start_button_state          = fast_digital_read(START_BUTTON_PIN_MASK, START_BUTTON_PIN_PORT);
        increase_laps_button_state  = fast_digital_read(INCREASE_RACE_LAPS_BUTTON_PIN_MASK, INCREASE_RACE_LAPS_BUTTON_PIN_PORT);
        decrease_laps_button_state  = fast_digital_read(DECREASE_RACE_LAPS_BUTTON_PIN_MASK, DECREASE_RACE_LAPS_BUTTON_PIN_PORT);
        portEXIT_CRITICAL_ISR(&timer_mutex);
    }
    xSemaphoreGiveFromISR(button_pressed_semaphore, NULL);
}