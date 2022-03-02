#pragma once

#include <Arduino.h>
#include <driver/dac.h>
#include <Wire.h>                   /* I2C for display*/
#include <U8g2lib.h>                /* Display driver for the OLED screen */
#include "display_image_data.h"     /* Contains graphics to draw on the display.*/

#define DEBUG                       0       /* debug output in via serial port. Disable for final code. 0:disabled 1:enabled */
#define SERIAL_USERDATA_PRINT       1       /* only outputs lap times via the serial terminal */
#define DISPLAY_OUTPUT_ENABLE       1
#define FAST_MODE_PLUS              3400000 /* I2C link speed */
#define NUMBER_LAPS_IN_RACE_DEFAULT 10      /* the number of laps that have to be driven for a race to finish and declare a winner */
#define WIRELESS_TRANSMISSION_TRIES 1       /* because it's not certain that the other uC has received the message, we send a couple times. */

/* states for race_status*/ 
#define NO_RACE_GOING                   0
#define RACE_GOING                      1
#define CAR_NO_0_PASSED_FINISH_LINE     2
#define RECEIVED_VALUE_WIRELESSLY       3

/* FreeRTOS Settings*/
#define INCLUDE_vTaskSuspend    1           /* if set to '1' then specifying the block time as portMAX_DELAY will cause the task to block indefinitely (without a timeout) */

/* Task priorities */
#define IDLE_PRIO               0
#define SERIAL2_COMMS_PRIO      IDLE_PRIO+1 /* needs to be lower priority, because it is a background task that runs when other tasks don't */
#define BUTTON_HANDLE_PRIO      IDLE_PRIO+2
#define LIGHT_STATE_PRIO        IDLE_PRIO+3
#define PRINT_DATA_PRIO         IDLE_PRIO+2

/* Task Cores
main loop and setup run on core 1, wireless receive runs on core 0.
Since low latency is important for wireless receive, as few tasks as possible run on core 0.
Tasks that share data should not be on different cores, since global variables are used instead of the 'proper' way with queues, because it's faster.
*/
#define SERIAL2_COMMS_CORE      1
#define BUTTON_HANDLE_CORE      1   /* Draws on the display. Every Task that draws needs to run on the same core. */
#define LIGHT_STATE_CORE        1
#define PRINT_DATA_CORE         1   /* Draws on the display */

#define DELAY_N_MS(n) (vTaskDelay(n/portTICK_PERIOD_MS))            /* macro for vTaskDelay + math */

extern DRAM_ATTR SemaphoreHandle_t button_pressed_semaphore;        /* is set when any button is pressed and the debounce timer has overflown */
extern DRAM_ATTR SemaphoreHandle_t dac_access_semaphore;            /* for the shared resource digital to analog converter (DAC) */
extern DRAM_ATTR SemaphoreHandle_t serial2_access_semaphore;        /* for the shared resource of the second serial interface, which is used to communicate to the Control Unit */
extern DRAM_ATTR SemaphoreHandle_t print_data_semaphore;            /* used for telling the print_car_data_task that new data is available for print */
extern DRAM_ATTR SemaphoreHandle_t process_light_state_semaphore;   /* used for telling the process_light_state_task that the light state has changed and has to be processed */
extern DRAM_ATTR uint8_t           number_laps_in_race;
extern DRAM_ATTR U8G2_SSD1306_128X64_NONAME_F_HW_I2C display_128x64; /* display class used for user interface via I2C display */

void init_dac();
void init_display();
IRAM_ATTR void time_difference(); /* function that's useful for debugging and benchmarking time performance */
