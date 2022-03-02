#include <Arduino.h>
#include <HardwareSerial.h>
#include "Ticker.h"
#include "globals.h"
#include "wireless_transmission.h"  /* libraries and functions for esp_now transmission. Also writes received speed values to DAC */

#define TIMEOUT_SECONDS                 300  /* after this much time has passed, the controller emulator will generate some activity to keep the CU awake. CU shuts down after 20min of inactivity. */
/* Carrera D132 protocol from http://slotbaer.de/carrera-digital-124-132/10-cu-rundenzaehler-protokoll.html and own trial-and-error */
#define PRESS_START                     "\"T2$"
#define PRESS_PACE_CAR                  "\"T1$"
#define RESET_INTERNAL_TIMER            "\"=101$"
#define REQUEST_LAST_PASSING_TIMESTAMP  "\"?"
#define ASK_VERSION_NUMBER              "\"0"

#define RECEIVE_BUFFER_LENGTH           20
#define TEXT_PUSH_DOWN_STRING           "\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n" /* used to "clear" modern autoscrolling terminals since they are not clearable with escape sequences */

extern DRAM_ATTR uint8_t light_state;
extern DRAM_ATTR uint8_t race_status;

            void init_serial2();
inline      void print_eva_logo();            
IRAM_ATTR   void get_data_from_control_unit();
inline      void parse_data_received();
IRAM_ATTR   void press_start_button();
IRAM_ATTR   void keep_cu_awake();
IRAM_ATTR   void print_car_data();
inline      void display_victory_screen();
inline      void print_victory_screen();
IRAM_ATTR   void process_light_state();
