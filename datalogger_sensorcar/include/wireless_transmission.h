#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "globals.h"

/* states for race_status*/ 
#define NO_RACE_GOING                   0
#define RACE_GOING                      1
#define CAR_NO_0_PASSED_FINISH_LINE     2
#define RECEIVED_VALUE_WIRELESSLY       3

const uint8_t newMACAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x66};    /* MAC this uC */
const uint8_t broadcastAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x65}; /* MAC receiver */

void init_wifi();
IRAM_ATTR void send_data_wirelessly(uint8_t data_to_transmit);
IRAM_ATTR void on_data_receive(const uint8_t * mac, const uint8_t *incoming_data, int len);