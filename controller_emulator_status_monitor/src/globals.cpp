#include "globals.h"

DRAM_ATTR SemaphoreHandle_t button_pressed_semaphore        = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t dac_access_semaphore            = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t serial2_access_semaphore        = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t print_data_semaphore            = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t process_light_state_semaphore   = xSemaphoreCreateBinary();
DRAM_ATTR uint8_t           number_laps_in_race             = NUMBER_LAPS_IN_RACE_DEFAULT;
DRAM_ATTR U8G2_SSD1306_128X64_NONAME_F_HW_I2C display_128x64(U8G2_R0, U8X8_PIN_NONE);

void init_dac(){
  dac_output_enable(DAC_CHANNEL_1); /* DAC is at pin GPIO25 */
  dac_output_voltage(DAC_CHANNEL_1, 0);
  /* dac is not freed yet, the dac_access_semaphore is controlled by the light_process_task. */
}

void init_display()
{
    display_128x64.begin();
    Wire.setClock(FAST_MODE_PLUS);
    display_128x64.clearBuffer();                   /* clear the ESP32 display write buffer */
    display_128x64.drawXBM(0,0,eva_splashscreen_width,eva_splashscreen_height, eva_splashscreen_bits);
    display_128x64.sendBuffer();                    /* transfer ESP32 display write buffer to the display */
    display_128x64.clearBuffer();
}

IRAM_ATTR void time_difference(){
  static unsigned long timestamp;

  if ( timestamp ){
    Serial.printf("Time difference: %ld us\n", micros() - timestamp);
  }
  timestamp = micros();
}