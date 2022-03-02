/* ###################################################
This is the receiver and controller emulator code.
It also communicates with the Carrera CU to obtain lap times and start races.
################################################### */

#include "globals.h"          /* constants, functions and variables used by multiple files */
#include "button_handling.h"
#include "serial_handling.h"  /* For communication with the control unit and wireless comms as well as processing the data. The bulk of the code lives here. */

/* ###################################################
Functions
################################################### */
inline void display_race_lap_number()
{
  #if DEBUG
    Serial.printf("Lap count for race to finish: %d\n", number_laps_in_race);
  #endif
  #if SERIAL_USERDATA_PRINT
    Serial.print(TEXT_PUSH_DOWN_STRING);
    Serial.printf("Ein Rennen um %d Runden.\n", number_laps_in_race);
  #endif
  #if DISPLAY_OUTPUT_ENABLE
    display_128x64.setFont(u8g2_font_ncenB12_tr);
    display_128x64.drawStr(0, 20,"Ein Rennen:");
    display_128x64.setCursor(0, 40);
    display_128x64.print(u8x8_u8toa(number_laps_in_race, 3)); /* ensures that the number of laps is always a consistent length of characters when being printed. Appends zeros */
    display_128x64.print(" Runde(n).");
    display_128x64.sendBuffer();
    display_128x64.clearBuffer();  
  #endif  
}

inline void display_eva_logo()
{
    display_128x64.drawXBM(0,0,eva_splashscreen_width,eva_splashscreen_height, eva_splashscreen_bits);
    display_128x64.sendBuffer();
    display_128x64.clearBuffer();
}

/*
Deferred interrupt handling for buttons, because they are not THAT time-critical and it keeps the time spent in interrupt service routine to a minimum.
After the buttons have been processed, the task blocks until a button is pressed again.
The buttons on the control unit work without delay, since there is no serial communication required to operate them.
Note however that the software is not designed to operate with those.
It will work for the most part, but there might be some unintuitive behavior.
*/
IRAM_ATTR void button_handle_task(void*)
{
  init_buttons();
  for(;;) 
  {
    if (xSemaphoreTake(button_pressed_semaphore, portMAX_DELAY) == pdTRUE)
    {
      /* nested ifs to ensure only one button is processed at a time. Priority: start button > increase laps in race > decrease laps in race */
      if (start_button_state)
      {
        press_start_button();
        #if DEBUG
          Serial.println("Start button pressed.");
        #endif
        #if DISPLAY_OUTPUT_ENABLE
          display_eva_logo();
        #endif
      }
      else
      {
        if (increase_laps_button_state)
        {
          /* protection from overflow */
          if (number_laps_in_race < 255)  { number_laps_in_race += 1; }
          else                            { number_laps_in_race  = 255; }
          #if DEBUG || SERIAL_USERDATA_PRINT || DISPLAY_OUTPUT_ENABLE
            display_race_lap_number();
          #endif
        }
        else
        {
          if (decrease_laps_button_state)
          {
            /* protection from underflow and omission of 0-lap races, since they are nonsensical and functionally equivalent to 1-lap races */
            if (number_laps_in_race > 1)  { number_laps_in_race -= 1; }
            else                          { number_laps_in_race  = 1; }
            #if DEBUG || SERIAL_USERDATA_PRINT || DISPLAY_OUTPUT_ENABLE
              display_race_lap_number();
            #endif  
          }
        }
      }
    }
  }
}

IRAM_ATTR void serial_communication_with_control_unit_task(void*)
{
  for(;;)
  {
    get_data_from_control_unit();
    taskYIELD(); /* If a task with higher priority is available (=not blocking), it well get executed. Else, this task gets executed again */
  }
}

IRAM_ATTR void print_car_data_task(void*)
{
  #if DISPLAY_OUTPUT_ENABLE
    init_display();
  #endif  
  for(;;)
  {
    /* 
    If new data has arrived, present it over serial or on the display, depending on the configuration in globals.h 
    After it has been presented, block the task until there is new data available
    */
    if ( xSemaphoreTake(print_data_semaphore,portMAX_DELAY) == pdTRUE )
    {
      print_car_data();
    }
  }
}

/* 
This task checks the light state of the control unit (five leds on top) to synchronize its internal state to it.
It is executed only if the light state has changed.
When the light state has been examined and acted upon, the task blocks again.
*/
IRAM_ATTR void process_light_state_task(void*)
{
  for(;;)
  {
    if ( xSemaphoreTake(process_light_state_semaphore,portMAX_DELAY) == pdTRUE )
    {
      process_light_state();
    }
  }
}

/* ###################################################
Initilization
################################################### */
void setup()
{
  /* Serial interfaces */
  Serial.begin(115200); /* Debug interface */
  init_serial2();       /* Control Unit serial interface */

  /* DAC */
  init_dac();
  
  /* Wi-Fi */
  init_wifi();

  /* Tasks
  main loop runs on core 1.
  Some tasks run their own inits to make sure the globals are initialized on the same core where they are used.
  Sometimes there's problems if that is not the case.
  That's a bug with the multicore port of FreeRTOS on the ESP32.
  */
  xTaskCreatePinnedToCore(button_handle_task, 
                          "button_handle_task", 
                          10000,
                          NULL,
                          BUTTON_HANDLE_PRIO, 
                          NULL,
                          BUTTON_HANDLE_CORE);
  xTaskCreatePinnedToCore(serial_communication_with_control_unit_task,
                          "serial_communication_with_control_unit_task",
                          10000,
                          NULL,
                          SERIAL2_COMMS_PRIO,
                          NULL,
                          SERIAL2_COMMS_CORE);
  xTaskCreatePinnedToCore(print_car_data_task, 
                          "print_car_data_task", 
                          10000,
                          NULL,
                          PRINT_DATA_PRIO, 
                          NULL,
                          PRINT_DATA_CORE);
  xTaskCreatePinnedToCore(process_light_state_task, 
                          "process_light_state_task", 
                          10000,
                          NULL,
                          LIGHT_STATE_PRIO, 
                          NULL,
                          LIGHT_STATE_CORE);                       
}

/* ###################################################
Main Loop
################################################### */
void loop()
{
  DELAY_N_MS(portMAX_DELAY); /* block Task indefinitely */
}