/* ###################################################
This code is intended to log sensor data to an SD card.
Additionally, it controls the car the sensors are on
by sending speed data over ESP-NOW to a second ESP32
running controller emulation.
################################################### */

#include "globals.h"                /* global constants */
#include <Arduino.h>
#include <cstdio>

#include "wireless_transmission.h"  /* libraries and functions for esp_now transmission */
#include "data_logging.h"           /* for logging sensor data to SD card */
#include "timer_setup.h"            /* for managing harware timer */
#include "imu_lsm6ds3.h"            /* for managing the two lsm6ds3 IMUs */
#include "ir_sensors.h"             /* for managing the two digital infrared reflectometers */
#include "track_data.h"             /* includes track parts lengths, etc. */

/* ###################################################
Variables
################################################### */

/* ####################################################
Functions
#################################################### */

/* sends speed value over ESP now if it changed */
inline void update_speed()
{
  if (speed_digital != speed_digital_previous)
  {
    send_data_wirelessly(speed_digital);
    speed_digital_previous = speed_digital;
  }
}

/* 
This task runs through a known number of speed control settings. Combined with data logging, the system response and other data can be aquired. In short, this tells the car what to do during data logging.
*/
IRAM_ATTR void measurement_task(void*)
{
  #if MEASURE_SYSTEM == MEASURE_MODE_SWEEP
    uint16_t speed_digital_test = VDIGI_INITIAL_VALUE; /* needs to hold more values to avoid overflow conditions. Also not reset at transition to idle state, contrary to speed_digital */
    bool          start_timestamp_set       = false;
    unsigned long timestamp_measure_start   = 0;
    unsigned long timestamp_measure_current = 0;

    for(;;)
    {
      if (xSemaphoreTake(measurement_timer_semaphore, portMAX_DELAY) == pdTRUE)
      {
        switch (sensorcar_state)
        {
          case SENSORCAR_IDLE_STATE:
            start_timestamp_set = false;
            reset_all_state_data(); /* also resets speed_digital */
            update_speed();         /* turn off engine after test */
          break;
          case SENSORCAR_MEASUREMENT_STATE:
            if (!start_timestamp_set) { timestamp_measure_start = millis(); start_timestamp_set = true; }
            timestamp_measure_current = millis();

            if (timestamp_measure_current >= (timestamp_measure_start + TIME_HELD_MS + TIME_MEASURE_MS) ) /* Measuring time has passed. Wrap up the current measurement. */
            {
              close_log_file();   /* save the progress */
              if ( (speed_digital_test + VDIGI_INCREMENT_VALUE) <= VDIGI_MAX_VALUE ) /* avoids overflow and checks for being smaller than VDIGI_MAX_VALUE*/
              {
                speed_digital_test += VDIGI_INCREMENT_VALUE; /* set up the speed for the next test */
              }
              else
              {
                #if DEBUG
                  Serial.println("Measurement complete. Goodbye!");
                #endif
                speed_digital = 0;    /* set speed to 0 here since the task will be deleted afterwards and will not have a chance then. */
                update_speed();
                sensorcar_state = SENSORCAR_IDLE_STATE;
                vTaskDelete(NULL);  /* task deletes itself */
              }              
              sensorcar_state = SENSORCAR_IDLE_STATE; /* measurement is complete, return to idle state */
            }
            else if (timestamp_measure_current >= (timestamp_measure_start + TIME_HELD_MS) ) /* Testing speed was held. Now slow down. */
            {
              speed_digital = 0;
              update_speed();
            }
            else if (timestamp_measure_current < (timestamp_measure_start + TIME_HELD_MS) ) /* TIME_HELD_MS milliseconds haven't passed yet, hold the testing speed. */
            {
              speed_digital = speed_digital_test;
              update_speed();           
            }
          break;
        }
      }
    }
  #endif
}

IRAM_ATTR void log_to_sdcard_task(void*)
{  
  /* SD card */
  const char LOG_FILE_HEADER[] = "Time\tTarget_Speed\tIR_Speed_Left\tIR_Speed_Right\tIR_Speed_Left_Trackbased\tIR_Speed_Right_Trackbased\tEstimated_Speed\tIR_Time_Difference_Left_Right\tAccel_Front_x\tAccel_Front_y\tAccel_Front_z\tRot_Front_x\tRot_Front_y\tRot_Front_z\tAccel_Heck_x\tAccel_Heck_y\tAccel_Heck_z\tRot_Heck_x\tRot_Heck_y\tRot_Heck_z\n";
  init_SD();
  init_log_file(LOG_FILE_HEADER);
  
  /* SD card initilized LED */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for(;;)
  {
    if (xSemaphoreTake(logging_semaphore, portMAX_DELAY) == pdTRUE)
    {
      char log_write_buffer [200];
      #if CALIBRATE_ACCELERATION
        sprintf(log_write_buffer, "%d\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%ld\t%lf\t%lf\t%lf\t%d\t%d\t%d\t%lf\t%lf\t%lf\t%d\t%d\t%d\n", 
          (int)imu_timestamp,
          (int)speed_digital,
          ir_left_speed,
          ir_right_speed,
          ir_left_speed_trackbased,
          ir_right_speed_trackbased,
          car_speed,
          ir_left_right_time_difference,
          front_imu_calibrated_acceleration_array[0],
          front_imu_calibrated_acceleration_array[1],
          front_imu_calibrated_acceleration_array[2],
          (int)front_imu_raw_data_array[0],
          (int)front_imu_raw_data_array[1],
          (int)front_imu_raw_data_array[2],    
          back_imu_calibrated_acceleration_array[0],
          back_imu_calibrated_acceleration_array[1],
          back_imu_calibrated_acceleration_array[2],
          (int)back_imu_raw_data_array[0],
          (int)back_imu_raw_data_array[1],
          (int)back_imu_raw_data_array[2]);
      #else
        sprintf(log_write_buffer, "%d\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%ld\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", 
          (int)imu_timestamp,
          (int)speed_digital,
          ir_left_speed,
          ir_right_speed,
          ir_left_speed_trackbased,
          ir_right_speed_trackbased,
          car_speed,
          ir_left_right_time_difference,
          (int)front_imu_raw_data_array[3],
          (int)front_imu_raw_data_array[4],
          (int)front_imu_raw_data_array[5],
          (int)front_imu_raw_data_array[0],
          (int)front_imu_raw_data_array[1],
          (int)front_imu_raw_data_array[2],    
          (int)back_imu_raw_data_array[3],
          (int)back_imu_raw_data_array[4],
          (int)back_imu_raw_data_array[5],
          (int)back_imu_raw_data_array[0],
          (int)back_imu_raw_data_array[1],
          (int)back_imu_raw_data_array[2]);
      #endif
      appendFile(SD, LOG_FILE_NAME, log_write_buffer);
      #if DEBUG
        Serial.printf("Logged: %s\n", log_write_buffer);
      #endif
    }
  }
}

/* calculate arithmatic mean of two double values */
inline double_t mean_two_doubles(double_t x, double_t y)
{
  return (x + y) * 0.5;
}

/* gets new acceleration sample and performs integration (only when calibrated, else it makes little sense) to obtain a rough speed estimate */
IRAM_ATTR void sample_imu_task(void*)
{
  init_imu();
  for(;;)
  {
    if (xSemaphoreTake(sampling_semaphore, portMAX_DELAY) == pdTRUE)
    {
      switch (sensorcar_state)
      {
        case SENSORCAR_TRACK_MAPPING_STATE: /* fallthrough on purpose */
        case SENSORCAR_MEASUREMENT_STATE:
        case SENSORCAR_RACING_STATE:
          #if CALIBRATE_ACCELERATION
            accel_previous = mean_two_doubles(front_imu_calibrated_acceleration_array[0],back_imu_calibrated_acceleration_array[0]);
            imu_read(); /* takes about 685us to get all data via I2C */
            accel_now = mean_two_doubles(front_imu_calibrated_acceleration_array[0],back_imu_calibrated_acceleration_array[0]);
            car_speed += mean_two_doubles(accel_now, accel_previous) * SAMPLING_INTERVAL / 1e6 * GRAVITY_FACTOR; /* trapezoidal integration of acceleration value */
            #if DEBUG
              Serial.printf("accel_previous=%lf; accel_now=%lf; car_speed=%lf.\n", accel_previous, accel_now, car_speed);
            #endif
          #else
            imu_read();
          #endif
      
          #if DATA_LOGGING
            xSemaphoreGive(logging_semaphore);
          #endif
          break;
      }
    }
  }
}

inline double_t clamp_value_smaller(double_t value, double_t threshold)
{
  if (value < threshold)
  {
    return threshold;
  }
  else
  {
    return value;
  }
}

/* obtains time values from most recent IR sensor passing and processes it */
IRAM_ATTR void ir_sensor_process_task(void*)
{
  for(;;)
  {
    /* block task until both IR sensors have a new value */
    if (xSemaphoreTake(ir_data_semaphore, portMAX_DELAY) == pdTRUE)
    {
      switch (sensorcar_state)
      {
        #if (MEASURE_SYSTEM == MEASURE_MODE_SWEEP) && (OPERATION_MODE == MEASURING_MODE) /* measurement track only contains straights */
          case SENSORCAR_MEASUREMENT_STATE: /* fallthrough on purpose */
            if (!(ir_left_history_time && ir_right_history_time)) { break; } /* avoid divide by zero*/
            /* Derive speed from part length, which can be more accurate if only straights are used in the track since the absolute errors cancel each other out */
            ir_left_speed_trackbased = TRACKPIECE_LENGTH[TRACK_STRAIGHT] * 1.0e6 / ir_left_history_time;
            ir_right_speed_trackbased = TRACKPIECE_LENGTH[TRACK_STRAIGHT] * 1.0e6 / ir_right_history_time;
        #else
          case SENSORCAR_MEASUREMENT_STATE: /* fallthrough on purpose */
        #endif
        case SENSORCAR_RACING_STATE:
          /* Derive speed from the passing times */
          #if CALIBRATE_IR_SPEED
            ir_left_speed  = CAL_LEFT[0] * TAPE_WIDTH * 1.0e6 / ir_left_passing_time + CAL_LEFT[1]; /* v = s / t. t in us, so a correction factor of 10^6 is required. */
            ir_right_speed = CAL_RIGHT[0] * TAPE_WIDTH * 1.0e6 / ir_right_passing_time + CAL_RIGHT[1];
            ir_left_speed  = clamp_value_smaller(ir_left_speed, 0.0);  /* due to the calibration offset, negative values are possible. Those are clamped to 0. */
            ir_right_speed = clamp_value_smaller(ir_right_speed, 0.0);
          #else
            ir_left_speed  = TAPE_WIDTH * 1.0e6 / ir_left_passing_time; /* v = s / t. t in us, so a correction factor of 10^6 is required. */
            ir_right_speed = TAPE_WIDTH * 1.0e6 / ir_right_passing_time;            
          #endif
          /* Overwrite previous speed value to avoid drift from accelerometer values */
          #if MEASURE_SYSTEM == MEASURE_MODE_SWEEP
            car_speed = (ir_left_speed_trackbased + ir_right_speed_trackbased) / 2;
          #else
            car_speed = (ir_left_speed + ir_right_speed) / 2; /* TODO: figure out a smarter way to get accurate curve speed */
          #endif
          if (xSemaphoreTake(finish_line_passed_semaphore, 0) == pdTRUE)
          {
            /* Sync car position if it desynced somewhere on the track. Last segment was zero (because finish line has been passed), so this segment has to be 1. This assumes, however, that the latency from lapping to receiving it wirelessly is low enough that the car does not pass a mark in between. Should this be the case, the car will be out of sync by one. */            
            track_position_index = 1;
          }
          else
          {
            track_position_index += 1;
            track_position_index %= number_track_pieces;
          }
          break;
        case SENSORCAR_TRACK_MAPPING_STATE:
          if (xSemaphoreTake(finish_line_passed_semaphore, 0) == pdTRUE)
          {
            /* this part executes one segment after the finish line */
            number_track_pieces = track_position_index;
            calculate_track_checkpoint_lengths(number_track_pieces);
            track_mapped_out_flag = true;
            #if DATA_LOGGING
              close_log_file();
            #endif
            #if DEBUG
              Serial.printf("Estimated track: {");
              for(uint8_t ii=0; ii < number_track_pieces; ii++) { Serial.printf(" %d", track_geometry[ii]); }
              Serial.printf("}\n");
            #endif
          }
          else
          {
            /* determine what kind of piece the last track piece was, then update the position. */
            track_geometry[track_position_index] = determine_track_piece(ir_left_right_time_difference);
            track_position_index += 1;
          }
      }
    }
  }
}

inline uint8_t increment_with_boundaries(uint8_t value, uint8_t increment, uint8_t modulo_divisior)
{
  return uint8_t(((uint16_t(value) + uint16_t(increment)) % uint16_t(modulo_divisior))); /* typecast to avoid overflow */
}

/* inefficient but highly functional. Faster search algorithms exist, but are not required. */
inline uint8_t find_closest_legal_vdigi(float value)
{
  uint8_t closest_value       = 0;
  float   smallest_difference = 10e3; /* init with big value*/
  float   current_difference  = 0.0;

  /* step through array and find the smallest absolute difference to the current value */
  for(uint8_t ii = 0; ii < sizeof(AVAILABLE_VDIGI); ii++)
  {
    current_difference = abs(float(AVAILABLE_VDIGI[ii]) - value);
    if ( current_difference < smallest_difference)
    {
      smallest_difference = current_difference;
      closest_value = AVAILABLE_VDIGI[ii];
    }
  }

  return closest_value;
}

inline uint8_t simple_algorithm(uint8_t track_position_index, uint8_t number_track_pieces, uint8_t* track_geometry)
{           
  /*
    'increment_with_boundaries(track_position_index, 1, number_track_pieces)' identifies the next track piece index (0...number_track_pieces-1)
    'track_geometry[index]' is a look up table (LUT) for the type of segment any piece of the track is (for example, a TRACK_STRAIGHT or TRACK_CURVE_LEFT_INNER_TRACK)
    'TARGET_TRACKPIECE_SPEED_DIGITAL[segment_type]' is a LUT for the target vdigi that should be driven on a specific type of track piece.
  */
  return TARGET_TRACKPIECE_SPEED_DIGITAL[track_geometry[increment_with_boundaries(track_position_index, 1, number_track_pieces)]];
}

inline uint8_t average_algorithm(uint8_t track_position_index, uint8_t number_track_pieces, uint8_t* track_geometry)
{
  uint sum_buffer = 0;
  for(uint8_t ii = 0; ii < ALGORITHM_AVERAGE_NUMBER; ii++)
  {
    sum_buffer += TARGET_TRACKPIECE_SPEED_DIGITAL[track_geometry[increment_with_boundaries(track_position_index, ii+1, number_track_pieces)]];
  }
  
  return find_closest_legal_vdigi( float(sum_buffer)/float(ALGORITHM_AVERAGE_NUMBER) );
}

IRAM_ATTR void velocity_controller_task(void*)
{
  for(;;)
  {
    /* execute every CONTROLLER_INTERVAL microseconds */
    if (xSemaphoreTake(controller_timer_semaphore, portMAX_DELAY) == pdTRUE)
    {
      switch (sensorcar_state)
      {
        case SENSORCAR_IDLE_STATE:
          reset_all_state_data();
          update_speed();
        break;
        case SENSORCAR_RACING_STATE:
          #if ALGORITHM_TYPE == ALGORITHM_SIMPLE
            /* depending on the next track piece, set a target speed
            '(track_position_index + 1) % number_track_pieces' identifies the next track piece index (0...number_track_pieces-1)
            'track_geometry[index]' is a look up table (LUT) for the type of segment any piece of the track is (for example, a TRACK_STRAIGHT or TRACK_CURVE_LEFT_INNER_TRACK)
            'MAXIMUM_TRACKPIECE_SPEED_DIGITAL[segment_type]' is a LUT for the maximum speed allowed on a specific type of track piece.
            */
            speed_digital = simple_algorithm(track_position_index, number_track_pieces, track_geometry);
          #elif ALGORITHM_TYPE == ALGORITHM_AVERAGE
            speed_digital = average_algorithm(track_position_index, number_track_pieces, track_geometry);
          #endif
          update_speed();
          break;
        case SENSORCAR_TRACK_MAPPING_STATE:
          if (track_mapped_out_flag)
          {
            speed_digital = 0;
          }
          else
          {
            speed_digital = 40;
          }
          update_speed();
          break;
      }
    }
  }
}

/* #####################################################
Initialization
##################################################### */
void setup()
{
  /* serial interface */
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  /* wireless comms */
  init_wifi();

  /* ir sensors */
  init_ir_sensors();

  /* timers */
  init_timers();

  /* Tasks. Some Tasks do their own initialization to avoid memory bugs with this version of FreeRTOS. */

  #if (OPERATION_MODE==MEASURING_MODE)
    xTaskCreatePinnedToCore(log_to_sdcard_task,       "log_to_sdcard_task",       10000, NULL, DATA_LOG_PRIO      ,       &log_to_sdcard_task_handle,       DATA_LOG_CORE);
    xTaskCreatePinnedToCore(measurement_task,         "measurement_task",         10000, NULL, MEASUREMENT_PRIO ,         &measurement_task_handle,         MEASUREMENT_CORE);
  #else
    xTaskCreatePinnedToCore(velocity_controller_task, "velocity_controller_task", 10000, NULL, VELOCITY_CONTROLLER_PRIO,  &velocity_controller_task_handle, VELOCITY_CONTROLLER_CORE);
  #endif

  xTaskCreatePinnedToCore(sample_imu_task,          "sample_imu_task",          10000, NULL, IMU_SAMPLE_PRIO,           &sample_imu_task_handle,         IMU_SAMPLE_CORE);
  xTaskCreatePinnedToCore(ir_sensor_process_task,   "ir_sensor_process_task",   10000, NULL, IR_SENSOR_PROCESS_PRIO,    &ir_sensor_process_task_handle,  IR_SENSOR_PROCESS_CORE);
}

/* #####################################################
Main Loop
##################################################### */
void loop()
{
  /* This loop does nothing, every function of the firmware is implemented in tasks. */
  DELAY_N_MS(portMAX_DELAY); /* block task forever */
}
