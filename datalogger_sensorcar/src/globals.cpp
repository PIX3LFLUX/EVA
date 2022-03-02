#include "globals.h"

DRAM_ATTR SemaphoreHandle_t sampling_semaphore              = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t logging_semaphore               = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t sd_card_access_semaphore        = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t measurement_timer_semaphore   = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t ir_data_semaphore               = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t controller_timer_semaphore      = xSemaphoreCreateBinary();
DRAM_ATTR SemaphoreHandle_t finish_line_passed_semaphore    = xSemaphoreCreateBinary();

DRAM_ATTR TaskHandle_t measurement_task_handle           = NULL;
DRAM_ATTR TaskHandle_t log_to_sdcard_task_handle         = NULL;
DRAM_ATTR TaskHandle_t sample_imu_task_handle            = NULL;
DRAM_ATTR TaskHandle_t ir_sensor_process_task_handle     = NULL;
DRAM_ATTR TaskHandle_t velocity_controller_task_handle   = NULL;
DRAM_ATTR TaskHandle_t track_mapper_task_handle          = NULL;

DRAM_ATTR unsigned long time_tic = 0;
DRAM_ATTR unsigned long time_toc = 0;
DRAM_ATTR unsigned long toc_tic_time_difference = 0;

/* car state data */
DRAM_ATTR uint8_t  track_position_index      = 0;    /* number of the track piece the car is currently on */
DRAM_ATTR uint8_t  speed_digital             = 0;    /* in counts, 0...255, only values from 10 to 172 actually do anything, since the CU only reads voltages from ~230mV to 2.17mV. Effective resolution is 7.3399 bits.*/
DRAM_ATTR uint8_t  speed_digital_previous    = 0;   /* in counts, 0...255*/
DRAM_ATTR double_t ir_left_speed             = 0.0; /* in m/s, calculated from effective tape width and time tape was detected */
DRAM_ATTR double_t ir_right_speed            = 0.0; /* in m/s, calculated from effective tape width and time tape was detected */
DRAM_ATTR double_t ir_left_speed_trackbased  = 0.0; /* in m/s, calculated from time between tape detections and track piece length - more prone to errors but more accurate */
DRAM_ATTR double_t ir_right_speed_trackbased = 0.0; /* in m/s, calculated from time between tape detections and track piece length - more prone to errors but more accurate */
DRAM_ATTR double_t car_speed                 = 0.0; /* in m/s, calculated from ir speeds and accelerometer x axes integration */
DRAM_ATTR double_t accel_now                 = 0.0; /* in m/s^2 */
DRAM_ATTR double_t accel_previous            = 0.0; /* in m/s^2 */

DRAM_ATTR uint8_t  sensorcar_state           = SENSORCAR_INITIAL_STATE;
DRAM_ATTR bool     track_mapped_out_flag     = false;   /* is true when the track geometry has been figured out */

IRAM_ATTR void reset_all_state_data()
{
    track_position_index      = 0;
    speed_digital             = 0;
    ir_left_speed             = 0.0;
    ir_right_speed            = 0.0;
    ir_left_speed_trackbased  = 0.0;
    ir_right_speed_trackbased = 0.0;
    car_speed                 = 0.0;
    accel_now                 = 0.0;
    accel_previous            = 0.0;
}

IRAM_ATTR void tic()
{
    time_tic = micros();
}

IRAM_ATTR void toc()
{
    time_toc = micros();
    toc_tic_time_difference = time_toc - time_tic;
    Serial.printf("Round trip time: %ld us\n", toc_tic_time_difference);
}