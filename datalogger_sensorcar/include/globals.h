#pragma once
#include <Arduino.h>

/* settings that affect much of the code functionality */
#define DEBUG                       0   /* debug output in main loop via serial port, slows down the timer, etc to ease debug. Disable for final code. 0:disabled 1:enabled */
#define MEASURE_RTT                 0   /* enable tic and toc functions for measuring round trip time and outputting it to the serial terminal */
#define CALIBRATE_ACCELERATION      1   /* use correction values measured and calculated externally to align axes of the accelerometers to that of the car. Set to 0 to obtain sensor raw values. */
#define CALIBRATE_IR_SPEED          0   /* use correction values measured and calculated externally to better match the speed data derived from passing tape to that of the speed derived from the time between segment passings an TRACK_STRAIGHTs. */
#define WIRELESS_TRANSMISSION_TRIES 1   /* because it's not certain that the other uC has received the message, we send a couple times. */

/* states for operational mode */
    #define RACING_MODE                 0   /* car drives a lap to determine track layout, then runs a strategy depending on ALGORITHM_TYPE */
    #define MEASURING_MODE              1   /* car drives according to set program rules according to MEASURE_SYSTEM while recording data */
#define OPERATION_MODE                  RACING_MODE   /* set one of the modes above */
#define DATA_LOGGING                    (OPERATION_MODE==MEASURING_MODE)   /* if true, log data to SD card */

/* states for MEASURE_SYSTEM */
    #define MEASURE_MODE_SWEEP          1   /* drive a different speed each race with a start, increment and running condition */
        #define VDIGI_INITIAL_VALUE         14
        #define VDIGI_INCREMENT_VALUE       3
        #define VDIGI_MAX_VALUE             110     /* if a testing value would go over this threshold, it will delete the task instead of testing it. */
        #define TIME_HELD_MS                6000    /* in milliseconds. Time to hold the current speed setting. Will be rounded to the nearest multiple of MEASUREMENT_TIMER_INTERVAL. About 100ms Latency will be added, too. */
        #define TIME_MEASURE_MS             0    /* in milliseconds. Time to measure after setting speed back to 0. Will be rounded to the nearest multiple of MEASUREMENT_TIMER_INTERVAL. About 100ms Latency will be added, too. */
#define MEASURE_SYSTEM                  MEASURE_MODE_SWEEP   /* set one of the modes above */

/* states for ALGORITHM_TYPE */
    #define ALGORITHM_DISABLE           0   /* disable automatic speed control */
    #define ALGORITHM_SIMPLE            1   /* drive a set speed based on the next track piece */
    #define ALGORITHM_AVERAGE           2   /* drive a set speed based on the average of the next ALGORITHM_AVERAGE_NEXT_NUMBER track pieces */
        #define ALGORITHM_AVERAGE_NUMBER   5
#define ALGORITHM_TYPE                  ALGORITHM_AVERAGE   /* set one of the modes above */

/* states for sensorcar */
#define SENSORCAR_IDLE_STATE            0   /* car does nothing */
#define SENSORCAR_RACING_STATE          1   /* car drives according to ALGORITHM_TYPE */
#define SENSORCAR_TRACK_MAPPING_STATE   2   /* car drives a lap to determine track layout, then stops */
#define SENSORCAR_MEASUREMENT_STATE     3   /* car drives according to set program rules according to MEASURE_SYSTEM while recording data */
#define SENSORCAR_INITIAL_STATE         SENSORCAR_IDLE_STATE   /* initialization value for the state variable */

/* FreeRTOS Settings*/
#define INCLUDE_vTaskSuspend        1 /* if set to '1' then specifying the block time as portMAX_DELAY will cause the task to block indefinitely (without a timeout) */

/* Task priorities
Note: If a task is run as only active task on a core and its priority is higher than 0 and it never idle waits, the core can never enter the idle task and the watchdog timer will be triggered, causing a reboot.
*/
#define IDLE_PRIO                   0
#define IMU_SAMPLE_PRIO             IDLE_PRIO+2
#define DATA_LOG_PRIO               IDLE_PRIO+3
#define MEASUREMENT_PRIO            IDLE_PRIO+1
#define IR_SENSOR_PROCESS_PRIO      IDLE_PRIO+1
#define VELOCITY_CONTROLLER_PRIO    IDLE_PRIO+1

/* Task Cores
main loop and setup run on core 1, wireless receive runs on core 0.
IMPORTANT: Tasks that share data should not be on different cores, since global variables are used instead of the 'proper' way with queues, because it's faster.
*/
#define IMU_SAMPLE_CORE             1
#define DATA_LOG_CORE               1
#define MEASUREMENT_CORE            1
#define IR_SENSOR_PROCESS_CORE      1
#define VELOCITY_CONTROLLER_CORE    1

#define DELAY_N_MS(n) (vTaskDelay(n/portTICK_PERIOD_MS))    /* puts a task into the blocked state for N microseconds */

/* Task handles
Used to suspend, resume or terminate specific tasks
*/
extern DRAM_ATTR TaskHandle_t measurement_task_handle;
extern DRAM_ATTR TaskHandle_t log_to_sdcard_task_handle;
extern DRAM_ATTR TaskHandle_t sample_imu_task_handle;
extern DRAM_ATTR TaskHandle_t ir_sensor_process_task_handle;
extern DRAM_ATTR TaskHandle_t velocity_controller_task_handle;
extern DRAM_ATTR TaskHandle_t track_mapper_task_handle;

/* Semaphores
Used to notify tasks or block shared resources.
*/
extern DRAM_ATTR SemaphoreHandle_t sampling_semaphore;              /* is set by a timer to sample the IMUs at set time intervals */
extern DRAM_ATTR SemaphoreHandle_t logging_semaphore;               /* is set when there is new IMU data to log to the sd card  */
extern DRAM_ATTR SemaphoreHandle_t sd_card_access_semaphore;        /* used for access to the shared resource 'sd card' */
extern DRAM_ATTR SemaphoreHandle_t measurement_timer_semaphore;   /* is set every time the measurement timer overflows */
extern DRAM_ATTR SemaphoreHandle_t ir_data_semaphore;               /* is set when new IR data has arrived */
extern DRAM_ATTR SemaphoreHandle_t controller_timer_semaphore;      /* is set every time the controller timer overflows */
extern DRAM_ATTR SemaphoreHandle_t finish_line_passed_semaphore;    /* is set every time the car passes the finish line and receives the notification for it wirelessly */

extern DRAM_ATTR unsigned long toc_tic_time_difference; /* time difference between calls of the tic() and toc() functions */

/* car state data */
extern DRAM_ATTR uint8_t  track_position_index;
extern DRAM_ATTR uint8_t  speed_digital;
extern DRAM_ATTR uint8_t  speed_digital_previous;
extern DRAM_ATTR double_t ir_left_speed;
extern DRAM_ATTR double_t ir_right_speed;
extern DRAM_ATTR double_t ir_left_speed_trackbased;
extern DRAM_ATTR double_t ir_right_speed_trackbased;
extern DRAM_ATTR double_t car_speed;        
extern DRAM_ATTR double_t accel_now;        
extern DRAM_ATTR double_t accel_previous;   

extern DRAM_ATTR uint8_t  sensorcar_state;
extern DRAM_ATTR bool     track_mapped_out_flag;

IRAM_ATTR void reset_all_state_data();
IRAM_ATTR void tic();
IRAM_ATTR void toc();