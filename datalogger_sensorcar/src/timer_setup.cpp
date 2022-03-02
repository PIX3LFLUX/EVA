#include "timer_setup.h"
hw_timer_t * sample_timer = NULL;
hw_timer_t * controller_timer = NULL;
hw_timer_t * measurement_timer = NULL;

inline void init_timer_generic(hw_timer_t * timer_object, uint8_t number_timer, void function_pointer(), int timer_interval_microseconds)
{
  /* 
  Use one timer of 4 (counted from zero).
  Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
  */
  timer_object = timerBegin(number_timer, 80, true);
  timerAttachInterrupt(timer_object, function_pointer, true);       /* trigger the referenced function on timer overflow */
  timerAlarmWrite(timer_object, timer_interval_microseconds, true); /* load the timer and enable autoreload */
  timerAlarmEnable(timer_object);                                   /* start the timer */
}

void init_timers()
{
  init_timer_generic(sample_timer,        3, &on_sample_timer,         SAMPLING_INTERVAL);
  init_timer_generic(measurement_timer, 2, &on_measurement_timer,  MEASUREMENT_TIMER_INTERVAL);
  init_timer_generic(controller_timer,    1, &on_controller_timer,     CONTROLLER_INTERVAL);
}

inline void halt_timer_generic(hw_timer_t * timer_object, int timer_interval_microseconds)
{
  timerAlarmDisable(timer_object); /* disable reaccuring triggering of interrupt service routine */
  timerAlarmWrite(timer_object, timer_interval_microseconds, true); /* reload timer */
}

IRAM_ATTR void halt_timers()
{
  halt_timer_generic(sample_timer,        SAMPLING_INTERVAL);
  halt_timer_generic(controller_timer,    CONTROLLER_INTERVAL);
  halt_timer_generic(measurement_timer, MEASUREMENT_TIMER_INTERVAL);
}

IRAM_ATTR void restart_timers()
{
  timerAlarmEnable(sample_timer);
  timerAlarmEnable(controller_timer);
  timerAlarmEnable(measurement_timer);
}

IRAM_ATTR void on_sample_timer()
{
  xSemaphoreGiveFromISR(sampling_semaphore, NULL); /* unblock imu sampling task */
}

IRAM_ATTR void on_controller_timer()
{
  xSemaphoreGiveFromISR(controller_timer_semaphore, NULL);
}

IRAM_ATTR void on_measurement_timer()
{
  xSemaphoreGiveFromISR(measurement_timer_semaphore, NULL);
}