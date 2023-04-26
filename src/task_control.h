#pragma once
#include "Arduino.h"

// task frequency
// the delay time in each loop (in delay ms)
const TickType_t cmd_logic_task_delay_time = pdMS_TO_TICKS(10);     // 100Hz
const TickType_t io_control_task_delay_time = pdMS_TO_TICKS(10);    // 100Hz
const TickType_t motor_control_task_delay_time = pdMS_TO_TICKS(10); // 100Hz

// task handle
TaskHandle_t io_control_handle;
TaskHandle_t cmd_logic_task_handle;
TaskHandle_t motor_control_handle;

// task priority
#define io_control_task_priority 2
#define cmd_logic_task_priority 1
#define motor_control_task_priority 1

// task stack size
#define io_control_task_stack 4096
#define cmd_logic_task_stack 4096
#define motor_control_task_stack 4096

// Semaphore
SemaphoreHandle_t paramSemaphore;
void createSemaphore()
{
  paramSemaphore = xSemaphoreCreateMutex();
  xSemaphoreGive((paramSemaphore));
}

// Lock the variable indefinietly. ( wait for it to be accessible )
void lockVariable()
{
  xSemaphoreTake(paramSemaphore, portMAX_DELAY);
}

// give back the semaphore.
void unlockVariable()
{
  xSemaphoreGive(paramSemaphore);
}
