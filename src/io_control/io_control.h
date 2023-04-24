#pragma once
#include "share_param.h"
#include "io_control\display\oled_control.h"
#include "io_control\gpio\gpio_control.h"


// update the ADC pin value
void io_control_task(void *pvParameters)
{
    (void)pvParameters;
    gpio_init();
    oled_init();
    TickType_t xLastWakeTime_io_control_task = xTaskGetTickCount();

    while (1)
    {
        xLastWakeTime_io_control_task = xTaskGetTickCount();
        gpio_update();
        oled_page_upadate();
        vTaskDelayUntil(&xLastWakeTime_io_control_task, io_control_task_delay_time);
    }
}