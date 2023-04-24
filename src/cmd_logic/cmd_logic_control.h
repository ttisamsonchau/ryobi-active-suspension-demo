#pragma once
#include "share_param.h"
#include "cmd_logic\imu\imu_control.h"

void cmd_logic_init()
{
    // can_bus_init();
    imu_init();
}

void cmd_logic_task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime_cmd_logic_task = xTaskGetTickCount();
    cmd_logic_init();
    while (1)
    {
        xLastWakeTime_cmd_logic_task = xTaskGetTickCount();
        imu_update();
        vTaskDelayUntil(&xLastWakeTime_cmd_logic_task, cmd_logic_task_delay_time);
    }
}
