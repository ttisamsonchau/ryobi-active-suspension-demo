#include "share_param.h"
#include "motor_control\suspension_control.h"
#include "io_control\io_control.h"
#include "cmd_logic\cmd_logic_control.h"

// task init function
void task_init(){
  createSemaphore();
  xTaskCreatePinnedToCore(motor_control_task, "motor_control_task", motor_control_task_stack, NULL, motor_control_task_priority, &motor_control_handle ,tskNO_AFFINITY);
  xTaskCreatePinnedToCore(cmd_logic_task, "cmd_logic_task", cmd_logic_task_stack, NULL, cmd_logic_task_priority, &cmd_logic_task_handle, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(io_control_task, "io_control_task", io_control_task_stack, NULL, io_control_task_priority, &io_control_handle, tskNO_AFFINITY);
}

void setup()
{
  serial_init();
  I2C_init();
  // task initialization and start running
  task_init();
}

void loop()
{
}