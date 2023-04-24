#include "share_param.h"
#include "motor_control\suspension_control.h"
#include "io_control\io_control.h"
#include "cmd_logic\cmd_logic_control.h"

// task init function
void task_init(){
  createSemaphore();
  xTaskCreatePinnedToCore(motor_control_task, "motor_control_task", 2048, NULL, 2, &motor_control_handle ,tskNO_AFFINITY);
  xTaskCreatePinnedToCore(cmd_logic_task, "cmd_logic_task", 4096, NULL, 1, &cmd_logic_task_handle, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(io_control_task, "io_control_task", 4096, NULL, 3, &io_control_handle, tskNO_AFFINITY);
}

void setup()
{
  Serial.begin(921600);
  I2C_init();
  // task initialization and start running
  task_init();
}

void loop()
{
}