#pragma once

#include "share_param.h"
#include "rmdx8_pro_canbus_v3.h"

// driver define
rmdx8_pro FL_motor(&can2, SUS_MOTOR_FL_ID);
rmdx8_pro FR_motor(&can2, SUS_MOTOR_FR_ID);
rmdx8_pro RL_motor(&can2, SUS_MOTOR_RL_ID);
rmdx8_pro RR_motor(&can2, SUS_MOTOR_RR_ID);

// joint parameter configuaration
void motor_param_config()
{
    lockVariable();
    // FL
    motor_FL.pos_upper_limit = FL_UPPER_LIMIT; // motor maximum position allowed in deg
    motor_FL.pos_lower_limit = FL_LOWER_LIMIT; // motor minimum position allowed in deg    suspension_motor.torq_current_limit = SUSPENSION_ABS_CURRENT_LIMIT;
    motor_FL.Kp = FL_KP;
    motor_FL.Kd = FL_KD;
    motor_FL.dir = FL_ROTATE_DIR; // 1 -> CW ; -1 -> CCW

    // FR
    motor_FR.pos_upper_limit = FR_UPPER_LIMIT; // motor maximum position allowed in deg
    motor_FR.pos_lower_limit = FR_LOWER_LIMIT; // motor minimum position allowed in deg    suspension_motor.torq_current_limit = SUSPENSION_ABS_CURRENT_LIMIT;
    motor_FR.Kp = FR_KP;
    motor_FR.Kd = FR_KD;
    motor_FR.dir = FR_ROTATE_DIR; // 1 -> CW ; -1 -> CCW
    // RL
    motor_RL.pos_upper_limit = RL_UPPER_LIMIT; // motor maximum position allowed in deg
    motor_RL.pos_lower_limit = RL_LOWER_LIMIT;              // motor minimum position allowed in deg    suspension_motor.torq_current_limit = SUSPENSION_ABS_CURRENT_LIMIT;
    motor_RL.Kp = RL_KP;
    motor_RL.Kd = RL_KD;
    motor_RL.dir = RL_ROTATE_DIR; // 1 -> CW ; -1 -> CCW
    // RR
    motor_RR.pos_upper_limit = RR_UPPER_LIMIT;   // motor maximum position allowed in deg
    motor_RR.pos_lower_limit = RR_LOWER_LIMIT; // motor minimum position allowed in deg    suspension_motor.torq_current_limit = SUSPENSION_ABS_CURRENT_LIMIT;
    motor_RR.Kp = RR_KP;
    motor_RR.Kd = RR_KD;
    motor_RR.dir = RR_ROTATE_DIR; // 1 -> CW ; -1 -> CCW
    unlockVariable();
}

uint8_t motor_driver_detect(suspension_motor *motor_joint, rmdx8_pro *motor){
    if (motor -> get_state_msg_2())
    {
        lockVariable();
        motor_joint -> state = READY;
        unlockVariable();
        return 1;
    }
    else
    {
        lockVariable();
        motor_joint -> state = UNKNOWN;
        unlockVariable();
        return 0;
    }
}

// motor driver initialization
void sus_motor_init()
{
    Serial.printf("canbus init start\r\n");
    Serial.printf("reset motor driver\r\n");
    FL_motor.motor_drv_reset();
    FR_motor.motor_drv_reset();
    RL_motor.motor_drv_reset();
    RR_motor.motor_drv_reset();

    Serial.printf("set motor controller mode\r\n");
    FL_motor.set_mode(MOTION_CONTROL_LOOP);
    FR_motor.set_mode(MOTION_CONTROL_LOOP);
    RL_motor.set_mode(MOTION_CONTROL_LOOP);
    RR_motor.set_mode(MOTION_CONTROL_LOOP);

    Serial.printf("init param\r\n");
    motor_param_config();

    vTaskDelay(pdMS_TO_TICKS(2000));

    Serial.printf("detect driver\r\n");
    if(motor_driver_detect(&motor_FL, &FL_motor) == 1 
        &&  motor_driver_detect(&motor_FR, &FR_motor) == 1
        && motor_driver_detect(&motor_RL, &RL_motor) == 1
        && motor_driver_detect(&motor_RR, &RR_motor) == 1){
            Serial.printf("All driver init finish\r\n");
        }
    else{
        Serial.printf("detect driver fail\r\n");
    }

    FL_motor.motor_drv_disable();
    FR_motor.motor_drv_disable();
    RL_motor.motor_drv_disable();
    RR_motor.motor_drv_disable();
    Serial.printf("motor idle state\r\n");
}

// perform the control action when the motor controller enter different state
void sus_motor_action_update(suspension_motor *target_joint, rmdx8_pro *target_motor)
{
    lockVariable();
    switch (target_joint->state)
    {
    case ACTIVE:
        if (target_motor->set_motion_control(LIMIT(target_joint->pos_setpt, target_joint->pos_lower_limit, target_joint->pos_upper_limit) * DEG_TO_RAD, 0.0, 0.0, target_joint->Kp, target_joint->Kd))
        {
            target_joint->current = (float)target_motor->get_motor_torq();
            target_joint->rpm = (float)target_motor->get_axis_rpm();
            target_joint->pos = (float)target_motor->get_axis_position();
            target_joint->temp = (float)target_motor->get_motor_temp();
            // if (target_joint->current > OVERCURRENT_THRESHOLD)
            // {
            //     target_joint->state = ERROR;
            //     target_joint->err_code = OVERCURRENT;
            // }
            // if (target_joint->temp > OVERTEMP_THRESHOLD)
            // {
            //     target_joint->state = ERROR;
            //     target_joint->err_code = OVERTEMP;
            // }
            target_joint->cmd_miss_cnt = 0;
        }
        else
        {
            target_joint->cmd_miss_cnt++;
            if (target_joint->cmd_miss_cnt > 20)
            {
                target_joint->state = DISCONNECT;
                target_joint->cmd_miss_cnt = 22;
            }
        }
        // if (cmd_bus_connection == false)
        // {
        //     target_joint->state = READY;
        // }
        // else
        // {
        //     target_joint->state = ACTIVE;
        // }
        break;

    case READY:
        // Serial.printf("motor in READYSTATE %d\r\n", joint.cmd_miss_cnt);
        if (target_joint->disable_flag == true)
        {
            target_motor->motor_drv_disable();
            target_joint->disable_flag = false;
        }
        else
        {
            if (target_motor->get_state_msg_2())
            {
                // Serial.printf("get_stuff\r\n");
                target_joint->current = (float)target_motor->get_motor_torq();
                target_joint->rpm = (float)target_motor->get_axis_rpm();
                target_joint->pos = (float)target_motor->get_axis_position();
                target_joint->temp = (float)target_motor->get_motor_temp();
                // Serial.printf("pos: %.1f\r\n", target_joint->pos);
                target_joint->cmd_miss_cnt = 0;
            }
            else
            {
                target_joint->cmd_miss_cnt++;
                if (target_joint->cmd_miss_cnt > 20)
                {
                    target_joint->state = DISCONNECT;
                    target_joint->cmd_miss_cnt = 22;
                }
            }
        }
        // if (cmd_bus_connection == true)
        // {
        //     target_joint->state = ACTIVE;
        // }
        // else
        // {
        //     target_joint->state = READY;
        // }
        break;

    case CALIBRATING:
        target_joint->current = (float)target_motor->get_motor_torq();
        target_joint->rpm = (float)target_motor->get_axis_rpm();
        target_joint->pos = (float)target_motor->get_axis_position();
        target_joint->temp = (float)target_motor->get_motor_temp();

        target_motor->motor_drv_disable();
        vTaskDelay(pdMS_TO_TICKS(1500));
        target_motor->set_curr_pos_zero();
        vTaskDelay(pdMS_TO_TICKS(1500));
        target_motor->motor_drv_reset();
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (target_motor->get_state_msg_2())
        {
            target_joint->current = (float)target_motor->get_motor_torq();
            target_joint->rpm = (float)target_motor->get_axis_rpm();
            target_joint->pos = (float)target_motor->get_axis_position();
            target_joint->temp = (float)target_motor->get_motor_temp();
            // target_joint->state = READY;
        }
        else
        {
            // target_joint->state = DISCONNECT;
        }
        target_joint->state = READY;
        break;

    case DISCONNECT:
        // sus_motor_state_update(&target_joint, &target_motor);
        break;

    case UNKNOWN:
        // sus_motor_state_update(&target_joint, &target_motor);
        break;

    case ERROR:
        break;

    default:
        break;
    }
    unlockVariable();
}

// motor_control_task
void motor_control_task(void *pvParameters)
{
    (void)pvParameters;
    sus_motor_init();
    TickType_t xLastWakeTime_motor_control_task = xTaskGetTickCount();
    while (1)
    {
        xLastWakeTime_motor_control_task = xTaskGetTickCount();
        sus_motor_action_update(&motor_FL, &FL_motor);
        sus_motor_action_update(&motor_FR, &FR_motor);
        sus_motor_action_update(&motor_RL, &RL_motor);
        sus_motor_action_update(&motor_RR, &RR_motor);
        vTaskDelayUntil(&xLastWakeTime_motor_control_task, motor_control_task_delay_time);
    }
}