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
    // joint physical limit
    // suspension param:
    // FL: 0 ~ -90 deg
    // FR: 0 ~ 90 deg
    // RL: 0 ~ 90 deg
    // RR: 0 ~ -90 deg
    lockVariable();
    // FL
    motor_FL.pos_upper_limit = 0;   // motor maximum position allowed in deg
    motor_FL.pos_lower_limit = -90; // motor minimum position allowed in deg    suspension_motor.torq_current_limit = SUSPENSION_ABS_CURRENT_LIMIT;
    motor_FL.Kp = 5.0;
    motor_FL.Kd = 0.5;
    motor_FL.dir = -1; // 1 -> CW ; -1 -> CCW

    // FR
    motor_FR.pos_upper_limit = 90; // motor maximum position allowed in deg
    motor_FR.pos_lower_limit = 0;  // motor minimum position allowed in deg    suspension_motor.torq_current_limit = SUSPENSION_ABS_CURRENT_LIMIT;
    motor_FR.Kp = 5.0;
    motor_FR.Kd = 0.5;
    motor_FR.dir = 1; // 1 -> CW ; -1 -> CCW
    // RL
    motor_RL.pos_upper_limit = 90; // motor maximum position allowed in deg
    motor_RL.pos_lower_limit = 0;  // motor minimum position allowed in deg    suspension_motor.torq_current_limit = SUSPENSION_ABS_CURRENT_LIMIT;
    motor_RL.Kp = 5.0;
    motor_RL.Kd = 0.5;
    motor_RL.dir = 1; // 1 -> CW ; -1 -> CCW
    // RR
    motor_RR.pos_upper_limit = 0;   // motor maximum position allowed in deg
    motor_RR.pos_lower_limit = -90; // motor minimum position allowed in deg    suspension_motor.torq_current_limit = SUSPENSION_ABS_CURRENT_LIMIT;
    motor_RR.Kp = 5.0;
    motor_RR.Kd = 0.5;
    motor_RR.dir = -1; // 1 -> CW ; -1 -> CCW
    unlockVariable();
}

// motor driver initialization
void sus_motor_init()
{
    Serial.printf("canbus init start\r\n");
    FL_motor.motor_drv_reset();
    FR_motor.motor_drv_reset();
    RL_motor.motor_drv_reset();
    RR_motor.motor_drv_reset();

    Serial.printf("reseted motor driver\r\n");
    FL_motor.set_mode(MOTION_CONTROL_LOOP);
    FR_motor.set_mode(MOTION_CONTROL_LOOP);
    RL_motor.set_mode(MOTION_CONTROL_LOOP);
    RR_motor.set_mode(MOTION_CONTROL_LOOP);

    motor_param_config();
    Serial.printf("configured mode\r\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
    // FL
    if (FL_motor.get_state_msg_2())
    {
        lockVariable();
        motor_FL.state = READY;
        unlockVariable();
        Serial.printf("FL motor done\r\n");
    }
    else
    {
        lockVariable();
        motor_FL.state = UNKNOWN;
        unlockVariable();
    }

    // FR
    if (FR_motor.get_state_msg_2())
    {
        lockVariable();
        motor_FR.state = READY;
        unlockVariable();
        Serial.printf("FR motor done\r\n");
    }
    else
    {
        lockVariable();
        motor_FR.state = UNKNOWN;
        unlockVariable();
    }

    // RL
    if (RL_motor.get_state_msg_2())
    {
        lockVariable();
        motor_RL.state = READY;
        unlockVariable();
        Serial.printf("RL motor done\r\n");
    }
    else
    {
        lockVariable();
        motor_RL.state = UNKNOWN;
        unlockVariable();
    }

    // RR
    if (RR_motor.get_state_msg_2())
    {
        lockVariable();
        motor_RR.state = READY;
        unlockVariable();
        Serial.printf("RR motor done\r\n");
    }
    else
    {
        lockVariable();
        motor_RR.state = UNKNOWN;
        unlockVariable();
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