#include "rmdx8_pro_canbus_v3.h"
// init motor driver
rmdx8_pro::rmdx8_pro(canbus_driver_twai *_can, int set_motor_id)
{
    can1 = _can;
    motor_id = set_motor_id;
    // printf("ID: 0x%x\r\n",motor_id);
    can1->add_subscribe_node(0x240 + motor_id);
    can1->add_subscribe_node(0x500 + motor_id);
    can_tx_buff[8] = {0};
    can_rx_buff[8] = {0};
}

// get the pid parameter
// parameter: Current loop Kp Ki uint8 0~256 (0 ~ max_value)
// parameter: Velocity loop Kp Ki uint8 0~256 (0 ~ max_value)
// parameter: Position loop Kp Ki uint8 0~256 (0 ~ max_value)
uint8_t rmdx8_pro::get_pid_param()
{
    can_tx_buff[0] = GET_PID_CMD;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == GET_PID_CMD)
        {
            rmd_x8_motor.I.Kp = can1->can_rx_message.data[2];
            rmd_x8_motor.I.Ki = can1->can_rx_message.data[3];
            rmd_x8_motor.V.Kp = can1->can_rx_message.data[4];
            rmd_x8_motor.V.Ki = can1->can_rx_message.data[5];
            rmd_x8_motor.P.Kp = can1->can_rx_message.data[6];
            rmd_x8_motor.P.Ki = can1->can_rx_message.data[7];
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
// get acceleration setting
// parameter: accerleration in dps (100 ~ 60000)
int32_t rmdx8_pro::get_acc()
{
    can_tx_buff[0] = GET_PID_CMD;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == GET_ACC_CMD)
        {
            rmd_x8_motor.acc_dps = (int32_t)can1->can_rx_message.data[7] << 24 | can1->can_rx_message.data[6] << 16 | can1->can_rx_message.data[5] << 8 | can1->can_rx_message.data[4];
            return rmd_x8_motor.acc_dps;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
// get the encoder value reading
// returns the multiturn encoder value
// parameter int32 (No of pulse)
int32_t rmdx8_pro::get_encoder_raw()
{
    can_tx_buff[0] = GET_ENCODER_CMD;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == GET_ENCODER_CMD)
        {

            rmd_x8_motor.encoder_raw_position = (int32_t)(can1->can_rx_message.data[7] << 24 | can1->can_rx_message.data[6] << 16 | can1->can_rx_message.data[5] << 8 | can1->can_rx_message.data[4]);
            return rmd_x8_motor.encoder_raw_position;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
// get the abs value of the motor
// get the output shaft angle
// parameter motorAngle int32 in 0.01 deg
double rmdx8_pro::get_motor_position()
{
    can_tx_buff[0] = GET_GLOBAL_POS_CMD;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == GET_GLOBAL_POS_CMD)
        {
            uint32_t global_pos = (int32_t)(can1->can_rx_message.data[7] << 24 | can1->can_rx_message.data[6] << 16 | can1->can_rx_message.data[5] << 8 | can1->can_rx_message.data[4]);
            rmd_x8_motor.motor_global_pos = (double)global_pos * 0.01;
            rmd_x8_motor.axis_postion = rmd_x8_motor.motor_global_pos;
            return rmd_x8_motor.motor_global_pos;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
// out put the axis position in deg (same as motor position)
// (-32767 ~ +32767)
double rmdx8_pro::get_axis_position()
{
    return rmd_x8_motor.motor_global_pos;
}

double rmdx8_pro::get_motor_torq()
{
    return rmd_x8_motor.torq_currnet;
}
// return the rpm value of the motor
float rmdx8_pro::get_motor_rpm()
{
    return rmd_x8_motor.motor_rpm * rmd_x8_motor.gear_ratio;
}

// return the rpm value of the output axis
float rmdx8_pro::get_axis_rpm()
{
    return rmd_x8_motor.motor_rpm;
}

int8_t rmdx8_pro::get_motor_temp(){
    return rmd_x8_motor.motor_temp;
}
// change the new motor id of the device for the controller(not the motor driver)
uint8_t rmdx8_pro::set_motor_id(int new_motor_id)
{
    motor_id = new_motor_id;
    can1->add_subscribe_node(0x240 + motor_id);
    can1->add_subscribe_node(0x500 + motor_id);
    return 1;
}
// config the pid value of the motor controller
// set the motor pid once at the time the rest will be
// copied the old value and rewrite it back
// The value will be the 0~256 which is the ratio from 0 ~ max value
uint8_t rmdx8_pro::set_pid(int8_t Kp, int8_t Ki, rmd_pid_mode mode)
{
    get_pid_param();
    can_tx_buff[0] = SET_PID_ROM_CMD;
    can_tx_buff[1] = 0x00;
    if (mode == POSITION_LOOP)
    {
        can_tx_buff[2] = Kp & 0xFF;
        can_tx_buff[3] = Ki & 0xFF;
        can_tx_buff[4] = rmd_x8_motor.V.Kp;
        can_tx_buff[5] = rmd_x8_motor.V.Ki;
        can_tx_buff[6] = rmd_x8_motor.I.Kp;
        can_tx_buff[7] = rmd_x8_motor.I.Ki;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    }
    else if (mode == VELOCITY_LOOP)
    {
        can_tx_buff[2] = rmd_x8_motor.P.Kp;
        can_tx_buff[3] = rmd_x8_motor.P.Ki;
        can_tx_buff[4] = Kp & 0xFF;
        can_tx_buff[5] = Ki & 0xFF;
        can_tx_buff[6] = rmd_x8_motor.I.Kp;
        can_tx_buff[7] = rmd_x8_motor.I.Ki;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    }
    else if (mode == CURRENT_LOOP)
    {
        can_tx_buff[2] = rmd_x8_motor.P.Kp;
        can_tx_buff[3] = rmd_x8_motor.P.Ki;
        can_tx_buff[4] = rmd_x8_motor.V.Kp;
        can_tx_buff[5] = rmd_x8_motor.V.Ki;
        can_tx_buff[6] = Kp & 0xFF;
        can_tx_buff[7] = Ki & 0xFF;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    }
    else
    {
        return 0;
    }
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == GET_PID_CMD)
        {
            rmd_x8_motor.P.Kp = can1->can_rx_message.data[2];
            rmd_x8_motor.P.Ki = can1->can_rx_message.data[3];
            rmd_x8_motor.V.Kp = can1->can_rx_message.data[4];
            rmd_x8_motor.V.Ki = can1->can_rx_message.data[5];
            rmd_x8_motor.I.Kp = can1->can_rx_message.data[6];
            rmd_x8_motor.I.Ki = can1->can_rx_message.data[7];
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
// config the motor acceleration in dps/s (100-60000)
uint8_t rmdx8_pro::set_acc(int32_t acc, rmd_pid_mode mode)
{
    can_tx_buff[0] = SET_ACC_CMD;
    buff_write_zeros(can_tx_buff, 2, 3);
    can_tx_buff[4] = acc & 0xFF;
    can_tx_buff[5] = (acc >> 8) & 0xFF;
    can_tx_buff[6] = (acc >> 16) & 0xFF;
    can_tx_buff[7] = (acc >> 24) & 0xFF;
    if (mode == POSITION_LOOP)
    {
        can_tx_buff[1] = 0x00;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
        can_tx_buff[1] = 0x01;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    }
    else if (mode == VELOCITY_LOOP)
    {
        can_tx_buff[1] = 0x02;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
        can_tx_buff[1] = 0x03;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    }
    else
    {
    }
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == GET_PID_CMD)
        {
            rmd_x8_motor.acc_dps = can1->can_rx_message.data[7] << 24 | can1->can_rx_message.data[6] << 16 | can1->can_rx_message.data[5] << 8 | can1->can_rx_message.data[4];
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
// set the current position as zero
// (the new position needs to be reset the motor controller to confirm)
uint8_t rmdx8_pro::set_curr_pos_zero()
{
    can_tx_buff[0] = SET_CURR_POS_ZERO;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == SET_CURR_POS_ZERO)
        {
            rmd_x8_motor.encoder_offset = (int32_t)(can1->can_rx_message.data[7] << 24 | can1->can_rx_message.data[6] << 16 | can1->can_rx_message.data[5] << 8 | can1->can_rx_message.data[4]);
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

// uint8_t rmdx8_pro::motor_drv_enable()
// {
//     can_tx_buff[0] = MOTOR_ENABLE_CMD;
//     buff_write_zeros(can_tx_buff, 1, 7);
//     can1->can_send_msg(motor_id, can_tx_buff);
//     if (can1->can_read_msg())
//     {
//         if (can1->can_rx_message.data[0] == MOTOR_ENABLE_CMD)
//         {
//             return 1;
//         }
//         else
//         {
//             return 0;
//         }
//     }
//     else
//     {
//         return 0;
//     }
// }

//  disable the motor output, motor will lose power output
uint8_t rmdx8_pro::motor_drv_disable()
{
    can_tx_buff[0] = MOTOR_DISABLE_CMD;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == MOTOR_DISABLE_CMD)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
// stop the motion output of the motor , motor wll lock position
uint8_t rmdx8_pro::motor_drv_stop()
{
    can_tx_buff[0] = MOTOR_STOP_CMD;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == MOTOR_STOP_CMD)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

// all motor output will stop, motor controller will reset
uint8_t rmdx8_pro::motor_drv_reset()
{
    can_tx_buff[0] = MOTOR_DRV_RESET;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    return 1;
}

// config the motor driver control mode
// POSITION_LOOP,
// VELOCITY_LOOP,
// CURRENT_LOOP,
// MOTION_LOOP
uint8_t rmdx8_pro::set_mode(rmd_pid_mode new_mode)
{
    if (new_mode != rmd_x8_motor.motor_mode)
    {
        rmd_x8_motor.motor_mode = new_mode;
        return 1;
    }
    else
    {
        return 0;
    }
}

// config the motor run in torque control cmd
// value in Amp (float32)
uint8_t rmdx8_pro::set_torq(float current)
{
    if (rmd_x8_motor.motor_mode == CURRENT_LOOP)
    {
        int16_t current_value = (int16_t)(current * 100);
        can_tx_buff[0] = TORQ_LOOP_CMD;
        buff_write_zeros(can_tx_buff, 1, 3);
        can_tx_buff[4] = current_value & 0xFF;
        can_tx_buff[5] = (current_value >> 8) & 0xFF;
        buff_write_zeros(can_tx_buff, 6, 7);
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
        return read_motor_state_feedback(CURRENT_LOOP);
    }
    else
    {
        return 0;
    }
}
// config the motor run in velocity control cmd
// value in RPM (int32)
uint8_t rmdx8_pro::set_rpm(int32_t rpm)
{
    if (rmd_x8_motor.motor_mode == VELOCITY_LOOP)
    {
        int32_t rpm_value = (int32_t)(rpm * 100.0 * RPM_2_DPS);
        // printf("rpm: %d\r\n", rpm_value);
        can_tx_buff[0] = VEL_LOOP_CMD;
        buff_write_zeros(can_tx_buff, 1, 3);
        can_tx_buff[4] = (uint8_t)(rpm_value & 0xFF);
        can_tx_buff[5] = (uint8_t)(rpm_value >> 8) & 0xFF;
        can_tx_buff[6] = (uint8_t)(rpm_value >> 16) & 0xFF;
        can_tx_buff[7] = (uint8_t)(rpm_value >> 24) & 0xFF;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
        return read_motor_state_feedback(VELOCITY_LOOP);
    }
    else
    {
        return 0;
    }
}
// config the motor hold its absolute position (single turn)
// value in degree (float32)
uint8_t rmdx8_pro::set_trc_pos(float pos)
{
    if (rmd_x8_motor.motor_mode == POSITION_LOOP)
    {
        int32_t position = (int32_t)(pos * 100.0);
        // printf("rpm: %d\r\n", rpm_value);
        can_tx_buff[0] = TRC_POS_LOOP_CMD;
        buff_write_zeros(can_tx_buff, 1, 3);
        can_tx_buff[4] = (uint8_t)(position & 0xFF);
        can_tx_buff[5] = (uint8_t)(position >> 8) & 0xFF;
        can_tx_buff[6] = (uint8_t)(position >> 16) & 0xFF;
        can_tx_buff[7] = (uint8_t)(position >> 24) & 0xFF;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
        return read_motor_state_feedback(TRC_POS_LOOP_CMD);
    }
    else
    {
        return 0;
    }
}
// config the motor hold its absolute position with speed limitation (multi-turn)
// angle value in degree (float32)
// speed value in degree per second
uint8_t rmdx8_pro::set_abs_pos(float pos, uint16_t speed)
{
    if (rmd_x8_motor.motor_mode == POSITION_LOOP)
    {
        int32_t position_value = (int32_t)(pos * 100);
        // printf("rpm: %d\r\n", rpm_value);
        can_tx_buff[0] = ABS_POS_LOOP_CMD;
        buff_write_zeros(can_tx_buff, 1, 3);
        can_tx_buff[1] = 0x00;
        can_tx_buff[2] = speed & 0XFF;
        can_tx_buff[3] = (speed >> 8) & 0xFF;
        can_tx_buff[4] = position_value & 0xFF;
        can_tx_buff[5] = (position_value >> 8) & 0xFF;
        can_tx_buff[6] = (position_value >> 16) & 0xFF;
        can_tx_buff[7] = (position_value >> 24) & 0xFF;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
        return read_motor_state_feedback(ABS_POS_LOOP_CMD);
    }
    else
    {
        return 0;
    }
}
// config the motor hold its absolute position with speed limitation (single turn)
// angle value in degree (float32)
// speed value in degree per second
uint8_t rmdx8_pro::set_trc_pos_vel(float angle, uint16_t speed)
{
    if (rmd_x8_motor.motor_mode == POSITION_LOOP)
    {
        int32_t position = (int32_t)(angle * 100.0);
        // printf("rpm: %d\r\n", rpm_value);
        can_tx_buff[0] = TRC_POS_LOOP_WITH_VEL_CMD;
        can_tx_buff[1] = 0x00;
        can_tx_buff[2] = speed & 0XFF;
        can_tx_buff[3] = (speed >> 8) & 0xFF;
        can_tx_buff[4] = position & 0xFF;
        can_tx_buff[5] = (position >> 8) & 0xFF;
        can_tx_buff[6] = (position >> 16) & 0xFF;
        can_tx_buff[7] = (position >> 24) & 0xFF;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
        return read_motor_state_feedback(TRC_POS_LOOP_WITH_VEL_CMD);
    }
    else
    {
        return 0;
    }
}
// incremental position pid control
// config the motor hold its incremental position relative to current position with speed limitation (multi-turn)
// angle value in degree (float32)
// speed value in degree per second
uint8_t rmdx8_pro::set_inc_pos(float angle, uint16_t speed)
{
    if (rmd_x8_motor.motor_mode == POSITION_LOOP)
    {
        int32_t position = (int32_t)(angle * 100.0);
        // printf("rpm: %d\r\n", rpm_value);
        can_tx_buff[0] = INC_POS_LOOP_CMD;
        can_tx_buff[1] = 0x00;
        can_tx_buff[2] = speed & 0XFF;
        can_tx_buff[3] = (speed >> 8) & 0xFF;
        can_tx_buff[4] = position & 0xFF;
        can_tx_buff[5] = (position >> 8) & 0xFF;
        can_tx_buff[6] = (position >> 16) & 0xFF;
        can_tx_buff[7] = (position >> 24) & 0xFF;
        can1->can_send_msg(0x140 + motor_id, can_tx_buff);
        return read_motor_state_feedback(INC_POS_LOOP_CMD);
    }
    else
    {
        return 0;
    }
}
// motion control pid cmd
// check documentation page 73
// range p_des -12.5 ~ 12.5 rad
//       v_des -45 ~ 45 rad/s
//       t_ff -24 ~ 24 Nm
//       kp    0  ~ 50
//       kd    0  ~  5
uint8_t rmdx8_pro::set_motion_control(float p_des, float v_des, float t_ff, float kp, float kd)
{
    uint16_t pos = (uint16_t)ceil((p_des + 12.5) * 65536.0 / 25.0);
    uint16_t vel = (uint16_t)ceil((v_des + 45.0) * 4095.0 / 90.0);
    uint16_t Kp = (uint16_t)ceil(kp *  4095.0 / 500.0);
    uint16_t Kd = (uint16_t)ceil(kd * 819.0);
    uint16_t torq = (uint16_t)ceil((t_ff + 24) * 85.3125);
    //printf("p: %2x v: %2x t: %2x kp: %2x kd: %2x \r\n", pos, vel, torq, Kp, Kd);
    if(rmd_x8_motor.motor_mode == MOTION_CONTROL_LOOP)
    {
        can_tx_buff[0] = (pos >> 8) & 0xFF;
        can_tx_buff[1] = pos & 0xFF;
        can_tx_buff[2] = (vel >> 4) & 0XFF;
        can_tx_buff[3] = ((vel << 4) & 0xF0) | ((Kp >> 8) & 0x0F);
        can_tx_buff[4] = Kp & 0xFF;
        can_tx_buff[5] = (Kd >> 4) & 0xFF;
        can_tx_buff[6] = ((Kd << 4) & 0xF0) | ((torq >> 8) & 0x0F);
        can_tx_buff[7] = torq & 0xFF;
        //printf("%2x %2x %2x %2x %2x %2x %2x %2x\r\n", can_tx_buff[0], can_tx_buff[1], can_tx_buff[2],can_tx_buff[3],can_tx_buff[4], can_tx_buff[5], can_tx_buff[6], can_tx_buff[7]);
        can1->can_send_msg(0x400 + motor_id, can_tx_buff);
        if (can1->can_read_msg())
        {
            if ((can1->can_rx_message.data[0]) == motor_id)
            { 
                rmd_x8_motor.motor_global_pos = (((float)(can1->can_rx_message.data[1] << 8 | can1->can_rx_message.data[2]) / 65536) * 25 - 12.5) * RAD_TO_DEG;
                rmd_x8_motor.motor_rpm = ((((float)(can1->can_rx_message.data[3] << 4 | (can1->can_rx_message.data[4] >> 4) & 0xFFF)) / 4095) * 90 - 45.0) * 9.55;
                rmd_x8_motor.torq_currnet = ((float)((can1->can_rx_message.data[4] & 0x0F) << 8 | can1->can_rx_message.data[5]) / 4095) * 48 - 24;
                //printf("p: %.2f v: %.2f t: %.2f\r\n",rmd_x8_motor.p_des, rmd_x8_motor.v_des, rmd_x8_motor.t_ff);
                return 1;
            }
            else
            {
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

uint8_t rmdx8_pro::get_state_msg_1()
{
    can_tx_buff[0] = GET_STATUS_1_CMD;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == GET_STATUS_1_CMD)
        {
            rmd_x8_motor.motor_temp = (int8_t)can1->can_rx_message.data[1];
            rmd_x8_motor.motor_brake = (int8_t)can1->can_rx_message.data[3];
            rmd_x8_motor.input_voltage = (float)((uint16_t)(can1->can_rx_message.data[5] << 8 | can1->can_rx_message.data[4]) * 0.1);
            rmd_x8_motor.error = (uint8_t)((uint16_t)(can1->can_rx_message.data[8] << 8 | can1->can_rx_message.data[7]));
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

uint8_t rmdx8_pro::get_state_msg_2()
{
    can_tx_buff[0] = GET_STATUS_2_CMD;
    buff_write_zeros(can_tx_buff, 1, 7);
    can1->can_send_msg(0x140 + motor_id, can_tx_buff);
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == GET_STATUS_2_CMD)
        {
            rmd_x8_motor.motor_temp = (int8_t)can1->can_rx_message.data[1];
            rmd_x8_motor.torq_currnet = (float)(((int16_t)(can1->can_rx_message.data[3] << 8 | can1->can_rx_message.data[2]))) / 100.0;
            rmd_x8_motor.motor_rpm = (float)(((int16_t)(can1->can_rx_message.data[5] << 8 | can1->can_rx_message.data[4]))) * DPS_2_RPM;
            rmd_x8_motor.motor_global_pos = (double)((int16_t)(can1->can_rx_message.data[7] << 8 | can1->can_rx_message.data[6]));
            //printf("p: %.2f v: %.2f t: %.2f temp: %d\r\n", rmd_x8_motor.motor_global_pos, rmd_x8_motor.motor_rpm, rmd_x8_motor.torq_currnet, rmd_x8_motor.motor_temp);
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
uint8_t rmdx8_pro::get_state_msg_3()
{
    return 0;
}
// read the motor state feedback after a control loop cmd
// * all except motion control loop
//
// updated value:
// temperature in C (int8_t)
// torque current in A (int16_t)
// rpm of the module output in rpm (int16_t)
// angle in deg (int16_t)
uint8_t rmdx8_pro::read_motor_state_feedback(uint8_t cmd)
{
    if (can1->can_read_msg())
    {
        if (can1->can_rx_message.data[0] == cmd)
        {
            rmd_x8_motor.motor_temp = (int8_t)can1->can_rx_message.data[1];
            rmd_x8_motor.torq_currnet = (float)((can1->can_rx_message.data[3] << 8 | can1->can_rx_message.data[2]) / 100.0f);
            rmd_x8_motor.motor_rpm = (float)((can1->can_rx_message.data[5] << 8 | can1->can_rx_message.data[4])) * DPS_2_RPM;
            rmd_x8_motor.encoder_raw_position = (int32_t)(can1->can_rx_message.data[7] << 8 | can1->can_rx_message.data[6]);
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

void rmdx8_pro::buff_write_zeros(uint8_t rx_buff[], uint8_t start, uint8_t end)
{
    for (uint8_t i = start; i <= end; i++)
    {
        rx_buff[i] = 0x00;
    }
}