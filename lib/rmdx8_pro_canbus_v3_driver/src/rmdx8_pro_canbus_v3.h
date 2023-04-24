#pragma once
#include "canbus_driver_twai.h"
#include "rmd_v3_canbus_id.h"
#include "math_utility.hpp"

class rmdx8_pro
{
public:
    rmdx8_pro(canbus_driver_twai *_can, int set_motor_id);

    uint8_t get_pid_param();
    int32_t get_acc();
    int32_t get_encoder_raw();
    double get_motor_position();
    double get_axis_position();
    double get_motor_torq();
    float get_motor_rpm();
    float get_axis_rpm();
    int8_t get_motor_temp();

    uint8_t set_motor_id(int new_motor_id);
    uint8_t set_pid(int8_t Kp, int8_t Ki, rmd_pid_mode mode);
    uint8_t set_acc(int32_t acc, rmd_pid_mode mode);
    uint8_t set_curr_pos_zero();

    // uint8_t motor_drv_enable();
    uint8_t motor_drv_disable();
    uint8_t motor_drv_stop();
    uint8_t motor_drv_reset();

    uint8_t set_mode(rmd_pid_mode new_mode);                                              // motor driver mode config
    uint8_t set_torq(float current);                                                      // motor torq pid control
    uint8_t set_rpm(int32_t rpm);                                                         // motor rpm pid control
    uint8_t set_trc_pos(float pos);                                                       // global position tracking position pid control
    uint8_t set_abs_pos(float pos, uint16_t speed);                                       // global absulute position control
    uint8_t set_trc_pos_vel(float angle, uint16_t speed);                                 // global position tracking position pid control with velocity limit
    uint8_t set_inc_pos(float angle, uint16_t speed);                                     // incremental position pid control
    uint8_t set_motion_control(float p_des, float v_des, float t_ff, float kp, float kd); // motion control pid cmd

    uint8_t get_state_msg_1();
    uint8_t get_state_msg_2();
    uint8_t get_state_msg_3();

   

private:
    int motor_id;

    uint8_t read_motor_state_feedback(uint8_t cmd);

    void buff_write_zeros(uint8_t rx_buff[], uint8_t start, uint8_t end);

    uint8_t can_tx_buff[8];
    uint8_t can_rx_buff[8];

    rmd_v3_motor rmd_x8_motor;
    canbus_driver_twai *can1;
};