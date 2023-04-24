#pragma once
#include "Arduino.h"
#include <stdio.h>
#include <stdint.h>

// #include "math_utility.hpp"

#define CMD_ID_HEADER               0x140

#define GET_PID_CMD                 0x30
#define SET_PID_RAM_CMD             0x31
#define SET_PID_ROM_CMD             0x32
#define GET_ACC_CMD                 0x42
#define SET_ACC_CMD                 0x43

#define GET_ENCODER_CMD             0x60
#define GET_ENCODER_RAW             0x61
#define GET_ENCODER_ZERO_OFFSET     0x62
#define SET_ENCODER_POS_AS_ZERO     0x63
#define SET_CURR_POS_ZERO           0x64

#define GET_GLOBAL_POS_CMD          0x92

#define GET_STATUS_1_CMD            0x9A
#define GET_STATUS_2_CMD            0x9C
#define GET_STATUS_3_CMD            0x9D

#define MOTOR_DISABLE_CMD           0x80
#define MOTOR_STOP_CMD              0x81
#define MOTOR_ENABLE_CMD            0x88

#define TORQ_LOOP_CMD               0xA1
#define VEL_LOOP_CMD                0xA2
#define TRC_POS_LOOP_CMD            0xA3
#define ABS_POS_LOOP_CMD            0xA4
#define TRC_POS_LOOP_WITH_VEL_CMD   0xA5
#define INC_POS_LOOP_CMD            0xA6
#define RMD_MOTOR_MOTION_CONTROL    0x400

#define MOTOR_DRV_MODE              0x70
#define MOTOR_DRV_POWER             0x71
#define MOTOR_DRV_RESET             0x76
#define MOTOR_DRV_OP_TIME           0xB1
#define MOTOR_DRV_VER               0xB2
#define MOTOR_DRV_COMM_PROTECT      0xB3
#define MOTOR_DRV_SET_BAUD          0xB4
#define MOTOR_VER_READ_CMD          0xB5
#define MOTOR_DRV_FUC_CMD           0x20
#define MOTOR_DRV_SET_ID            0x79

#define MOTOR_DRV_CRIMP_ENABLE      0x77
#define MOTOR_DRV_CRIMP_disable     0x78

#define POS_LOOP_5_CMD              0xA7
#define POS_LOOP_6_CMD              0xA8

#define RPM_2_DPS 6.0000000000001f
#define DPS_2_RPM 0.16666666666667f

enum rmd_pid_mode
{
    POSITION_LOOP,
    VELOCITY_LOOP,
    CURRENT_LOOP,
    MOTION_CONTROL_LOOP
};

struct rmd_pi
{
    uint8_t Kp = 0;
    uint8_t Ki = 0;
};

struct rmd_v3_motor
{
    rmd_pi P;
    rmd_pi V;
    rmd_pi I;

    float gear_ratio = 9.0;            // motor gear_ratio
    int32_t acc_dps = 0;               // in deg per sec-2  0~60000
    int32_t encoder_raw_position = 0; // encoder raw position 0 ~ 16383
    int32_t encoder_offset = 0;       // encoder offset with true zero 0~16383

    double motor_global_pos = 0; // motor multi-turn angle
    double motor_abs_pos = 0;    // motor single turn 0~360 deg

    int8_t motor_temp = 0;   // in Celeus
    int8_t motor_brake = 1;  // 0 = engaged, 1 = released
    float input_voltage = 0; // in voltage
    uint8_t error = 0;       // error code

    float torq_currnet = 0.0;                                     // torq current of the motor in amp -33~33A
    float motor_rpm = 0;                                          // motor rpm
    double axis_postion = 0.0;                                    // motor output shaft position
    float max_current_value = 2000;                               // maximum current cmd of the motor
    int32_t max_dps = floor(1000 * gear_ratio * RPM_2_DPS * 100); // maximum rpm of the motor

    // motion control cmd feedback
    float p_des = 0.0;
    float v_des = 0.0;
    float t_ff = 0.0;

    rmd_pid_mode motor_mode = VELOCITY_LOOP; // control mode of the motor driver
};

// error code explain:
// 0x0002 motor stall 
// 0x0004 motor under-input-voltage 
// 0x0008 motor over-input-voltage 
// 0x0010 motor phase-over-current
// 0x0040 motor overload 
// 0x0080 input param error 
// 0x0100 motor overspeed 
// 0x1000 motor overtemp
// 0x2000 encoder error
