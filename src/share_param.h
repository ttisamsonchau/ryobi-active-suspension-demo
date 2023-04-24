#pragma once
#include "task_control.h"
#include "io_config.h"

// input bus parameter
volatile float input_bus_voltage = 0.0;

// cmd bus validation variable
volatile bool cmd_bus_connection = false;

// volatge level
#define batt_low_voltage 34.0
#define batt_high_voltage 42.0
#define OVERCURRENT_THRESHOLD 10.0 // overcurrent protection threshold in Amp
#define OVERTEMP_THRESHOLD 60.0    // motor overtemperature protection threshold in Amp

// Motor Device ID
#define SUS_MOTOR_FL_ID 1
#define SUS_MOTOR_FR_ID 2
#define SUS_MOTOR_RL_ID 3
#define SUS_MOTOR_RR_ID 4

// motor status parameters
enum motor_status
{
    UNKNOWN,     // motor status unknown, debug operation is required
    READY,       // motor idle mode (no output), passed calibration, not receive any cmd
    ACTIVE,      // motor under actruation
    DISCONNECT,  // motor is not detected
    CALIBRATING, // motor calibration mode, the motor is under calibration
    ERROR,       // motor controller report error
};

// motor error code
enum motor_error_code
{
    NORMAL,         // operation normal
    OVERTEMP,       // overtemperature protection triggered
    OVERCURRENT,    // overcurrent protection triggered
    UNDERVOLTAGE,   // undervoltage protection triggered
    COMM_BUS_ERROR, // communication bus lose packet
    DRIVER_ERROR,   // local driver error
    FEEDBACK_ERROR, // motor feedback value error
};

// motor parameter
struct suspension_motor
{
    motor_status state = UNKNOWN;       // motor state
    motor_error_code err_code = NORMAL; // motor error code
    float pos_setpt;                    // motor position cmd setpt in deg
    float pos;                          // motor current position (output shaft) in deg
    float rpm;                          // motor current rpm (output shaft) in rpm
    float current;                      // motor current torq (torq current) in A
    int8_t temp;                        // motor current temperature in deg Cel
    float pos_upper_limit;              // motor maximum position allowed in deg
    float pos_lower_limit;              // motor minimum position allowed in deg
    float Kp;                           // motor pid Kp value
    float Kd;                           // motor pid Kd value
    int8_t dir = 0;                     // direction of the motor run
    int8_t cmd_miss_cnt = 0;            // count of the unsucessful exceute cmd
    bool disable_flag = false;           // flag that will case the driver disable in next cycle
};
suspension_motor motor_FL;
suspension_motor motor_FR;
suspension_motor motor_RL;
suspension_motor motor_RR;

// imu parameters
struct WITMOTION_IMU
{
    float gx = 0.0;
    float gy = 0.0;
    float gz = 0.0;
    float ax = 0.0;
    float ay = 0.0;
    float az = 0.0;
    float mx = 0.0;
    float my = 0.0;
    float mz = 0.0;
    float yaw = 0.0;
    float pitch = 0.0;
    float roll = 0.0;
    float pressure = 0.0;
    float altitude = 0.0;
    float temp = 0.0;
};
WITMOTION_IMU imu;



