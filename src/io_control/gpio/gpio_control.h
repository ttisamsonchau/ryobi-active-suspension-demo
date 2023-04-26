#pragma once
#include "share_param.h"

// ADC value max min
#define ADC_MIN_VALUE 0
#define ADC_MAX_VALUE 4095

// btn state
static bool user_btn_press = false;
static uint8_t user_btn_counter = 0;
static float x_axis_offset = 0.0;
static float y_axis_offset = 0.0;

// slow speed gpio initalization
void gpio_init()
{
    // pin config
    pinMode(SW_1_ADC_PIN, INPUT);
    pinMode(SW_2_ADC_PIN, INPUT);
    pinMode(SW_3_ADC_PIN, INPUT);
    pinMode(SW_4_ADC_PIN, INPUT);
    pinMode(SW_5_PIN, INPUT);

    pinMode(CURRENT_ADC_PIN, INPUT);
    pinMode(VOLTAGE_ADC_PIN, INPUT);

    pinMode(USER_BTN, INPUT);

    pinMode(USER_LED_1, OUTPUT);
    pinMode(USER_LED_2, OUTPUT);

    // attachInterrupt(USER_BTN, user_btn_isr, FALLING);

    // ADC config
    analogReadResolution(12);
    // analogSetPinAttenuation(SW_1_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(SW_2_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(SW_3_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(SW_4_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(CURRENT_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(VOLTAGE_ADC_PIN, ADC_11db);
}

// user btn state control
void user_btn_control()
{
    if (digitalRead(USER_BTN) == LOW)
    {
        user_btn_counter++;
        user_btn_press = true;
        // Serial.printf("btn cnt %d\r\n", user_btn_counter);
    }
    else
    {
        user_btn_counter = 0;
        user_btn_press = false;
    }

    if (user_btn_counter > 30)
    {
        lockVariable();
        motor_FL.state = CALIBRATING;
        motor_FR.state = CALIBRATING;
        motor_RL.state = CALIBRATING;
        motor_RR.state = CALIBRATING;
        imu.set_zero_flag = true;
        unlockVariable();
        user_btn_counter = 0;
    }

    if (digitalRead(SW_1_ADC_PIN) == HIGH)
    {
        // Serial.printf("SW1_ACTIVE! %d\r\n", user_btn_counter);
        lockVariable();
        if (motor_FL.state == ACTIVE || motor_FR.state == ACTIVE || motor_RL.state == ACTIVE || motor_RR.state == ACTIVE)
        {
            motor_FL.disable_flag = true;
            motor_FR.disable_flag = true;
            motor_RL.disable_flag = true;
            motor_RR.disable_flag = true;

            motor_FL.state = READY;
            motor_FR.state = READY;
            motor_RL.state = READY;
            motor_RR.state = READY;
        }
        unlockVariable();
    }
    else
    {
        lockVariable();
        if (motor_FL.state == READY && motor_FR.state == READY && motor_RL.state == READY && motor_RR.state == READY)
        {
            motor_FL.state = ACTIVE;
            motor_FR.state = ACTIVE;
            motor_RL.state = ACTIVE;
            motor_RR.state = ACTIVE;
        }
        unlockVariable();
    }
}

void gpio_update()
{
    
    user_btn_control();
    // joystick control
    // x_axis_offset = (float)map(analogRead(SW_4_ADC_PIN), ADC_MIN_VALUE, ADC_MAX_VALUE, -MOTOR_ADJ_RANGE, MOTOR_ADJ_RANGE);
    // y_axis_offset = (float)map(analogRead(SW_3_ADC_PIN), ADC_MIN_VALUE, ADC_MAX_VALUE, -MOTOR_ADJ_RANGE, MOTOR_ADJ_RANGE);
    // imu leveling
    y_axis_offset = (float)map(imu.pitch, -15, 15, -MOTOR_ADJ_RANGE, MOTOR_ADJ_RANGE);
    x_axis_offset = (float)map(imu.roll, -15, 15, -MOTOR_ADJ_RANGE, MOTOR_ADJ_RANGE);
    lockVariable();
    // update volatage reading
    input_bus_voltage = (((float)analogReadMilliVolts(VOLTAGE_ADC_PIN)) / 1000 / 0.06976744186) - 0.5;
    if (input_bus_voltage < batt_low_voltage)
    {
        //low bus voltage error
    }

    // update led state
    if (motor_FL.state == ACTIVE || motor_FR.state == ACTIVE || motor_RL.state == ACTIVE || motor_RR.state == ACTIVE)
    {

        digitalWrite(USER_LED_1, HIGH);
    }
    else
    {
        digitalWrite(USER_LED_1, LOW);
    }

    if (motor_FL.state == CALIBRATING || motor_FR.state == CALIBRATING || motor_RL.state == CALIBRATING || motor_RR.state == CALIBRATING)
    {
        digitalWrite(USER_LED_1, HIGH);
    }
    else
    {
        digitalWrite(USER_LED_1, LOW);
    }

    // update suspension position setpoint
    motor_FL.pos_setpt = (float)map(analogRead(SW_2_ADC_PIN), ADC_MIN_VALUE, ADC_MAX_VALUE, 0, 90) * motor_FL.dir + x_axis_offset + y_axis_offset;
    motor_FR.pos_setpt = (float)map(analogRead(SW_2_ADC_PIN), ADC_MIN_VALUE, ADC_MAX_VALUE, 0, 90) * motor_FR.dir - x_axis_offset + y_axis_offset;
    motor_RL.pos_setpt = (float)map(analogRead(SW_2_ADC_PIN), ADC_MIN_VALUE, ADC_MAX_VALUE, 0, 90) * motor_RL.dir + x_axis_offset - y_axis_offset;
    motor_RR.pos_setpt = (float)map(analogRead(SW_2_ADC_PIN), ADC_MIN_VALUE, ADC_MAX_VALUE, 0, 90) * motor_RR.dir - x_axis_offset - y_axis_offset;
    unlockVariable();
}