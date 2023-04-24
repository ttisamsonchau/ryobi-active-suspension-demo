#pragma once
#include "share_param.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

static float bus_volt_display = 0.0;
static float bus_connection_display = false;
static float imu_display[3] = {};
static float sus_angle_display[4] = {};
static float sus_angle_setpt_display[4] = {};
static String motor_state_display[] = {"UNK", "RDY", "ACT", "DIS", "CAL", "ER"};
static String sus_motor_state_display[4] = {};

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 oled_1(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C, -1);

// clear existing item in oled
void clear_page()
{
    oled_1.clearDisplay();
    oled_1.setTextSize(1);
    oled_1.setTextColor(WHITE);
    oled_1.setCursor(0, 10);
}

// init the oled interface
void oled_init()
{
    if (!oled_1.begin(SSD1306_SWITCHCAPVCC, OLED_DEV_ADDR))
    { // Address 0x3D for 128x64
        // Serial.println(F("SSD1306 allocation failed"));
        for (;;)
        {
            vTaskDelay(1000);
        }
    }
    clear_page();
}

// update the local data from public data
void oled_display_data_update(){
    lockVariable();
    bus_volt_display = input_bus_voltage;
    bus_connection_display = cmd_bus_connection;

    imu_display[0] = imu.yaw;
    imu_display[1] = imu.pitch;
    imu_display[2] = imu.roll;

    sus_motor_state_display[0] =  motor_state_display[motor_FL.state];
    sus_motor_state_display[1] =  motor_state_display[motor_FR.state];
    sus_motor_state_display[2] =  motor_state_display[motor_RL.state];
    sus_motor_state_display[3] =  motor_state_display[motor_RR.state];

    sus_angle_display[0] = motor_FL.pos;
    sus_angle_display[1] = motor_FR.pos;
    sus_angle_display[2] = motor_RL.pos;
    sus_angle_display[3] = motor_RR.pos;

    sus_angle_setpt_display[0] = motor_FL.pos_setpt;
    sus_angle_setpt_display[1] = motor_FR.pos_setpt;
    sus_angle_setpt_display[2] = motor_RL.pos_setpt;
    sus_angle_setpt_display[3] = motor_RR.pos_setpt;
    unlockVariable();
}

// oled control task
void oled_page_upadate()
{
    clear_page();
    oled_display_data_update();
    oled_1.printf("%.1fV sig: %d\r\n", bus_volt_display, bus_connection_display);
    oled_1.printf("imu: %.1f %.1f %.1f \r\n", imu_display[0], imu_display[1], imu_display[2]);
    oled_1.printf("FL: %s %.1f %.1f\r\n", sus_motor_state_display[0], sus_angle_display[0], sus_angle_setpt_display[0]);
    oled_1.printf("FR: %s %.1f %.1f\r\n", sus_motor_state_display[1], sus_angle_display[1], sus_angle_setpt_display[1]);
    oled_1.printf("RL: %s %.1f %.1f\r\n", sus_motor_state_display[2], sus_angle_display[2], sus_angle_setpt_display[2]);
    oled_1.printf("RR: %s %.1f %.1f\r\n", sus_motor_state_display[3], sus_angle_display[3], sus_angle_setpt_display[3]);

    // show display
    oled_1.display();
}