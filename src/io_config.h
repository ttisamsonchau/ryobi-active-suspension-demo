#pragma once
#include <Arduino.h>
#include "math_utility.hpp"
#include <Wire.h>
#include <canbus_driver_twai.h>
// #include <mcp2515.h>

// ADC pin
#define SW_1_ADC_PIN 34     // empty
#define SW_2_ADC_PIN 35     // rotary resistor
#define SW_3_ADC_PIN 13     // joystick y
#define SW_4_ADC_PIN 12     // joystick x
#define SW_5_PIN     14     // joystick sw
#define CURRENT_ADC_PIN 25  
#define VOLTAGE_ADC_PIN 26

// user button and led
#define USER_BTN 18
#define USER_LED_1 2
#define USER_LED_2 19

// canbus 2
const gpio_num_t can_2_tx = GPIO_NUM_5;
const gpio_num_t can_2_rx = GPIO_NUM_4;
canbus_driver_twai can2(0x00, can_2_tx, can_2_rx, TWAI_TIMING_CONFIG_1MBITS(), TWAI_MODE_NORMAL);


// I2C config
#define OLED_DEV_ADDR 0x3C
#define OLED_ENABLE true
#define I2C_CLK 400000
#define I2C_SDA 33
#define I2C_SCL 32
TwoWire I2C = TwoWire(0);

void I2C_init(){
    I2C.begin(I2C_SDA, I2C_SCL, I2C_CLK);
}


// esp32_HSPI
// spi_device_handle_t spi;
// #define GPIO_MOSI 13
// #define GPIO_MISO 12
// #define GPIO_SCLK 14
// #define GPIO_CS 15
// #define SPI_CLK 8000000
// MCP2515 mcp2515(&spi, GPIO_MISO, GPIO_MOSI, GPIO_SCLK, GPIO_CS, SPI_CLK);

// struct can_frame can1_tx_msg;
// struct can_frame can1_rx_msg;

// static long action_packet_stamp = 0;
// static long cmd_last_time, cmd_curr_time = 0;

// cmd bus init function
// void can_bus_init()
// {
//     mcp2515.reset();
//     mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); // clock rate should be adjust according to HW
//     mcp2515.setNormalMode();
//     can1_tx_msg.can_dlc = 8;
// }


















