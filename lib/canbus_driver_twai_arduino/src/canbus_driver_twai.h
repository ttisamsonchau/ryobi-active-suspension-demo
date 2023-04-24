#pragma once
#include "driver/gpio.h"
#include "driver/twai.h"
class canbus_driver_twai
{
public:
    canbus_driver_twai(int device_id, gpio_num_t can_tx, gpio_num_t can_rx, twai_timing_config_t baudrate, twai_mode_t mode);
    int8_t can_send_msg(uint32_t id, uint8_t data[]);
    int8_t can_read_msg();

    void set_can_device_id(uint32_t id);
    void add_subscribe_node(uint32_t id);
    twai_message_t can_tx_message;
    twai_message_t can_rx_message;

private:
    uint32_t subscribed_id[30];
    uint8_t no_of_subcribe_node = 0;
    int8_t canbus_msg_data_len = 8;
    int canbus_device_id = 0;
    bool check_id(uint32_t data_frame[], uint32_t test);
};