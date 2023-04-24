#include "canbus_driver_twai.h"

canbus_driver_twai::canbus_driver_twai(int device_id, gpio_num_t can_tx, gpio_num_t can_rx, twai_timing_config_t baudrate, twai_mode_t mode)
{
    canbus_device_id = device_id;

    can_tx_message.identifier = canbus_device_id;
    can_tx_message.data_length_code = canbus_msg_data_len;
    for (int i = 0; i < canbus_msg_data_len; i++)
    {
        can_tx_message.data[i] = 0;
    }
    no_of_subcribe_node = 0;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(can_tx, can_rx, mode);
    twai_timing_config_t t_config = baudrate;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        printf("CANBUS Driver installed\n");
    }
    else
    {
        printf("Failed to install CANBUS driver\n");
    }

    // Start CAN driver
    if (twai_start() == ESP_OK)
    {
        printf("CANBUS Driver started\n");
    }
    else
    {
        printf("Failed to start CANBUS driver\n");
    }
}


void canbus_driver_twai::set_can_device_id(uint32_t device_id)
{
    canbus_device_id = device_id;
}

void canbus_driver_twai::add_subscribe_node(uint32_t id){
    if(no_of_subcribe_node < 30 && no_of_subcribe_node >= 0){
        subscribed_id[no_of_subcribe_node] = id;
        no_of_subcribe_node ++;
    }
}

int8_t canbus_driver_twai::can_send_msg(uint32_t device_id, uint8_t* msg_data)
{
    // printf("len: %d\r\n",sizeof(msg_data));
    can_tx_message.identifier = device_id;
    can_tx_message.data_length_code = 8;
    can_tx_message.extd = false;
    for (int i = 0; i < 8; i++)
    {
        can_tx_message.data[i] = msg_data[i];
    }
    // Queue message for transmission
    if (twai_transmit(&can_tx_message, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
        // printf("Message queued for transmission\n");
        twai_clear_transmit_queue();
        return 1;
    }
    else
    {
        printf("Failed to queue message for transmission\n");
        return 0;
    }
}
int8_t canbus_driver_twai::can_read_msg()
{
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        //printf("Read one msg\r\n");
        if (!(message.rtr))
        {
            // if (check_id(subscribed_id, message.identifier))
            // {
                can_rx_message.identifier = message.identifier;
                can_rx_message.data_length_code = message.data_length_code;
                can_rx_message.extd = message.extd;
                for (int i = 0; i < message.data_length_code; i++)
                {
                    can_rx_message.data[i] = message.data[i];
                }
                twai_clear_receive_queue();
                return 1;
            // }
            // else{
            //     return 0;
            // }
        }
        else
        {
            //printf("Failed to receive CAN message wrong rtr \n"); // debug
            return 0;
        }
    }
    else{
        //printf("Failed to receive CAN message timeout\n");
        return 0;
    }
}

bool canbus_driver_twai::check_id(uint32_t data_frame[], uint32_t test)
{
  for (int x = 0; x < sizeof(data_frame); x++)
  {
    if (data_frame[x] == test)
    {
      return true;
    }    
  } 
  return false;
}