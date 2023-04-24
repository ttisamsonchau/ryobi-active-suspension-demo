#include <Arduino.h>
#include <canbus_driver_twai.h>

int can1_device_id = 0x54;

const gpio_num_t can_1_tx = GPIO_NUM_5;
const gpio_num_t can_1_rx = GPIO_NUM_4;

canbus_driver_twai can1 (can1_device_id, can_1_tx, can_1_rx, TWAI_TIMING_CONFIG_1MBITS(), TWAI_MODE_NORMAL);

void canbus1_control(void *pvParameters){
  (void)pvParameters;
  uint8_t msg_len = 8;
  uint8_t data[msg_len];
  uint8_t counter = 0;
  can1.add_subscribe_node(can1_device_id);
  while(1){
    for(int i=0; i<= msg_len; i++){
      data[i] = counter + i;
    }
    if(can1.can_send_msg(can1_device_id, data)){
      Serial.printf("can1 sent msg\r\n");
    }
    counter ++;
    vTaskDelay(1); // for msg transfer via quenue and bus
    if(can1.can_read_msg()){
      for (int i = 0; i < can1.can_rx_message.data_length_code; i++) {
        Serial.printf("Data byte %d = %d\n", i, can1.can_rx_message.data[i]);
      }
    }
    vTaskDelay(1);
  }
}


void setup() {
  Serial.begin(115200);
  xTaskCreatePinnedToCore(canbus1_control,
                            "canbus1_control",
                            2048, // usStackDepth
                            NULL, // pvParameters
                            1,    // uxPriority // 1 for equal priority
                            NULL, // pvCreatedTask
                            0);   // xCoreID // 0 or 1 //tskNO_AFFINITY
  Serial.println("Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
