#include <mcp2515.h>

spi_device_handle_t spi;

// esp32_HSPI
#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15
#define SPI_CLK 8000000

MCP2515 mcp2515(&spi, GPIO_MISO, GPIO_MOSI, GPIO_SCLK, GPIO_CS, SPI_CLK);
struct can_frame can_tx_msg;
struct can_frame can_rx_msg;

void setup()
{
  can_tx_msg.can_id = 0x0F6;
  can_tx_msg.can_dlc = 8;
  can_tx_msg.data[0] = 0x8E;
  can_tx_msg.data[1] = 0x87;
  can_tx_msg.data[2] = 0x32;
  can_tx_msg.data[3] = 0xFA;
  can_tx_msg.data[4] = 0x26;
  can_tx_msg.data[5] = 0x8E;
  can_tx_msg.data[6] = 0xBE;
  can_tx_msg.data[7] = 0x86;

  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop()
{
  mcp2515.sendMessage(&can_tx_msg);
  if (mcp2515.readMessage(&can_rx_msg) == MCP2515::ERROR_OK)
  {
    Serial.print(can_rx_msg.can_id, HEX); // print ID
    Serial.print(" ");
    Serial.print(can_rx_msg.can_dlc, HEX); // print DLC
    Serial.print(" ");

    for (int i = 0; i < can_rx_msg.can_dlc; i++)
    { // print the data
      Serial.print(can_rx_msg.data[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }
  delay(1000);
  // put your main code here, to run repeatedly:
}