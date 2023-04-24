# canbus_driver_twai_arduino
An arduino implementation of the can bus twai driver

## Compactible MCU
ESP32 series:
  - ESP32 WROOM 32E/U/UE/D
  - ESP32 C3

## Hardware connection
Compactible Can tranceiver:
* SN65HVD23x
* TJA105x
* MCP2551

| function | pin number |
| -- | -- |
| CTX | GPIO 5 |
| CRX | GPIO 6 | 

## Support baudrate
* 1Mbps
* 500kbps
* 250kbps
* 125kbps
