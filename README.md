# Ryobi Active Suspension Demo

A testing program of the ryobi gen pi active suspension. 

### Feature
1. control the roll and pitch of the robot
2. control the platform level
3. Auto leveling 

### Hardware Platform
```
PLATFORM: Espressif 32 (6.1.0) > Espressif ESP32 Dev Module
HARDWARE: ESP32 240MHz, 320KB RAM, 4MB Flash
```

### Dependency
build tools: 
```
PACKAGES:
 - framework-arduinoespressif32 @ 3.20007.0 (2.0.7)
 - tool-esptoolpy @ 1.40500.0 (4.5.0)
 - tool-mkfatfs @ 2.0.1
 - tool-mklittlefs @ 1.203.210628 (2.3)
 - tool-mkspiffs @ 2.230.0 (2.30)
 - toolchain-xtensa-esp32 @ 8.4.0+2021r2-patch5
```
library:
```shell
Dependency Graph
|-- Adafruit SSD1306 @ 2.5.7
|   |-- Adafruit GFX Library @ 1.11.5
|   |   |-- Adafruit BusIO @ 1.14.1
|   |   |   |-- Wire @ 2.0.0
|   |   |   |-- SPI @ 2.0.0
|   |   |-- Wire @ 2.0.0
|   |   |-- SPI @ 2.0.0
|   |-- Adafruit BusIO @ 1.14.1
|   |   |-- Wire @ 2.0.0
|   |   |-- SPI @ 2.0.0
|   |-- Wire @ 2.0.0
|   |-- SPI @ 2.0.0
|-- Adafruit GFX Library @ 1.11.5
|   |-- Adafruit BusIO @ 1.14.1
|   |   |-- Wire @ 2.0.0
|   |   |-- SPI @ 2.0.0
|   |-- Wire @ 2.0.0
|   |-- SPI @ 2.0.0
|-- rmdx8_pro_canbus_v3_driver
|   |-- canbus_driver_twai_arduino
|   |-- math_utility
|-- JY901
|   |-- Wire @ 2.0.0
|-- math_utility
|-- canbus_driver_twai_arduino
|-- Wire @ 2.0.0
```

### Chassis Startup 
1. Check the electronics connection is correct and the enable switch & eStop Button is in off state. Plug in the 18v / 40V battery 
2. Turn on the eStop & power up the chassis
3. Check the OLED display, if every joint is in "RDY" you can turn on the motor with the motor enable switch (SW2)
4. If the zero point of each joint is not correct (0 at the lowest point), a calibration is require for the robot
   
### Chassis Calibration
1. Disable the motor (SW2 off) and power up the chassis
2. Put the chassis to the lowest position
3. Press & hold the user button until the blue led turn on and OLED display CAL for every joint
4. Wait until the blue led turn off and OLED shows RDY for each joint
5. Calibration complete
   
### Application notes

1. Parameter adjustment
    
    Modify the param in file ```/src/share_param.h```. Details of the chassis parmeter is willthin the ```/doc/[#WLRP-264] Phase 1 Frame Dimension and Specification.pdf```
