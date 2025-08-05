# trolley-testing

**Version:** 0.0.0 

## Overview

This repository contains the testing firmware for the Everzip AlphaTrolley. This software will be responsible for: 
* Testing LiDAR capabilities
* Testing RFID cpabilities
* Testing motor/controller capabilities
* Testing LoRa communication


## Hardware 

* **Main Board:** Arduinio [Giga R1 Wifi](https://store.arduino.cc/products/giga-r1-wifi)
* **Motor Controller:** VESC based controller [e.g., [Flipsky 75100 V2 Pro](https://flipsky.net/collections/v75-series/products/75100-pro-v2-0-with-aluminum-pcb)]
* **LiDAR Sensor:** TFmini plus [Micro LiDAR Module](https://www.sparkfun.com/tfmini-plus-micro-lidar-module.html) 
* **RFID Peripheral:** Hecto M7E [Sparkfun Simltaneous RFID Reader](https://www.sparkfun.com/sparkfun-simultaneous-rfid-reader-m7e-hecto.html)
* **Ultrasonic Sensor** MaxBotix [MB7070 XL-MaxSonar-WRA](https://maxbotix.com/products/mb7070?variant=48771229581599)
* **LoRAWAN Transceiver** MultiTEch [mDOT 915](https://multitech.com/product/mtdot-programmable-rf-modules-915-mhz-us-2/)

### Pinout

| Pin       | Function   | Notes                             |
| --------- | ---------- | --------------------------------- |
| D0 (RX0)  | Serial1 RX | For VESC communication            |
| D1 (TX0)  | Serial1 TX | For VESC communication            |
| D19 (RX1) | Serial2 RX | For front LiDAR communication     |
| D18 (TX1) | Serial2 TX | For front LiDAR communication     |
| 17 (RX2)  | Serial3 RX | For rear LiDAR communication      |
| 16 (TX2)  | Serial3 TX | For rear LiDAR communication      |
| 15 (RX3)  | Serial4 RX | For RFID communication            |
| 14 (TX3)  | Serial4 TX | For RFID communication            |
| TBD       | ADC        | Analog input for front ultrasonic |
| TBD       | ADC        | Analog input for rear ultrasonic  |

## Toolchain & Dependencies

This project is built using the [PlatformIO](https://platformio.org/) IDE for VS Code.

1.  **VS Code:** Install from [here](https://code.visualstudio.com/).
2.  **PlatformIO IDE Extension:** Install from the VS Code Marketplace.


All library dependencies are managed automatically by PlatformIO via the `platformio.ini` file. Key libraries include:
* Custom libraries located in /lib folder
  - HectoRFID (RFID reader library)
  - TFminiLidar (LiDAR distance library)
  - VescUart (Motor controller library)
  - TrolleyFunctions (Funtions library)

## Building and Flashing

### Building the Firmware

1.  Open the project folder in VS Code.
2.  Click the **PlatformIO icon** on the left-hand sidebar.
3.  Choose the environment for the test you want to build (e.g arduino_back_forth, arduino_lidar_test, etc) and click **build**


### Flashing the Firmware (Upload)

1.  Connect the device to your computer via USB.
2.  Ensure the correct port is selected in PlatformIO.
3.  Under the project's environment, for the test you want to build (e.g arduino_back_forth, arduino_lidar_test, etc) click **upload**


## Configuration

1. Key operational parameters can be configured in the `src/config.h` file. This includes things like motor acceleration ramps, PID constants, and UART baud rates.
2. For motor driving tests, number of reps and delay between start button and test execution can be adjusted in `platformi.ini` file.

## License

Copyright © 2025 ClimbWorks Inc. All Rights Reserved. This software is proprietary and confidential.