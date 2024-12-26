# Cloud Based Tracking System for Mobile Units

This project enables mobile units to collect data such as location, altitude, temperature, and acceleration, save it to an SD card, and upload it to the Firebase cloud platform using Wi-Fi.

## Features
- **Sensor Support**: MPU6050, BMP280, PTC thermistor, and Neo M8N GPS module.
- **Location Tracking**: Precise location data is captured using the Neo M8N GPS module and sent to Firebase.
- **Data Logging**: Collected data is stored locally on an SD card.
- **Wireless Communication**: ESP32 sends data to Firebase cloud using the HTTP protocol.
- **Power Management**: The system is powered by a 7.4V LiPo battery and features an optimized power regulation circuit.

## Hardware Components
- **Main Processor**: STM32F103C8T6
- **Wireless Module**: ESP32 (Compatible with version 2.0.5)
- **Sensors**:
  - MPU6050 (IMU)
  - BMP280 (Pressure and Temperature)
  - Neo M8N GPS module
  - PTC thermistor (Analog temperature sensing)
- **Storage**: SD card module
- **Other Components**: 7.4V LiPo battery, regulator, and various passive components.

## Software Requirements
- **Arduino IDE**: For ESP32 programming.
- **STM32CubeIDE**: For STM32 programming.
- **Libraries**:
  - [lwgps](https://github.com/MaJerle/lwgps) library for GPS data processing.
  - `Firebase ESP32` library for Firebase communication.
    > **Note**: You must manually add `lwgps` and `Firebase ESP32` libraries to the project.

## Setup
1. **ESP32 Configuration**:
   - Install version 2.0.5 of ESP32 in the Arduino IDE.
   - Follow the [ESP32 setup guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/).
2. **STM32 Programming**:
   - Open the project files in STM32CubeIDE and upload the code to the STM32 processor.
3. **Add Required Libraries**:
   - Download and add the `lwgps` and `Firebase ESP32` libraries to your project.
   - Install other required libraries (e.g., WiFi, HTTPClient) in Arduino IDE.
4. **Firebase Configuration**:
   - Create a Firebase account and project.
   - Add your API key and database URL to the ESP32 code.
5. **Connect Hardware**:
   - Assemble the components on the PCB.
   - Connect the 7.4V LiPo battery to power the system.

## How It Works
1. **Data Collection**:
   - The Neo M8N GPS module collects location and altitude data.
   - MPU6050 and BMP280 sensors gather temperature, acceleration, and pressure data.
   - The STM32 processor processes all data and saves it to an SD card.
2. **Data Transmission to Firebase**:
   - The ESP32 connects to a Wi-Fi network and communicates with Firebase.
   - Processed GPS and sensor data are uploaded to the Firebase database.
3. **Cloud and Local Storage**:
   - Data can be accessed in real time from the Firebase cloud.
   - All data is also backed up locally on the SD card.

## PCB Image

![pcb img v2](https://github.com/user-attachments/assets/b03a2557-d033-4c7a-805d-287bfa9ae8c4)

## Contributing

Contributions to the project are welcome! Feel free to open an issue or submit a pull request with your ideas and improvements.
