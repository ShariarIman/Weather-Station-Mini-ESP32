# MicroPython IoT Weather Station

A MicroPython-based weather station for ESP32 or similar microcontrollers. The project collects local environmental data using a BME280 sensor, monitors battery voltage, fetches weather data from Open-Meteo, uploads data to ThingSpeak, and reads wind speed via Modbus RTU. Data is displayed on an SH1106 OLED display, with button interactions for navigation and recalibration.

## Features
- **Sensors**: BME280 for temperature, humidity, pressure, and altitude.
- **Battery Monitoring**: Measures battery voltage and estimates percentage for a 2000mAh LiPo/18650 battery.
- **Display**: SH1106 OLED (128x64) for showing local sensor data, online weather, and Modbus data.
- **Connectivity**: Wi-Fi for fetching weather from Open-Meteo and uploading to ThingSpeak.
- **Modbus RTU**: Reads wind speed and sensor status from a Modbus device.
- **Power Management**: Deep sleep mode to save battery, with wake on button press.
- **User Interaction**: Button for switching between display pages and recalibrating sensors.

## Hardware Requirements
- ESP32 or compatible MicroPython-supported microcontroller.
- BME280 sensor (I2C, address 0x76).
- SH1106 OLED display (128x64, SPI).
- Push button (GPIO 14).
- RS485 to TTL Module Suitable for UART Serial Port
- Modbus RTU device (connected via UART, pins 16/17).
- Voltage divider (100kΩ resistors) for battery voltage measurement (GPIO 34).
- Wi-Fi access for internet connectivity.

## Pin Configuration
- **I2C (BME280)**: SCL=GPIO 19, SDA=GPIO 21.
- **SPI (SH1106)**: SCK=GPIO 18, MOSI=GPIO 23, DC=GPIO 12, RST=GPIO 13, CS=GPIO 5.
- **Button**: GPIO 14 (pull-up, active low).
- **Modbus UART**: TX=GPIO 16, RX=GPIO 17.
- **Battery ADC**: GPIO 34.

## Software Requirements
- MicroPython firmware for ESP32 (tested with v1.20 or later).
- Python libraries (included or external):
  - `sh1106.py` (included, for OLED display).
  - `bme280.py` (included, for BME280 sensor).
  - `umodbus` (external, for Modbus RTU; see [umodbus GitHub](https://github.com/brainelectronics/micropython-modbus)).
  - Standard MicroPython libraries: `machine`, `network`, `urequests`, `ujson`, `ntptime`.

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/ShariarIman/Weather-Station-Mini-ESP32.git
   cd weather-station
