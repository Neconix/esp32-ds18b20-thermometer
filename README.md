# esp32-ds18b20-thermometer
ESP32 DS18B20 digital thermometer driver

# Software requirements
esp-idf v4.4 or later.

# Hardware Required

* An ESP development board. (Tested with ESP32 NodeMCU)
* An DS18B20 thermometer (sensor).
* An 4.7 KOhm resistor
* An USB cable for power supply and programming.

## Hardware Connection

The connection between ESP Board and the DS18B20 sensor is as follows:

```
      ESP Board                      DS18B20 Digital thermometer
      +---------+              +------+
      |         |              |      |
      |     3V3 +----+---------+ VDD  |
      |         |    |         |      |
      |         |    |         |      |
      |         |    R 4.7KOhm |      |
      |         |    |         |      |
      |      D4 +----+---------+ DQ   |
      |         |              |      |
      |     GND +--------------+ GND  |
      |         |              |      |
      |         |              |      |
      |         |              |      |
      +---------+              +------+
```

The GPIO number used by this example can be changed in [main.c](main/main.c), where:

| GPIO number              | DS18B20 pin |
| ------------------------ | ----------- |
| CONFIG_DS18B20_GPIO      | DQ          |


# Build and flash

## ESP-IDF from command line

```shell
git clone https://github.com/Neconix/esp32-ds18b20-thermometer.git
cd esp32-ds18b20-thermometer
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB0 flash   # where /dev/ttyUSB0 is a port to connected ESP32 board
idf.py -p /dev/ttyUSB0 monitor # used to see output from ESP_LOG
```
With some Linux distributions, you may get the `Failed to open port /dev/ttyUSB0` error message when flashing the ESP32. Run something like this to add current user to `dialout` group:

```shell
sudo usermod -a -G dialout $USER
```
## VSCode

In VSCode with installed [ESP-IDF extension](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md):

- git clone https://github.com/Neconix/esp32-st7789-thermometer.git
- open project folder in VSCode and run from command pallete (`Ctrl + Shift + P`):
- ESP-IDF: Add vscode configuration folder
- ESP-IDF: SDK configuration editor (menuconfig)
- ESP-IDF: Select port to use
- ESP-IDF: Set espressif device target
- ESP-IDF: Build your project
- ESP-IDF: Flash (UART) your project
- ESP-IDF: Monitor your device

Or run ESP-IDF: Build, Flash and start a monitor on your device.

# Supported functions

```C
bool sensorInit(sensor_t *sensor, sensor_config_t *sensorConfig);
bool sensorGetPresense(sensor_t *sensor);
bool sensorGetId(sensor_t *sensor, uint8_t *id);
bool sensorGetScratchpad(sensor_t *sensor, scratchpad_t *scratchpad);
bool sensorGetTempSync(sensor_t *sensor, float *temp);
bool sensorSetConfig(sensor_t *sensor, sensor_resolution_t resolution, uint8_t tempHigh, uint8_t tempLow);
```

See [ds18c20.h](main/ds18c20.h) and [st7789.c](main/ds18c20.c)   

# Docs
esp-idf: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/
