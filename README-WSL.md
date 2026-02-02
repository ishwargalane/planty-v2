# Multi Device Example

## Enabling USB Device Access in Dev Container

To upload firmware (e.g., to ESP32) via USB in the dev container on Windows:

1. **Install usbipd-win on Windows**:
   - Download from [GitHub releases](https://github.com/dorssel/usbipd-win/releases).
   - Run as administrator.

2. **Attach USB device to WSL2**:
   - Open PowerShell as admin.
   - List devices: `usbipd list`.
   - Attach (replace `<BUSID>`): `usbipd attach --wsl --busid <BUSID>`.
   - Verify in WSL2: `ls /dev/tty*` or `lsusb`.

## Install ESP-IDF and ESP RainMaker on WSL2 manually
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
python --version

mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git

cd ~/esp/esp-idf
./install.sh esp32
. ./export.sh

### set alias 
alias get_idf='. $HOME/esp/esp-idf/export.sh'

## in project directory
idf.py set-target esp32
idf.py menuconfig

idf.py build
ls /dev/tty* # get the port for USB
idf.py -p PORT flash
idf.py -p <PORT> monitor

### Or combile build flash monitor
idf.py -p PORT flash monitor


## Build and Flash firmware

Follow the ESP RainMaker Documentation [Get Started](https://rainmaker.espressif.com/docs/get-started.html) section to build and flash this firmware.

## What to expect in this example?

- This example just demonstrates how you can have multiple devices on the same board.
- It has 4 devices
    - Switch
    - Lightbulb
    - Fan
    - Temperature Sensor
- It uses the BOOT button and RGB LED on the ESP32-S2-Saola-1/ESP32-C3-DevKitC board to demonstrate a switch, and the esp timer to demonstrate the temperature sensor.
- The LED state (green color) indicates the state of the switch.
- Pressing the BOOT button will toggle the state of the switch and hence the LED. This will also reflect on the phone app.
- Toggling the button on the phone app should toggle the LED on your board, and also print messages like these on the ESP32-S2 monitor:

```
I (16073) app_main: Received value = true for Switch - power
```

- The temperature value is changed by 0.5 every minute.
- It starts at some default value (25.0) and goes on increasing till 99.5. Then it starts reducing till it comes to 0.5. The cycle keeps repeating.
- You can check the temperature changes in the phone app.
- Lightbulb and Fan are dummy devices, but you can try setting the values from the phone app and see them reflect on the ESP32-S2 monitor.

### LED not working?

The ESP32-S2-Saola-1 board has the RGB LED connected to GPIO 18. However, a few earlier boards may have it on GPIO 17. Please use `CONFIG_WS2812_LED_GPIO` to set the appropriate value.

### Reset to Factory

Press and hold the BOOT button for more than 3 seconds to reset the board to factory defaults. You will have to provision the board again to use it.

---

## Developer References

This project uses ESP-IDF and ESP RainMaker. Always consult the following resources when implementing features:

### ESP RainMaker Documentation
- **Main Documentation**: https://docs.rainmaker.espressif.com/
- **Technical Overview**: https://docs.rainmaker.espressif.com/docs/product_overview/technical_overview/introduction
- **API Reference**: https://docs.rainmaker.espressif.com/docs/apis/
- **Firmware Developer Guide**: https://evaluation.rainmaker.espressif.com/firmware-developer-guide
- **Device API**: https://docs.rainmaker.espressif.com/docs/apis/device-api
- **Node API**: https://docs.rainmaker.espressif.com/docs/apis/node-api
- **Parameter API**: https://docs.rainmaker.espressif.com/docs/apis/parameter-api

### ESP-IDF Documentation
- **ESP-IDF v5.0 API Reference**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/
- **I2C Driver API**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
- **NVS Storage**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html
- **WiFi API**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
- **FreeRTOS**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html
- **Logging System**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html

### Component Libraries & GitHub Repositories
- **ESP RainMaker GitHub**: https://github.com/espressif/esp-rainmaker
- **ESP RainMaker Examples**: https://github.com/espressif/esp-rainmaker/tree/master/examples
- **esp-idf-lib** (ADS111x driver): https://github.com/UncleRus/esp-idf-lib
- **ADS111x I2C Component**: https://github.com/UncleRus/esp-idf-lib/tree/master/components/ads111x

### Community & Support
- **ESP32 Forum**: https://esp32.com/
- **ESP-IDF GitHub Issues**: https://github.com/espressif/esp-idf/issues
- **ESP RainMaker Forum**: https://rainmaker.espressif.com/forum/

### Project-Specific Notes
- See [.github/copilot-instructions.md](.github/copilot-instructions.md) for detailed coding conventions and patterns
- Hardware configuration in [main/sensor_config.h](main/sensor_config.h)
- Code patterns and examples in [.vscode/copilot-patterns.md](.vscode/copilot-patterns.md)

