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

3. **Edit .devcontainer/devcontainer.json**:
   - Add mounts for USB access (after `runArgs`):
     ```json
     "mounts": ["source=/dev/bus/usb,target=/dev/bus/usb,type=bind"]
     ```
     - For security, use device-specific if known: `"source=/dev/ttyUSB0,target=/dev/ttyUSB0,type=bind"`.

4. **Rebuild the dev container**:
   - In VS Code, Command Palette: "Dev Containers: Rebuild Container".

5. **Verify in container**:
   - Open terminal: `ls /dev/tty*` or `lsusb`.
   - Test upload: `idf.py flash` or `esptool.py`.

**Notes**: Ensure WSL2 backend in Docker Desktop. Reattach device if disconnected. For issues, check udev or restart WSL2.


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

