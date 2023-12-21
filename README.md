| Supported Targets | ESP32-C6 |
| ----------------- | -------- |

# I2C connected SD3031 Real Time Clock with battery backed static memory

Code originally based on the "I2C Simple Example" (i2c_simple) and using the esp32-c6 as the development microprocessor.

`Note:` The structure of the directories and files have been moved around to better encapsulate a "driver" project similar to the ones used by Espressif on their [component registry](https://components.espressif.com/). To compile this code you must be within the "test_apps/" directory as this is where the main project resides. The component code is in the project root directory.

`Warning:` This project includes my version of the ".vscode" directory. You will need to delete this and replace it with the esp-idf auto generated version for your system. The project also includes an ".editorconfig" file containing formatting instructions. This can either be retained or replaced with your own version.

## Overview

The idea behind this code is to be able to write and read information from the SD3031 RTC device using the I2C serial interface. In addition the code for the SD3031 is separated out as a "driver module". This driver may be included in other projects using the standard idf components system.

## How to use this code

### Hardware Required

To run this code, you should have an ESP32-C6<sup>(1)</sup> microprocessor connected to an SD3031 device. The SD3031 device is a precision RTC with an accuracy of ±3.8ppm at 25℃. The device can be found on the breakout board from DFRobot and the [product brief is available](https://wiki.dfrobot.com/SKU_DFR0998_Fermion_SD3031_RTC_Module_Breakout). I have managed to find a [datasheet](https://jlcpcb.com/partdetail/Wave-SD3031/C2988356) but it will require translation.

#### Configuration parameters:

The code no longer relies on `menuconfig`.

To configure the device you will need to change the I2C... and SD3031... #defines within the i2c_sd3031_main.c file.

This is to make the project more consistent with a component device.

`Note:` If you use vscode and are getting warning errors (cannot open source file "xtensa/xtensa_api.h" (dependency of "freertos/FreeRTOS.h") or similar) even if the code compiles correctly and outputs the correct binary code for your device there is a good chance that the c_cpp_properties.json file contains directives pointing towards a different compiler. To resolve the issue press the "esp32c6" button on the task bar, select the "test_apps" directory, and set the target to the device you are using.

### Build and Flash

`Note:` It is assumed you have set up and initialised the esp idf environment.

Enter:

`cd [where you cloned the repository]/test_apps/`

`idf.py set-target esp32c6` (replace esp32c6 with esp32 device).

`idf.py -p PORT flash monitor` to build, flash and monitor the project (replace PORT with the usb port).

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build standard esp-idf projects. While not specifically for this project the steps should be very similar.

## Example Output

```bash
I (...)
I (...)
I (...)

```

## Troubleshooting

(I'll be lucky if this works for myself so...)

## Notes

<sup>(1)</sup> Only tested with ESP32-C6, may work unchanged with other devices.
