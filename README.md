| Supported Targets | ESP32-C6 |
| ----------------- | -------- |

# I2C connected SD3031 Real Time Clock with battery backed static memory

Code originally based on the esp - idf "I2C EEPROM Example"(i2c_eeprom) and using the ESP32-C6 as the development microprocessor.

`Note:` The structure of the directories and files have been moved around to better encapsulate a "driver" component project similar to the ones used by Espressif on their [component registry](https://components.espressif.com/). To compile the example code you must be within the "examples/" directory as this is where the main project resides. The component code is in the root directory.

`Warning:` The project includes an ".editorconfig" file containing formatting instructions. This can either be retained or replaced with your own version.

## Overview

The idea behind this code is to be able to write and read information from the SD3031 RTC device using the I2C serial interface. In addition the code for the SD3031 is separated out as a "driver module". This driver may be included in other projects using the standard idf components system.

## How to use this code

### Hardware Required

To run this code, you should have an ESP32-C6<sup>(1) </sup> microprocessor connected to an SD3031 device. The SD3031 device is a precision RTC with an accuracy of ±3.8ppm at 25℃. The device can be found on a breakout board from DFRobot and the [product brief is available](https : //wiki.dfrobot.com/SKU_DFR0998_Fermion_SD3031_RTC_Module_Breakout). I have managed to find a [datasheet](https://jlcpcb.com/partdetail/Wave-SD3031/C2988356) with more complete information about the SD3031 but it will require translation (Google translate does an OK job, but the formatting gets a bit messed up).

#### Configuration parameters:

Unlike the original example code the component code and example program no longer relies on project specific `menuconfig` settings(the example should create a default `sdkconfig` file when the target is set).

To configure the device you will need to change the I2C* #defines and SD3031* #defines within the example/i2c_sd3031_main.c file.

This is to make the project more consistent with how a standard component would be used within your project.

To make the component work with a new project you will need to create/amend an `idf_component.yml` within your project folder(similar to the one within the example/ directory). You will then need to add/modify the `path: "../../"` statement to point to the location you cloned this project into or alternatively you can add/modify the `path:` statement with `path: "./"` and add the following new line `git: "git@github.com:i400s/i2c-sd3031.git"` which will point to this git repository.

### Build and Flash

`Note:` It is assumed you have set up and initialised the esp idf environment and/or vscode environment.

Enter:

`cd [where you cloned the repository] / example / `

`idf.py set - target esp32c6` (replace esp32c6 with your esp32* device).

`idf.py - p PORT flash monitor` to build, flash and monitor the project (replace PORT with the usb port).

(To exit the serial monitor type `Ctrl+]`.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build standard esp-idf projects. While not specifically for this project the steps should be very similar.

`Note:` If you use vscode and are getting warning errors (cannot open source file "xtensa/xtensa_api.h" (dependency of "freertos/FreeRTOS.h") or similar) even if the code compiles correctly and outputs the correct binary code for your device there is a good chance that the c_cpp_properties.json file contains directives pointing towards a different compiler. To resolve the issue press the `esp32*` button on the task bar, select the `example/` directory, and set the target to the device you are using (even if the button already matches your device).

## Example Output

```bash
I (288) i2c-sd3031-main: alive and in app_main()
I (298) gpio: GPIO[6]| InputEn: 1| OutputEn: 1| OpenDrain: 1| Pullup: 0| Pulldown: 0| Intr:0
I (308) gpio: GPIO[7]| InputEn: 1| OutputEn: 1| OpenDrain: 1| Pullup: 0| Pulldown: 0| Intr:0
I (318) i2c-sd3031: adding device to I2C bus at address 0032
I (318) i2c-sd3031: reading from user defined sram
I (328) i2c-sd3031-main: Read buf_in[5] FF, C3, FF, 46, FF
I (328) i2c-sd3031-main: write buf_out[5] FF, D2, FF, 94, FF
I (338) i2c-sd3031: writing to user defined sram
I (348) i2c-sd3031: reading from user defined sram
I (348) i2c-sd3031-main: Read buf_in[5] FF, D2, FF, 94, FF
I (358) i2c-sd3031: getting date/time
I (358) i2c-sd3031-main: date time: 2024/1/11 14:48:35
I (368) i2c-sd3031: setting date/time: 2024/1/11 14:48:31
I (368) i2c-sd3031: calculated day of week: 4
I (378) i2c-sd3031: getting date/time
I (378) i2c-sd3031-main: date time: 2024/1/11 14:48:31
I (388) i2c-sd3031: getting voltage
I (388) i2c-sd3031-main: voltage: 2.98
I (398) i2c-sd3031: getting temperature
I (398) i2c-sd3031-main: temperature: 17
I (10408) i2c-sd3031: getting date/time
I (10408) i2c-sd3031-main: date time: 2024/1/11 14:48:41
I (10408) i2c-sd3031: getting voltage
I (10408) i2c-sd3031-main: voltage: 2.98
I (10408) i2c-sd3031: getting temperature
I (10418) i2c-sd3031-main: temperature: 17
I (20418) i2c-sd3031: getting date/time
I (20418) i2c-sd3031-main: date time: 2024/1/11 14:48:51
I (20418) i2c-sd3031: getting voltage
I (20418) i2c-sd3031-main: voltage: 2.98
I (20418) i2c-sd3031: getting temperature
I (20428) i2c-sd3031-main: temperature: 17
I (30428) i2c-sd3031: getting date/time
I (30428) i2c-sd3031-main: date time: 2024/1/11 14:49:01
I (30428) i2c-sd3031: getting voltage
I (30428) i2c-sd3031-main: voltage: 2.98
I (30428) i2c-sd3031: getting temperature
I (30438) i2c-sd3031-main: temperature: 17
I (40438) i2c-sd3031: removed device from I2C bus at address: 0032
I (40438) main_task: Returned from app_main()

```

## Troubleshooting

(I'll be lucky if this works for myself so...)

## Notes

<sup>(1)</sup> Only tested with ESP32-C6, should work unchanged with other devices.
