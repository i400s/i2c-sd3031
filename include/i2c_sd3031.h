/*
 * © 2023 Jonathan Wilson, <i400s@hotmail.com>
 *
 * SD3031 RTC component.
 *
 * This code has been tested, and is known to work, on the esp32-c6 microcontroller.
 *
 * The code should run without modification on other esp32* microcontrollers that
 * provide an I2C interface.
 *
 * For the original and other examples please check:
 * https://github.com/espressif/esp-idf/tree/master/examples
 *
 * See README.md file to get detailed usage of this example.
 *
 * See LICENSE.txt for licensing information.
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 *
 */

#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SD3031 device configuration.
 */
typedef struct {
    i2c_device_config_t i2c_dev_conf;     /*!< I2C device configuration details */
    uint8_t interrupt_gpio;               /*!< GPIO for interrupt line */
} sd3031_device_config_t;

/**
 * @brief SD3031 device object.
 */
typedef struct sd3031_device_t {
    i2c_master_dev_handle_t i2c_dev;      /*!< I2C master device */
    uint16_t device_address;              /*!< Address assigned to sd3031 device */
} sd3031_device_t;

/**
 * @brief Handle of SD3031 device.
 */
typedef struct sd3031_device_t* sd3031_handle_t;

/**
 * @brief SD3031 time structure.
 * @note The day of week is ignored for writes.
*/
typedef struct {
    uint16_t year;           /*!< Year */
    uint8_t  month;          /*!< Month */
    uint8_t  day;            /*!< Day */
    uint8_t  hour;           /*!< Hour */
    uint8_t  minute;         /*!< Minute */
    uint8_t  second;         /*!< Second */
    struct {
        uint8_t hour_24 : 1; /*!< 24 hour flag */
        uint8_t hour_pm : 1; /*!< pm flag */
        uint8_t week    : 3; /*!< day of week (starts on Monday) */
    } flags;                 /*!< Control flags */
} sd3031_time_t;

/**
 * @brief SD3031 date structure.
*/
typedef struct {
    uint16_t year;  /*!< Year */
    uint8_t  month; /*!< Month */
    uint8_t  day;   /*!< Day */
} sd3031_date_t;

/**
 * @brief Add SD3031 device to the i2c bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] device_config SD3031 configuration details.
 * @param[out] handle SD3031 device handle.
 * @return
 *      - ESP_OK: added SD3031 device to the I2C bus successfully.
 *      - ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.
 *      - ESP_ERR_NO_MEM: create I2C bus device failed because of out of memory.
 *
*/
esp_err_t i2c_sd3031_add_device(i2c_master_bus_handle_t bus_handle, sd3031_device_config_t device_config, sd3031_handle_t* handle);

/**
 * @brief Delete SD3031 device from the i2c bus.
 *
 * @param[in] handle SD3031 device handle.
 * @return
 *      - ESP_OK: device is successfully deleted.
*/
esp_err_t i2c_sd3031_del_device(sd3031_handle_t handle);

/**
 * @brief Set time.
 *
 * @param[in] handle SD3031 device handle.
 * @param[in] time Date/time value.
 * @note Date/time is validated for basic correctness.
 * @return
 *      - ESP_OK: time set successfully.
*/
esp_err_t sd3031_time_set(const sd3031_handle_t handle, sd3031_time_t time);

/**
 * @brief Get time.
 *
 * @param[in] handle SD3031 device handle.
 * @param[out] time Time value.
 * @return
 *      - ESP_OK: time read successfully.
*/
esp_err_t sd3031_time_get(const sd3031_handle_t handle, sd3031_time_t *time);

/**
 * @brief Set alarm.
 *
 * @param[in] handle SD3031 device handle.
 * @param[in] time Time value.
 * @return
 *      - ESP_OK: alarm set successfully.
*/
esp_err_t sd3031_alarm_set(const sd3031_handle_t handle, sd3031_time_t time);

/**
 * @brief Temperature of device.
 *
 * @param[in] handle SD3031 device handle.
 * @param[out] temperature Temperature value.
 * @return
 *      - ESP_OK: value retrieved successfully.
*/
esp_err_t sd3031_temperature(const sd3031_handle_t handle, int8_t* temperature);

/**
 * @brief Voltage of battery.
 *
 * @param[in] handle SD3031 device handle.
 * @param[out] voltage Voltage value.
 * @return
 *      - ESP_OK: value retrieved successfully.
*/
esp_err_t sd3031_voltage(const sd3031_handle_t handle, float* voltage);

/**
 * @brief Clear alarm.
 *
 * @param[in] handle SD3031 device handle.
 * @return
 *      - ESP_OK: alarm cleared successfully.
*/
esp_err_t sd3031_clear_alarm(const sd3031_handle_t handle);

/**
 * @brief User SRAM write.
 *
 * @param[in] handle SD3031 device handle.
 * @param[in] offset Offset within the user SRAM space.
 * @param[in] buffer Data buffer to be written.
 * @param[in] size Size of data buffer.
 * @return
 *      - ESP_OK: value stored successfully.
*/
esp_err_t sd3031_sram_write(const sd3031_handle_t handle, uint8_t offset, const uint8_t* buffer, uint8_t size);

/**
 * @brief User SRAM read.
 *
 * @param[in] handle SD3031 device handle.
 * @param[in] offset Offset within the user SRAM space.
 * @param[out] buffer Data buffer to return read data.
 * @param[in] size Size of data buffer.
 * @return
 *      - ESP_OK: value retrieved successfully.
*/
esp_err_t sd3031_sram_read(const sd3031_handle_t handle, uint8_t offset, uint8_t* buffer, uint8_t size);

#ifdef __cplusplus
}
#endif
