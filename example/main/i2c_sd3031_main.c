/*
 * © 2023 Jonathan Wilson, <i400s@hotmail.com>
 *
 * SD3031 RTC example program.
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

#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "bootloader_random.h"
#include "esp_random.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_sd3031.h"

static const char *TAG = "i2c-sd3031-main";

/**
 * @brief I2C Configuration options.
 */
#define I2C_MASTER_SCL                7      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA                6      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_PORT               0      /*!< I2C port number */
#define I2C_MASTER_FREQ_HZ            400000 /*!< I2C clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE     0      /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE     0      /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS         1000   /*!< I2C master time out duration */

/**
 * @brief SD3031 Configuration options.
 */
#define SD3031_SENSOR_ADDRESS         0x32   /*!< Device address */
#define SD3031_SENSOR_INTERRUPT_GPIO  3      /*!< Device interrupt GPIO */

/**
 * @brief Read a small block of sram.
 *
 * @param[in] handle Handle of the sd3031 device.
 * @return
 *      - ESP_OK: Device read successfully.
*/
static esp_err_t s_i2c_sd3031_sram_read(const sd3031_handle_t handle)
{
    uint8_t buf_in[5] = {};
    ESP_RETURN_ON_ERROR(sd3031_sram_read(handle, 0, buf_in, sizeof(buf_in)), TAG, "sram read failed");
    ESP_LOGI(TAG, "Read buf_in[5] %02X, %02X, %02X, %02X, %02X",
             buf_in[0], buf_in[1], buf_in[2], buf_in[3], buf_in[4]);

    return ESP_OK;
}

/**
 * @brief Write a small block of sram.
 *
 * @param[in] handle Handle of the sd3031 device.
 * @return
 *      - ESP_OK: Device written successfully.
*/
static esp_err_t s_i2c_sd3031_sram_write(const sd3031_handle_t handle)
{
    // Enable truer randomness.
    bootloader_random_enable();
    uint8_t buf_out[5] = {0xFF, (esp_random() % 256), 0xFF, (esp_random() % 256), 0xFF};
    ESP_LOGI(TAG, "write buf_out[5] %02X, %02X, %02X, %02X, %02X",
             buf_out[0], buf_out[1], buf_out[2], buf_out[3], buf_out[4]);
    ESP_RETURN_ON_ERROR(sd3031_sram_write(handle, 0, buf_out, sizeof(buf_out)), TAG, "sram write failed");
    bootloader_random_disable();

    return ESP_OK;
}

/**
 * @brief Print the RTC clock time.
 *
 * @param[in] handle Handle of the sd3031 device.
 * @return
 *      - ESP_OK: Clock time retrieved and printed successfully.
*/
static esp_err_t s_i2c_sd3031_time_print(const sd3031_handle_t handle)
{
    sd3031_time_t rtc_time;
    ESP_RETURN_ON_ERROR(sd3031_time_get(handle, &rtc_time), TAG, "rtc read failed");
    ESP_LOGI(TAG, "date time: %d/%d/%d %02d:%02d:%02d",
             rtc_time.year, rtc_time.month, rtc_time.day,
             rtc_time.hour, rtc_time.minute, rtc_time.second);

    return ESP_OK;
}

/**
 * @brief Set the RTC clock time to an arbitrary value.
 *
 * @param[in] handle Handle of the sd3031 device.
 * @return
 *      - ESP_OK: Clock time set successfully.
*/
static esp_err_t s_i2c_sd3031_time_set(const sd3031_handle_t handle)
{
    sd3031_time_t rtc_time = {
        .year = 2024,
        .month = 1,
        .day = 11,
        .flags.hour_24 = true,
        .hour = 14,
        .minute = 48,
        .second = 31,
    };
    ESP_RETURN_ON_ERROR(sd3031_time_set(handle, rtc_time), TAG, "rtc set time failed");

    return ESP_OK;
}

/**
 * @brief Print the RTC temperature.
 *
 * @param[in] handle Handle of the sd3031 device.
 * @return
 *      - ESP_OK: Temperature retrieved and printed successfully.
*/
static esp_err_t s_i2c_sd3031_temperature_print(const sd3031_handle_t handle)
{
    int8_t temperature;
    ESP_ERROR_CHECK(sd3031_temperature(handle, &temperature));
    ESP_LOGI(TAG, "temperature: %02d", temperature);

    return ESP_OK;
}

/**
 * @brief Print the RTC battery voltage.
 *
 * @param[in] handle Handle of the sd3031 device.
 * @return
 *      - ESP_OK: Battery voltage retrieved and printed successfully.
*/
static esp_err_t s_i2c_sd3031_voltage_print(const sd3031_handle_t handle)
{
    float voltage;
    ESP_ERROR_CHECK(sd3031_voltage(handle, &voltage));
    ESP_LOGI(TAG, "voltage: %3.2f", voltage);

    return ESP_OK;
}

/**
 * @brief Application main entry point.
*/
void app_main(void)
{
    ESP_LOGI(TAG, "alive and in app_main()");

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_io_num = I2C_MASTER_SDA,
        .glitch_ignore_cnt = 7,
    };

    // Initialise I2C bus.
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    // The complete device configuration.
    sd3031_device_config_t sd3031_device_config = {
        .i2c_dev_conf.scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .i2c_dev_conf.device_address = SD3031_SENSOR_ADDRESS,
        .i2c_dev_conf.dev_addr_length =I2C_ADDR_BIT_7,
        .interrupt_gpio = SD3031_SENSOR_INTERRUPT_GPIO
    };

    // Add device to the bus.
    sd3031_handle_t sd3031_handle = NULL;
    ESP_ERROR_CHECK(i2c_sd3031_add_device(bus_handle, sd3031_device_config, &sd3031_handle));

    // Read a small buffer of data. If battery backed, or warm boot, this will contain the previous values.
    ESP_ERROR_CHECK(s_i2c_sd3031_sram_read(sd3031_handle));

    // Write a small buffer of data.
    ESP_ERROR_CHECK(s_i2c_sd3031_sram_write(sd3031_handle));

    // Read a small buffer of data. This should contain the same data as previously written.
    ESP_ERROR_CHECK(s_i2c_sd3031_sram_read(sd3031_handle));

    // Print current rtc time. If battery backed, or warm boot, this should contain linear time since last update.
    ESP_ERROR_CHECK(s_i2c_sd3031_time_print(sd3031_handle));

    // Set the time to some preset value.
    ESP_ERROR_CHECK(s_i2c_sd3031_time_set(sd3031_handle));

    for (int i = 1; i < 5; ++i) {

        // Print the current RTC time value.
        ESP_ERROR_CHECK(s_i2c_sd3031_time_print(sd3031_handle));

        // Print the current battery voltage.
        // Note: this is only updated every 30 seconds on power and 60 minutes on battery.
        ESP_ERROR_CHECK(s_i2c_sd3031_voltage_print(sd3031_handle));

        // Print the device temperature.
        ESP_ERROR_CHECK(s_i2c_sd3031_temperature_print(sd3031_handle));

        // Delay loop by 10 seconds.
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    // Remove device from the bus.
    ESP_ERROR_CHECK(i2c_sd3031_del_device(sd3031_handle));

    // Destroy the I2C bus.
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));

}
