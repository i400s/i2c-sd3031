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

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "i2c_sd3031.h"

/**
 * @brief SD3031 Configuration options.
 * @note The SD3031 allows block read/write data from a given start register address
 */
#define SD3031_REG_SECONDS        0x00 /*!< RTC seconds register */
#define SD3031_REG_MINUTES        0x01 /*!< RTC minutes register */
#define SD3031_REG_HOURS          0x02 /*!< RTC hours register */
#define SD3031_REG_WEEK           0x03 /*!< RTC week register */
#define SD3031_REG_DAY            0x04 /*!< RTC day register */
#define SD3031_REG_MONTH          0x05 /*!< RTC month register */
#define SD3031_REG_YEAR           0x06 /*!< RTC year register */
#define SD3031_REG_ALARM_SECONDS  0x07 /*!< RTC seconds alarm register */
#define SD3031_REG_ALARM_MINUTES  0x08 /*!< RTC minutes alarm register */
#define SD3031_REG_ALARM_HOURS    0x09 /*!< RTC hours alarm register */
#define SD3031_REG_ALARM_WEEK     0x0A /*!< RTC week alarm register */
#define SD3031_REG_ALARM_DAY      0x0B /*!< RTC day alarm register */
#define SD3031_REG_ALARM_MONTH    0x0C /*!< RTC month alarm register */
#define SD3031_REG_ALARM_YEAR     0x0D /*!< RTC year alarm register */
#define SD3031_REG_ALARM_CONTROL  0x0E /*!< RTC alarm control register */
#define SD3031_REG_CTR1           0x0F /*!< Control register 1 */
#define SD3031_REG_CTR2           0x10 /*!< Control register 2 */
#define SD3031_REG_CTR3           0x11 /*!< Control register 3 */
#define SD3031_REG_COUNTDOWN      0X13 /*!< Countdown register */
#define SD3031_REG_TEMPERATURE    0x16 /*!< Internal temperature register */
#define SD3031_REG_I2C_CONTROL    0x17 /*!< I2C control */
#define SD3031_REG_BATTERY_LEVEL  0x1A /*!< Battery level */
#define SD3031_REG_USER_RAM_START 0x2C /*!< Start of user ram */
#define SD3031_REG_USER_RAM_END   0x71 /*!< End of user ram */
#define SD3031_ACK_TIMEOUT_MS       10 /*!< Time to ACK in MS before timing out */

static const char *TAG = "i2c-sd3031";
static const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

esp_err_t i2c_sd3031_add_device(i2c_master_bus_handle_t bus_handle, sd3031_device_config_t device_config, sd3031_handle_t* handle)
{
    ESP_RETURN_ON_FALSE((bus_handle != NULL), ESP_ERR_INVALID_ARG, TAG, "i2c bus not initialized");

    ESP_LOGI(TAG, "adding device to I2C bus at address %04X", device_config.i2c_dev_conf.device_address);

    esp_err_t ret = ESP_OK;
    sd3031_handle_t out_handle;
    out_handle = (sd3031_handle_t)calloc(1, sizeof(sd3031_device_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err_tag, TAG, "could not allocate memory for device");

    if (out_handle->i2c_dev == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &device_config.i2c_dev_conf, &out_handle->i2c_dev),
                          err_tag, TAG, "unable to add device to I2C bus at address %04X",
                          device_config.i2c_dev_conf.device_address);
        ESP_GOTO_ON_ERROR(i2c_master_probe(bus_handle, device_config.i2c_dev_conf.device_address, SD3031_ACK_TIMEOUT_MS),
                          err_tag, TAG, "device at address %04X did not respond",
                          device_config.i2c_dev_conf.device_address);
    }

    out_handle->device_address = device_config.i2c_dev_conf.device_address;
    *handle = out_handle;
    return ret;

err_tag:
    if (out_handle) {
        if (out_handle->i2c_dev) {
            i2c_master_bus_rm_device(out_handle->i2c_dev);
        }
        free(out_handle);
    }
    return ret;
}

esp_err_t i2c_sd3031_del_device(sd3031_handle_t handle)
{
    ESP_RETURN_ON_FALSE((handle != NULL), ESP_ERR_INVALID_ARG, TAG, "this device is not initialized");
    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(handle->i2c_dev), TAG, "failed to remove sd3031 from i2c bus");

    ESP_LOGI(TAG, "removed device from I2C bus at address: %04X", handle->device_address);
    if (handle) {
        free(handle);
        handle = NULL;
    }

    return ESP_OK;
}

/**
 * @brief Set write allow flags.
 *
 * @param[in] i2c_dev Handle of the i2c bus device.
 * @return
 *      - ESP_OK: Device unlocked successfully.
*/
static esp_err_t s_sd3031_set_write_allow(const i2c_master_dev_handle_t i2c_dev)
{
    uint8_t data[3];
    data[0] = SD3031_REG_CTR1;
    // Get registers
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(i2c_dev, data, 1, &data[1], 2, -1), TAG, "failed to get control registers");
    // Set write allow flags
    uint8_t swapCTR = data[2] | 0x80; // CTR2 WRTC1
    data[2] = data[1] | 0x84; // CTR1 WRTC3 & WRTC2
    data[1] = swapCTR;
    // Write the second control register first
    data[0] = SD3031_REG_CTR2;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(i2c_dev, &data[0], 2, -1), TAG, "failed to set WRTC1 write allow flags");
    // Write the first control register last
    data[1] = SD3031_REG_CTR1;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(i2c_dev, &data[1], 2, -1), TAG, "failed to set WRTC3 & WRTC2 write allow flags");

    return ESP_OK;
}

/**
 * @brief Unset write allow flags.
 *
 * @param[in] i2c_dev Handle of the i2c bus device.
 * @return
 *      - ESP_OK: Device locked successfully.
*/
static esp_err_t s_sd3031_unset_write_allow(const i2c_master_dev_handle_t i2c_dev)
{
    uint8_t data[3];
    data[0] = SD3031_REG_CTR1;
    // Get registers
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(i2c_dev, data, 1, &data[1], 2, -1), TAG, "failed to get control registers");
    // Set off write allow flags
    uint8_t swapCTR = data[2] ^ 0x80; // CTR2 WRTC1
    data[2] = data[1] ^ 0x84; // CTR1 WRTC3 WRTC2
    data[1] = swapCTR;
    data[0] = SD3031_REG_CTR2;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(i2c_dev, &data[0], 2, -1), TAG, "failed to unset WRTC1 write allow flags");
    data[1] = SD3031_REG_CTR1;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(i2c_dev, &data[1], 2, -1), TAG, "failed to unset WRTC3 & WRTC2 write allow flags");

    return ESP_OK;
}

/**
 * @brief Write data to registers surrounded by write enable/write disable.
 *
 * @param[in] i2c_dev Handle of the i2c bus device.
 * @param[in] reg First register address to write.
 * @param[in] buffer Buffer to write.
 * @param[in] size Buffer size.
 * @return
 *      - ESP_OK: Data written successfully.
*/
static esp_err_t s_sd3031_write_register_block(const i2c_master_dev_handle_t i2c_dev, uint8_t reg,
                                               const uint8_t* reg_buffer, uint8_t reg_buffer_size)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "writing %d registers starting at address %02X", reg_buffer_size, reg);

    uint8_t write_buf_size = reg_buffer_size + 1;
    uint8_t *write_buf = (uint8_t*)malloc(write_buf_size);
    ESP_GOTO_ON_FALSE(write_buf, ESP_ERR_NO_MEM, clean, TAG, "unable to allocate memory for sd3031 device");
    memcpy(write_buf, &reg, 1);
    memcpy(write_buf + 1, reg_buffer, reg_buffer_size);

    ESP_GOTO_ON_ERROR(s_sd3031_set_write_allow(i2c_dev), clean, TAG, "failed to set write allow flags");
    ESP_GOTO_ON_ERROR(i2c_master_transmit(i2c_dev, write_buf, write_buf_size, -1), clean, TAG, "failed to write buffered data");
    ESP_GOTO_ON_ERROR(s_sd3031_unset_write_allow(i2c_dev), clean, TAG, "failed to unset write allow flags");

    // always deallocate the buffer
clean:
    if (write_buf) {
        free(write_buf);
        write_buf = NULL;
    }
    return ret;
}

/**
 * @brief Read data from registers.
 *
 * @param[in] i2c_dev Handle of the i2c bus device.
 * @param[in] reg First register address to read.
 * @param[out] buffer Buffer to read.
 * @param[in] size Buffer size.
 * @return
 *      - ESP_OK: Data read successfully.
*/
static esp_err_t s_sd3031_read_register_block(const i2c_master_dev_handle_t i2c_dev, uint8_t reg,
                                              uint8_t* reg_buffer, uint8_t reg_buffer_size)
{

    ESP_LOGD(TAG, "reading %d registers starting at address %02X", reg_buffer_size, reg);
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(i2c_dev, &reg, 1, reg_buffer, reg_buffer_size, -1),
                        TAG, "failed to write buffered data");

    return ESP_OK;
}

/**
 * @brief Convert binary to binary coded decimal.
 *
 * @param[in] value Value to be converted.
 * @return Converted value.
*/
static uint8_t s_sd3031_bin2bcd(uint8_t value)
{
    return value + 6 * (value / 10);
}

/**
 * @brief Convert binary coded decimal to binary.
 *
 * @param[in] value Value to be converted.
 * @return Converted value.
*/
static uint8_t s_sd3031_bcd2bin(uint8_t value)
{
    return value - 6 * (value >> 4);
}

/**
 * @brief Check for leap year.
 *
 * @param[in] year Year to be tested.
 * @return 1 if leap year or 0 if not.
*/
uint8_t s_sd3031_leap_year(uint16_t year)
{
    if ((year % 4 && !year % 100) || year % 400) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Calculate the day of the week.
 *
 * @param[in] date Date to be tested.
 * @return Day of the week.
 * @note Week starts on Monday.
*/
static uint8_t s_sd3031_day_of_week(sd3031_date_t date) /* 1 <= m <= 12,  y > 1752 (in the U.K.) */
{
    static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (date.month < 3) {
        date.year -= 1;
    }
    uint8_t day_of_week = (date.year + date.year / 4 - date.year / 100 + date.year / 400 + t[date.month - 1] + date.day) % 7;
    ESP_LOGI(TAG, "calculated day of week: %d", day_of_week);
    return day_of_week;
}

esp_err_t sd3031_time_set(const sd3031_handle_t i2c_sd3031_handle, sd3031_time_t time)
{
    ESP_LOGI(TAG, "setting date/time: %d/%d/%d %d:%d:%d", time.year, time.month, time.day, time.hour, time.minute, time.second);
    ESP_RETURN_ON_FALSE(time.month >= 1 && time.month <= 12, ESP_ERR_INVALID_ARG, TAG, "month must be between 1 and 12");
    ESP_RETURN_ON_FALSE(time.day >= 1 && time.day <= (daysInMonth[time.month - 1] + s_sd3031_leap_year(time.year)),
                        ESP_ERR_INVALID_ARG, TAG, "day must be between 1 and %d", daysInMonth[time.month - 1]);
    if (time.flags.hour_24) {
        ESP_RETURN_ON_FALSE(time.hour <= 23, ESP_ERR_INVALID_ARG, TAG, "hour must be between 0 and 24");
    } else {
        ESP_RETURN_ON_FALSE(time.hour >= 1 && time.hour <= 12, ESP_ERR_INVALID_ARG, TAG, "hour must be between 1 and 12");
    }
    ESP_RETURN_ON_FALSE(time.minute <= 59, ESP_ERR_INVALID_ARG, TAG, "minutes must be between 0 and 59");
    ESP_RETURN_ON_FALSE(time.second <= 59, ESP_ERR_INVALID_ARG, TAG, "seconds must be between 0 and 59");

    uint8_t data[7];
    data[0] = s_sd3031_bin2bcd(time.second); // Seconds
    data[1] = s_sd3031_bin2bcd(time.minute); // Minutes
    data[2] = s_sd3031_bin2bcd(time.hour);   // Hours
    data[2] |= time.flags.hour_24 << 7;      // 24/12 hour bit (true=24)
    if (!time.flags.hour_24) {
        data[2] |= time.flags.hour_pm << 5;  // pm/am bit (true=pm)
    }
    sd3031_date_t date = {.year = time.year,
                          .month = time.month,
                          .day = time.day,
                         };
    data[3] = s_sd3031_bin2bcd(s_sd3031_day_of_week(date)); // Day of week starting at 0
    data[4] = s_sd3031_bin2bcd(time.day); // Day
    data[5] = s_sd3031_bin2bcd(time.month); // Month
    data[6] = s_sd3031_bin2bcd(time.year - 2000); // Year
    ESP_LOGD(TAG, "write data[7] %02X:%02X:%02X:%02X:%02X:%02X:%02X",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

    ESP_RETURN_ON_ERROR(s_sd3031_write_register_block(i2c_sd3031_handle->i2c_dev, SD3031_REG_SECONDS, data, 7),
                        TAG, "unable to write register(s)");

    return ESP_OK;
}

esp_err_t sd3031_time_get(const sd3031_handle_t handle, sd3031_time_t *time)
{
    ESP_LOGI(TAG, "getting date/time");
    uint8_t buffer[7];
    s_sd3031_read_register_block(handle->i2c_dev, SD3031_REG_SECONDS, buffer, sizeof(buffer));
    time->second = s_sd3031_bcd2bin(buffer[0]);
    time->minute = s_sd3031_bcd2bin(buffer[1]);
    time->flags.hour_24 = buffer[2] >> 7;
    if (time->flags.hour_24) {
        time->hour = s_sd3031_bcd2bin(buffer[2] & 0x7F);
    } else {
        time->flags.hour_pm = buffer[2] & (1 << 5) >> 5;
        time->hour = s_sd3031_bcd2bin(buffer[2] & 0x1F);
    }
    time->flags.week = s_sd3031_bcd2bin(buffer[3]);
    time->day = s_sd3031_bcd2bin(buffer[4]);
    time->month = s_sd3031_bcd2bin(buffer[5]);
    time->year = 2000 + s_sd3031_bcd2bin(buffer[6]);
    ESP_LOGD(TAG, "buffer[7] %02X:%02X:%02X:%02X:%02X:%02X:%02X",
             buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);

    return ESP_OK;
}

esp_err_t sd3031_temperature(const sd3031_handle_t handle, int8_t* temperature)
{
    ESP_LOGI(TAG, "getting temperature");
    uint8_t buffer[1];
    s_sd3031_read_register_block(handle->i2c_dev, SD3031_REG_TEMPERATURE, buffer, sizeof(buffer));
    ESP_LOGD(TAG, "buffer[1] %02X", buffer[0]);
    if (buffer[0] & 0x80) {
        *temperature = -(buffer[0] & 0x7F);
    } else {
        *temperature = buffer[0] & 0x7F;
    }

    return ESP_OK;
}

esp_err_t sd3031_voltage(const sd3031_handle_t handle, float* voltage)
{
    ESP_LOGI(TAG, "getting voltage");
    uint8_t buffer[2];
    s_sd3031_read_register_block(handle->i2c_dev, SD3031_REG_BATTERY_LEVEL, buffer, sizeof(buffer));
    ESP_LOGD(TAG, "buffer[2] %02X:%02X", buffer[0], buffer[1]);
    *voltage = ((((buffer[0] & 0x80) >> 7) << 8) | buffer[1]) / 100.0;

    return ESP_OK;
}


esp_err_t sd3031_sram_write(const sd3031_handle_t handle, uint8_t offset, const uint8_t* buffer, uint8_t size)
{
    ESP_LOGI(TAG, "writing to user defined sram");
    ESP_RETURN_ON_FALSE((SD3031_REG_USER_RAM_START + offset + size - 1)
                         <= SD3031_REG_USER_RAM_END, ESP_ERR_INVALID_SIZE,
                         TAG, "invalid sd3031 user memory address or size");
    s_sd3031_write_register_block(handle->i2c_dev, SD3031_REG_USER_RAM_START + offset, buffer, size);
    return ESP_OK;
}

esp_err_t sd3031_sram_read(const sd3031_handle_t handle, uint8_t offset, uint8_t* buffer, uint8_t buffer_size)
{
    ESP_LOGI(TAG, "reading from user defined sram");
    ESP_RETURN_ON_FALSE((SD3031_REG_USER_RAM_START + offset + buffer_size - 1)
                        <= SD3031_REG_USER_RAM_END, ESP_ERR_INVALID_SIZE,
                        TAG, "invalid sd3031 user memory address or size");
    s_sd3031_read_register_block(handle->i2c_dev, SD3031_REG_USER_RAM_START + offset, buffer, buffer_size);
    return ESP_OK;
}
