#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include <stdint.h>

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0                       /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                    /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                    /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
#define CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2

// --- OLED ---
#define OLED_ADDR 0x3C

// --- weather sensor ---
#define BME280_ADDR 0x76
#define BME280_REG_ADDR 0xF7

// --- air quality sensor ---
#define DS3231_ADDR 0x68

static const char *TAG = "WEATHER_STATION";

typedef struct
{
    BME280_calibration_coefficients coefficients;
    int32_t adc_T; // raw temp reading from the analog to digital converter (adc)
    // t_fine = fine temperature, meaning processed temp reading with higher
    // precision than the final output (used internally by the compensation algorithms)
    int32_t t_fine;
    double temperature;
    double humidity;
    double pressure;
} BME280;

typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} BME280_calibration_coefficients;

void BME280_compensate_temperature(BME280 *bme)
{
    // compensates the temperature using factory calibration coefficients and formats it in human readable form to BME280.temperature
    // also calculates BME280.t_fine for later use in pressure and humidity compensation
    // the calculations itself were copied from the datasheet
    int32_t var1, var2, T;
    var1 = ((((bme->adc_T >> 3) - ((int32_t)bme->coefficients.dig_T1 << 1))) * ((int32_t)bme->coefficients.dig_T2)) >> 11;
    var2 = (((((bme->adc_T >> 4) - ((int32_t)bme->coefficients.dig_T1)) * ((bme->adc_T >> 4) - ((int32_t)bme->coefficients.dig_T1))) >> 12) * ((int32_t)bme->coefficients.dig_T3)) >> 14;
    bme->t_fine = var1 + var2;
    bme->temperature = ((bme->t_fine * 5 + 128) >> 8) / 100.0;
}

void app_main(void)
{
    ESP_LOGI(TAG, "init");

    // i2c setup

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 32,
        .scl_io_num = 33,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .intr_priority = 0,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t bme280_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_ADDR,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t bme280_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bme280_config, &bme280_handle));

    // BME280 reset

    uint8_t reset_bytes[2] = {0xE0, 0xB6};
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_handle, reset_bytes, 2, 1000));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    uint8_t buf = 0xF7;
    uint8_t bme280_data[8] = {0};

    while (1)
    {
        ESP_LOGI(TAG, "loop start");
        ESP_ERROR_CHECK(i2c_master_transmit_receive(bme280_handle, &buf, 1, bme280_data, 8, 1000));
        for (int i = 0; i < 8; i++)
        {
            printf("%d\n", bme280_data[i]);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}