/*
 * MPU6050 ESP-IDF Driver by TNY Robotics
 *
 * SPDX-FileCopyrightText: 2025 TNY Robotics
 * SPDX-License-Identifier: MIT
 * 
 * 
 * Copyright (C) 2025 TNY Robotics
 * 
 * This file is part of the MPU6050 ESP-IDF Driver.
 * 
 * License: MIT
 * Repository: https://github.com/tny-robotics/mpu6050
 * 
 * Author: TNY Robotics
 * Date: 19/06/2025
 * Version: 1.0
 */

#include <freertos/FreeRTOS.h>
#include "mpu6050.h"

#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_47
#define I2C_HOST I2C_NUM_0

#ifdef __cplusplus
extern "C"
#endif
void app_main(void)
{
    /* I2C CONFIGURATION */

    // i2c bus configuration
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_HOST,               // I2C port number
        .sda_io_num = I2C_SDA_GPIO,         // GPIO number for I2C sda signal
        .scl_io_num = I2C_SCL_GPIO,         // GPIO number for I2C scl signal
        .clk_source = I2C_CLK_SRC_DEFAULT,  // I2C clock source, just use the default
        .glitch_ignore_cnt = 7,             // glitch filter, again, just use the default
        .flags = {
            .enable_internal_pullup = true, // enable internal pullup resistors (oled screen does not have one)
        },
    };

    // Create the i2c bus handle
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    // Create the mpu6050 device handle
    mpu6050_handle_t mpu_handle;
    ESP_ERROR_CHECK(mpu6050_create(i2c_bus_handle, MPU6050_DEFAULT_INFO(), &mpu_handle));

    // Configure the mpu6050 with default settings
    ESP_ERROR_CHECK(mpu6050_config(mpu_handle, MPU6050_DEFAULT_CONFIG()));

    // NOTE : If wake_auto is disabled in the config, use mpu6050_wake_up() to wake up the device from sleep mode before reading values.

    // Get the accelerometer, gyroscope and temperature values
    while (true)
    {
        mpu6050_accel_value_t raw_accel;
        mpu6050_gyro_value_t raw_gyro;
        mpu6050_temp_value_t temp;

        ESP_ERROR_CHECK(mpu6050_get_accel(mpu_handle, &raw_accel));
        ESP_ERROR_CHECK(mpu6050_get_gyro(mpu_handle, &raw_gyro));
        ESP_ERROR_CHECK(mpu6050_get_temp(mpu_handle, &temp));

        // Print the values to the console
        printf("Accelerometer: X = %.2f g, Y = %.2f g, Z = %.2f g\n", raw_accel.accel_x, raw_accel.accel_y, raw_accel.accel_z);
        printf("Gyroscope: X = %.2f dps, Y = %.2f dps, Z = %.2f dps\n", raw_gyro.gyro_x, raw_gyro.gyro_y, raw_gyro.gyro_z);
        printf("Temperature: %.2f °C\n", temp.temp);

        vTaskDelay(pdMS_TO_TICKS(1000)); // wait a bit to avoid spamming the console with values
    }

    // Delete the mpu6050 device handle
    ESP_ERROR_CHECK(mpu6050_delete(mpu_handle));
}