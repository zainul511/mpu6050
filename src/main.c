#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define PCA9548A_ADDR 0x70           // PCA9548A I2C address
#define MPU6050_ADDR 0x68            // MPU6050 I2C address
#define MPU6050_WHO_AM_I 0x75        // Who am I register
#define MPU6050_PWR_MGMT_1 0x6B      // Power management register
#define MPU6050_ACCEL_XOUT_H 0x3B    // Accelerometer data register

#define I2C_MASTER_SCL_IO 22         // GPIO pin for SCL
#define I2C_MASTER_SDA_IO 21         // GPIO pin for SDA
#define I2C_MASTER_FREQ_HZ 100000    // I2C master clock frequency
#define I2C_MASTER_PORT I2C_NUM_0

static const char *TAG = "MPU6050";

// I2C initialization (unchanged)
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// Select PCA9548A channel
esp_err_t pca9548a_select_channel(uint8_t channel) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9548A_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (1 << channel), true);  // Select specific channel
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Write byte to I2C device (modified to include channel selection)
esp_err_t i2c_write_byte(uint8_t channel, uint8_t reg_addr, uint8_t data) {
    pca9548a_select_channel(channel);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read bytes from I2C device (modified to include channel selection)
esp_err_t i2c_read_bytes(uint8_t channel, uint8_t reg_addr, uint8_t *data, size_t len) {
    pca9548a_select_channel(channel);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Initialize MPU6050 on specific channel
esp_err_t mpu6050_init(uint8_t channel) {
    // Wake up MPU6050
    esp_err_t ret = i2c_write_byte(channel, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050 on channel %d", channel);
        return ret;
    }
    
    // Verify device ID
    uint8_t who_am_i;
    ret = i2c_read_bytes(channel, MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register on channel %d", channel);
        return ret;
    }
    
    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I value on channel %d: 0x%02X", channel, who_am_i);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "MPU6050 on channel %d initialized successfully (WHO_AM_I: 0x%02X)", channel, who_am_i);
    return ESP_OK;
}

// Read accelerometer data from specific channel
esp_err_t mpu6050_read_accel(uint8_t channel, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t data[6];
    esp_err_t ret = i2c_read_bytes(channel, MPU6050_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *accel_x = (int16_t)(data[0] << 8) | data[1];
    *accel_y = (int16_t)(data[2] << 8) | data[3];
    *accel_z = (int16_t)(data[4] << 8) | data[5];
    
    return ESP_OK;
}

void app_main(void) {
    // Initialize I2C
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized");
    
    // Initialize all 3 MPU6050 sensors
    for (uint8_t channel = 0; channel < 3; channel++) {
        if (mpu6050_init(channel) != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050 on channel %d initialization failed!", channel);
        }
    }
    
    int16_t accel_x, accel_y, accel_z;
    
    while (1) {
        for (uint8_t channel = 0; channel < 3; channel++) {
            // Read accelerometer data from each sensor
            if (mpu6050_read_accel(channel, &accel_x, &accel_y, &accel_z) == ESP_OK) {
                float g_x = accel_x / 16384.0;
                float g_y = accel_y / 16384.0;
                float g_z = accel_z / 16384.0;
                
                printf("Sensor %d - G-force X: %6.2f | Y: %6.2f | Z: %6.2f\n", 
                       channel, g_x, g_y, g_z);
            } else {
                ESP_LOGE(TAG, "Failed to read accelerometer data from channel %d", channel);
            }
        }
        printf("\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}