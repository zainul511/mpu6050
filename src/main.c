#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
//#include "protocol_examples_common.h"
#include "esp_http_client.h"
#include "esp_timer.h"

#define PCA9548A_ADDR 0x70
#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT I2C_NUM_0

// Configure your server details here
#define SERVER_URL "http://your-python-server-ip:5000/data"
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASS "your-wifi-password"

static const char *TAG = "MPU6050";
static bool data_collection_active = false;

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

// PCA9548A channel selection (unchanged)
esp_err_t pca9548a_select_channel(uint8_t channel) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9548A_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (1 << channel), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C write with channel (unchanged)
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

// I2C read with channel (unchanged)
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

// MPU6050 initialization (unchanged)
esp_err_t mpu6050_init(uint8_t channel) {
    esp_err_t ret = i2c_write_byte(channel, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;
    
    uint8_t who_am_i;
    ret = i2c_read_bytes(channel, MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK || who_am_i != 0x68) return ESP_ERR_NOT_FOUND;
    
    ESP_LOGI(TAG, "MPU6050 on channel %d initialized", channel);
    return ESP_OK;
}

// Read accelerometer data (unchanged)
esp_err_t mpu6050_read_accel(uint8_t channel, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t data[6];
    esp_err_t ret = i2c_read_bytes(channel, MPU6050_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) return ret;
    
    *accel_x = (int16_t)(data[0] << 8) | data[1];
    *accel_y = (int16_t)(data[2] << 8) | data[3];
    *accel_z = (int16_t)(data[4] << 8) | data[5];
    return ESP_OK;
}

// HTTP Event Handler
esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADERS_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADERS_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

// Send data to Python server
void send_to_server(int16_t accel[3][3]) {
    char post_data[256];
    snprintf(post_data, sizeof(post_data),
             "{\"sensor1\":{\"x\":%d,\"y\":%d,\"z\":%d},"
             "\"sensor2\":{\"x\":%d,\"y\":%d,\"z\":%d},"
             "\"sensor3\":{\"x\":%d,\"y\":%d,\"z\":%d},"
             "\"timestamp\":%lld}",
             accel[0][0], accel[0][1], accel[0][2],
             accel[1][0], accel[1][1], accel[1][2],
             accel[2][0], accel[2][1], accel[2][2],
             esp_timer_get_time() / 1000);

    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .event_handler = http_event_handler,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed");
    }
    esp_http_client_cleanup(client);
}

// Button press handler (using BOOT button)
void check_button_press() {
    // Configure BOOT button (GPIO0) as input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    while (true) {
        if (gpio_get_level(GPIO_NUM_0) == 0) { // Button pressed
            vTaskDelay(50 / portTICK_PERIOD_MS); // Debounce
            if (gpio_get_level(GPIO_NUM_0) == 0) {
                data_collection_active = !data_collection_active;
                ESP_LOGI(TAG, "Data collection %s", data_collection_active ? "STARTED" : "STOPPED");
                while (gpio_get_level(GPIO_NUM_0) == 0) {
                    vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for button release
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void data_collection_task(void *pvParameters) {
    int16_t accel_data[3][3]; // [sensor][x,y,z]
    
    while (true) {
        if (data_collection_active) {
            for (uint8_t channel = 0; channel < 3; channel++) {
                if (mpu6050_read_accel(channel, &accel_data[channel][0], 
                                      &accel_data[channel][1], 
                                      &accel_data[channel][2]) == ESP_OK) {
                    ESP_LOGI(TAG, "Sensor %d - X: %6d, Y: %6d, Z: %6d",
                            channel, accel_data[channel][0], 
                            accel_data[channel][1], accel_data[channel][2]);
                }
            }
            send_to_server(accel_data);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 10Hz sampling
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized");

    // Initialize MPU6050 sensors
    for (uint8_t channel = 0; channel < 3; channel++) {
        if (mpu6050_init(channel) != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050 on channel %d initialization failed!", channel);
        }
    }

    // Initialize WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Replace example_connect() with WiFi initialization
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi initialization finished.");

    // Create tasks
    xTaskCreate(data_collection_task, "data_collection", 4096, NULL, 5, NULL);
    xTaskCreate(check_button_press, "button_check", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "System ready. Press BOOT button to start/stop data collection");
}