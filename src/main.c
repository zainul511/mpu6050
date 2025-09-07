#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "esp_netif.h"

#define PCA9548A_ADDR 0x70
#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT I2C_NUM_0

#define SERVER_URL "http://192.168.8.102:5000/data"
#define WIFI_SSID "HUAWEI-A5CE"
#define WIFI_PASS "i861qene"

static const char *TAG = "MPU6050";
static bool data_collection_active = false;

// Queue to hold sensor data for non-blocking HTTP send
static QueueHandle_t http_queue;

typedef struct {
    int16_t accel[3][3]; // [sensor][x,y,z]
    int64_t timestamp_ms;
} sensor_packet_t;

// WiFi event handling
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: %s",ip4addr_ntoa((ip4_addr_t *)&event->ip_info.ip));
    }
}

void wait_for_wifi_connection()
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected!");
}

// I2C initialization
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

// PCA9548A channel selection
esp_err_t pca9548a_select_channel(uint8_t channel) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9548A_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (1 << channel), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C write
esp_err_t i2c_write_byte(uint8_t channel, uint8_t reg_addr, uint8_t data) {
    if (pca9548a_select_channel(channel) != ESP_OK) return ESP_FAIL;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C read
esp_err_t i2c_read_bytes(uint8_t channel, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (pca9548a_select_channel(channel) != ESP_OK) return ESP_FAIL;

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
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// MPU6050 initialization
esp_err_t mpu6050_init(uint8_t channel) {
    esp_err_t ret = i2c_write_byte(channel, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;

    uint8_t who_am_i;
    ret = i2c_read_bytes(channel, MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK || who_am_i != 0x68) return ESP_ERR_NOT_FOUND;

    ESP_LOGI(TAG, "MPU6050 on channel %d initialized", channel);
    return ESP_OK;
}

// Read accelerometer
esp_err_t mpu6050_read_accel(uint8_t channel, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t data[6];
    esp_err_t ret = i2c_read_bytes(channel, MPU6050_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) return ret;

    *accel_x = (int16_t)(data[0] << 8 | data[1]);
    *accel_y = (int16_t)(data[2] << 8 | data[3]);
    *accel_z = (int16_t)(data[4] << 8 | data[5]);
    return ESP_OK;
}

// HTTP event handler
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
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER"); 
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

        case HTTP_EVENT_REDIRECT: 
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT"); 
            break;
    }
    return ESP_OK;
}

// Task to send HTTP requests
void http_send_task(void *pvParameters) {
    sensor_packet_t packet;
    while (true) {
        if (xQueueReceive(http_queue, &packet, portMAX_DELAY)) {
            char post_data[256];
            snprintf(post_data, sizeof(post_data),
                     "{\"sensor1\":{\"x\":%d,\"y\":%d,\"z\":%d},"
                     "\"sensor2\":{\"x\":%d,\"y\":%d,\"z\":%d},"
                     "\"sensor3\":{\"x\":%d,\"y\":%d,\"z\":%d},"
                     "\"timestamp\":%lld}",
                     packet.accel[0][0], packet.accel[0][1], packet.accel[0][2],
                     packet.accel[1][0], packet.accel[1][1], packet.accel[1][2],
                     packet.accel[2][0], packet.accel[2][1], packet.accel[2][2],
                     packet.timestamp_ms);

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
                ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
            }
            esp_http_client_cleanup(client);
        }
    }
}

// Button check task
void check_button_press() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    while (true) {
        if (gpio_get_level(GPIO_NUM_0) == 0) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            if (gpio_get_level(GPIO_NUM_0) == 0) {
                data_collection_active = !data_collection_active;
                ESP_LOGI(TAG, "Data collection %s", data_collection_active ? "STARTED" : "STOPPED");
                while (gpio_get_level(GPIO_NUM_0) == 0) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Data collection task
void data_collection_task(void *pvParameters) {
    int16_t accel_data[3][3];
    sensor_packet_t packet;

    while (true) {
        if (data_collection_active) {
            for (uint8_t channel = 0; channel < 3; channel++) {
                if (mpu6050_read_accel(channel, &accel_data[channel][0], 
                                                &accel_data[channel][1], 
                                                &accel_data[channel][2]) == ESP_OK) {
                    ESP_LOGI(TAG, "Sensor %d - X: %6d, Y: %6d, Z: %6d",
                            channel, accel_data[channel][0], accel_data[channel][1], accel_data[channel][2]);
                }
            }
            memcpy(packet.accel, accel_data, sizeof(accel_data));
            packet.timestamp_ms = esp_timer_get_time() / 1000;

            // Send to HTTP queue (non-blocking)
            xQueueSend(http_queue, &packet, 0);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 10 Hz
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize I2C
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized");

    // Initialize MPU6050
    for (uint8_t channel = 0; channel < 3; channel++) {
        if (mpu6050_init(channel) != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050 on channel %d initialization failed!", channel);
        }
    }

    // Wait for WiFi
    wait_for_wifi_connection();

    // Create queue for non-blocking HTTP
    http_queue = xQueueCreate(10, sizeof(sensor_packet_t));

    // Create tasks
    xTaskCreate(data_collection_task, "data_collection", 4096, NULL, 5, NULL);
    xTaskCreate(check_button_press, "button_check", 2048, NULL, 5, NULL);
    xTaskCreate(http_send_task, "http_send", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System ready. Press BOOT button to start/stop data collection");
}
