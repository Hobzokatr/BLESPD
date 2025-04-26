#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Define SPI pins
#define PIN_NUM_MOSI 7
#define PIN_NUM_MISO 2
#define PIN_NUM_CLK  6

#define PIN_NUM_CS   21

// BMX055 Accelerometer register definitions (example)
#define BMX055_ACC_CHIP_ID 0xFA
#define BMX055_ACC_CHIP_ID_REG 0x00
#define BMX055_ACC_X_LSB 0x02
#define BMX055_ACC_READ_MASK 0x80

static const char *TAG = "BMX055_ACC";
volatile int16_t current_x = 0, current_y = 0, current_z = 0;

spi_device_handle_t spi;

// Pomocná funkce pro SPI čtení z registru
esp_err_t bmx055_acc_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    spi_transaction_t t = {
        .length = 8 * (len + 1),
        .tx_buffer = NULL,
        .rx_buffer = NULL
    };
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];

    tx[0] = reg_addr | BMX055_ACC_READ_MASK;  // MSB=1 pro čtení
    for (int i = 1; i <= len; ++i) tx[i] = 0x00;

    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) return ret;

    for (int i = 0; i < len; ++i) data[i] = rx[i + 1];
    return ESP_OK;
}

esp_err_t bmx055_acc_write_reg(uint8_t reg_addr, uint8_t data) {
    spi_transaction_t t = {
        .length = 8 * 2,
        .tx_buffer = NULL
    };
    uint8_t tx[2] = {reg_addr & 0x7F, data}; // MSB=0 for write
    t.tx_buffer = tx;
    return spi_device_transmit(spi, &t);
}

// Inicializace SPI a zařízení BMX055 (pouze ACC)
void bmx055_acc_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,  // 1 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Kontrola ID
    uint8_t chip_id = 0;
    ESP_ERROR_CHECK(bmx055_acc_read_reg(BMX055_ACC_CHIP_ID_REG, &chip_id, 1));
    if (chip_id != BMX055_ACC_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X", chip_id);
    } else {
        ESP_LOGI(TAG, "BMX055 ACC connected! Chip ID: 0x%02X", chip_id);
    }
    ESP_ERROR_CHECK(bmx055_acc_write_reg(0x0F, 0x0C));  // 0x03 = ±2g
}

// Získání hrubých dat z akcelerometru
void bmx055_acc_read_xyz(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t raw[6];
    esp_err_t ret = bmx055_acc_read_reg(BMX055_ACC_X_LSB, raw, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read acc data");
        *x = *y = *z = -1;
        return;
    }

    *x = ((int16_t)(raw[1] << 8 | raw[0])) >> 4;
    *y = ((int16_t)(raw[3] << 8 | raw[2])) >> 4;
    *z = ((int16_t)(raw[5] << 8 | raw[4])) >> 4;
}

void wifi_init_softap(void) {
    // Initialize TCP/IP stack and default event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default WiFi AP network interface
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_AP",
            .ssid_len = strlen("ESP32_AP"),
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .password = "esp32pass"
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi softAP initialized. SSID:%s password:%s", wifi_config.ap.ssid, wifi_config.ap.password);
}

void udp_broadcast_task(void *pvParameter) {
    char udp_message[100];
    struct sockaddr_in broadcast_addr;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    int broadcast_enable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        ESP_LOGE(TAG, "Error setting socket option: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }

    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(3333);
    broadcast_addr.sin_addr.s_addr = inet_addr("255.255.255.255");

    while (1) {
        // Format the current sensor readings into a message
        snprintf(udp_message, sizeof(udp_message), "X: %d, Y: %d, Z: %d", current_x, current_y, current_z);
        int err = sendto(sock, udp_message, strlen(udp_message), 0, (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "UDP message sent: %s", udp_message);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    close(sock);
    vTaskDelete(NULL);
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi in softAP mode
    wifi_init_softap();

    // Create UDP broadcast task
    xTaskCreate(udp_broadcast_task, "udp_broadcast_task", 4096, NULL, 5, NULL);

    bmx055_acc_init();

    while (1) {
        int16_t x, y, z;
        bmx055_acc_read_xyz(&x, &y, &z);

        // Update global sensor data variables
        current_x = x;
        current_y = y;
        current_z = z;

        ESP_LOGI(TAG, "Sensor data (X, Y, Z): %d, %d, %d", x, y, z);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}