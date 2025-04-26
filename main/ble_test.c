#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define GATTS_TAG "BLE_EXAMPLE"
#define DEVICE_NAME "ESP32_BLE"
#define SERVICE_UUID 0x00FF
#define CHAR_UUID 0xFF01

static uint16_t gatts_if_global = 0;
static uint16_t conn_id_global = 0;
static esp_gatt_srvc_id_t service_id;
static uint16_t char_handle;

static esp_attr_value_t gatts_char_val = {
    .attr_max_len = 4,
    .attr_len     = 4,
    .attr_value   = (uint8_t*)"\x00\x00\x00\x00",
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {

    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(DEVICE_NAME);
            service_id.is_primary = true;
            service_id.id.inst_id = 0x00;
            service_id.id.uuid.len = ESP_UUID_LEN_16;
            service_id.id.uuid.uuid.uuid16 = SERVICE_UUID;

            esp_ble_gatts_create_service(gatts_if, &service_id, 4);
            break;

        case ESP_GATTS_CREATE_EVT:
            esp_ble_gatts_start_service(param->create.service_handle);
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = CHAR_UUID,
            };
            esp_ble_gatts_add_char(param->create.service_handle, &char_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ,
                                   &gatts_char_val, NULL);
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            char_handle = param->add_char.attr_handle;
            ESP_LOGI(GATTS_TAG, "Characteristic added");
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Device connected");
            gatts_if_global = gatts_if;
            conn_id_global = param->connect.conn_id;
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Device disconnected");
            break;

        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: {
            ESP_LOGI(GATTS_TAG, "Advertisement data set. Starting advertising...");
            esp_ble_adv_params_t adv_params = {
                .adv_int_min = 0x20,
                .adv_int_max = 0x40,
                .adv_type = ADV_TYPE_IND,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .channel_map = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            };
            esp_ble_gap_start_advertising(&adv_params);
            break;
        }
        default:
            break;
    }
}

void ble_init(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Set the device name early, before configuring advertising data
    esp_ble_gap_set_device_name(DEVICE_NAME);

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(0);
    esp_ble_gap_register_callback(gap_event_handler);

    // Define a 128-bit UUID similar to the one used in Espressif examples
static uint8_t service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f,
    0x80, 0x00,
    0x00, 0x80,
    0x00, 0x10,
    0x00, 0x00,
    0x18, 0x0D, 0x00, 0x00,
};

esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Failed to configure adv data: %s", esp_err_to_name(ret));
    }
}
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ble_init();

    uint32_t count = 0;

    while (1) {
        if (conn_id_global) {
            uint8_t notify_data[4];
            notify_data[0] = count & 0xFF;
            notify_data[1] = (count >> 8) & 0xFF;
            notify_data[2] = (count >> 16) & 0xFF;
            notify_data[3] = (count >> 24) & 0xFF;

            esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global,
                                        char_handle, sizeof(notify_data),
                                        notify_data, false);

            ESP_LOGI(GATTS_TAG, "Notification sent: %lu", count);
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Send data every second
    }
}