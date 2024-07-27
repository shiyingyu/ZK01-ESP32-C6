
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define GATTS_TAG "GATTS"

#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01

#define CURTAIN_SERVICE_UUID        0x180F  // Example UUID for the curtain service
#define CURTAIN_CONTROL_CHAR_UUID    0x2A6E  // Example UUID for control characteristic
#define CURTAIN_STATUS_CHAR_UUID     0x2A6F  // Example UUID for status characteristic
// Define the service and characteristics
static const uint8_t curtain_service_uuid[] = { CURTAIN_SERVICE_UUID & 0xFF, CURTAIN_SERVICE_UUID >> 8 };
static const uint8_t curtain_control_char_uuid[] = { CURTAIN_CONTROL_CHAR_UUID & 0xFF, CURTAIN_CONTROL_CHAR_UUID >> 8 };
//static const uint8_t curtain_status_char_uuid[] = { CURTAIN_STATUS_CHAR_UUID & 0xFF, CURTAIN_STATUS_CHAR_UUID >> 8 };

#define PROFILE_A_APP_ID 0

// todo: make clear the count
#define GATTS_NUM_HANDLE 7


/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTS_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

// GATT server callback function
void esp_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT Server Registered");
            // Create a service here
            esp_ble_gatts_create_service(gatts_if, curtain_service_uuid, GATTS_NUM_HANDLE);
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(GATTS_TAG, "Service Created");
            // Add characteristics here
            esp_ble_gatts_add_char(param->create.service_handle, curtain_control_char_uuid,
                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                    NULL, NULL);
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT: {
            ESP_LOGI(GATTS_TAG, "Characteristic Added");
            // Start the service after adding characteristics
            esp_ble_gatts_start_service(param->add_char.service_handle);
            break;
        }
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(GATTS_TAG, "Read Request");
            // Handle read request here
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(GATTS_TAG, "Write Request");
            // Handle write request here
            break;
        }
        default:
            break;
    }
}

void ble_init(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // advertise this device name
    char *name = "Rabbitoose Smart Curtain";
    esp_ble_gap_set_device_name(name);
    esp_ble_adv_params_t adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.adv_int_min = 0x20;
    adv_params.adv_int_max = 0x40;
    adv_params.adv_type = ADV_TYPE_IND;
    adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map = ADV_CHNL_ALL;
    esp_ble_gap_start_advertising(&adv_params);

    esp_ble_adv_data_t adv_data;
    adv_data.set_scan_rsp = false;
    adv_data.include_name = true; 
    adv_data.min_interval = 0x100;  // Minimum advertising interval
    adv_data.max_interval = 0x200;  // Maximum advertising interval
    adv_data.appearance = 0x00;     // Appearance (optional)
    adv_data.manufacturer_len = 0;  // Manufacturer data length
    adv_data.p_manufacturer_data = NULL; // Manufacturer data
    adv_data.service_data_len = 0;  // Service data length
    adv_data.p_service_data = NULL;  // Service data
    adv_data.service_uuid_len = 0;   // Service UUID length
    adv_data.p_service_uuid = NULL;   // Service UUID
    adv_data.manufacturer_len = 6;
    adv_data.p_manufacturer_data = (uint8_t *)"Chingo";

    esp_ble_gap_config_adv_data(&adv_data);

    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "%s gatts app register failed, error code = %x", __func__, ret);
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTS_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gatts_register_callback(esp_gatts_cb);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s gatts register callback failed, error code = %x", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

}

// Function to handle control commands
void control_curtain(uint8_t command) {
    // Implement logic to open/close the curtain based on command
}

// Function to read the curtain status
uint8_t read_curtain_status() {
    // Implement logic to return the current status of the curtain
    return 0;
}