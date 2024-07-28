
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "ble.h"
#include "bdc.h"

#define GATTS_TAG "GATTS"

// use 4347????-0000-1000-8000-00805F9B34FB
// Curtain Control Service UUID
// there is no service id assigned for smart curtain, so use generic GATT service id
const uint16_t smart_curtain_service_uuid = 0xfff0;
// Curtain Position Characteristic UUID
const uint8_t curtain_position_char_uuid[] =    {0x43, 0x47, 0x00, 0x01, 0x00, 0x00, 0x10, 0x00, 
                                                 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
// Curtain Movement Characteristic UUID
const uint8_t curtain_movement_char_uuid[] =    {0x43, 0x47, 0x00, 0x02, 0x00, 0x00, 0x10, 0x00, 
                                                 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
// Curtain Speed Characteristic UUID
const uint8_t curtain_speed_char_uuid[] =       {0x43, 0x47, 0x00, 0x03, 0x00, 0x00, 0x10, 0x00, 
                                                 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
// Curtain State Characteristic UUID
const uint8_t curtain_state_char_uuid[] =       {0x43, 0x47, 0x00, 0x04, 0x00, 0x00, 0x10, 0x00, 
                                                 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
// Curtain Obstruction Detection Characteristic UUID
const uint8_t curtain_obstruction_char_uuid[] = {0x43, 0x47, 0x00, 0x05, 0x00, 0x00, 0x10, 0x00, 
                                                 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
// Curtain Battery Level Characteristic UUID
const uint8_t curtain_calibration_char_uuid[] = {0x43, 0x47, 0x00, 0x06, 0x00, 0x00, 0x10, 0x00, 
                                                   0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
// Curtain Calibration Characteristic UUID
const uint8_t curtain_timer_open_char_uuid[] = {0x43, 0x47, 0x00, 0x07, 0x00, 0x00, 0x10, 0x00, 
                                                 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
// Curtain Timer Characteristic UUID
const uint8_t curtain_timer_close_char_uuid[] =       {0x43, 0x47, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 
                                                 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
// Curtain Light Sensor Characteristic UUID
const uint8_t curtain_light_sensor_char_uuid[] = {0x43, 0x47, 0x00, 0x09, 0x00, 0x00, 0x10, 0x00, 
                                                  0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};

uint16_t curtain_handle_table[HRS_IDX_NB];

// Characteristic values
// Allows setting and reading the curtain position (0% for fully closed, 100% for fully open).
static uint8_t curtain_position = 100;
// Commands to open, close, or stop the curtain (0x01 for open, 0x02 for close, 0x03 for stop)
static uint8_t curtain_movement = 3;
// Adjusts the speed of opening/closing the curtain (1-5).
static uint8_t curtain_speed = 3;
// Reports the current state of the curtain (0x00 for stopped, 0x01 for opening, 0x02 for closing).
static uint8_t curtain_state = 0;
// Indicates if an obstruction is detected (0 for no, 1 for yes).
static uint8_t curtain_obstruction = 0;
// Reports the battery level of the curtain opener.
static uint8_t curtain_timer_open = 0;
// Allows calibration of the curtain opener to ensure accurate positioning.
static uint8_t curtain_calibration = 0;
// Allows setting timers for automatic opening and closing at specified times (e.g., open at 7 AM, close at 10 PM).
static uint8_t curtain_timer_close = 0;
// Adjusts curtain behavior based on ambient light conditions.
static uint8_t curtain_light_sensor = 0;

// Notification status
static bool curtain_state_notify_enabled = false;
static bool curtain_obstruction_notify_enabled = false;
static bool curtain_battery_level_notify_enabled = false;
static bool curtain_light_sensor_notify_enabled = false;

static uint32_t passkey = 798301;
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
//static const uint16_t character_desc_uuid = ESP_GATT_UUID_CHAR_DESCRIPTION;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

//static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
//static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
//static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//static const uint8_t char_prop_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE_NR |ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

// Attribute Table
static const esp_gatts_attr_db_t gatt_db[] = {
    // Service Declaration
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
        sizeof(uint16_t), sizeof(smart_curtain_service_uuid), (uint8_t *)&smart_curtain_service_uuid}
    },

    // Curtain Position Characteristic Declaration
    [IDX_CHAR_POSITION] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}
    },

    // Curtain Position Characteristic Value
    [IDX_CHAR_POSITION_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_position_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint8_t), sizeof(curtain_position), &curtain_position}
    },

    // Curtain Movement Characteristic Declaration
    [IDX_CHAR_MOVEMENT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}
    },

    // Curtain Movement Characteristic Value
    [IDX_CHAR_MOVEMENT_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_movement_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint8_t), sizeof(curtain_movement), &curtain_movement}
    },

    // Curtain Speed Characteristic Declaration
    [IDX_CHAR_SPEED] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}
    },

    // Curtain Speed Characteristic Value
    [IDX_CHAR_SPEED_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_speed_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint8_t), sizeof(curtain_speed), &curtain_speed}
    },

    // Curtain State Characteristic Declaration
    [IDX_CHAR_STATE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}
    },

    // Curtain State Characteristic Value
    [IDX_CHAR_STATE_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_state_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint8_t), sizeof(curtain_state), &curtain_state}
    },

    // Curtain State Characteristic Configuration
    [IDX_CHAR_STATE_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(curtain_state_notify_enabled), (uint8_t *)&curtain_state_notify_enabled}
    },

    // Curtain Obstruction Characteristic Declaration
    [IDX_CHAR_OBSTRUCTION] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}
    },

    // Curtain Obstruction Characteristic Value
    [IDX_CHAR_OBSTRUCTION_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_obstruction_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint8_t), sizeof(curtain_obstruction), &curtain_obstruction}
    },

    // Curtain Obstruction Characteristic Configuration
    [IDX_CHAR_OBSTRUCTION_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(curtain_obstruction_notify_enabled), (uint8_t *)&curtain_obstruction_notify_enabled}
    },

    // Curtain Calibration Characteristic Declaration
    [IDX_CHAR_CALIBRATION] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}
    },

    // Curtain Calibration Characteristic Value
    [IDX_CHAR_CALIBRATION_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_calibration_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint8_t), sizeof(curtain_calibration), &curtain_calibration}
    },

    // Curtain Timer Open Characteristic Declaration
    [IDX_CHAR_TIMER_OPEN] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}
    },

    // Curtain Timer Open Characteristic Value
    [IDX_CHAR_TIMER_OPEN_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_timer_open_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint8_t), sizeof(curtain_timer_open), &curtain_timer_open}
    },
    
    // Curtain Timer Close Characteristic Declaration
    [IDX_CHAR_TIMER_CLOSE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}
    },

    // Curtain TimerClose Characteristic Value
    [IDX_CHAR_TIMER_CLOSE_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_timer_close_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint8_t), sizeof(curtain_timer_close), &curtain_timer_close}
    },

    // Curtain Light Sensor Characteristic Declaration
    [IDX_CHAR_LIGHT_SENSOR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}
    },

    // Curtain Light Sensor Characteristic Value
    [IDX_CHAR_LIGHT_SENSOR_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, curtain_light_sensor_char_uuid, ESP_GATT_PERM_READ,
        sizeof(uint8_t), sizeof(curtain_light_sensor), &curtain_light_sensor}
    },

    // Curtain Light Sensor Characteristic Configuration
    [IDX_CHAR_LIGHT_SENSOR_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(curtain_light_sensor_notify_enabled), (uint8_t *)&curtain_light_sensor_notify_enabled}
    }
};

/* Declare static functions */
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
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(GATTS_TAG, "Authentication success");
        } else {
            ESP_LOGE(GATTS_TAG, "Authentication failed, reason: %d", param->ble_security.auth_cmpl.fail_reason);
        }
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(GATTS_TAG, "Passkey request");
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, passkey);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        ESP_LOGI(GATTS_TAG, "Numeric Comparison request, passkey: %lu", param->ble_security.key_notif.passkey);
        // Confirm or reject the numeric comparison value
        esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        break;

    default:
        break;
    }
}

// GATT server callback function
void esp_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_ADD_CHAR_EVT: {
            ESP_LOGI(GATTS_TAG, "Characteristic Added");
            // Start the service after adding characteristics
            esp_ble_gatts_start_service(param->add_char.service_handle);
            break;
        }
        case ESP_GATTS_REG_EVT:
            // Set the device name and create the attribute table
            esp_ble_gap_set_device_name("Smart Curtain Opener");
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, 0);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB) {
                ESP_LOGE(GATTS_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(curtain_handle_table, param->add_attr_tab.handles, sizeof(curtain_handle_table));
                esp_ble_gatts_start_service(curtain_handle_table[IDX_SVC]);
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            // Initiate security upon connection
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
            break;

        case ESP_GATTS_READ_EVT: {
            uint16_t handle = param->read.handle;
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = handle;
            rsp.attr_value.len = 1; // Assuming all characteristics are 1 byte for simplicity

            if (handle == curtain_handle_table[IDX_CHAR_POSITION_VAL]) {
                rsp.attr_value.value[0] = curtain_position;
            } else if (handle == curtain_handle_table[IDX_CHAR_SPEED_VAL]) {
                rsp.attr_value.value[0] = curtain_speed;
            } else if (handle == curtain_handle_table[IDX_CHAR_STATE_VAL]) {
                rsp.attr_value.value[0] = curtain_state;
            } else if (handle == curtain_handle_table[IDX_CHAR_OBSTRUCTION_VAL]) {
                rsp.attr_value.value[0] = curtain_obstruction;
            } else if (handle == curtain_handle_table[IDX_CHAR_TIMER_OPEN_VAL]) {
                rsp.attr_value.value[0] = curtain_timer_open;
            } else if (handle == curtain_handle_table[IDX_CHAR_TIMER_CLOSE_VAL]) {
                rsp.attr_value.value[0] = curtain_timer_close;
            } else if (handle == curtain_handle_table[IDX_CHAR_LIGHT_SENSOR_VAL]) {
                rsp.attr_value.value[0] = curtain_light_sensor;
            }

            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                uint16_t handle = param->write.handle;
                uint8_t *value = param->write.value;
                uint16_t len = param->write.len;

                if (handle == curtain_handle_table[IDX_CHAR_POSITION_VAL]) {
                    curtain_position = value[0];
                    if (curtain_position == 100) {
                        bdc_down();
                    }
                    else if (curtain_position == 0) {
                        bdc_up();
                    }
                    else {
                        bdc_stop();
                    }
                } else if (handle == curtain_handle_table[IDX_CHAR_MOVEMENT_VAL]) {
                    curtain_movement = value[0];
                } else if (handle == curtain_handle_table[IDX_CHAR_SPEED_VAL]) {
                    curtain_speed = value[0];
                } else if (handle == curtain_handle_table[IDX_CHAR_CALIBRATION_VAL]) {
                    curtain_calibration = value[0];
                } else if (handle == curtain_handle_table[IDX_CHAR_TIMER_OPEN_VAL]) {
                    curtain_timer_open = value[0];
                } else if (handle == curtain_handle_table[IDX_CHAR_TIMER_CLOSE_VAL]) {
                    curtain_timer_close = value[0];
                } else if (handle == curtain_handle_table[IDX_CHAR_STATE_VAL] + 1) {
                    if (value[0] == 1) {
                        curtain_state_notify_enabled = true;
                    } else {
                        curtain_state_notify_enabled = false;
                    }
                } else if (handle == curtain_handle_table[IDX_CHAR_OBSTRUCTION_VAL] + 1) {
                    if (value[0] == 1) {
                        curtain_obstruction_notify_enabled = true;
                    } else {
                        curtain_obstruction_notify_enabled = false;
                    }
                } else if (handle == curtain_handle_table[IDX_CHAR_LIGHT_SENSOR_VAL] + 1) {
                    if (value[0] == 1) {
                        curtain_light_sensor_notify_enabled = true;
                    } else {
                        curtain_light_sensor_notify_enabled = false;
                    }
                }

                // Send response if needed
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            break;
        // Handle other GATTS events here
        default:
            break;
    }
}

static void gap_security_init() {
    // Set GAP security parameters
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;  // Authentication request: Secure Connection, MITM protection, and bonding
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;                    // IO capabilities: No IO capabilities

    uint8_t key_size = 16;                                       // Encryption key size
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;  // Initiator key distribution
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;  // Responder key distribution

    // Set the device as non-connectable until security is configured
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
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
    adv_data.appearance = BLE_APPEARANCE_WINDOW_CURTAIN;     // Appearance (optional)
    adv_data.manufacturer_len = 0;  // Manufacturer data length
    adv_data.p_manufacturer_data = NULL; // Manufacturer data
    adv_data.service_data_len = 0;  // Service data length
    adv_data.p_service_data = NULL;  // Service data
    adv_data.service_uuid_len = 0;   // Service UUID length
    adv_data.p_service_uuid = NULL;   // Service UUID
    adv_data.manufacturer_len = 6;
    adv_data.p_manufacturer_data = (uint8_t *)"Chingo";

    esp_ble_gap_config_adv_data(&adv_data);

    ret = esp_ble_gatts_register_callback(esp_gatts_cb);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s gatts register callback failed, error code = %x", __func__, ret);
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTS_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gatts_app_register(0);
    if (ret){
        ESP_LOGE(GATTS_TAG, "%s gatts app register failed, error code = %x", __func__, ret);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    // Set BLE appearance
    uint16_t appearance = BLE_APPEARANCE_WINDOW_CURTAIN;
    ret = esp_ble_gap_config_local_icon(appearance);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Failed to set BLE appearance, error code = %x", ret);
    }

    // Initialize security features
    gap_security_init();
}
