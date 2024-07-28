#ifndef BLE_H
#define BLE_H

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// Appearance value for a Window Curtain
#define BLE_APPEARANCE_WINDOW_CURTAIN ESP_BLE_APPEARANCE_GENERIC_REMOTE

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

enum {
    IDX_SVC,
    IDX_CHAR_POSITION,
    IDX_CHAR_POSITION_VAL,
    
    IDX_CHAR_MOVEMENT,
    IDX_CHAR_MOVEMENT_VAL,

    IDX_CHAR_SPEED,
    IDX_CHAR_SPEED_VAL,

    IDX_CHAR_STATE,
    IDX_CHAR_STATE_VAL,
    IDX_CHAR_STATE_CFG,

    IDX_CHAR_OBSTRUCTION,
    IDX_CHAR_OBSTRUCTION_VAL,
    IDX_CHAR_OBSTRUCTION_CFG,

    IDX_CHAR_CALIBRATION,
    IDX_CHAR_CALIBRATION_VAL,

    IDX_CHAR_TIMER_OPEN,
    IDX_CHAR_TIMER_OPEN_VAL,

    IDX_CHAR_TIMER_CLOSE,
    IDX_CHAR_TIMER_CLOSE_VAL,

    IDX_CHAR_LIGHT_SENSOR,
    IDX_CHAR_LIGHT_SENSOR_VAL,
    IDX_CHAR_LIGHT_SENSOR_CFG,

    HRS_IDX_NB,
};

// Initialize BLE
void ble_init();
// Disconnect from a BLE device
void ble_disconnect(esp_bd_addr_t bd_addr);

#endif /* BLE_H */