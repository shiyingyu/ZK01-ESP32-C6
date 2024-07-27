#ifndef BLE_H
#define BLE_H

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// Initialize BLE
void ble_init();
void curtain_service_init();

// Start BLE scanning
void ble_start_scan();

// Stop BLE scanning
void ble_stop_scan();

// Connect to a BLE device
void ble_connect(esp_bd_addr_t bd_addr);

// Disconnect from a BLE device
void ble_disconnect(esp_bd_addr_t bd_addr);

#endif /* BLE_H */