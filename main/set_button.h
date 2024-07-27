#ifndef SET_BUTTON_H
#define SET_BUTTON_H
#include "iot_button.h"
#include "bdc.h"
#include "esp_log.h"

static const char *TAG = "set_button";
button_handle_t set_btn = NULL;
static int dir = 0;

void set_btn_single_click_cb(void *arg,void *usr_data) {
    ESP_LOGI(TAG, "Set button single click callback.");
    if (dir == 0) {
        bdc_up();
        dir = 1;
    }
    else if (dir == 1) {
        bdc_down();
        dir = 2;
    }
    else {
        bdc_stop();
        dir = 0;
    }
}

void set_btn_double_click_cb(void *arg,void *usr_data) {
    ESP_LOGI(TAG, "Set button double click callback.");
    bdc_down();
}

void set_button_init() {
    ESP_LOGI(TAG, "Init SET button.");
    button_config_t button_config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = SET_PIN,
            .active_level = 0,
        },
        .long_press_time = 1500,
        .short_press_time = 100,
    };
    set_btn = iot_button_create(&button_config);
    if(NULL == set_btn) {
        ESP_LOGE(TAG, "Button create failed");
    }
    iot_button_register_cb(set_btn, BUTTON_SINGLE_CLICK, set_btn_single_click_cb, NULL);
    iot_button_register_cb(set_btn, BUTTON_DOUBLE_CLICK, set_btn_double_click_cb, NULL);
}
#endif // SET_BUTTON_H