/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "zk01_pin.h"
#include "bdc.h"
#include "set_button.h"
#include "ble.h"


void gpio_init() {
    gpio_config_t conf = {};

    // pins used as gpio output
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_down_en = 0;
    conf.pull_up_en = 0;
    conf.pin_bit_mask = (1U << RS485_DIR_PIN) | 
                        (1U << UP_EN_PIN) | 
                        (1U << DOWN_EN_PIN) | 
                        (1U << LED_RUN1_PIN) |
                        0;
    gpio_config(&conf);

    // pins used as gpio input
    conf.mode = GPIO_MODE_INPUT;
    conf.pin_bit_mask = (1U << UP_PIN) | (1U << DOWN_PIN) | (1U << SET_PIN);
    gpio_config(&conf);

    // pins used as ext interrupt
    conf.intr_type = GPIO_INTR_POSEDGE;
    conf.mode = GPIO_MODE_INPUT;
    conf.pin_bit_mask = (1U << STALL_PIN) | (1U << P_E0_PIN) | (1U << P_E1_PIN);
    gpio_config(&conf);

}

void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    uint32_t cnt = 0;
    printf("Init gpio pins.\n");
    gpio_init();
    bdc_init();
    set_button_init();
    ble_init();

    while(1) {
        cnt ++;
        //printf("Toggling system led.\n");
        //printf("/*%d*/\r\n", motor_ctrl_ctx.report_pulses);
        gpio_set_level(LED_RUN1_PIN, cnt % 2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        fflush(stdout);
    }
}
