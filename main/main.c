/*
 * Copyright (c) 2023, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_bt.h>
#include <nvs_flash.h>
#include "sdkconfig.h"

static void bt_tx_pkt_ready(void);
static int bt_rx_pkt(uint8_t *data, uint16_t len);

static esp_vhci_host_callback_t vhci_host_cb = {
    bt_tx_pkt_ready,
    bt_rx_pkt
};

static void bt_tx_pkt_ready(void) {
}

static int bt_rx_pkt(uint8_t *data, uint16_t len) {
    return 0;
}

void app_main()
{
    printf("ESP32-BT-Dongle\n");

    /* Initialize NVS — it is used to store PHY calibration data */
    int32_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        printf("# Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM)) != ESP_OK) {
        printf("# Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_vhci_host_register_callback(&vhci_host_cb);
}
