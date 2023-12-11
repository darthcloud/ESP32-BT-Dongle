/*
 * Copyright (c) 2023, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_bt.h>
#include <driver/uart.h>
#include <nvs_flash.h>
#include "sdkconfig.h"

#define BT_HCI_H4_TYPE_CMD 0x01
#define BT_HCI_H4_TYPE_ACL 0x02
#define BT_HCI_H4_TYPE_SCO 0x03
#define BT_HCI_H4_TYPE_EVT 0x04
#define BT_HCI_H4_TYPE_ISO 0x05
#define UART_RX_BUFFER (8 * 1024)

struct bt_hci_evt_hdr {
	uint8_t  evt;
	uint8_t  len;
} __packed;

struct bt_hci_acl_hdr {
	uint16_t handle;
	uint16_t len;
} __packed;

struct bt_hci_cmd_hdr {
	uint16_t opcode;
	uint8_t  param_len;
} __packed;

struct bt_hci_pkt {
    uint8_t h4_type;
    union {
        struct {
            struct bt_hci_cmd_hdr cmd_hdr;
            uint8_t cmd_data[0];
        };
        struct {
            struct bt_hci_acl_hdr acl_hdr;
            uint8_t acl_data[0];
        };
        struct {
            struct bt_hci_evt_hdr evt_hdr;
            uint8_t evt_data[0];
        };
    };
} __packed;

static void bt_tx_pkt_ready(void);
static int bt_rx_pkt(uint8_t *data, uint16_t len);

static uint8_t uart_buffer[UART_RX_BUFFER];
static esp_vhci_host_callback_t vhci_host_cb = {
    bt_tx_pkt_ready,
    bt_rx_pkt
};

static void bt_tx_pkt_ready(void) {
    uart_set_rts(UART_NUM_0, 1);
}

static int bt_rx_pkt(uint8_t *data, uint16_t len) {
    uart_write_bytes(UART_NUM_0, data, len);
    return 0;
}

static void hci_rx_task(void *arg) {
    int32_t rx_len;
    struct bt_hci_pkt *pkt = (struct bt_hci_pkt *)uart_buffer;

    printf("# %s: ready\n", __FUNCTION__);

    while (1) {
        rx_len = uart_read_bytes(UART_NUM_0, &pkt->h4_type, sizeof(pkt->h4_type), portMAX_DELAY);
        if (rx_len) {
            switch (pkt->h4_type) {
                case BT_HCI_H4_TYPE_CMD:
                rx_len += uart_read_bytes(UART_NUM_0, &pkt->cmd_hdr, sizeof(pkt->cmd_hdr), portMAX_DELAY);
                if (rx_len == (1 +sizeof(pkt->cmd_hdr))) {
                    rx_len += uart_read_bytes(UART_NUM_0, pkt->cmd_data, pkt->cmd_hdr.param_len, portMAX_DELAY);
                    uart_set_rts(UART_NUM_0, 0);
                    esp_vhci_host_send_packet(uart_buffer, rx_len);
                }
                break;
                case BT_HCI_H4_TYPE_ACL:
                rx_len += uart_read_bytes(UART_NUM_0, &pkt->acl_hdr, sizeof(pkt->acl_hdr), portMAX_DELAY);
                if (rx_len == (1 + sizeof(pkt->acl_hdr))) {
                    rx_len += uart_read_bytes(UART_NUM_0, pkt->acl_data, pkt->acl_hdr.len, portMAX_DELAY);
                    uart_set_rts(UART_NUM_0, 0);
                    esp_vhci_host_send_packet(uart_buffer, rx_len);
                }
                break;
                case BT_HCI_H4_TYPE_SCO:
                    printf("Unsupported H4 type: SCO\n");
                    break;
                break;
                case BT_HCI_H4_TYPE_EVT:
                rx_len += uart_read_bytes(UART_NUM_0, &pkt->evt_hdr, sizeof(pkt->evt_hdr), portMAX_DELAY);
                if (rx_len == (1 + sizeof(pkt->evt_hdr))) {
                    rx_len += uart_read_bytes(UART_NUM_0, pkt->evt_data, pkt->evt_hdr.len, portMAX_DELAY);
                    uart_set_rts(UART_NUM_0, 0);
                    esp_vhci_host_send_packet(uart_buffer, rx_len);
                }
                break;
                case BT_HCI_H4_TYPE_ISO:
                    printf("Unsupported H4 type: ISO\n");
                break;
                default:
                    printf("Invalid H4 type: %02X\n", pkt->h4_type);
                    break;
            }
        }
    }
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

    uart_driver_install(UART_NUM_0, sizeof(uart_buffer), 0, 0, NULL, 0);

    xTaskCreatePinnedToCore(hci_rx_task, "hci_rx_task", 4096, NULL, 5, NULL, 0);
}
