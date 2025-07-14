#pragma once

#include "LD2410C.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace LD2410C {
    struct ESP32_UART_Adapter {
        uart_port_t port_num;
        uart_config_t config;

        int write_bytes(uint8_t *data, uint32_t len);
        int read_bytes(uint8_t *data, uint32_t len, uint16_t timeout);
        bool set_baud_rate(uint16_t baud_rate);

        ESP32_UART_Adapter();
        ESP32_UART_Adapter(uart_port_t port_num, uart_config_t config, uint16_t rx_pin, uint16_t tx_pin);
    };

    inline void log_frame(std::span<uint8_t> frame) {
        char *hex = new char[frame.size() * 3];
        constexpr auto fmt = "%02X ";
        constexpr auto fmt_render_len = 3;

        for (size_t i = 0; i < frame.size(); i++)
            sprintf(hex + i * fmt_render_len, fmt, frame[i]);

        hex[frame.size() * fmt_render_len - 1] = '\0';

        ESP_LOGI("LD2410C frame", "{ %s }", hex);

        delete[] hex;
    }
}