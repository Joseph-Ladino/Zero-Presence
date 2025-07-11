#pragma once
#include <concepts>
#include <cstdint>

namespace LD2410C {

    template <typename T>
    concept uart_ctrl_req = requires(T t, uint8_t *data, uint32_t len, uint16_t timeout) {
        { t.write_bytes(data, len) } -> std::same_as<int>;
        { t.read_bytes(data, len, timeout) } -> std::same_as<int>;
        { t.set_baud_rate(uint16_t()) } -> std::same_as<bool>;
        // { t.set_rx_pin(uint16_t()) } -> std::same_as<bool>;
        // { t.set_tx_pin(uint16_t()) } -> std::same_as<bool>;
    };

    template <uart_ctrl_req uart_ctrl_t>
    struct PresenceSensor {
        uart_ctrl_t& uart;
        
        PresenceSensor(uart_ctrl_t &uart) {
        }
    };

} // namespace LD2410C
