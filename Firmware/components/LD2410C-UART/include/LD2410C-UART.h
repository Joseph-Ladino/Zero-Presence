#pragma once
#include <concepts>
#include <cstdint>
#include <span>
#include <vector>

namespace LD2410C {

    consteval bool is_little_endian() {
        union {
            uint16_t num;
            uint8_t bytes[2];
        } u = {0x0001};
        return u.bytes[0] == uint8_t{0x01};
    };

    template <typename T>
    concept uart_ctrl_req = requires(T t, uint8_t *data, uint32_t len, uint16_t timeout) {
        { t.write_bytes(data, len) } -> std::same_as<int>;
        { t.read_bytes(data, len, timeout) } -> std::same_as<int>;
        { t.set_baud_rate(uint16_t()) } -> std::same_as<bool>;
        // { t.set_rx_pin(uint16_t()) } -> std::same_as<bool>;
        // { t.set_tx_pin(uint16_t()) } -> std::same_as<bool>;
    };

    enum class sensor_baud_rate_t : uint16_t {
        BAUD_9600 = 0x0001,
        BAUD_19200 = 0x0002,
        BAUD_38400 = 0x0003,
        BAUD_57600 = 0x0004,
        BAUD_115200 = 0x0005,
        BAUD_230400 = 0x0006,
        BAUD_256000 = 0x0007,
        BAUD_460800 = 0x0008
    };

    enum class sensor_target_state_t : uint8_t {
        NONE = 0x00,
        MOVING = 0x01,
        STILL = 0x02,
        MOVING_AND_STILL = 0x03,
        NOISE_CALIBRATION_WIP = 0x04,
        NOISE_CALIBRATION_PASS = 0x05,
        NOISE_CALIBRATION_FAIL = 0x06
    };

    struct presence_sensor_config_t {
        using enum sensor_baud_rate_t;

        bool config_mode_en = false;
        bool engr_mode_en = false;
        sensor_baud_rate_t baud_rate = BAUD_460800; // default
    };

    template <uart_ctrl_req uart_ctrl_t>
    class PresenceSensor {
        uart_ctrl_t &uart;
        presence_sensor_config_t config;

        constexpr std::vector<uint8_t> static create_command_frame(uint16_t command, std::span<uint8_t> data);
        std::vector<uint8_t> static parse_data_frame(std::span<uint8_t> data);

    public:
        void set_engineering_mode(bool enabled);
        void set_bluetooth_mode(bool enabled);
        void set_config_mode(bool enabled);

        void set_baud_rate(sensor_baud_rate_t baud_rate);
        void set_baud_rate(uint16_t baud_rate);

        void restart();
        void reset();

        PresenceSensor(uart_ctrl_t &uart) {}
    };

    template <uart_ctrl_req uart_ctrl_t>
    inline constexpr std::vector<uint8_t> PresenceSensor<uart_ctrl_t>::create_command_frame(uint16_t command, std::span<uint8_t> data) {
        std::vector<uint8_t> out_frame = {0xFD, 0xFC, 0xFB, 0xFA}; // frame header

        uint16_t endian_command, endian_size;
        uint8_t command_upper, command_lower;
        uint8_t data_len_upper, data_len_lower;

        // if constexpr (is_little_endian()) {
        //     command_upper = (command >> 8) & 0xFF;
        //     command_lower = command & 0xFF;
            
        // } else {
        //     endian_command = ((command & 0xFF) << 8) | ((command >> 8) & 0xFF);
        //     endian_size = ((data.size() & 0xFF) << 8) | ((data.size() >> 8) & 0xFF);
        // }

        // out_frame.insert(
        //     out_frame.back(),
        //     endian_command & 0xFF, (endian_command >> 8) & 0xFF, // command code
        //     endian_size & 0xFF, (endian_size >> 8) & 0xFF        // data length
        // );

        out_frame.insert(out_frame.end(), data.begin(), data.end()); // data
        out_frame.insert(out_frame.end(), {0x04, 0x03, 0x02, 0x01}); // frame footer

        return out_frame;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline std::vector<uint8_t> PresenceSensor<uart_ctrl_t>::parse_data_frame(std::span<uint8_t> data) {
        return std::vector<uint8_t>();
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::set_engineering_mode(bool enabled) {
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::set_bluetooth_mode(bool enabled) {
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::set_config_mode(bool enabled) {
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::set_baud_rate(sensor_baud_rate_t baud_rate) {
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::restart() {
    }

} // namespace LD2410C
