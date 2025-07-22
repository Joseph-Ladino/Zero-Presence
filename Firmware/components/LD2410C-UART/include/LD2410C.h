#pragma once
#include "FrameParser.h"
#include <algorithm>
#include <atomic>
#include <bit>
#include <concepts>
#include <cstdint>
#include <span>
#include <vector>

namespace LD2410C {
    constexpr bool is_little_endian = std::endian::native == std::endian::little;

    extern void log_frame(std::span<uint8_t> frame);
    extern void log_frame(const char *tag, std::span<uint8_t> frame);

    template <typename T>
    concept uart_ctrl_req = requires(T t, uint8_t *data, uint32_t len, uint16_t timeout) {
        { t.write_bytes(data, len) } -> std::same_as<int>;
        { t.read_bytes(data, len, timeout) } -> std::same_as<int>;
        { t.set_baud_rate(uint16_t()) } -> std::same_as<bool>;
    };

    enum class sensor_baud_rate : uint16_t {
        BAUD_9600 = 0x0001,
        BAUD_19200 = 0x0002,
        BAUD_38400 = 0x0003,
        BAUD_57600 = 0x0004,
        BAUD_115200 = 0x0005,
        BAUD_230400 = 0x0006,
        BAUD_256000 = 0x0007,
        BAUD_460800 = 0x0008
    };

    enum class sensor_target_state : uint8_t {
        NONE = 0x00,
        MOVING = 0x01,
        STILL = 0x02,
        MOVING_AND_STILL = 0x03,
        NOISE_CALIBRATION_WIP = 0x04,
        NOISE_CALIBRATION_PASS = 0x05,
        NOISE_CALIBRATION_FAIL = 0x06
    };

    struct sensor_config {
        using enum sensor_baud_rate;

        bool config_mode_en = false;
        bool engr_mode_en = false;
        sensor_baud_rate baud_rate = BAUD_460800; // default
    };

    template <uart_ctrl_req uart_ctrl_t>
    struct PresenceSensor {
        uart_ctrl_t &uart;
        sensor_config config;

        std::array<uint8_t, CONFIG_SENSOR_UART_RX_BUFFER_SIZE> rx_buffer{0};
        FrameParser frame_parser{};

        std::atomic_flag command_sent{false}, // used to signal that a command is being sent
            command_response_ready{false};    // used to signal that a command response is ready
        std::vector<uint8_t> command_response_frame{};

        static constexpr std::array<uint8_t, 4> command_header{0xFD, 0xFC, 0xFB, 0xFA}, command_footer{0x04, 0x03, 0x02, 0x01};
        static constexpr std::array<uint8_t, 4> status_header{0xF4, 0xF3, 0xF2, 0xF1}, status_footer{0xF8, 0xF7, 0xF6, 0xF5};

        static constexpr std::vector<uint8_t> create_command_frame(sensor_command sensor_command, std::span<const uint8_t> data);

        std::vector<uint8_t> handle_status_frame(std::span<uint8_t> data);
        void handle_command_frame(std::span<uint8_t> data);

        std::optional<std::vector<uint8_t>> send_command(sensor_command sensor_command, std::span<const uint8_t> data);

    public:
        bool set_engineering_mode(bool enabled);
        bool set_bluetooth_mode(bool enabled);
        bool set_config_mode(bool enabled);

        bool set_baud_rate(sensor_baud_rate baud_rate);
        bool set_baud_rate(uint16_t baud_rate);

        bool restart();
        bool reset();

        void run();

        PresenceSensor(uart_ctrl_t &uart) : uart(uart) {}
    };

    template <uart_ctrl_req uart_ctrl_t>
    inline constexpr std::vector<uint8_t> PresenceSensor<uart_ctrl_t>::create_command_frame(sensor_command sensor_command, std::span<const uint8_t> data) {
        std::vector<uint8_t> out_frame{command_header.begin(), command_header.end()}; // frame header

        uint16_t data_len = static_cast<uint16_t>(2 + data.size()); // length includes 2 byte command
        uint16_t command = static_cast<uint16_t>(sensor_command);

        uint8_t command_upper, command_lower;
        uint8_t data_len_upper, data_len_lower;

        if constexpr (is_little_endian) {
            command_upper = (command >> 8) & 0xFF;
            command_lower = command & 0xFF;
            data_len_upper = (data_len >> 8) & 0xFF;
            data_len_lower = data_len & 0xFF;
        } else {
            command_upper = command & 0xFF;
            command_lower = (command >> 8) & 0xFF;
            data_len_upper = data_len & 0xFF;
            data_len_lower = (data_len >> 8) & 0xFF;
        }
        out_frame.insert(out_frame.end(), {data_len_lower, data_len_upper});             // (command+data) length
        out_frame.insert(out_frame.end(), {command_lower, command_upper});               // command
        out_frame.insert(out_frame.end(), data.begin(), data.end());                     // data
        out_frame.insert(out_frame.end(), command_footer.begin(), command_footer.end()); // frame footer

        return out_frame;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline std::vector<uint8_t> PresenceSensor<uart_ctrl_t>::handle_status_frame(std::span<uint8_t> frame) {

        // TODO: refactor maybe?? why return vector???
        // log_frame(frame);

        return std::vector<uint8_t>();
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::handle_command_frame(std::span<uint8_t> data) {
        // blocks until command_sent is true
        command_sent.wait(false);
        command_sent.clear();

        command_response_frame.clear();
        command_response_frame.assign(data.cbegin(), data.cend());

        command_response_ready.test_and_set();
        command_response_ready.notify_one();
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline std::optional<std::vector<uint8_t>> PresenceSensor<uart_ctrl_t>::send_command(sensor_command command, std::span<const uint8_t> data) {
        auto frame = create_command_frame(command, data);

        // blocks until command_sent is false
        // this is to ensure that only one command is sent at a time
        command_sent.wait(true);
        command_sent.test_and_set();

        log_frame("sent frame", std::span<uint8_t>{frame.data() + 6, frame.size() - 10}); // log without header and footer

        auto bytes_written = uart.write_bytes(frame.data(), frame.size());
        if (bytes_written != frame.size()) {
            // TODO: handle this
        }

        command_response_ready.wait(false); // wait for command response

        // data ready
        auto response = command_response_frame;
        command_response_ready.clear();

        log_frame("rcvd frame", response);

        // verify success

        auto response_command = FrameParser::get_command_from_frame(response);
        if (!response_command.has_value()) return std::nullopt; // no command in response

        auto expected_command = static_cast<sensor_command>(static_cast<uint16_t>(command) | 0x0100); // ACK command

        bool success = response_command.value() == expected_command && response.size() >= 4 && response[2] == 0x00 && response[3] == 0x00; // check if ACK and no error

        if(success)
            return std::vector<uint8_t> { response.begin() + 4, response.end() };

        return std::nullopt; // command failed, return empty optional
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline bool PresenceSensor<uart_ctrl_t>::set_engineering_mode(bool enabled) {
        constexpr std::array<uint8_t, 2> enable_config_data{0x01, 0x0};
        constexpr std::array<uint8_t, 0> disable_config_data{};

        std::optional<std::vector<uint8_t>> response;
        if (enabled)
            response = send_command(sensor_command::ENABLE_ENGINEERING_MODE, enable_config_data);
        else
            response = send_command(sensor_command::DISABLE_ENGINEERING_MODE, disable_config_data);

        if (!response.has_value()) {
            return false; // command failed
        }

        config.engr_mode_en = enabled;

        return true; // command succeeded
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline bool PresenceSensor<uart_ctrl_t>::set_bluetooth_mode(bool enabled) {
        
        constexpr std::array<uint8_t, 2> enable_bluetooth_data{0x01, 0x00};
        constexpr std::array<uint8_t, 2> disable_bluetooth_data{0x00, 0x00};

        auto response = send_command(enabled ? sensor_command::SET_BLUETOOTH : sensor_command::SET_BLUETOOTH, enabled ? enable_bluetooth_data : disable_bluetooth_data);

        if (!response.has_value()) {
            return false; // command failed
        }

        // maybe add config.bluetoot_en = enabled; // if needed

        return true;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline bool PresenceSensor<uart_ctrl_t>::set_config_mode(bool enabled) {

        auto response = send_command(enabled ? sensor_command::ENABLE_CONFIG_MODE : sensor_command::DISABLE_CONFIG_MODE, {});

        if (!response.has_value()) {
            return false; // command failed
        }

        config.config_mode_en = enabled;

        return true; // command succeeded
    }

    template <uart_ctrl_req uart_ctrl_t>
    bool LD2410C::PresenceSensor<uart_ctrl_t>::restart() {
        auto response = send_command(sensor_command::RESTART_SENSOR, {});
        
        return response.has_value(); // return true if command succeeded, false otherwise
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::run() {

        auto len_read = uart.read_bytes(rx_buffer.data(), rx_buffer.size(), 20);

        if (len_read < 1) return;

        auto rx_view = std::span<uint8_t>{rx_buffer.begin(), static_cast<uint32_t>(len_read)};

        for (auto it = rx_view.begin(); it != rx_view.end(); ++it) {
            uint8_t &f_byte = *it;

            frame_parser.parse_byte(f_byte);

            if (frame_parser.done()) {
                if (frame_parser.is_frame_command()) {
                    handle_command_frame(frame_parser.get_frame_data());
                } else if (frame_parser.is_frame_status()) {
                    handle_status_frame(frame_parser.get_frame_data());
                }

                frame_parser.reset();
            }

        } // namespace LD2410C
    }
}