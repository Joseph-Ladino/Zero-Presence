#pragma once
#include <FrameParser.h>
#include <algorithm>
#include <bit>
#include <concepts>
#include <cstdint>
#include <span>

namespace LD2410C {
    constexpr bool is_little_endian = std::endian::native == std::endian::little;

    template <typename T>
    concept uart_ctrl_req = requires(T t, uint8_t *data, uint32_t len, uint16_t timeout) {
        { t.write_bytes(data, len) } -> std::same_as<int>;
        { t.read_bytes(data, len, timeout) } -> std::same_as<int>;
        { t.set_baud_rate(uint16_t()) } -> std::same_as<bool>;
    };

    enum class sensor_command : uint16_t {
        ENABLE_CONFIG_MODE = 0x00FF,
        ENABLE_CONFIG_MODE_ACK = 0x01FF,

        DISABLE_CONFIG_MODE = 0x00FE,
        DISABLE_CONFIG_MODE_ACK = 0x01FE,

        SET_MAX_DISTANCE_DURATION = 0x0060,
        SET_MAX_DISTANCE_DURATION_ACK = 0x00160,

        GET_CONFIG = 0x0061,
        GET_CONFIG_ACK = 0x0161,

        ENABLE_ENGINEERING_MODE = 0x0062,
        ENABLE_ENGINEERING_MODE_ACK = 0x0162,

        DISABLE_ENGINEERING_MODE = 0x0063,
        DISABLE_ENGINEERING_MODE_ACK = 0x0163,

        SET_GATE_SENSITIVITY = 0x0064,
        SET_GATE_SENSITIVITY_ACK = 0x0164,

        GET_FIRMWARE_VERSION = 0x00A0,
        // GET_FIRMWARE_VERSION_ACK = 0x01A0, // no ack ??

        SET_SERIAL_BAUD = 0x00A1,
        SET_SERIAL_BAUD_ACK = 0x01A1,

        RESET_CONFIG = 0x00A2,
        RESET_CONFIG_ACK = 0x01A2,

        RESTART_SENSOR = 0x00A3,
        RESTART_SENSOR_ACK = 0x01A3,

        SET_BLUETOOTH = 0x00A4,
        SET_BLUETOOTH_ACK = 0x01A4,

        GET_MAC_ADDRESS = 0x00A5,
        GET_MAC_ADDRESS_ACK = 0x01A5,

        GET_BLUETOOTH_PERMISSIONS = 0x00A8,
        GET_BLUETOOTH_PERMISSIONS_ACK = 0x01A8,

        SET_BLUETOOTH_PASSWORD = 0x00A8,
        SET_BLUETOOTH_PASSWORD_ACK = 0x01A8,

        SET_DISTANCE_RESOLUTION = 0x00AA,
        SET_DISTANCE_RESOLUTION_ACK = 0x01A1, // 0x01AA

        GET_DISTANCE_RESOLUTION = 0x00AB,
        GET_DISTANCE_RESOLUTION_ACK = 0x01AB,

        SET_AUXILIARY_CONTROL = 0x00AD,
        SET_AUXILIARY_CONTROL_ACK = 0x01AD,

        GET_AUXILIARY_CONTROL = 0x00AE,
        GET_AUXILIARY_CONTROL_ACK = 0x01AE,

        START_NOISE_CALIBRATION = 0x000B,
        START_NOISE_CALIBRATION_ACK = 0x010B,

        GET_NOISE_CALIBRATION_STATUS = 0x001B,
        GET_NOISE_CALIBRATION_STATUS_ACK = 0x011B
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

        FramerParser frame_parser{};

        static constexpr std::array<uint8_t, 4> command_header{0xFD, 0xFC, 0xFB, 0xFA}, command_footer{0x04, 0x03, 0x02, 0x01};
        static constexpr std::array<uint8_t, 4> status_header{0xF4, 0xF3, 0xF2, 0xF1}, status_footer{0xF8, 0xF7, 0xF6, 0xF5};

        constexpr std::vector<uint8_t> static create_command_frame(sensor_command sensor_command, std::span<uint8_t> data);
        std::vector<uint8_t> static handle_status_frame(std::span<uint8_t> data);
        void static handle_command_frame(std::span<uint8_t> data);

        void write_frame(std::span<uint8_t> frame);

    public:
        void set_engineering_mode(bool enabled);
        void set_bluetooth_mode(bool enabled);
        void set_config_mode(bool enabled);

        void set_baud_rate(sensor_baud_rate baud_rate);
        void set_baud_rate(uint16_t baud_rate);

        void restart();
        void reset();

        void run();

        PresenceSensor(uart_ctrl_t &uart) : uart(uart) {}
    };

    template <uart_ctrl_req uart_ctrl_t>
    inline constexpr std::vector<uint8_t> PresenceSensor<uart_ctrl_t>::create_command_frame(sensor_command sensor_command, std::span<uint8_t> data) {
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

        return std::vector<uint8_t>();
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::handle_command_frame(std::span<uint8_t> data) {
        
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::write_frame(std::span<uint8_t> frame) {
        auto bytes_written = uart.write_bytes(frame.data(), frame.size_bytes());
        if (bytes_written != frame.size_bytes()) {
            // TODO: handle this
        }
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::set_config_mode(bool enabled) {
        constexpr std::array<uint8_t, 2> enable_config_data{0x01, 0x0};
        constexpr std::array<uint8_t, 0> disable_config_data{};

        if (enabled) {
            auto frame = create_command_frame(sensor_command::ENABLE_CONFIG_MODE, std::span(enable_config_data));
            write_frame(frame);
        } else {
            auto frame = create_command_frame(sensor_command::DISABLE_CONFIG_MODE, std::span(disable_config_data));
            write_frame(frame);
        }
    }

    /*
        This whole function should be refactored into a class and/or several static functions.

        TODO NOTE:
            Commands that need a response will somehow need to be dealt with.
            Maybe separate status frames from command responses somehow. Also, race conditions etc if run() is called from separate thread/cpu
            Restructuring necessary, core logic should work okay (untested, but builds)
    */
    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::run() {

        std::array<uint8_t, CONFIG_SENSOR_UART_RX_BUFFER_SIZE> rx_buffer{0};
        // std::size_t rx_buffer_written{0};

        while (true) {
            auto len_read = uart.read_bytes(rx_buffer.data(), rx_buffer.size(), 20);

            if (len_read < 1) continue;

            auto rx_view = std::span<uint8_t>{rx_buffer.begin(), rx_buffer.end() + len_read};

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
            }

        } // namespace LD2410C
    }
}