#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
#include <optional>

namespace LD2410C {

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
        GET_FIRMWARE_VERSION_ACK = 0x01A0, // no ack ??

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

    // converts back and forth between little-endian and big-endian representations
    constexpr uint16_t little_endian_conv(uint16_t value) {
        if (std::endian::native == std::endian::little)
            return value;
        return (value >> 8) | (value << 8);
    }

    // converts back and forth between little-endian and big-endian representations
    constexpr uint32_t little_endian_conv(uint32_t value) {
        if (std::endian::native == std::endian::little)
            return value;
        return ((value & 0x000000FF) << 24) | ((value & 0x0000FF00) << 8) |
               ((value & 0x00FF0000) >> 8) | ((value & 0xFF000000) >> 24);
    }

    // parses a single data frame from sensor
    class FrameParser {

        enum class parsing_state {
            HEADER,

            READ_SIZE_LOWER,
            READ_SIZE_UPPER,

            DATA,
            FOOTER
        };

        static constexpr std::array<uint8_t, 4> command_header{0xFD, 0xFC, 0xFB, 0xFA}, command_footer{0x04, 0x03, 0x02, 0x01};
        static constexpr std::array<uint8_t, 4> status_header{0xF4, 0xF3, 0xF2, 0xF1}, status_footer{0xF8, 0xF7, 0xF6, 0xF5};

        parsing_state state{parsing_state::HEADER};

        size_t command_header_found_idx{0}, status_header_found_idx{0};
        size_t command_footer_found_idx{0}, status_footer_found_idx{0};

        uint16_t frame_data_size{0};

        std::vector<uint8_t> frame_data{};

        bool is_status{false}, is_done{false};

    public:
        inline static constexpr std::optional<sensor_command> get_command_from_frame(std::span<const uint8_t> frame) {
            if (frame.size() < 2) return std::nullopt; // default command

            uint16_t command = (frame[1] << 8) | frame[0];
            return static_cast<sensor_command>(little_endian_conv(command));
        }

        void parse_byte(uint8_t);
        bool is_frame_command();
        bool is_frame_status();
        bool done();
        void reset();
        std::span<uint8_t> get_frame_data();

        FrameParser() = default;
    };
}