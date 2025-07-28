#pragma once
#include "FrameParser.h"
#include <algorithm>
#include <atomic>
#include <bit>
#include <concepts>
#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <span>
#include <string_view>
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

    enum class sensor_motion_state : uint8_t {
        NONE = 0x00,
        MOVING = 0x01,
        STILL = 0x02,
        MOVING_AND_STILL = 0x03,
        NOISE_CALIBRATION_WIP = 0x04,
        NOISE_CALIBRATION_PASS = 0x05,
        NOISE_CALIBRATION_FAIL = 0x06
    };

    enum class sensor_distance_resolution : uint8_t {
        RESOLUTION_75CM = 0x00,
        RESOLUTION_20CM = 0x01,
    };

    struct sensor_config {
        using enum sensor_baud_rate;

        bool config_mode_en = false;
        bool engr_mode_en = false;
        sensor_baud_rate baud_rate = BAUD_460800; // default
    };

    struct sensor_state {
        sensor_motion_state target_state{sensor_motion_state::NONE};
        uint16_t movement_distance{0};
        uint8_t movement_energy{0};
        uint16_t stationary_distance{0};
        uint8_t stationary_energy{0};
        uint16_t detection_distance{0};

        std::array<uint8_t, 9> motion_distance_energy{0}, stationary_distance_energy{0};
        uint8_t photosensitivy{0};
        bool output_status{false};
    };

    template <uart_ctrl_req uart_ctrl_t>
    struct PresenceSensor {
        uart_ctrl_t &uart;
        sensor_config config;

        FrameParser frame_parser{};

        std::atomic_flag command_sent{false}, // used to signal that a command is being sent
            command_response_ready{false};    // used to signal that a command response is ready
        std::vector<uint8_t> command_response_frame{};

        std::shared_mutex state_mutex;
        sensor_state current_state{};

        static constexpr std::array<uint8_t, 4> command_header{0xFD, 0xFC, 0xFB, 0xFA}, command_footer{0x04, 0x03, 0x02, 0x01};
        static constexpr std::array<uint8_t, 4> status_header{0xF4, 0xF3, 0xF2, 0xF1}, status_footer{0xF8, 0xF7, 0xF6, 0xF5};

        static constexpr std::vector<uint8_t> create_command_frame(sensor_command sensor_command, std::span<const uint8_t> data);

        void handle_status_frame(std::span<uint8_t> data);
        void handle_command_frame(std::span<uint8_t> data);

        std::optional<std::vector<uint8_t>> send_command(sensor_command sensor_command, std::span<const uint8_t> data);

    public:
        bool set_config_mode(bool enabled);
        bool set_engineering_mode(bool enabled);
        bool set_max_distance_duration(uint8_t movement_distance, uint8_t stationary_distance, uint16_t nobody_detected_timeout_s);

        bool set_baud_rate(sensor_baud_rate baud_rate);
        bool set_baud_rate(uint16_t baud_rate);

        bool reset();
        bool restart();

        std::optional<std::array<uint8_t, 6>> get_mac_address();

        bool set_bluetooth_mode(bool enabled);
        bool set_bluetooth_password(std::string_view password);

        bool set_distance_resolution(sensor_distance_resolution resolution);
        std::optional<sensor_distance_resolution> get_distance_resolution();

        // getters for current state
        sensor_state get_state();
        sensor_motion_state get_motion_state();
        uint16_t get_movement_distance();
        uint8_t get_movement_energy();
        uint16_t get_stationary_distance();
        uint8_t get_stationary_energy();
        uint16_t get_detection_distance();

        std::array<uint8_t, 9> get_motion_distance_energy();
        std::array<uint8_t, 9> get_stationary_distance_energy();
        uint8_t get_photosensitivy();
        bool get_output_status();

        void run();

        PresenceSensor(uart_ctrl_t &uart) : uart(uart) {}
    };

} // namespace LD2410C

#include "LD2410C.tpp"