#pragma once

#include "LD2410C.h"
namespace LD2410C {

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
    inline void PresenceSensor<uart_ctrl_t>::handle_status_frame(std::span<uint8_t> frame) {

        // lots of magic numbers here, but they are pulled from the datasheet

        // invalid status frame
        if (frame.size() < 13 || !(frame[1] == 0xAA && frame[frame.size() - 2] == 0x55)) {
            return;
        }

        bool is_engineering_mode = frame[0] == 0x01;

        // skip first byte, header, footer, and empty trailing byte
        auto target_data = frame.subspan(2, frame.size() - 4);

        auto target_status = static_cast<sensor_motion_state>(target_data[0]);

        auto movement_distance = bytes_to_uint16(target_data[1], target_data[2]);
        auto movement_energy = target_data[3];

        auto stationary_distance = bytes_to_uint16(target_data[4], target_data[5]);
        auto stationary_energy = target_data[6];

        auto detection_distance = bytes_to_uint16(target_data[7], target_data[8]);

        std::unique_lock lock(state_mutex);

        current_state.target_state = target_status;
        current_state.movement_distance = movement_distance;
        current_state.movement_energy = movement_energy;
        current_state.stationary_distance = stationary_distance;
        current_state.stationary_energy = stationary_energy;
        current_state.detection_distance = detection_distance;

        if (is_engineering_mode) {
            std::copy(target_data.begin() + 9, target_data.begin() + 18, current_state.motion_distance_energy.begin());
            std::copy(target_data.begin() + 19, target_data.begin() + 28, current_state.stationary_distance_energy.begin());
            current_state.photosensitivy = target_data[28];
            current_state.output_status = target_data[29];
        }
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

        if (success)
            return std::vector<uint8_t>{response.begin() + 4, response.end()};

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
    bool PresenceSensor<uart_ctrl_t>::set_bluetooth_password(std::string_view password) {
        // password must be 6 characters long
        if (password.size() != 6) {
            // ESP_LOGE("LD2410C", "Bluetooth password must be 6 characters long.");

            return false;
        }

        auto response = send_command(sensor_command::SET_BLUETOOTH_PASSWORD, password);

        if (!response.has_value()) {
            // ESP_LOGE("LD2410C", "Failed to set Bluetooth password.");
            return false; // command failed
        }

        return true;
    }

    template <uart_ctrl_req uart_ctrl_t>
    bool PresenceSensor<uart_ctrl_t>::set_distance_resolution(sensor_distance_resolution resolution) {

        auto response = send_command(sensor_command::SET_DISTANCE_RESOLUTION, {static_cast<uint8_t>(resolution)});

        return response.has_value(); // return true if command succeeded, false otherwise
    }

    template <uart_ctrl_req uart_ctrl_t>
    std::optional<sensor_distance_resolution> PresenceSensor<uart_ctrl_t>::get_distance_resolution() {

        auto response = send_command(sensor_command::GET_DISTANCE_RESOLUTION, {});

        if (!response.has_value() || response->size() != 1) {
            return std::nullopt; // command failed or invalid response
        }

        if (response->front() > static_cast<uint8_t>(sensor_distance_resolution::RESOLUTION_20CM))
            return std::nullopt; // invalid resolution

        return static_cast<sensor_distance_resolution>(response->front());
    }

    template <uart_ctrl_req uart_ctrl_t>
    std::optional<std::array<uint8_t, 6>> PresenceSensor<uart_ctrl_t>::get_mac_address() {
        auto response = send_command(sensor_command::GET_MAC_ADDRESS, {1});

        if (!response.has_value() || response->size() != 6) {
            return std::nullopt; // command failed or invalid response
        }

        return std::array<uint8_t, 6>{response->begin(), response->end()}; // return MAC address
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
    inline bool PresenceSensor<uart_ctrl_t>::set_max_distance_duration(uint8_t movement_distance, uint8_t stationary_distance, uint16_t nobody_detected_timeout_s) {
        if (movement_distance < 2 || movement_distance > 8 ||
            stationary_distance < 2 || stationary_distance > 8) {
            return false; // invalid parameters
        }

        auto timeout = little_endian_conv(nobody_detected_timeout_s);

        std::array<uint8_t, 18> data{
            // movement distance gate
            0x00, 0x00, movement_distance, 0x00, 0x00, 0x00,

            // stationary distance gate
            0x01, 0x00, stationary_distance, 0x00, 0x00, 0x00,

            // nobody detected timeout
            0x02, 0x00,
            static_cast<uint8_t>(nobody_detected_timeout_s & 0xFF),        // lower byte
            static_cast<uint8_t>((nobody_detected_timeout_s >> 8) & 0xFF), // upper byte
            0x00, 0x00};

        return send_command(sensor_command::SET_MAX_DISTANCE_DURATION, data).has_value();
    }

    template <uart_ctrl_req uart_ctrl_t>
    bool PresenceSensor<uart_ctrl_t>::set_baud_rate(sensor_baud_rate baud_rate) {
        auto response = send_command(sensor_command::SET_SERIAL_BAUD, {static_cast<uint8_t>(baud_rate)});

        if (!response.has_value()) {
            return false; // command failed
        }

        // restart module to apply new baud rate
        if (!restart()) {
            return false; // restart failed
        }

        config.baud_rate = baud_rate;

        // set the new baud rate in the UART controller
        return uart.set_baud_rate(static_cast<uint16_t>(baud_rate));
    }

    template <uart_ctrl_req uart_ctrl_t>
    bool PresenceSensor<uart_ctrl_t>::set_baud_rate(uint16_t baud_rate) {
        // convert baud rate to sensor_baud_rate enum
        auto sensor_baud = static_cast<sensor_baud_rate>(baud_rate);

        // check if the baud rate is valid
        if (sensor_baud < sensor_baud_rate::BAUD_9600 || sensor_baud > sensor_baud_rate::BAUD_460800) {
            return false; // invalid baud rate
        }

        return set_baud_rate(sensor_baud);
    }

    template <uart_ctrl_req uart_ctrl_t>
    bool PresenceSensor<uart_ctrl_t>::restart() {
        auto response = send_command(sensor_command::RESTART_SENSOR, {});

        return response.has_value(); // return true if command succeeded, false otherwise
    }

    template <uart_ctrl_req uart_ctrl_t>
    bool PresenceSensor<uart_ctrl_t>::reset() {
        auto response = send_command(sensor_command::RESET_CONFIG, {});

        if (!response.has_value()) {
            return false; // command failed
        }

        // reset the config
        config = sensor_config{};
        uart.set_baud_rate(static_cast<uint16_t>(config.baud_rate)); // reset baud rate

        return true; // command succeeded
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline sensor_state PresenceSensor<uart_ctrl_t>::get_state() {
        std::shared_lock lock(state_mutex);
        return current_state;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline sensor_motion_state PresenceSensor<uart_ctrl_t>::get_motion_state() {
        std::shared_lock lock(state_mutex);
        return current_state.target_state;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline uint16_t PresenceSensor<uart_ctrl_t>::get_movement_distance() {
        std::shared_lock lock(state_mutex);
        return current_state.movement_distance;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline uint8_t PresenceSensor<uart_ctrl_t>::get_movement_energy() {
        std::shared_lock lock(state_mutex);
        return current_state.movement_energy;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline uint16_t PresenceSensor<uart_ctrl_t>::get_stationary_distance() {
        std::shared_lock lock(state_mutex);
        return current_state.stationary_distance;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline uint8_t PresenceSensor<uart_ctrl_t>::get_stationary_energy() {
        std::shared_lock lock(state_mutex);
        return current_state.stationary_energy;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline uint16_t PresenceSensor<uart_ctrl_t>::get_detection_distance() {
        std::shared_lock lock(state_mutex);
        return current_state.detection_distance;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline std::array<uint8_t, 9> PresenceSensor<uart_ctrl_t>::get_motion_distance_energy() {
        std::shared_lock lock(state_mutex);
        return current_state.motion_distance_energy;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline std::array<uint8_t, 9> PresenceSensor<uart_ctrl_t>::get_stationary_distance_energy() {
        std::shared_lock lock(state_mutex);
        return current_state.stationary_distance_energy;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline uint8_t PresenceSensor<uart_ctrl_t>::get_photosensitivy() {
        std::shared_lock lock(state_mutex);
        return current_state.photosensitivy;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline bool PresenceSensor<uart_ctrl_t>::get_output_status() {
        std::shared_lock lock(state_mutex);
        return current_state.output_status;
    }

    template <uart_ctrl_req uart_ctrl_t>
    inline void PresenceSensor<uart_ctrl_t>::run() {
        std::array<uint8_t, CONFIG_SENSOR_UART_RX_BUFFER_SIZE> rx_buffer{0};

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
        }
    }
}