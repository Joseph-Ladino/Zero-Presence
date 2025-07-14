#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <cstddef>
#include <span>

namespace LD2410C {

    // parses a single data frame from sensor
    struct FramerParser {

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
        void parse_byte(uint8_t);
        bool is_frame_command();
        bool is_frame_status();
        bool done();
        void reset();
        std::span<uint8_t> get_frame_data();

        FramerParser() = default;
    };
}