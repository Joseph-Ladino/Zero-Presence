#include "FrameParser.h"
#include "LD2410C.h"

using namespace LD2410C;
void FrameParser::parse_byte(uint8_t f_byte) {

    using enum parsing_state;

    switch (state) {
        case HEADER: {

            if (f_byte == status_header[status_header_found_idx]) {
                status_header_found_idx++;

                if (status_header_found_idx == status_header.size()) {
                    state = READ_SIZE_LOWER;
                    is_status = true;
                    frame_data.clear();
                }

                command_header_found_idx = 0;
            } else if (f_byte == command_header[command_header_found_idx]) {
                command_header_found_idx++;

                if (command_header_found_idx == command_header.size()) {
                    state = READ_SIZE_LOWER;
                    is_status = false;
                    frame_data.clear();
                }

                status_header_found_idx = 0;
            } else {
                command_header_found_idx = 0;
                status_header_found_idx = 0;
            }

        } break;

        case READ_SIZE_LOWER: {

            frame_data_size = (is_little_endian) ? f_byte : f_byte << 8; // lower byte first if little-endian
            state = READ_SIZE_UPPER;

        } break;

        case READ_SIZE_UPPER: {

            frame_data_size |= (is_little_endian) ? f_byte << 8 : f_byte; // upper byte second if little-endian
            frame_data.reserve(frame_data_size);                          // reserve space for data
            state = DATA;

        } break;

        case DATA: {

            frame_data.push_back(f_byte);
            if (frame_data.size() == frame_data_size)
                state = FOOTER;

        } break;

        case FOOTER: {
            if (is_status) {
                if (f_byte == status_footer[status_footer_found_idx]) {
                    status_footer_found_idx++;
                    if (status_footer_found_idx == status_footer.size())
                        is_done = true;

                } else {
                    // If footer is not found, error handling could be added here
                    // For now, we just reset the parser
                    reset();
                }
            } else {
                if (f_byte == command_footer[command_footer_found_idx]) {
                    command_footer_found_idx++;
                    if (command_footer_found_idx == command_footer.size())
                        is_done = true;

                } else {
                    // If footer is not found, error handling could be added here
                    // For now, we just reset the parser
                    reset();
                }
            }

        } break;
        default:
            break;
    }
}

bool FrameParser::is_frame_command() {
    return !is_status;
}
bool FrameParser::is_frame_status() {
    return is_status;
}
bool FrameParser::done() {
    return is_done;
}

void FrameParser::reset() {
    state = parsing_state::HEADER;

    command_header_found_idx = 0;
    status_header_found_idx = 0;
    command_footer_found_idx = 0;
    status_footer_found_idx = 0;

    frame_data_size = 0;
    frame_data.clear();

    is_status = false;
    is_done = false;
}

std::span<uint8_t> FrameParser::get_frame_data() {
    return std::span{frame_data.begin(), frame_data.end()};
}
