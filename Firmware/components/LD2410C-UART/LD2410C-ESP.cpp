#include "LD2410C-ESP.h"
#include "sdkconfig.h"

using namespace LD2410C;

static auto tag = "ESP32_UART_Adapter";

int ESP32_UART_Adapter::write_bytes(uint8_t *data, uint32_t len) {
    return uart_write_bytes(port_num, data, len);
}

int LD2410C::ESP32_UART_Adapter::read_bytes(uint8_t *data, uint32_t max_length, uint16_t timeout_ms) {
    size_t buffered_length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(port_num, (size_t *)&buffered_length));

    if(buffered_length == 0) return 0;

    return uart_read_bytes(port_num, data, std::min(max_length, static_cast<uint32_t>(buffered_length)), timeout_ms / portTICK_PERIOD_MS);
}

bool LD2410C::ESP32_UART_Adapter::set_baud_rate(uint16_t baud_rate) {
    auto res = ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_baudrate(port_num, baud_rate));

    return res == ESP_OK;
}

ESP32_UART_Adapter::ESP32_UART_Adapter() : ESP32_UART_Adapter(
                                               static_cast<uart_port_t>(CONFIG_SENSOR_UART_PORT_NUM),
                                               uart_config_t{
                                                   .baud_rate = CONFIG_SENSOR_UART_INITIAL_BAUD,
                                                   .data_bits = UART_DATA_8_BITS,
                                                   .parity = UART_PARITY_DISABLE,
                                                   .stop_bits = UART_STOP_BITS_1,
                                                   .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                                   .source_clk = UART_SCLK_DEFAULT,
                                               },
                                               CONFIG_SENSOR_UART_RXD,
                                               CONFIG_SENSOR_UART_TXD) {}

LD2410C::ESP32_UART_Adapter::ESP32_UART_Adapter(uart_port_t port_num, uart_config_t config, uint16_t rx_pin, uint16_t tx_pin) : port_num(port_num), config(config) {
    // ESP_ERROR_CHECK(uart_driver_install(port_num, CONFIG_SENSOR_UART_RX_BUFFER_SIZE, CONFIG_SENSOR_UART_TX_BUFFER_SIZE, CONFIG_SENSOR_UART_QUEUE_SIZE, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_driver_install(port_num, CONFIG_SENSOR_UART_RX_BUFFER_SIZE, CONFIG_SENSOR_UART_TX_BUFFER_SIZE, 0, NULL, 0));
    ESP_LOGI(tag, "UART driver installed on port %d", port_num);

    ESP_ERROR_CHECK(uart_param_config(port_num, &this->config));
    ESP_LOGI(tag, "UART parameters configured on port %d", port_num);

    ESP_ERROR_CHECK(uart_set_pin(port_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(tag, "UART pins set on port %d: TX=%d, RX=%d", port_num, tx_pin, rx_pin);
}
