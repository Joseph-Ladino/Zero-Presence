/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "LD2410C-ESP.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <array>
#include <inttypes.h>
#include <stdio.h>

extern "C" void app_main(void);

auto tag = "app_main";

LD2410C::ESP32_UART_Adapter uart_adapter{};
LD2410C::PresenceSensor sensor(uart_adapter);

void app_main(void) {
    using namespace LD2410C;

    auto config_task = [](void *pvParameters) {
        auto *sensorPtr = static_cast<PresenceSensor<ESP32_UART_Adapter> *>(pvParameters);

        while (true) {
            // Example: Set engineering mode
            sensorPtr->set_config_mode(true);
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for 5 seconds
            sensorPtr->set_config_mode(false);
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for 5 seconds
        }
        vTaskDelete(NULL);
    };

    auto engineer_task = [](void *pvParameters) {
        auto *sensorPtr = static_cast<PresenceSensor<ESP32_UART_Adapter> *>(pvParameters);

        while (true) {
            // Example: Set engineering mode
            sensorPtr->set_engineering_mode(true);
            vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait for 5 seconds
            sensorPtr->set_engineering_mode(false);
            vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait for 5 seconds
        }
        vTaskDelete(NULL);
    };

    // Create the task, passing the sensor pointer as parameter
    xTaskCreate([](void *pvParameters) {
        auto *sensorPtr = static_cast<PresenceSensor<ESP32_UART_Adapter> *>(pvParameters);

        while (true) {
            sensorPtr->run();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    },
                "presence_sensor_uart", 3072, &sensor, 5, NULL);

    xTaskCreate(config_task, "engineering_mode_task_1", 2048, &sensor, 5, NULL);
    xTaskCreate(engineer_task, "engineering_mode_task_2", 2048, &sensor, 5, NULL);

    // ESP_LOGI(tag, "Main task running...");
    // while (true) {
    //     // Simulate some work in the main task
    //     // vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 1 second

    //     constexpr size_t buffer_size = 1024;
    //     uint8_t buffer[buffer_size];
    //     int bytes_read = uart_adapter.read_bytes(buffer, buffer_size, 20);
    //     if (bytes_read > 0) {
    //         // ESP_LOGI(tag, "Read %d bytes from UART", bytes_read);
    //         std::span<uint8_t> data_span(buffer);
    //         log_frame(data_span.subspan(0, bytes_read));
    //     }
    // }
}