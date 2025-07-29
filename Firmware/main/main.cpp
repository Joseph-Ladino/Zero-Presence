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
#include "provisioning-example.h"

extern "C" void app_main(void);

auto tag = "app_main";

LD2410C::ESP32_UART_Adapter uart_adapter{};
LD2410C::PresenceSensor sensor(uart_adapter);

void app_main(void) {
    using namespace LD2410C;

    auto test_task = [](void *pvParameters) {
        auto *sensorPtr = static_cast<PresenceSensor<ESP32_UART_Adapter> *>(pvParameters);
        while (true) {

            ESP_LOGI(tag, "Enabling config mode...");
            if (sensorPtr->set_config_mode(true))
                ESP_LOGI(tag, "Config mode enabled.");
            else
                ESP_LOGE(tag, "Failed");

            ESP_LOGI(tag, "Enabling engineering mode...");
            if (sensorPtr->set_engineering_mode(true))
                ESP_LOGI(tag, "Engineering mode enabled.");
            else
                ESP_LOGE(tag, "Failed");

            ESP_LOGI(tag, "Enabling Bluetooth mode...");
            if (sensorPtr->set_bluetooth_mode(true))
                ESP_LOGI(tag, "Bluetooth mode enabled.");
            else
                ESP_LOGE(tag, "Failed");

            vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second

            ESP_LOGI(tag, "Restarting sensor...");
            if (sensorPtr->restart())
                ESP_LOGI(tag, "Sensor restarted.");
            else
                ESP_LOGE(tag, "Failed");

            vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 1 second
        }

        vTaskDelete(NULL);
    };

    // Create the task, passing the sensor pointer as parameter
    xTaskCreate([](void *pvParameters) {
        auto *sensorPtr = static_cast<PresenceSensor<ESP32_UART_Adapter> *>(pvParameters);

        while (true) {
            sensorPtr->run();
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    },
                "presence_sensor_uart", 3072, &sensor, 5, NULL);

    // Create the task, passing the sensor pointer as parameter
    xTaskCreate([](void *pvParameters) {
        auto *sensorPtr = static_cast<PresenceSensor<ESP32_UART_Adapter> *>(pvParameters);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        while (true) {
            auto state = sensorPtr->get_state();

            ESP_LOGI(tag, "Current state: target_state=%d, movement_distance=%d, movement_energy=%d, stationary_distance=%d, stationary_energy=%d, detection_distance=%d",
                     static_cast<int>(state.target_state),
                     state.movement_distance,
                     state.movement_energy,
                     state.stationary_distance,
                     state.stationary_energy,
                     state.detection_distance);


            log_frame("Motion distance energy", state.motion_distance_energy);
            log_frame("Stationary distance energy", state.stationary_distance_energy);
            ESP_LOGI(tag, "Photosensitivity: %d", state.photosensitivy);
            ESP_LOGI(tag, "Output status: %s", state.output_status ? "ON" : "OFF");

            vTaskDelay(500 / portTICK_PERIOD_MS);

            ESP_LOGI(tag, "Setting config mode...");
            if (sensorPtr->set_config_mode(true))
                ESP_LOGI(tag, "Config mode set.");
            else
                ESP_LOGE(tag, "Failed to set config mode.");

            ESP_LOGI(tag, "Setting engineering mode...");
            if (sensorPtr->set_engineering_mode(true))
                ESP_LOGI(tag, "Engineering mode set successfully.");
            else
                ESP_LOGE(tag, "Failed to set engineering mode.");
        }   
        vTaskDelete(NULL);
    },
                "test_uart_status_frames", 3072, &sensor, 5, NULL);

    // xTaskCreate(test_task, "test_sensor_task", 2048, &sensor, 5, NULL);

    ESP_LOGI(tag, "Starting provisioning example...");
    init_wifi_provisioning();
}