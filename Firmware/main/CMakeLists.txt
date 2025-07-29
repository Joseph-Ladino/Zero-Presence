idf_component_register(SRCS "provisioning-example.cpp" "main.cpp"
                    REQUIRES LD2410C-UART esp_driver_uart spi_flash esp_system
                    PRIV_REQUIRES esp_wifi nvs_flash wifi_provisioning
                    INCLUDE_DIRS "./include")
