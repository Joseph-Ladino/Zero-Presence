idf_component_register(
    SRCS LD2410C-ESP.cpp FrameParser.cpp
    INCLUDE_DIRS "include"
    REQUIRES "esp_driver_uart"
)
