idf_component_register(
    SRCS "main.cpp" "gnss_driver.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_driver_uart freertos log
)
