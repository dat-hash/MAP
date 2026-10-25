idf_component_register(
    SRCS "main.cpp"  "gnss_driver.hpp" "sd_logger.hpp" "imu_driver.hpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_driver_uart freertos esp_timer fatfs sdmmc vfs log
)
