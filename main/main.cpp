// =========================================================================
// main.cpp
// =========================================================================
 
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_common.h"
#include "esp_log.h"

#include "gnss_driver.hpp"
#include "sd_logger.hpp"
#include "imu_driver.hpp"

// module TX -> GPIO 16, module RX <- GPIO 17
constexpr gnss::Pins GnssPins = { .uart = UART_NUM_2, .rx_pin = GPIO_NUM_16, .tx_pin = GPIO_NUM_17 };

// CS 10, MOSI 11, CLK 12, MISO 13
constexpr sd_log::Pins SdPins = { .spi_host = SPI2_HOST, .mosi = GPIO_NUM_11, .miso = GPIO_NUM_13, .sclk = GPIO_NUM_12, .cs = GPIO_NUM_10 };

// CS 4, MOSI 5, CLK 7, MISO 6
constexpr IMU::Pins IMUPins = { .spi_host = SPI3_HOST, .mosi = GPIO_NUM_5, .miso = GPIO_NUM_6, .sclk = GPIO_NUM_7, .cs = GPIO_NUM_4 };


extern "C" void app_main(void){

    if (gnss::start(GnssPins)) {
        ESP_LOGI("main", "GNSS task started");
    } else {
        ESP_LOGE("main", "GNSS could not start (see above)");
    }

    if (sd_log::start(SdPins)) {
        ESP_LOGI("main", "SD logger task started");
    } else {
        ESP_LOGE("main", "SD logger could not start (see above)");
    }

    if (IMU::start(IMUPins)) {
        ESP_LOGI("main", "IMU task started");
    } else {
        ESP_LOGE("main", "IMU could not start (see above)");
    }
}