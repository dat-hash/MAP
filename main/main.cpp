// =========================================================================
// main.cpp
// =========================================================================

#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gnss_driver.hpp"

// extern "C": app_main must keep its plain C name so ESP-IDF's own C
// startup code can find and call it
extern "C" void app_main(void) {

    setvbuf(stdout, nullptr, _IONBF, 0);

    static gnss::GnssDriver driver;

    // START GNSS DRIVER
    driver.begin();

    vTaskDelete(nullptr);
}
