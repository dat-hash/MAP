#pragma once

#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <dirent.h>
#include <unistd.h>

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"

#include "gnss_driver.hpp"
#include "imu_driver.hpp"

namespace sd_log {

constexpr const char *TAG = "  SD";
constexpr const char *MOUNT_POINT = "/sdcard";

constexpr int SPI_CLOCK_KHZ = 10000; // 10 MHz: kind to jumper wires
constexpr uint32_t SYNC_EVERY_S = 1000; // at most 1 s of rows lost at a power cut
constexpr uint32_t RETRY_MS = 5000; // no card: look again this often
constexpr uint32_t RETRY_BAD_CARD_MS = 60000;  // card there but unusable (full...)
constexpr uint32_t STATUS_PER_MS = 60000;

constexpr uint32_t TASK_STACK_BYTES = 8192;
constexpr UBaseType_t TASK_PRIORITY = 3; // below the GNSS task (5)


// 2. DATA
struct Pins {
    spi_host_device_t spi_host;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
};

// Only the logger task uses these.
Pins g_pins = {};
sdmmc_card_t *g_card = nullptr; // not null while mounted
FILE *g_file = nullptr; // not null while logging
char g_name[24] = {}; // "RIDE0007.CSV"
char g_buffer[4096]; // rows collect here between syncs
uint32_t g_rows = 0;
uint32_t g_mount_fails = 0;

FILE *g_imu_file = nullptr; // IMU0007.CSV, opened and closed together with g_file
char g_imu_name[24] = {};
char g_imu_buffer[4096]; // own buffer: setvbuf can't share one between two files
uint32_t g_imu_rows = 0;

bool mount(){
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = g_pins.spi_host;
    host.max_freq_khz = SPI_CLOCK_KHZ;

    sdspi_device_config_t device = SDSPI_DEVICE_CONFIG_DEFAULT();
    device.host_id = g_pins.spi_host;
    device.gpio_cs = g_pins.cs;

    esp_vfs_fat_sdmmc_mount_config_t options = {};
    options.format_if_mount_failed = false;
    options.max_files = 2;
    options.allocation_unit_size = 16 * 1024;

    esp_err_t err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &device, &options, &g_card);

    if(err != ESP_OK){
        g_card = nullptr;
        g_mount_fails++;

        if (g_mount_fails == 1 || g_mount_fails % 12 == 0) {
            if (err == ESP_FAIL) {
                ESP_LOGW(TAG, "Card has no FAT filesystem");
            } else {
                ESP_LOGW(TAG, "No SD card (%s). Are you sure the card in? MOSI %d MISO %d CLK %d CS %d", esp_err_to_name(err),
                         g_pins.mosi, g_pins.miso, g_pins.sclk, g_pins.cs);
            }
        }
        return false;

    }

    g_mount_fails = 0;
    sdmmc_card_print_info(stdout,g_card);
    return true;
}

void unmount(){
    if(g_card != nullptr){
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT,g_card);
    g_card = nullptr;
    }
}

int give_next_file_version_number(){
    int highest_ver = 0;
    DIR *log_folder = opendir(MOUNT_POINT);

    if(log_folder == nullptr){
        return 1; // no file being log yet
    }

    struct dirent *entry = readdir(log_folder);

    while (entry != nullptr){
        int ver_number = 0;
        if(sscanf(entry->d_name,"PATH%4d.CSV",&ver_number) == 1 && ver_number > highest_ver){
            highest_ver = ver_number;
        }
        entry = readdir(log_folder);
    }

    closedir(log_folder);
    return highest_ver + 1;
}

bool sync_data_from_controller_to_card(){
    if (fflush(g_file) != 0 || fsync(fileno(g_file)) != 0) {
        return false;
    }
    if (g_imu_file != nullptr) {
        if (fflush(g_imu_file) != 0 || fsync(fileno(g_imu_file)) != 0) {
            return false;
        }
    }
    return true;
}

bool open_log(){

    int file_number = give_next_file_version_number();
    if(file_number > 9999){
        ESP_LOGE(TAG,"THE SD CARD IS FULL AT 9999 FILES -> PLEASE COPY IT OFF");
        return false;
    }

    // file name
    snprintf(g_name, sizeof(g_name), "PATH%04d.CSV", file_number);

    char path[64];
    snprintf(path, sizeof(path), "%s/%s", MOUNT_POINT, g_name);

    g_file = fopen(path, "a");
    if (g_file == nullptr) {
        ESP_LOGE(TAG, "Could not create %s (%s)", path, strerror(errno));
        return false;
    }

    setvbuf(g_file, g_buffer, _IOFBF, sizeof(g_buffer));

    fputs("rx_ms,itow_ms,utc,fix_ok,fix_type,satellites,"
          "lat_deg,lon_deg,alt_m,h_acc_m,v_acc_m,"
          "speed_kmh,speed_acc_kmh,heading_deg,vel_n_mps,vel_e_mps,vel_d_mps,pdop,"
          "reject,filt_lat_deg,filt_lon_deg,filt_alt_m\n", g_file);

    // IMU file with the same number, so the pair is obvious on the card
    snprintf(g_imu_name, sizeof(g_imu_name), "IMU%04d.CSV", file_number);

    char imu_path[64];
    snprintf(imu_path, sizeof(imu_path), "%s/%s", MOUNT_POINT, g_imu_name);

    g_imu_file = fopen(imu_path, "a");
    if (g_imu_file == nullptr) {
        ESP_LOGE(TAG, "Could not create %s (%s)", imu_path, strerror(errno));
        fclose(g_file);
        g_file = nullptr;
        remove(path);
        return false;
    }

    setvbuf(g_imu_file, g_imu_buffer, _IOFBF, sizeof(g_imu_buffer)); // before the first write

    fputs("t_us,gx_rps,gy_rps,gz_rps,ax_mps2,ay_mps2,az_mps2,temp_c,saturated\n", g_imu_file);

    if (!sync_data_from_controller_to_card()) {
        ESP_LOGE(TAG, "Could not write to %s / %s", path, imu_path);
        fclose(g_file);
        g_file = nullptr;
        fclose(g_imu_file);
        g_imu_file = nullptr;
        remove(path);
        remove(imu_path);
        return false;
    }

    g_rows = 0;
    g_imu_rows = 0;
    ESP_LOGI(TAG, "Logging to %s + %s", g_name, g_imu_name);
    return true;
}

void close_log(const char *reason){
    ESP_LOGW(TAG, "%s stopped after %" PRIu32 " rows: %s", g_name, g_rows, reason);

    if (g_file != nullptr) {
        fclose(g_file);
        g_file = nullptr;
    }
    if (g_imu_file != nullptr) {
        fclose(g_imu_file);
        g_imu_file = nullptr;
    }
    unmount();
}

// 5. Rows
// One epoch = one line in the RAM buffer. False only if the write failed.
bool write_gnss_row(const gnss::Fix &fix){
    char utc[32] = ""; // stays empty until the receiver knows the time

    if (fix.utc_valid) {
        time_t seconds = static_cast<time_t>(fix.utc_ms / 1000);
        struct tm t;
        gmtime_r(&seconds, &t);

        // "2026-10-03T14:03:22" then ".100Z"
        size_t length = strftime(utc, sizeof(utc), "%Y-%m-%dT%H:%M:%S", &t);
        snprintf(utc + length, sizeof(utc) - length, ".%03dZ", static_cast<int>(fix.utc_ms % 1000));
    }

    int written = fprintf(g_file,
        "%" PRIu32 ",%" PRIu32 ",%s,%d,%u,%u,"
        "%.7f,%.7f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.2f,"
        "%u,%.7f,%.7f,%.3f\n",
        fix.rx_ms, fix.itow_ms, utc, static_cast<int>(fix.fix_ok),
        static_cast<unsigned>(fix.fix_type), static_cast<unsigned>(fix.satellites),
        fix.lat_deg, fix.lon_deg, fix.alt_m, fix.h_acc_m, fix.v_acc_m,
        fix.speed_kmh, fix.speed_acc_kmh, fix.heading_deg,
        fix.vel_n_mps, fix.vel_e_mps, fix.vel_d_mps, fix.pdop,
        static_cast<unsigned>(fix.reject), fix.filt_lat_deg, fix.filt_lon_deg, fix.filt_alt_m);

    if (written < 0) {
        return false; // utc invalid
    }

    g_rows++;
    return true;
}

// One IMU sample = one line. Gyro 7 decimals, accel 5: the sensor's smallest step
// is 0.0000109 rad/s and 0.0003 m/s^2
bool write_imu_row(const IMU::Sample &sample){
    int written = fprintf(g_imu_file,
        "%" PRId64 ",%.7f,%.7f,%.7f,%.5f,%.5f,%.5f,%.2f,%d\n",
        sample.timestamp,
        sample.gyro_rad_per_s[0], sample.gyro_rad_per_s[1], sample.gyro_rad_per_s[2],
        sample.accel_m_per_s2[0], sample.accel_m_per_s2[1], sample.accel_m_per_s2[2],
        sample.temp, sample.saturated ? 1 : 0);

    if (written < 0) {
        return false;
    }

    g_imu_rows++;
    return true;
}

// 6. The task
uint32_t try_start(){
    if (g_card == nullptr && !mount()) {
        return RETRY_MS; // no card yet
    }

    if (open_log()) {
        return 0; // 0 = logging now
    }
    // Card there but can not be use :(
    unmount();
    return RETRY_BAD_CARD_MS;
}

void logger_task(void *){
    uint32_t retry_ms = try_start();
    uint32_t last_try_ms  = gnss::now_ms();
    uint32_t last_sync_ms = gnss::now_ms();
    uint32_t last_note_ms = gnss::now_ms();

    while (true) {
        // Block on the FAST stream (IMU, every 5 ms), 20 ms at most so the loop
        // keeps turning if the IMU is dead. Then empty both queues without waiting.
        IMU::Sample sample;
        bool have_sample = IMU::imu_wait_for_sample(sample, 20);
        uint32_t now = gnss::now_ms();

        if (g_file == nullptr && (now - last_try_ms) >= retry_ms) {
            retry_ms = try_start();
            last_try_ms = now;
            last_sync_ms = now;
        }

        // Write every IMU sample waiting (after an SD stall that can be dozens)
        while (have_sample) {
            if (g_imu_file != nullptr && !write_imu_row(sample)) {
                close_log("IMU write failed (card pulled out or full?)");
            }
            have_sample = IMU::imu_wait_for_sample(sample, 0);
        }

        gnss::Fix fix;
        while (gnss::wait_for_next_fix(fix, 0)) {
            if (g_file != nullptr && !write_gnss_row(fix)) {
                close_log("write failed (card pulled out or full?)");
            }
        }

        if (g_file != nullptr && (now - last_sync_ms) >= SYNC_EVERY_S) {
            last_sync_ms = now;
            if (!sync_data_from_controller_to_card()) {
                close_log("sync failed (pls check if card pulled out or full?)");
            }
        }

        if (g_file != nullptr && (now - last_note_ms) >= STATUS_PER_MS) {
            last_note_ms = now;
            ESP_LOGI(TAG, "%s: %" PRIu32 " rows | %s: %" PRIu32 " rows, dropped %" PRIu32 " late %" PRIu32 " bad %" PRIu32 " restarts %" PRIu32,
                     g_name, g_rows, g_imu_name, g_imu_rows,
                     IMU::imu_samples_dropped.load(), IMU::imu_samples_late.load(),
                     IMU::imu_samples_bad.load(), IMU::imu_restarts.load());
        }
    }
}

// Sets up the SPI bus and start the logger task.
bool start(const Pins &pins){
    g_pins = pins;

    spi_bus_config_t bus = {};
    bus.mosi_io_num = pins.mosi;
    bus.miso_io_num = pins.miso;
    bus.sclk_io_num = pins.sclk;
    bus.quadwp_io_num = -1; // -1 = not used
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 4000;

    esp_err_t err = spi_bus_initialize(pins.spi_host, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus setup failed (%s)",
                 esp_err_to_name(err));
        return false;
    }

    if (xTaskCreate(logger_task, "sd_log", TASK_STACK_BYTES, nullptr, TASK_PRIORITY, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Could not start the logger task");
        return false;
    }
    return true;
}
}