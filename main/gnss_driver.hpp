// =========================================================================
// gnss.hpp -- the u-blox receiver
// 1. Settings every tunable number, in one place
// 2. UBX facts ids, keys, and the NAV-PVT struct
// 3. Our data Pins, Fix, and the module's variables
// 4. Reading frames read_frame(): bytes -> one checked UBX message
// 5. Sending config configure(): one CFG-VALSET, wait for ACK/NAK
// 6. Finding the module find_module(): baud search + setup
// 7. Decoding NAV-PVT decode_nav_pvt(): memcpy into a struct
// 8. Rejection + filter check_and_filter(): impossible fixes out, smooth the rest
// 9. The task gnss_cycle(), gnss_task()
// 10. Public functions start(), wait_for_next_fix()
// =========================================================================

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <optional>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace gnss {

// SETTING
constexpr const char *TAG = "GNSS";

// BAUD RATE
constexpr uint32_t TARGET_BAUD = 115200;
constexpr std::array<uint32_t, 4> CANDIDATES_BAUDS = { 115200, 38400, 9600, 57600 };

// MEASUREMENT
constexpr uint16_t MEASUREMENT_PERIOD_MS = 100; // 10 Hz

// K GAIN CALCULATION:
constexpr double q_scale = 0.3;
constexpr double r_scale = 3.0;
constexpr double min_gain = 0.05;

// DYNAMIC MODEL:
constexpr uint8_t PORTABLE = 0;
constexpr uint8_t STATIONARY = 2;
constexpr uint8_t PEDESTRIAN = 3;
constexpr uint8_t AUTOMOTIVE = 4;
constexpr uint8_t SEA = 5;
constexpr uint8_t AIRBORNE_1G = 6;
constexpr uint8_t AIRBORNE_2G = 7;
constexpr uint8_t AIRBORNE_4G = 8;
constexpr uint8_t WRIST = 9;
constexpr uint8_t BIKE = 11;

// TIMING
constexpr uint32_t SETTING_ACK_TIMEOUT_MS = 500; 
constexpr uint32_t FLASH_ACK_TIMEOUT_MS = 1500;
constexpr uint32_t LINK_CHECK_MS = 2000;
constexpr uint32_t LINK_LOST_MS = 5000;
constexpr uint32_t READ_WINDOW_MS = 250;

// Resources
constexpr int UART_RX_BUFFER_BYTES = 2048;
constexpr UBaseType_t QUEUE_LENGTH = 50; // 5s
constexpr uint32_t TASK_STACK_BYTES = 6144;
constexpr UBaseType_t TASK_PRIORITY = 5; // above the SD logger (3)

// Rejection
constexpr double MAX_H_ACC_M = 20.0;
constexpr double MAX_SPEED_KMH = 25.0;
constexpr double MIN_ALTITUDE_M = -500.0;
constexpr double MAX_ALTITUDE_M = 9000.0;
constexpr double MAX_ACCEL_MPS2 = 2 * 9.81; // 2G
constexpr double MIN_GATE_M = 10.0; // gatekeep
constexpr double GATE_SIGMAS = 5.0;
constexpr uint32_t MAX_GAP_MS = 1000;

// Metres per degree of latitude (WGS84 radius x pi / 180).
constexpr double METRES_PER_DEGREE = 111319.49;
constexpr double PI = 3.14159265358979323846;

// 2. UBX
// B5 62 | class | id | length (2, little-endian) | payload | CK_A CK_B
constexpr uint8_t SYNC_1 = 0xB5;
constexpr uint8_t SYNC_2 = 0x62;

constexpr uint8_t CLASS_NAV = 0x01;
constexpr uint8_t ID_NAV_PVT = 0x07;
constexpr uint8_t CLASS_ACK = 0x05;
constexpr uint8_t ID_NAK = 0x00;
constexpr uint8_t ID_ACK = 0x01;
constexpr uint8_t CLASS_CFG = 0x06;
constexpr uint8_t ID_VALSET = 0x8A;

constexpr uint8_t LAYER_RAM = 0x01;
constexpr uint8_t LAYER_BBR = 0x02;
constexpr uint8_t LAYER_FLASH = 0x04;
constexpr uint8_t RAM_AND_BBR = LAYER_RAM | LAYER_BBR;

// Configuration keys. 
// The top hex digit of a key says how big its value
// 1 or 2 = one byte, 3 = two bytes, 4 = four bytes
constexpr uint32_t KEY_UART_1_BAUD = 0x40520001;
constexpr uint32_t KEY_OUTPUT_UBX = 0x10740001;
constexpr uint32_t KEY_OUTPUT_NMEA = 0x10740002;
constexpr uint32_t KEY_NAV_PVT_RATE = 0x20910007;
constexpr uint32_t KEY_MEAS_RATE_MS = 0x30210001;
constexpr uint32_t KEY_DYNAMIC_MODEL = 0x20110021;
constexpr uint32_t KEY_L5_HEALTH_OVERRIDE = 0x10320001;

constexpr size_t MAX_PAYLOAD = 256; // NAV PVT is 92 bytes

struct Frame {
    uint8_t msg_class = 0;
    uint8_t msg_id = 0;
    uint16_t length = 0;
    std::array<uint8_t, MAX_PAYLOAD> payload = {};
};

struct NavPvt {
    uint32_t iTOW; // 0 ms, GPS time of week
    uint16_t year; // 4 UTC
    uint8_t month; // 6
    uint8_t day; // 7
    uint8_t hour; // 8
    uint8_t min; // 9
    uint8_t sec; // 10 rounded pls see nano
    uint8_t valid; // 11 bit 0 date valid, bit 1 time valid
    uint32_t tAcc; // 12 ns
    int32_t nano; // 16 ns, signed correction to sec
    uint8_t fixType; // 20 0 none, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
    uint8_t flags; // 21 bit 0 gnssFixOK
    uint8_t flags2; // 22
    uint8_t numSV; // 23
    int32_t lon; // 24 1e-7 deg
    int32_t lat; // 28 1e-7 deg
    int32_t height; // 32 mm above ellipsoid
    int32_t hMSL; // 36 mm above mean sea level
    uint32_t hAcc; // 40 mm
    uint32_t vAcc; // 44 mm
    int32_t velN; // 48 mm/s
    int32_t velE; // 52 mm/s
    int32_t velD; // 56 mm/s (+ = down)
    int32_t gSpeed; // 60 mm/s
    int32_t headMot; // 64 1e-5 deg
    uint32_t sAcc; // 68 mm/s
    uint32_t headAcc; // 72 1e-5 deg
    uint16_t pDOP; // 76 0.01
    uint16_t flags3; // 78 bit 0 invalidLlh
    uint8_t reserved[4]; // 80
    int32_t headVeh; // 84
    int16_t magDec; // 88
    uint16_t magAcc; // 90
};

static_assert(sizeof(NavPvt) == 92, "NavPvt must match the 92-byte payload");
static_assert(offsetof(NavPvt, nano) == 16, "NavPvt layout");
static_assert(offsetof(NavPvt, lon) == 24, "NavPvt layout");
static_assert(offsetof(NavPvt, lat) == 28, "NavPvt layout");
static_assert(offsetof(NavPvt, hMSL) == 36, "NavPvt layout");
static_assert(offsetof(NavPvt, velN) == 48, "NavPvt layout");
static_assert(offsetof(NavPvt, pDOP) == 76, "NavPvt layout");
static_assert(offsetof(NavPvt, flags3) == 78, "NavPvt layout");
static_assert(std::endian::native == std::endian::little, "memcpy decoding needs a little-endian CPU (the ESP32 is)");


// 3. Data
struct Pins {
    uart_port_t uart;
    gpio_num_t rx_pin; // ESP32 RX <- module TX
    gpio_num_t tx_pin; // ESP32 TX -> module RX
};

// reject reason
enum class Reject : uint8_t {
    None = 0,
    NoFix = 1, // receiver says the position is garbage
    PoorAccuracy = 2, // hAcc above MAX_H_ACC_M
    TooFast = 3, // speed above MAX_SPEED_KMH
    BadAltitude = 4, // outside MIN_ALTITUDE_M and MAX_ALTITUDE_M
    TooMuchAccel = 5, // velocity changed faster than MAX_ACCEL_MPS2
    Jump = 6, // position too far from where it should be
};

struct Fix {
    // Time
    uint32_t rx_ms = 0; // ESP32 ms since boot when it arrived
    uint32_t itow_ms = 0; // GPS time of week (steps of exactly 100)
    bool utc_valid = false;
    int64_t utc_ms = 0; // ms since 1970-01-01 UTC

    // Quality
    bool fix_ok = false; // the receiver's own verdict
    uint8_t fix_type = 0;
    uint8_t satellites = 0;
    double pdop = 0.0;
    double h_acc_m = 0.0;
    double v_acc_m = 0.0;
    double speed_acc_kmh = 0.0;

    // Position and motion, exactly as the receiver sent them
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m = 0.0;
    double vel_n_mps = 0.0;
    double vel_e_mps = 0.0;
    double vel_d_mps = 0.0; // + = going down
    double speed_kmh = 0.0;
    double heading_deg = 0.0;

    // verdict, and the filtered position
    Reject reject = Reject::NoFix;
    double filt_lat_deg = 0.0;
    double filt_lon_deg = 0.0;
    double filt_alt_m = 0.0;
};

// The filter's memory: the last ACCEPTED epoch.
struct FilterState {
    bool seeded = false;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m = 0.0;
    double vel_n_mps = 0.0;
    double vel_e_mps = 0.0;
    double vel_d_mps = 0.0;
    uint32_t itow_ms = 0; // receiver time of that epoch
    double var_h = 0.0;
    double var_v = 0.0;
};

// module own variable
uart_port_t g_port = UART_NUM_2;
QueueHandle_t g_queue = nullptr;
FilterState g_filter;

bool g_search_needed = true;
uint32_t g_last_pvt_ms = 0;
uint32_t g_bad_checksums = 0;
uint32_t g_epochs = 0;
uint32_t g_rejected = 0; // since the last status line
uint32_t g_dropped = 0; // epochs the logger had no room for
bool g_had_fix = false;

portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;
Fix g_latest;
bool g_have_latest = false;

// 4. Reading frames
uint32_t now_ms(){
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void add_to_checksum(const uint8_t *data, size_t length, uint8_t &a, uint8_t &b){
    for (size_t i = 0; i < length; i++) {
        a = static_cast<uint8_t>(a + data[i]);
        b = static_cast<uint8_t>(b + a);
    }
}

bool read_exact(uint8_t *buffer, size_t count, uint32_t timeout_ms){
    int got = uart_read_bytes(g_port, buffer, count, pdMS_TO_TICKS(timeout_ms));
    return got == static_cast<int>(count);
}

bool read_frame(Frame &frame, uint32_t timeout_ms){
    // Wait at least one tick per byte, or the loop below would spin.
    const TickType_t byte_wait = std::max<TickType_t>(1, pdMS_TO_TICKS(10));
    const uint32_t start = now_ms();
    uint8_t previous = 0;

    while ((now_ms() - start) < timeout_ms) {
        uint8_t byte = 0;

        if (uart_read_bytes(g_port, &byte, 1, byte_wait) != 1) {
            continue;
        } // wait

        bool found_sync = (previous == SYNC_1 && byte == SYNC_2);
        previous = byte;

        if (!found_sync) {
            continue;
        }

        uint8_t header[4] = {};
        if (!read_exact(header, sizeof(header), 100)) {
            continue;
        }

        frame.msg_class = header[0];
        frame.msg_id = header[1];
        frame.length = static_cast<uint16_t>(header[2] | (header[3] << 8));

        if (frame.length > MAX_PAYLOAD) {
            previous = 0;
            continue;
        }

        uint8_t received_ck[2] = {};
        if (!read_exact(frame.payload.data(), frame.length, 300) ||
            !read_exact(received_ck, sizeof(received_ck), 100)) {
            previous = 0;
            continue;
        }

        uint8_t a = 0;
        uint8_t b = 0;
        add_to_checksum(header, sizeof(header), a, b);
        add_to_checksum(frame.payload.data(), frame.length, a, b);

        if (a == received_ck[0] && b == received_ck[1]) {
            return true;
        }

        g_bad_checksums++;
        previous = 0;
    }

    return false;
}


// 5. Sending configuration
struct KeyValue {
    uint32_t key;
    uint32_t value;
};

enum class Reply { Ack, Nak, None }; // ubx ack nak answer

// Bytes taken by a key's value, from the size code in the key's top digit.
size_t value_size(uint32_t key){
    uint32_t size_code = (key >> 28) & 0x07;

    if (size_code == 4) {
        return 4;
    } else if (size_code == 3) {
        return 2;
    } else {
        return 1;
    }
}

void send_valset(uint8_t layers, std::initializer_list<KeyValue> items){
    // 6 header + 4 VALSET header + up to 8 per item + 2 checksum.
    std::array<uint8_t, 128> frame = {};

    if (items.size() > 14) {
        ESP_LOGE(TAG, "BUG: too many keys in one CFG-VALSET");
        return;
    }

    frame[0] = SYNC_1;
    frame[1] = SYNC_2;
    frame[2] = CLASS_CFG;
    frame[3] = ID_VALSET;
    // payload
    frame[6] = 0x01;
    frame[7] = layers;
    frame[8] = 0x00;
    frame[9] = 0x00;
    size_t length = 10;

    for (const KeyValue &item : items) {
        for (size_t i = 0; i < 4; i++) {
            frame[length] = static_cast<uint8_t>(item.key >> (8 * i));
            length++;
        }
        for (size_t i = 0; i < value_size(item.key); i++) {
            frame[length] = static_cast<uint8_t>(item.value >> (8 * i));
            length++;
        }
    }

    size_t payload_length = length - 6;
    frame[4] = static_cast<uint8_t>(payload_length);
    frame[5] = static_cast<uint8_t>(payload_length >> 8);

    uint8_t a = 0;
    uint8_t b = 0;
    add_to_checksum(&frame[2], length - 2, a, b);
    frame[length] = a;
    frame[length + 1] = b;
    length += 2;

    uart_write_bytes(g_port, frame.data(), length);
}

Reply wait_for_ack(uint32_t timeout_ms){
    const uint32_t start = now_ms();
    Frame frame;

    while (true) {
        uint32_t elapsed = now_ms() - start;
        if (elapsed >= timeout_ms) {
            return Reply::None;
        }

        if (!read_frame(frame, timeout_ms - elapsed)) {
            continue;
        }

        bool answers_valset = (frame.msg_class == CLASS_ACK) && (frame.length >= 2) &&
                            (frame.payload[0] == CLASS_CFG) && (frame.payload[1] == ID_VALSET);

        if (answers_valset && frame.msg_id == ID_ACK) {
            return Reply::Ack;
        } else if (answers_valset && frame.msg_id == ID_NAK) {
            return Reply::Nak;
        } 
    }
}

Reply configure(uint8_t layers, std::initializer_list<KeyValue> items, uint32_t timeout_ms){
    uart_flush_input(g_port);
    send_valset(layers, items);
    return wait_for_ack(timeout_ms);
}

// 6. Finding the module
void set_baud(uint32_t baud){
    uart_set_baudrate(g_port, baud);
    uart_flush_input(g_port);
}

bool wait_for_nav_pvt(uint32_t timeout_ms){
    const uint32_t start = now_ms();
    Frame frame;

    while (true) {
        uint32_t elapsed = now_ms() - start;
        if (elapsed >= timeout_ms) {
            return false;
        }

        if (read_frame(frame, timeout_ms - elapsed) && frame.msg_class == CLASS_NAV && frame.msg_id == ID_NAV_PVT) {
            return true;
        }
    }
}

void set_dynamic_model(){
    Reply reply = configure(RAM_AND_BBR, { {KEY_DYNAMIC_MODEL, PEDESTRIAN} }, SETTING_ACK_TIMEOUT_MS);

    if (reply == Reply::None) {
        reply = configure(RAM_AND_BBR, { {KEY_DYNAMIC_MODEL, PEDESTRIAN} }, SETTING_ACK_TIMEOUT_MS);
    }

    if (reply == Reply::Ack) {
        ESP_LOGI(TAG, "Dynamic model: Bike");
    } else if (reply == Reply::None) {
        ESP_LOGW(TAG, "No answer to the dynamic model setting");
    } else {
        ESP_LOGW(TAG, "Could not set a dynamic model");
    }
}

bool find_module(){
    for (uint32_t baud : CANDIDATES_BAUDS) {
        ESP_LOGI(TAG, "Looking for the module at %" PRIu32 " baud...", baud);
        set_baud(baud);

        Reply reply = configure(RAM_AND_BBR, { {KEY_OUTPUT_UBX, 1}, {KEY_OUTPUT_NMEA, 0}, {KEY_NAV_PVT_RATE, 1}, 
                                             {KEY_MEAS_RATE_MS, MEASUREMENT_PERIOD_MS} }, SETTING_ACK_TIMEOUT_MS);

        if (reply == Reply::None) {
            ESP_LOGI(TAG, " no answer at %" PRIu32 " baud", baud);
            continue;
        }

        if (reply == Reply::Nak) {
            ESP_LOGE(TAG, "The module REFUSED the settings: probably a u-blox M8 or older, "
                            "which has no CFG-VALSET");
            return false;
        }

        ESP_LOGI(TAG, "Module found at %" PRIu32 " baud", baud);
        set_dynamic_model();

        if (configure(RAM_AND_BBR, { {KEY_L5_HEALTH_OVERRIDE, 1} }, SETTING_ACK_TIMEOUT_MS) == Reply::Ack) {
            ESP_LOGI(TAG, "GPS L5 enabled");
        } else {
            ESP_LOGI(TAG, "No GPS L5 on this module (single-band)");
        }

        if (baud != TARGET_BAUD) {
            send_valset(RAM_AND_BBR, { {KEY_UART_1_BAUD, TARGET_BAUD} });
            uart_wait_tx_done(g_port, pdMS_TO_TICKS(100));
            vTaskDelay(pdMS_TO_TICKS(100));
            set_baud(TARGET_BAUD);
        }

        if (!wait_for_nav_pvt(LINK_CHECK_MS)) {
            ESP_LOGW(TAG, "Module configured, but no NAV-PVT at %" PRIu32 " baud", TARGET_BAUD);
            return false;
        }

        return true;
    }

    ESP_LOGW(TAG, "No answer at any baud rate: check power, and module TX -> ESP32 RX");
    return false;
}

// 7. Decoding NAV-PVT
bool is_leap_year(int year){
    if (year % 400 == 0) {
        return true;
    } else if (year % 100 == 0) {
        return false;
    } else if (year % 4 == 0) {
        return true;
    } else {
        return false;
    }
}

// UTC date + time + signed nano -> ms since 1970-01-01
int64_t utc_to_unix_ms(const NavPvt &pvt){
    constexpr std::array<int, 12> DAYS_BEFORE_MONTH = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

    int64_t days = 0;
    for (int y = 1970; y < pvt.year; y++) {
        if (is_leap_year(y)) {
            days += 366;
        } else {
            days += 365;
        }
    }

    days += DAYS_BEFORE_MONTH[pvt.month - 1];
    if (pvt.month > 2 && is_leap_year(pvt.year)) {
        days += 1;
    }
    days += pvt.day - 1;

    int64_t seconds = days * 86400 + pvt.hour * 3600 + pvt.min * 60 + pvt.sec;
    return seconds * 1000 + std::llround(pvt.nano / 1000000.0);
}

// DECODE FRAME
bool decode_nav_pvt(const Frame &frame, Fix &fix){
    if (frame.msg_class != CLASS_NAV || frame.msg_id != ID_NAV_PVT || frame.length < sizeof(NavPvt)) {
        return false;
    }

    NavPvt pvt;
    std::memcpy(&pvt, frame.payload.data(), sizeof(pvt));

    // TIME RANGE CHECK
    fix.itow_ms = pvt.iTOW;
    bool time_flags_ok = (pvt.valid & 0x03) == 0x03;
    bool in_range = pvt.year >= 1970 && pvt.year <= 2200 && pvt.month >= 1 && pvt.month <= 12 &&
                    pvt.day >= 1 && pvt.day <= 31 && pvt.hour <= 23 && pvt.min <= 59 && pvt.sec <= 60 &&
                    pvt.nano > -1000000000 && pvt.nano < 1000000000;

    if (time_flags_ok && in_range) {
        fix.utc_valid = true;
        fix.utc_ms = utc_to_unix_ms(pvt);
    }

    // Quality. Usable = gnssFixOK, not invalidLlh, and a type with a position.
    fix.fix_type = pvt.fixType;
    fix.satellites = pvt.numSV;
    bool gnss_fix_ok = (pvt.flags & 0x01) != 0;
    bool invalid_llh = (pvt.flags3 & 0x01) != 0;
    bool type_position = pvt.fixType >= 2 && pvt.fixType <= 4;
    fix.fix_ok = gnss_fix_ok && !invalid_llh && type_position;

    fix.pdop = pvt.pDOP * 0.01;
    fix.h_acc_m = pvt.hAcc * 0.001;
    fix.v_acc_m = pvt.vAcc * 0.001;
    fix.speed_acc_kmh = pvt.sAcc * 0.0036; // mm/s -> km/h

    // Position and motion
    fix.lat_deg = pvt.lat * 1e-7;
    fix.lon_deg = pvt.lon * 1e-7;
    fix.alt_m = pvt.hMSL * 0.001;
    fix.vel_n_mps = pvt.velN * 0.001;
    fix.vel_e_mps = pvt.velE * 0.001;
    fix.vel_d_mps = pvt.velD * 0.001;
    fix.speed_kmh = pvt.gSpeed * 0.0036;
    fix.heading_deg = pvt.headMot * 1e-5;

    return true;
}


// 8. Rejection + filter
Reject basic_check(const Fix &fix){
    if (!fix.fix_ok) {
        return Reject::NoFix;
    } else if (fix.h_acc_m > MAX_H_ACC_M) {
        return Reject::PoorAccuracy;
    } else if (fix.speed_kmh > MAX_SPEED_KMH) {
        return Reject::TooFast;
    } else if (fix.alt_m < MIN_ALTITUDE_M || fix.alt_m > MAX_ALTITUDE_M) {
        return Reject::BadAltitude;
    } else {
        return Reject::None;
    }
}

void check_and_filter(Fix &fix){
    fix.reject = basic_check(fix);
    if (fix.reject != Reject::None) {
        return;
    }

    uint32_t dt_ms = fix.itow_ms - g_filter.itow_ms;

    // REFRESH EITHER FIRST FIX OR NOTHING BEING ACCEPTED FOR TOO LONG
    if (!g_filter.seeded || dt_ms > MAX_GAP_MS || dt_ms == 0) {
        g_filter.seeded = true;
        g_filter.lat_deg = fix.lat_deg;
        g_filter.lon_deg = fix.lon_deg;
        g_filter.alt_m = fix.alt_m;
        g_filter.vel_n_mps = fix.vel_n_mps;
        g_filter.vel_e_mps = fix.vel_e_mps;
        g_filter.vel_d_mps = fix.vel_d_mps;
        g_filter.itow_ms = fix.itow_ms;

        fix.filt_lat_deg = fix.lat_deg;
        fix.filt_lon_deg = fix.lon_deg;
        fix.filt_alt_m = fix.alt_m;

        g_filter.var_h = (fix.h_acc_m * r_scale) * (fix.h_acc_m * r_scale);
        g_filter.var_v = (fix.v_acc_m * r_scale) * (fix.v_acc_m * r_scale);
        return;
    }

    double dt = dt_ms / 1000.0;

    // ACCEL CHECK (HORIZONTAL)
    double accel = std::hypot(fix.vel_n_mps - g_filter.vel_n_mps, fix.vel_e_mps - g_filter.vel_e_mps) / dt;
    if (accel > MAX_ACCEL_MPS2) {
        fix.reject = Reject::TooMuchAccel;
        return;
    }

    // PREDICT PHASE
    double metres_per_deg_lon = METRES_PER_DEGREE * std::cos(g_filter.lat_deg * PI / 180.0);
    double delta_north_m = 0.5 * (fix.vel_n_mps + g_filter.vel_n_mps) * dt;
    double delta_east_m = 0.5 * (fix.vel_e_mps + g_filter.vel_e_mps) * dt;
    double delta_down_m = 0.5 * (fix.vel_d_mps + g_filter.vel_d_mps) * dt;

    double pred_lat = g_filter.lat_deg + delta_north_m / METRES_PER_DEGREE;
    double pred_lon = g_filter.lon_deg + delta_east_m / metres_per_deg_lon;
    double pred_alt = g_filter.alt_m - delta_down_m;

    // STATISTIC CALC
    double speed_acc_mps = fix.speed_acc_kmh / 3.6;
    double q = (q_scale * speed_acc_mps * dt) * (q_scale * speed_acc_mps * dt);
    double r_h = (fix.h_acc_m * r_scale) * (fix.h_acc_m * r_scale);
    double r_v = (fix.v_acc_m * r_scale) * (fix.v_acc_m * r_scale);
    double M_h = g_filter.var_h + q;
    double M_v = g_filter.var_v + q;

    // JUMP CHECK
    double miss_north_m = (fix.lat_deg - pred_lat) * METRES_PER_DEGREE;
    double miss_east_m = (fix.lon_deg - pred_lon) * metres_per_deg_lon;
    double miss_m = std::hypot(miss_north_m, miss_east_m);
    double gate_m = std::max<double>(MIN_GATE_M, GATE_SIGMAS * std::sqrt(M_h + r_h));

    if (miss_m > gate_m) {
        fix.reject = Reject::Jump;
        return;
    }

    // K GAIN CALC
    double k_gain_h = M_h / (M_h + r_h);
    double k_gain_v = M_v / (M_v + r_v);

    // BLENDING
    g_filter.lat_deg = pred_lat + k_gain_h * (fix.lat_deg - pred_lat);
    g_filter.lon_deg = pred_lon + k_gain_h * (fix.lon_deg - pred_lon);
    g_filter.alt_m = pred_alt + k_gain_v * (fix.alt_m - pred_alt);
    g_filter.vel_n_mps = fix.vel_n_mps;
    g_filter.vel_e_mps = fix.vel_e_mps;
    g_filter.vel_d_mps = fix.vel_d_mps;
    g_filter.itow_ms = fix.itow_ms;

    g_filter.var_h = (1 - k_gain_h) * M_h;
    g_filter.var_v = (1 - k_gain_v) * M_v;

    fix.filt_lat_deg = g_filter.lat_deg;
    fix.filt_lon_deg = g_filter.lon_deg;
    fix.filt_alt_m = g_filter.alt_m;
}


// 9. The task
// Hands one epoch to everyone who wants it, and keeps the monitor informed.
void publish(const Fix &fix){
    portENTER_CRITICAL(&g_lock);
    g_latest = fix;
    g_have_latest = true;
    portEXIT_CRITICAL(&g_lock);

    if (xQueueSend(g_queue, &fix, 0) != pdTRUE) {
        g_dropped++;
    }

    g_epochs++;
    if (fix.reject != Reject::None) {
        g_rejected++;
    }

    if (fix.fix_ok && !g_had_fix) {
        ESP_LOGI(TAG, "Fix acquired: %u satellites, hAcc %.1f m", static_cast<unsigned>(fix.satellites), fix.h_acc_m);
    } else if (!fix.fix_ok && g_had_fix) {
        ESP_LOGW(TAG, "Fix lost");
    }
    g_had_fix = fix.fix_ok;

    if (g_epochs % 100 == 0) {
        ESP_LOGI(TAG, "%u sats | hAcc %.2f m | %.1f km/h | rejected %" PRIu32 "/100 | ""bad checksums %" PRIu32 " | dropped %" PRIu32,
                 static_cast<unsigned>(fix.satellites), fix.h_acc_m, fix.speed_kmh, g_rejected, g_bad_checksums, g_dropped);
        g_rejected = 0;
    }
}

void gnss_cycle(){
    if (g_search_needed) {
        if (!find_module()) {
            set_baud(TARGET_BAUD);
            ESP_LOGW(TAG, "Listening at %" PRIu32 " baud anyway; searching again in 5 s if silent", TARGET_BAUD);
        }
        g_search_needed = false;
        g_last_pvt_ms = now_ms();
    }

    Frame frame;
    Fix fix;

    if (read_frame(frame, READ_WINDOW_MS) && decode_nav_pvt(frame, fix)) {
        fix.rx_ms = now_ms();
        g_last_pvt_ms = fix.rx_ms;
        check_and_filter(fix);
        publish(fix);
    }

    if ((now_ms() - g_last_pvt_ms) > LINK_LOST_MS) {
        ESP_LOGW(TAG, "No NAV-PVT for 5 s: searching for the module again");
        g_search_needed = true;
    }
}

void gnss_task(void *){
    vTaskDelay(pdMS_TO_TICKS(500));
    while (true) {
        gnss_cycle();
    }
}


// 10. Public functions
bool start(const Pins &pins){
    g_port = pins.uart;

    uart_config_t config = {};
    config.baud_rate = static_cast<int>(TARGET_BAUD);
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    if (uart_driver_install(g_port, UART_RX_BUFFER_BYTES, 0, 0, nullptr, 0) != ESP_OK ||
        uart_param_config(g_port, &config) != ESP_OK ||
        uart_set_pin(g_port, pins.tx_pin, pins.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "UART setup failed: check the port and pins in main.cpp");
        return false;
    }

    g_queue = xQueueCreate(QUEUE_LENGTH, sizeof(Fix));
    if (g_queue == nullptr) {
        ESP_LOGE(TAG, "Not enough memory for the epoch queue");
        return false;
    }

    if (xTaskCreate(gnss_task, "gnss", TASK_STACK_BYTES, nullptr, TASK_PRIORITY, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Could not start the GNSS task");
        return false;
    }

    return true;
}

bool wait_for_next_fix(Fix &fix, uint32_t timeout_ms){
    if (g_queue == nullptr) {
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        return false;
    }

    return xQueueReceive(g_queue, &fix, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

}
