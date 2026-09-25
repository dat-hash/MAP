// =========================================================================
// gnss_driver.cpp
// =========================================================================

#include "gnss_driver.hpp"

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <cinttypes>

#include "esp_log.h"


namespace gnss {

// =========================================================================
// Construction / destruction
// =========================================================================

// GnssDriver() hands off to GnssDriver(const Config&) with a default-built Config. One place defines what construction means.
GnssDriver::GnssDriver() : GnssDriver(Config{}) {}

GnssDriver::GnssDriver(const Config &config) : config_(config) {}

GnssDriver::~GnssDriver() {

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
    }

    // NO ESP_ERROR_CHECK a destructor should not abort the program.
    uart_driver_delete(config_.port);
}

// =========================================================================
// UART bring-up
// =========================================================================

void GnssDriver::configure_uart(uint32_t baud_rate) {
    
    uart_config_t uart_config = {
        .baud_rate  = static_cast<int>(baud_rate),
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {}
    };

    ESP_ERROR_CHECK(uart_param_config(config_.port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config_.port, config_.tx_pin, config_.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(config_.port, kUartRxBufferSize * 2, 0, 0, nullptr, 0));
}

// =========================================================================
// begin() / the background task
// =========================================================================

bool GnssDriver::begin() {

    configure_uart(config_.default_baud);
    vTaskDelay(pdMS_TO_TICKS(500));

    bool ready = false;

    for (int attempt = 1; attempt <= kMaxAttempts; attempt++) {

        ESP_LOGI(kTag, "GNSS configuration attempt %d of %d", attempt, kMaxAttempts);

        if (find_and_configure_module()) {
            ready = true;
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
        }
    }

    if (!ready) {
        ESP_LOGE(kTag,"Giving up on GNSS module after %d attempts -- check wiring/power. " "Continuing at %" PRIu32 " baud regardless, please send hopes and prayers.", kMaxAttempts, config_.target_baud);
        ESP_ERROR_CHECK(uart_set_baudrate(config_.port,config_.target_baud));
    }

    xTaskCreate(&GnssDriver::task_trampoline, "gnss_parser", kTaskStackBytes, this, kTaskPriority, &task_handle_);

    return ready;
}

void GnssDriver::task_trampoline(void *pvParameters) {
    static_cast<GnssDriver *>(pvParameters)->run_task();
}

void GnssDriver::run_task() {
    NmeaBuffer nmea_buffer{};
    Fix raw{};

    uint32_t valid_sentence_count = 0;
    uint32_t no_fix_count = 0;

    while (true) {
        if (!read_sentence(nmea_buffer, 2000)) {
            ESP_LOGW(kTag, "No valid NMEA sentence for 2 s -- is the module alive?");
            continue;
        }

        valid_sentence_count++;

        if (sentence_is(nmea_buffer.data(), "GGA")) {
            if (extract_gga(nmea_buffer.data(), raw)) {
                update_altitude_filter(raw);
            } else {
                // No fix this epoch: nothing to feed the altitude filter.
            }
        } else if (sentence_is(nmea_buffer.data(), "RMC")) {
            if (extract_rmc(nmea_buffer.data(), raw) && raw.fix_quality > 0) {
                update_position_filter(raw);

                if (filter_.position_seeded) {
                    Fix filtered{};
                    filtered.latitude    = filter_.latitude;
                    filtered.longitude   = filter_.longitude;
                    filtered.altitude    = filter_.altitude;
                    filtered.speed_kmh   = filter_.speed_kmh;
                    filtered.satellites  = raw.satellites;
                    filtered.fix_quality = raw.fix_quality;
                    filtered.hours       = raw.hours;
                    filtered.minutes     = raw.minutes;
                    filtered.seconds     = raw.seconds;

                    portENTER_CRITICAL(&lock_);
                    latest_filtered_ = filtered;
                    has_fix_ = true;
                    portEXIT_CRITICAL(&lock_);

                    // CTRL+T then CTRL+L to start and stop log to a file
                    ESP_LOGI(kTag, "%02uH-%02uM-%02uS| LAT: %.7f | LON: %.7f | ALT: %.2fm | SPEED: %.2f km/h | SAT: %u | FIX: %u",
                             raw.hours, raw.minutes, raw.seconds,
                             filter_.latitude, filter_.longitude,
                             filter_.altitude, filter_.speed_kmh,
                             raw.satellites, raw.fix_quality);
                } else {
                    // Filter not seeded yet; nothing meaningful to print.
                }
            } else {
                no_fix_count++;

                if ((no_fix_count % 100) == 0) {
                    ESP_LOGW(kTag, "%" PRIu32 " epochs parsed, still no GNSS fix (SAT: %u)", no_fix_count, raw.satellites);
                } else {
                    // Stay quiet between periodic reports.
                }
            }
        } else {
            // Any other sentence type: ignored.
        }
    }
}

// for future log or fusion
std::optional<Fix> GnssDriver::latest_fix() const {
    Fix copy{};
    bool have_one = false;

    portENTER_CRITICAL(&lock_);
    if (has_fix_) {
        copy = latest_filtered_;
        have_one = true;
    }
    portEXIT_CRITICAL(&lock_);

    if (have_one) {
        return copy;
    } else {
        return std::nullopt;
    }
}

// =========================================================================
// Module discovery / configuration
// =========================================================================

bool GnssDriver::find_and_configure_module() {
    const std::array<uint32_t, 2> candidate_baud_rates = {
        config_.target_baud, config_.default_baud
    };

    // for candidate_baud : candidate_baud_rates == for candidate_baud in candidate_baud_rates
    for (uint32_t candidate_baud : candidate_baud_rates) {
        ESP_LOGI(kTag, "Trying to reach GNSS module at %" PRIu32 " baud", candidate_baud);

        ESP_ERROR_CHECK(uart_set_baudrate(config_.port, candidate_baud));
        uart_flush_input(config_.port);
        vTaskDelay(pdMS_TO_TICKS(50));

        if (!send_main_config()) {
            ESP_LOGW(kTag, "No ACK at %" PRIu32 " baud", candidate_baud);
            continue;
        }

        ESP_LOGI(kTag, "GNSS module ACKed configuration at %" PRIu32 " baud", candidate_baud);

        if (candidate_baud == config_.target_baud) {
            // Already where we want it. No baud message, no flash write.
        } else {
            ESP_LOGI(kTag, "Switching module to %" PRIu32 " baud and persisting to flash", config_.target_baud);

            send_baud_config(config_.target_baud);
            vTaskDelay(pdMS_TO_TICKS(150));

            ESP_ERROR_CHECK(uart_set_baudrate(config_.port, config_.target_baud));
            uart_flush_input(config_.port);
        }

        if (link_is_alive(2000)) {
            ESP_LOGI(kTag, "GNSS link verified at %" PRIu32 " baud", config_.target_baud);
            return true;
        } else {
            ESP_LOGW(kTag, "Configured, but no valid NMEA at %" PRIu32 " baud -- retrying", config_.target_baud);
            continue;
        }
    }

    ESP_LOGE(kTag, "GNSS module did not respond at any known baud rate -- check wiring/power");
    return false;
}

bool GnssDriver::send_main_config() {
    UbxFrame frame(UbxFrame::Layer::Ram | UbxFrame::Layer::Bbr);

    bool ok = true;
    ok = ok && frame.append(kKeyNavspgDynmodel, static_cast<uint8_t>(config_.dynamic_model));
    ok = ok && frame.append(kKeyRateMeas, static_cast<uint16_t>(100));
    ok = ok && frame.append(kKeyUart1OutprotUbx, static_cast<uint8_t>(1));
    ok = ok && frame.append(kKeyUart1OutprotNmea, static_cast<uint8_t>(1));
    ok = ok && frame.append(kKeyMsgoutRmc, static_cast<uint8_t>(1));
    ok = ok && frame.append(kKeyMsgoutGga, static_cast<uint8_t>(1));
    ok = ok && frame.append(kKeyMsgoutGll, static_cast<uint8_t>(0));
    ok = ok && frame.append(kKeyMsgoutGsa, static_cast<uint8_t>(0));
    ok = ok && frame.append(kKeyMsgoutGsv, static_cast<uint8_t>(0));
    ok = ok && frame.append(kKeyMsgoutVtg, static_cast<uint8_t>(0));
    ok = ok && frame.append(kKeySignalGpsL5HealthOverride, static_cast<uint8_t>(1));

    if (!ok) {
        ESP_LOGE(kTag, "UBX frame buffer too small for the configured key list");
        return false;
    }

    size_t total = frame.finish();

    uart_flush_input(config_.port);
    uart_write_bytes(config_.port, reinterpret_cast<const char *>(frame.data()), total);

    return wait_for_ack(kUbxClassCfg, kUbxIdCfgValset, 500);
}

void GnssDriver::send_baud_config(uint32_t target_baud) {
    UbxFrame frame(UbxFrame::Layer::Ram | UbxFrame::Layer::Bbr | UbxFrame::Layer::Flash);

    if (!frame.append(kKeyUart1Baudrate, target_baud)) {
        ESP_LOGE(kTag, "UBX frame buffer too small for the baud rate key");
        return;
    }

    size_t total = frame.finish();
    uart_write_bytes(config_.port, reinterpret_cast<const char *>(frame.data()), total);
    ESP_ERROR_CHECK(uart_wait_tx_done(config_.port, pdMS_TO_TICKS(200)));
}

bool GnssDriver::link_is_alive(uint32_t timeout_ms) {
    NmeaBuffer scratch{};
    return read_sentence(scratch, timeout_ms);
}

bool GnssDriver::wait_for_ack(uint8_t acked_class, uint8_t acked_id, uint32_t timeout_ms) {
    uint8_t byte = 0;
    uint8_t state = 0;
    uint8_t payload_class = 0;
    bool is_nak = false;
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {

        int bytes_read = uart_read_bytes(config_.port, &byte, 1, pdMS_TO_TICKS(20));

        if (bytes_read <= 0) {
            continue;
        }

        if (state == 0) {
            if (byte == 0xB5) {
                state = 1;
            } 
            else {
                state = 0;
            }
        } 
        else if (state == 1) {
            if (byte == 0x62) {
                state = 2;
            } 
            else if (byte == 0xB5) {
                state = 1; // back-to-back sync bytes: stay armed
            } 
            else {
                state = 0;
            }
        } 
        else if (state == 2) {
            if (byte == kUbxClassAck) {
                state = 3;
            } 
            else {
                state = 0;
            }
        } 
        else if (state == 3) {
            if (byte == kUbxIdAckAck) {
                is_nak = false;
                state = 4;
            } 
            else if (byte == kUbxIdAckNak) {
                is_nak = true;
                state = 4;
            } 
            else {
                state = 0;
            }
        } 
        else if (state == 4) {
            state = 5; // length low byte
        } 
        else if (state == 5) {
            state = 6; // length high byte
        } 
        else if (state == 6) {
            payload_class = byte;
            state = 7;
        } 
        else if (state == 7) {
            uint8_t payload_id = byte;

            if (payload_class == acked_class && payload_id == acked_id) {
                return !is_nak;
            } 
            else {
                state = 0;
            }
        } else {
            state = 0;
        }
    }

    return false; // timed out
}

bool GnssDriver::read_sentence(NmeaBuffer &out, uint32_t timeout_ms) {
    uint8_t byte = 0;
    size_t buffer_index = 0;
    bool accumulating = false;
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        int bytes_read = uart_read_bytes(config_.port, &byte, 1, pdMS_TO_TICKS(20));

        if (bytes_read <= 0) {
            continue;
        }

        if (byte == '$') {
            buffer_index = 0;
            out[buffer_index++] = static_cast<char>(byte);
            accumulating = true;
            continue;
        }

        if (!accumulating) {
            continue;
        }

        if (buffer_index >= out.size() - 1) {
            buffer_index = 0;
            accumulating = false;
            continue;
        }

        out[buffer_index++] = static_cast<char>(byte);

        if (byte == '\n') {
            out[buffer_index] = '\0';
            accumulating = false;

            if (checksum_ok(out.data())) {
                return true;
            } else {
                continue;
            }
        }
    }

    return false;
}

// =========================================================================
// UbxFrame
// =========================================================================

GnssDriver::UbxFrame::UbxFrame(Layer layers) {
    frame_[index_++] = kSync1;
    frame_[index_++] = kSync2;
    frame_[index_++] = kClassCfg;
    frame_[index_++] = kIdCfgValset;
    frame_[index_++] = 0x00; // Length low byte, patched in finish()
    frame_[index_++] = 0x00; // Length high byte
    frame_[index_++] = 0x01; // Message version
    frame_[index_++] = static_cast<uint8_t>(layers);
    frame_[index_++] = 0x00; // Reserved
    frame_[index_++] = 0x00; // Reserved
}

void GnssDriver::UbxFrame::append_key_bytes(uint32_t key) {
    frame_[index_++] = static_cast<uint8_t>(key & 0xFF);
    frame_[index_++] = static_cast<uint8_t>((key >> 8) & 0xFF);
    frame_[index_++] = static_cast<uint8_t>((key >> 16) & 0xFF);
    frame_[index_++] = static_cast<uint8_t>((key >> 24) & 0xFF);
}

bool GnssDriver::UbxFrame::append(uint32_t key, uint8_t value) {
    if (!ok_) {
        return false;
    }

    if ((index_ + 4 + 1) > capacity_) {
        ok_ = false;
        return false;
    }

    append_key_bytes(key);
    frame_[index_++] = value;
    return true;
}

bool GnssDriver::UbxFrame::append(uint32_t key, uint16_t value) {
    if (!ok_) {
        return false;
    }

    if ((index_ + 4 + 2) > capacity_) {
        ok_ = false;
        return false;
    }

    append_key_bytes(key);
    frame_[index_++] = static_cast<uint8_t>(value & 0xFF);
    frame_[index_++] = static_cast<uint8_t>((value >> 8) & 0xFF);
    return true;
}

bool GnssDriver::UbxFrame::append(uint32_t key, uint32_t value) {
    if (!ok_) {
        return false;
    }

    if ((index_ + 4 + 4) > capacity_) {
        ok_ = false;
        return false;
    }

    append_key_bytes(key);
    frame_[index_++] = static_cast<uint8_t>(value & 0xFF);
    frame_[index_++] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame_[index_++] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame_[index_++] = static_cast<uint8_t>((value >> 24) & 0xFF);
    return true;
}

size_t GnssDriver::UbxFrame::finish() {
    size_t payload_length = index_ - 6;

    frame_[4] = static_cast<uint8_t>(payload_length & 0xFF);
    frame_[5] = static_cast<uint8_t>((payload_length >> 8) & 0xFF);

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (size_t i = 2; i < index_; i++) {
        ck_a = static_cast<uint8_t>(ck_a + frame_[i]);
        ck_b = static_cast<uint8_t>(ck_b + ck_a);
    }

    frame_[index_++] = ck_a;
    frame_[index_++] = ck_b;

    return index_;
}

// =========================================================================
// NMEA parsing 
// =========================================================================

bool GnssDriver::checksum_ok(const char *sentence) {
    if (sentence[0] != '$') {
        return false;
    }

    int i = 1;
    uint8_t computed = 0;

    while (sentence[i] != '*' && sentence[i] != '\0') {
        computed ^= static_cast<uint8_t>(sentence[i]);
        i++;
    }

    if (sentence[i] != '*') {
        return false;
    }

    if (!std::isxdigit(static_cast<unsigned char>(sentence[i + 1])) ||
        !std::isxdigit(static_cast<unsigned char>(sentence[i + 2]))) {
        return false;
    }

    uint8_t received = static_cast<uint8_t>(std::strtol(&sentence[i + 1], nullptr, 16));
    return computed == received;
}

double GnssDriver::degree_to_decimal(const char *coordinate, char direction) {
    if (coordinate == nullptr || std::strlen(coordinate) < 4) {
        return 0.0;
    }

    double raw_num = std::strtod(coordinate, nullptr);
    int degree = static_cast<int>(raw_num / 100.0);
    double minute = raw_num - (static_cast<double>(degree) * 100.0);
    double decimal = static_cast<double>(degree) + (minute / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    } else {
        // Northern / eastern hemisphere: sign stays positive.
    }

    return decimal;
}

void GnssDriver::parse_time(const char *time_field, Fix &fix) {

    if (time_field == nullptr || std::strlen(time_field) < 6) {
        return;
    }

    fix.hours   = static_cast<uint8_t>((time_field[0] - '0') * 10 + (time_field[1] - '0'));
    fix.minutes = static_cast<uint8_t>((time_field[2] - '0') * 10 + (time_field[3] - '0'));
    fix.seconds = static_cast<uint8_t>((time_field[4] - '0') * 10 + (time_field[5] - '0'));
}

bool GnssDriver::sentence_is(const char *sentence, const char *type3) {

    if (sentence[0] != '$') {
        return false;
    }

    return std::strncmp(&sentence[3], type3, 3) == 0;
}

int GnssDriver::split_fields(const char *sentence, char *scratch, size_t scratch_size, std::array<char *, kMaxFields> &field) {

    std::strncpy(scratch, sentence, scratch_size - 1);
    scratch[scratch_size - 1] = '\0';

    char *raw_read = scratch;
    char *token = nullptr;
    int field_index = 0;

    while (field_index < static_cast<int>(field.size())) {
        token = strsep(&raw_read, ",");

        if (token == nullptr) {
            break;
        }

        field[field_index++] = token;
    }

    return field_index;
}

bool GnssDriver::extract_rmc(const char *sentence, Fix &fix) {

    std::array<char, 83> sentence_copy{};
    std::array<char *, kMaxFields> field{};

    int field_index = split_fields(sentence, sentence_copy.data(), sentence_copy.size(), field);

    if (field_index > 7 && field[2] != nullptr && field[2][0] == 'A') {
        parse_time(field[1], fix);

        if (field[4] != nullptr && field[4][0] != '\0') {
            fix.latitude = degree_to_decimal(field[3], field[4][0]);
        } else {
            fix.latitude = degree_to_decimal(field[3], 'N');
        }

        if (field[6] != nullptr && field[6][0] != '\0') {
            fix.longitude = degree_to_decimal(field[5], field[6][0]);
        } else {
            fix.longitude = degree_to_decimal(field[5], 'E');
        }

        fix.speed_kmh = std::strtod(field[7], nullptr) * 1.852;
        return true;
    } else {
        return false;
    }
}

bool GnssDriver::extract_gga(const char *sentence, Fix &fix) {

    std::array<char, 83> sentence_copy{};
    std::array<char *, kMaxFields> field{};

    int field_index = split_fields(sentence, sentence_copy.data(), sentence_copy.size(), field);

    if (field_index > 9 && field[6] != nullptr && field[6][0] > '0') {
        parse_time(field[1], fix);
        fix.fix_quality = static_cast<uint8_t>(std::atoi(field[6]));
        fix.satellites  = static_cast<uint8_t>(std::atoi(field[7]));
        fix.altitude    = std::strtod(field[9], nullptr);
        return true;
    } else {
        fix.fix_quality = 0;

        if (field_index > 7 && field[7] != nullptr) {
            fix.satellites = static_cast<uint8_t>(std::atoi(field[7]));
        } else {
            fix.satellites = 0;
        }

        return false;
    }
}

double GnssDriver::distance_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {

    constexpr double kMetersPerDegreeLatitude = 111320.0;
    constexpr double kDegreesToRadians = M_PI / 180.0;

    double mean_lat_rad = ((lat1_deg + lat2_deg) / 2.0) * kDegreesToRadians;
    double delta_lat_deg = lat2_deg - lat1_deg;
    double delta_lon_deg = lon2_deg - lon1_deg;

    double north_m = delta_lat_deg * kMetersPerDegreeLatitude;
    double east_m  = delta_lon_deg * kMetersPerDegreeLatitude * std::cos(mean_lat_rad);

    return std::sqrt((north_m * north_m) + (east_m * east_m));
}

// =========================================================================
// Filter
// =========================================================================

void GnssDriver::update_position_filter(const Fix &raw) {
    if (!filter_.position_seeded) {
        filter_.latitude  = raw.latitude;
        filter_.longitude = raw.longitude;
        filter_.speed_kmh = raw.speed_kmh;
        filter_.position_seeded = true;
        return;
    }

    if (raw.speed_kmh > kMaxSpeedAllowKmh) {
        return;
    }

    double jump_m = distance_m(filter_.latitude, filter_.longitude, raw.latitude, raw.longitude);

    if (jump_m > kMaxPositionJumpM) {
        ESP_LOGW(kTag,"Rejected position jump of %.1f m", jump_m);
        return;
    }

    filter_.latitude = (kEmaAlphaPosition * raw.latitude) + ((1.0 - kEmaAlphaPosition) * filter_.latitude);
    filter_.longitude = (kEmaAlphaPosition * raw.longitude) + ((1.0 - kEmaAlphaPosition) * filter_.longitude);
    filter_.speed_kmh = (kEmaAlphaSpeed    * raw.speed_kmh) + ((1.0 - kEmaAlphaSpeed) * filter_.speed_kmh);
}

void GnssDriver::update_altitude_filter(const Fix &raw) {

    if (!filter_.altitude_seeded) {
        filter_.altitude = raw.altitude;
        filter_.altitude_seeded = true;
        return;
    }

    filter_.altitude = (kEmaAlphaAltitude * raw.altitude) + ((1.0 - kEmaAlphaAltitude) * filter_.altitude);
}

} // namespace gnss
