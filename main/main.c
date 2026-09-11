#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define GNSS_UART_NUM UART_NUM_2
#define RX_GNSS_PIN 16
#define TX_GNSS_PIN 17
#define GNSS_BUFFER_SIZE 1024
#define NMEA_MAX_LENGTH 83
#define GNSS_UART_DEFAULT_BAUD 38400
#define GNSS_UART_TARGET_BAUD  115200
static const char *TAG = "GNSS_PARSER";

// tunable alpha for ema
#define ema_alpha_position 0.02
#define ema_alpha_altitude 0.005
#define ema_alpha_speed 0.04

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed_kmh;
    uint8_t satellites;
    uint8_t fix_quality;
    uint8_t hours;   // 0 - 23 (UTC)
    uint8_t minutes; // 0 - 59
    uint8_t seconds; // 0 - 59
    bool valid;
} gnss_data_t;

// UBX Protocol Framing Constants
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_CLASS_CFG 0x06
#define UBX_ID_CFG_VALSET 0x8A
#define UBX_CLASS_ACK 0x05
#define UBX_ID_ACK_ACK 0x01
#define UBX_ID_ACK_NAK 0x00

#define UBX_LAYER_RAM 0x01
#define UBX_LAYER_BBR 0x02
#define UBX_LAYER_FLASH 0x04

#define UBX_KEY_CFG_UART1_BAUDRATE 0x40520001UL
#define UBX_KEY_CFG_UART1OUTPROT_UBX 0x10740001UL
#define UBX_KEY_CFG_UART1OUTPROT_NMEA 0x10740002UL
#define UBX_KEY_CFG_MSGOUT_NMEA_ID_RMC_UART1 0x209100acUL
#define UBX_KEY_CFG_MSGOUT_NMEA_ID_GGA_UART1 0x209100bbUL
#define UBX_KEY_CFG_MSGOUT_NMEA_ID_GLL_UART1 0x209100caUL
#define UBX_KEY_CFG_MSGOUT_NMEA_ID_GSA_UART1 0x209100c0UL
#define UBX_KEY_CFG_MSGOUT_NMEA_ID_GSV_UART1 0x209100c5UL
#define UBX_KEY_CFG_MSGOUT_NMEA_ID_VTG_UART1 0x209100b1UL
#define UBX_KEY_CFG_RATE_MEAS 0x30210001UL
#define UBX_KEY_CFG_NAVSPG_DYNMODEL 0x20110021UL
#define UBX_KEY_CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE 0x10320001UL

#define UBX_TX_BUFFER_SIZE 128

// FLIP TO 1 FOR RAW NMEA (STILL VALID CHECKSUM AND "GOOD" QUALITY)
#define ENABLE_RAW_NMEA_LOGGING 0

// CHEAT CODE
typedef enum {
    UBX_PORTABLE_MODE  = 0, // Default 
    UBX_STATIONARY_MODE = 2, // Stand still
    UBX_PEDESTRIAN_MODE = 3, // < 30 km/h) but static wander
    UBX_AUTOMOTIVE_MODE = 4, // < 100 km/h
    UBX_SEA_MODE        = 5, // Zero alt assumes sea-level travel
    UBX_AIR_1G_MODE     = 6  // Need for speed
} VIRTUAL_KALMAN_CHEAT;

void start_gnss_uart(uint32_t baud_rate) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(GNSS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_NUM, TX_GNSS_PIN, RX_GNSS_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_NUM, GNSS_BUFFER_SIZE * 2, 0, 0, NULL, 0));
}

bool check_sum(const char *gnss_sentence) {
    if (gnss_sentence[0] != '$') return false;

    int i = 1;
    uint8_t checksum_result = 0;

    while (gnss_sentence[i] != '*' && gnss_sentence[i] != '\0') {
        checksum_result ^= (uint8_t)gnss_sentence[i];
        i++;
    }

    if (gnss_sentence[i] != '*') return false;

    uint8_t received_checksum = (uint8_t)strtol(&gnss_sentence[i + 1], NULL, 16);
    return (checksum_result == received_checksum);
}

float degree_to_decimal(const char *coordinate, char direction) {
    if (!coordinate || strlen(coordinate) < 4) return 0.0f;

    float raw_num = atof(coordinate);
    int degree = (int)(raw_num / 100);
    float minute = raw_num - (degree * 100);
    float decimal = degree + (minute / 60.0f);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

void parse_nmea_time (const char *time_str, gnss_data_t *gnss){
    // Field format: HHMMSS.ss
    if (!time_str ||strlen(time_str) < 6){
        return;
    }
    gnss->hours = (uint8_t)((time_str[0] - '0') * 10 + (time_str[1]- '0'));
    gnss->minutes = (uint8_t)((time_str[2] - '0') * 10 + (time_str[3]- '0'));
    gnss->seconds = (uint8_t)((time_str[4] - '0') * 10 + (time_str[5]- '0'));

}

void extract_gnss_rmc(const char *gnss_sentence, gnss_data_t *gnss) {
    char sentence_copy[NMEA_MAX_LENGTH];
    strncpy(sentence_copy, gnss_sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';

    char *raw_read = sentence_copy;
    char *token;
    char *field[20] = {0};
    int field_index = 0;

    while ((token = strsep(&raw_read, ",")) != NULL && field_index < 20) {
        field[field_index++] = token;
    }

    if (field_index > 7 && field[2] && field[2][0] == 'A') {
        parse_nmea_time(field[1],gnss);

        if (field[4]) {
            gnss->latitude = degree_to_decimal(field[3], field[4][0]);
        }
        else {
            gnss->latitude = degree_to_decimal(field[3], 'N');
        }

        if (field[6]) {
            gnss->longitude = degree_to_decimal(field[5], field[6][0]);
        }
        else {
            gnss->longitude = degree_to_decimal(field[5],'E');
        }

        gnss->speed_kmh = atof(field[7]) * 1.852f;
        gnss->valid = true;
    } else {
        gnss->valid = false;
    }
}

void extract_gnss_gga(const char *gnss_sentence, gnss_data_t *gnss) {
    char sentence_copy[NMEA_MAX_LENGTH];
    strncpy(sentence_copy, gnss_sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';

    char *raw_read = sentence_copy;
    char *token;
    char *field[20] = {0};
    int field_index = 0;

    while ((token = strsep(&raw_read, ",")) != NULL && field_index < 20) {
        field[field_index++] = token;
    }

    if (field_index > 9 && field[6] && field[6][0] > '0') {
        parse_nmea_time(field[1],gnss);
        gnss->fix_quality = (uint8_t)atoi(field[6]);
        gnss->satellites = (uint8_t)atoi(field[7]);
        gnss->altitude   = atof(field[9]);
    } else if (field_index > 6 && field[6]) {
        gnss->fix_quality = 0;
    }
}

void get_gnss_data_task(void *pvParameters) {
    float ema_lat = 0.0f;
    float ema_lon = 0.0f;
    float ema_alt = 0.0f;
    float ema_speed = 0.0f;
    bool first_run = true;
    uint8_t byte;
    char nmea_buffer[NMEA_MAX_LENGTH];
    int buffer_index = 0;
    bool accumulation = false;
    gnss_data_t current_gnss = {0};

    uint32_t valid_sentence_count = 0;
    uint32_t checksum_fail_count = 0;

    while (1) {
        if (uart_read_bytes(GNSS_UART_NUM, &byte, 1, portMAX_DELAY) > 0) {
            if (byte == '$') {
                buffer_index = 0;
                nmea_buffer[buffer_index++] = (char)byte;
                accumulation = true;
                continue;
            }

            if (accumulation) {
                if (buffer_index >= NMEA_MAX_LENGTH - 1) {
                    buffer_index = 0;
                    accumulation = false;
                    continue;
                }

                nmea_buffer[buffer_index++] = (char)byte;

                if (byte == '\n') {
                    nmea_buffer[buffer_index] = '\0';
                    accumulation = false;

                    if (check_sum(nmea_buffer)) {
                        valid_sentence_count++;

                        if (strncmp(nmea_buffer, "$GNRMC", 6) == 0 || strncmp(nmea_buffer, "$GPRMC", 6) == 0) {
                            extract_gnss_rmc(nmea_buffer, &current_gnss);
                        } else if (strncmp(nmea_buffer, "$GNGGA", 6) == 0 || strncmp(nmea_buffer, "$GPGGA", 6) == 0) {
                            extract_gnss_gga(nmea_buffer, &current_gnss);
                        }

                        if (current_gnss.valid && current_gnss.fix_quality > 0) {
                                if(first_run){
                                    ema_lat = current_gnss.latitude;
                                    ema_lon = current_gnss.longitude;
                                    ema_alt = current_gnss.altitude;
                                    ema_speed = current_gnss.speed_kmh;
                                    first_run = false;
                                }
                                else{
                                    ema_lat = ema_alpha_position * current_gnss.latitude + (1-ema_alpha_position)*ema_lat;
                                    ema_lon = ema_alpha_position * current_gnss.longitude + (1-ema_alpha_position)*ema_lon;
                                    ema_alt = ema_alpha_altitude * current_gnss.altitude + (1-ema_alpha_altitude)*ema_alt;
                                    ema_speed = ema_alpha_speed * current_gnss.speed_kmh + (1-ema_alpha_speed)*ema_speed;
                                }

                            current_gnss.latitude  = ema_lat;
                            current_gnss.longitude = ema_lon;
                            current_gnss.altitude  = ema_alt;
                            current_gnss.speed_kmh = ema_speed;
                            
                        #if ENABLE_RAW_NMEA_LOGGING
                            ESP_LOGI("RAW_NMEA", "%s", nmea_buffer);
                        #else
                            // CTRL+T then CTRL+L to start and stop log to a file
                            ESP_LOGI(TAG, "TIME: %dH-%dM-%dS| LAT: %.6f | LON: %.6f | ALT: %.6fm | SPEED: %.6f km/h | SAT: %d | FIX: %d", 
                                current_gnss.hours,current_gnss.minutes,current_gnss.seconds,current_gnss.latitude, current_gnss.longitude,
                                 current_gnss.altitude, current_gnss.speed_kmh, current_gnss.satellites, current_gnss.fix_quality);
                        #endif
                        } else if ((valid_sentence_count % 100) == 0) {
                            ESP_LOGW(TAG, "%lu valid NMEA sentences parsed, still no GNSS fix", (unsigned long)valid_sentence_count);
                        }
                    } else {
                        checksum_fail_count++;
                        ESP_LOGD(TAG, "Checksum fail (%lu total)", (unsigned long)checksum_fail_count);
                    }
                }
            }
        }
    }
}

static void ubx_checksum(const uint8_t *data, size_t length, uint8_t *ck_a_out, uint8_t *ck_b_out) {
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 0; i < length; i++) {
        ck_a += data[i];
        ck_b += ck_a;
    }
    *ck_a_out = ck_a;
    *ck_b_out = ck_b;
}

static size_t ubx_build_valset_header(uint8_t *frame, uint8_t layers) {
    size_t index = 0;
    frame[index++] = UBX_SYNC_CHAR_1;
    frame[index++] = UBX_SYNC_CHAR_2;
    frame[index++] = UBX_CLASS_CFG;
    frame[index++] = UBX_ID_CFG_VALSET;
    frame[index++] = 0x00; // Reserved length low byte
    frame[index++] = 0x00; // Reserved length high byte
    frame[index++] = 0x01; // Message version
    frame[index++] = layers;
    frame[index++] = 0x00;
    frame[index++] = 0x00;
    return index;
}

static void ubx_append_key_value_u1(uint8_t *frame, size_t *index, uint32_t key_id, uint8_t value) {
    frame[(*index)++] = (uint8_t)(key_id & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 8) & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 16) & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 24) & 0xFF);
    frame[(*index)++] = value;
}

static void ubx_append_key_value_u4(uint8_t *frame, size_t *index, uint32_t key_id, uint32_t value) {
    frame[(*index)++] = (uint8_t)(key_id & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 8) & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 16) & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 24) & 0xFF);
    frame[(*index)++] = (uint8_t)(value & 0xFF);
    frame[(*index)++] = (uint8_t)((value >> 8) & 0xFF);
    frame[(*index)++] = (uint8_t)((value >> 16) & 0xFF);
    frame[(*index)++] = (uint8_t)((value >> 24) & 0xFF);
}

static void ubx_append_key_value_u2(uint8_t *frame, size_t *index, uint32_t key_id, uint16_t value) {
    frame[(*index)++] = (uint8_t)(key_id & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 8) & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 16) & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 24) & 0xFF);
    frame[(*index)++] = (uint8_t)(value & 0xFF);
    frame[(*index)++] = (uint8_t)((value >> 8) & 0xFF);
}

static void ubx_finalize_and_send(uint8_t *frame, size_t index) {
    size_t payload_length = index - 6;
    uint8_t ck_a, ck_b;

    frame[4] = (uint8_t)(payload_length & 0xFF);
    frame[5] = (uint8_t)((payload_length >> 8) & 0xFF);

    ubx_checksum(&frame[2], index - 2, &ck_a, &ck_b);

    frame[index++] = ck_a;
    frame[index++] = ck_b;

    uart_write_bytes(GNSS_UART_NUM, (const char *)frame, index);
}

// -------------------------------------------------------------------------
// ubx_wait_for_ack()
//
// Reads bytes off the GNSS UART, byte by byte, looking for a UBX-ACK-ACK
// (class 0x05, id 0x01) or UBX-ACK-NAK (0x05, 0x00) whose payload names
// the class/id of the message we're waiting on. Returns true only on a
// matching ACK within timeout_ms; false on a NAK, a non-matching ACK/NAK,
// or timeout.
// -------------------------------------------------------------------------
static bool ubx_wait_for_ack(uint8_t acked_class, uint8_t acked_id, uint32_t timeout_ms) {
    uint8_t byte;
    uint8_t state = 0;
    uint8_t payload_class = 0;
    bool is_nak = false;
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        int bytes_read = uart_read_bytes(GNSS_UART_NUM, &byte, 1, pdMS_TO_TICKS(20));

        if (bytes_read <= 0) {
            continue;
        }

        if (state == 0) {
            if (byte == UBX_SYNC_CHAR_1) {
                state = 1;
            }
        } else if (state == 1) {
            if (byte == UBX_SYNC_CHAR_2) {
                state = 2;
            } else {
                state = 0;
            }
        } else if (state == 2) {
            if (byte == UBX_CLASS_ACK) {
                state = 3;
            } else {
                state = 0;
            }
        } else if (state == 3) {
            if (byte == UBX_ID_ACK_ACK) {
                is_nak = false;
                state = 4;
            } else if (byte == UBX_ID_ACK_NAK) {
                is_nak = true;
                state = 4;
            } else {
                state = 0;
            }
        } else if (state == 4) {
            // length low byte -- ACK/NAK payload is always 2 bytes, not validated here
            state = 5;
        } else if (state == 5) {
            // length high byte
            state = 6;
        } else if (state == 6) {
            payload_class = byte; // class of the message being acked/nak'd
            state = 7;
        } else if (state == 7) {
            uint8_t payload_id = byte; // id of the message being acked/nak'd
            if (payload_class == acked_class && payload_id == acked_id) {
                return !is_nak;
            }
            state = 0;
        }
    }

    return false; // timed out waiting for a matching ACK/NAK
}

// Returns true if the module ACKed the configuration (so it's safe to
// switch the host UART to GNSS_UART_TARGET_BAUD), false otherwise (in
// which case the module is still at whatever baud it was already on).
//
// 'persist_to_flash' controls whether the FLASH layer is included.
// FLASH survives a full power cycle, but has finite write endurance --
// on hardware that gets power-cycled often (e.g. every time you switch
// the bike's ignition on), writing to it on every single boot wears it
// down for no benefit once the value already matches. Pass true only
// the first time you provision a fresh/reset module; every later boot
// should pass false and rely on RAM + BBR, which is all a boot actually
// needs since the FLASH layer already holds the right value.
bool configure_gnss_module(bool persist_to_flash) {
    uint8_t frame[UBX_TX_BUFFER_SIZE];
    uint8_t layers = UBX_LAYER_RAM | UBX_LAYER_BBR;

    if (persist_to_flash) {
        layers |= UBX_LAYER_FLASH;
    }

    size_t index = ubx_build_valset_header(frame, layers);

    // config the gnss board
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_NAVSPG_DYNMODEL, UBX_PEDESTRIAN_MODE);
    ubx_append_key_value_u4(frame, &index, UBX_KEY_CFG_UART1_BAUDRATE, GNSS_UART_TARGET_BAUD);
    ubx_append_key_value_u2(frame, &index, UBX_KEY_CFG_RATE_MEAS, 100);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_UART1OUTPROT_UBX, 1);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_UART1OUTPROT_NMEA, 1);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_MSGOUT_NMEA_ID_RMC_UART1, 1);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_MSGOUT_NMEA_ID_GGA_UART1, 1);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_MSGOUT_NMEA_ID_GLL_UART1, 0);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_MSGOUT_NMEA_ID_GSA_UART1, 0);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_MSGOUT_NMEA_ID_GSV_UART1, 0);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_MSGOUT_NMEA_ID_VTG_UART1, 0);
    ubx_append_key_value_u1(frame, &index, UBX_KEY_CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE, 1);

    ubx_finalize_and_send(frame, index);

    // wait for the board to say ok to the config so to switch to target bauld
    if (!ubx_wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, 300)) {
        return false;
    }

    return true;
}

// -------------------------------------------------------------------------
// find_and_configure_gnss_module()
//
// The module's ACTIVE baud rate depends on what's stored in its FLASH
// config layer, not just the factory default. A fresh/reset module boots
// at GNSS_UART_DEFAULT_BAUD (38400), but a module that has previously run
// this firmware may already have GNSS_UART_TARGET_BAUD (115200) persisted
// to flash -- so we can't assume which one it'll be on. Try each
// candidate rate in turn and use whichever one gets an ACK.
//
// Returns true if the module was found and is now running at
// GNSS_UART_TARGET_BAUD; false if it didn't respond at any known rate
// (a wiring problem, or a rate outside this candidate list).
// -------------------------------------------------------------------------
bool find_and_configure_gnss_module(void) {
    static const uint32_t candidate_baud_rates[] = { GNSS_UART_DEFAULT_BAUD, GNSS_UART_TARGET_BAUD };
    static const size_t candidate_count = sizeof(candidate_baud_rates) / sizeof(candidate_baud_rates[0]);
    size_t candidate_index;

    for (candidate_index = 0; candidate_index < candidate_count; candidate_index++) {
        uint32_t candidate_baud = candidate_baud_rates[candidate_index];

        ESP_LOGI(TAG, "Trying to reach GNSS module at %lu baud", (unsigned long)candidate_baud);

        ESP_ERROR_CHECK(uart_set_baudrate(GNSS_UART_NUM, candidate_baud));
        uart_flush_input(GNSS_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(50));

        // Only persist to flash the very first time we discover the
        // module isn't already at our target baud -- once it's there,
        // later boots skip the flash write entirely (see the comment on
        // configure_gnss_module()).
        bool persist_to_flash = (candidate_baud != GNSS_UART_TARGET_BAUD);

        if (configure_gnss_module(persist_to_flash)) {
            ESP_LOGI(TAG, "GNSS module responded at %lu baud, now configured for %d baud",
                     (unsigned long)candidate_baud, GNSS_UART_TARGET_BAUD);

            ESP_ERROR_CHECK(uart_wait_tx_done(GNSS_UART_NUM, pdMS_TO_TICKS(100)));
            vTaskDelay(pdMS_TO_TICKS(150));
            ESP_ERROR_CHECK(uart_set_baudrate(GNSS_UART_NUM, GNSS_UART_TARGET_BAUD));
            uart_flush_input(GNSS_UART_NUM);
            return true;
        }
    }

    ESP_LOGE(TAG, "GNSS module did not respond at any known baud rate -- check wiring/power");
    return false;
}

void app_main(void) {

    setvbuf(stdout, NULL, _IONBF, 0);

    // Start at whichever candidate we'll try first; find_and_configure_gnss_module()
    // will switch the host baud as it works through the candidate list.
    start_gnss_uart(GNSS_UART_DEFAULT_BAUD);

    // Give the module time to finish its own power-on boot before we send
    // it anything. Without this, a fast-booting ESP32 can win the race and
    // send the config before the module's UART is actually listening.
    vTaskDelay(pdMS_TO_TICKS(500));

    // A single pass through find_and_configure_gnss_module() can still
    // lose the race on a cold boot -- module boot timing isn't perfectly
    // consistent every power-up. Rather than accept failure after one
    // try, retry a few times with a growing delay between attempts. This
    // is what turns "sometimes it just never comes up until I re-power
    // it" into "it takes an extra second or two, every time."
    const int max_attempts = 10;
    bool gnss_ready = false;
    int attempt;

    for (attempt = 1; attempt <= max_attempts; attempt++) {
        ESP_LOGI(TAG, "GNSS configuration attempt %d of %d", attempt, max_attempts);

        if (find_and_configure_gnss_module()) {
            gnss_ready = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(300 * attempt)); // back off a little more each retry
    }

    if (!gnss_ready) {
        ESP_LOGE(TAG, "Giving up on GNSS module after %d attempts -- check wiring/power. Continuing at %d baud regardless, please send hopes and prayers.",
                 max_attempts, GNSS_UART_TARGET_BAUD);
    }

    xTaskCreate(get_gnss_data_task, "gnss_parser", 4096, NULL, 5, NULL);

    vTaskDelete(NULL);
}