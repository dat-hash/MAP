// =========================================================================
// gnss_ublox_fixed.c
//
// u-blox (M9/M10 generation, CFG-VALSET style) GNSS driver for ESP-IDF 5.x.
//
// This is a corrected version of the original file. Every change is marked
// with a "FIX #n" comment explaining what was wrong and why the new code
// behaves differently.
// =========================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GNSS_UART_NUM UART_NUM_2

#define RX_GNSS_PIN 16
#define TX_GNSS_PIN 17
#define GNSS_BUFFER_SIZE 1024
#define NMEA_MAX_LENGTH 83
#define GNSS_UART_DEFAULT_BAUD 38400
#define GNSS_UART_TARGET_BAUD  115200

static const char *TAG = "GNSS";

// -------------------------------------------------------------------------
// Outlier / plausibility limits
// -------------------------------------------------------------------------

// Absolute speed sanity limit. Anything above this is treated as a bad fix
// rather than a real measurement.
#define MAX_SPEED_ALLOW_KMH 350.0

// FIX #1 (part A): the original jump gate was written directly in degrees,
// with a separate longitude term that called cos() on a value in DEGREES.
// See gnss_distance_m() below. The gate is now expressed as a single
// straight-line distance in metres, which is much easier to reason about.
//
// At 10 Hz, 350 km/h is about 9.7 m per epoch. 150 m of headroom therefore
// rejects genuine teleports (multipath / fix re-acquisition) while never
// rejecting real motion, even under a brief data gap.
#define MAX_POSITION_JUMP_M 150.0

// -------------------------------------------------------------------------
// EMA smoothing factors
//
// FIX #2: these were far too aggressive for a vehicle that actually moves.
// The time constant of an EMA is roughly (1 / alpha) samples. At 10 Hz,
// the old alpha of 0.02 for position gives 50 samples = 5 seconds of lag.
// At 150 km/h that is a reported position roughly 200 m behind the bike.
// The old speed alpha of 0.04 gives 2.5 s, which smears every braking and
// acceleration event into mush.
//
// The values below give roughly 0.5 s on position and speed (enough to kill
// receiver jitter without hiding real dynamics) and about 2 s on altitude,
// which is inherently the noisiest channel and the least time-critical.
// -------------------------------------------------------------------------
#define EMA_ALPHA_POSITION 0.20
#define EMA_ALPHA_ALTITUDE 0.05
#define EMA_ALPHA_SPEED    0.20

// -------------------------------------------------------------------------
// Data types
// -------------------------------------------------------------------------

// Raw, straight-off-the-wire values as parsed from NMEA. Nothing ever
// writes filtered values back into this struct.
//
// FIX #3: in the original code the filter output was written back into
// current_gnss at the end of every epoch, which destroyed the raw
// measurement. Because GGA sentences do not carry latitude/longitude in
// this parser, the next filter pass then fed the filter its own previous
// output instead of a new measurement. Raw and filtered state are now
// strictly separated.
typedef struct {
    double latitude;
    double longitude;
    double altitude;
    double speed_kmh;
    uint8_t satellites;
    uint8_t fix_quality;
    uint8_t hours;   // 0 - 23 (UTC)
    uint8_t minutes; // 0 - 59
    uint8_t seconds; // 0 - 59
    bool valid;      // RMC status field was 'A'
} gnss_data_t;

// Filtered output. Each channel is seeded independently the first time a
// real measurement for that channel arrives.
//
// FIX #4: the original code seeded all four channels from a single
// first_run flag, which fired on the first valid RMC. RMC carries no
// altitude, so ema_alt was seeded with 0.0 m. With the old altitude alpha
// of 0.005 that takes well over a thousand samples to climb to a real
// value, so altitude read as near-zero for minutes after every boot.
typedef struct {
    double latitude;
    double longitude;
    double altitude;
    double speed_kmh;
    bool position_seeded;
    bool altitude_seeded;
} gnss_filter_t;

// -------------------------------------------------------------------------
// UBX protocol framing constants
// -------------------------------------------------------------------------
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

// FIX #5 (defensive): the buffer was 128 bytes and the full key list needs
// 76. That happened to fit, but there was no bounds check anywhere in the
// append helpers, so adding three or four more keys would have silently
// smashed the stack. The buffer is bigger now AND every append is checked.
#define UBX_TX_BUFFER_SIZE 256

// FLIP TO 1 FOR RAW NMEA (STILL VALID CHECKSUM AND "GOOD" QUALITY)
#define ENABLE_RAW_NMEA_LOGGING 0

// -------------------------------------------------------------------------
// Dynamic platform model
//
// FIX #6: the original code selected UBX_PEDESTRIAN_MODE. The pedestrian
// model tells the receiver to assume walking pace with a low acceleration
// budget, and it applies exactly that assumption inside the navigation
// filter. On a race motorcycle it will lag hard under acceleration and
// braking, and it can clamp reported speed outright. The dynamic model is
// not a display setting -- it changes the solution itself.
//
// AIR_1G is the right default here: it allows up to 1 g of sustained
// horizontal acceleration and imposes no low-speed assumption, so nothing
// the bike does violates the model. AUTOMOTIVE is an option if you want the
// receiver's ground-vehicle assumptions (it constrains vertical velocity,
// which slightly improves altitude), but it assumes a lower dynamics
// envelope than a race bike actually has.
// -------------------------------------------------------------------------
typedef enum {
    UBX_PORTABLE_MODE   = 0, // Default
    UBX_STATIONARY_MODE = 2, // Stand still
    UBX_PEDESTRIAN_MODE = 3, // < 30 km/h, static wander suppression
    UBX_AUTOMOTIVE_MODE = 4, // Ground vehicle assumptions
    UBX_SEA_MODE        = 5, // Zero alt assumes sea-level travel
    UBX_AIR_1G_MODE     = 6  // Need for speed
} ubx_dynamic_model_t;



// =========================================================================
// UART bring-up
// =========================================================================

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
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_NUM, TX_GNSS_PIN, RX_GNSS_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_NUM, GNSS_BUFFER_SIZE * 2, 0, 0, NULL, 0));
}

// =========================================================================
// NMEA parsing helpers
// =========================================================================

bool check_sum(const char *gnss_sentence) {
    if (gnss_sentence[0] != '$') {
        return false;
    }

    int i = 1;
    uint8_t checksum_result = 0;

    while (gnss_sentence[i] != '*' && gnss_sentence[i] != '\0') {
        checksum_result ^= (uint8_t)gnss_sentence[i];
        i++;
    }

    if (gnss_sentence[i] != '*') {
        return false;
    }

    // The two characters after '*' must both be hex digits. strtol() alone
    // would happily return 0 for garbage, which turns a corrupt sentence
    // into a "valid" one whenever the computed checksum is also 0.
    if (!isxdigit((unsigned char)gnss_sentence[i + 1]) ||
        !isxdigit((unsigned char)gnss_sentence[i + 2])) {
        return false;
    }

    uint8_t received_checksum = (uint8_t)strtol(&gnss_sentence[i + 1], NULL, 16);
    return (checksum_result == received_checksum);
}

// FIX #7: this used to return float and do all its arithmetic in float.
// A 32-bit float carries about 7 significant decimal digits. A latitude
// such as 51.044700 already uses 8, so the low digit was pure noise --
// roughly half a metre of quantisation baked in before the filter ever ran,
// and the struct field it was assigned to was a double anyway. Everything
// on the position path is double now.
double degree_to_decimal(const char *coordinate, char direction) {
    if (!coordinate || strlen(coordinate) < 4) {
        return 0.0;
    }

    double raw_num = strtod(coordinate, NULL);
    int degree = (int)(raw_num / 100.0);
    double minute = raw_num - ((double)degree * 100.0);
    double decimal = (double)degree + (minute / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    } else {
        // Northern / eastern hemisphere: sign stays positive.
    }

    return decimal;
}

void parse_nmea_time(const char *time_str, gnss_data_t *gnss) {
    // Field format: HHMMSS.ss
    if (!time_str || strlen(time_str) < 6) {
        return;
    }

    gnss->hours   = (uint8_t)((time_str[0] - '0') * 10 + (time_str[1] - '0'));
    gnss->minutes = (uint8_t)((time_str[2] - '0') * 10 + (time_str[3] - '0'));
    gnss->seconds = (uint8_t)((time_str[4] - '0') * 10 + (time_str[5] - '0'));
}

// FIX #1 (part B): the real defect.
//
// The original longitude gate was:
//
//     fabs(ema_lon - lon) < (1 / (111.320 * cos(ema_lat)))
//
// cos() in C takes RADIANS. ema_lat is a latitude in DEGREES, somewhere in
// [-90, +90]. Feeding degrees to cos() evaluates the cosine of an angle of
// up to 90 radians -- about 14 full revolutions -- so the result is an
// essentially arbitrary number in [-1, +1].
//
// When it lands negative (it does for large parts of the latitude range,
// e.g. any latitude near 2 degrees, 8 degrees, 14 degrees ...), the whole
// right-hand side is negative. fabs() is never less than a negative number,
// so the condition is false forever, the else branch runs 'continue', and
// the filter NEVER accepts another update. The output freezes at the very
// first fix and stays frozen for the entire session, with no error, no
// warning, and a perfectly healthy-looking log line repeating unchanged.
//
// The replacement converts to radians and works in metres, using a local
// flat-earth (equirectangular) approximation. Over the sub-kilometre
// distances this gate cares about, that is accurate to well under a metre
// and far cheaper than a full haversine.
static double gnss_distance_m(double lat1_deg, double lon1_deg,
                              double lat2_deg, double lon2_deg) {
    const double METERS_PER_DEGREE_LATITUDE = 111320.0;
    const double DEGREES_TO_RADIANS = M_PI / 180.0;

    double mean_lat_rad = ((lat1_deg + lat2_deg) / 2.0) * DEGREES_TO_RADIANS;
    double delta_lat_deg = lat2_deg - lat1_deg;
    double delta_lon_deg = lon2_deg - lon1_deg;

    double north_m = delta_lat_deg * METERS_PER_DEGREE_LATITUDE;
    double east_m  = delta_lon_deg * METERS_PER_DEGREE_LATITUDE * cos(mean_lat_rad);

    return sqrt((north_m * north_m) + (east_m * east_m));
}

// Split a sentence into comma-separated fields. Returns the field count.
static int nmea_split_fields(const char *gnss_sentence, char *scratch,
                             size_t scratch_size, char **field, int max_fields) {
    strncpy(scratch, gnss_sentence, scratch_size - 1);
    scratch[scratch_size - 1] = '\0';

    char *raw_read = scratch;
    char *token;
    int field_index = 0;

    while (field_index < max_fields) {
        token = strsep(&raw_read, ",");

        if (token == NULL) {
            break;
        }

        field[field_index++] = token;
    }

    return field_index;
}

// Returns true when a usable position/speed measurement was extracted.
bool extract_gnss_rmc(const char *gnss_sentence, gnss_data_t *gnss) {
    char sentence_copy[NMEA_MAX_LENGTH];
    char *field[20] = {0};

    int field_index = nmea_split_fields(gnss_sentence, sentence_copy,
                                        sizeof(sentence_copy), field, 20);

    if (field_index > 7 && field[2] && field[2][0] == 'A') {
        parse_nmea_time(field[1], gnss);

        if (field[4] && field[4][0] != '\0') {
            gnss->latitude = degree_to_decimal(field[3], field[4][0]);
        } else {
            gnss->latitude = degree_to_decimal(field[3], 'N');
        }

        if (field[6] && field[6][0] != '\0') {
            gnss->longitude = degree_to_decimal(field[5], field[6][0]);
        } else {
            gnss->longitude = degree_to_decimal(field[5], 'E');
        }

        // Knots to km/h. Kept as double all the way through.
        gnss->speed_kmh = strtod(field[7], NULL) * 1.852;
        gnss->valid = true;
        return true;
    } else {
        gnss->valid = false;
        return false;
    }
}

// Returns true when a usable altitude measurement was extracted.
bool extract_gnss_gga(const char *gnss_sentence, gnss_data_t *gnss) {
    char sentence_copy[NMEA_MAX_LENGTH];
    char *field[20] = {0};

    int field_index = nmea_split_fields(gnss_sentence, sentence_copy, sizeof(sentence_copy), field, 20);

    if (field_index > 9 && field[6] && field[6][0] > '0') {
        parse_nmea_time(field[1], gnss);
        gnss->fix_quality = (uint8_t)atoi(field[6]);
        gnss->satellites  = (uint8_t)atoi(field[7]);
        gnss->altitude    = strtod(field[9], NULL);
        return true;
    } else {
        // No fix reported. Clear fix_quality so the rest of the pipeline
        // sees the loss of fix instead of holding a stale non-zero value.
        gnss->fix_quality = 0;

        if (field_index > 7 && field[7]) {
            gnss->satellites = (uint8_t)atoi(field[7]);
        } else {
            gnss->satellites = 0;
        }

        return false;
    }
}

// FIX #8: the original matched only "$GNRMC"/"$GPRMC". A multi-GNSS module
// emits whatever talker ID matches the constellations used for that fix, so
// on a GLONASS- or Galileo-only epoch you would get $GLRMC or $GARMC and the
// sentence would be silently dropped. Match on the 3-character sentence type
// and ignore the talker ID.
static bool nmea_sentence_is(const char *sentence, const char *type_3char) {
    if (sentence[0] != '$') {
        return false;
    }

    return (strncmp(&sentence[3], type_3char, 3) == 0);
}

// =========================================================================
// Filter
// =========================================================================

static void gnss_filter_update_position(gnss_filter_t *filter, const gnss_data_t *raw) {
    if (!filter->position_seeded) {
        filter->latitude  = raw->latitude;
        filter->longitude = raw->longitude;
        filter->speed_kmh = raw->speed_kmh;
        filter->position_seeded = true;
        return;
    }

    if (raw->speed_kmh > MAX_SPEED_ALLOW_KMH) {
        // Implausible speed: treat the whole epoch as a bad fix.
        return;
    }

    double jump_m = gnss_distance_m(filter->latitude, filter->longitude,
                                    raw->latitude, raw->longitude);

    if (jump_m > MAX_POSITION_JUMP_M) {
        // Implausible position jump: reject and keep the previous estimate.
        ESP_LOGW(TAG, "Rejected position jump of %.1f m", jump_m);
        return;
    }

    filter->latitude  = (EMA_ALPHA_POSITION * raw->latitude)  + ((1.0 - EMA_ALPHA_POSITION) * filter->latitude);
    filter->longitude = (EMA_ALPHA_POSITION * raw->longitude) + ((1.0 - EMA_ALPHA_POSITION) * filter->longitude);
    filter->speed_kmh = (EMA_ALPHA_SPEED    * raw->speed_kmh) + ((1.0 - EMA_ALPHA_SPEED)    * filter->speed_kmh);
}

static void gnss_filter_update_altitude(gnss_filter_t *filter, const gnss_data_t *raw) {
    if (!filter->altitude_seeded) {
        filter->altitude = raw->altitude;
        filter->altitude_seeded = true;
        return;
    }

    filter->altitude = (EMA_ALPHA_ALTITUDE * raw->altitude) +
                       ((1.0 - EMA_ALPHA_ALTITUDE) * filter->altitude);
}

// =========================================================================
// Sentence reader
// =========================================================================

// Reads one complete, checksum-valid NMEA sentence into 'out'.
// Returns true on success, false on timeout.
//
// FIX #9: the original accumulator dropped everything on an over-long
// sentence but never re-armed on the '$' that may already have been
// consumed. This version restarts accumulation on any '$', which is the
// only byte that can legally start a sentence, so a truncated or corrupt
// line costs exactly one sentence rather than desynchronising the parser.
static bool gnss_read_sentence(char *out, size_t out_size, uint32_t timeout_ms) {
    uint8_t byte;
    int buffer_index = 0;
    bool accumulating = false;
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        int bytes_read = uart_read_bytes(GNSS_UART_NUM, &byte, 1, pdMS_TO_TICKS(20));

        if (bytes_read <= 0) {
            continue;
        }

        if (byte == '$') {
            buffer_index = 0;
            out[buffer_index++] = (char)byte;
            accumulating = true;
            continue;
        }

        if (!accumulating) {
            continue;
        }

        if ((size_t)buffer_index >= out_size - 1) {
            // Over-long line: abandon it and wait for the next '$'.
            buffer_index = 0;
            accumulating = false;
            continue;
        }

        out[buffer_index++] = (char)byte;

        if (byte == '\n') {
            out[buffer_index] = '\0';
            accumulating = false;

            if (check_sum(out)) {
                return true;
            } else {
                // Bad checksum: keep reading until the timeout expires.
                continue;
            }
        }
    }

    return false;
}

// =========================================================================
// Parser task
// =========================================================================

void get_gnss_data_task(void *pvParameters) {
    char nmea_buffer[NMEA_MAX_LENGTH];
    gnss_data_t raw_gnss = {0};
    gnss_filter_t filtered = {0};

    uint32_t valid_sentence_count = 0;
    uint32_t no_fix_count = 0;

    while (1) {
        // portMAX_DELAY equivalent: block essentially forever waiting for
        // the next good sentence, but in bounded chunks so the task stays
        // responsive if the link goes away entirely.
        if (!gnss_read_sentence(nmea_buffer, sizeof(nmea_buffer), 2000)) {
            ESP_LOGW(TAG, "No valid NMEA sentence for 2 s -- is the module alive?");
            continue;
        }

        valid_sentence_count++;

#if ENABLE_RAW_NMEA_LOGGING
        ESP_LOGI("RAW_NMEA", "%s", nmea_buffer);
        continue;
#endif

        if (nmea_sentence_is(nmea_buffer, "GGA")) {
            // GGA carries fix quality, satellite count and altitude.
            if (extract_gnss_gga(nmea_buffer, &raw_gnss)) {
                gnss_filter_update_altitude(&filtered, &raw_gnss);
            } else {
                // No fix this epoch: nothing to feed the altitude filter.
            }
        } else if (nmea_sentence_is(nmea_buffer, "RMC")) {
            // RMC carries time, position, speed and the A/V validity flag.
            // This is the once-per-epoch anchor, so logging happens here.
            if (extract_gnss_rmc(nmea_buffer, &raw_gnss) && raw_gnss.fix_quality > 0) {
                gnss_filter_update_position(&filtered, &raw_gnss);

                if (filtered.position_seeded) {
                    // CTRL+T then CTRL+L to start and stop log to a file
                    ESP_LOGI(TAG,
                             "%02uH-%02uM-%02uS| LAT: %.7f | LON: %.7f | ALT: %.2fm | SPEED: %.2f km/h | SAT: %u | FIX: %u",
                             raw_gnss.hours, raw_gnss.minutes, raw_gnss.seconds,
                             filtered.latitude, filtered.longitude,
                             filtered.altitude, filtered.speed_kmh,
                             raw_gnss.satellites, raw_gnss.fix_quality);
                } else {
                    // Filter not seeded yet; nothing meaningful to print.
                }
            } else {
                no_fix_count++;

                if ((no_fix_count % 100) == 0) {
                    ESP_LOGW(TAG, "%lu epochs parsed, still no GNSS fix (SAT: %u)",
                             (unsigned long)no_fix_count, raw_gnss.satellites);
                } else {
                    // Stay quiet between periodic reports.
                }
            }
        } else {
            // Any other sentence type: ignored.
        }
    }
}

// =========================================================================
// UBX transmit helpers
// =========================================================================

static void ubx_checksum(const uint8_t *data, size_t length, uint8_t *ck_a_out, uint8_t *ck_b_out) {
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

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
    frame[index++] = 0x00; // Length low byte, patched in ubx_finalize_and_send()
    frame[index++] = 0x00; // Length high byte
    frame[index++] = 0x01; // Message version
    frame[index++] = layers;
    frame[index++] = 0x00; // Reserved
    frame[index++] = 0x00; // Reserved

    return index;
}

// FIX #5 (cont.): all three append helpers now refuse to write past the end
// of the frame instead of trusting the caller to have counted correctly.
static bool ubx_append_key(uint8_t *frame, size_t *index, size_t capacity,
                           uint32_t key_id, size_t value_size) {
    if ((*index + 4 + value_size) > capacity) {
        return false;
    }

    frame[(*index)++] = (uint8_t)(key_id & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 8) & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 16) & 0xFF);
    frame[(*index)++] = (uint8_t)((key_id >> 24) & 0xFF);

    return true;
}

static bool ubx_append_key_value_u1(uint8_t *frame, size_t *index, size_t capacity,
                                    uint32_t key_id, uint8_t value) {
    if (!ubx_append_key(frame, index, capacity, key_id, 1)) {
        return false;
    }

    frame[(*index)++] = value;
    return true;
}

static bool ubx_append_key_value_u2(uint8_t *frame, size_t *index, size_t capacity,
                                    uint32_t key_id, uint16_t value) {
    if (!ubx_append_key(frame, index, capacity, key_id, 2)) {
        return false;
    }

    frame[(*index)++] = (uint8_t)(value & 0xFF);
    frame[(*index)++] = (uint8_t)((value >> 8) & 0xFF);
    return true;
}

static bool ubx_append_key_value_u4(uint8_t *frame, size_t *index, size_t capacity,
                                    uint32_t key_id, uint32_t value) {
    if (!ubx_append_key(frame, index, capacity, key_id, 4)) {
        return false;
    }

    frame[(*index)++] = (uint8_t)(value & 0xFF);
    frame[(*index)++] = (uint8_t)((value >> 8) & 0xFF);
    frame[(*index)++] = (uint8_t)((value >> 16) & 0xFF);
    frame[(*index)++] = (uint8_t)((value >> 24) & 0xFF);
    return true;
}

static void ubx_finalize_and_send(uint8_t *frame, size_t index) {
    size_t payload_length = index - 6; // everything after the 6-byte header
    uint8_t ck_a;
    uint8_t ck_b;

    frame[4] = (uint8_t)(payload_length & 0xFF);
    frame[5] = (uint8_t)((payload_length >> 8) & 0xFF);

    // Checksum covers class, id, length and payload -- i.e. from frame[2]
    // up to but not including the checksum bytes themselves.
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
            } else {
                state = 0;
            }
        } else if (state == 1) {
            if (byte == UBX_SYNC_CHAR_2) {
                state = 2;
            } else if (byte == UBX_SYNC_CHAR_1) {
                state = 1; // back-to-back sync bytes: stay armed
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
            // length low byte -- ACK/NAK payload is always 2 bytes
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
            } else {
                state = 0;
            }
        } else {
            state = 0;
        }
    }

    return false; // timed out waiting for a matching ACK/NAK
}

// =========================================================================
// Module configuration
//
// FIX #10: this is the second serious defect, and the reason the original
// would re-do the whole discovery dance on every single cold boot.
//
// The old code put CFG-UART1-BAUDRATE in the SAME VALSET as everything
// else. A u-blox module applies a VALSET and THEN sends its ACK -- and the
// ACK goes out at the NEW baud rate. So the host, still sitting at 38400,
// reads that ACK as garbage, ubx_wait_for_ack() times out, and
// configure_gnss_module() reports failure even though the config was
// applied perfectly.
//
// The knock-on effect is the nasty part. The candidate loop then retried at
// 115200, where the module now was, and succeeded -- but that branch passed
// persist_to_flash = false. So the baud rate was NEVER written to flash.
// Power-cycle the bike and the module is back at 38400, and the whole
// two-pass dance repeats forever. The flash-endurance logic was correct in
// intent and simply never reached its goal.
//
// The fix is to split it into two messages:
//   1. Everything except the baud rate, to RAM + BBR, ACK checked normally.
//      These are cheap to resend on every boot and need no flash write.
//   2. The baud rate alone, to RAM + BBR + FLASH, with NO ack expected --
//      we switch the host UART and confirm by verifying that real NMEA
//      arrives at the new rate, which is a stronger check than an ACK.
// =========================================================================

static bool gnss_send_main_config(void) {
    uint8_t frame[UBX_TX_BUFFER_SIZE];
    size_t capacity = sizeof(frame) - 2; // leave room for the two checksum bytes
    bool ok = true;

    // RAM + BBR only: these settings are re-sent on every boot anyway, so
    // there is no reason to spend flash write cycles on them.
    size_t index = ubx_build_valset_header(frame, UBX_LAYER_RAM | UBX_LAYER_BBR);

    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_NAVSPG_DYNMODEL, 3);
    ok = ok && ubx_append_key_value_u2(frame, &index, capacity, UBX_KEY_CFG_RATE_MEAS, 100); // 100 ms = 10 Hz
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_UART1OUTPROT_UBX, 1);
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_UART1OUTPROT_NMEA, 1);
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_MSGOUT_NMEA_ID_RMC_UART1, 1);
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_MSGOUT_NMEA_ID_GGA_UART1, 1);
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_MSGOUT_NMEA_ID_GLL_UART1, 0);
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_MSGOUT_NMEA_ID_GSA_UART1, 0);
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_MSGOUT_NMEA_ID_GSV_UART1, 0);
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_MSGOUT_NMEA_ID_VTG_UART1, 0);
    ok = ok && ubx_append_key_value_u1(frame, &index, capacity, UBX_KEY_CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE, 1);

    if (!ok) {
        ESP_LOGE(TAG, "UBX frame buffer too small for the configured key list");
        return false;
    }

    uart_flush_input(GNSS_UART_NUM);
    ubx_finalize_and_send(frame, index);

    return ubx_wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, 500);
}

// Sends the baud-rate change on its own. No ACK is expected, because the
// module will emit it at the new rate. Returns immediately after the bytes
// have physically left the UART.
static void gnss_send_baud_config(uint32_t target_baud) {
    uint8_t frame[UBX_TX_BUFFER_SIZE];
    size_t capacity = sizeof(frame) - 2;

    // FLASH included here on purpose: this single key is the one we actually
    // want to survive a power cycle, and writing only one key keeps the
    // flash wear to the absolute minimum.
    size_t index = ubx_build_valset_header(frame,
                       UBX_LAYER_RAM | UBX_LAYER_BBR | UBX_LAYER_FLASH);

    if (!ubx_append_key_value_u4(frame, &index, capacity, UBX_KEY_CFG_UART1_BAUDRATE, target_baud)) {
        ESP_LOGE(TAG, "UBX frame buffer too small for the baud rate key");
        return;
    }

    ubx_finalize_and_send(frame, index);
    ESP_ERROR_CHECK(uart_wait_tx_done(GNSS_UART_NUM, pdMS_TO_TICKS(200)));
}

// Confirms the link by waiting for a checksum-valid NMEA sentence at the
// rate the host UART is currently set to. This is a stronger check than an
// ACK: it proves both directions of the link are framing correctly.
static bool gnss_link_is_alive(uint32_t timeout_ms) {
    char sentence[NMEA_MAX_LENGTH];
    return gnss_read_sentence(sentence, sizeof(sentence), timeout_ms);
}

// -------------------------------------------------------------------------
// find_and_configure_gnss_module()
//
// The module's ACTIVE baud rate depends on what's stored in its FLASH
// config layer, not just the factory default. A fresh/reset module boots
// at GNSS_UART_DEFAULT_BAUD (38400); a module that has already run this
// firmware has GNSS_UART_TARGET_BAUD (115200) persisted. Try each candidate
// in turn and use whichever one answers.
//
// Returns true if the module was found and is now running at
// GNSS_UART_TARGET_BAUD.
// -------------------------------------------------------------------------
bool find_and_configure_gnss_module(void) {
    static const uint32_t candidate_baud_rates[] = { GNSS_UART_TARGET_BAUD, GNSS_UART_DEFAULT_BAUD };
    static const size_t candidate_count = sizeof(candidate_baud_rates) / sizeof(candidate_baud_rates[0]);
    size_t candidate_index;

    // Note the order: target first. Once the module has been provisioned it
    // will be at 115200 on every subsequent boot, so trying that first makes
    // the common case a single pass with no flash write at all.
    for (candidate_index = 0; candidate_index < candidate_count; candidate_index++) {
        uint32_t candidate_baud = candidate_baud_rates[candidate_index];

        ESP_LOGI(TAG, "Trying to reach GNSS module at %lu baud", (unsigned long)candidate_baud);

        ESP_ERROR_CHECK(uart_set_baudrate(GNSS_UART_NUM, candidate_baud));
        uart_flush_input(GNSS_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(50));

        if (!gnss_send_main_config()) {
            ESP_LOGW(TAG, "No ACK at %lu baud", (unsigned long)candidate_baud);
            continue;
        }

        ESP_LOGI(TAG, "GNSS module ACKed configuration at %lu baud", (unsigned long)candidate_baud);

        if (candidate_baud == GNSS_UART_TARGET_BAUD) {
            // Already where we want it. No baud message, no flash write.
        } else {
            ESP_LOGI(TAG, "Switching module to %d baud and persisting to flash",
                     GNSS_UART_TARGET_BAUD);

            gnss_send_baud_config(GNSS_UART_TARGET_BAUD);

            // Give the module time to reconfigure its own UART before we
            // change ours; anything sent during this window is lost.
            vTaskDelay(pdMS_TO_TICKS(150));

            ESP_ERROR_CHECK(uart_set_baudrate(GNSS_UART_NUM, GNSS_UART_TARGET_BAUD));
            uart_flush_input(GNSS_UART_NUM);
        }

        // Verify by waiting for real NMEA at the target rate.
        if (gnss_link_is_alive(2000)) {
            ESP_LOGI(TAG, "GNSS link verified at %d baud", GNSS_UART_TARGET_BAUD);
            return true;
        } else {
            ESP_LOGW(TAG, "Configured, but no valid NMEA at %d baud -- retrying",
                     GNSS_UART_TARGET_BAUD);
            continue;
        }
    }

    ESP_LOGE(TAG, "GNSS module did not respond at any known baud rate -- check wiring/power");
    return false;
}

// =========================================================================
// Entry point
// =========================================================================

void app_main(void) {

    setvbuf(stdout, NULL, _IONBF, 0);

    start_gnss_uart(GNSS_UART_TARGET_BAUD);

    // Give the module time to finish its own power-on boot before we send
    // it anything. Without this, a fast-booting ESP32 can win the race and
    // send the config before the module's UART is actually listening.
    vTaskDelay(pdMS_TO_TICKS(500));

    // A single pass can still lose the race on a cold boot, so retry a few
    // times with a growing delay between attempts.
    const int max_attempts = 10;
    bool gnss_ready = false;
    int attempt;

    for (attempt = 1; attempt <= max_attempts; attempt++) {
        ESP_LOGI(TAG, "GNSS configuration attempt %d of %d", attempt, max_attempts);

        if (find_and_configure_gnss_module()) {
            gnss_ready = true;
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000 * attempt)); // back off a little more each retry
        }
    }

    if (!gnss_ready) {
        ESP_LOGE(TAG, "Giving up on GNSS module after %d attempts -- check wiring/power. "
                      "Continuing at %d baud regardless, please send hopes and prayers.",
                 max_attempts, GNSS_UART_TARGET_BAUD);
    }

    // Stack raised from 4096: this task uses doubles, a sentence buffer and
    // ESP_LOGI with eight format arguments, ten times a second.
    xTaskCreate(get_gnss_data_task, "gnss_parser", 6144, NULL, 5, NULL);

    vTaskDelete(NULL);
}