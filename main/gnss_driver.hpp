// =========================================================================
// gnss_driver.hpp
// =========================================================================

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <optional>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gnss {

// Fix
// GNSS FIX WITH IT METADATA

struct Fix {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double speed_kmh = 0.0;
    uint8_t satellites = 0;
    uint8_t fix_quality = 0;
    uint8_t hours = 0;
    uint8_t minutes = 0;
    uint8_t seconds = 0;
};

// -------------------------------------------------------------------------
// GnssDriver
//
// Owns one UART-connected u-blox receiver: brings it up, auto-discovers
// its baud rate, configures it over UBX, and runs a background FreeRTOS
// task that parses NMEA and keeps a filtered Fix available to the rest of
// the program via latest_fix().
// -------------------------------------------------------------------------
class GnssDriver {
public:
    // ---------------------------------------------------------------
    // Config
    // Everything needed to construct a driver for this board config
    // ---------------------------------------------------------------
    struct Config {
        // u-blox dynamic platform model. 
        enum class DynamicModel : uint8_t {
            Portable = 0, // default
            Stationary = 2, // survey mode
            Pedestrian = 3, // slowish < 30km/h
            Automotive = 4, // vehicle
            Sea = 5, // levry low alt
            Air1G = 6, // super fast and high
        };

        uart_port_t port = UART_NUM_2;
        int rx_pin = 16;
        int tx_pin = 17;
        uint32_t default_baud = 38400;
        uint32_t target_baud  = 115200;
        DynamicModel dynamic_model = DynamicModel::Pedestrian;
    };

    GnssDriver();
    explicit GnssDriver(const Config &config);
    ~GnssDriver();

    // This object owns a UART peripheral and a running FreeRTOS task.
    // Copying or moving it would mean two objects believing they each
    // own the same hardware. Deleting all four keeps that impossible
    GnssDriver(const GnssDriver &) = delete;
    GnssDriver &operator=(const GnssDriver &) = delete;
    GnssDriver(GnssDriver &&) = delete;
    GnssDriver &operator=(GnssDriver &&) = delete;

    // Brings up the UART, finds the module (trying the already-configured
    // baud rate first), configures it, and starts the background parsing
    // task. Returns true if the module was found and verified; the task
    // is started either way (aka send hopes and prayers)
    bool begin();

    // A snapshot of the most recent filtered fix. Returns std::nullopt 
    // until the filter has been seeded by at least one real measurement.
    std::optional<Fix> latest_fix() const;

private:
    // ---------------------------------------------------------------
    // Internal types
    // ---------------------------------------------------------------

    // Filter state
    struct FilterState {
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        double speed_kmh = 0.0;
        bool position_seeded = false;
        bool altitude_seeded = false;
    };

    // A fixed-capacity byte buffer for one NMEA sentence, including
    // the terminating NUL.
    using NmeaBuffer = std::array<char, 83>;

    // ---------------------------------------------------------------
    // UbxFrame
    //
    // Builds one UBX-CFG-VALSET frame byte by byte: sync bytes, header,
    // a run of key/value pairs, then length + checksum. 
    // ---------------------------------------------------------------
    class UbxFrame {
    public:
        enum class Layer : uint8_t {
            Ram = 0x01,
            Bbr = 0x02,
            Flash = 0x04,
        };

        // Combines two layers into one bitmask.
        friend constexpr Layer operator|(Layer a, Layer b) {
            return static_cast<Layer>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
        }

        explicit UbxFrame(Layer layers);

        // (ubx_append_key_value_u1/u2/u4).
        bool append(uint32_t key, uint8_t value);
        bool append(uint32_t key, uint16_t value);
        bool append(uint32_t key, uint32_t value);

        // Patches in the length, appends the checksum, and returns the
        // total frame size in bytes.
        size_t finish();

        const uint8_t *data() const { 
            return frame_.data(); 
        }

        size_t size() const { 
            return index_; 
        }

    private:

        void append_key_bytes(uint32_t key);

        static constexpr uint8_t kSync1 = 0xB5;
        static constexpr uint8_t kSync2 = 0x62;
        static constexpr uint8_t kClassCfg = 0x06;
        static constexpr uint8_t kIdCfgValset = 0x8A;

        std::array<uint8_t, 256> frame_{};
        size_t capacity_ = frame_.size() - 2; // reserve room for CK_A/CK_B
        size_t index_ = 0;
        bool ok_ = true;
    };


    static constexpr const char *kTag = "GNSS";

    static constexpr size_t kMaxFields = 20;

    static constexpr double kMaxSpeedAllowKmh = 350.0;
    static constexpr double kMaxPositionJumpM = 150.0;
    static constexpr double kEmaAlphaPosition = 0.20;
    static constexpr double kEmaAlphaAltitude = 0.05;
    static constexpr double kEmaAlphaSpeed = 0.20;

    static constexpr int kMaxAttempts = 10;
    static constexpr uint32_t kTaskStackBytes = 6144;
    static constexpr UBaseType_t kTaskPriority = 5;
    static constexpr int kUartRxBufferSize = 1024;

    static constexpr uint32_t kKeyUart1Baudrate = 0x40520001UL;
    static constexpr uint32_t kKeyUart1OutprotUbx = 0x10740001UL;
    static constexpr uint32_t kKeyUart1OutprotNmea = 0x10740002UL;
    static constexpr uint32_t kKeyMsgoutRmc = 0x209100acUL;
    static constexpr uint32_t kKeyMsgoutGga = 0x209100bbUL;
    static constexpr uint32_t kKeyMsgoutGll = 0x209100caUL;
    static constexpr uint32_t kKeyMsgoutGsa = 0x209100c0UL;
    static constexpr uint32_t kKeyMsgoutGsv = 0x209100c5UL;
    static constexpr uint32_t kKeyMsgoutVtg = 0x209100b1UL;
    static constexpr uint32_t kKeyRateMeas = 0x30210001UL;
    static constexpr uint32_t kKeyNavspgDynmodel = 0x20110021UL;
    static constexpr uint32_t kKeySignalGpsL5HealthOverride = 0x10320001UL;

    static constexpr uint8_t kUbxClassCfg = 0x06;
    static constexpr uint8_t kUbxIdCfgValset = 0x8A;
    static constexpr uint8_t kUbxClassAck = 0x05;
    static constexpr uint8_t kUbxIdAckAck = 0x01;
    static constexpr uint8_t kUbxIdAckNak = 0x00;

    // ---------------------------------------------------------------
    // FreeRTOS trampoline
    // ---------------------------------------------------------------
    static void task_trampoline(void *pvParameters);
    void run_task();

    // ---------------------------------------------------------------
    // UART bring-up / module discovery
    // ---------------------------------------------------------------
    void configure_uart(uint32_t baud_rate);
    bool find_and_configure_module();
    bool send_main_config();
    void send_baud_config(uint32_t target_baud);
    bool link_is_alive(uint32_t timeout_ms);
    bool wait_for_ack(uint8_t acked_class, uint8_t acked_id, uint32_t timeout_ms);
    bool read_sentence(NmeaBuffer &out, uint32_t timeout_ms);

    // ---------------------------------------------------------------
    // NMEA parsing 
    // ---------------------------------------------------------------
    static bool checksum_ok(const char *sentence);
    static double degree_to_decimal(const char *coordinate, char direction);
    static void parse_time(const char *time_field, Fix &fix);
    static bool sentence_is(const char *sentence, const char *type3);
    static int split_fields(const char *sentence, char *scratch, size_t scratch_size, std::array<char *, kMaxFields> &field);
    static bool extract_rmc(const char *sentence, Fix &fix);
    static bool extract_gga(const char *sentence, Fix &fix);
    static double distance_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

    // ---------------------------------------------------------------
    // Filter
    // ---------------------------------------------------------------
    void update_position_filter(const Fix &raw);
    void update_altitude_filter(const Fix &raw);

    // ---------------------------------------------------------------
    // Data members
    // ---------------------------------------------------------------
    Config config_;
    FilterState filter_{};
    Fix latest_filtered_{};
    bool has_fix_ = false;
    
    // lock mux
    mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;

    TaskHandle_t task_handle_ = nullptr;
};

} // namespace gnss
