#pragma once

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <ctime>  

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "gnss_driver.hpp"

namespace IMU{

constexpr const char *TAG = "IMU";

struct Pins {
    spi_host_device_t spi_host;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
};


// ---- SPI (3.12, 6.1) ----
constexpr int IMU_SPI_CLOCK_HZ = 5000000; // chip allows 0.095 to 10.5 MHz; use 1 MHz on jumper wires
constexpr uint8_t IMU_SPI_MODE = 0; // CPOL 0, CPHA 0
constexpr int IMU_FRAME_BYTES = 6; // 48 bits
 
// ---- Request layout, MOSI (6.2 Figure 14) ----
// [47:46] TA9:TA8 = 0 | [45:38] address | [37] RW | [36] 0 | [35] FT | [34:28] AE = 0 | [27:8] DATAI | [7:0] CRC
constexpr int IMU_FRAME_ADDR_SHIFT = 38; // register address, bits 45..38
constexpr int IMU_FRAME_RW_SHIFT = 37; // 1 = write, 0 = read
constexpr int IMU_FRAME_FT_SHIFT = 35; // always 1 = next reply is a 48-bit frame
constexpr int IMU_FRAME_DATA_SHIFT = 8; // DATAI is 20 bits (27..8); 16-bit values sit at the bottom (23..8)
constexpr int IMU_FRAME_CRC_BITS = 8;
 
// ---- Reply layout, MISO (6.2 Figure 14) ----
// [47] D | [46:37] address | [36] IDS | [35] CE | [34:33] S1:S0 | [32:29] DCNT | [28] unused | [27:8] data | [7:0] CRC
constexpr int IMU_REPLY_D_SHIFT = 47; // 1 = gyro/accel/temp data, 0 = anything else
constexpr int IMU_REPLY_ADDR_SHIFT = 37; // 10 bits, 46..37 (one lower than in the request)
constexpr int IMU_REPLY_IDS_SHIFT = 36; // 1 = internal (common) error
constexpr int IMU_REPLY_CE_SHIFT = 35; // 1 = your previous frame was invalid
constexpr int IMU_REPLY_STATUS_SHIFT = 33; // S1:S0, bits 34..33
constexpr int IMU_REPLY_DCNT_SHIFT = 29; // 4-bit sample counter, bits 32..29
constexpr int IMU_REPLY_DATA_SHIFT = 8; // data field 27..8
 
// ---- Masks: apply after shifting down ----
constexpr uint64_t IMU_MASK_ADDR = 0xFF;
constexpr uint64_t IMU_MASK_ADDR10 = 0x3FF; // reply address incl. TA9:TA8
constexpr uint64_t IMU_MASK_ERRORS = 0x3; // IDS + CE together, after >> IMU_REPLY_CE_SHIFT
constexpr uint64_t IMU_MASK_STATUS = 0x3;
constexpr uint64_t IMU_MASK_DCNT = 0xF;
constexpr uint64_t IMU_MASK_DATA16 = 0xFFFF; // status/config registers (bits 23..8)
constexpr uint64_t IMU_MASK_DATA20 = 0xFFFFF; // rate/accel/temp (bits 27..8)
constexpr uint64_t IMU_MASK_CRC = 0xFF;
constexpr uint32_t IMU_DATA20_SIGN_BIT = 0x80000; // bit 19
constexpr uint32_t IMU_DATA20_SIGN_EXTEND = 0xFFF00000; // bits 31..20
 
// ---- Status codes S1:S0 (6.4) ----
// Always 00 in the reply to a write.
constexpr uint8_t IMU_STATUS_OK = 0b00;
constexpr uint8_t IMU_STATUS_ERROR = 0b01;
constexpr uint8_t IMU_STATUS_SATURATED = 0b10; // value is real but clipped: keep it, set a flag
constexpr uint8_t IMU_STATUS_INIT = 0b11; // still starting up
 
// ---- CRC (6.5.1) ----
constexpr uint8_t IMU_CRC_POLY = 0x2F; // generator 0x12F, top bit implied
constexpr uint8_t IMU_CRC_INIT = 0xFF;
 
// ---- Registers (7.1.2) ----
constexpr uint8_t IMU_REG_RATE_X = 0x0A; // decimated gyro: X 0x0A, Y 0x0B, Z 0x0C
constexpr uint8_t IMU_REG_ACC_X = 0x0D; // decimated accel: X 0x0D, Y 0x0E, Z 0x0F
constexpr uint8_t IMU_REG_TEMP = 0x10;
constexpr uint8_t IMU_REG_STAT0 = 0x14; // STAT_SUM; 10 status registers 0x14..0x1D
constexpr uint8_t IMU_REG_FILT_RATE = 0x25;
constexpr uint8_t IMU_REG_FILT_ACC12 = 0x26;
constexpr uint8_t IMU_REG_FILT_ACC3 = 0x27;
constexpr uint8_t IMU_REG_RATE_CTRL = 0x28;
constexpr uint8_t IMU_REG_ACC12_CTRL = 0x29;
constexpr uint8_t IMU_REG_ACC3_CTRL = 0x2A;
constexpr uint8_t IMU_REG_USER_IF = 0x33;
constexpr uint8_t IMU_REG_MODE = 0x35;
constexpr uint8_t IMU_REG_RESET = 0x36;
constexpr uint8_t IMU_REG_SYS_TEST = 0x37; // free R/W register, not locked by EOI: first bring-up test
constexpr uint8_t IMU_REG_ASIC_ID = 0x3B;
constexpr uint8_t IMU_REG_COMP_ID = 0x3C;
 
// ---- Register values (7.4) ----
constexpr uint16_t IMU_VAL_FILT_68HZ = 0x0000; // LPF0 = 68 Hz on X, Y, Z
constexpr uint16_t IMU_VAL_RATE_CTRL = 0x1324; // DYN1 +-300 deg/s (1600 LSB) + decimation /16 = 737.5 Hz
constexpr uint16_t IMU_VAL_ACC12_CTRL = 0x1324; // DYN1 +-80 m/s^2 (3200 LSB) + decimation /16 = 737.5 Hz
constexpr uint16_t IMU_VAL_ACC3_CTRL = 0x0000;
constexpr uint16_t IMU_VAL_USER_IF_DEFAULT = 0x200C; // power-on value: TDEL 2.5 ms, MISO/DRY slew control on
constexpr uint16_t IMU_VAL_USER_IF = 0x202C; // default + DRY_DRV_EN (bit 5). NOT 0x0020: that clears the defaults
constexpr uint16_t IMU_VAL_MODE_EN = 0b01; // EN_SENSOR
constexpr uint16_t IMU_VAL_MODE_EN_EOI = 0b11; // EN_SENSOR + EOI: locks settings
constexpr uint16_t IMU_VAL_SOFT_RESET = 0b1010;
 
// ---- Expected values ----
constexpr int IMU_STATUS_REG_COUNT = 10;
constexpr uint16_t IMU_STATUS_HEALTHY = 0xFFFF; // every flag 1 = OK (7.3)
constexpr uint16_t IMU_ASIC_ID_EXPECTED = 0x0021; // NOT in the datasheet (from PX4): log yours and replace
constexpr uint16_t IMU_COMP_ID_EXPECTED = 0x0023; // same
 
// ---- Scaling (3.4, 3.5, 3.8, 7.2) ----
// Sensitivities are only valid for DYN1, i.e. with IMU_VAL_RATE_CTRL / IMU_VAL_ACC12_CTRL above.
constexpr float IMU_GYRO_LSB_PER_DPS = 1600.0f;
constexpr float IMU_ACCEL_LSB_PER_MS2 = 3200.0f;
constexpr int IMU_TEMP_SHIFT = 4; // TEMP is 16 bits at the TOP of the 20-bit field: >> 4 (= / 16)
constexpr float IMU_TEMP_LSB_PER_C = 100.0f;
constexpr float IMU_DEG_TO_RAD = 3.14159265f / 180.0f;
constexpr float IMU_AXIS_SIGN[3] = {1.0f, -1.0f, -1.0f}; // chip axes -> FRD; depends on mounting, check by hand
 
// ---- Timing (Figure 8, 3.12, 7.4.5) ----
constexpr uint32_t IMU_EXTRESN_PULSE_MS = 2;
constexpr uint32_t IMU_POWER_SETTLE_MS = 1; // supplies within spec
constexpr uint32_t IMU_NVM_LOAD_MS = 32; // after power-on or any reset, before talking SPI
constexpr uint32_t IMU_SOFT_RESET_QUIET_MS = 2; // no SPI at all after a soft reset (then the 32 ms above)
constexpr uint32_t IMU_EN_SENSOR_WAIT_MS = 250; // datasheet flowchart says 215; 250 is the spec max
constexpr uint32_t IMU_EOI_SETTLE_MS = 3;
constexpr uint32_t IMU_CS_GAP_AFTER_WRITE_NS = 750; // CS high between frames; ESP32 polling meets this
constexpr uint32_t IMU_CS_GAP_AFTER_READ_NS = 450;
constexpr int64_t IMU_SAMPLE_PERIOD_US = 5000; // 200 Hz
constexpr uint32_t IMU_WAKE_TIMEOUT_MS = 50; // task not woken this long = timer is broken
 
// ---- Policy ----
constexpr int IMU_BAD_IN_A_ROW_LIMIT = 10; // then run start_up() again
constexpr UBaseType_t IMU_QUEUE_LENGTH = 200; // 1 s of samples


uint8_t crc8 (uint64_t message) {
    uint8_t crc= 0b1111'1111;
    uint16_t polynomial =  0b00101111;
    uint64_t padded = message << 8;

    for (int i = 47; i >= 0; i--) {
        uint8_t incoming_bit = (padded >> (i)) & 0b0001; // pull till the newest bit then isolate it
        uint8_t fall_off_bit = crc & 0b1000'0000;
        crc = ((crc << 1) | incoming_bit);
        if (fall_off_bit != 0) {
            crc ^= polynomial;
        }
    }
    return crc;
}

uint64_t make_frame(uint8_t register_address, bool write, uint16_t value) {
    uint64_t frame = 0x0;

    frame = frame | (static_cast<uint64_t>(register_address) << IMU_FRAME_ADDR_SHIFT);
    if (write) {
        frame = frame | (static_cast<uint64_t>(write) << IMU_FRAME_RW_SHIFT); // read or write
        frame = frame | (static_cast<uint64_t>(value) << IMU_FRAME_DATA_SHIFT); // data
    }
    frame = frame | (static_cast<uint64_t>(0b1) << IMU_FRAME_FT_SHIFT); // frame type
    frame = frame | static_cast<uint64_t>(crc8(frame >> IMU_FRAME_CRC_BITS)); // check_sum the 40 bit
    return frame;
}


bool reply_check(uint64_t reply, bool &saturated){
    uint8_t crc_received = static_cast<uint8_t>(reply & IMU_MASK_CRC);
    uint8_t crc_expected = crc8(reply >> IMU_FRAME_CRC_BITS);

    if (crc_received != crc_expected) {
        return false;   // crc fail
    }

    uint64_t error_bits = (reply >> IMU_REPLY_CE_SHIFT) & IMU_MASK_ERRORS;
    if (error_bits != 0) {
        return false;
    }

    uint8_t status = static_cast<uint8_t>((reply >> IMU_REPLY_STATUS_SHIFT) & IMU_MASK_STATUS);
    if (status == IMU_STATUS_OK) {
        return true;
    } else if (status == IMU_STATUS_SATURATED) {
        saturated = true; // value is real but clipped keep it but flag it
        return true;
    } else {
        return false; // IMU_STATUS_ERROR or IMU_STATUS_INIT
    }
}

int32_t data_20bit_extract (uint64_t reply) {
    uint32_t data_raw = static_cast<uint32_t>((reply >> IMU_FRAME_CRC_BITS) & IMU_MASK_DATA20); // pull out data 20 bit
    // check if negative
    if ((data_raw & IMU_DATA20_SIGN_BIT) != 0) {
        data_raw |= IMU_DATA20_SIGN_BIT; // flood bit 20 to bit 32 with 1 (now negative)
    }
    return static_cast<int32_t>(data_raw);
}




}












