#pragma once

#include "cc1101_driver.h"
#include <array>
#include <utility>

namespace esphome {
namespace wmbus_radio {

/**
 * @brief CC1101 RF register configuration for wM-Bus reception
 *
 * Based on Texas Instruments application note SWRA234A:
 * "Wireless MBUS Implementation on CC1101"
 *
 * Configuration targets:
 * - Frequency: 868.95 MHz (EU wM-Bus band)
 * - Mode T (T1): 100 kbps, 2-FSK, Manchester encoding
 * - Mode C (C1): 100 kbps, 2-FSK
 * - Deviation: ±50 kHz
 * - Receiver bandwidth: 203 kHz
 *
 * Register values optimized for:
 * - Infinite packet length mode (for variable-length wM-Bus packets)
 * - GDO0: Asserts when RX FIFO threshold reached
 * - GDO2: Asserts when sync word detected
 */

/// @brief RF configuration as (register, value) pairs
constexpr std::array<std::pair<CC1101Register, uint8_t>, 47>
    CC1101_WMBUS_RF_SETTINGS = {{
        // GDO2 output configuration: Asserts when sync word detected
        {CC1101Register::IOCFG2, 0x06},

        // GDO1 output configuration: High impedance (not used)
        {CC1101Register::IOCFG1, 0x2E},

        // GDO0 output configuration: Asserts when FIFO threshold reached
        {CC1101Register::IOCFG0, 0x00},

        // RX FIFO and TX FIFO thresholds: 33 bytes in FIFO
        {CC1101Register::FIFOTHR, 0x07},

        // Sync word, high byte: 0x54 (wM-Bus Mode C/T preamble)
        {CC1101Register::SYNC1, 0x54},

        // Sync word, low byte: 0x3D (wM-Bus Mode C/T preamble)
        {CC1101Register::SYNC0, 0x3D},

        // Packet length: 255 bytes (maximum, will use infinite mode)
        {CC1101Register::PKTLEN, 0xFF},

        // Packet automation control: No address check, no append status
        {CC1101Register::PKTCTRL1, 0x00},

        // Packet automation control: Normal mode, no CRC, infinite length
        {CC1101Register::PKTCTRL0, 0x00},

        // Device address: 0x00 (not used)
        {CC1101Register::ADDR, 0x00},

        // Channel number: 0
        {CC1101Register::CHANNR, 0x00},

        // Frequency synthesizer control: IF frequency
        {CC1101Register::FSCTRL1, 0x08},

        // Frequency synthesizer control: Frequency offset
        {CC1101Register::FSCTRL0, 0x00},

        // Frequency control word, high byte: 868.95 MHz
        // Formula: freq = (f_carrier / f_xosc) * 2^16
        // Where f_xosc = 26 MHz
        {CC1101Register::FREQ2, 0x21},

        // Frequency control word, middle byte
        {CC1101Register::FREQ1, 0x6B},

        // Frequency control word, low byte
        {CC1101Register::FREQ0, 0xD0},

        // Modem configuration: Receiver bandwidth ~203 kHz
        {CC1101Register::MDMCFG4, 0x5C},

        // Modem configuration: Data rate mantissa (~100 kbps)
        {CC1101Register::MDMCFG3, 0x04},

        // Modem configuration: 2-FSK modulation, 30/32 sync word bits
        {CC1101Register::MDMCFG2, 0x06},

        // Modem configuration: FEC disabled, 2 preamble bytes, channel spacing
        {CC1101Register::MDMCFG1, 0x22},

        // Modem configuration: Channel spacing mantissa
        {CC1101Register::MDMCFG0, 0xF8},

        // Modem deviation setting: ±50 kHz
        {CC1101Register::DEVIATN, 0x44},

        // Main Radio Control State Machine configuration
        {CC1101Register::MCSM2, 0x07},

        // Main Radio Control State Machine configuration:
        // CCA mode: Always, RX->IDLE transition: Stay in RX
        {CC1101Register::MCSM1, 0x00},

        // Main Radio Control State Machine configuration:
        // Calibration: From IDLE to RX/TX, PO_TIMEOUT: Approx. 149-155 μs
        {CC1101Register::MCSM0, 0x18},

        // Frequency Offset Compensation configuration
        {CC1101Register::FOCCFG, 0x2E},

        // Bit Synchronization configuration
        {CC1101Register::BSCFG, 0xBF},

        // AGC control: Target amplitude 33 dB
        {CC1101Register::AGCCTRL2, 0x43},

        // AGC control: AGC LNA priority, Relative carrier sense threshold disabled
        {CC1101Register::AGCCTRL1, 0x09},

        // AGC control: Medium hysteresis, Filter length 16 samples
        {CC1101Register::AGCCTRL0, 0xB5},

        // High byte Event0 timeout
        {CC1101Register::WOREVT1, 0x87},

        // Low byte Event0 timeout
        {CC1101Register::WOREVT0, 0x6B},

        // Wake On Radio control
        {CC1101Register::WORCTRL, 0xFB},

        // Front end RX configuration: LNA current
        {CC1101Register::FREND1, 0xB6},

        // Front end TX configuration: PA power setting
        {CC1101Register::FREND0, 0x10},

        // Frequency synthesizer calibration: VCO current calibration
        {CC1101Register::FSCAL3, 0xEA},

        // Frequency synthesizer calibration: VCO current calibration
        {CC1101Register::FSCAL2, 0x2A},

        // Frequency synthesizer calibration: Charge pump current
        {CC1101Register::FSCAL1, 0x00},

        // Frequency synthesizer calibration: VCO capacitor array
        {CC1101Register::FSCAL0, 0x1F},

        // RC oscillator configuration
        {CC1101Register::RCCTRL1, 0x41},

        // RC oscillator configuration
        {CC1101Register::RCCTRL0, 0x00},

        // Frequency synthesizer calibration control
        {CC1101Register::FSTEST, 0x59},

        // Production test
        {CC1101Register::PTEST, 0x7F},

        // AGC test
        {CC1101Register::AGCTEST, 0x3F},

        // Various test settings: VCO selection
        {CC1101Register::TEST2, 0x81},

        // Various test settings: Modulation format
        {CC1101Register::TEST1, 0x35},

        // Various test settings: Digital test output
        {CC1101Register::TEST0, 0x09},
    }};

/**
 * @brief Apply wM-Bus RF settings to CC1101
 * @param driver CC1101 low-level driver
 */
inline void apply_wmbus_rf_settings(CC1101Driver &driver) {
  for (const auto &[reg, value] : CC1101_WMBUS_RF_SETTINGS) {
    driver.write_register(reg, value);
  }
}

/**
 * @brief Set CC1101 carrier frequency
 * @param driver CC1101 low-level driver
 * @param freq_mhz Frequency in MHz
 *
 * Formula: FREQ = (f_carrier / f_xosc) * 2^16
 * Where f_xosc = 26 MHz (CC1101 crystal frequency)
 */
inline void set_carrier_frequency(CC1101Driver &driver, float freq_mhz) {
  uint32_t freq_reg = static_cast<uint32_t>(freq_mhz * 65536.0f / 26.0f);
  uint8_t freq2 = (freq_reg >> 16) & 0xFF;
  uint8_t freq1 = (freq_reg >> 8) & 0xFF;
  uint8_t freq0 = freq_reg & 0xFF;

  driver.write_register(CC1101Register::FREQ2, freq2);
  driver.write_register(CC1101Register::FREQ1, freq1);
  driver.write_register(CC1101Register::FREQ0, freq0);
}

} // namespace wmbus_radio
} // namespace esphome
