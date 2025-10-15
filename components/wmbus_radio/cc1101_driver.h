#pragma once

#include "esphome/components/spi/spi.h"
#include <cstdint>

namespace esphome {
namespace wmbus_radio {

/// @brief CC1101 register addresses
enum class CC1101Register : uint8_t {
  IOCFG2 = 0x00,
  IOCFG1 = 0x01,
  IOCFG0 = 0x02,
  FIFOTHR = 0x03,
  SYNC1 = 0x04,
  SYNC0 = 0x05,
  PKTLEN = 0x06,
  PKTCTRL1 = 0x07,
  PKTCTRL0 = 0x08,
  ADDR = 0x09,
  CHANNR = 0x0A,
  FSCTRL1 = 0x0B,
  FSCTRL0 = 0x0C,
  FREQ2 = 0x0D,
  FREQ1 = 0x0E,
  FREQ0 = 0x0F,
  MDMCFG4 = 0x10,
  MDMCFG3 = 0x11,
  MDMCFG2 = 0x12,
  MDMCFG1 = 0x13,
  MDMCFG0 = 0x14,
  DEVIATN = 0x15,
  MCSM2 = 0x16,
  MCSM1 = 0x17,
  MCSM0 = 0x18,
  FOCCFG = 0x19,
  BSCFG = 0x1A,
  AGCCTRL2 = 0x1B,
  AGCCTRL1 = 0x1C,
  AGCCTRL0 = 0x1D,
  WOREVT1 = 0x1E,
  WOREVT0 = 0x1F,
  WORCTRL = 0x20,
  FREND1 = 0x21,
  FREND0 = 0x22,
  FSCAL3 = 0x23,
  FSCAL2 = 0x24,
  FSCAL1 = 0x25,
  FSCAL0 = 0x26,
  RCCTRL1 = 0x27,
  RCCTRL0 = 0x28,
  FSTEST = 0x29,
  PTEST = 0x2A,
  AGCTEST = 0x2B,
  TEST2 = 0x2C,
  TEST1 = 0x2D,
  TEST0 = 0x2E,
};

/// @brief CC1101 status registers
enum class CC1101Status : uint8_t {
  PARTNUM = 0x30,
  VERSION = 0x31,
  FREQEST = 0x32,
  LQI = 0x33,
  RSSI = 0x34,
  MARCSTATE = 0x35,
  WORTIME1 = 0x36,
  WORTIME0 = 0x37,
  PKTSTATUS = 0x38,
  VCO_VC_DAC = 0x39,
  TXBYTES = 0x3A,
  RXBYTES = 0x3B,
  RCCTRL1_STATUS = 0x3C,
  RCCTRL0_STATUS = 0x3D,
};

/// @brief CC1101 command strobes
enum class CC1101Strobe : uint8_t {
  SRES = 0x30,    // Reset chip
  SFSTXON = 0x31, // Enable and calibrate frequency synthesizer
  SXOFF = 0x32,   // Turn off crystal oscillator
  SCAL = 0x33,    // Calibrate frequency synthesizer
  SRX = 0x34,     // Enable RX
  STX = 0x35,     // Enable TX
  SIDLE = 0x36,   // Exit RX/TX, turn off frequency synthesizer
  SWOR = 0x38,    // Start automatic RX polling sequence
  SPWD = 0x39,    // Enter power down mode
  SFRX = 0x3A,    // Flush the RX FIFO buffer
  SFTX = 0x3B,    // Flush the TX FIFO buffer
  SWORRST = 0x3C, // Reset real time clock
  SNOP = 0x3D,    // No operation
};

/// @brief CC1101 FIFO access
constexpr uint8_t CC1101_FIFO = 0x3F;

/// @brief SPI access modes
constexpr uint8_t CC1101_READ_SINGLE = 0x80;
constexpr uint8_t CC1101_READ_BURST = 0xC0;
constexpr uint8_t CC1101_WRITE_BURST = 0x40;

/**
 * @brief Low-level CC1101 SPI communication driver
 *
 * Provides register-level access to CC1101 transceiver.
 * Single responsibility: SPI communication only.
 * No state management or protocol logic.
 *
 * Uses ESPHome SPI infrastructure for hardware abstraction.
 */
class CC1101Driver {
public:
  /**
   * @brief Initialize the CC1101 driver with SPI device
   * @param spi_device SPI device handle from ESPHome
   */
  explicit CC1101Driver(spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                        spi::CLOCK_POLARITY_LOW,
                                        spi::CLOCK_PHASE_LEADING,
                                        spi::DATA_RATE_2MHZ> *spi_device)
      : spi_(spi_device) {}

  /**
   * @brief Read a single CC1101 register
   * @param reg Register address
   * @return Register value
   */
  uint8_t read_register(CC1101Register reg);

  /**
   * @brief Write a single CC1101 register
   * @param reg Register address
   * @param value Value to write
   */
  void write_register(CC1101Register reg, uint8_t value);

  /**
   * @brief Read a CC1101 status register
   * @param status Status register address
   * @return Status register value
   */
  uint8_t read_status(CC1101Status status);

  /**
   * @brief Read multiple bytes from CC1101 (burst read)
   * @param reg Starting register address or FIFO
   * @param buffer Destination buffer
   * @param length Number of bytes to read
   */
  void read_burst(CC1101Register reg, uint8_t *buffer, size_t length);

  /**
   * @brief Write multiple bytes to CC1101 (burst write)
   * @param reg Starting register address or FIFO
   * @param buffer Source buffer
   * @param length Number of bytes to write
   */
  void write_burst(CC1101Register reg, const uint8_t *buffer, size_t length);

  /**
   * @brief Send a command strobe to CC1101
   * @param strobe Command strobe
   * @return Chip status byte
   */
  uint8_t send_strobe(CC1101Strobe strobe);

  /**
   * @brief Read RX FIFO
   * @param buffer Destination buffer
   * @param length Number of bytes to read
   */
  void read_rx_fifo(uint8_t *buffer, size_t length);

  /**
   * @brief Write TX FIFO
   * @param buffer Source buffer
   * @param length Number of bytes to write
   */
  void write_tx_fifo(const uint8_t *buffer, size_t length);

private:
  spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                 spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> *spi_;
};

} // namespace wmbus_radio
} // namespace esphome
