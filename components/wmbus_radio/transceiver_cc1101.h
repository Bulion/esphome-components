#pragma once

#include "cc1101_driver.h"
#include "transceiver.h"
#include <memory>
#include <vector>

namespace esphome {
namespace wmbus_radio {

/// @brief CC1101 MARC state machine states
enum class CC1101State : uint8_t {
  SLEEP = 0x00,
  IDLE = 0x01,
  RX = 0x0D,
  RX_OVERFLOW = 0x11,
  TX = 0x13,
  TX_UNDERFLOW = 0x16,
};

/// @brief RX loop state machine for wM-Bus frame reception
enum class RxLoopState : uint8_t {
  INIT_RX = 0,       ///< Initialize receiver
  WAIT_FOR_SYNC = 1, ///< Waiting for sync word detection
  WAIT_FOR_DATA = 2, ///< Waiting for enough data in FIFO
  READ_DATA = 3,     ///< Reading data from FIFO
};

/// @brief CC1101 packet length mode
enum class LengthMode : uint8_t {
  INFINITE = 0, ///< Infinite packet length (variable)
  FIXED = 1,    ///< Fixed packet length
};

/// @brief wM-Bus frame mode detected
enum class WMBusMode : char {
  MODE_T = 'T', ///< Mode T (T1): 100 kbps, 2-FSK with 3-of-6 encoding
  MODE_C = 'C', ///< Mode C (C1): 100 kbps, 2-FSK
  UNKNOWN = '?'
};

/// @brief wM-Bus block type
enum class WMBusBlock : char {
  BLOCK_A = 'A',
  BLOCK_B = 'B',
  UNKNOWN = '?'
};

/**
 * @brief CC1101 transceiver for wM-Bus reception
 *
 * Implements RadioTransceiver interface for CC1101 chip.
 * Handles wM-Bus Mode T and Mode C frame reception with
 * state machine-based FIFO reading.
 *
 * Uses composition pattern: owns CC1101Driver for hardware access.
 */
class CC1101 : public RadioTransceiver {
public:
  void setup() override;
  void restart_rx() override;
  int8_t get_rssi() override;
  const char *get_name() override { return "CC1101"; }

  /**
   * @brief Set GDO0 pin (FIFO threshold indicator)
   * @param pin GPIO pin
   */
  void set_gdo0_pin(InternalGPIOPin *pin) { this->gdo0_pin_ = pin; }

  /**
   * @brief Set GDO2 pin (sync word detected indicator)
   * @param pin GPIO pin
   */
  void set_gdo2_pin(InternalGPIOPin *pin) { this->gdo2_pin_ = pin; }

  /**
   * @brief Set carrier frequency
   * @param freq_mhz Frequency in MHz (default: 868.95)
   */
  void set_frequency(float freq_mhz) { this->frequency_mhz_ = freq_mhz; }

protected:
  optional<uint8_t> read() override;

private:
  /// @brief Initialize RX state machine
  void init_rx_();

  /// @brief Check for sync word detection (GDO2 high)
  bool wait_for_sync_();

  /// @brief Wait for FIFO threshold and read header
  bool wait_for_data_();

  /// @brief Read remaining data from FIFO
  bool read_data_();

  /// @brief Switch CC1101 to IDLE state
  void set_idle_();

  /// @brief Check if RX FIFO has overflowed
  bool check_rx_overflow_();

  std::unique_ptr<CC1101Driver> driver_;
  InternalGPIOPin *gdo0_pin_{nullptr};
  InternalGPIOPin *gdo2_pin_{nullptr};

  float frequency_mhz_{868.95f}; ///< Carrier frequency in MHz

  // RX state machine variables
  RxLoopState rx_state_{RxLoopState::INIT_RX};
  std::vector<uint8_t> rx_buffer_;
  size_t bytes_received_{0};
  size_t expected_length_{0};
  uint8_t length_field_{0};
  LengthMode length_mode_{LengthMode::INFINITE};
  WMBusMode wmbus_mode_{WMBusMode::UNKNOWN};
  WMBusBlock wmbus_block_{WMBusBlock::UNKNOWN};

  uint32_t sync_time_{0};      ///< Time when sync was detected
  uint32_t max_wait_time_{50}; ///< Max wait for data (ms)

  // wM-Bus constants
  static constexpr uint8_t WMBUS_MODE_C_PREAMBLE = 0x54;
  static constexpr uint8_t WMBUS_BLOCK_A_PREAMBLE = 0xCD;
  static constexpr uint8_t WMBUS_BLOCK_B_PREAMBLE = 0x3D;
  static constexpr uint8_t RX_FIFO_THRESHOLD = 10;
  static constexpr size_t MAX_FIXED_LENGTH = 256;
  static constexpr size_t MAX_FRAME_SIZE = 512;
};

} // namespace wmbus_radio
} // namespace esphome
