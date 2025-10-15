#include "transceiver_cc1101.h"
#include "cc1101_rf_settings.h"
#include "decode3of6.h"
#include "esphome/core/log.h"

namespace esphome {
namespace wmbus_radio {

static const char *const TAG = "cc1101";

/// @brief Calculate expected packet size for Mode T with 3-of-6 encoding
static size_t mode_t_packet_size(uint8_t l_field) {
  // L-field is the number of data bytes (excluding CRC)
  // For Mode T: add CRC blocks (2 bytes per 16 data bytes)
  size_t data_bytes = l_field;
  size_t crc_bytes = (data_bytes + 15) / 16 * 2; // 2-byte CRC per 16-byte block
  return data_bytes + crc_bytes + 1;              // +1 for L-field itself
}

void CC1101::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CC1101...");

  // Initialize GPIO pins
  if (this->gdo0_pin_ != nullptr) {
    this->gdo0_pin_->setup();
    this->gdo0_pin_->pin_mode(gpio::FLAG_INPUT);
  }
  if (this->gdo2_pin_ != nullptr) {
    this->gdo2_pin_->setup();
    this->gdo2_pin_->pin_mode(gpio::FLAG_INPUT);
  }

  // Common transceiver setup (reset, SPI init)
  this->common_setup();

  // Create CC1101 driver with SPI device
  this->driver_ = std::make_unique<CC1101Driver>(this);

  // Reset CC1101
  this->driver_->send_strobe(CC1101Strobe::SRES);
  delay(10);

  // Apply wM-Bus RF settings
  apply_wmbus_rf_settings(*this->driver_);

  // Set frequency if not default
  if (this->frequency_mhz_ != 868.95f) {
    set_carrier_frequency(*this->driver_, this->frequency_mhz_);
  }

  // Calibrate frequency synthesizer
  this->driver_->send_strobe(CC1101Strobe::SCAL);
  delay(4);

  // Check chip version
  uint8_t version = this->driver_->read_status(CC1101Status::VERSION);
  if (version == 0 || version == 0xFF) {
    ESP_LOGE(TAG, "CC1101 not detected! Version: 0x%02X", version);
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "CC1101 version: 0x%02X", version);
  ESP_LOGCONFIG(TAG, "Frequency: %.2f MHz", this->frequency_mhz_);

  // Start receiver
  this->restart_rx();

  ESP_LOGCONFIG(TAG, "CC1101 setup complete");
}

void CC1101::restart_rx() {
  this->set_idle_();
  this->init_rx_();
}

int8_t CC1101::get_rssi() {
  // Read RSSI register
  uint8_t rssi_raw = this->driver_->read_status(CC1101Status::RSSI);

  // Convert to dBm according to CC1101 datasheet:
  // RSSI_dBm = (RSSI_dec / 2) - RSSI_offset
  // where RSSI_offset is typically 74 dBm for 868 MHz
  int16_t rssi_dbm;
  if (rssi_raw >= 128) {
    rssi_dbm = ((rssi_raw - 256) / 2) - 74;
  } else {
    rssi_dbm = (rssi_raw / 2) - 74;
  }

  return static_cast<int8_t>(rssi_dbm);
}

optional<uint8_t> CC1101::read() {
  // State machine for wM-Bus frame reception
  switch (this->rx_state_) {
  case RxLoopState::INIT_RX:
    this->init_rx_();
    return {};

  case RxLoopState::WAIT_FOR_SYNC:
    if (this->wait_for_sync_()) {
      this->rx_state_ = RxLoopState::WAIT_FOR_DATA;
      this->sync_time_ = millis();
    }
    return {};

  case RxLoopState::WAIT_FOR_DATA:
    if (millis() - this->sync_time_ > this->max_wait_time_) {
      ESP_LOGV(TAG, "Timeout waiting for data");
      this->rx_state_ = RxLoopState::INIT_RX;
      return {};
    }
    if (this->wait_for_data_()) {
      this->rx_state_ = RxLoopState::READ_DATA;
    }
    return {};

  case RxLoopState::READ_DATA:
    if (this->read_data_()) {
      // Frame complete!
      ESP_LOGD(TAG, "Frame received: %zu bytes, mode: %c, block: %c",
               this->bytes_received_, static_cast<char>(this->wmbus_mode_),
               static_cast<char>(this->wmbus_block_));

      // Decode 3-of-6 if Mode T
      if (this->wmbus_mode_ == WMBusMode::MODE_T) {
        auto decoded = decode3of6(this->rx_buffer_);
        if (!decoded.has_value()) {
          ESP_LOGW(TAG, "3-of-6 decode failed");
          this->rx_state_ = RxLoopState::INIT_RX;
          return {};
        }
        this->rx_buffer_ = std::move(decoded.value());
        ESP_LOGV(TAG, "Decoded to %zu bytes", this->rx_buffer_.size());
      }

      // Return first byte to indicate frame ready
      this->rx_state_ = RxLoopState::INIT_RX;
      if (!this->rx_buffer_.empty()) {
        return this->rx_buffer_[0];
      }
    }
    return {};

  default:
    this->rx_state_ = RxLoopState::INIT_RX;
    return {};
  }
}

void CC1101::init_rx_() {
  // Flush FIFOs and set to RX mode
  this->set_idle_();
  this->driver_->send_strobe(CC1101Strobe::SFTX);
  this->driver_->send_strobe(CC1101Strobe::SFRX);

  // Set FIFO threshold for initial reception
  this->driver_->write_register(CC1101Register::FIFOTHR, 0x00);

  // Set to infinite packet length mode
  this->driver_->write_register(CC1101Register::PKTCTRL0, 0x02);

  // Clear state
  this->rx_buffer_.clear();
  this->bytes_received_ = 0;
  this->expected_length_ = 0;
  this->length_field_ = 0;
  this->length_mode_ = LengthMode::INFINITE;
  this->wmbus_mode_ = WMBusMode::UNKNOWN;
  this->wmbus_block_ = WMBusBlock::UNKNOWN;

  // Start RX
  this->driver_->send_strobe(CC1101Strobe::SRX);

  // Wait for RX state
  uint8_t marc_state;
  for (int i = 0; i < 10; i++) {
    marc_state = this->driver_->read_status(CC1101Status::MARCSTATE);
    if (marc_state == static_cast<uint8_t>(CC1101State::RX))
      break;
    delay(1);
  }

  this->rx_state_ = RxLoopState::WAIT_FOR_SYNC;
}

bool CC1101::wait_for_sync_() {
  // Check GDO2 pin - asserts when sync word detected
  if (this->gdo2_pin_ != nullptr) {
    return this->gdo2_pin_->digital_read();
  }
  return false;
}

bool CC1101::wait_for_data_() {
  // Check GDO0 pin - asserts when FIFO threshold reached
  if (this->gdo0_pin_ == nullptr || !this->gdo0_pin_->digital_read()) {
    return false;
  }

  // Check for overflow
  if (this->check_rx_overflow_()) {
    ESP_LOGW(TAG, "RX FIFO overflow");
    this->rx_state_ = RxLoopState::INIT_RX;
    return false;
  }

  // Read first 3 bytes to determine frame type
  uint8_t header[3];
  this->driver_->read_rx_fifo(header, 3);

  // Detect Mode C or Mode T
  if (header[0] == WMBUS_MODE_C_PREAMBLE) {
    // Mode C
    this->wmbus_mode_ = WMBusMode::MODE_C;

    if (header[1] == WMBUS_BLOCK_A_PREAMBLE) {
      this->wmbus_block_ = WMBusBlock::BLOCK_A;
      this->length_field_ = header[2];
      this->expected_length_ = 2 + mode_t_packet_size(this->length_field_);
    } else if (header[1] == WMBUS_BLOCK_B_PREAMBLE) {
      this->wmbus_block_ = WMBusBlock::BLOCK_B;
      this->length_field_ = header[2];
      this->expected_length_ = 2 + 1 + this->length_field_;
    } else {
      ESP_LOGV(TAG, "Unknown Mode C block type: 0x%02X", header[1]);
      return false;
    }

    // Store L-field only (skip C preamble)
    this->rx_buffer_.push_back(this->length_field_);
    this->bytes_received_ = 1;

  } else {
    // Try Mode T (3-of-6 encoded)
    uint8_t decoded[2];
    // Temporarily store in vector for decode3of6
    std::vector<uint8_t> temp_header(header, header + 3);
    auto decoded_opt = decode3of6(temp_header);

    if (decoded_opt.has_value() && decoded_opt->size() >= 1) {
      this->wmbus_mode_ = WMBusMode::MODE_T;
      this->wmbus_block_ = WMBusBlock::BLOCK_A;
      this->length_field_ = (*decoded_opt)[0];
      this->expected_length_ = encoded_size(mode_t_packet_size(this->length_field_));

      // Store encoded bytes
      this->rx_buffer_.insert(this->rx_buffer_.end(), header, header + 3);
      this->bytes_received_ = 3;
    } else {
      ESP_LOGV(TAG, "Unknown frame type, header: %02X %02X %02X", header[0],
               header[1], header[2]);
      return false;
    }
  }

  ESP_LOGV(TAG, "Frame detected: mode=%c, block=%c, L=0x%02X, expected=%zu",
           static_cast<char>(this->wmbus_mode_),
           static_cast<char>(this->wmbus_block_), this->length_field_,
           this->expected_length_);

  // Switch to fixed length mode if possible
  if (this->expected_length_ < MAX_FIXED_LENGTH) {
    this->driver_->write_register(CC1101Register::PKTLEN,
                                   static_cast<uint8_t>(this->expected_length_));
    this->driver_->write_register(CC1101Register::PKTCTRL0, 0x00); // Fixed
    this->length_mode_ = LengthMode::FIXED;
  }

  // Set FIFO threshold for remaining data
  this->driver_->write_register(CC1101Register::FIFOTHR, RX_FIFO_THRESHOLD);

  return true;
}

bool CC1101::read_data_() {
  // Check GDO0 - asserts when FIFO threshold reached
  if (this->gdo0_pin_ == nullptr || !this->gdo0_pin_->digital_read()) {
    return false;
  }

  // Check for overflow
  if (this->check_rx_overflow_()) {
    ESP_LOGW(TAG, "RX FIFO overflow during read");
    return false;
  }

  // Read available bytes (leave at least 1 byte in FIFO per errata)
  uint8_t bytes_in_fifo = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
  if (bytes_in_fifo > 1) {
    size_t bytes_to_read = bytes_in_fifo - 1;
    size_t bytes_remaining = this->expected_length_ - this->bytes_received_;
    bytes_to_read = std::min(bytes_to_read, bytes_remaining);

    if (this->rx_buffer_.size() + bytes_to_read > MAX_FRAME_SIZE) {
      ESP_LOGW(TAG, "Frame too large");
      return false;
    }

    size_t old_size = this->rx_buffer_.size();
    this->rx_buffer_.resize(old_size + bytes_to_read);
    this->driver_->read_rx_fifo(this->rx_buffer_.data() + old_size,
                                 bytes_to_read);
    this->bytes_received_ += bytes_to_read;

    ESP_LOGVV(TAG, "Read %zu bytes, total: %zu/%zu", bytes_to_read,
              this->bytes_received_, this->expected_length_);
  }

  // Check if complete
  if (this->bytes_received_ >= this->expected_length_) {
    // Read any remaining bytes
    uint8_t bytes_in_fifo = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
    if (bytes_in_fifo > 0) {
      size_t old_size = this->rx_buffer_.size();
      this->rx_buffer_.resize(old_size + bytes_in_fifo);
      this->driver_->read_rx_fifo(this->rx_buffer_.data() + old_size,
                                   bytes_in_fifo);
    }
    return true; // Frame complete
  }

  return false;
}

void CC1101::set_idle_() {
  this->driver_->send_strobe(CC1101Strobe::SIDLE);
  uint8_t marc_state;
  for (int i = 0; i < 10; i++) {
    marc_state = this->driver_->read_status(CC1101Status::MARCSTATE);
    if (marc_state == static_cast<uint8_t>(CC1101State::IDLE))
      break;
    delay(1);
  }
}

bool CC1101::check_rx_overflow_() {
  uint8_t rxbytes = this->driver_->read_status(CC1101Status::RXBYTES);
  return (rxbytes & 0x80) != 0; // Bit 7 set indicates overflow
}

} // namespace wmbus_radio
} // namespace esphome
