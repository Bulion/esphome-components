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
  ESP_LOGD(TAG, "Sending software reset (SRES strobe)...");
  this->driver_->send_strobe(CC1101Strobe::SRES);
  delay(10);

  // Check chip version and part number to verify SPI communication
  uint8_t partnum = this->driver_->read_status(CC1101Status::PARTNUM);
  uint8_t version = this->driver_->read_status(CC1101Status::VERSION);

  ESP_LOGD(TAG, "CC1101 PARTNUM: 0x%02X (expected: 0x00)", partnum);
  ESP_LOGD(TAG, "CC1101 VERSION: 0x%02X (expected: 0x04 or 0x14)", version);

  if (version == 0 || version == 0xFF) {
    ESP_LOGE(TAG, "CC1101 not detected! SPI communication failed. Check wiring:");
    ESP_LOGE(TAG, "  - CS pin: connected and correct?");
    ESP_LOGE(TAG, "  - MOSI/MISO/SCK: connected and correct?");
    ESP_LOGE(TAG, "  - VCC: 3.3V supplied?");
    ESP_LOGE(TAG, "  - GND: connected?");
    this->mark_failed();
    return;
  }

  if (partnum != 0x00) {
    ESP_LOGW(TAG, "Unexpected PARTNUM 0x%02X (expected 0x00). Chip may not be CC1101.", partnum);
  }

  ESP_LOGCONFIG(TAG, "CC1101 detected - PARTNUM: 0x%02X, VERSION: 0x%02X", partnum, version);

  // Apply wM-Bus RF settings
  ESP_LOGD(TAG, "Applying wM-Bus RF settings (%zu registers)...", CC1101_WMBUS_RF_SETTINGS.size());
  apply_wmbus_rf_settings(*this->driver_);

  // Verify a few critical registers were written correctly
  uint8_t iocfg2 = this->driver_->read_register(CC1101Register::IOCFG2);
  uint8_t iocfg0 = this->driver_->read_register(CC1101Register::IOCFG0);
  uint8_t sync1 = this->driver_->read_register(CC1101Register::SYNC1);
  uint8_t sync0 = this->driver_->read_register(CC1101Register::SYNC0);

  ESP_LOGD(TAG, "Register verification:");
  ESP_LOGD(TAG, "  IOCFG2 (GDO2 config): 0x%02X (expected: 0x06)", iocfg2);
  ESP_LOGD(TAG, "  IOCFG0 (GDO0 config): 0x%02X (expected: 0x00)", iocfg0);
  ESP_LOGD(TAG, "  SYNC1: 0x%02X (expected: 0x54)", sync1);
  ESP_LOGD(TAG, "  SYNC0: 0x%02X (expected: 0x3D)", sync0);

  bool registers_ok = (iocfg2 == 0x06) && (iocfg0 == 0x00) &&
                      (sync1 == 0x54) && (sync0 == 0x3D);

  if (!registers_ok) {
    ESP_LOGW(TAG, "Register verification failed! SPI communication may be unreliable.");
  } else {
    ESP_LOGD(TAG, "Register verification passed - RF settings applied successfully");
  }

  // Set frequency if not default
  if (this->frequency_mhz_ != 868.95f) {
    ESP_LOGD(TAG, "Setting custom frequency: %.2f MHz", this->frequency_mhz_);
    set_carrier_frequency(*this->driver_, this->frequency_mhz_);

    // Read back frequency registers to verify
    uint8_t freq2 = this->driver_->read_register(CC1101Register::FREQ2);
    uint8_t freq1 = this->driver_->read_register(CC1101Register::FREQ1);
    uint8_t freq0 = this->driver_->read_register(CC1101Register::FREQ0);
    uint32_t freq_reg = (static_cast<uint32_t>(freq2) << 16) |
                        (static_cast<uint32_t>(freq1) << 8) |
                        freq0;
    float actual_freq = (freq_reg * 26.0f) / 65536.0f;
    ESP_LOGD(TAG, "Frequency registers: 0x%02X%02X%02X (%.2f MHz)", freq2, freq1, freq0, actual_freq);
  }

  // Calibrate frequency synthesizer
  ESP_LOGD(TAG, "Calibrating frequency synthesizer (SCAL strobe)...");
  this->driver_->send_strobe(CC1101Strobe::SCAL);
  delay(4);

  // Check calibration result
  uint8_t marcstate = this->driver_->read_status(CC1101Status::MARCSTATE);
  ESP_LOGD(TAG, "MARCSTATE after calibration: 0x%02X (IDLE=0x01)", marcstate);

  ESP_LOGCONFIG(TAG, "CC1101 initialized successfully");
  ESP_LOGCONFIG(TAG, "  Chip version: 0x%02X", version);
  ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz", this->frequency_mhz_);

  // Test GDO pin states before starting RX
  bool gdo0_initial = (this->gdo0_pin_ != nullptr) ? this->gdo0_pin_->digital_read() : false;
  bool gdo2_initial = (this->gdo2_pin_ != nullptr) ? this->gdo2_pin_->digital_read() : false;
  ESP_LOGD(TAG, "GDO pin initial states: GDO0=%d, GDO2=%d", gdo0_initial, gdo2_initial);

  // Start receiver
  this->restart_rx();

  // Check GDO pin states after entering RX
  delay(5);
  bool gdo0_rx = (this->gdo0_pin_ != nullptr) ? this->gdo0_pin_->digital_read() : false;
  bool gdo2_rx = (this->gdo2_pin_ != nullptr) ? this->gdo2_pin_->digital_read() : false;
  ESP_LOGD(TAG, "GDO pin states in RX mode: GDO0=%d, GDO2=%d", gdo0_rx, gdo2_rx);

  if (gdo0_initial == gdo0_rx && gdo2_initial == gdo2_rx) {
    ESP_LOGW(TAG, "GDO pins did not change state - check pin connections!");
  }

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
  // Periodic diagnostics every 10 seconds
  static uint32_t last_diag_time = 0;
  uint32_t now = millis();
  if (now - last_diag_time > 10000) {
    last_diag_time = now;
    uint8_t marcstate = this->driver_->read_status(CC1101Status::MARCSTATE);
    uint8_t rxbytes = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
    bool gdo2_state = (this->gdo2_pin_ != nullptr) ? this->gdo2_pin_->digital_read() : false;
    bool gdo0_state = (this->gdo0_pin_ != nullptr) ? this->gdo0_pin_->digital_read() : false;

    ESP_LOGD(TAG, "Status: MARCSTATE=0x%02X, RXBYTES=%d, GDO2=%d, GDO0=%d, RX_STATE=%d",
             marcstate, rxbytes, gdo2_state, gdo0_state, static_cast<int>(this->rx_state_));

    if (marcstate != static_cast<uint8_t>(CC1101State::RX)) {
      ESP_LOGW(TAG, "Not in RX mode (MARCSTATE=0x%02X), recovering...", marcstate);
      if (marcstate == static_cast<uint8_t>(CC1101State::RX_OVERFLOW)) {
        ESP_LOGW(TAG, "Forcing INIT_RX to recover from overflow");
        this->rx_state_ = RxLoopState::INIT_RX;
      }
    }
  }

  // State machine for wM-Bus frame reception
  switch (this->rx_state_) {
  case RxLoopState::INIT_RX:
    this->init_rx_();
    return {};

  case RxLoopState::WAIT_FOR_SYNC:
    // Preemptively check for FIFO overflow and drain if needed
    // This prevents overflow from accumulated noise/other transmitters
    {
      uint8_t rxbytes_status = this->driver_->read_status(CC1101Status::RXBYTES);
      if (rxbytes_status & 0x80) {
        // Overflow detected in WAIT_FOR_SYNC - flush and restart
        ESP_LOGW(TAG, "FIFO overflow while waiting for sync, flushing");
        this->rx_state_ = RxLoopState::INIT_RX;
        return {};
      }
      uint8_t rxbytes = rxbytes_status & 0x7F;
      // If FIFO has >32 bytes without sync, likely noise - flush it
      if (rxbytes > 32) {
        ESP_LOGD(TAG, "Flushing %d bytes of noise from FIFO", rxbytes);
        this->driver_->send_strobe(CC1101Strobe::SFRX);
      }
    }

    if (this->wait_for_sync_()) {
      // Read CC1101 status when sync detected
      uint8_t marcstate = this->driver_->read_status(CC1101Status::MARCSTATE);
      uint8_t rxbytes = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
      bool gdo0 = (this->gdo0_pin_ != nullptr) ? this->gdo0_pin_->digital_read() : false;

      ESP_LOGD(TAG, "Sync detected: GDO0=%d, MARCSTATE=0x%02X, RXBYTES=%d",
               gdo0, marcstate, rxbytes);

      this->rx_state_ = RxLoopState::WAIT_FOR_DATA;
      this->sync_time_ = millis();

      // Immediately try to process data instead of waiting for next poll cycle
      // Fall through to WAIT_FOR_DATA case
    } else {
      return {};
    }
    [[fallthrough]];

  case RxLoopState::WAIT_FOR_DATA:
    if (millis() - this->sync_time_ > this->max_wait_time_) {
      ESP_LOGW(TAG, "Timeout waiting for data after sync! Resetting RX.");
      this->rx_state_ = RxLoopState::INIT_RX;
      return {};
    }
    if (this->wait_for_data_()) {
      ESP_LOGD(TAG, "Header received, processing frame data");
      this->rx_state_ = RxLoopState::READ_DATA;
      // Immediately try to read remaining data
      // Fall through to READ_DATA case
    } else {
      return {};
    }
    [[fallthrough]];

  case RxLoopState::READ_DATA:
    // Keep reading in a tight loop until frame complete or no progress
    // This prevents 10ms delay between read() calls during which FIFO overflows
    while (true) {
      size_t bytes_before = this->bytes_received_;

      // Log state every 10 iterations to track progress
      static uint32_t loop_counter = 0;
      if (++loop_counter % 10 == 0) {
        uint8_t marcstate = this->driver_->read_status(CC1101Status::MARCSTATE);
        uint8_t rxbytes = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
        ESP_LOGVV(TAG, "READ_DATA loop: bytes=%zu/%zu, buffer=%zu, MARCSTATE=0x%02X, RXBYTES=%d",
                  this->bytes_received_, this->expected_length_, this->rx_buffer_.size(),
                  marcstate, rxbytes);
      }

      if (this->read_data_()) {
        // Frame complete!
        ESP_LOGD(TAG, "Frame received: %zu bytes (buffer: %zu), mode: %c, block: %c",
                 this->bytes_received_, this->rx_buffer_.size(),
                 static_cast<char>(this->wmbus_mode_),
                 static_cast<char>(this->wmbus_block_));

        // Log complete frame data BEFORE 3-of-6 decode (so we see encoded data even if decode fails)
        if (this->rx_buffer_.size() > 0) {
          std::string hex_str;
          hex_str.reserve(this->rx_buffer_.size() * 3);
          for (size_t i = 0; i < this->rx_buffer_.size(); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", this->rx_buffer_[i]);
            hex_str += buf;
          }
          if (this->wmbus_mode_ == WMBusMode::MODE_T) {
            ESP_LOGD(TAG, "Encoded frame data (%zu bytes): %s", this->rx_buffer_.size(), hex_str.c_str());
          } else {
            ESP_LOGD(TAG, "Frame data (%zu bytes): %s", this->rx_buffer_.size(), hex_str.c_str());
          }
        }

        // Decode 3-of-6 if Mode T
        if (this->wmbus_mode_ == WMBusMode::MODE_T) {
          auto decoded = decode3of6(this->rx_buffer_);
          if (!decoded.has_value()) {
            ESP_LOGW(TAG, "3-of-6 decode failed");
            this->rx_state_ = RxLoopState::INIT_RX;
            return {};
          }
          this->rx_buffer_ = std::move(decoded.value());
          ESP_LOGD(TAG, "3-of-6 decode successful, decoded to %zu bytes", this->rx_buffer_.size());

          // Log decoded frame data as HEX for analysis
          if (this->rx_buffer_.size() > 0) {
            std::string hex_str;
            hex_str.reserve(this->rx_buffer_.size() * 3);
            for (size_t i = 0; i < this->rx_buffer_.size(); i++) {
              char buf[4];
              snprintf(buf, sizeof(buf), "%02X ", this->rx_buffer_[i]);
              hex_str += buf;
            }
            ESP_LOGD(TAG, "Decoded frame data (%zu bytes): %s", this->rx_buffer_.size(), hex_str.c_str());
          }
        }

        // Return first byte to indicate frame ready
        this->rx_state_ = RxLoopState::INIT_RX;
        if (!this->rx_buffer_.empty()) {
          return this->rx_buffer_[0];
        }
        return {};
      }

      // If no progress made (no bytes read), exit loop and wait for next poll
      // This prevents busy-waiting when FIFO is empty
      if (this->bytes_received_ == bytes_before) {
        break;
      }

      // Made progress, continue reading without delay
      // (loop continues immediately)
    }
    return {};

  default:
    this->rx_state_ = RxLoopState::INIT_RX;
    return {};
  }
}

void CC1101::init_rx_() {
  ESP_LOGVV(TAG, "Initializing RX mode...");

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
  ESP_LOGVV(TAG, "Sending SRX strobe to enter RX mode...");
  this->driver_->send_strobe(CC1101Strobe::SRX);

  // Wait for RX state
  uint8_t marc_state;
  bool rx_entered = false;
  for (int i = 0; i < 10; i++) {
    marc_state = this->driver_->read_status(CC1101Status::MARCSTATE);
    if (marc_state == static_cast<uint8_t>(CC1101State::RX)) {
      rx_entered = true;
      break;
    }
    delay(1);
  }

  if (rx_entered) {
    ESP_LOGVV(TAG, "Entered RX mode successfully (MARCSTATE: 0x%02X)", marc_state);
  } else {
    ESP_LOGW(TAG, "Failed to enter RX mode! MARCSTATE: 0x%02X (expected: 0x0D)", marc_state);
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
  bool gdo0 = (this->gdo0_pin_ != nullptr) ? this->gdo0_pin_->digital_read() : false;
  bool gdo2 = (this->gdo2_pin_ != nullptr) ? this->gdo2_pin_->digital_read() : false;

  if (!gdo0) {
    // Log once per sync detection
    static uint32_t last_log_time = 0;
    if (millis() - last_log_time > 100) {
      last_log_time = millis();
      uint8_t rxbytes = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
      ESP_LOGD(TAG, "Waiting for GDO0 (FIFO data): GDO0=%d, GDO2=%d, RXBYTES=%d",
               gdo0, gdo2, rxbytes);
    }
    return false;
  }

  ESP_LOGI(TAG, "GDO0 asserted! Reading FIFO header...");

  // Check for overflow
  if (this->check_rx_overflow_()) {
    ESP_LOGW(TAG, "RX FIFO overflow");
    this->rx_state_ = RxLoopState::INIT_RX;
    return false;
  }

  // Read first 4 bytes to determine frame type
  // 4 bytes needed for Mode T detection: 4*8=32 bits = 5 complete 6-bit segments
  uint8_t header[4];
  this->driver_->read_rx_fifo(header, 4);
  ESP_LOGD(TAG, "Header bytes: %02X %02X %02X %02X", header[0], header[1], header[2], header[3]);

  // Detect Mode C or Mode T
  // Mode C with preamble: starts with 0x54
  if (header[0] == WMBUS_MODE_C_PREAMBLE) {
    // Mode C with preamble present
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
    // Not Mode C with preamble - could be Mode C without preamble or Mode T
    // CC1101 strips the sync word (54 3D), leaving either:
    //   Mode C: L C M M A A A A A A CI [data]
    //   Mode T: 3-of-6 encoded L-field + data

    // Mode C is the default for wireless M-Bus - assume Mode C unless we detect Mode T
    // Mode T detection: Check if first byte looks like 3-of-6 encoded data
    // Valid 3-of-6 codes have exactly 3 ones and 3 zeros in each 6-bit symbol

    // For now, assume Mode C by default (safer, more common)
    // TODO: Implement better Mode T detection based on bit patterns

    this->wmbus_mode_ = WMBusMode::MODE_C;
    this->wmbus_block_ = WMBusBlock::BLOCK_A;

    // First byte is L-field
    this->length_field_ = header[0];

    // Calculate expected size using same formula as Mode C with preamble
    this->expected_length_ = 2 + mode_t_packet_size(this->length_field_);

    // Prepend 54 CD, then store all 4 header bytes
    this->rx_buffer_.push_back(WMBUS_MODE_C_PREAMBLE);   // 0x54
    this->rx_buffer_.push_back(WMBUS_BLOCK_A_PREAMBLE);  // 0xCD
    this->rx_buffer_.insert(this->rx_buffer_.end(), header, header + 4);
    this->bytes_received_ = 6;  // 2 (preamble) + 4 (header)

    ESP_LOGD(TAG, "Mode C (no preamble) assumed: L=0x%02X, expected_length=%zu",
             this->length_field_, this->expected_length_);
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

  // CRITICAL: Drain any additional bytes already in FIFO to prevent overflow
  // When sync is detected, FIFO may already have 30-50 bytes. If we don't drain
  // them immediately, the FIFO will overflow before next read() call.
  uint8_t bytes_in_fifo = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
  if (bytes_in_fifo > 0) {
    ESP_LOGVV(TAG, "Draining %d bytes from FIFO after header to prevent overflow", bytes_in_fifo);

    // Calculate how many more bytes we need
    size_t bytes_remaining = this->expected_length_ - this->bytes_received_;
    size_t bytes_to_read = std::min(static_cast<size_t>(bytes_in_fifo), bytes_remaining);

    if (bytes_to_read > 0) {
      size_t old_size = this->rx_buffer_.size();
      this->rx_buffer_.resize(old_size + bytes_to_read);
      this->driver_->read_rx_fifo(this->rx_buffer_.data() + old_size, bytes_to_read);
      this->bytes_received_ += bytes_to_read;

      ESP_LOGVV(TAG, "Drained %zu bytes, total received: %zu/%zu",
               bytes_to_read, this->bytes_received_, this->expected_length_);
    }
  }

  return true;
}

bool CC1101::read_data_() {
  // Check for overflow first - if overflowed, abort and reinitialize
  if (this->check_rx_overflow_()) {
    ESP_LOGW(TAG, "RX FIFO overflow during read - aborting frame");
    this->rx_state_ = RxLoopState::INIT_RX;
    return false;
  }

  // Read available bytes regardless of GDO0 state
  // At 100kbps, we can't afford to wait for FIFO threshold - must read continuously
  uint8_t bytes_in_fifo = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;

  if (bytes_in_fifo > 0) {
    // Calculate how many bytes to read
    size_t bytes_remaining = this->expected_length_ - this->bytes_received_;

    // If FIFO is getting full (>48 bytes), read everything to prevent overflow
    // Otherwise, leave 1 byte per CC1101 errata (unless we're close to frame end)
    size_t bytes_to_read;
    if (bytes_in_fifo > 48) {
      bytes_to_read = bytes_in_fifo;  // Aggressive read when FIFO filling
    } else if (bytes_remaining <= bytes_in_fifo) {
      bytes_to_read = bytes_remaining;  // Near frame end, read what we need
    } else {
      bytes_to_read = (bytes_in_fifo > 1) ? (bytes_in_fifo - 1) : 0;  // Leave 1 byte
    }

    if (bytes_to_read > 0) {
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

      ESP_LOGVV(TAG, "Read %zu bytes from FIFO (had %d), total: %zu/%zu (buffer: %zu)",
                bytes_to_read, bytes_in_fifo, this->bytes_received_, this->expected_length_,
                this->rx_buffer_.size());
    }
  }

  // Check if complete
  if (this->bytes_received_ >= this->expected_length_) {
    ESP_LOGVV(TAG, "Frame completion check: bytes_received=%zu, expected=%zu, buffer_size=%zu",
             this->bytes_received_, this->expected_length_, this->rx_buffer_.size());

    // Read any remaining bytes in FIFO
    uint8_t bytes_in_fifo = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
    if (bytes_in_fifo > 0) {
      ESP_LOGVV(TAG, "Frame complete, reading final %d bytes from FIFO", bytes_in_fifo);
      size_t old_size = this->rx_buffer_.size();
      this->rx_buffer_.resize(old_size + bytes_in_fifo);
      this->driver_->read_rx_fifo(this->rx_buffer_.data() + old_size,
                                   bytes_in_fifo);
      ESP_LOGVV(TAG, "After final read: buffer_size=%zu", this->rx_buffer_.size());
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
