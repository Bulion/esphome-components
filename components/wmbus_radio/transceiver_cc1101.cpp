#include "transceiver_cc1101.h"
#include "cc1101_rf_settings.h"
#include "decode3of6.h"
#include "packet.h"
#include "esphome/core/log.h"
#include <algorithm>

namespace esphome {
namespace wmbus_radio {

static const char *const TAG = "cc1101";

static void log_cc1101_snapshot_(CC1101Driver &driver, InternalGPIOPin *gdo0_pin, InternalGPIOPin *gdo2_pin,
                                 const char *reason) {
  const bool gdo0 = (gdo0_pin != nullptr) ? gdo0_pin->digital_read() : false;
  const bool gdo2 = (gdo2_pin != nullptr) ? gdo2_pin->digital_read() : false;

  const uint8_t marc = driver.read_status(CC1101Status::MARCSTATE);
  const uint8_t partnum = driver.read_status(CC1101Status::PARTNUM);
  const uint8_t version = driver.read_status(CC1101Status::VERSION);
  const uint8_t rxbytes = driver.read_status(CC1101Status::RXBYTES);
  const uint8_t txbytes = driver.read_status(CC1101Status::TXBYTES);
  const uint8_t pktstatus = driver.read_status(CC1101Status::PKTSTATUS);
  const uint8_t rssi = driver.read_status(CC1101Status::RSSI);
  const uint8_t lqi = driver.read_status(CC1101Status::LQI);
  const uint8_t freqest = driver.read_status(CC1101Status::FREQEST);
  const uint8_t vco_vc_dac = driver.read_status(CC1101Status::VCO_VC_DAC);

  const uint8_t iocfg2 = driver.read_register(CC1101Register::IOCFG2);
  const uint8_t iocfg0 = driver.read_register(CC1101Register::IOCFG0);
  const uint8_t fifothr = driver.read_register(CC1101Register::FIFOTHR);
  const uint8_t pktctrl1 = driver.read_register(CC1101Register::PKTCTRL1);
  const uint8_t pktctrl0 = driver.read_register(CC1101Register::PKTCTRL0);
  const uint8_t pktlen = driver.read_register(CC1101Register::PKTLEN);
  const uint8_t mcsm2 = driver.read_register(CC1101Register::MCSM2);
  const uint8_t mcsm1 = driver.read_register(CC1101Register::MCSM1);
  const uint8_t mcsm0 = driver.read_register(CC1101Register::MCSM0);
  const uint8_t mdmcfg2 = driver.read_register(CC1101Register::MDMCFG2);
  const uint8_t sync1 = driver.read_register(CC1101Register::SYNC1);
  const uint8_t sync0 = driver.read_register(CC1101Register::SYNC0);
  const uint8_t freq2 = driver.read_register(CC1101Register::FREQ2);
  const uint8_t freq1 = driver.read_register(CC1101Register::FREQ1);
  const uint8_t freq0 = driver.read_register(CC1101Register::FREQ0);

  ESP_LOGW(TAG,
           "CC1101 snapshot (%s): MARC=0x%02X PARTNUM=0x%02X VERSION=0x%02X RXBYTES=0x%02X (n=%u ovf=%u) TXBYTES=0x%02X (n=%u) PKTSTATUS=0x%02X "
           "RSSI=0x%02X LQI=0x%02X FREQEST=0x%02X VCO_VC_DAC=0x%02X GDO0=%d GDO2=%d",
           reason, marc, partnum, version, rxbytes, rxbytes & 0x7F, (rxbytes & 0x80) ? 1 : 0, txbytes, txbytes & 0x7F, pktstatus, rssi, lqi,
           freqest, vco_vc_dac, gdo0, gdo2);
  ESP_LOGW(TAG,
           "CC1101 regs: IOCFG2=0x%02X IOCFG0=0x%02X FIFOTHR=0x%02X PKTCTRL1=0x%02X PKTCTRL0=0x%02X PKTLEN=0x%02X "
           "MCSM2=0x%02X MCSM1=0x%02X MCSM0=0x%02X MDMCFG2=0x%02X SYNC=0x%02X%02X FREQ=0x%02X%02X%02X",
           iocfg2, iocfg0, fifothr, pktctrl1, pktctrl0, pktlen, mcsm2, mcsm1, mcsm0, mdmcfg2, sync1, sync0, freq2, freq1,
           freq0);
}

CC1101::CC1101()
    : gdo0_pin_(nullptr)
    , gdo2_pin_(nullptr)
    , frequency_mhz_(868.95f)
    , rx_state_(RxLoopState::INIT_RX)
    , rx_read_index_(0)
    , bytes_received_(0)
    , expected_length_(0)
    , length_field_(0)
    , length_mode_(LengthMode::INFINITE)
    , wmbus_mode_(WMBusMode::UNKNOWN)
    , wmbus_block_(WMBusBlock::UNKNOWN)
    , sync_time_(0)
    , max_wait_time_(150) {}

const char *CC1101::get_name() {
  return "CC1101";
}

bool CC1101::is_frame_oriented() const {
  return true;
}

gpio::InterruptType CC1101::irq_interrupt_type() const {
  // GDO0 is configured as RXFIFO_THR (IOCFG0=0x00): asserts HIGH when FIFO >= threshold.
  // Use RISING_EDGE to wake the receiver task as soon as data fills the FIFO.
  return gpio::INTERRUPT_RISING_EDGE;
}

void CC1101::set_gdo0_pin(InternalGPIOPin *pin) {
  this->gdo0_pin_ = pin;
}

void CC1101::set_gdo2_pin(InternalGPIOPin *pin) {
  this->gdo2_pin_ = pin;
}

void CC1101::set_frequency(float freq_mhz) {
  this->frequency_mhz_ = freq_mhz;
}
static size_t mode_a_decoded_size(uint8_t l_field) {
  size_t num_blocks = (l_field < 26) ? 2 : ((l_field - 26) / 16 + 3);
  return l_field + 1 + 2 * num_blocks;
}
static size_t mode_t_packet_size(uint8_t l_field) {
  return encoded_size(mode_a_decoded_size(l_field));
}
static size_t mode_c_expected_length(uint8_t l_field, WMBusBlock block_type,
                                     bool has_preamble) {
  size_t base = 0;
  size_t l = l_field;
  if (block_type == WMBusBlock::BLOCK_A) {
    base = mode_a_decoded_size(l_field);
  } else if (block_type == WMBusBlock::BLOCK_B) {
    base = 1 + l;
  } else {
    return 0;
  }
  return has_preamble ? (base + 2) : base;
}
void CC1101::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CC1101...");
  if (this->gdo0_pin_ != nullptr) {
    this->gdo0_pin_->setup();
    this->gdo0_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLDOWN);
  }
  if (this->gdo2_pin_ != nullptr) {
    this->gdo2_pin_->setup();
    this->gdo2_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLDOWN);
  }
  this->common_setup();
  this->driver_ = std::make_unique<CC1101Driver>(this);
  ESP_LOGD(TAG, "Sending software reset (SRES strobe)...");
  this->driver_->send_strobe(CC1101Strobe::SRES);
  delay(10);
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
    this->status_set_error(LOG_STR("CC1101 not detected (SPI)"));
    this->mark_failed();
    return;
  }
  if (partnum != 0x00) {
    ESP_LOGW(TAG, "Unexpected PARTNUM 0x%02X (expected 0x00). Chip may not be CC1101.", partnum);
  }
  ESP_LOGCONFIG(TAG, "CC1101 detected - PARTNUM: 0x%02X, VERSION: 0x%02X", partnum, version);
  ESP_LOGD(TAG, "Applying wM-Bus RF settings (%zu registers)...", CC1101_WMBUS_RF_SETTINGS.size());
  apply_wmbus_rf_settings(*this->driver_);
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
  if (this->frequency_mhz_ != 868.95f) {
    ESP_LOGD(TAG, "Setting custom frequency: %.2f MHz", this->frequency_mhz_);
    set_carrier_frequency(*this->driver_, this->frequency_mhz_);
    uint8_t freq2 = this->driver_->read_register(CC1101Register::FREQ2);
    uint8_t freq1 = this->driver_->read_register(CC1101Register::FREQ1);
    uint8_t freq0 = this->driver_->read_register(CC1101Register::FREQ0);
    uint32_t freq_reg = (static_cast<uint32_t>(freq2) << 16) |
                        (static_cast<uint32_t>(freq1) << 8) |
                        freq0;
    float actual_freq = (freq_reg * 26.0f) / 65536.0f;
    ESP_LOGD(TAG, "Frequency registers: 0x%02X%02X%02X (%.2f MHz)", freq2, freq1, freq0, actual_freq);
  }
  ESP_LOGD(TAG, "Calibrating frequency synthesizer (SCAL strobe)...");
  this->driver_->send_strobe(CC1101Strobe::SCAL);
  delay(4);
  uint8_t marcstate = this->driver_->read_status(CC1101Status::MARCSTATE);
  ESP_LOGD(TAG, "MARCSTATE after calibration: 0x%02X (IDLE=0x01)", marcstate);
  ESP_LOGCONFIG(TAG, "CC1101 initialized successfully");
  ESP_LOGCONFIG(TAG, "  Chip version: 0x%02X", version);
  ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz", this->frequency_mhz_);
  bool gdo0_initial = (this->gdo0_pin_ != nullptr) ? this->gdo0_pin_->digital_read() : false;
  bool gdo2_initial = (this->gdo2_pin_ != nullptr) ? this->gdo2_pin_->digital_read() : false;
  ESP_LOGD(TAG, "GDO pin initial states: GDO0=%d, GDO2=%d", gdo0_initial, gdo2_initial);
  this->restart_rx();
  delay(5);
  bool gdo0_rx = (this->gdo0_pin_ != nullptr) ? this->gdo0_pin_->digital_read() : false;
  bool gdo2_rx = (this->gdo2_pin_ != nullptr) ? this->gdo2_pin_->digital_read() : false;
  ESP_LOGD(TAG, "GDO pin states in RX mode: GDO0=%d, GDO2=%d", gdo0_rx, gdo2_rx);
  if (this->gdo0_pin_ != nullptr && this->gdo2_pin_ != nullptr) {
    if (gdo0_initial == gdo0_rx && gdo2_initial == gdo2_rx) {
      ESP_LOGW(TAG, "GDO pins did not change state - check pin connections!");
    }
  }
  // Use GDO0 (FIFO threshold) as the interrupt pin so the receiver task is woken
  // by hardware as soon as data fills the FIFO, instead of busy-polling every 2 ms.
  // GDO0 is configured as RXFIFO_THR (IOCFG0=0x00): HIGH when FIFO >= threshold.
  if (this->gdo0_pin_ != nullptr) {
    this->irq_pin_ = this->gdo0_pin_;
    ESP_LOGD(TAG, "GDO0 registered as IRQ pin (RISING_EDGE = FIFO threshold reached)");
  }
  ESP_LOGCONFIG(TAG, "CC1101 setup complete");
}
void CC1101::restart_rx() {
  if (this->is_failed())
    return;
  this->set_idle_();
  this->init_rx_();
}
void CC1101::run_receiver() {
  RxLoopState state_before;
  do {
    state_before = this->rx_state_;
    this->read();
    if (this->rx_state_ == RxLoopState::FRAME_READY) {
      break;
    }
    if (this->rx_state_ == RxLoopState::WAIT_FOR_SYNC) {
      return;
    }
    if (this->rx_state_ == state_before) {
      return;
    }
  } while (this->rx_state_ != RxLoopState::FRAME_READY);
  if (this->rx_state_ != RxLoopState::FRAME_READY) {
    return;
  }
  auto packet = std::make_unique<Packet>();
  bool requires_decode = true;
  std::vector<uint8_t> frame_data = this->rx_buffer_;
  if (this->wmbus_mode_ == WMBusMode::MODE_T) {
    auto decoded = decode3of6(frame_data);
    if (!decoded.has_value()) {
      ESP_LOGW(TAG, "3-of-6 decode failed");
      this->rx_state_ = RxLoopState::INIT_RX;
      return;
    }
    frame_data = std::move(decoded.value());
    requires_decode = false;
    ESP_LOGD(TAG, "3-of-6 decode successful, decoded to %zu bytes", frame_data.size());
  }
  packet->set_data(frame_data);
  packet->set_requires_decode(requires_decode && this->wmbus_mode_ == WMBusMode::MODE_T);
  if (this->wmbus_mode_ == WMBusMode::MODE_C) {
    packet->set_link_mode_hint(LinkMode::C1);
  } else if (this->wmbus_mode_ == WMBusMode::MODE_T) {
    packet->set_link_mode_hint(LinkMode::T1);
  }
  packet->set_rssi(this->get_rssi());
  this->rx_read_index_ = this->rx_buffer_.size();
  if (!packet->calculate_payload_size()) {
    ESP_LOGD(TAG, "Cannot calculate payload size");
    this->rx_state_ = RxLoopState::INIT_RX;
    return;
  }
  auto packet_ptr = packet.get();
  if (xQueueSend(this->packet_queue_, &packet_ptr, 0) == pdTRUE) {
    ESP_LOGV(TAG, "Frame queued successfully");
    packet.release();
  } else {
    ESP_LOGW(TAG, "Queue send failed");
  }
}
int8_t CC1101::get_rssi() {
  uint8_t rssi_raw = this->driver_->read_status(CC1101Status::RSSI);
  int16_t rssi_dbm;
  if (rssi_raw >= 128) {
    rssi_dbm = ((rssi_raw - 256) / 2) - 74;
  } else {
    rssi_dbm = (rssi_raw / 2) - 74;
  }
  return static_cast<int8_t>(rssi_dbm);
}
optional<uint8_t> CC1101::read() {
  if (this->is_failed()) {
    // Setup already decided the device is not usable (e.g. not detected on SPI).
    // Returning empty keeps the receiver task quiet.
    return {};
  }
  switch (this->rx_state_) {
  case RxLoopState::FRAME_READY:
    if (this->rx_read_index_ < this->rx_buffer_.size()) {
      return this->rx_buffer_[this->rx_read_index_++];
    }
    this->rx_state_ = RxLoopState::INIT_RX;
    return {};
  case RxLoopState::INIT_RX:
    this->init_rx_();
    return {};
  case RxLoopState::WAIT_FOR_SYNC:
    if (this->wait_for_sync_()) {
      // If SPI is failing (0xFF reads), GDO may be floating or the radio may be unresponsive.
      // Don't treat this as a real sync edge; restart RX and let the setup/driver warnings guide wiring fixes.
      const uint8_t marc = this->driver_->read_status(CC1101Status::MARCSTATE);
      const uint8_t rxbytes = this->driver_->read_status(CC1101Status::RXBYTES);
      if (marc == 0xFF || rxbytes == 0xFF) {
        ESP_LOGW(TAG, "GDO2 indicates sync but SPI reads are 0xFF; ignoring and restarting RX");
        log_cc1101_snapshot_(*this->driver_, this->gdo0_pin_, this->gdo2_pin_, "sync while spi failing");
        this->init_rx_();
        return {};
      }
      ESP_LOGD(TAG, "Sync detected");
      this->rx_state_ = RxLoopState::WAIT_FOR_DATA;
      this->sync_time_ = millis();
      return {};
    }
    {
      uint8_t rxbytes_status = this->driver_->read_status(CC1101Status::RXBYTES);
      if (rxbytes_status & 0x80) {
        ESP_LOGW(TAG, "FIFO overflow while waiting for sync, flushing");
        log_cc1101_snapshot_(*this->driver_, this->gdo0_pin_, this->gdo2_pin_, "wait_for_sync overflow");
        this->init_rx_();
        return {};
      }

      // If RX FIFO is accumulating data but we never see sync (GDO2), we may be receiving noise or
      // the GDO2 wiring/config is wrong. Flush early to avoid overflow and log when it starts happening.
      const uint8_t bytes_in_fifo = rxbytes_status & 0x7F;
      if (bytes_in_fifo >= 24) {
        ESP_LOGD(TAG, "RX FIFO accumulating without sync (n=%u), flushing", bytes_in_fifo);
        log_cc1101_snapshot_(*this->driver_, this->gdo0_pin_, this->gdo2_pin_, "wait_for_sync fifo accumulating");
        this->init_rx_();
        return {};
      }
    }
    return {};
  case RxLoopState::WAIT_FOR_DATA:
    if (millis() - this->sync_time_ > this->max_wait_time_) {
      ESP_LOGW(TAG, "Timeout waiting for data after sync! Resetting RX.");
      this->rx_state_ = RxLoopState::INIT_RX;
      return {};
    }
    if (this->wait_for_data_()) {
      ESP_LOGD(TAG, "Header received, processing frame data");
      this->rx_state_ = RxLoopState::READ_DATA;
    } else {
      return {};
    }
    [[fallthrough]];
  case RxLoopState::READ_DATA:
    while (true) {
      size_t bytes_before = this->bytes_received_;
      if (this->read_data_()) {
        ESP_LOGI(TAG, "Frame received: %zu bytes, mode: %c, L=0x%02X",
                 this->rx_buffer_.size(),
                 static_cast<char>(this->wmbus_mode_),
                 this->length_field_);
        this->rx_state_ = RxLoopState::FRAME_READY;
        this->rx_read_index_ = 0;
        if (!this->rx_buffer_.empty()) {
          return this->rx_buffer_[this->rx_read_index_++];
        }
        ESP_LOGW(TAG, "RX buffer empty after frame reception");
        this->rx_state_ = RxLoopState::INIT_RX;
        return {};
      }
      if (this->bytes_received_ == bytes_before) {
        break;
      }
    }
    return {};
  default:
    this->rx_state_ = RxLoopState::INIT_RX;
    return {};
  }
}
void CC1101::init_rx_() {
  this->set_idle_();
  this->driver_->send_strobe(CC1101Strobe::SFTX);
  this->driver_->send_strobe(CC1101Strobe::SFRX);
  this->driver_->write_register(CC1101Register::FIFOTHR, 0x0A);
  this->driver_->write_register(CC1101Register::PKTCTRL0, 0x02);
  this->rx_buffer_.clear();
  this->rx_read_index_ = 0;
  this->bytes_received_ = 0;
  this->expected_length_ = 0;
  this->length_field_ = 0;
  this->length_mode_ = LengthMode::INFINITE;
  this->wmbus_mode_ = WMBusMode::UNKNOWN;
  this->wmbus_block_ = WMBusBlock::UNKNOWN;

  // Enter RX and verify MARCSTATE transitions. If the chip reports SLEEP (0x00) or RX_OVERFLOW (0x11),
  // attempt a minimal recovery sequence and log a snapshot.
  this->driver_->send_strobe(CC1101Strobe::SRX);
  delay(1);
  uint8_t marc_state = 0xFF;
  uint8_t first_marc_state = 0xFF;
  bool rx_entered = false;
  for (int i = 0; i < 50; i++) {
    marc_state = this->driver_->read_status(CC1101Status::MARCSTATE);
    if (i == 0)
      first_marc_state = marc_state;

    if (marc_state == static_cast<uint8_t>(CC1101State::RX)) {
      rx_entered = true;
      break;
    }

    if (marc_state == static_cast<uint8_t>(CC1101State::RX_OVERFLOW)) {
      ESP_LOGD(TAG, "MARCSTATE indicates RX overflow during RX entry, flushing and retrying");
      this->driver_->send_strobe(CC1101Strobe::SFRX);
      delay(1);
      this->driver_->send_strobe(CC1101Strobe::SRX);
    } else if (marc_state == static_cast<uint8_t>(CC1101State::SLEEP)) {
      // If we ever read SLEEP here, it's a strong hint of SPI/chip-select issues or unintended powerdown.
      // Try to wake to IDLE then re-enter RX.
      this->driver_->send_strobe(CC1101Strobe::SIDLE);
      delay(1);
      this->driver_->send_strobe(CC1101Strobe::SRX);
    }

    delay(1);
  }
  if (!rx_entered) {
    ESP_LOGW(TAG, "Failed to enter RX mode! MARCSTATE first=0x%02X last=0x%02X (expected RX=0x0D)", first_marc_state,
             marc_state);
    log_cc1101_snapshot_(*this->driver_, this->gdo0_pin_, this->gdo2_pin_, "init_rx failed");
  }
  this->rx_state_ = RxLoopState::WAIT_FOR_SYNC;
}
bool CC1101::wait_for_sync_() {
  if (this->gdo2_pin_ != nullptr) {
    return this->gdo2_pin_->digital_read();
  }
  // No GDO2 wired/configured: fall back to polling RX FIFO.
  // This is less robust than GDO2 sync, but is very useful for bring-up/debug.
  const uint8_t rxbytes_status = this->driver_->read_status(CC1101Status::RXBYTES);
  if (rxbytes_status == 0xFF) {
    return false;
  }
  // If at least a header's worth of bytes is present, treat it as a potential frame start.
  return (rxbytes_status & 0x7F) >= 4;
}
bool CC1101::wait_for_data_() {
  uint8_t rxbytes_status = this->driver_->read_status(CC1101Status::RXBYTES);
  if (rxbytes_status == 0xFF) {
    ESP_LOGW(TAG, "SPI read failed (RXBYTES=0xFF) while reading header; restarting RX");
    log_cc1101_snapshot_(*this->driver_, this->gdo0_pin_, this->gdo2_pin_, "wait_for_data spi fail");
    this->rx_state_ = RxLoopState::INIT_RX;
    return false;
  }
  if (rxbytes_status & 0x80) {
    ESP_LOGW(TAG, "RX FIFO overflow while reading header");
    this->rx_state_ = RxLoopState::INIT_RX;
    return false;
  }
  uint8_t bytes_in_fifo = rxbytes_status & 0x7F;
  if (bytes_in_fifo < 4) {
    return false;
  }
  ESP_LOGD(TAG, "FIFO has %d bytes, reading header", bytes_in_fifo);
  uint8_t header[4];
  this->driver_->read_rx_fifo(header, 4);
  ESP_LOGD(TAG, "Header bytes: %02X %02X %02X %02X", header[0], header[1], header[2], header[3]);
  if (header[0] == WMBUS_MODE_C_PREAMBLE) {
    this->wmbus_mode_ = WMBusMode::MODE_C;
    if (header[1] == WMBUS_BLOCK_A_PREAMBLE) {
      this->wmbus_block_ = WMBusBlock::BLOCK_A;
    } else if (header[1] == WMBUS_BLOCK_B_PREAMBLE) {
      this->wmbus_block_ = WMBusBlock::BLOCK_B;
    } else {
      ESP_LOGV(TAG, "Unknown Mode C block type: 0x%02X", header[1]);
      return false;
    }
    this->length_field_ = header[2];
    this->rx_buffer_.insert(this->rx_buffer_.end(), header, header + 4);
    this->expected_length_ =
        mode_c_expected_length(this->length_field_, this->wmbus_block_, true);
  } else {
    std::vector<uint8_t> header_vec(header, header + 3);
    auto decoded_header = decode3of6(header_vec);
    if (decoded_header.has_value() && decoded_header->size() >= 1) {
      uint8_t decoded_l_field = (*decoded_header)[0];
      if (decoded_l_field >= 10) {
        this->wmbus_mode_ = WMBusMode::MODE_T;
        this->wmbus_block_ = WMBusBlock::BLOCK_A;
        this->length_field_ = decoded_l_field;
        this->expected_length_ = mode_t_packet_size(this->length_field_);
        this->rx_buffer_.insert(this->rx_buffer_.end(), header, header + 4);
        ESP_LOGD(TAG, "Mode T detected: L=0x%02X (decoded from 3-of-6), expected_length=%zu",
                 this->length_field_, this->expected_length_);
      } else {
        this->wmbus_mode_ = WMBusMode::MODE_C;
        this->wmbus_block_ = WMBusBlock::BLOCK_A;
        this->length_field_ = header[0];
        this->expected_length_ =
            mode_c_expected_length(this->length_field_, this->wmbus_block_, false);
        this->rx_buffer_.push_back(WMBUS_MODE_C_PREAMBLE);
        this->rx_buffer_.push_back(WMBUS_BLOCK_A_PREAMBLE);
        this->rx_buffer_.insert(this->rx_buffer_.end(), header, header + 4);
        ESP_LOGD(TAG, "Mode C (no preamble): L=0x%02X, expected_length=%zu",
                 this->length_field_, this->expected_length_);
      }
    } else {
      this->wmbus_mode_ = WMBusMode::MODE_C;
      this->wmbus_block_ = WMBusBlock::BLOCK_A;
      this->length_field_ = header[0];
      this->expected_length_ =
          mode_c_expected_length(this->length_field_, this->wmbus_block_, false);
      this->rx_buffer_.push_back(WMBUS_MODE_C_PREAMBLE);
      this->rx_buffer_.push_back(WMBUS_BLOCK_A_PREAMBLE);
      this->rx_buffer_.insert(this->rx_buffer_.end(), header, header + 4);
      ESP_LOGD(TAG, "Mode C (fallback): L=0x%02X, expected_length=%zu",
               this->length_field_, this->expected_length_);
    }
  }
  if (this->expected_length_ == 0) {
    ESP_LOGW(TAG, "Unable to determine expected frame length (block=%c, L=0x%02X)",
             static_cast<char>(this->wmbus_block_), this->length_field_);
    return false;
  }
  this->bytes_received_ = 4;
  if (this->expected_length_ < this->bytes_received_) {
    ESP_LOGW(TAG, "Expected length %zu smaller than bytes already read %zu, adjusting",
             this->expected_length_, this->bytes_received_);
    this->expected_length_ = this->bytes_received_;
  }
  ESP_LOGD(TAG, "Frame detected: mode=%c, block=%c, L=0x%02X, expected=%zu",
           static_cast<char>(this->wmbus_mode_),
           static_cast<char>(this->wmbus_block_), this->length_field_,
           this->expected_length_);
  if (this->expected_length_ < MAX_FIXED_LENGTH) {
    this->driver_->write_register(CC1101Register::PKTLEN,
                                   static_cast<uint8_t>(this->expected_length_));
    this->driver_->write_register(CC1101Register::PKTCTRL0, 0x00);
    this->length_mode_ = LengthMode::FIXED;
  }
  this->driver_->write_register(CC1101Register::FIFOTHR, RX_FIFO_THRESHOLD);
  bytes_in_fifo = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
  if (bytes_in_fifo > 0) {
    size_t bytes_remaining = this->expected_length_ - this->bytes_received_;
    size_t bytes_to_read = std::min(static_cast<size_t>(bytes_in_fifo), bytes_remaining);
    if (bytes_to_read > 0) {
      size_t old_size = this->rx_buffer_.size();
      this->rx_buffer_.resize(old_size + bytes_to_read);
      this->driver_->read_rx_fifo(this->rx_buffer_.data() + old_size, bytes_to_read);
      this->bytes_received_ += bytes_to_read;
    }
  }
  return true;
}
bool CC1101::read_data_() {
  bool gdo2 = (this->gdo2_pin_ != nullptr) ? this->gdo2_pin_->digital_read() : true;
  if (!gdo2 && this->bytes_received_ > 0) {
    uint8_t bytes_in_fifo = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
    if (bytes_in_fifo > 0) {
      // Cap reads to the remaining expected bytes so we never consume bytes that
      // belong to the next frame (which may already be flowing into the FIFO).
      size_t bytes_remaining = this->expected_length_ - this->bytes_received_;
      size_t bytes_to_read = std::min(static_cast<size_t>(bytes_in_fifo), bytes_remaining);
      if (bytes_to_read > 0) {
        ESP_LOGD(TAG, "GDO2 LOW detected, reading final %zu bytes", bytes_to_read);
        size_t old_size = this->rx_buffer_.size();
        this->rx_buffer_.resize(old_size + bytes_to_read);
        this->driver_->read_rx_fifo(this->rx_buffer_.data() + old_size, bytes_to_read);
        this->bytes_received_ += bytes_to_read;
      }
    }
    ESP_LOGD(TAG, "Frame complete via GDO2: %zu bytes", this->bytes_received_);
    return true;
  }
  if (this->check_rx_overflow_()) {
    ESP_LOGW(TAG, "RX FIFO overflow during read, aborting frame");
    this->rx_state_ = RxLoopState::INIT_RX;
    return false;
  }
  // MARCSTATE fallback: if the chip has already returned to IDLE or RX_END the packet
  // is complete. Drain whatever is left in the FIFO and treat it as frame end.
  // This mirrors the errata workaround used in SzczepanLeon's implementation and
  // makes reception robust even when GDO2 timing is imperfect.
  if (this->bytes_received_ > 0) {
    uint8_t marc = this->driver_->read_status(CC1101Status::MARCSTATE) & 0x1F;
    constexpr uint8_t MARC_IDLE = static_cast<uint8_t>(CC1101State::IDLE);
    constexpr uint8_t MARC_RX_END = 0x0E;
    if (marc == MARC_IDLE || marc == MARC_RX_END) {
      // CC1101 errata: the state machine can advance to IDLE while the last few bytes
      // are still being written to the FIFO.  Retry reading RXBYTES a few times before
      // declaring the frame incomplete.
      for (int retry = 0; retry < MARCSTATE_RETRY_COUNT && this->bytes_received_ < this->expected_length_; retry++) {
        uint8_t remaining = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
        if (remaining > 0) {
          size_t to_drain = std::min(static_cast<size_t>(remaining),
                                     this->expected_length_ - this->bytes_received_);
          if (to_drain > 0 && this->rx_buffer_.size() + to_drain <= MAX_FRAME_SIZE) {
            size_t old_size = this->rx_buffer_.size();
            this->rx_buffer_.resize(old_size + to_drain);
            this->driver_->read_rx_fifo(this->rx_buffer_.data() + old_size, to_drain);
            this->bytes_received_ += to_drain;
          }
        } else {
          delayMicroseconds(MARCSTATE_RETRY_DELAY_US);
        }
      }
      if (this->bytes_received_ >= this->expected_length_) {
        ESP_LOGD(TAG, "Frame complete via MARCSTATE (0x%02X): %zu bytes", marc, this->bytes_received_);
        return true;
      }
      ESP_LOGW(TAG, "MARCSTATE end (0x%02X) but only %zu/%zu bytes received, discarding",
               marc, this->bytes_received_, this->expected_length_);
      this->rx_state_ = RxLoopState::INIT_RX;
      return false;
    }
  }
  uint8_t bytes_in_fifo = this->driver_->read_status(CC1101Status::RXBYTES) & 0x7F;
  if (bytes_in_fifo > 0) {
    size_t bytes_remaining = this->expected_length_ - this->bytes_received_;
    size_t bytes_to_read;
    if (bytes_remaining <= bytes_in_fifo) {
      bytes_to_read = bytes_remaining;
    } else {
      bytes_to_read = (bytes_in_fifo > 1) ? (bytes_in_fifo - 1) : 0;
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
    }
  }
  if (this->bytes_received_ >= this->expected_length_) {
    // Do NOT drain the FIFO further: any remaining bytes belong to the next frame.
    // The FIFO will be flushed by init_rx_() before the next reception cycle.
    return true;
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
  return (rxbytes & 0x80) != 0;
}
}
}
