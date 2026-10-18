#include "cc1101_driver.h"
#include "esphome/core/log.h"
#include <inttypes.h>

namespace esphome {
namespace wmbus_radio {

static const char *const TAG = "cc1101_driver";

static constexpr uint8_t CC1101_SPI_MAX_RETRIES = 5;
static constexpr uint32_t CC1101_SPI_RETRY_DELAY_US = 50;

static bool should_log_spi_ff_warning_() {
  static uint32_t last_warn_ms = 0;
  static uint32_t suppressed = 0;
  const uint32_t now = millis();
  if (now - last_warn_ms >= 1000) {
    last_warn_ms = now;
    if (suppressed > 0) {
      ESP_LOGW(TAG, "suppressed %" PRIu32 " repeated SPI 0xFF warnings", suppressed);
      suppressed = 0;
    }
    return true;
  }
  suppressed++;
  return false;
}

uint8_t CC1101Driver::read_register(CC1101Register reg) {
  uint8_t addr = static_cast<uint8_t>(reg) | CC1101_READ_SINGLE;
  uint8_t value = 0xFF;
  uint8_t status_byte = 0xFF;

  for (uint8_t attempt = 0; attempt < CC1101_SPI_MAX_RETRIES; attempt++) {
    this->spi_->enable();
    status_byte = this->spi_->transfer_byte(addr);
    value = this->spi_->transfer_byte(0x00);
    this->spi_->disable();

    if (status_byte != 0xFF) {
      if (attempt > 0) {
        ESP_LOGV(TAG, "read_register retry ok reg=0x%02X attempts=%u", static_cast<uint8_t>(reg), attempt + 1);
      }
      return value;
    }
    delayMicroseconds(CC1101_SPI_RETRY_DELAY_US);
  }

  if (should_log_spi_ff_warning_()) {
    ESP_LOGW(TAG, "read_register returned status 0xFF reg=0x%02X (SPI not ready/wiring?)", static_cast<uint8_t>(reg));
  }
  return 0xFF;
}

void CC1101Driver::write_register(CC1101Register reg, uint8_t value) {
  uint8_t addr = static_cast<uint8_t>(reg);

  this->spi_->enable();
  this->spi_->transfer_byte(addr);
  this->spi_->transfer_byte(value);
  this->spi_->disable();
}

uint8_t CC1101Driver::read_status(CC1101Status status) {
  uint8_t addr = static_cast<uint8_t>(status) | CC1101_READ_BURST;
  uint8_t value = 0xFF;
  uint8_t status_byte = 0xFF;

  for (uint8_t attempt = 0; attempt < CC1101_SPI_MAX_RETRIES; attempt++) {
    this->spi_->enable();
    status_byte = this->spi_->transfer_byte(addr);
    value = this->spi_->transfer_byte(0x00);
    this->spi_->disable();

    if (status_byte != 0xFF) {
      if (attempt > 0) {
        ESP_LOGV(TAG, "read_status retry ok status=0x%02X attempts=%u", static_cast<uint8_t>(status), attempt + 1);
      }
      return value;
    }
    delayMicroseconds(CC1101_SPI_RETRY_DELAY_US);
  }

  if (should_log_spi_ff_warning_()) {
    ESP_LOGW(TAG, "read_status returned status 0xFF status=0x%02X (SPI not ready/wiring?)", static_cast<uint8_t>(status));
  }
  return 0xFF;
}

void CC1101Driver::read_burst(CC1101Register reg, uint8_t *buffer,
                               size_t length) {
  if (length == 0)
    return;

  uint8_t addr = static_cast<uint8_t>(reg) | CC1101_READ_BURST;

  this->spi_->enable();
  this->spi_->transfer_byte(addr);
  this->spi_->transfer_array(buffer, length);
  this->spi_->disable();
}

void CC1101Driver::write_burst(CC1101Register reg, const uint8_t *buffer,
                                size_t length) {
  if (length == 0)
    return;

  uint8_t addr = static_cast<uint8_t>(reg) | CC1101_WRITE_BURST;

  this->spi_->enable();
  this->spi_->transfer_byte(addr);
  for (size_t i = 0; i < length; i++) {
    this->spi_->transfer_byte(buffer[i]);
  }
  this->spi_->disable();
}

uint8_t CC1101Driver::send_strobe(CC1101Strobe strobe) {
  uint8_t addr = static_cast<uint8_t>(strobe);
  uint8_t status = 0xFF;

  for (uint8_t attempt = 0; attempt < CC1101_SPI_MAX_RETRIES; attempt++) {
    this->spi_->enable();
    status = this->spi_->transfer_byte(addr);
    this->spi_->disable();

    if (status != 0xFF) {
      if (attempt > 0) {
        ESP_LOGV(TAG, "send_strobe retry ok strobe=0x%02X attempts=%u", static_cast<uint8_t>(strobe), attempt + 1);
      }
      return status;
    }
    delayMicroseconds(CC1101_SPI_RETRY_DELAY_US);
  }

  if (should_log_spi_ff_warning_()) {
    ESP_LOGW(TAG, "send_strobe returned 0xFF strobe=0x%02X (SPI not ready/wiring?)", static_cast<uint8_t>(strobe));
  }
  return status;
}

void CC1101Driver::read_rx_fifo(uint8_t *buffer, size_t length) {
  if (length == 0)
    return;

  uint8_t addr = CC1101_FIFO | CC1101_READ_BURST;

  this->spi_->enable();
  this->spi_->transfer_byte(addr);
  this->spi_->transfer_array(buffer, length);
  this->spi_->disable();
}

void CC1101Driver::write_tx_fifo(const uint8_t *buffer, size_t length) {
  if (length == 0)
    return;

  uint8_t addr = CC1101_FIFO | CC1101_WRITE_BURST;

  this->spi_->enable();
  this->spi_->transfer_byte(addr);
  for (size_t i = 0; i < length; i++) {
    this->spi_->transfer_byte(buffer[i]);
  }
  this->spi_->disable();
}

} // namespace wmbus_radio
} // namespace esphome
