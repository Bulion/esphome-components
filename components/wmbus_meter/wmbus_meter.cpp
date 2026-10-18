#include "wmbus_meter.h"
#include "esphome/components/wmbus_common/meters_common_implementation.h"

namespace esphome {
namespace wmbus_meter {
static const char *TAG = "wmbus_meter";

void Meter::set_meter_params(std::string id, std::string driver,
                             std::string key,
                             std::initializer_list<LinkMode> linkModes) {
  MeterInfo meter_info;
  meter_info.parse(driver + '-' + id, driver, id + ",", key);

  this->meter = createMeter(&meter_info);

  if (this->meter == nullptr) {
    ESP_LOGE(TAG, "Failed to create meter driver '%s' for meter_id=0x%s", driver.c_str(), id.c_str());
    ESP_LOGE(TAG, "This usually means the driver was not compiled in. Ensure wmbus_common: drivers includes '%s' (or set drivers: all).", driver.c_str());
    this->mark_failed();
    return;
  }

  for (auto linkMode : linkModes)
    this->link_modes_.addLinkMode(linkMode);
}
void Meter::set_radio(wmbus_radio::Radio *radio) {
  this->radio = radio;
  if (this->radio == nullptr) {
    ESP_LOGE(TAG, "Radio not set");
    this->mark_failed();
    return;
  }
  radio->add_frame_handler(
      [this](wmbus_radio::Frame *frame) { return this->handle_frame(frame); });
}
void Meter::dump_config() {
  if (this->meter == nullptr) {
    ESP_LOGCONFIG(TAG, "wM-Bus Meter:");
    ESP_LOGCONFIG(TAG, "  Driver: (not initialized)");
    ESP_LOGCONFIG(TAG, "  Status: FAILED (driver not available)");
    return;
  }

  std::string id = this->get_id();
  std::string driver = this->get_driver();
  MeterKeys *keys = this->meter->meterKeys();

  ESP_LOGCONFIG(TAG, "wM-Bus Meter:");
  ESP_LOGCONFIG(TAG, "  ID: 0x%s", id.c_str());
  ESP_LOGCONFIG(TAG, "  Driver: %s", driver.c_str());
  ESP_LOGCONFIG(TAG, "  Key: %s",
                keys->hasConfidentialityKey() ? "configured"
                                              : "not configured");
}

std::string Meter::get_id() {
  if (this->meter == nullptr)
    return "unknown";
  std::vector<AddressExpression> address_expressions =
      this->meter->addressExpressions();
  return address_expressions.size() > 0 ? address_expressions[0].id : "unknown";
}

std::string Meter::get_driver() {
  return this->meter != nullptr ? this->meter->driverName().str() : "unknown";
}

std::string Meter::get_key() {
  if (this->meter == nullptr)
    return "unknown";
  MeterKeys *keys = this->meter->meterKeys();
  return keys->hasConfidentialityKey() ? bin2hex(keys->confidentiality_key)
                                       : "not-encrypted";
}

void Meter::handle_frame(wmbus_radio::Frame *frame) {
  if (this->is_failed() || this->meter == nullptr)
    return;

  if (!this->link_modes_.has(frame->link_mode())) {
    ESP_LOGW(TAG, "Frame link mode %s not supported by meter %s",
             toString(frame->link_mode()), this->meter->name().c_str());
    return;
  }

  // Quick header-only check before allocating the full Telegram on the heap.
  // This avoids expensive heap allocation for frames that don't match this meter.
  Telegram header_check(false);
  if (!header_check.parseHeader(frame->data()))
    return;
  // nullptr for MeterInfo is correct: isTelegramForMeter requires exactly one
  // of meter or mi to be non-null; we supply meter, so mi must be nullptr.
  if (!MeterCommonImplementation::isTelegramForMeter(&header_check, this->meter.get(), nullptr))
    return;

  auto about =
      AboutTelegram(App.get_friendly_name(), frame->rssi(), FrameType::WMBUS);

  std::vector<Address> adresses;
  bool id_match = false;
  auto telegram = std::make_unique<Telegram>();

  this->meter->handleTelegram(about, frame->data(), false, &adresses, &id_match,
                              telegram.get());

  if (id_match) {
    ESP_LOGI(TAG, "Telegram matched %s (RSSI: %d dBm, mode: %s)", this->meter->name().c_str(), frame->rssi(),
             toString(frame->link_mode()));
    this->last_telegram = std::move(telegram);
    this->defer([this]() {
      this->on_telegram_callback_manager();
      this->last_telegram = nullptr;
    });

    frame->mark_as_handled();
  }
}

std::string Meter::as_json(bool pretty_print) {
  if (this->meter == nullptr || this->last_telegram == nullptr)
    return "{}";
  std::string json;
  this->meter->printMeter(this->last_telegram.get(), nullptr, nullptr, '\t',
                          &json, nullptr, nullptr, nullptr, pretty_print);
  return json;
}

optional<std::string> Meter::get_string_field(std::string field_name) {

  if (this->meter == nullptr)
    return {};

  if (field_name == "timestamp")
    return this->meter->datetimeOfUpdateHumanReadable();

  if (field_name == "timestamp_zulu")
    return this->meter->datetimeOfUpdateRobot();

  auto field_info = this->meter->findFieldInfo(field_name, Quantity::Text);
  if (field_info)
    return this->meter->getStringValue(field_info);

  // Fallback: try PointInTime (date/datetime) fields.
  // Date fields like "history_1_date" are stored as numeric values keyed by
  // (vname, unit), e.g. ("history_1", DateLT). findFieldInfo() cannot be used
  // here because for template-based fields (e.g. "history_{storage_counter - 1
  // counter}") it matches by fi->vname() which returns the raw template string,
  // not the resolved instance name "history_1". Instead we mirror what the JSON
  // generator does: extract vname and unit from the field name, verify the unit
  // belongs to Quantity::PointInTime, then call getNumericValue(vname, unit)
  // directly using the string-based overload.
  std::string vname;
  Unit unit;
  if (extractUnit(field_name, &vname, &unit) && toQuantity(unit) == Quantity::PointInTime) {
    double value = this->meter->getNumericValue(vname, unit);
    if (!std::isnan(value)) {
      if (unit == Unit::DateLT)
        return strdate(value);
      if (unit == Unit::DateTimeLT)
        return strdatetime(value);
      if (unit == Unit::DateTimeUTC)
        return strTimestampUTC(value);
      // Covers Unit::TimeLT, Unit::UnixTimestamp and any future PointInTime units.
      return valueToString(value, unit);
    }
  }

  return {};
}

optional<float> Meter::get_numeric_field(std::string field_name) {
  // RSSI is not handled by meter but by telegram :/
  if (field_name == "rssi_dbm") {
    if (this->last_telegram == nullptr)
      return {};
    return this->last_telegram->about.rssi_dbm;
  }

  if (this->meter == nullptr)
    return {};

  if (field_name == "timestamp")
    return this->meter->timestampLastUpdate();

  std::string name;
  Unit unit;
  extractUnit(field_name, &name, &unit);

  auto value = this->meter->getNumericValue(name, unit);

  if (!std::isnan(value))
    return value;

  return {};
}

void Meter::on_telegram(std::function<void()> &&callback) {
  this->on_telegram_callback_manager.add(std::move(callback));
}

} // namespace wmbus_meter
} // namespace esphome
