#include "component.h"

#include "freertos/queue.h"
#include "freertos/task.h"

// wmbus components require substantial stack space for AES decryption, DV parsing, and
// meter driver dispatch.  ESPHome >=2026.6 reduced the default loop task stack from 32 KB
// to 8 KB, which is not enough and causes a LoadProhibited crash at boot.
// Fix: add the following to your YAML config:
//
//   esp32:
//     advanced:
//       loop_task_stack_size: 32768
//
#ifdef ESPHOME_LOOP_TASK_STACK_SIZE
static_assert(ESPHOME_LOOP_TASK_STACK_SIZE >= 16384,
              "wmbus_radio requires a larger loop task stack. "
              "Add 'loop_task_stack_size: 32768' under 'esp32: advanced:' in your YAML.");
#endif

namespace esphome {
namespace wmbus_radio {
static const char *TAG = "wmbus";
Radio::Radio()
    : radio(nullptr)
    , receiver_task_handle_(nullptr)
    , packet_queue_(nullptr) {}

void Radio::set_radio(RadioTransceiver *radio) {
  this->radio = radio;
}

void Radio::setup() {
  this->packet_queue_ = xQueueCreate(3, sizeof(Packet *));
  if (this->packet_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create packet queue");
    this->mark_failed();
    return;
  }

  if (this->radio == nullptr) {
    ESP_LOGE(TAG, "Radio transceiver is not configured");
    this->mark_failed();
    return;
  }

  this->radio->set_packet_queue(this->packet_queue_);

  if (this->radio->is_failed()) {
    ESP_LOGE(TAG, "Radio transceiver failed during setup; not starting receiver task");
    this->mark_failed();
    return;
  }

  if (xTaskCreate((TaskFunction_t)this->receiver_task, "radio_recv", 3 * 1024,
                  this, 2, &(this->receiver_task_handle_)) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create receiver task");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "Receiver task created [%p]", this->receiver_task_handle_);

  if (this->radio->has_irq_pin()) {
    this->radio->attach_data_interrupt(Radio::wakeup_receiver_task_from_isr,
                                       &(this->receiver_task_handle_));
  }
}

void Radio::wakeup_polling_receiver_task() {
  // Intentionally empty.
  // CC1101 now uses GDO0 as an interrupt pin (RISING_EDGE on FIFO threshold) so it
  // no longer relies on periodic polling via this function.  It is retained for any
  // future frame-oriented radio that truly has no interrupt capability.
}

void Radio::loop() {
  if (this->is_failed() || this->radio == nullptr)
    return;

  Packet *p;
  if (xQueueReceive(this->packet_queue_, &p, 0) != pdPASS)
    return;

  // ESP_LOGI(TAG, "Have RAW data from radio (%zu bytes)",
  //          p->calculate_payload_size());

  auto frame = p->convert_to_frame();

  if (!frame)
    return;

  ESP_LOGI(TAG, "Have data (%zu bytes) [RSSI: %ddBm, mode: %s %s]",
           frame->data().size(), frame->rssi(), toString(frame->link_mode()),
           frame->format().c_str());

  uint8_t packet_handled = 0;
  for (auto &handler : this->handlers_)
    handler(&frame.value());

  if (frame->handlers_count())
    ESP_LOGI(TAG, "Telegram handled by %d handlers", frame->handlers_count());
  else {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    ESP_LOGD(TAG, "Telegram not handled by any handler");
    Telegram t;
    if (t.parseHeader(frame->data()) && t.addresses.empty()) {
      ESP_LOGD(TAG, "Check if telegram can be parsed on:");
    } else {
      ESP_LOGD(TAG, "Check if telegram with address %s can be parsed on:",
               t.addresses.back().id.c_str());
    }
    ESP_LOGD(TAG,
             (std::string{"https://wmbusmeters.org/analyze/"} + frame->as_hex())
                 .c_str());
#endif
  }
}

void Radio::wakeup_receiver_task_from_isr(TaskHandle_t *arg) {
  BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(*arg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Radio::receive_frame() {
  if (this->is_failed() || this->radio == nullptr || this->radio->is_failed()) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  }
  bool is_frame_oriented = this->radio->is_frame_oriented();
  bool use_interrupt = this->radio->has_irq_pin();

  if (is_frame_oriented && !use_interrupt) {
    this->radio->run_receiver();
    vTaskDelay(pdMS_TO_TICKS(this->radio->get_polling_interval()));
    return;
  }

  static bool rx_initialized = false;
  if (is_frame_oriented && !rx_initialized) {
    this->radio->restart_rx();
    rx_initialized = true;
  } else if (!is_frame_oriented) {
    this->radio->restart_rx();
  }

  uint32_t timeout_ms = is_frame_oriented
                        ? this->radio->get_polling_interval()
                        : 60000;

  if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms))) {
    if (!is_frame_oriented) {
      ESP_LOGD(TAG, "Radio interrupt timeout");
    } else if (use_interrupt) {
      // Frame-oriented + IRQ: on timeout, still run the receiver as a safety net.
      // This handles the tail bytes of a frame that fall below the FIFO threshold
      // and would not trigger another GDO0 rising-edge interrupt on their own.
      this->radio->run_receiver();
    }
    return;
  }

  if (is_frame_oriented) {
    this->radio->run_receiver();
    return;
  }

  auto packet = std::make_unique<Packet>();

  if (!this->radio->read_in_task(packet->rx_data_ptr(),
                                 packet->rx_capacity())) {
    ESP_LOGV(TAG, "Failed to read preamble");
    return;
  }

  if (!packet->calculate_payload_size()) {
    ESP_LOGD(TAG, "Cannot calculate payload size");
    return;
  }

  if (!this->radio->read_in_task(packet->rx_data_ptr(),
                                 packet->rx_capacity())) {
    ESP_LOGW(TAG, "Failed to read data");
    return;
  }

  packet->set_rssi(this->radio->get_rssi());
  auto packet_ptr = packet.get();

  if (xQueueSend(this->packet_queue_, &packet_ptr, 0) == pdTRUE) {
    ESP_LOGV(TAG, "Queue items: %zu",
             uxQueueMessagesWaiting(this->packet_queue_));
    ESP_LOGV(TAG, "Queue send success");
    packet.release();
  } else
    ESP_LOGW(TAG, "Queue send failed");
}

void Radio::receiver_task(Radio *arg) {
  int counter = 0;
  while (true)
    arg->receive_frame();
}

void Radio::add_frame_handler(std::function<void(Frame *)> &&callback) {
  this->handlers_.push_back(std::move(callback));
}

} // namespace wmbus_radio
} // namespace esphome
