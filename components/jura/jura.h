#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

namespace esphome {
namespace jura {

// ESP-IDF compatible replacements for Arduino bitRead()/bitWrite()
static inline uint8_t jura_bit_read(uint8_t value, uint8_t bit) {
  return static_cast<uint8_t>((value >> bit) & 0x01U);
}

static inline void jura_bit_write(uint8_t &value, uint8_t bit, bool state) {
  if (state) {
    value = static_cast<uint8_t>(value | (1U << bit));
  } else {
    value = static_cast<uint8_t>(value & ~(1U << bit));
  }
}


class Jura : public PollingComponent, public uart::UARTDevice {
 public:
  void set_model(const std::string &m) {
    model_ = m;
  }

  void register_metric_sensor(const std::string &key, sensor::Sensor *s) {
    numeric_[key] = s;
  }

  void register_text_sensor(const std::string &key, text_sensor::TextSensor *t) {
    text_[key] = t;
  }

  void publish_number(const std::string &key, float value) {
    auto it = numeric_.find(key);

    if (it != numeric_.end() && it->second != nullptr) {
      it->second->publish_state(value);
    }
  }

  void publish_text(const std::string &key, const std::string &value) {
    auto it = text_.find(key);

    if (it != text_.end() && it->second != nullptr) {
      it->second->publish_state(value);
    }
  }


  std::string cmd2jura(std::string outbytes) {
    std::string inbytes;
    const uint32_t start_time = millis();

    // Clear pending UART data
    while (available()) {
      read();
    }

    // Jura command termination
    outbytes += "\r\n";

    // -----------------------------------------------------------------------
    // Encode outgoing bytes
    //
    // Jura protocol transfers two useful bits per physical UART byte.
    // Useful bits are placed into bit 2 and bit 5.
    // -----------------------------------------------------------------------
    for (size_t i = 0; i < outbytes.size(); ++i) {
      uint8_t src = static_cast<uint8_t>(outbytes[i]);

      for (uint8_t s = 0; s < 8; s += 2) {
        uint8_t rawbyte = 0xFF;

        jura_bit_write(
            rawbyte,
            2,
            jura_bit_read(src, static_cast<uint8_t>(s + 0)));

        jura_bit_write(
            rawbyte,
            5,
            jura_bit_read(src, static_cast<uint8_t>(s + 1)));

        write(rawbyte);
      }

      delay(8);
    }


    // -----------------------------------------------------------------------
    // Decode incoming Jura data
    // -----------------------------------------------------------------------
    uint8_t bit_position = 0;
    uint8_t inbyte = 0;

    while (!(inbytes.size() >= 2 &&
             inbytes[inbytes.size() - 2] == '\r' &&
             inbytes[inbytes.size() - 1] == '\n')) {

      if (available()) {
        uint8_t rawbyte = static_cast<uint8_t>(read());

        jura_bit_write(
            inbyte,
            static_cast<uint8_t>(bit_position + 0),
            jura_bit_read(rawbyte, 2));

        jura_bit_write(
            inbyte,
            static_cast<uint8_t>(bit_position + 1),
            jura_bit_read(rawbyte, 5));

        bit_position = static_cast<uint8_t>(bit_position + 2);

        if (bit_position >= 8) {
          bit_position = 0;

          inbytes.push_back(static_cast<char>(inbyte));

          inbyte = 0;
        }

      } else {
        delay(10);
      }

      if (timeout_counter++ > 1500) {
        ESP_LOGW("jura", "Timeout waiting for Jura response");
        return "";
      }
    }

    // Strip CR/LF
    return inbytes.substr(0, inbytes.size() - 2);
  }


  void update() override {
    // -----------------------------------------------------------------------
    // Counters
    // -----------------------------------------------------------------------
    std::string result = cmd2jura("RT:0000");

    if (result.empty() || result.size() < 64) {
      ESP_LOGW(
          "jura",
          "Unexpected RT:0000 response len=%d",
          static_cast<int>(result.size()));

      return;
    }

    std::vector<long> current = parse_all_counters_(result);

    publish_number("counter_1",  get_counter_n_(current, 1));
    publish_number("counter_2",  get_counter_n_(current, 2));
    publish_number("counter_3",  get_counter_n_(current, 3));
    publish_number("counter_4",  get_counter_n_(current, 4));
    publish_number("counter_5",  get_counter_n_(current, 5));
    publish_number("counter_6",  get_counter_n_(current, 6));
    publish_number("counter_7",  get_counter_n_(current, 7));
    publish_number("counter_8",  get_counter_n_(current, 8));
    publish_number("counter_9",  get_counter_n_(current, 9));
    publish_number("counter_10", get_counter_n_(current, 10));
    publish_number("counter_11", get_counter_n_(current, 11));
    publish_number("counter_12", get_counter_n_(current, 12));
    publish_number("counter_13", get_counter_n_(current, 13));
    publish_number("counter_14", get_counter_n_(current, 14));
    publish_number("counter_15", get_counter_n_(current, 15));
    publish_number("counter_16", get_counter_n_(current, 16));

    publish_counter_changes_(current);


    // -----------------------------------------------------------------------
    // IC flags / machine state
    // -----------------------------------------------------------------------
    std::string ic = cmd2jura("IC:");

    if (ic.size() >= 7) {
      uint8_t a = static_cast<uint8_t>(
          std::strtol(ic.substr(3, 2).c_str(), nullptr, 16));

      uint8_t b = static_cast<uint8_t>(
          std::strtol(ic.substr(5, 2).c_str(), nullptr, 16));

      publish_ic_bits_if_changed_(a, b);

      uint8_t tray_bit =
          jura_bit_read(a, 4);

      uint8_t left_ready_bit =
          jura_bit_read(a, 2);

      uint8_t tank_bit =
          jura_bit_read(b, 5);

      uint8_t right_busy_bit =
          jura_bit_read(b, 6);


      std::string tray_status =
          (tray_bit == 1) ? "Present" : "Missing";

      std::string tank_status =
          (tank_bit == 1) ? "Fill Tank" : "OK";

      std::string machine_status = "Ready";


      if (tray_bit == 0) {
        machine_status = "Tray Missing";
      }

      if (tank_bit == 1) {
        machine_status = "Fill Tank";
      }

      if (right_busy_bit == 1) {
        machine_status = "Busy (Milk Drink)";
      }

      if (left_ready_bit == 0) {
        machine_status = "Busy (Coffee Drink)";
      }


      publish_text(
          "tray_status",
          tray_status);

      publish_text(
          "water_tank_status",
          tank_status);

      publish_text(
          "machine_status",
          machine_status);

    } else {
      ESP_LOGW(
          "jura",
          "Unexpected IC response len=%d",
          static_cast<int>(ic.size()));
    }
  }


 protected:
  // -------------------------------------------------------------------------
  // Counter helpers
  // -------------------------------------------------------------------------

  long get_counter_n_(const std::vector<long> &values, int n) const {
    if (n < 1) {
      return -1;
    }

    const size_t index = static_cast<size_t>(n - 1);

    if (index < values.size()) {
      return values[index];
    }

    return -1;
  }


  std::vector<long> parse_all_counters_(const std::string &rt) const {
    std::vector<long> values;

    // Counter fields contain four hex characters,
    // starting at position 3.
    for (size_t pos = 3;
         pos + 4 <= rt.size();
         pos += 4) {

      long value = std::strtol(
          rt.substr(pos, 4).c_str(),
          nullptr,
          16);

      values.push_back(value);
    }

    return values;
  }


  void publish_counter_changes_(const std::vector<long> &current) {
    if (!last_counters_initialized_) {
      last_counters_ = current;
      last_counters_initialized_ = true;

      return;
    }

    std::string message;
    bool any = false;

    const size_t max_count =
        std::max(last_counters_.size(), current.size());

    char buffer[48];


    for (size_t i = 0; i < max_count; ++i) {
      long previous =
          (i < last_counters_.size())
              ? last_counters_[i]
              : -1;

      long now =
          (i < current.size())
              ? current[i]
              : -1;


      if (previous != now) {
        if (any) {
          message += ", ";
        }

        snprintf(
            buffer,
            sizeof(buffer),
            "counter_%u %ld->%ld",
            static_cast<unsigned>(i + 1),
            previous,
            now);

        message += buffer;
        any = true;
      }
    }


    if (any) {
      publish_text(
          "counters_changed",
          message);

      ESP_LOGD(
          "jura",
          "Changed: %s",
          message.c_str());
    }


    last_counters_ = current;
  }


  // -------------------------------------------------------------------------
  // IC bit helpers
  // -------------------------------------------------------------------------

  static inline void byte_to_bits_(uint8_t value, char out[9]) {
    for (int i = 7; i >= 0; --i) {
      out[7 - i] =
          (value & (1U << i))
              ? '1'
              : '0';
    }

    out[8] = '\0';
  }


  void publish_ic_bits_if_changed_(uint8_t a, uint8_t b) {
    if (!ic_bits_initialized_ ||
        a != last_ic_a_ ||
        b != last_ic_b_) {

      char a_bits[9];
      char b_bits[9];
      char buffer[32];

      byte_to_bits_(a, a_bits);
      byte_to_bits_(b, b_bits);

      snprintf(
          buffer,
          sizeof(buffer),
          "A=%s B=%s",
          a_bits,
          b_bits);

      publish_text(
          "ic_bits",
          std::string(buffer));

      last_ic_a_ = a;
      last_ic_b_ = b;

      ic_bits_initialized_ = true;

      ESP_LOGD(
          "jura",
          "IC bits changed: %s",
          buffer);
    }
  }


  // -------------------------------------------------------------------------
  // Internal state
  // -------------------------------------------------------------------------

  std::string model_{"UNKNOWN"};

  std::map<std::string, sensor::Sensor *> numeric_;

  std::map<std::string, text_sensor::TextSensor *> text_;

  std::vector<long> last_counters_;

  bool last_counters_initialized_{false};

  uint8_t last_ic_a_{0};

  uint8_t last_ic_b_{0};

  bool ic_bits_initialized_{false};
};


}  // namespace jura
}  // namespace esphome
