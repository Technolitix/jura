#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>


namespace esphome {
namespace jura {


static const char *const TAG = "jura";


// =============================================================================
// ESP-IDF Ersatz für Arduino bitRead / bitWrite
// =============================================================================

static inline uint8_t jura_bit_read(uint8_t value, uint8_t bit) {
  return static_cast<uint8_t>(
      (value >> bit) & 0x01U
  );
}


static inline void jura_bit_write(
    uint8_t &value,
    uint8_t bit,
    bool state) {

  const uint8_t mask =
      static_cast<uint8_t>(1U << bit);

  if (state) {
    value =
        static_cast<uint8_t>(
            value | mask
        );
  } else {
    value =
        static_cast<uint8_t>(
            value & static_cast<uint8_t>(~mask)
        );
  }
}


// =============================================================================
// Jura
// =============================================================================

class Jura :
    public PollingComponent,
    public uart::UARTDevice {

 public:

  // ===========================================================================
  // Modell
  // ===========================================================================

  void set_model(const std::string &model) {
    model_ = model;
  }


  // ===========================================================================
  // Sensorregistrierung
  // ===========================================================================

  void register_metric_sensor(
      const std::string &key,
      sensor::Sensor *sensor_ptr) {

    numeric_sensors_[key] =
        sensor_ptr;
  }


  void register_text_sensor(
      const std::string &key,
      text_sensor::TextSensor *sensor_ptr) {

    text_sensors_[key] =
        sensor_ptr;
  }


  // ===========================================================================
  // Werte veröffentlichen
  // ===========================================================================

  void publish_number(
      const std::string &key,
      float value) {

    auto it =
        numeric_sensors_.find(key);

    if (it == numeric_sensors_.end()) {
      return;
    }

    if (it->second == nullptr) {
      return;
    }

    it->second->publish_state(value);
  }


  void publish_text(
      const std::string &key,
      const std::string &value) {

    auto it =
        text_sensors_.find(key);

    if (it == text_sensors_.end()) {
      return;
    }

    if (it->second == nullptr) {
      return;
    }

    it->second->publish_state(value);
  }


  // ===========================================================================
  // Jura UART Kommunikation
  // ===========================================================================

  std::string cmd2jura(std::string command) {

    std::string response;


    // -------------------------------------------------------------------------
    // Alte Daten aus RX Buffer entfernen
    // -------------------------------------------------------------------------

    while (available()) {
      read();
    }


    // -------------------------------------------------------------------------
    // Jura Kommandoabschluss
    // -------------------------------------------------------------------------

    command += "\r\n";


    ESP_LOGD(
        TAG,
        "TX command: %s",
        command.c_str()
    );


    // -------------------------------------------------------------------------
    // Jura Encoding
    //
    // Je logischem Byte werden 4 UART Bytes gesendet.
    //
    // Nutzbits:
    // Bit 2
    // Bit 5
    // -------------------------------------------------------------------------

    for (
        size_t index = 0;
        index < command.size();
        ++index
    ) {

      const uint8_t source_byte =
          static_cast<uint8_t>(
              command[index]
          );


      for (
          uint8_t source_bit = 0;
          source_bit < 8;
          source_bit += 2
      ) {

        uint8_t encoded_byte =
            0xFF;


        jura_bit_write(
            encoded_byte,
            2,
            jura_bit_read(
                source_byte,
                source_bit
            )
        );


        jura_bit_write(
            encoded_byte,
            5,
            jura_bit_read(
                source_byte,
                static_cast<uint8_t>(
                    source_bit + 1
                )
            )
        );


        write(encoded_byte);
      }


      // Jura benötigt Pause zwischen Zeichen
      delay(8);
    }


    // -------------------------------------------------------------------------
    // Antwort empfangen
    // -------------------------------------------------------------------------

    constexpr uint32_t RESPONSE_TIMEOUT_MS =
        1200;


    const uint32_t response_start =
        millis();


    uint8_t decoded_byte =
        0;


    uint8_t decoded_bit_position =
        0;


    while (true) {


      // -----------------------------------------------------------------------
      // Timeout
      // -----------------------------------------------------------------------

      if (
          static_cast<uint32_t>(
              millis() - response_start
          )
          >= RESPONSE_TIMEOUT_MS
      ) {

        ESP_LOGW(
            TAG,
            "Timeout waiting for Jura response after %u ms",
            static_cast<unsigned>(
                RESPONSE_TIMEOUT_MS
            )
        );

        return "";
      }


      // -----------------------------------------------------------------------
      // Noch keine UART Daten
      // -----------------------------------------------------------------------

      if (!available()) {

        delay(1);

        continue;
      }


      // -----------------------------------------------------------------------
      // Physisches UART Byte lesen
      // -----------------------------------------------------------------------

      const uint8_t raw_byte =
          static_cast<uint8_t>(
              read()
          );


      // Bit 2
      jura_bit_write(
          decoded_byte,
          decoded_bit_position,
          jura_bit_read(
              raw_byte,
              2
          )
      );


      // Bit 5
      jura_bit_write(
          decoded_byte,
          static_cast<uint8_t>(
              decoded_bit_position + 1
          ),
          jura_bit_read(
              raw_byte,
              5
          )
      );


      decoded_bit_position =
          static_cast<uint8_t>(
              decoded_bit_position + 2
          );


      // -----------------------------------------------------------------------
      // 4 UART Bytes ergeben ein logisches Byte
      // -----------------------------------------------------------------------

      if (decoded_bit_position >= 8) {

        decoded_bit_position =
            0;


        response.push_back(
            static_cast<char>(
                decoded_byte
            )
        );


        decoded_byte =
            0;


        // ---------------------------------------------------------------------
        // Antwort endet mit CR LF
        // ---------------------------------------------------------------------

        const size_t length =
            response.size();


        if (
            length >= 2 &&
            response[length - 2] == '\r' &&
            response[length - 1] == '\n'
        ) {

          response.resize(
              length - 2
          );


          ESP_LOGD(
              TAG,
              "RX response: %s",
              response.c_str()
          );


          return response;
        }
      }
    }
  }


  // ===========================================================================
  // Polling
  // ===========================================================================

  void update() override {

    ESP_LOGD(
        TAG,
        "Polling Jura model %s",
        model_.c_str()
    );


    // =======================================================================
    // RT:00
    //
    // Langzeitregister / Counter
    // =======================================================================

    const std::string rt_response =
        cmd2jura("RT:00");


    if (rt_response.empty()) {

      ESP_LOGW(
          TAG,
          "No response to RT:00"
      );

      // Wenn überhaupt keine Verbindung besteht,
      // nicht noch RR hinterher schicken.
      return;
    }


    ESP_LOGD(
        TAG,
        "RT:00 raw response: %s",
        rt_response.c_str()
    );


    publish_text(
        "rt_raw",
        rt_response
    );


    const std::vector<uint16_t> counters =
        parse_registers_(
            rt_response
        );


    publish_rt_registers_(
        counters
    );


    publish_counter_changes_(
        counters
    );


    // =======================================================================
    // RR:00
    //
    // Aktuelle Maschinenregister
    // =======================================================================

    const std::string rr_response =
        cmd2jura("RR:00");


    if (rr_response.empty()) {

      ESP_LOGW(
          TAG,
          "No response to RR:00"
      );

      return;
    }


    ESP_LOGD(
        TAG,
        "RR:00 raw response: %s",
        rr_response.c_str()
    );


    publish_text(
        "rr_raw",
        rr_response
    );


    const std::vector<uint16_t> rr_registers =
        parse_registers_(
            rr_response
        );


    publish_rr_registers_(
        rr_registers
    );


    publish_rr_changes_(
        rr_registers
    );
  }


 protected:

  // ===========================================================================
  // Antwort in 16-Bit Hexregister zerlegen
  //
  // Beispiel:
  //
  // rr:00020C00042900000019001900000052
  //
  // ->
  //
  // 0002
  // 0C00
  // 0429
  // 0000
  // 0019
  // 0019
  // 0000
  // 0052
  // ===========================================================================

  std::vector<uint16_t> parse_registers_(
      const std::string &response) const {

    std::vector<uint16_t> values;


    if (response.size() <= 3) {
      return values;
    }


    size_t start_position =
        0;


    // Normalerweise:
    //
    // rt:
    // rr:
    //
    // Prefix entfernen

    if (
        response.size() >= 3 &&
        response[2] == ':'
    ) {

      start_position =
          3;
    }


    for (
        size_t position = start_position;
        position + 4 <= response.size();
        position += 4
    ) {

      const std::string field =
          response.substr(
              position,
              4
          );


      const unsigned long value =
          std::strtoul(
              field.c_str(),
              nullptr,
              16
          );


      values.push_back(
          static_cast<uint16_t>(
              value
          )
      );
    }


    return values;
  }


  // ===========================================================================
  // RT Register veröffentlichen
  // ===========================================================================

  void publish_rt_registers_(
      const std::vector<uint16_t> &values) {


    // Bestehende Sensoren aus der ursprünglichen Komponente.
    //
    // Die Bezeichnungen der Getränke sind für die E70/E75
    // NOCH NICHT als korrekt anzusehen.
    //
    // Wir behalten die Keys vorerst bei, damit dein __init__.py
    // weiterhin funktioniert.


    publish_number(
        "counter_1",
        get_register_(
            values,
            1
        )
    );


    publish_number(
        "counter_2",
        get_register_(
            values,
            2
        )
    );


    publish_number(
        "counter_3",
        get_register_(
            values,
            3
        )
    );


    publish_number(
        "counter_4",
        get_register_(
            values,
            4
        )
    );


    publish_number(
        "counter_5",
        get_register_(
            values,
            5
        )
    );


    publish_number(
        "counter_6",
        get_register_(
            values,
            6
        )
    );


    publish_number(
        "counter_7",
        get_register_(
            values,
            7
        )
    );


    publish_number(
        "counter_8",
        get_register_(
            values,
            8
        )
    );


    publish_number(
        "counter_9",
        get_register_(
            values,
            9
        )
    );


    publish_number(
        "counter_10",
        get_register_(
            values,
            10
        )
    );


    publish_number(
        "counter_11",
        get_register_(
            values,
            11
        )
    );


    publish_number(
        "counter_12",
        get_register_(
            values,
            12
        )
    );


    publish_number(
        "counter_13",
        get_register_(
            values,
            13
        )
    );


    publish_number(
        "counter_14",
        get_register_(
            values,
            14
        )
    );


    publish_number(
        "counter_15",
        get_register_(
            values,
            15
        )
    );


    publish_number(
        "counter_16",
        get_register_(
            values,
            16
        )
    );
  }


  // ===========================================================================
  // RR Register veröffentlichen
  //
  // Falls später im __init__.py Sensoren rr_1 ... rr_8 angelegt werden,
  // werden sie automatisch hier befüllt.
  // ===========================================================================

  void publish_rr_registers_(
      const std::vector<uint16_t> &values) {


    for (
        size_t index = 0;
        index < values.size();
        ++index
    ) {

      const unsigned register_number =
          static_cast<unsigned>(
              index + 1
          );


      char key[16];


      std::snprintf(
          key,
          sizeof(key),
          "rr_%u",
          register_number
      );


      publish_number(
          key,
          static_cast<float>(
              values[index]
          )
      );
    }
  }


  // ===========================================================================
  // Einzelnes Register
  // ===========================================================================

  float get_register_(
      const std::vector<uint16_t> &values,
      unsigned number) const {


    if (number == 0) {
      return -1;
    }


    const size_t index =
        static_cast<size_t>(
            number - 1
        );


    if (index >= values.size()) {
      return -1;
    }


    return static_cast<float>(
        values[index]
    );
  }


  // ===========================================================================
  // RT Änderungen erkennen
  // ===========================================================================

  void publish_counter_changes_(
      const std::vector<uint16_t> &current) {


    // -------------------------------------------------------------------------
    // Erstes Lesen
    // -------------------------------------------------------------------------

    if (!last_rt_initialized_) {

      last_rt_registers_ =
          current;


      last_rt_initialized_ =
          true;


      ESP_LOGI(
          TAG,
          "Initial RT register dump:"
      );


      log_register_dump_(
          "RT",
          current
      );


      return;
    }


    // -------------------------------------------------------------------------
    // Änderungen
    // -------------------------------------------------------------------------

    std::string changed_text;


    bool changed =
        false;


    const size_t count =
        std::max(
            last_rt_registers_.size(),
            current.size()
        );


    for (
        size_t index = 0;
        index < count;
        ++index
    ) {

      const int32_t old_value =
          index < last_rt_registers_.size()
              ? last_rt_registers_[index]
              : -1;


      const int32_t new_value =
          index < current.size()
              ? current[index]
              : -1;


      if (old_value == new_value) {
        continue;
      }


      char buffer[96];


      std::snprintf(
          buffer,
          sizeof(buffer),
          "RT%u: %04lX (%ld) -> %04lX (%ld)",
          static_cast<unsigned>(
              index + 1
          ),
          static_cast<unsigned long>(
              old_value & 0xFFFF
          ),
          static_cast<long>(
              old_value
          ),
          static_cast<unsigned long>(
              new_value & 0xFFFF
          ),
          static_cast<long>(
              new_value
          )
      );


      ESP_LOGI(
          TAG,
          "%s",
          buffer
      );


      if (changed) {
        changed_text += ", ";
      }


      changed_text +=
          buffer;


      changed =
          true;
    }


    if (changed) {

      publish_text(
          "counters_changed",
          changed_text
      );
    }


    last_rt_registers_ =
        current;
  }


  // ===========================================================================
  // RR Änderungen erkennen
  // ===========================================================================

  void publish_rr_changes_(
      const std::vector<uint16_t> &current) {


    // -------------------------------------------------------------------------
    // Erstes Lesen
    // -------------------------------------------------------------------------

    if (!last_rr_initialized_) {

      last_rr_registers_ =
          current;


      last_rr_initialized_ =
          true;


      ESP_LOGI(
          TAG,
          "Initial RR register dump:"
      );


      log_register_dump_(
          "RR",
          current
      );


      publish_rr_summary_(
          current
      );


      return;
    }


    // -------------------------------------------------------------------------
    // Änderungen
    // -------------------------------------------------------------------------

    std::string changed_text;


    bool changed =
        false;


    const size_t count =
        std::max(
            last_rr_registers_.size(),
            current.size()
        );


    for (
        size_t index = 0;
        index < count;
        ++index
    ) {

      const int32_t old_value =
          index < last_rr_registers_.size()
              ? last_rr_registers_[index]
              : -1;


      const int32_t new_value =
          index < current.size()
              ? current[index]
              : -1;


      if (old_value == new_value) {
        continue;
      }


      char buffer[96];


      std::snprintf(
          buffer,
          sizeof(buffer),
          "RR%u: %04lX (%ld) -> %04lX (%ld)",
          static_cast<unsigned>(
              index + 1
          ),
          static_cast<unsigned long>(
              old_value & 0xFFFF
          ),
          static_cast<long>(
              old_value
          ),
          static_cast<unsigned long>(
              new_value & 0xFFFF
          ),
          static_cast<long>(
              new_value
          )
      );


      // Das ist die wichtigste Meldung fürs Reverse Engineering.
      ESP_LOGW(
          TAG,
          "RR CHANGE >>> %s",
          buffer
      );


      if (changed) {
        changed_text += ", ";
      }


      changed_text +=
          buffer;


      changed =
          true;
    }


    // -------------------------------------------------------------------------
    // Nur wenn sich etwas geändert hat
    // -------------------------------------------------------------------------

    if (changed) {

      publish_text(
          "rr_changed",
          changed_text
      );


      publish_rr_summary_(
          current
      );


      ESP_LOGI(
          TAG,
          "Current RR register dump:"
      );


      log_register_dump_(
          "RR",
          current
      );
    }


    last_rr_registers_ =
        current;
  }


  // ===========================================================================
  // Register Dump
  // ===========================================================================

  void log_register_dump_(
      const char *prefix,
      const std::vector<uint16_t> &values) const {


    for (
        size_t index = 0;
        index < values.size();
        ++index
    ) {

      ESP_LOGI(
          TAG,
          "%s%u = 0x%04X = %u",
          prefix,
          static_cast<unsigned>(
              index + 1
          ),
          static_cast<unsigned>(
              values[index]
          ),
          static_cast<unsigned>(
              values[index]
          )
      );
    }
  }


  // ===========================================================================
  // RR Zusammenfassung als Text
  // ===========================================================================

  void publish_rr_summary_(
      const std::vector<uint16_t> &values) {


    std::string summary;


    for (
        size_t index = 0;
        index < values.size();
        ++index
    ) {

      if (index > 0) {
        summary += " ";
      }


      char buffer[32];


      std::snprintf(
          buffer,
          sizeof(buffer),
          "RR%u=%04X",
          static_cast<unsigned>(
              index + 1
          ),
          static_cast<unsigned>(
              values[index]
          )
      );


      summary +=
          buffer;
    }


    publish_text(
        "rr_registers",
        summary
    );


    ESP_LOGD(
        TAG,
        "RR registers: %s",
        summary.c_str()
    );
  }


  // ===========================================================================
  // Interne Variablen
  // ===========================================================================

  std::string model_{
      "UNKNOWN"
  };


  std::map<
      std::string,
      sensor::Sensor *
  > numeric_sensors_;


  std::map<
      std::string,
      text_sensor::TextSensor *
  > text_sensors_;


  // RT
  std::vector<uint16_t>
      last_rt_registers_;


  bool
      last_rt_initialized_{
          false
      };


  // RR
  std::vector<uint16_t>
      last_rr_registers_;


  bool
      last_rr_initialized_{
          false
      };
};


}  // namespace jura
}  // namespace esphome
