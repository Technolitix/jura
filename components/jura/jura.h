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


// -----------------------------------------------------------------------------
// ESP-IDF-kompatibler Ersatz für Arduino bitRead()
// -----------------------------------------------------------------------------

static inline uint8_t jura_bit_read(uint8_t value, uint8_t bit) {
  return static_cast<uint8_t>(
      (value >> bit) & 0x01U
  );
}


// -----------------------------------------------------------------------------
// ESP-IDF-kompatibler Ersatz für Arduino bitWrite()
// -----------------------------------------------------------------------------

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
// Jura-Komponente
// =============================================================================

class Jura :
    public PollingComponent,
    public uart::UARTDevice {

 public:

  // ---------------------------------------------------------------------------
  // Modell
  // ---------------------------------------------------------------------------

  void set_model(const std::string &model) {
    model_ = model;
  }


  // ---------------------------------------------------------------------------
  // ESPHome Sensoren registrieren
  // ---------------------------------------------------------------------------

  void register_metric_sensor(
      const std::string &key,
      sensor::Sensor *sensor_ptr) {

    numeric_sensors_[key] = sensor_ptr;
  }


  void register_text_sensor(
      const std::string &key,
      text_sensor::TextSensor *sensor_ptr) {

    text_sensors_[key] = sensor_ptr;
  }


  // ---------------------------------------------------------------------------
  // Werte veröffentlichen
  // ---------------------------------------------------------------------------

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
    // Vorhandene Daten im RX Buffer verwerfen
    // -------------------------------------------------------------------------

    while (available()) {
      read();
    }


    // -------------------------------------------------------------------------
    // Jura erwartet CR/LF
    // -------------------------------------------------------------------------

    command += "\r\n";


    ESP_LOGD(
        TAG,
        "TX command: %s",
        command.c_str()
    );


    // -------------------------------------------------------------------------
    // Jura Daten codieren
    //
    // Ein logisches Byte wird als 4 UART-Bytes übertragen.
    //
    // Nutzdaten:
    //   UART Bit 2
    //   UART Bit 5
    //
    // Je physischem UART-Byte werden somit 2 Nutzbits übertragen.
    // -------------------------------------------------------------------------

    for (size_t index = 0;
         index < command.size();
         ++index) {

      const uint8_t source_byte =
          static_cast<uint8_t>(
              command[index]
          );


      for (uint8_t source_bit = 0;
           source_bit < 8;
           source_bit += 2) {

        uint8_t encoded_byte = 0xFF;


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


      // Jura benötigt eine Pause zwischen logischen Zeichen.
      delay(8);
    }


    // -------------------------------------------------------------------------
    // Antwort empfangen
    //
    // Wichtig:
    // Wir warten NICHT mehr ~5 Sekunden.
    //
    // Der alte Code hat dadurch den ESPHome Loop Watchdog ausgelöst.
    // -------------------------------------------------------------------------

    constexpr uint32_t RESPONSE_TIMEOUT_MS = 1000;

    const uint32_t response_start =
        millis();


    uint8_t decoded_byte = 0;
    uint8_t decoded_bit_position = 0;


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
      // Noch keine Daten
      // -----------------------------------------------------------------------

      if (!available()) {

        // CPU kurz freigeben.
        delay(1);

        continue;
      }


      // -----------------------------------------------------------------------
      // Physisches Jura UART Byte empfangen
      // -----------------------------------------------------------------------

      const uint8_t raw_byte =
          static_cast<uint8_t>(
              read()
          );


      // Nutzbit aus Position 2 übernehmen

      jura_bit_write(
          decoded_byte,
          decoded_bit_position,
          jura_bit_read(
              raw_byte,
              2
          )
      );


      // Nutzbit aus Position 5 übernehmen

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
      // Vier UART Bytes ergeben ein dekodiertes Byte
      // -----------------------------------------------------------------------

      if (decoded_bit_position >= 8) {

        decoded_bit_position = 0;


        response.push_back(
            static_cast<char>(
                decoded_byte
            )
        );


        decoded_byte = 0;


        // ---------------------------------------------------------------------
        // Jura Antwort endet auf CR/LF
        // ---------------------------------------------------------------------

        const size_t response_length =
            response.size();


        if (
            response_length >= 2 &&
            response[response_length - 2] == '\r' &&
            response[response_length - 1] == '\n'
        ) {

          response.resize(
              response_length - 2
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
  // ESPHome Polling
  // ===========================================================================

  void update() override {

    ESP_LOGD(
        TAG,
        "Polling Jura model %s",
        model_.c_str()
    );


    // -------------------------------------------------------------------------
    // Zähler auslesen
    // -------------------------------------------------------------------------

    const std::string counter_response =
        cmd2jura("RT:0000");


    // Wenn Jura nicht antwortet:
    // Update sofort verlassen.
    //
    // Dadurch kommt nicht direkt noch ein zweiter Timeout durch IC: hinzu.

    if (counter_response.empty()) {

      ESP_LOGW(
          TAG,
          "No response to RT:0000"
      );

      return;
    }


    ESP_LOGD(
        TAG,
        "RT:0000 raw response: %s",
        counter_response.c_str()
    );


    const std::vector<long> counters =
        parse_all_counters_(
            counter_response
        );


    // -------------------------------------------------------------------------
    // Alle möglichen Counter veröffentlichen
    //
    // __init__.py entscheidet, welche davon tatsächlich als ESPHome Sensor
    // existieren.
    // -------------------------------------------------------------------------

    publish_number(
        "counter_1",
        get_counter_n_(counters, 1)
    );

    publish_number(
        "counter_2",
        get_counter_n_(counters, 2)
    );

    publish_number(
        "counter_3",
        get_counter_n_(counters, 3)
    );

    publish_number(
        "counter_4",
        get_counter_n_(counters, 4)
    );

    publish_number(
        "counter_5",
        get_counter_n_(counters, 5)
    );

    publish_number(
        "counter_6",
        get_counter_n_(counters, 6)
    );

    publish_number(
        "counter_7",
        get_counter_n_(counters, 7)
    );

    publish_number(
        "counter_8",
        get_counter_n_(counters, 8)
    );

    publish_number(
        "counter_9",
        get_counter_n_(counters, 9)
    );

    publish_number(
        "counter_10",
        get_counter_n_(counters, 10)
    );

    publish_number(
        "counter_11",
        get_counter_n_(counters, 11)
    );

    publish_number(
        "counter_12",
        get_counter_n_(counters, 12)
    );

    publish_number(
        "counter_13",
        get_counter_n_(counters, 13)
    );

    publish_number(
        "counter_14",
        get_counter_n_(counters, 14)
    );

    publish_number(
        "counter_15",
        get_counter_n_(counters, 15)
    );

    publish_number(
        "counter_16",
        get_counter_n_(counters, 16)
    );


    publish_counter_changes_(
        counters
    );


    // -------------------------------------------------------------------------
    // Maschinenstatus auslesen
    // -------------------------------------------------------------------------




    if (ic_response.empty()) {

      ESP_LOGW(
          TAG,
          "No response to IC:"
      );

      return;
    }


    ESP_LOGD(
        TAG,
        "IC raw response: %s",
        ic_response.c_str()
    );


    // Erwartet wird mindestens:
    //
    // IC:XXXX
    //
    // Wir brauchen Zeichen 3..6.

    if (ic_response.size() < 7) {

      ESP_LOGW(
          TAG,
          "IC response too short: %u",
          static_cast<unsigned>(
              ic_response.size()
          )
      );

      return;
    }


    // -------------------------------------------------------------------------
    // Zwei Statusbytes parsen
    // -------------------------------------------------------------------------

    const uint8_t status_a =
        static_cast<uint8_t>(
            std::strtoul(
                ic_response.substr(
                    3,
                    2
                ).c_str(),
                nullptr,
                16
            )
        );


    const uint8_t status_b =
        static_cast<uint8_t>(
            std::strtoul(
                ic_response.substr(
                    5,
                    2
                ).c_str(),
                nullptr,
                16
            )
        );


    publish_ic_bits_if_changed_(
        status_a,
        status_b
    );


    // -------------------------------------------------------------------------
    // Momentan bekannte Bits
    //
    // Achtung:
    // Diese Zuordnung stammt von anderen Jura-Modellen.
    // Bei der E75 müssen wir sie später anhand echter Daten verifizieren.
    // -------------------------------------------------------------------------

    const uint8_t tray_bit =
        jura_bit_read(
            status_a,
            4
        );


    const uint8_t coffee_ready_bit =
        jura_bit_read(
            status_a,
            2
        );


    const uint8_t tank_bit =
        jura_bit_read(
            status_b,
            5
        );


    const uint8_t milk_busy_bit =
        jura_bit_read(
            status_b,
            6
        );


    // -------------------------------------------------------------------------
    // Textstatus
    // -------------------------------------------------------------------------

    const std::string tray_status =
        tray_bit
            ? "Present"
            : "Missing";


    const std::string tank_status =
        tank_bit
            ? "Fill Tank"
            : "OK";


    std::string machine_status =
        "Ready";


    if (!tray_bit) {
      machine_status =
          "Tray Missing";
    }


    if (tank_bit) {
      machine_status =
          "Fill Tank";
    }


    if (milk_busy_bit) {
      machine_status =
          "Busy (Milk Drink)";
    }


    if (!coffee_ready_bit) {
      machine_status =
          "Busy (Coffee Drink)";
    }


    publish_text(
        "tray_status",
        tray_status
    );


    publish_text(
        "water_tank_status",
        tank_status
    );


    publish_text(
        "machine_status",
        machine_status
    );
  }


 protected:

  // ===========================================================================
  // Counter auslesen
  // ===========================================================================

  long get_counter_n_(
      const std::vector<long> &values,
      int number) const {

    if (number <= 0) {
      return -1;
    }


    const size_t index =
        static_cast<size_t>(
            number - 1
        );


    if (index >= values.size()) {
      return -1;
    }


    return values[index];
  }


  // ===========================================================================
  // RT Antwort in 4-stellige Hex-Felder zerlegen
  // ===========================================================================

  std::vector<long> parse_all_counters_(
      const std::string &response) const {

    std::vector<long> result;


    // Jura Antworten beginnen typischerweise mit "RT:"
    if (response.size() <= 3) {
      return result;
    }


    for (
        size_t position = 3;
        position + 4 <= response.size();
        position += 4
    ) {

      const std::string field =
          response.substr(
              position,
              4
          );


      const long value =
          std::strtol(
              field.c_str(),
              nullptr,
              16
          );


      result.push_back(
          value
      );
    }


    return result;
  }


  // ===========================================================================
  // Änderungen der Counter erkennen
  // ===========================================================================

  void publish_counter_changes_(
      const std::vector<long> &current) {


    // Beim ersten Durchlauf nur Ausgangszustand speichern.

    if (!last_counters_initialized_) {

      last_counters_ =
          current;

      last_counters_initialized_ =
          true;

      return;
    }


    std::string changed_text;

    bool changed =
        false;


    const size_t count =
        std::max(
            last_counters_.size(),
            current.size()
        );


    for (size_t index = 0;
         index < count;
         ++index) {


      const long old_value =
          index < last_counters_.size()
              ? last_counters_[index]
              : -1;


      const long new_value =
          index < current.size()
              ? current[index]
              : -1;


      if (old_value == new_value) {
        continue;
      }


      if (changed) {
        changed_text += ", ";
      }


      char buffer[64];


      std::snprintf(
          buffer,
          sizeof(buffer),
          "counter_%u %ld->%ld",
          static_cast<unsigned>(
              index + 1
          ),
          old_value,
          new_value
      );


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


      ESP_LOGD(
          TAG,
          "Counter changes: %s",
          changed_text.c_str()
      );
    }


    last_counters_ =
        current;
  }


  // ===========================================================================
  // Byte als Binärstring darstellen
  // ===========================================================================

  static void byte_to_bits_(
      uint8_t value,
      char output[9]) {

    for (uint8_t index = 0;
         index < 8;
         ++index) {

      const uint8_t bit =
          static_cast<uint8_t>(
              7 - index
          );


      output[index] =
          jura_bit_read(
              value,
              bit
          )
              ? '1'
              : '0';
    }


    output[8] =
        '\0';
  }


  // ===========================================================================
  // IC Statusbits nur bei Änderung publizieren
  // ===========================================================================

  void publish_ic_bits_if_changed_(
      uint8_t value_a,
      uint8_t value_b) {


    if (
        ic_bits_initialized_ &&
        value_a == last_ic_a_ &&
        value_b == last_ic_b_
    ) {

      return;
    }


    char bits_a[9];
    char bits_b[9];


    byte_to_bits_(
        value_a,
        bits_a
    );


    byte_to_bits_(
        value_b,
        bits_b
    );


    char buffer[32];


    std::snprintf(
        buffer,
        sizeof(buffer),
        "A=%s B=%s",
        bits_a,
        bits_b
    );


    publish_text(
        "ic_bits",
        std::string(buffer)
    );


    ESP_LOGD(
        TAG,
        "IC bits changed: %s",
        buffer
    );


    last_ic_a_ =
        value_a;

    last_ic_b_ =
        value_b;

    ic_bits_initialized_ =
        true;
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


  std::vector<long>
      last_counters_;


  bool
      last_counters_initialized_{
          false
      };


  uint8_t
      last_ic_a_{
          0
      };


  uint8_t
      last_ic_b_{
          0
      };


  bool
      ic_bits_initialized_{
          false
      };
};


}  // namespace jura
}  // namespace esphome
