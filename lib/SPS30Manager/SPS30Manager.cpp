#include "SPS30Manager.h"

extern float heltec_vbat();

void SPS30Manager::begin(PowerManager* pm, uint8_t enPin) {
  _pm = pm;
  _enPin = enPin;
  pinMode(_enPin, OUTPUT);
  digitalWrite(_enPin, LOW); // výchozí stav: step-up vypnutý
  sensirion_i2c_init();
}

void SPS30Manager::startMeasurement() {
  float vbat = heltec_vbat();
  bool batteryPowered = !isExternalPower(vbat);

  Serial.print("[SPS30Manager] Power source: ");
  Serial.println(batteryPowered ? "Battery (step-up ON)" : "External (5V direct)");

  _pm->on("SPS30");
  digitalWrite(_enPin, batteryPowered ? LOW : HIGH); // řízení EN pinu
  delay(500);

  if (sps30_probe() != 0) {
    Serial.println("SPS30: Sensor not found.");
    return;
  }

  if (sps30_start_measurement() != 0) {
    Serial.println("SPS30: Failed to start measurement.");
    return;
  }

  _startTime = millis();
  _started = true;
}

void SPS30Manager::finishMeasurement(SensorData& dest) {
  if (!_started) return;

  const int sampleCount = 30;
  float sum_pm1_0 = 0, sum_pm2_5 = 0, sum_pm4_0 = 0, sum_pm10 = 0, sum_size = 0;
  int successfulReads = 0;

  for (int i = 0; i < sampleCount; ++i) {
    uint16_t data_ready = 0;
    const unsigned long timeout_ms = 5000;
    const unsigned long t_start = millis();

    // Čekej až budou data ready
    while ((millis() - t_start) < timeout_ms) {
      if (sps30_read_data_ready(&data_ready) == 0 && data_ready) {
        break;
      }
      delay(100);
    }

    if (!data_ready) {
      Serial.printf("SPS30: Data not ready for sample %d.\n", i + 1);
      continue;
    }

    struct sps30_measurement m;
    if (sps30_read_measurement(&m) == 0) {
      sum_pm1_0 += m.mc_1p0;
      sum_pm2_5 += m.mc_2p5;
      sum_pm4_0 += m.mc_4p0;
      sum_pm10  += m.mc_10p0;
      sum_size  += m.typical_particle_size;
      ++successfulReads;
    } else {
      Serial.printf("SPS30: Failed to read sample %d.\n", i + 1);
    }

    delay(1000); // vzorky jsou připraveny cca každou sekundu
  }

  if (successfulReads > 0) {
    dest.pm1_0 = sum_pm1_0 / successfulReads;
    dest.pm2_5 = sum_pm2_5 / successfulReads;
    dest.pm4_0 = sum_pm4_0 / successfulReads;
    dest.pm10  = sum_pm10  / successfulReads;
    dest.typical_size = sum_size / successfulReads;

    // Výpočty pro velikostní frakce
    dest.pm1_0_2_5 = std::max(0.0f, dest.pm2_5 - dest.pm1_0);
    dest.pm2_5_4_0 = std::max(0.0f, dest.pm4_0 - dest.pm2_5);
    dest.pm4_0_10  = std::max(0.0f, dest.pm10  - dest.pm4_0);
    Serial.println("SPS30: Averaged measurement successful.");
  } else {
    Serial.println("SPS30: No valid samples collected.");
  }

  sps30_stop_measurement();
  _pm->off("SPS30");
  digitalWrite(_enPin, LOW);
  _started = false;
}

bool SPS30Manager::isExternalPower(float vbat) {
  static bool wasExternal = false;
  if (vbat > 4.15) {
    wasExternal = true;
  } else if (vbat < 4.00) {
    wasExternal = false;
  }
  return wasExternal;
}

unsigned long SPS30Manager::getStartTime() const {
  return _startTime;
}

void SPS30Manager::forceReadMeasurement(SensorData& dest) {
  struct sps30_measurement m;
  if (sps30_read_measurement(&m) == 0) {
    dest.pm1_0 = m.mc_1p0;
    dest.pm2_5 = m.mc_2p5;
    dest.pm4_0 = m.mc_4p0;
    dest.pm10  = m.mc_10p0;
    dest.typical_size = m.typical_particle_size;
    Serial.println("SPS30: Measurement successful (forced).");
  } else {
    Serial.println("SPS30: Failed to read measurement.");
  }

  sps30_stop_measurement();
  _pm->off("SPS30");
  digitalWrite(_enPin, LOW);
  _started = false;
}
