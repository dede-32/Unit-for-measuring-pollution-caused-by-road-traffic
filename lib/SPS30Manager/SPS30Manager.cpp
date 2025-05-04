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

  unsigned long t_start = millis();

  if (sps30_probe() != 0) {
    Serial.println("SPS30: Sensor not found.");
    return;
  }

  if (sps30_start_measurement() != 0) {
    Serial.println("SPS30: Failed to start measurement.");
    return;
  }

  Serial.printf("SPS30: startMeasurement() took %lu ms\n", millis() - t_start);

  _startTime = millis();
  _started = true;
}

void SPS30Manager::finishMeasurement(SensorData& dest) {
  if (!_started) return;

  const unsigned long minRuntime = 10000UL;
  unsigned long elapsed = millis() - _startTime;
  if (elapsed < minRuntime) {
    Serial.printf("SPS30: Waiting %lu ms to reach min runtime\n", minRuntime - elapsed);
    delay(minRuntime - elapsed);
  }

  uint16_t data_ready = 0;
  const unsigned long timeout_ms = 5000;
  const unsigned long t_start = millis();

  while ((millis() - t_start) < timeout_ms) {
    if (sps30_read_data_ready(&data_ready) == 0 && data_ready) {
      break;
    }
    delay(200);
  }

  if (!data_ready) {
    Serial.println("SPS30: Data not ready in time.");
    sps30_stop_measurement();
    _pm->off("SPS30");
    digitalWrite(_enPin, LOW);
    _started = false;
    return;
  }

  struct sps30_measurement m;
  if (sps30_read_measurement(&m) == 0) {
    dest.pm1_0 = m.mc_1p0;
    dest.pm2_5 = m.mc_2p5;
    dest.pm4_0 = m.mc_4p0;
    dest.pm10  = m.mc_10p0;
    dest.typical_size = m.typical_particle_size;
    Serial.println("SPS30: Measurement successful.");
  } else {
    Serial.println("SPS30: Failed to read measurement.");
  }

  sps30_stop_measurement();
  _pm->off("SPS30");
  digitalWrite(_enPin, LOW);
  _started = false;

  Serial.printf("SPS30: finishMeasurement() completed in %lu ms\n", millis() - t_start);
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
