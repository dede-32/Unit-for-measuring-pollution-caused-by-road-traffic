#pragma once

struct SensorData {
  float iaq = NAN;
  uint8_t iaqAccuracy = 0;
  float temperature = NAN;
  float humidity = NAN;
  float pressure = NAN;
  uint16_t bsec_co2 = 0;
  float bvoc = NAN;

  uint16_t scd41_co2 = 0;
  float scd41_temp = NAN;
  float scd41_rh = NAN;

  float dBC = NAN;

  float pm1_0 = NAN;
  float pm2_5 = NAN;
  float pm4_0 = NAN;
  float pm10  = NAN;
  float typical_size = NAN;

  float battery_percent = NAN;

  bool updated = false;
};
