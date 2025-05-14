#pragma once

#include "PowerManager.h"
#include "sps30.h"
#include "SensorData.h"

class SPS30Manager {
public:
  void begin(PowerManager* pm, uint8_t enPin = 19);
  void startMeasurement();
  void finishMeasurement(SensorData& dest);
  unsigned long getStartTime() const;
  void forceReadMeasurement(SensorData& dest);

private:
  PowerManager* _pm = nullptr;
  uint8_t _enPin = 19;
  unsigned long _startTime = 0;
  bool _started = false;

  bool isExternalPower(float vbat);
};