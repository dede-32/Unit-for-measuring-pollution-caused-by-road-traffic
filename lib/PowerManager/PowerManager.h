#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include <map>

class PowerManager {
  public:
    void addSensor(const String& name, uint8_t gpioPin, bool activeHigh = true);
    void on(const String& name);
    void off(const String& name);
    void offAll();
    bool isOn(const String& name);

  private:
    struct SensorControl {
        uint8_t pin;
        bool activeHigh;
        bool state;
    };

    std::map<String, SensorControl> sensors;
};

#endif
