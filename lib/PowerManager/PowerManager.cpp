#include "PowerManager.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>

void PowerManager::addSensor(const String& name, uint8_t gpioPin, bool activeHigh) {
    SensorControl s;
    s.pin = gpioPin;
    s.activeHigh = activeHigh;
    s.state = false;

    pinMode(gpioPin, OUTPUT);
    digitalWrite(gpioPin, activeHigh ? HIGH : LOW); // výchozí stav = fyzicky vypnuto

    sensors[name] = s;
}


void PowerManager::on(const String& name) {
    auto it = sensors.find(name);
    if (it != sensors.end()) {
        digitalWrite(it->second.pin, it->second.activeHigh ? LOW : HIGH);
        it->second.state = true;
    }
}

void PowerManager::off(const String& name) {
    auto it = sensors.find(name);
    if (it != sensors.end()) {
        digitalWrite(it->second.pin, it->second.activeHigh ? HIGH : LOW);
        it->second.state = false;
    }
}

void PowerManager::offAll() {
    for (auto& entry : sensors) {
        digitalWrite(entry.second.pin, entry.second.activeHigh ? HIGH : LOW);
        entry.second.state = false;
    }
}

bool PowerManager::isOn(const String& name) {
    auto it = sensors.find(name);
    return (it != sensors.end()) ? it->second.state : false;
}

void PowerManager::disableWiFiAndBT() {
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();
    btStop();
    esp_bt_controller_disable();
}