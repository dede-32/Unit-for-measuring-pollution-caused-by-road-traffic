#include <heltec_unofficial.h>
#include "PowerManager.h"
#include "SensirionI2cScd4x.h"
#include <RadioLib.h>
#include <LoRaWAN_ESP32.h>

#define MINIMUM_DELAY 300

#define SDA_PIN 18
#define SCL_PIN 20

PowerManager pm;
SensirionI2cScd4x scd4x;
TwoWire I2CBus = TwoWire(0);  // Custom I2C

void goToSleep(uint32_t desiredDelay = MINIMUM_DELAY);

void setup() {
  Serial.begin(115200);
  delay(1000);

  pm.addSensor("SCD41", 5, false);
  pm.addSensor("DMM4026", 6, false);
  pm.addSensor("SPS30", 7, false);
  pm.addSensor("BME688", 4, false);

  // Inicializace I2C na vlastních pinech
  I2CBus.begin(SDA_PIN, SCL_PIN);
  scd4x.begin(I2CBus, 0x62);  // Předání I2C sběrnice knihovně

}

void loop() {

  pm.on("SCD41");  // zapni napájení
  delay(100);      // stabilizace

  uint16_t co2;
  float temperature;
  float humidity;

  int16_t error = scd4x.measureAndReadSingleShot(co2, temperature, humidity);
  if (error) {
    Serial.print("Chyba měření: ");
    Serial.println(error);
  } else {
    Serial.printf("CO2: %u ppm | Teplota: %.2f °C | Vlhkost: %.1f %%\n",
                  co2, temperature, humidity);
    // tady později můžeš data odeslat přes LoRa
  }

  pm.off("SCD41");  // vypni napájení senzoru

  goToSleep();      // uspání ESP na několik minut

}

void goToSleep(uint32_t desiredDelay) {
  Serial.println("Going to deep sleep now");

  // Vyber největší z hodnot: desiredDelay vs. MINIMUM_DELAY
  uint32_t delaySec = max(desiredDelay, (uint32_t)MINIMUM_DELAY);
  Serial.printf("Sleep time: %u s\n", delaySec);
  delay(100);  // nechat doběhnout výpis

  esp_sleep_enable_timer_wakeup((uint64_t)delaySec * 1000000ULL);
  esp_deep_sleep_start();
}