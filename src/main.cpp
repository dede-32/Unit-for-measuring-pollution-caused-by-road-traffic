#include <heltec_unofficial.h>
#include <RadioLib.h>
#include <LoRaWAN_ESP32.h>
#include "PowerManager.h"
#include "SensorData.h"
#include <bsec2.h>
// #include <Preferences.h>
#include "SensirionI2cScd4x.h"
#include "SoundMeter.h"
#include "SPS30Manager.h"

// ==== I2C piny ====
#define SDA_PIN 18
#define SCL_PIN 20

const bool LORA_ENABLED = true;
static int64_t nextCycleUs = 0;

// ==== Globální proměnné ====
PowerManager pm;
SensorData sensorData;
// Preferences prefs;
Bsec2 bsec;
SensirionI2cScd4x scd4x;
SoundLevelMeter dmm4026;
SPS30Manager sps30;
LoRaWANNode* node;


void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
void checkBsecStatus(Bsec2 bsec);
void readSCD41();
void measureNoise();
void sendLoRaWANData();


// ==== Setup ====
void setup() {
  heltec_setup();
  pm.disableWiFiAndBT();
  Wire.begin(SDA_PIN, SCL_PIN);

  // Register sensors and their power control pins
  pm.addSensor("SCD41", 5, true); // Always on for automatic self-calibration (ASC)
  pm.addSensor("SPS30", 7, false);
  pm.addSensor("BME688", 4, true); // Always on for AQI
  pm.addSensor("DMM4026", 6, false); 



  // // ==== Načtení stavu BSEC ====
  // prefs.begin("bsec", true);
  // int stateLen = prefs.getBytesLength("iaqState");
  // Serial.print("Stored state length in flash: ");
  // Serial.println(stateLen);
  // if (stateLen > 0 && stateLen <= BSEC_MAX_STATE_BLOB_SIZE) {
  //   uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  //   prefs.getBytes("iaqState", state, stateLen);
  //   if (bsec.setState(state)) {
  //     Serial.println("BSEC state loaded from flash");
  //   } else {
  //     Serial.println("Failed to apply BSEC state");
  //   }
  // } else {
  //   Serial.println("No BSEC state found in flash");
  // }
  // prefs.end();

  // ==== Inicializace BME688 ====
  if (!bsec.begin(BME68X_I2C_ADDR_LOW, Wire)) {
    checkBsecStatus(bsec);
  }
  bsec.setTemperatureOffset(TEMP_OFFSET_ULP);

  bsecSensor outputs[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
  };

  if (!bsec.updateSubscription(outputs, ARRAY_LEN(outputs), BSEC_SAMPLE_RATE_ULP)) {
    checkBsecStatus(bsec);
  }
  bsec.attachCallback(newDataCallback);

  // ==== Inicializace SCD41 ====
  scd4x.begin(Wire, 0x62);
  int16_t err = scd4x.performFactoryReset();
  if (err != 0) {
    Serial.print("Reset failed! Error: ");
    Serial.println(err);
  } else {
    Serial.println("SCD41 reset successful.");
  }

  // ==== Inicializace DMM4026 ====
  pm.on("DMM4026");
  delay(100);
  dmm4026.begin();
  pm.off("DMM4026");

  // ==== Inicializace SPS30 ====
  sps30.begin(&pm, 19); // 19 je pin EN, uprav dle potřeby

  // ==== Inicializace RADIO ====
  if (LORA_ENABLED) {
    Serial.println("Radio init");
    int16_t state = radio.begin(868, 125.0, 12, 5, 0x34, 10, 8, 1.6, false);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.println("Radio did not initialize.");
    }
  
    node = persist.manage(&radio);
  
    if (!node->isActivated()) {
      Serial.println("Could not join network.");
    }
  
    persist.saveSession(node);
    node->setDutyCycle(true, 1250);
  }
  
  Serial.println("Setup complete. Waiting for first data...");
}

// ==== Loop ====
void loop() {
  const int64_t CYCLE_INTERVAL_US = 310000000LL;
  int64_t nowUs = esp_timer_get_time();

  if (nextCycleUs == 0) {
    nextCycleUs = nowUs + CYCLE_INTERVAL_US;
  }

  Serial.printf("Cycle started at: %lld s\n", nowUs / 1000000);

  float vbat = heltec_vbat();
  sensorData.battery_percent = heltec_battery_percent(vbat);

  sps30.startMeasurement();

  sensorData.updated = false;
  while (!sensorData.updated) {
    bsec.run();
    delay(100);
  }

  readSCD41();
  measureNoise();
  sps30.finishMeasurement(sensorData);

  Serial.println("----------------------Battery----------------------------");
  Serial.printf("Battery: %.0f %%\n", sensorData.battery_percent);
  Serial.println("----------------------BME688----------------------------");
  Serial.printf("IAQ: %.2f (accuracy: %d)\n", sensorData.iaq, sensorData.iaqAccuracy);
  Serial.printf("Temp: %.2f °C, Humidity: %.2f %%\n", sensorData.temperature, sensorData.humidity);
  Serial.printf("Pressure: %.2f hPa\n", sensorData.pressure);
  Serial.printf("CO₂eq: %.2u ppm, bVOC: %.2f ppm\n", sensorData.bsec_co2, sensorData.bvoc);
  Serial.println("----------------------SCD41----------------------------");
  Serial.printf("CO₂: %.2u ppm, Temp: %.2f °C, Humidity: %.2f %%\n", sensorData.scd41_co2, sensorData.scd41_temp, sensorData.scd41_rh);
  Serial.println("----------------------DMM4026----------------------------");
  Serial.printf("Noise: %.2f dB(C)\n", sensorData.dBC);
  Serial.println("----------------------SPS30----------------------------");
  Serial.printf("PM1.0:     %.2f µg/m³\n", sensorData.pm1_0);
  Serial.printf("PM2.5:     %.2f µg/m³\n", sensorData.pm1_0_2_5);
  Serial.printf("PM4.0:     %.2f µg/m³\n", sensorData.pm2_5_4_0);
  Serial.printf("PM10:      %.2f µg/m³\n", sensorData.pm4_0_10);
  Serial.printf("Typical Particle Size: %.2f µm\n", sensorData.typical_size);
  Serial.println("--------------------------------------------------");
  

  if (LORA_ENABLED) {
    for (int attempt = 1; attempt <= 1; attempt++) {
      if (!node || !node->isActivated()) {
        Serial.printf("Session not active. Rejoin attempt %d...\n", attempt);
        node = persist.manage(&radio);
        if (node && node->isActivated()) {
          break; // připojení úspěšné
        }
        delay(2000); // krátká pauza mezi pokusy
      }
    }
  
    if (node && node->isActivated()) {
      Serial.println("Jump to sendLoRaWANData function.");
      sendLoRaWANData();
      persist.saveSession(node);
    } else {
      Serial.println("LoRa rejoin failed. Skipping transmission.");
    }
  }

  int64_t afterUs = esp_timer_get_time();
  int64_t sleepUs = (nextCycleUs > afterUs) ? (nextCycleUs - afterUs) : 1000000;

  Serial.printf("Cycle finished at: %lld s\n", afterUs / 1000000);
  Serial.printf("Sleeping for: %lld seconds...\n\n", sleepUs / 1000000);

  delay(100);  // bezpečnostní rezerva
  esp_sleep_enable_timer_wakeup(sleepUs);
  esp_light_sleep_start();

  nextCycleUs += CYCLE_INTERVAL_US;  // Posuň se na další pevný interval
}




// ==== Callback z BSEC2 ====
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
    for (uint8_t i = 0; i < outputs.nOutputs; i++) {
      const bsecData o = outputs.output[i];
  
      switch (o.sensor_id) {
        case BSEC_OUTPUT_IAQ:
          sensorData.iaq = o.signal;
          sensorData.iaqAccuracy = o.accuracy;
  
          // // Uložení stavu při dosažení plné kalibrace
          // static bool stateSaved = false;
          // if (!stateSaved && sensorData.iaqAccuracy == 3) {
          //   uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
          //   uint32_t stateLen = bsec.getState(state);
          //   if (stateLen > 10 && stateLen <= BSEC_MAX_STATE_BLOB_SIZE) {
          //       prefs.begin("bsec", false);
          //       prefs.putBytes("iaqState", state, stateLen);
          //       prefs.end();
          //     }
  
          //   Serial.println("BSEC state saved to flash");
          //   stateSaved = true;
          // }
          break;
  
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
          sensorData.temperature = o.signal;
          break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
          sensorData.humidity = o.signal;
          break;
        case BSEC_OUTPUT_RAW_PRESSURE:
          sensorData.pressure = o.signal;
          break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
          sensorData.bsec_co2 = o.signal;
          break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
          sensorData.bvoc = o.signal;
          break;
      }
    }
  
    sensorData.updated = true;
  }

  void readSCD41() {
    uint16_t co2, error;
    float temp, rh;
    char errorMessage[64];

    error = scd4x.setAmbientPressureRaw((uint16_t)(sensorData.pressure + 0.5f));
    if (error != 0) {
      Serial.print("SCD41 setAmbientPressureRaw() failed: ");
      errorToString(error, errorMessage, sizeof(errorMessage));
      Serial.println(errorMessage);
      pm.off("SCD41");
      return;
    }

    // Ignoruj první měření po zapnutí napájení
    error = scd4x.measureSingleShot();
    if (error != 0) {
      Serial.print("SCD41 measureSingleShot() failed: ");
      errorToString(error, errorMessage, sizeof(errorMessage));
      Serial.println(errorMessage);
      pm.off("SCD41");
      return;
    }

    // Hlavní měření a čtení dat
    error = scd4x.measureAndReadSingleShot(co2, temp, rh);
    if (error != 0) {
      Serial.print("SCD41 measureAndReadSingleShot() failed: ");
      errorToString(error, errorMessage, sizeof(errorMessage));
      Serial.println(errorMessage);
      pm.off("SCD41");
      return;
    }
    
    // Uložení do struktury
    sensorData.scd41_co2 = co2;
    sensorData.scd41_temp = temp;
    sensorData.scd41_rh = rh;
}

  void measureNoise() {
    pm.on("DMM4026");
    delay(100);
    sensorData.dBC = dmm4026.measureLeq(5);
    pm.off("DMM4026");
  }

  void checkBsecStatus(Bsec2 bsec) {
    if (bsec.status < BSEC_OK) {
      Serial.println("BSEC error: " + String(bsec.status));
      while (true);
    }
    if (bsec.sensor.status < BME68X_OK) {
      Serial.println("BME68X error: " + String(bsec.sensor.status));
      while (true);
    }
  }

  void sendLoRaWANData() {
    if (!node || !node->isActivated()) {
      Serial.println("LoRaWAN node not ready.");
      return;
    }
  
    uint8_t payload[32];
    uint8_t j = 0;

    auto addUint8 = [&](uint8_t value) {
      if (j < sizeof(payload)) payload[j++] = value;
    };
  
    auto addUint16 = [&](uint16_t value) {
      if (j + 1 < sizeof(payload)) {
        payload[j++] = value >> 8;
        payload[j++] = value & 0xFF;
      }
    };
    
    addUint8(constrain((int)sensorData.iaq, 0, 500));
    addUint8(sensorData.iaqAccuracy);
    addUint8((int)(sensorData.temperature + 40));
    addUint8((int)sensorData.humidity);
    addUint16((uint16_t)(sensorData.pressure * 10));
    // addUint16(sensorData.bsec_co2);
    addUint16((uint16_t)(sensorData.bvoc * 100));
    addUint16(sensorData.scd41_co2);
    addUint8((int)sensorData.dBC);
    addUint16((uint16_t)(sensorData.pm1_0 * 10));
    addUint16((uint16_t)(sensorData.pm1_0_2_5 * 10));
    addUint16((uint16_t)(sensorData.pm2_5_4_0 * 10));
    addUint16((uint16_t)(sensorData.pm4_0_10 * 10));
    addUint8((uint8_t)(sensorData.typical_size * 10));

    // addUint8((int)(sensorData.scd41_temp + 40));
    // addUint8((int)sensorData.scd41_rh);

    uint8_t downlinkData[256];
    size_t lenDown = sizeof(downlinkData);
  
    int16_t state = node->sendReceive(payload, j, 1, downlinkData, &lenDown);
    if(state == RADIOLIB_ERR_NONE) {
      Serial.println("Message sent, no downlink received.");
    } else if (state > 0) {
      Serial.println("Message sent, downlink received.");
    } else {
      Serial.printf("sendReceive returned error %d, we'll try again later.\n", state);
    }
  }
  