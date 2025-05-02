#define HELTEC_WIRELESS_STICK_LITE
#include <heltec_unofficial.h>
#include <RadioLib.h>
#include <LoRaWAN_ESP32.h>
#include "PowerManager.h"
#include <bsec2.h>
// #include <Preferences.h>
#include "SensirionI2cScd4x.h"
#include "SoundMeter.h"
#include "sps30.h"


// ==== I2C piny (přizpůsob podle potřeby) ====
#define SDA_PIN 18
#define SCL_PIN 20

// ==== Globální proměnné ====
PowerManager pm;
// Preferences prefs;
Bsec2 bsec;
SensirionI2cScd4x scd4x;
SoundLevelMeter dmm4026;

// SX1262 radio = new Module(8, 14, 12, 13);
LoRaWANNode* node;


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

  bool updated = false;
};

SensorData sensorData;


void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
void checkBsecStatus(Bsec2 bsec);
void readSCD41();
void measureNoise();
void measureSPS30();
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
  pm.addSensor("EN", 19, false);

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

  // ==== Inicializace DMM4026 ====
  pm.on("DMM4026");
  delay(100);
  dmm4026.begin();
  pm.off("DMM4026");

  // ==== Inicializace SPS30 ====
  sensirion_i2c_init();

  // ==== Inicializace RADIO ====
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
  
  node->setDutyCycle(true, 1250); // FUP dodržení
  
  Serial.println("Setup complete. Waiting for first data...");
}

// ==== Loop ====
void loop() {

    if (!node || !node->isActivated()) {
      Serial.println("Session not active. Trying to rejoin.");
      node = persist.manage(&radio);
      if (!node->isActivated()) {
        Serial.println("Rejoin failed.");
        delay(10000);
        return;
      }
    }
    sensorData.updated = false;
  
    while (!sensorData.updated) {
      bsec.run();
      delay(100);
    }
  
    readSCD41();

    measureNoise();

    measureSPS30();

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
    Serial.printf("PM1.0: %.2f µg/m³\n", sensorData.pm1_0);
    Serial.printf("PM2.5: %.2f µg/m³\n", sensorData.pm2_5);
    Serial.printf("PM4.0: %.2f µg/m³\n", sensorData.pm4_0);
    Serial.printf("PM10 : %.2f µg/m³\n", sensorData.pm10);
    Serial.printf("Typical Particle Size: %.2f µm\n", sensorData.typical_size);
    Serial.println("--------------------------------------------------");

    Serial.println("Jump to sendLoRaWANData function.");
    sendLoRaWANData();
    delay(500);

    const unsigned long sleepTimeMs = 300000UL - 10000UL;
    persist.saveSession(node);
    Serial.print("Going to light sleep for ");
    Serial.print(sleepTimeMs / 1000);
    Serial.println(" seconds...\n");
    delay(100);

    esp_sleep_enable_timer_wakeup(sleepTimeMs * 1000ULL);
    esp_light_sleep_start();
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
  
    // // Wake up sensor from low-power mode
    // error = scd4x.wakeUp();
    // if (error != 0) {
    //     Serial.print("SCD41 wakeUp() failed: ");
    //     errorToString(error, errorMessage, sizeof(errorMessage));
    //     Serial.println(errorMessage);
    //     return;
    // }

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

    // // Power down sensor
    // error = scd4x.powerDown();
    // if (error != 0) {
    //     Serial.print("SCD41 powerDown() failed: ");
    //     errorToString(error, errorMessage, sizeof(errorMessage));
    //     Serial.println(errorMessage);
    // }
    
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

  void measureSPS30() {
    pm.on("SPS30");
    delay(500);
  
    if (sps30_probe() != 0) {
      Serial.println("SPS30: Sensor not found or not responding.");
      pm.off("SPS30");
      return;
    }
  
    if (sps30_start_measurement() != 0) {
      Serial.println("SPS30: Failed to start measurement.");
      pm.off("SPS30");
      return;
    }
    
    delay(10000);

    uint16_t data_ready = 0;
    const uint32_t start = millis();
    const uint32_t timeout_ms = 10000;  // maximálně čekej 10 sekund

    while ((millis() - start) < timeout_ms) {
        if (sps30_read_data_ready(&data_ready) == 0 && data_ready) {
            break;
        }
        delay(200);
    }
  
    if (!data_ready) {
      Serial.println("SPS30: Data not ready in time.");
      sps30_stop_measurement();
      pm.off("SPS30");
      return;
    }
  
    struct sps30_measurement m;
    if (sps30_read_measurement(&m) == 0) {
      sensorData.pm1_0 = m.mc_1p0;
      sensorData.pm2_5 = m.mc_2p5;
      sensorData.pm4_0 = m.mc_4p0;
      sensorData.pm10  = m.mc_10p0;
      sensorData.typical_size = m.typical_particle_size;
  
    } else {
      Serial.println("SPS30: Failed to read measurement.");
    }
  
    sps30_stop_measurement();
    pm.off("SPS30");
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
    addUint16(sensorData.bsec_co2);
    addUint16((uint16_t)(sensorData.bvoc * 100));
    addUint16(sensorData.scd41_co2);
    addUint8((int)(sensorData.scd41_temp + 40));
    addUint8((int)sensorData.scd41_rh);
    addUint8((int)sensorData.dBC);
    addUint16((uint16_t)(sensorData.pm1_0 * 10));
    addUint16((uint16_t)(sensorData.pm2_5 * 10));
    addUint16((uint16_t)(sensorData.pm4_0 * 10));
    addUint16((uint16_t)(sensorData.pm10 * 10));
    addUint8((uint8_t)(sensorData.typical_size * 10));

    uint8_t downlinkData[256];  // Případně menší, pokud nečekáš nic
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
  