#include <heltec_unofficial.h>
#include "PowerManager.h"
#include <bsec2.h>
#include <Preferences.h>
#include "SensirionI2cScd4x.h"


// ==== I2C piny (přizpůsob podle potřeby) ====
#define SDA_PIN 18
#define SCL_PIN 20

// ==== Globální proměnné ====
PowerManager pm;
Preferences prefs;
Bsec2 bsec;
SensirionI2cScd4x scd4x;


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



  bool updated = false;
};

SensorData sensorData;


void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
void checkBsecStatus(Bsec2 bsec);
void readSCD41();
bool waitForSCD41Data(uint16_t timeout_ms);

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  pm.disableWiFiAndBT();
  Wire.begin(SDA_PIN, SCL_PIN);

  // Register sensors and their power control pins
  pm.addSensor("SCD41", 5, true); // Always on for automatic self-calibration (ASC)
  pm.addSensor("SPS30", 7, false);
  pm.addSensor("BME688", 4, true); // Always on for AQI
  pm.addSensor("DMM4026", 6, false); 
  pm.addSensor("EN", 19, false);

  // ==== Načtení stavu BSEC ====
  prefs.begin("bsec", true);
  int stateLen = prefs.getBytesLength("iaqState");
  Serial.print("Stored state length in flash: ");
  Serial.println(stateLen);
  if (stateLen > 0 && stateLen <= BSEC_MAX_STATE_BLOB_SIZE) {
    uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
    prefs.getBytes("iaqState", state, stateLen);
    if (bsec.setState(state)) {
      Serial.println("BSEC state loaded from flash");
    } else {
      Serial.println("Failed to apply BSEC state");
    }
  } else {
    Serial.println("No BSEC state found in flash");
  }
  prefs.end();

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


  Serial.println("Setup complete. Waiting for first data...");
}

// ==== Loop ====
void loop() {
    sensorData.updated = false;
  
    while (!sensorData.updated) {
      bsec.run();
      delay(100);
    }
  
    readSCD41();



    Serial.println("----------------------BME688----------------------------");
    Serial.printf("IAQ: %.2f (accuracy: %d)\n", sensorData.iaq, sensorData.iaqAccuracy);
    Serial.printf("Temp: %.2f °C, Humidity: %.2f %%\n", sensorData.temperature, sensorData.humidity);
    Serial.printf("Pressure: %.2f hPa\n", sensorData.pressure);
    Serial.printf("CO₂eq: %.2u ppm, bVOC: %.2f ppm\n", sensorData.bsec_co2, sensorData.bvoc);
    Serial.println("----------------------SCD41----------------------------");
    Serial.printf("CO₂: %.2u ppm, Temp: %.2f °C, Humidity: %.2f %%\n", sensorData.scd41_co2, sensorData.scd41_temp, sensorData.scd41_rh);
    Serial.println("--------------------------------------------------");
  

    const unsigned long sleepTimeMs = 300000UL - 10000UL;
  
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
  
          // Uložení stavu při dosažení plné kalibrace
          static bool stateSaved = false;
          if (!stateSaved && sensorData.iaqAccuracy == 3) {
            uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
            uint32_t stateLen = bsec.getState(state);
            if (stateLen > 10 && stateLen <= BSEC_MAX_STATE_BLOB_SIZE) {
                prefs.begin("bsec", false);
                prefs.putBytes("iaqState", state, stateLen);
                prefs.end();
              }
  
            Serial.println("BSEC state saved to flash");
            stateSaved = true;
          }
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
  
    // Wake up sensor from low-power mode
    error = scd4x.wakeUp();
    if (error != 0) {
        Serial.print("SCD41 wakeUp() failed: ");
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
        return;
    }

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

    // Power down sensor
    error = scd4x.powerDown();
    if (error != 0) {
        Serial.print("SCD41 powerDown() failed: ");
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
    }
    
    // Uložení do struktury
    sensorData.scd41_co2 = co2;
    sensorData.scd41_temp = temp;
    sensorData.scd41_rh = rh;
}

// bool waitForSCD41Data(uint16_t timeout_ms) {
//     uint32_t start = millis();
//     bool ready = false;
  
//     while ((millis() - start) < timeout_ms) {
//       scd4x.getDataReadyStatus(ready);
//       if (ready) return true;
//       delay(100);
//     }
  
//     return false;
//   }


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