#include <heltec_unofficial.h>
#include "PowerManager.h"
#include "SensorData.h"
#include <bsec2.h>
#include "SensirionI2cScd4x.h"
#include "SoundMeter.h"
#include "SPS30Manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"

#define SDA_PIN 18
#define SCL_PIN 20

// --- Globály ---
PowerManager pm;
SensorData sensorData;

Bsec2 bsec;
SensirionI2cScd4x scd4x;
SoundLevelMeter dmm4026;
SPS30Manager sps30;

// ---- Noise ring buffer ----
static float noiseBuf[60];
static volatile uint8_t noiseIdx = 0;
static volatile bool noiseFull = false;
static volatile bool noiseValid = false;

static TaskHandle_t noiseTaskHandle = nullptr;
static portMUX_TYPE noiseMux = portMUX_INITIALIZER_UNLOCKED;

// --- Timery (ms) ---
static uint32_t tBsec  = 0;
static uint32_t tSps   = 0;
static uint32_t tScd   = 0;
static uint32_t tBat   = 0;
static uint32_t tNoise = 0;
static uint32_t tPub   = 0;

static float lastPressureSent = NAN;

struct AvgF {
  double sum = 0;
  uint32_t n = 0;

  void add(float x) { if (isfinite(x)) { sum += x; n++; } }
  float mean() const { return (n ? (float)(sum / (double)n) : NAN); }
  void reset() { sum = 0; n = 0; }
};

struct AvgU16 {
  uint64_t sum = 0;
  uint32_t n = 0;

  void add(uint16_t x) { sum += x; n++; }
  uint16_t meanU16(uint16_t fallback = 0) const {
    return n ? (uint16_t)lround((double)sum / (double)n) : fallback;
  }
  void reset() { sum = 0; n = 0; }
};

// --- 1-minute accumulators ---
// BME/BSEC (primární T/RH/P + VOC)
static AvgF a_bme_t, a_bme_rh, a_bme_p, a_bvoc, a_co2eq;


// SCD41 (CO2 primární, T/RH jen volitelně)
static AvgU16 a_scd_co2;

// SPS30
static AvgF a_pm1, a_pm25, a_pm4, a_pm10, a_size;


// --- Forward decl ---
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
void checkBsecStatus(Bsec2 bsec);
void readScd41();
void updateScd41PressureIfNeeded();
float computeLAeq60();

bool initBsec();
bool initScd41();
bool initNoise();
bool initSps30();

void tickBsec(uint32_t now);
void tickSps30(uint32_t now);
void tickScd41(uint32_t now);
void tickBattery(uint32_t now);
void noiseTask(void* pv);
void publishEveryMinute(uint32_t now);

// ====================== SETUP ======================
void setup() {
  heltec_setup();
  Wire.begin(SDA_PIN, SCL_PIN);

  // Power rails
  pm.addSensor("SCD41",   5, true);
  pm.addSensor("SPS30",   7, true);
  pm.addSensor("BME688",  4, true);
  pm.addSensor("DMM4026", 6, true);

  pm.on("SCD41");
  pm.on("BME688");
  pm.on("DMM4026");
  pm.on("SPS30");

  Serial.println("Boot: ALWAYS-ON / NO-SLEEP mode");

  if (!initBsec())  Serial.println("BSEC init failed");
  if (!initScd41()) Serial.println("SCD41 init failed");
  if (!initNoise()) Serial.println("Noise init failed");
  if (!initSps30()) Serial.println("SPS30 init failed");

  // Reset publish timers so first publish happens after 60s (ne hned)
  uint32_t now = millis();
  tBsec = tSps = tScd = tBat = tNoise = tPub = now;

  Serial.println("Setup done.");
}

// ====================== LOOP ======================
void loop() {
  //heltec_loop();
  uint32_t now = millis();

  tickBsec(now);
  tickSps30(now);
  tickScd41(now);
  tickBattery(now);
  publishEveryMinute(now); // 1×/min serial

  // žádný sleep, žádné blokující čekání (kromě noise ticku)
}

// ====================== INIT ======================
bool initBsec() {
  delay(500); // po zapnutí napájení
if (!bsec.begin(BME68X_I2C_ADDR_LOW, Wire)) {
  Serial.println("BME688 not found");
  return false;
}

// LP, 3.3V, 3s, 28d (selectivity)
static const uint8_t bsec_config_sel_33v_3s_28d[] = {
#include "config/bme688/bme688_sel_33v_3s_28d/bsec_selectivity.txt"
};

if (!bsec.setConfig(bsec_config_sel_33v_3s_28d)) {
  Serial.printf("bsec.setConfig failed, status=%d\n", bsec.status);
  return false;
}

bsecSensor outputs[] = {
  BSEC_OUTPUT_IAQ,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  BSEC_OUTPUT_RAW_PRESSURE,
  BSEC_OUTPUT_CO2_EQUIVALENT,
  BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
};

if (!bsec.updateSubscription(outputs, ARRAY_LEN(outputs), BSEC_SAMPLE_RATE_LP)) {
  Serial.printf("updateSubscription(LP) failed, status=%d\n", bsec.status);
  return false;
}

bsec.attachCallback(newDataCallback);
return true;

}


bool initScd41() {
  scd4x.begin(Wire, 0x62);

  scd4x.stopPeriodicMeasurement();
  delay(200);
/*
  int16_t err = scd4x.performFactoryReset();
  if (err != 0) {
    Serial.print("Reset failed! Error: ");
    Serial.println(err);
  } else {
    Serial.println("SCD41 reset successful.");
  }
*/
  scd4x.startPeriodicMeasurement();
  return true;
}

bool initNoise() {
  delay(100);
  dmm4026.begin();

  xTaskCreatePinnedToCore(
    noiseTask,
    "noise",
    8192,           // stack (když crashne, dej 12288)
    nullptr,
    1,              // nízká priorita
    &noiseTaskHandle,
    0               // core 0 (můžeš dát 1)
  );

  return true;
}

bool initSps30() {
  // 19 = EN pin (uprav dle HW)
  sps30.begin(&pm, 19);

  // ideálně continuous start 1×
  sps30.startMeasurement();
  return true;
}

// ====================== TICKS ======================
void tickBsec(uint32_t now) {
  // volat často; callback plní sensorData.*
  if (now - tBsec >= 250) {
    tBsec = now;
    bsec.run();
  }
}

void tickSps30(uint32_t now) {
  if (now - tSps >= 1000) {
    tSps = now;
    if (sps30.readIfReady(sensorData)) {

     /* if (sensorData.pm10 > 10.0f || sensorData.pm10 < 0.0f) {
  Serial.printf("SPS30 OUTLIER raw: pm1=%.2f pm2.5=%.2f pm4=%.2f pm10=%.2f size=%.2f\n",
                sensorData.pm1_0, sensorData.pm2_5, sensorData.pm4_0,
                sensorData.pm10, sensorData.typical_size);
}*/
      // absolutní
      a_pm1.add(sensorData.pm1_0);
      a_pm25.add(sensorData.pm2_5);
      a_pm4.add(sensorData.pm4_0);
      a_pm10.add(sensorData.pm10);
      a_size.add(sensorData.typical_size);
    }
  }
}

void tickScd41(uint32_t now) {
  if (now - tScd >= 5000) {
    tScd = now;
    readScd41();
  }
}

void tickBattery(uint32_t now) {
  if (now - tBat >= 10000) {
    tBat = now;
    float vbat = heltec_vbat();
    sensorData.battery_percent = heltec_battery_percent(vbat);
  }
}

void noiseTask(void* pv) {
   vTaskDelay(pdMS_TO_TICKS(500)); // nech vše doběhnout po bootu

  for (;;) {
    float L = dmm4026.measureLeq(1); // 1 sekunda (blokuje task, ale ne loop)

    if (isfinite(L)) {
      // krátká kritická sekce – zápis do bufferu
      portENTER_CRITICAL(&noiseMux);
      noiseBuf[noiseIdx] = L;
      noiseIdx = (noiseIdx + 1) % 60;
      if (noiseIdx == 0) noiseFull = true;
      noiseValid = true;
      portEXIT_CRITICAL(&noiseMux);
    }

    // pokud measureLeq(1) opravdu trvá ~1 s, delay netřeba
    // ale když by vracel rychle, udrž rytmus:
    // vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void publishEveryMinute(uint32_t now) {
  if (now - tPub >= 60000) {
    tPub = now;

    updateScd41PressureIfNeeded();

    sensorData.dBC = computeLAeq60();

    sensorData.temperature = a_bme_t.mean();
    sensorData.humidity    = a_bme_rh.mean();
    sensorData.pressure    = a_bme_p.mean();
    sensorData.bvoc        = a_bvoc.mean();
    sensorData.bsec_co2    = (uint16_t)lround(a_co2eq.mean());

    sensorData.scd41_co2 = a_scd_co2.meanU16(sensorData.scd41_co2);

    sensorData.pm1_0 = a_pm1.mean();
    sensorData.pm2_5 = a_pm25.mean();
    sensorData.pm4_0 = a_pm4.mean();
    sensorData.pm10  = a_pm10.mean();
    sensorData.typical_size = a_size.mean();

    // Frakce počítané z minutových průměrů (konzistentní)
    sensorData.pm1_0_2_5 = std::max(0.0f, sensorData.pm2_5 - sensorData.pm1_0);
    sensorData.pm2_5_4_0 = std::max(0.0f, sensorData.pm4_0 - sensorData.pm2_5);
    sensorData.pm4_0_10  = std::max(0.0f, sensorData.pm10  - sensorData.pm4_0);


    Serial.println("--------------------------------------------------");
    Serial.printf("Battery: %.0f %%\n", sensorData.battery_percent);
    Serial.printf("IAQ: %.2f (acc: %d)\n", sensorData.iaq, sensorData.iaqAccuracy);
    Serial.printf("Temp: %.2f °C, RH: %.2f %%\n", sensorData.temperature, sensorData.humidity);
    Serial.printf("Pressure: %.2f hPa\n", sensorData.pressure);
    Serial.printf("CO2eq: %.0f ppm, bVOC: %.2f ppm\n", sensorData.bsec_co2, sensorData.bvoc);

    Serial.printf("SCD41 CO2: %u ppm, Temp: %.2f °C, RH: %.2f %%\n",
                  sensorData.scd41_co2, sensorData.scd41_temp, sensorData.scd41_rh);

    Serial.printf("Noise: %.2f dB(C)\n", sensorData.dBC);

    Serial.printf("PM1-2.5: %.2f, PM2.5-4: %.2f, PM4-10: %.2f, PM10: %.2f, Size: %.2f\n",
              sensorData.pm1_0_2_5, sensorData.pm2_5_4_0, sensorData.pm4_0_10,
              sensorData.pm10, sensorData.typical_size);

    a_bme_t.reset(); a_bme_rh.reset(); a_bme_p.reset();
    a_bvoc.reset(); a_co2eq.reset();

    a_scd_co2.reset();

    a_pm1.reset(); a_pm25.reset(); a_pm4.reset(); a_pm10.reset(); a_size.reset();
  }
}

// ====================== BSEC CALLBACK ======================
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData o = outputs.output[i];
    switch (o.sensor_id) {
      case BSEC_OUTPUT_IAQ: sensorData.iaq = o.signal; sensorData.iaqAccuracy = o.accuracy; break;
      
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        sensorData.temperature = o.signal;
        a_bme_t.add(o.signal);
        break;

      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        sensorData.humidity = o.signal;
        a_bme_rh.add(o.signal);
        break;

      case BSEC_OUTPUT_RAW_PRESSURE:
        sensorData.pressure = o.signal;
        a_bme_p.add(o.signal);
        break;

      case BSEC_OUTPUT_CO2_EQUIVALENT:
        sensorData.bsec_co2 = o.signal;
        a_co2eq.add(o.signal);
        break;

      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
        sensorData.bvoc = o.signal;
        a_bvoc.add(o.signal);
        break;
          }
        }
}

// ====================== SCD41 READ ======================
void readScd41() {
  uint16_t co2, error;
  char errorMessage[64];
  float temp, rh;
  char errMsg[64];
  bool ready = false;


  error = scd4x.getDataReadyStatus(ready);
  if (error != 0) {
    // volitelně: logovat občas
    return;
  }

  if (!ready) return;
  error = scd4x.readMeasurement(co2, temp, rh);
  if (error != 0) return;

  sensorData.scd41_co2 = co2;
  sensorData.scd41_temp = temp;
  sensorData.scd41_rh = rh;
  a_scd_co2.add(co2);
}

void updateScd41PressureIfNeeded() {
  if (isnan(sensorData.pressure)) return;

  if (isnan(lastPressureSent) ||
      fabs(sensorData.pressure - lastPressureSent) > 2.0f) { // > 2 hPa
    uint16_t amb = (uint16_t)(sensorData.pressure + 0.5f);
    scd4x.setAmbientPressureRaw(amb);
    lastPressureSent = sensorData.pressure;
  }
}

void checkBsecStatus(Bsec2 bsec) {
  if (bsec.status < BSEC_OK) { Serial.println("BSEC error: " + String(bsec.status)); while (true) {} }
  if (bsec.sensor.status < BME68X_OK) { Serial.println("BME68X error: " + String(bsec.sensor.status)); while (true) {} }
}

float computeLAeq60() {
  float local[60];
  bool full, valid;
  uint8_t idx;

  portENTER_CRITICAL(&noiseMux);
  full  = noiseFull;
  valid = noiseValid;
  idx   = noiseIdx;
  for (int i = 0; i < 60; i++) local[i] = noiseBuf[i];
  portEXIT_CRITICAL(&noiseMux);

  if (!valid) return NAN;

  int N = full ? 60 : idx;
  if (N <= 0) return NAN;

  double sum = 0.0;
  int used = 0;

  for (int i = 0; i < N; i++) {
    float L = local[i];
    if (!isfinite(L)) continue;
    sum += pow(10.0, (double)L / 10.0);
    used++;
  }

  if (used == 0) return NAN;

  sum /= (double)used;
  return (float)(10.0 * log10(sum));
}
