#include <heltec_unofficial.h>
#include "PowerManager.h"
#include <driver/i2s.h>

#define SAMPLE_RATE     48000
#define SAMPLE_BITS     32
#define I2S_PORT        I2S_NUM_0
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
#define MIC_VREF        0.0501f    // referenční hodnota RMS pro 94 dB SPL (nastavíš podle kalibrace)

PowerManager pm;

// Kompenzační filtr (4. řád)
float eq_B[] = {1.3944, -2.3296, 1.7232, -2.0349, 1.2470};
float eq_A[] = {1.0000, -0.7357, -0.6994, -0.3940, 0.8291};

// A-weighting filtr (2. řád)
float aweight_B[] = {0.1699, -0.3399, 0.1699};
float aweight_A[] = {1.0000, -1.7600, 0.8026};

// Filtrační paměti
float eq_x[5] = {}, eq_y[5] = {};
float aw_x[3] = {}, aw_y[3] = {};

float iir_filter(float input, float* x, float* y, float* b, float* a, int order) {
  for (int i = order; i > 0; i--) {
    x[i] = x[i-1];
    y[i] = y[i-1];
  }
  x[0] = input;
  y[0] = b[0]*x[0];
  for (int i = 1; i <= order; i++) {
    y[0] += b[i]*x[i] - a[i]*y[i];
  }
  return y[0];
}

void setupI2S() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_MIC_CHANNEL,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = 34,
    .ws_io_num = 17,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 21  // přizpůsob si podle zapojení
  };

  i2s_driver_install(I2S_PORT, &config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() {
  Serial.begin(115200);
  setupI2S();

  // Register sensors and their power control pins
  pm.addSensor("SCD41", 5, false);
  pm.addSensor("SPS30", 7, false);
  pm.addSensor("BME688", 4, true); // Always on for AQI
  pm.addSensor("DMM4026", 6, true); 
  pm.addSensor("EN", 19, false);
}

void loop() {
  // pm.on("DMM4026");
  // delay(100);  // stabilizace napájení

  // Nulování filtrační paměti
  memset(eq_x, 0, sizeof(eq_x));
  memset(eq_y, 0, sizeof(eq_y));
  memset(aw_x, 0, sizeof(aw_x));
  memset(aw_y, 0, sizeof(aw_y));


  const int durationSec = 5;
  const int totalSamples = SAMPLE_RATE * durationSec;
  size_t bytesRead;
  int32_t sample = 0;
  float sumSquares = 0;

  for (int i = 0; i < totalSamples; i++) {
    i2s_read(I2S_PORT, &sample, sizeof(sample), &bytesRead, portMAX_DELAY);
    float s = sample / 2147483648.0f;  // normalizace 32bit signed int na -1.0 až +1.0

    // Kompenzační IIR filtr
    float eq_out = iir_filter(s, eq_x, eq_y, eq_B, eq_A, 4);

    // A-weighting filtr
    float aw_out = iir_filter(eq_out, aw_x, aw_y, aweight_B, aweight_A, 2);

    sumSquares += aw_out * aw_out;
  }

  float rms = sqrt(sumSquares / totalSamples);
  float dB = 20.0 * log10(rms / MIC_VREF);

  Serial.print("SPL: ");
  Serial.print(dB, 1);
  Serial.println(" dB(A)");

  // pm.off("DMM4026");

  // delay(5 * 60 * 1000);  // spánek 5 minut
}
