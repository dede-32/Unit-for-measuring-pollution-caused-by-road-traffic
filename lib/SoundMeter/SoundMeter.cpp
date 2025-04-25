#include "SoundMeter.h"
#include "sos-iir-filter.h"
#include <driver/i2s.h>
#include <cmath>

// ==== Konfigurace ====
#define MIC_BITS        24
#define MIC_OFFSET_DB   3.0103
#define MIC_SENSITIVITY -26
#define MIC_REF_DB      94.0
#define MIC_OVERLOAD_DB 123.0
#define MIC_NOISE_DB    30
#define MIC_CONVERT(s)  ((s) >> (SAMPLE_BITS - MIC_BITS))

#define SAMPLE_RATE     48000
#define SAMPLE_BITS     32
#define SAMPLES_SHORT   (SAMPLE_RATE / 8)

#define I2S_WS          17
#define I2S_SCK         34
#define I2S_SD          21
#define I2S_PORT        I2S_NUM_0

// ==== Filtrování ====
static SOS_Coefficients dc_blocker_sos[] = { { -1.0f, 0.0f, 0.9992f, 0.0f } };
static SOS_IIR_Filter DC_BLOCKER(1.0f, dc_blocker_sos);

static SOS_Coefficients dmm4026_sos[] = {
    { +0.3292655f, +0.8943116f, -1.2642873f, -0.8291328f },
    { -1.999979f, +0.999993f, +1.999980f, -0.999993f }
};
static SOS_IIR_Filter DMM4026(1.3944033f, dmm4026_sos);

static SOS_Coefficients c_weighting_sos[] = {
    { +1.4604386f, +0.5275070f, +1.9946145f, -0.9946217f },
    { +0.2376222f, +0.0140411f, -1.3396586f, -0.4421458f },
    { -2.0000000f, +1.0000000f, +0.3775800f, -0.0356366f }
};
static SOS_IIR_Filter C_weighting(0.169995f, c_weighting_sos);

// ==== Statické buffery ====
static int32_t i2s_raw[SAMPLES_SHORT];
static float samples[SAMPLES_SHORT];

// ==== Výpočet referenční amplitudy ====
static const double MIC_REF_AMPL = pow(10.0, double(MIC_SENSITIVITY) / 20.0) * ((1 << (MIC_BITS - 1)) - 1);

// ==== Inicializace I2S ====
void SoundLevelMeter::begin() {
    i2s_config_t config = {
        .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = true
    };

    i2s_pin_config_t pins = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    i2s_driver_install(I2S_PORT, &config, 0, nullptr);
    i2s_set_pin(I2S_PORT, &pins);
}

// ==== Hlavní synchronní měření ====
float SoundLevelMeter::measureLeq(uint16_t seconds) {
    uint32_t total_samples = seconds * SAMPLE_RATE;
    double Leq_sum = 0;

    while (total_samples > 0) {
        size_t bytes_read = 0;
        i2s_read(I2S_PORT, i2s_raw, sizeof(i2s_raw), &bytes_read, portMAX_DELAY);

        for (int i = 0; i < SAMPLES_SHORT; ++i) {
            samples[i] = MIC_CONVERT(i2s_raw[i]);
        }

        DC_BLOCKER.filter(samples, samples, SAMPLES_SHORT);
        DMM4026.filter(samples, samples, SAMPLES_SHORT);
        Leq_sum += C_weighting.filter(samples, samples, SAMPLES_SHORT);

        total_samples -= SAMPLES_SHORT;
    }

    double Leq_rms = sqrt(Leq_sum / (seconds * SAMPLE_RATE));
    double dB = MIC_OFFSET_DB + MIC_REF_DB + 20.0 * log10(Leq_rms / MIC_REF_AMPL);
    return float(dB);
}
