#include "audio_frontend.h"
#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>
#include <string.h>

// ================== CẤU HÌNH PHẦN CỨNG I2S ==================
#define I2S_WS   15   // Chân Word Select (LRCL)
#define I2S_SD   13   // Chân Serial Data (DOUT)
#define I2S_SCK  2    // Chân Serial Clock (BCLK)
#define I2S_PORT I2S_NUM_0

// ================== CẤU HÌNH XỬ LÝ TÍN HIỆU (DSP) ==================
#define SAMPLE_RATE     16000
#define FRAME_LEN       512
#define HOP_LEN         256
#define FFT_BINS        (FRAME_LEN / 2 + 1)
#define NUM_MEL_FILTERS 26
#define MEL_LOW_HZ      0.0f
#define MEL_HIGH_HZ     (SAMPLE_RATE / 2.0f)

// ================== GIÁM SÁT PHẦN CỨNG MIC ==================
#define HW_FAULT_RAW_ENERGY_THRESHOLD   50.0f
#define HW_FAULT_CONSECUTIVE_HOPS       125

static int hwFaultConsecutiveCount = 0;
static volatile bool micHardwareFaultFlag = false;

// ================== DỮ LIỆU BỘ ĐỆM NỘI BỘ ==================
static AudioFeatureWindow_t windowBuffer[2]; // Double-buffer (Ping-Pong)
static int writeBufferIndex = 0;
static QueueHandle_t windowReadyQueue = NULL;

static int32_t i2sReadBuf[HOP_LEN];
static float filteredRing[FRAME_LEN];

static float dcOffset = 0.0f;
static const float dcAlpha = 0.001f;

static float hpAlpha = 0.97f;
static float hpPrevInput = 0.0f;
static float hpPrevOutput = 0.0f;

// ================== FFT & MEL FILTERBANK ==================
static float vReal[FRAME_LEN];
static float vImag[FRAME_LEN];
static ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FRAME_LEN, (float)SAMPLE_RATE);

static float melFilterbank[NUM_MEL_FILTERS][FFT_BINS];

// ================== CÁC HÀM XỬ LÝ NỘI BỘ (STATIC) ==================

static float hzToMel(float hz) {
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

static float melToHz(float mel) {
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

static void buildMelFilterbank(void) {
    float melLow = hzToMel(MEL_LOW_HZ);
    float melHigh = hzToMel(MEL_HIGH_HZ);

    float melPoints[NUM_MEL_FILTERS + 2];
    for (int i = 0; i < NUM_MEL_FILTERS + 2; i++) {
        melPoints[i] = melLow + (melHigh - melLow) * i / (NUM_MEL_FILTERS + 1);
    }

    int binIndices[NUM_MEL_FILTERS + 2];
    for (int i = 0; i < NUM_MEL_FILTERS + 2; i++) {
        float hz = melToHz(melPoints[i]);
        binIndices[i] = (int)floorf((FRAME_LEN + 1) * hz / SAMPLE_RATE);
    }

    memset(melFilterbank, 0, sizeof(melFilterbank));

    for (int m = 1; m <= NUM_MEL_FILTERS; m++) {
        int left = binIndices[m - 1];
        int center = binIndices[m];
        int right = binIndices[m + 1];

        for (int k = left; k < center; k++) {
            if (k >= 0 && k < FFT_BINS) {
                melFilterbank[m - 1][k] = (float)(k - left) / (float)(center - left);
            }
        }
        for (int k = center; k < right; k++) {
            if (k >= 0 && k < FFT_BINS) {
                melFilterbank[m - 1][k] = (float)(right - k) / (float)(right - center);
            }
        }
    }
}

static void computeMFCC(float mfccOut[NUM_MFCC]) {
    float melEnergy[NUM_MEL_FILTERS];
    for (int m = 0; m < NUM_MEL_FILTERS; m++) {
        float sum = 0.0f;
        for (int k = 0; k < FFT_BINS; k++) {
            float power = vReal[k] * vReal[k];
            sum += power * melFilterbank[m][k];
        }
        melEnergy[m] = sum;
    }

    float logMel[NUM_MEL_FILTERS];
    for (int m = 0; m < NUM_MEL_FILTERS; m++) {
        logMel[m] = logf(melEnergy[m] + 1e-6f);
    }

    for (int c = 0; c < NUM_MFCC; c++) {
        float sum = 0.0f;
        for (int m = 0; m < NUM_MEL_FILTERS; m++) {
            sum += logMel[m] * cosf(PI / NUM_MEL_FILTERS * (m + 0.5f) * c);
        }
        mfccOut[c] = sum;
    }
}

static void setupI2S(void) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = HOP_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    esp_err_t err;
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[LOI] i2s_driver_install that bai, ma loi = %d\n", err);
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[LOI] i2s_set_pin that bai, ma loi = %d\n", err);
    }

    i2s_zero_dma_buffer(I2S_PORT);
}

static inline float filterOneSample(int32_t rawSample32bitFrame) {
    int32_t rawSample = rawSample32bitFrame >> 8;
    float sample = (float)rawSample;

    dcOffset += dcAlpha * (sample - dcOffset);
    float sampleNoDC = sample - dcOffset;

    float hpOutput = hpAlpha * (hpPrevOutput + sampleNoDC - hpPrevInput);
    hpPrevInput = sampleNoDC;
    hpPrevOutput = hpOutput;

    return hpOutput;
}

static void audioTask(void *pvParameters) {
    int frameCounter = 0;
    memset(filteredRing, 0, sizeof(filteredRing));

    while (true) {
        size_t bytesRead = 0;
        esp_err_t err = i2s_read(I2S_PORT, i2sReadBuf, sizeof(i2sReadBuf),
                                  &bytesRead, portMAX_DELAY);

        int samplesRead = bytesRead / sizeof(int32_t);
        if (err != ESP_OK || samplesRead != HOP_LEN) {
            Serial.printf("[CANH BAO] i2s_read doc thieu mau: %d/%d, err=%d\n",
                          samplesRead, HOP_LEN, err);
        }

        // Giám sát phần cứng Mic
        float rawEnergy = 0.0f;
        for (int i = 0; i < samplesRead; i++) {
            float rawVal = (float)(i2sReadBuf[i] >> 8);
            rawEnergy += rawVal * rawVal;
        }
        if (samplesRead > 0) {
            rawEnergy /= (float)samplesRead;
        }

        if (rawEnergy < HW_FAULT_RAW_ENERGY_THRESHOLD) {
            hwFaultConsecutiveCount++;
            if (hwFaultConsecutiveCount >= HW_FAULT_CONSECUTIVE_HOPS && !micHardwareFaultFlag) {
                micHardwareFaultFlag = true;
                Serial.println("[LOI PHAN CUNG] Mic nghi ngo bi long/dut day - "
                                "khong co tin hieu thuc trong ~2 giay lien tiep.");
            }
        } else {
            hwFaultConsecutiveCount = 0;
            micHardwareFaultFlag = false;
        }

        // Tiền xử lý
        float newFiltered[HOP_LEN];
        for (int i = 0; i < samplesRead; i++) {
            newFiltered[i] = filterOneSample(i2sReadBuf[i]);
        }
        for (int i = samplesRead; i < HOP_LEN; i++) {
            newFiltered[i] = 0.0f;
        }

        // Ring buffer & sliding window
        memmove(filteredRing, filteredRing + HOP_LEN,
                (FRAME_LEN - HOP_LEN) * sizeof(float));
        memcpy(filteredRing + (FRAME_LEN - HOP_LEN), newFiltered,
               HOP_LEN * sizeof(float));

        memcpy(vReal, filteredRing, FRAME_LEN * sizeof(float));
        memset(vImag, 0, FRAME_LEN * sizeof(float));

        // FFT & MFCC
        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude();

        computeMFCC(windowBuffer[writeBufferIndex].mfccWindow[frameCounter]);

        if (frameCounter == 0) {
            windowBuffer[writeBufferIndex].timestampStartUs = esp_timer_get_time();
        }

        frameCounter++;

        if (frameCounter >= FRAMES_PER_WINDOW) {
            int readyIndex = writeBufferIndex;
            writeBufferIndex = 1 - writeBufferIndex;
            frameCounter = 0;

            if (windowReadyQueue != NULL) {
                xQueueOverwrite(windowReadyQueue, &readyIndex);
            }
        }
    }
}

// ================== IMPLEMENTATION CÁC API CÔNG KHAI ==================

void audioFrontend_init(void) {
    setupI2S();
    buildMelFilterbank();

    windowReadyQueue = xQueueCreate(1, sizeof(int));

    xTaskCreatePinnedToCore(
        audioTask,
        "AudioTask",
        8192,
        NULL,
        5,
        NULL,
        1
    );

    Serial.println("Mic + pipeline MFCC (FreeRTOS, sliding window) san sang.");
}

bool audioFeatures_getLatestWindow(AudioFeatureWindow_t *outWindow) {
    if (windowReadyQueue == NULL) return false;

    int readyIndex;
    if (xQueueReceive(windowReadyQueue, &readyIndex, 0) == pdTRUE) {
        memcpy(outWindow, &windowBuffer[readyIndex], sizeof(AudioFeatureWindow_t));
        return true;
    }
    return false;
}

bool audioFeatures_isHardwareFaulted(void) {
    return micHardwareFaultFlag;
}