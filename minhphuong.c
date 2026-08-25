/*
 * MODULE: Thu thập & trích xuất đặc trưng âm thanh (Audio Front-end)
 * ESP32-S3 + Mic INMP441 (I2S). Đọc audio -> MFCC -> đưa cho Multimodal Fusion.
 *
 * - Đọc I2S liên tục trong 1 FreeRTOS task riêng.
 * - Tiền xử lý: bỏ DC offset, pre-emphasis (lọc thông cao).
 * - Sliding window: Frame 512 mẫu (32ms), Hop 256 mẫu (16ms), overlap 50%.
 * - FFT -> Mel filterbank (26 dải) -> log -> DCT -> 13 hệ số MFCC/khung.
 * - Gộp 62 khung (~1s) thành feature window [62 x 13], có timestamp.
 * - API để Fusion lấy dữ liệu ra + kiểm tra lỗi phần cứng mic (mất tín hiệu).
 */

#include <driver/i2s.h>
#include <arduinoFFT.h>

// ================== CẤU HÌNH PHẦN CỨNG ==================
#define I2S_WS   15   // Word Select (LRCL)
#define I2S_SD   13   // Serial Data
#define I2S_SCK  2    // Serial Clock (BCLK)
#define I2S_PORT      I2S_NUM_0

// ================== CẤU HÌNH XỬ LÝ TÍN HIỆU ==================
#define SAMPLE_RATE     16000   // đủ cho giọng nói/tiếng động (Nyquist < 8kHz)
#define FRAME_LEN       512     // 32ms/khung, lũy thừa 2 để FFT nhanh
#define HOP_LEN         256     // 16ms, overlap 50% -> 62.5 khung/giây
#define FFT_BINS        (FRAME_LEN / 2 + 1)   // nửa phổ + 1
#define NUM_MEL_FILTERS 26      // số dải lọc Mel
#define NUM_MFCC        13      // số hệ số DCT giữ lại
#define MEL_LOW_HZ      0.0f
#define MEL_HIGH_HZ     (SAMPLE_RATE / 2.0f)

// ================== FEATURE WINDOW ==================
#define FRAMES_PER_WINDOW   62
// Gộp 62 khung (~1 giây âm thanh thực) thành ma trận [62 x 13] cho Frozen
// Feature Extractor -> Trainable Classifier. Double-buffer ~6.3KB SRAM.

// ================== GIÁM SÁT PHẦN CỨNG MIC ==================
// i2s_read() OK không đảm bảo tín hiệu thật (dây lỏng/đứt vẫn đọc "thành
// công" nhưng toàn 0). Nên đo năng lượng raw (trước lọc DC/pre-emphasis).
#define HW_FAULT_RAW_ENERGY_THRESHOLD   50.0f   // ngưỡng năng lượng, cần hiệu chỉnh thực tế
#define HW_FAULT_CONSECUTIVE_HOPS       125     // ~2s liên tục mới báo lỗi thật

static int hwFaultConsecutiveCount = 0;
static volatile bool micHardwareFaultFlag = false;

// ================== DỮ LIỆU XUẤT CHO FUSION ==================
typedef struct {
    float mfccWindow[FRAMES_PER_WINDOW][NUM_MFCC];
    int64_t timestampStartUs;   // để đồng bộ với luồng IMU
} AudioFeatureWindow_t;

static AudioFeatureWindow_t windowBuffer[2]; // double-buffer ping-pong
static int writeBufferIndex = 0;

static QueueHandle_t windowReadyQueue = NULL; // chỉ truyền index buffer, không copy struct

// ================== BUFFER I2S THÔ ==================
static int32_t i2sReadBuf[HOP_LEN];

// ================== RING BUFFER TÍN HIỆU ĐÃ LỌC ==================
static float filteredRing[FRAME_LEN]; // lưu dữ liệu cũ để overlap 50% giữa 2 khung

float dcOffset = 0.0f;
const float dcAlpha = 0.001f; // hệ số lọc trung bình động DC

float hpAlpha = 0.97f;
float hpPrevInput = 0.0f;
float hpPrevOutput = 0.0f; // pre-emphasis 1 cực

// ================== FFT ==================
static float vReal[FRAME_LEN];
static float vImag[FRAME_LEN]; // dùng float thay double để tiết kiệm SRAM
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FRAME_LEN, (float)SAMPLE_RATE);

// ================== MEL FILTERBANK ==================
static float melFilterbank[NUM_MEL_FILTERS][FFT_BINS]; // tính 1 lần lúc khởi động

float hzToMel(float hz) {
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

float melToHz(float mel) {
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

void buildMelFilterbank() {
    float melLow = hzToMel(MEL_LOW_HZ);
    float melHigh = hzToMel(MEL_HIGH_HZ);

    // Chia đều thang Mel thành các điểm, đổi lại Hz -> chỉ số bin FFT
    float melPoints[NUM_MEL_FILTERS + 2];
    for (int i = 0; i < NUM_MEL_FILTERS + 2; i++) {
        melPoints[i] = melLow + (melHigh - melLow) * i / (NUM_MEL_FILTERS + 1);
    }

    int binIndices[NUM_MEL_FILTERS + 2];
    for (int i = 0; i < NUM_MEL_FILTERS + 2; i++) {
        float hz = melToHz(melPoints[i]);
        binIndices[i] = (int)floorf((FRAME_LEN + 1) * hz / SAMPLE_RATE);
    }

    // Dựng từng bộ lọc tam giác (left - center - right)
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

// ================== TÍNH MFCC CHO 1 KHUNG ==================
void computeMFCC(float mfccOut[NUM_MFCC]) {
    // Áp Mel filterbank lên phổ (vReal đang chứa magnitude)
    float melEnergy[NUM_MEL_FILTERS];
    for (int m = 0; m < NUM_MEL_FILTERS; m++) {
        float sum = 0.0f;
        for (int k = 0; k < FFT_BINS; k++) {
            float power = vReal[k] * vReal[k];
            sum += power * melFilterbank[m][k];
        }
        melEnergy[m] = sum;
    }

    // Log năng lượng
    float logMel[NUM_MEL_FILTERS];
    for (int m = 0; m < NUM_MEL_FILTERS; m++) {
        logMel[m] = logf(melEnergy[m] + 1e-6f);
    }

    // DCT-II -> hệ số MFCC
    for (int c = 0; c < NUM_MFCC; c++) {
        float sum = 0.0f;
        for (int m = 0; m < NUM_MEL_FILTERS; m++) {
            sum += logMel[m] * cosf(PI / NUM_MEL_FILTERS * (m + 0.5f) * c);
        }
        mfccOut[c] = sum;
    }
}

// ================== I2S SETUP ==================
void setupI2S() {
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

// ================== LỌC 1 MẪU (DC removal + pre-emphasis) ==================
static inline float filterOneSample(int32_t rawSample32bitFrame) {
    int32_t rawSample = rawSample32bitFrame >> 8; // lấy 24-bit thật của INMP441
    float sample = (float)rawSample;

    dcOffset += dcAlpha * (sample - dcOffset);
    float sampleNoDC = sample - dcOffset;

    float hpOutput = hpAlpha * (hpPrevOutput + sampleNoDC - hpPrevInput);
    hpPrevInput = sampleNoDC;
    hpPrevOutput = hpOutput;

    return hpOutput;
}

// ================== TASK CHÍNH XỬ LÝ ÂM THANH ==================
void audioTask(void *pvParameters) {
    int frameCounter = 0;

    memset(filteredRing, 0, sizeof(filteredRing));

    while (true) {
        // Đọc HOP_LEN mẫu mới từ mic
        size_t bytesRead = 0;
        esp_err_t err = i2s_read(I2S_PORT, i2sReadBuf, sizeof(i2sReadBuf),
                                  &bytesRead, portMAX_DELAY);

        int samplesRead = bytesRead / sizeof(int32_t);
        if (err != ESP_OK || samplesRead != HOP_LEN) {
            Serial.printf("[CANH BAO] i2s_read doc thieu mau: %d/%d, err=%d\n",
                          samplesRead, HOP_LEN, err);
        }

        // Đo năng lượng raw (trước lọc) để giám sát lỗi phần cứng
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

        // Lọc DC + pre-emphasis từng mẫu mới
        float newFiltered[HOP_LEN];
        for (int i = 0; i < samplesRead; i++) {
            newFiltered[i] = filterOneSample(i2sReadBuf[i]);
        }
        for (int i = samplesRead; i < HOP_LEN; i++) {
            newFiltered[i] = 0.0f;
        }

        // Trượt ring buffer (sliding window, overlap 50%)
        memmove(filteredRing, filteredRing + HOP_LEN,
                (FRAME_LEN - HOP_LEN) * sizeof(float));
        memcpy(filteredRing + (FRAME_LEN - HOP_LEN), newFiltered,
               HOP_LEN * sizeof(float));

        // Copy sang vReal để tính FFT (không phá dữ liệu gốc trong ring buffer)
        memcpy(vReal, filteredRing, FRAME_LEN * sizeof(float));
        memset(vImag, 0, FRAME_LEN * sizeof(float));

        // Windowing + FFT
        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude();

        // Tính MFCC cho khung hiện tại
        computeMFCC(windowBuffer[writeBufferIndex].mfccWindow[frameCounter]);

        if (frameCounter == 0) {
            windowBuffer[writeBufferIndex].timestampStartUs = esp_timer_get_time();
        }

        frameCounter++;

        // Đủ FRAMES_PER_WINDOW khung -> window hoàn chỉnh, báo cho Fusion
        if (frameCounter >= FRAMES_PER_WINDOW) {
            int readyIndex = writeBufferIndex;
            writeBufferIndex = 1 - writeBufferIndex; // đổi buffer (ping-pong)
            frameCounter = 0;

            if (windowReadyQueue != NULL) {
                xQueueOverwrite(windowReadyQueue, &readyIndex);
                // ưu tiên độ trễ thấp hơn giữ window cũ nếu Fusion chưa kịp đọc
            }
        }

        // Không delay() - tốc độ tự nhiên bị giới hạn bởi i2s_read() (~16ms)
    }
}

// ================== API CÔNG KHAI CHO MODULE FUSION ==================
bool audioFeatures_getLatestWindow(AudioFeatureWindow_t *outWindow) {
    if (windowReadyQueue == NULL) return false;

    int readyIndex;
    if (xQueueReceive(windowReadyQueue, &readyIndex, 0) == pdTRUE) {
        memcpy(outWindow, &windowBuffer[readyIndex], sizeof(AudioFeatureWindow_t));
        return true;
    }
    return false;
}

bool audioFeatures_isHardwareFaulted() {
    // true -> Fusion nên hạ trọng số nhánh audio, không coi im lặng là "Normal"
    return micHardwareFaultFlag;
}

// ================== SETUP / LOOP ==================
void setup() {
    Serial.begin(115200);
    delay(1000);

    setupI2S();
    buildMelFilterbank();

    windowReadyQueue = xQueueCreate(1, sizeof(int));

    // Task xử lý âm thanh riêng, ghim core 1 (core 0 dành cho WiFi/BLE)
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

void loop() {
    static AudioFeatureWindow_t latestWindow;
    if (audioFeatures_getLatestWindow(&latestWindow)) {
        Serial.printf("Window moi, t=%lld us, MFCC khung dau: ", latestWindow.timestampStartUs);
        for (int c = 0; c < NUM_MFCC; c++) {
            Serial.print(latestWindow.mfccWindow[0][c], 2);
            Serial.print(c < NUM_MFCC - 1 ? ", " : "\n");
        }
    }

    static bool lastFaultState = false;
    bool currentFaultState = audioFeatures_isHardwareFaulted();
    if (currentFaultState != lastFaultState) {
        Serial.printf("[TRANG THAI MIC] %s\n",
                      currentFaultState ? "LOI PHAN CUNG (mat tin hieu)"
                                         : "Da phuc hoi / binh thuong");
        lastFaultState = currentFaultState;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}