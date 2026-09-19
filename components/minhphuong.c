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

#include <driver/i2s.h>   // Thư viện ESP-IDF hỗ trợ giao tiếp chuẩn I2S phần cứng
#include <arduinoFFT.h>   // Thư viện tính toán FFT biến đổi Fourier nhanh

// ================== CẤU HÌNH PHẦN CỨNG ==================
#define I2S_WS   15   // Chân Word Select (LRCL - Left/Right Clock) chọn kênh âm thanh
#define I2S_SD   13   // Chân Serial Data (DOUT từ mic INMP441)
#define I2S_SCK  2    // Chân Serial Clock (BCLK - Bit Clock)
#define I2S_PORT      I2S_NUM_0 // Sử dụng bộ điều khiển phần cứng I2S cổng 0

// ================== CẤU HÌNH XỬ LÝ TÍN HIỆU ==================
#define SAMPLE_RATE     16000   // Tần số lấy mẫu 16kHz (đủ phủ dải tần giọng nói/tiếng động theo định lý Nyquist < 8kHz)
#define FRAME_LEN       512     // Kích thước khung 512 mẫu (~32ms ở 16kHz), là lũy thừa của 2 để tối ưu tốc độ FFT
#define HOP_LEN         256     // Bước trượt 256 mẫu (~16ms), tạo độ đè (overlap) 50% -> tương đương 62.5 khung/giây
#define FFT_BINS        (FRAME_LEN / 2 + 1)   // Số lượng bin tần số sau FFT (kết quả FFT thực có tính đối xứng: 512/2 + 1 = 257)
#define NUM_MEL_FILTERS 26      // Số lượng bộ lọc dạng tam giác trên thang đo Mel (Mel filterbank)
#define NUM_MFCC        13      // Số hệ số MFCC giữ lại sau biến đổi DCT (bỏ các hệ số bậc cao chứa nhiễu)
#define MEL_LOW_HZ      0.0f    // Tần số cắt thấp nhất của thang Mel (Hz)
#define MEL_HIGH_HZ     (SAMPLE_RATE / 2.0f) // Tần số cắt cao nhất của thang Mel (8000 Hz - giới hạn Nyquist)

// ================== FEATURE WINDOW ==================
#define FRAMES_PER_WINDOW   62
// Gộp 62 khung (~1 giây âm thanh thực) thành ma trận [62 x 13] cho Frozen
// Feature Extractor -> Trainable Classifier. Double-buffer ~6.3KB SRAM.

// ================== GIÁM SÁT PHẦN CỨNG MIC ==================
// i2s_read() OK không đảm bảo tín hiệu thật (dây lỏng/đứt vẫn đọc "thành
// công" nhưng toàn 0). Nên đo năng lượng raw (trước lọc DC/pre-emphasis).
#define HW_FAULT_RAW_ENERGY_THRESHOLD   50.0f   // Ngưỡng năng lượng bình phương trung bình tối thiểu để coi là có tín hiệu
#define HW_FAULT_CONSECUTIVE_HOPS       125     // Số hop liên tục dưới ngưỡng để kết luận lỗi (125 hops * 16ms ~ 2 giây)

static int hwFaultConsecutiveCount = 0;         // Đếm số lần hop liên tục bị câm/mất tín hiệu
static volatile bool micHardwareFaultFlag = false; // Cờ báo lỗi phần cứng mic (dùng volatile vì truy cập đa luồng)

// ================== DỮ LIỆU XUẤT CHO FUSION ==================
typedef struct {
    float mfccWindow[FRAMES_PER_WINDOW][NUM_MFCC]; // Ma trận đặc trưng MFCC 2D kích thước [62 x 13]
    int64_t timestampStartUs;   // Thời điểm bắt đầu ghi nhận window (microsecond) để đồng bộ thời gian với IMU
} AudioFeatureWindow_t;

static AudioFeatureWindow_t windowBuffer[2]; // Cơ chế Ping-Pong Double-buffer: 1 buffer để ghi, 1 buffer để đọc
static int writeBufferIndex = 0;             // Chỉ số buffer hiện tại đang được ghi dữ liệu (0 hoặc 1)

static QueueHandle_t windowReadyQueue = NULL; // Queue FreeRTOS truyền index của buffer đã ghi xong (tránh copy dữ liệu lớn)

// ================== BUFFER I2S THÔ ==================
static int32_t i2sReadBuf[HOP_LEN]; // Bộ đệm chứa các mẫu dữ liệu thô 32-bit vừa đọc từ DMA của I2S

// ================== RING BUFFER TÍN HIỆU ĐÃ LỌC ==================
static float filteredRing[FRAME_LEN]; // Bộ đệm trượt lưu 512 mẫu đã lọc để chuẩn bị dữ liệu đầu vào cho FFT (overlap 50%)

float dcOffset = 0.0f;          // Giá trị DC Offset trung bình ước tính
const float dcAlpha = 0.001f;   // Hệ số học (learning rate) cho bộ lọc trung bình động tích lũy (EMA) để loại bỏ DC

float hpAlpha = 0.97f;          // Hệ số lọc Pre-emphasis (lọc thông cao tăng cường tần số cao)
float hpPrevInput = 0.0f;       // Lưu giá trị đầu vào của mẫu phía trước ($x[n-1]$)
float hpPrevOutput = 0.0f;      // Lưu giá trị đầu ra của mẫu phía trước ($y[n-1]$)

// ================== FFT ==================
static float vReal[FRAME_LEN];  // Phần thực của dữ liệu đầu vào/ra FFT
static float vImag[FRAME_LEN];  // Phần ảo của dữ liệu đầu vào/ra FFT (khởi tạo bằng 0)
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FRAME_LEN, (float)SAMPLE_RATE); // Khởi tạo đối tượng FFT kiểu float

// ================== MEL FILTERBANK ==================
static float melFilterbank[NUM_MEL_FILTERS][FFT_BINS]; // Ma trận bộ lọc Mel [26 x 257], được tính trước 1 lần để tiết kiệm CPU

// Hàm chuyển đổi từ tần số Hz sang thang đo Mel (mô phỏng cảm nhận tai người)
float hzToMel(float hz) {
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

// Hàm chuyển đổi ngược từ thang đo Mel về tần số Hz
float melToHz(float mel) {
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

// Khởi tạo và dựng ma trận bộ lọc Mel (Mel Filterbank) dạng các hình tam giác xếp chồng
void buildMelFilterbank() {
    float melLow = hzToMel(MEL_LOW_HZ);   // Chuyển giới hạn dưới sang Mel
    float melHigh = hzToMel(MEL_HIGH_HZ); // Chuyển giới hạn trên sang Mel

    // Chia đều thang Mel thành (NUM_MEL_FILTERS + 2) điểm mốc (bao gồm 2 điểm biên)
    float melPoints[NUM_MEL_FILTERS + 2];
    for (int i = 0; i < NUM_MEL_FILTERS + 2; i++) {
        melPoints[i] = melLow + (melHigh - melLow) * i / (NUM_MEL_FILTERS + 1);
    }

    // Chuyển các điểm mốc Mel ngược lại Hz, sau đó quy đổi thành chỉ số bin tương ứng trong FFT
    int binIndices[NUM_MEL_FILTERS + 2];
    for (int i = 0; i < NUM_MEL_FILTERS + 2; i++) {
        float hz = melToHz(melPoints[i]);
        binIndices[i] = (int)floorf((FRAME_LEN + 1) * hz / SAMPLE_RATE);
    }

    // Dựng trọng số cho từng bộ lọc tam giác m: tăng dần từ left->center và giảm dần từ center->right
    for (int m = 1; m <= NUM_MEL_FILTERS; m++) {
        int left = binIndices[m - 1];   // Điểm bắt đầu tam giác
        int center = binIndices[m];     // Đỉnh tam giác
        int right = binIndices[m + 1];  // Điểm kết thúc tam giác

        // Sườn lên của bộ lọc tam giác
        for (int k = left; k < center; k++) {
            if (k >= 0 && k < FFT_BINS) {
                melFilterbank[m - 1][k] = (float)(k - left) / (float)(center - left);
            }
        }
        // Sườn xuống của bộ lọc tam giác
        for (int k = center; k < right; k++) {
            if (k >= 0 && k < FFT_BINS) {
                melFilterbank[m - 1][k] = (float)(right - k) / (float)(right - center);
            }
        }
    }
}

// ================== TÍNH MFCC CHO 1 KHUNG ==================
// Hàm tính toán 13 hệ số MFCC cho một khung tín hiệu đã qua FFT (kết quả phổ nằm trong vReal)
void computeMFCC(float mfccOut[NUM_MFCC]) {
    // 1. Áp dụng Mel Filterbank lên phổ năng lượng (Power Spectrum = Magnitude^2)
    float melEnergy[NUM_MEL_FILTERS];
    for (int m = 0; m < NUM_MEL_FILTERS; m++) {
        float sum = 0.0f;
        for (int k = 0; k < FFT_BINS; k++) {
            float power = vReal[k] * vReal[k];  // Năng lượng tại bin k
            sum += power * melFilterbank[m][k]; // Nhân tích lũy với trọng số bộ lọc
        }
        melEnergy[m] = sum;
    }

    // 2. Lấy Logarit tự nhiên năng lượng các dải Mel (thêm 1e-6f tránh lỗi log(0))
    float logMel[NUM_MEL_FILTERS];
    for (int m = 0; m < NUM_MEL_FILTERS; m++) {
        logMel[m] = logf(melEnergy[m] + 1e-6f);
    }

    // 3. Biến đổi Discrete Cosine Transform (DCT-II) để nén năng lượng và thu được các hệ số MFCC
    for (int c = 0; c < NUM_MFCC; c++) {
        float sum = 0.0f;
        for (int m = 0; m < NUM_MEL_FILTERS; m++) {
            sum += logMel[m] * cosf(PI / NUM_MEL_FILTERS * (m + 0.5f) * c);
        }
        mfccOut[c] = sum; // Lưu hệ số MFCC thứ c
    }
}

// ================== I2S SETUP ==================
// Hàm cấu hình và khởi tạo ngoại vi I2S thu âm từ microphone INMP441
void setupI2S() {
    // Cấu hình tham số hoạt động I2S
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Chế độ Master, nhận dữ liệu (RX)
        .sample_rate = SAMPLE_RATE,                         // Tần số lấy mẫu 16000 Hz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,       // Chuẩn đệm I2S 32-bit (INMP441 trả về dữ liệu 24-bit căn lề trái trong 32-bit)
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,        // Chỉ lấy dữ liệu kênh trái (Left channel)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // Định dạng chuẩn I2S Philips
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           // Mức ưu tiên ngắt thấp (Level 1)
        .dma_buf_count = 4,                                 // Số lượng bộ đệm DMA
        .dma_buf_len = HOP_LEN,                              // Độ dài mỗi bộ đệm DMA bằng HOP_LEN (256 mẫu)
        .use_apll = false,                                  // Không dùng APLL (dùng PLL nội mặc định)
        .tx_desc_auto_clear = false,                        // Không dùng cho chiều TX
        .fixed_mclk = 0                                     // Không dùng Master Clock cố định
    };

    // Cấu hình sơ đồ chân GPIO nối với mic INMP441
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,           // Chân BCLK
        .ws_io_num = I2S_WS,             // Chân LRCL
        .data_out_num = I2S_PIN_NO_CHANGE,// Không dùng TX
        .data_in_num = I2S_SD            // Chân DOUT
    };

    esp_err_t err;

    // Cài đặt driver I2S với cấu hình trên
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[LOI] i2s_driver_install that bai, ma loi = %d\n", err);
    }

    // Gán các chân GPIO cho ngoại vi I2S
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[LOI] i2s_set_pin that bai, ma loi = %d\n", err);
    }

    // Xóa sạch bộ đệm DMA ban đầu
    i2s_zero_dma_buffer(I2S_PORT);
}

// ================== LỌC 1 MẪU (DC removal + pre-emphasis) ==================
// Hàm xử lý tín hiệu theo thời gian thực cho từng mẫu âm thanh đơn lẻ
static inline float filterOneSample(int32_t rawSample32bitFrame) {
    // 1. Dịch phải 8 bit để trích xuất tín hiệu 24-bit thực tế từ khung 32-bit của INMP441
    int32_t rawSample = rawSample32bitFrame >> 8; 
    float sample = (float)rawSample;

    // 2. Loại bỏ thành phần một chiều (DC Offset removal): $DC[n] = DC[n-1] + \alpha \cdot (x[n] - DC[n-1])$
    dcOffset += dcAlpha * (sample - dcOffset);
    float sampleNoDC = sample - dcOffset;

    // 3. Lọc Pre-emphasis (High-pass filter): Tăng cường các tần số cao bị suy giảm trong giọng nói
    float hpOutput = hpAlpha * (hpPrevOutput + sampleNoDC - hpPrevInput);
    hpPrevInput = sampleNoDC;
    hpPrevOutput = hpOutput;

    return hpOutput;
}

// ================== TASK CHÍNH XỬ LÝ ÂM THANH ==================
// Task FreeRTOS chạy ngầm thu thập I2S và trích xuất đặc trưng MFCC
void audioTask(void *pvParameters) {
    int frameCounter = 0; // Đếm số khung đã xử lý trong window hiện tại (0 -> 61)

    // Khởi tạo bộ đệm trượt bằng 0
    memset(filteredRing, 0, sizeof(filteredRing));

    while (true) {
        // Đọc HOP_LEN (256) mẫu mới từ I2S qua DMA. Task sẽ tạm khóa (block) ở đây chờ DMA đủ dữ liệu (~16ms)
        size_t bytesRead = 0;
        esp_err_t err = i2s_read(I2S_PORT, i2sReadBuf, sizeof(i2sReadBuf),
                                  &bytesRead, portMAX_DELAY);

        int samplesRead = bytesRead / sizeof(int32_t); // Số mẫu thực tế đọc được
        if (err != ESP_OK || samplesRead != HOP_LEN) {
            Serial.printf("[CANH BAO] i2s_read doc thieu mau: %d/%d, err=%d\n",
                          samplesRead, HOP_LEN, err);
        }

        // --- GIÁM SÁT PHẦN CỨNG MIC ---
        // Đo năng lượng trung bình tín hiệu thô (Raw Energy) để phát hiện dây đứt/lỏng
        float rawEnergy = 0.0f;
        for (int i = 0; i < samplesRead; i++) {
            float rawVal = (float)(i2sReadBuf[i] >> 8);
            rawEnergy += rawVal * rawVal;
        }
        if (samplesRead > 0) {
            rawEnergy /= (float)samplesRead;
        }

        // Tín hiệu quá câm liên tục -> Cảnh báo lỗi phần cứng
        if (rawEnergy < HW_FAULT_RAW_ENERGY_THRESHOLD) {
            hwFaultConsecutiveCount++;
            if (hwFaultConsecutiveCount >= HW_FAULT_CONSECUTIVE_HOPS && !micHardwareFaultFlag) {
                micHardwareFaultFlag = true; // Bật cờ lỗi phần cứng mic
                Serial.println("[LOI PHAN CUNG] Mic nghi ngo bi long/dut day - "
                                "khong co tin hieu thuc trong ~2 giay lien tiep.");
            }
        } else {
            hwFaultConsecutiveCount = 0; // Reset bộ đếm nếu có tín hiệu bình thường trở lại
            micHardwareFaultFlag = false;
        }

        // --- TIỀN XỬ LÝ TÍN HIỆU ---
        // Lọc DC Offset + Pre-emphasis cho 256 mẫu mới vừa đọc
        float newFiltered[HOP_LEN];
        for (int i = 0; i < samplesRead; i++) {
            newFiltered[i] = filterOneSample(i2sReadBuf[i]);
        }
        // Đệm số 0 nếu đọc thiếu mẫu từ I2S
        for (int i = samplesRead; i < HOP_LEN; i++) {
            newFiltered[i] = 0.0f;
        }

        // --- SLIDING WINDOW (TỔ CHỨC RING BUFFER) ---
        // Dịch chuyển 256 mẫu cũ sang trái (loại bỏ 256 mẫu cũ nhất)
        memmove(filteredRing, filteredRing + HOP_LEN,
                (FRAME_LEN - HOP_LEN) * sizeof(float));
        // Chép 256 mẫu mới lọc xong vào nửa sau của ring buffer
        memcpy(filteredRing + (FRAME_LEN - HOP_LEN), newFiltered,
               HOP_LEN * sizeof(float));

        // Nạp dữ liệu 512 mẫu đã trượt vào mảng FFT
        memcpy(vReal, filteredRing, FRAME_LEN * sizeof(float));
        memset(vImag, 0, FRAME_LEN * sizeof(float)); // Reset phần ảo về 0

        // --- XỬ LÝ TẦN SỐ (DSP) ---
        // 1. Áp dụng cửa sổ Hamming để giảm hiện tượng rò rỉ phổ (spectral leakage)
        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        // 2. Thực hiện biến đổi Fourier nhanh (FFT)
        FFT.compute(FFTDirection::Forward);
        // 3. Tính độ lớn phổ (Magnitude Spectrum): $\sqrt{Real^2 + Imag^2}$, lưu đè vào vReal
        FFT.complexToMagnitude();

        // --- TRÍCH XUẤT ĐẶC TRƯNG MFCC ---
        // Tính 13 hệ số MFCC cho khung này và lưu thẳng vào Ping-Pong buffer hiện tại
        computeMFCC(windowBuffer[writeBufferIndex].mfccWindow[frameCounter]);

        // Nếu là khung đầu tiên trong window, ghi nhận timestamp thời gian thực
        if (frameCounter == 0) {
            windowBuffer[writeBufferIndex].timestampStartUs = esp_timer_get_time();
        }

        frameCounter++;

        // --- ĐỦ 62 KHUNG (~1 GiÂY AUDI0) -> ĐỔI BUFFER & BÁO CHO FUSION ---
        if (frameCounter >= FRAMES_PER_WINDOW) {
            int readyIndex = writeBufferIndex;        // Lưu chỉ số buffer đã ghi xong
            writeBufferIndex = 1 - writeBufferIndex; // Đổi sang buffer còn lại để tiếp tục ghi (Ping-Pong)
            frameCounter = 0;                         // Reset bộ đếm khung

            // Báo cho Fusion Task thông qua Queue (chèn/ghi đè chỉ số buffer sẵn sàng)
            if (windowReadyQueue != NULL) {
                xQueueOverwrite(windowReadyQueue, &readyIndex);
                // ưu tiên độ trễ thấp hơn giữ window cũ nếu Fusion chưa kịp đọc
            }
        }

        // Không delay() - tốc độ tự nhiên bị giới hạn bởi i2s_read() (~16ms)
    }
}

// ================== API CÔNG KHAI CHO MODULE FUSION ==================

// API để module Fusion lấy ma trận đặc trưng âm thanh [62 x 13] mới nhất
bool audioFeatures_getLatestWindow(AudioFeatureWindow_t *outWindow) {
    if (windowReadyQueue == NULL) return false;

    int readyIndex;
    // Kiểm tra không khóa (non-blocking, timeout=0) xem có window nào mới sẵn sàng không
    if (xQueueReceive(windowReadyQueue, &readyIndex, 0) == pdTRUE) {
        // Copy dữ liệu từ Ping-Pong buffer sẵn sàng ra struct bên ngoài
        memcpy(outWindow, &windowBuffer[readyIndex], sizeof(AudioFeatureWindow_t));
        return true;
    }
    return false; // Chưa có window mới
}

// API để module Fusion kiểm tra tình trạng phần cứng của Microphone
bool audioFeatures_isHardwareFaulted() {
    // true -> Fusion nên hạ trọng số nhánh audio, không coi im lặng là "Normal"
    return micHardwareFaultFlag;
}

// ================== SETUP / LOOP ==================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // 1. Khởi tạo ngoại vi phần cứng I2S
    setupI2S();
    // 2. Tính toán trước ma trận Mel Filterbank để sẵn sàng tính MFCC
    buildMelFilterbank();

    // 3. Khởi tạo Queue FreeRTOS độ dài 1 chứa chỉ số buffer (int)
    windowReadyQueue = xQueueCreate(1, sizeof(int));

    // 4. Tạo FreeRTOS Task chạy xử lý Audio trên Core 1 (Core 0 dành cho Wi-Fi/BLE)
    xTaskCreatePinnedToCore(
        audioTask,     // Hàm thực thi task
        "AudioTask",   // Tên task
        8192,          // Kích thước Stack (8KB)
        NULL,          // Tham số truyền vào task
        5,             // Mức ưu tiên (Priority = 5, khá cao)
        NULL,          // Con trỏ quản lý task
        1              // Ghim task vào Core 1
    );

    Serial.println("Mic + pipeline MFCC (FreeRTOS, sliding window) san sang.");
}

void loop() {
    static AudioFeatureWindow_t latestWindow;

    // Lấy dữ liệu MFCC window mới nếu có
    if (audioFeatures_getLatestWindow(&latestWindow)) {
        Serial.printf("Window moi, t=%lld us, MFCC khung dau: ", latestWindow.timestampStartUs);
        for (int c = 0; c < NUM_MFCC; c++) {
            Serial.print(latestWindow.mfccWindow[0][c], 2);
            Serial.print(c < NUM_MFCC - 1 ? ", " : "\n");
        }
    }

    // Giám sát sự thay đổi trạng thái lỗi phần cứng của Mic
    static bool lastFaultState = false;
    bool currentFaultState = audioFeatures_isHardwareFaulted();
    if (currentFaultState != lastFaultState) {
        Serial.printf("[TRANG THAI MIC] %s\n",
                      currentFaultState ? "LOI PHAN CUNG (mat tin hieu)"
                                         : "Da phuc hoi / binh thuong");
        lastFaultState = currentFaultState;
    }

    // Tạm dừng loop 50ms (nhường thời gian cho background tasks)
    vTaskDelay(pdMS_TO_TICKS(50));
}