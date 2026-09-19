#ifndef AUDIO_FRONTEND_H
#define AUDIO_FRONTEND_H

#include <stdint.h>
#include <stdbool.h>

// ================== CẤU HÌNH XỬ LÝ TÍN HIỆU & FEATURE WINDOW ==================
#define NUM_MFCC            13
#define FRAMES_PER_WINDOW   62

// ================== KIỂU DỮ LIỆU ĐẦU RA (STRUCT) ==================
typedef struct {
    float mfccWindow[FRAMES_PER_WINDOW][NUM_MFCC]; // Ma trận đặc trưng MFCC 2D [62 x 13]
    int64_t timestampStartUs;                       // Thời điểm bắt đầu ghi nhận window (us)
} AudioFeatureWindow_t;

// ================== KHAI BÁO API CÔNG KHAI ==================

/**
 * @brief Khởi tạo phần cứng I2S, dựng Mel Filterbank, khởi tạo Queue và tạo FreeRTOS Task.
 *        Gói gọn toàn bộ quá trình setup của module Audio Front-end.
 */
void audioFrontend_init(void);

/**
 * @brief Lấy ma trận đặc trưng âm thanh [62 x 13] mới nhất cho module Fusion.
 * @param outWindow Con trỏ chứa dữ liệu struct nhận được
 * @return true nếu có window mới sẵn sàng, false nếu chưa có dữ liệu mới
 */
bool audioFeatures_getLatestWindow(AudioFeatureWindow_t *outWindow);

/**
 * @brief Kiểm tra tình trạng phần cứng của Microphone (dây lỏng/đứt/mất tín hiệu).
 * @return true nếu phát hiện lỗi phần cứng, false nếu mic hoạt động bình thường
 */
bool audioFeatures_isHardwareFaulted(void);

#endif // AUDIO_FRONTEND_H