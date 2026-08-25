#include "bmi160.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BMI160";

esp_err_t bmi160_read_regs(bmi160_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (dev->intf == BMI160_INTF_I2C) {
        // Đọc qua I2C (ESP-IDF v4/v5 API)
        return i2c_master_write_read_device(dev->i2c_port, dev->i2c_addr, 
                                            &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
    } else {
        // Đọc qua SPI (Cần set bit MSB = 1 cho địa chỉ thanh ghi để báo hiệu lệnh READ)
        uint8_t tx_data = reg_addr | 0x80;
        
        spi_transaction_t t = {
            .length = 8,                 // Số bit truyền (1 byte địa chỉ)
            .rxlength = len * 8,         // Số bit nhận
            .tx_buffer = &tx_data,       // Gửi địa chỉ thanh ghi với bit MSB = 1
            .rx_buffer = data            // Nhận dữ liệu trả về từ BMI160
        };
        return spi_device_transmit(dev->spi_handle, &t);
    }
}

esp_err_t bmi160_write_regs(bmi160_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (dev->intf == BMI160_INTF_I2C) {
        // I2C: Cần tạo buffer chứa [reg_addr] + [data...]
        uint8_t *buffer = (uint8_t *)malloc(len + 1);
        if (!buffer) return ESP_ERR_NO_MEM;
        buffer[0] = reg_addr;
        memcpy(&buffer[1], data, len);
        
        esp_err_t err = i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr, 
                                                   buffer, len + 1, pdMS_TO_TICKS(1000));
        free(buffer);
        return err;     //trả về 0 nếu thành công
    } else {
        // SPI: Set MSB = 0 cho địa chỉ thanh ghi để báo hiệu lệnh WRITE
        uint8_t tx_addr = reg_addr & 0x7F;
        
        // Ghi tuần tự từng byte (BMI160 SPI hỗ trợ burst write, nhưng để đơn giản ta có thể gửi chuỗi)
        // SPI cần gửi địa chỉ, sau đó là data. Ta sẽ cấu hình transaction chứa cả 2.
        uint8_t *tx_buf = (uint8_t *)malloc(len + 1);
        if (!tx_buf) return ESP_ERR_NO_MEM;
        tx_buf[0] = tx_addr;
        memcpy(&tx_buf[1], data, len);
        
        spi_transaction_t t = {
            .length = (len + 1) * 8,
            .tx_buffer = tx_buf,
            .rx_buffer = NULL
        };
        esp_err_t err = spi_device_transmit(dev->spi_handle, &t);
        free(tx_buf);
        return err;
    }
}

esp_err_t bmi160_write_reg(bmi160_dev_t *dev, uint8_t reg_addr, uint8_t data) {
    return bmi160_write_regs(dev, reg_addr, &data, 1);
}

esp_err_t bmi160_init(bmi160_dev_t *dev) {
    uint8_t chip_id = 0;
    
    // 1. Đọc Chip ID để kiểm tra kết nối
    esp_err_t err = bmi160_read_regs(dev, BMI160_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lỗi giao tiếp với BMI160!");
        return err;
    }
    
    if (chip_id != BMI160_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "Chip ID không khớp. Đọc được: 0x%02X, Cần: 0x%02X", chip_id, BMI160_CHIP_ID_VAL);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "Tìm thấy BMI160, Chip ID: 0x%02X", chip_id);

    // 2. Soft Reset (Đưa về trạng thái mặc định)
    bmi160_write_reg(dev, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(15)); // Đợi BMI160 reset xong

    // 3. Bật Accelerometer (chuyển từ Suspend sang Normal Mode)
    bmi160_write_reg(dev, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(5)); // Thời gian khởi động Accel tối đa khoảng 3.8ms

    ESP_LOGI(TAG, "BMI160 khởi tạo thành công.");
    return ESP_OK;
}

esp_err_t bmi160_config_wake_on_motion(bmi160_dev_t *dev) {
    ESP_LOGI(TAG, "Đang cấu hình ngắt Wake-on-Motion...");

    // 1. Cấu hình hành vi chân INT1 (Thanh ghi INT_OUT_CTRL 0x53)
    // Bit 3 = 1: Bật ngắt INT1
    // Bit 2 = 0: Push-pull
    // Bit 1 = 1: Active High (Ngắt kéo chân INT1 lên mức cao)
    // Bit 0 = 0: Edge triggered (Kích hoạt theo sườn)
    uint8_t int_out_ctrl = 0x0A; // 0b00001010
    bmi160_write_reg(dev, BMI160_REG_INT_OUT_CTRL, int_out_ctrl);

    // 2. Map Any-Motion Interrupt vào chân INT1 (Thanh ghi INT_MAP_0 0x55)
    // Bit 2 = 1: Ánh xạ ngắt Any-motion tới INT1
    uint8_t int_map_0 = (1 << 2);
    bmi160_write_reg(dev, BMI160_REG_INT_MAP_0, int_map_0);

    // 3. Chế độ Latch cho ngắt (Thanh ghi INT_LATCH 0x54)
    // 0x00: Non-latched (Ngắt tự động xóa khi hết motion)
    // 0x0F: Latched (Giữ ngắt cho đến khi đọc thanh ghi INT_STATUS)
    bmi160_write_reg(dev, BMI160_REG_INT_LATCH, 0x00); // Chọn non-latched cho đơn giản

    // 4. Cấu hình thông số Any-Motion (Ngưỡng và thời gian)
    // Thanh ghi INT_MOTION_1 (0x5F): Ngưỡng kích hoạt (Threshold)
    // Giá trị này phụ thuộc vào Range của Accel (mặc định 2g). Ngưỡng = (val * 3.91mg)
    bmi160_write_reg(dev, BMI160_REG_INT_MOTION_1, 0x14); // Khoảng 78mg 

    // Thanh ghi INT_MOTION_2 (0x60): Any-motion behavior
    // Bits 1:0 = Số mẫu cần vượt ngưỡng liên tiếp (VD: 0x00 = 1 mẫu, 0x01 = 2 mẫu...)
    bmi160_write_reg(dev, BMI160_REG_INT_MOTION_2, 0x01); // Kích hoạt nếu 2 mẫu liên tiếp vượt ngưỡng

    // 5. Bật tính năng ngắt Any-motion (Thanh ghi INT_EN_0 0x50)
    // Bit 2, 1, 0 tương ứng bật ngắt cho trục Z, Y, X
    uint8_t int_en_0 = 0x07; // Bật cho cả 3 trục (0b00000111)
    bmi160_write_reg(dev, BMI160_REG_INT_EN_0, int_en_0);

    ESP_LOGI(TAG, "Hoàn tất cấu hình Wake-on-Motion trên INT1.");
    return ESP_OK;
}