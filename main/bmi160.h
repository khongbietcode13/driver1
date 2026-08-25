#ifndef MAIN_BMI160_H_
#define MAIN_BMI160_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <driver/i2s.h>
#include<esp_nn.h>


/* Các thanh ghi cơ bản của BMI160 */
#define BMI160_REG_CHIP_ID       0x00   // Thanh ghi Chip ID
#define BMI160_CHIP_ID_VAL       0xD1   // Giá trị Chip ID của BMI160

#define BMI160_REG_PMU_STATUS    0x03   // Thanh ghi trạng thái PMU (Power Management Unit)
#define BMI160_REG_INT_STATUS_0  0x1C   // Thanh ghi trạng thái ngắt 0
#define BMI160_REG_INT_STATUS_1  0x1D   // Thanh ghi trạng thái ngắt 1
#define BMI160_REG_ACC_CONF      0x40   // Thanh ghi cấu hình gia tốc kế
#define BMI160_REG_ACC_RANGE     0x41   // Thanh ghi phạm vi gia tốc kế
#define BMI160_REG_INT_EN_0      0x50   // Thanh ghi kích hoạt ngắt 0
#define BMI160_REG_INT_OUT_CTRL  0x53   // Thanh ghi điều khiển đầu ra ngắt
#define BMI160_REG_INT_LATCH     0x54   // Thanh ghi chế độ latch ngắt
#define BMI160_REG_INT_MAP_0     0x55   // Thanh ghi ánh xạ ngắt 0
#define BMI160_REG_INT_MOTION_1  0x5F   // Thanh ghi cấu hình chuyển động 1
#define BMI160_REG_INT_MOTION_2  0x60   // Thanh ghi cấu hình chuyển động 2
#define BMI160_REG_CMD           0x7E   // Thanh ghi Command

/* Các lệnh Command (Ghi vào thanh ghi CMD) */
#define BMI160_CMD_SOFT_RESET    0xB6   // Lệnh Soft Reset (Đưa về trạng thái mặc định)
#define BMI160_CMD_ACC_NORMAL    0x11   // Lệnh chuyển gia tốc kế sang chế độ normal
#define BMI160_CMD_GYRO_NORMAL   0x15   // Lệnh chuyển con quay hồi chuyển sang chế độ normal

/* Cấu trúc chọn giao thức giao tiếp */
typedef enum {
    BMI160_INTF_I2C,
    BMI160_INTF_SPI
} bmi160_intf_t;

/* Cấu trúc lưu trữ thông tin thiết bị */
typedef struct {
    bmi160_intf_t intf;          // Chuẩn giao tiếp: I2C hoặc SPI

    /* Cấu hình I2C */
    i2c_port_t i2c_port;         // I2C_NUM_0 hoặc I2C_NUM_1
    uint8_t i2c_addr;            // Địa chỉ I2C (Thường là 0x68 hoặc 0x69)

    /* Cấu hình SPI */
    spi_device_handle_t spi_handle; // Handle sau khi add SPI device

    /* Chân ngắt */
    gpio_num_t int_pin;          // Chân GPIO của ESP32-S3 nối với INT1 của BMI160
} bmi160_dev_t;

/**
 * @brief Đọc nhiều byte từ thanh ghi
 */
esp_err_t bmi160_read_regs(bmi160_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Ghi nhiều byte vào thanh ghi
 */
esp_err_t bmi160_write_regs(bmi160_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Ghi 1 byte vào thanh ghi
 */
esp_err_t bmi160_write_reg(bmi160_dev_t *dev, uint8_t reg_addr, uint8_t data);

/**
 * @brief Khởi tạo BMI160, kiểm tra Chip ID và chuyển sang Normal Mode
 */
esp_err_t bmi160_init(bmi160_dev_t *dev);

/**
 * @brief Cấu hình ngắt Any-Motion (Wake-on-Motion) xuất ra chân INT1
 */
esp_err_t bmi160_config_wake_on_motion(bmi160_dev_t *dev);

#endif /* MAIN_BMI160_H_ */