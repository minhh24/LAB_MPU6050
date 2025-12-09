#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// --- THƯ VIỆN MICRO-ROS ---
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h> 
#include "pico_uart_transports.h"

// ================= CẤU HÌNH (SỬA CẤU HÌNH CHÂN TẠI ĐÂY) =================
#define LED_PIN         25
#define I2C_PORT        i2c0
#define I2C_SDA         4   // Chân SDA (GP4)
#define I2C_SCL         5   // Chân SCL (GP5)
#define MPU6050_ADDR    0x68

// Hệ số chuyển đổi (Raw -> Đơn vị chuẩn SI)
// Accel: +/- 2g  -> chia 16384, nhân 9.81 để ra m/s^2
#define ACCEL_SCALE     (9.81 / 16384.0)
// Gyro:  +/- 250 -> chia 131.0, nhân PI/180 để ra rad/s
#define GYRO_SCALE      (0.01745 / 131.0) 

// ================= BIẾN TOÀN CỤC =================
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;

// ================= HÀM XỬ LÝ PHẦN CỨNG (I2C/MPU6050) =================

void mpu6050_init() {
    i2c_init(I2C_PORT, 400 * 1000); // Tốc độ 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Đánh thức MPU6050 (Ghi 0 vào thanh ghi Power Management 0x6B)
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[6];
    uint8_t reg;

    // 1. Đọc Gia tốc (Accel) - 6 bytes từ thanh ghi 0x3B
    reg = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    // Ghép 2 byte cao/thấp thành số nguyên 16-bit
    accel[0] = (buffer[0] << 8) | buffer[1]; // X
    accel[1] = (buffer[2] << 8) | buffer[3]; // Y
    accel[2] = (buffer[4] << 8) | buffer[5]; // Z

    // 2. Đọc Góc quay (Gyro) - 6 bytes từ thanh ghi 0x43
    reg = 0x43;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    gyro[0] = (buffer[0] << 8) | buffer[1]; // X
    gyro[1] = (buffer[2] << 8) | buffer[3]; // Y
    gyro[2] = (buffer[4] << 8) | buffer[5]; // Z
}

// ================= HÀM CALLBACK MICRO-ROS (CHẠY ĐỊNH KỲ) =================

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    if (timer == NULL) return;

    // 1. Lấy thời gian thực để vẽ đồ thị
    int64_t time_ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec = time_ns / 1000000000;
    imu_msg.header.stamp.nanosec = time_ns % 1000000000;

    // 2. Đọc cảm biến
    int16_t a_raw[3], g_raw[3];
    mpu6050_read_raw(a_raw, g_raw);

    // 3. Chuyển đổi sang đơn vị chuẩn và gán vào message
    imu_msg.linear_acceleration.x = a_raw[0] * ACCEL_SCALE;
    imu_msg.linear_acceleration.y = a_raw[1] * ACCEL_SCALE;
    imu_msg.linear_acceleration.z = a_raw[2] * ACCEL_SCALE;

    imu_msg.angular_velocity.x = g_raw[0] * GYRO_SCALE;
    imu_msg.angular_velocity.y = g_raw[1] * GYRO_SCALE;
    imu_msg.angular_velocity.z = g_raw[2] * GYRO_SCALE;

    // 4. Gửi dữ liệu lên PC
    rcl_publish(&publisher, &imu_msg, NULL);
    
    // 5. Nháy LED báo hiệu
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

// ================= CHƯƠNG TRÌNH CHÍNH =================

int main() {
    // --- BƯỚC 1: Cấu hình giao tiếp UART cho Micro-ROS ---
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    mpu6050_init(); // Khởi tạo I2C MPU6050

    // --- BƯỚC 2: Ping máy tính (Chờ kết nối Agent) ---
    // Đèn sẽ chớp liên tục đến khi kết nối được với máy tính
    while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    
    // Đồng bộ thời gian với máy tính (QUAN TRỌNG ĐỂ VẼ ĐỒ THỊ)
    rmw_uros_sync_session(1000);

    // --- BƯỚC 3: Khởi tạo các Node Micro-ROS ---
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_imu_node", "", &support);

    // Tạo Publisher gửi topic tên là "imu/data"
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );

    // Tạo Timer: Gọi hàm callback mỗi 50ms (20Hz)
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    // --- BƯỚC 4: Cấp phát bộ nhớ cho tên Frame (Bắt buộc) ---
    imu_msg.header.frame_id.data = (char*)malloc(20 * sizeof(char));
    sprintf(imu_msg.header.frame_id.data, "imu_link");
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = 20;

    // --- BƯỚC 5: Vòng lặp chính ---
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}