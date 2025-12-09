cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# --- CẤU HÌNH ĐƯỜNG DẪN (Dựa trên máy của bạn) ---
set(PICO_SDK_PATH "/home/pc12/micro_ros_ws/src/pico-sdk")
set(MICROROS_PATH "/home/pc12/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/libmicroros")

set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(lab_mpu6050 C CXX ASM) # Đặt tên không dấu cách cho an toàn
pico_sdk_init()

# Cấu hình Micro-ROS
include_directories(${MICROROS_PATH}/include)
link_directories(${MICROROS_PATH})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Thêm file nguồn (Lưu ý tên file có dấu cách phải để trong ngoặc kép)
add_executable(lab_mpu6050 "LAB MPU6050.c" pico_uart_transport.c)

# --- CẤU HÌNH NGÕ RA ---
# Tắt USB, Bật UART để dùng GP0/GP1
pico_enable_stdio_usb(lab_mpu6050 0)
pico_enable_stdio_uart(lab_mpu6050 1)

# Link thư viện
target_link_libraries(lab_mpu6050
        pico_stdlib
        microros          # Thư viện Micro-ROS
        hardware_pio
        hardware_i2c      # Thư viện I2C cho MPU6050
        hardware_uart     # Thư viện UART
        )

pico_add_extra_outputs(lab_mpu6050)