# Micro-ROS MPU6050 Publisher trên Raspberry Pi Pico

Dự án Lab hướng dẫn giao tiếp giữa **Raspberry Pi Pico** và máy tính (PC) chạy **ROS 2** thông qua giao thức **Micro-ROS**.
Dự án thực hiện đọc dữ liệu từ cảm biến gia tốc/góc nghiêng **MPU6050** và gửi lên ROS 2 Topic để vẽ đồ thị thời gian thực.

##  Tác giả
- Họ và tên:** Nguyễn Quang Minh
- Liên hệ:** 0916254336

---

##  Phần cứng yêu cầu

1.  **Raspberry Pi Pico** (RP2040).
2.  **Cảm biến MPU6050** (Gia tốc kế & Con quay hồi chuyển).
3.  **Mạch chuyển đổi USB-to-TTL** (CP2102, CH340, hoặc PL2303).
4.  Dây cắm testboard.

##  Sơ đồ đấu nối (Pinout)

### 1. Kết nối UART (Giao tiếp với PC)
| Chân Pico | Chân USB-to-TTL | Chức năng |
| :--- | :--- | :--- |
| **GP0** (Pin 1) | **RX** | Truyền dữ liệu (TX -> RX) |
| **GP1** (Pin 2) | **TX** | Nhận dữ liệu (RX <- TX) |
| **GND** (Pin 3) | **GND** | Mass chung (Bắt buộc) |

### 2. Kết nối I2C (Cảm biến MPU6050)
| Chân Pico | Chân MPU6050 | Chức năng |
| :--- | :--- | :--- |
| **GP4** (Pin 6) | **SDA** | Dữ liệu I2C |
| **GP5** (Pin 7) | **SCL** | Xung nhịp I2C |
| **3V3** (Pin 36) | **VCC** | Nguồn 3.3V |
| **GND** (Pin 38) | **GND** | Mass |

---

##  Cài đặt môi trường

Đảm bảo máy tính đã cài đặt:
* **Ubuntu 22.04** (hoặc tương đương).
* **ROS 2 Humble Hawksbill**.
* **Micro-ROS Agent**.

### Cài đặt các gói hiển thị (nếu chưa có)
```bash
sudo apt update
sudo apt install ros-humble-rqt-plot ros-humble-rqt-common-plugins
