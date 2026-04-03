# Hướng dẫn Nhúng Firmware Quadcopter
**ESP32-S3 N16R8 + MPU-9265 + BMP280 + GPS NEO-6M + ELRS + 4x BLDC A2212 930KV**  
Firmware: [madflight v2.3.1](https://madflight.com) — MIT License

---

## Mục lục
1. [Phần cứng](#1-phần-cứng)
2. [Sơ đồ đấu dây](#2-sơ-đồ-đấu-dây)
3. [Cấu trúc file](#3-cấu-trúc-file)
4. [Nhúng bằng Arduino IDE](#4-nhúng-bằng-arduino-ide)
5. [Nhúng bằng VS Code + PlatformIO](#5-nhúng-bằng-vs-code--platformio)
6. [Kiểm tra khởi động](#6-kiểm-tra-khởi-động)
7. [Calibration](#7-calibration)
8. [Lệnh CLI hữu ích](#8-lệnh-cli-hữu-ích)
9. [Xử lý sự cố](#9-xử-lý-sự-cố)

---

## 1. Phần cứng

| Linh kiện | Model | Giao tiếp |
|-----------|-------|-----------|
| Vi điều khiển | ESP32-S3 N16R8 (16MB Flash, 8MB OPI PSRAM) | — |
| IMU | MPU-9265 / MPU-9250 (Gyro + Accel + Mag) | I2C Bus 1 |
| Barometer | BMP280 | I2C Bus 0 |
| GPS | NEO-6M V2 | Serial Bus 1 |
| Receiver | ELRS (CRSF protocol) | Serial Bus 0 |
| Motor | 4x BLDC A2212 930KV + ESC 40A | PWM 400Hz |

**Nguồn:** Cấp **5V** vào chân **Vin/VBUS** của ESP32-S3. Board tự tạo 3.3V cho cảm biến.

---

## 2. Sơ đồ đấu dây

### IMU MPU-9265 — I2C Bus 1

| Chân MPU-9265 | GPIO ESP32-S3 | Ghi chú |
|---------------|---------------|---------|
| VCC | 3.3V | |
| GND | GND | |
| SCL | **GPIO 13** | |
| SDA | **GPIO 11** | |
| INT | **GPIO 14** | Interrupt bắt buộc |
| ADO | **GPIO 12** | Code kéo xuống LOW → địa chỉ I2C = `0x68` |
| NCS | **GPIO 10** | Code kéo lên HIGH → enable I2C mode (không phải SPI) |

> **Quan trọng:** ADO và NCS được điều khiển bởi `setup()` trong `main.cpp`.  
> Không cần đấu thêm điện trở pull-up/pull-down bên ngoài.

### Barometer BMP280 — I2C Bus 0

| Chân BMP280 | GPIO ESP32-S3 | Ghi chú |
|-------------|---------------|---------|
| VCC | 3.3V | |
| GND | GND | |
| SCL | **GPIO 9** | |
| SDA | **GPIO 8** | |
| CSB | 3.3V | Nếu có — chọn I2C mode |
| SDO/ADR | GND | Địa chỉ I2C = `0x76` |

### GPS NEO-6M V2 — Serial Bus 1

| Chân NEO-6M | GPIO ESP32-S3 | Ghi chú |
|-------------|---------------|---------|
| VCC | 3.3V | |
| GND | GND | |
| TX | **GPIO 3** | GPS gửi → ESP nhận |
| RX | **GPIO 46** | ESP gửi → GPS nhận |

### RC Receiver ELRS — Serial Bus 0

| Chân Receiver | GPIO ESP32-S3 | Ghi chú |
|---------------|---------------|---------|
| VCC | 3.3V | |
| GND | GND | |
| TX | **GPIO 18** | Receiver gửi RC data → ESP nhận |
| RX | **GPIO 17** | ESP gửi telemetry → Receiver nhận |

### Motor ESC 40A

```
        TRƯỚC (mũi máy bay)
  M4[GPIO7]       M2[GPIO5]
    (CW)    \   /   (CCW)
             \ /
              X
             / \
    (CCW)   /   \   (CW)
  M3[GPIO6]       M1[GPIO4]
        SAU
```

| Motor | Vị trí | Chiều quay | GPIO | Config |
|-------|---------|-----------|------|--------|
| M1 | Sau Phải | CW | **GPIO 4** | pin_out0 |
| M2 | Trước Phải | CCW | **GPIO 5** | pin_out1 |
| M3 | Sau Trái | CCW | **GPIO 6** | pin_out2 |
| M4 | Trước Trái | CW | **GPIO 7** | pin_out3 |

### Tổng hợp pin

| GPIO | Chức năng |
|------|-----------|
| 2 | LED built-in |
| 3 | GPS TX → ESP RX |
| 4 | Motor 1 ESC |
| 5 | Motor 2 ESC |
| 6 | Motor 3 ESC |
| 7 | Motor 4 ESC |
| 8 | BMP280 SDA |
| 9 | BMP280 SCL |
| 10 | MPU-9265 NCS (HIGH = I2C mode) |
| 11 | MPU-9265 SDA |
| 12 | MPU-9265 ADO (LOW = addr 0x68) |
| 13 | MPU-9265 SCL |
| 14 | MPU-9265 INT |
| 17 | ESP TX → Receiver RX (telemetry) |
| 18 | Receiver TX → ESP RX (RC data) |
| 46 | ESP TX → GPS RX |

---

## 3. Cấu trúc file

```
madflight-main/
├── HUONG_DAN_NHUNG.md          ← File này
├── platformio.ini              ← Cấu hình PlatformIO (đã chỉnh ESP32-S3-N16R8)
├── src/                        ← Thư viện madflight (nguồn)
│   ├── madflight.h             ← Header chính
│   └── ...
└── examples/
    └── 10.Quadcopter/
        ├── 10.Quadcopter.ino   ← Mở file này nếu dùng Arduino IDE
        ├── main.cpp            ← Code chính (đã thêm ADO/NCS init)
        └── madflight_config.h  ← Cấu hình pin và hardware
```

### Các file đã chỉnh sửa

**`madflight_config.h`** — Cấu hình phần cứng:
- Dùng `default_ESP32-S3.h` làm nền
- Override I2C Bus 1: SDA=11, SCL=13 (cho MPU-9265)
- `imu_gizmo = MPU9250`, `bar_gizmo = BMP280`, `gps_gizmo = UBLOX`, `rcl_gizmo = CRSF`

**`main.cpp`** — Code chính:
- Thêm `pinMode(12, OUTPUT); digitalWrite(12, LOW);` → ADO = LOW
- Thêm `pinMode(10, OUTPUT); digitalWrite(10, HIGH);` → NCS = HIGH (I2C mode)

**`platformio.ini`** — Build config:
- Thêm env `ESP32-S3-N16R8` với 16MB Flash + OPI PSRAM
- `src_dir = examples/10.Quadcopter`
- `lib_extra_dirs = src`

---

## 4. Nhúng bằng Arduino IDE

### Bước 1: Cài đặt

1. Cài **Arduino IDE 2.x** từ [arduino.cc](https://www.arduino.cc/en/software)
2. Vào **File → Preferences**, thêm vào _Additional boards manager URLs_:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. **Tools → Board → Board Manager** → Tìm **esp32 by Espressif Systems** → Install
4. **Tools → Manage Libraries** → Tìm **madflight** → Install

### Bước 2: Cấu hình Board

```
Tools → Board             : ESP32S3 Dev Module
Tools → Flash Size        : 16MB (128Mb)
Tools → PSRAM             : OPI PSRAM          ← BẮT BUỘC đúng với N16R8
Tools → Flash Mode        : QIO 80MHz
Tools → USB CDC On Boot   : Enabled
Tools → Partition Scheme  : 16M Flash (3MB APP/9.9MB FATFS)
Tools → Upload Speed      : 921600
Tools → Port              : COMx (chọn đúng port)
```

> **Lưu ý PSRAM:** Chọn đúng **OPI PSRAM** cho ESP32-S3 N16R8. Chọn sai sẽ crash khi boot.

### Bước 3: Mở và Upload

1. Mở `examples/10.Quadcopter/10.Quadcopter.ino`
2. Nhấn **Upload** (`Ctrl+U`)

### Bước 4: Nếu không tự vào boot mode

1. Giữ nút **BOOT** trên board
2. Nhấn **RESET** một lần
3. Thả **BOOT**
4. Nhấn Upload trong IDE

---

## 5. Nhúng bằng VS Code + PlatformIO

### Bước 1: Cài đặt

1. Cài **VS Code** từ [code.visualstudio.com](https://code.visualstudio.com)
2. Trong VS Code: Extensions (`Ctrl+Shift+X`) → Tìm **PlatformIO IDE** → Install
3. Khởi động lại VS Code

### Bước 2: Mở Project

```
File → Open Folder → chọn thư mục: madflight-main (1)/madflight-main/
```

> Phải mở **thư mục chứa `platformio.ini`**, không mở file lẻ.

### Bước 3: Build và Upload

Dùng **thanh công cụ PlatformIO ở dưới cùng**:

| Nút | Phím tắt | Tác dụng |
|-----|----------|---------|
| ✓ Build | `Ctrl+Alt+B` | Compile kiểm tra lỗi |
| → Upload | `Ctrl+Alt+U` | Compile + flash firmware |
| 🔌 Monitor | `Ctrl+Alt+S` | Mở Serial Monitor 115200 baud |
| 🗑️ Clean | — | Xóa cache build |

### Bước 4: Chọn COM port (nếu cần)

Thêm vào `platformio.ini` trong section `[env:ESP32-S3-N16R8]`:
```ini
upload_port = COM3     ; ← đổi thành COM port của bạn
monitor_port = COM3
```

### Lưu ý về lỗi IntelliSense

VS Code có thể hiển thị lỗi đỏ như `'madflight.h' file not found` hay `OUTPUT undeclared`.  
**Đây là lỗi giả của IntelliSense** — không ảnh hưởng đến compile thật.  
Build bằng PlatformIO (nút ✓) sẽ thành công vì đã cấu hình `lib_extra_dirs = src`.

---

## 6. Kiểm tra khởi động

Sau khi upload, mở Serial Monitor (**115200 baud**). Đặt board **đứng yên hoàn toàn** trong 6 giây đầu để gyro tự calibrate.

### Output mong đợi

```
madflight v2.3.1-DEV starting 12 ...
madflight v2.3.1-DEV starting 11 ...
...
Board: ESP32-S3 DevKitC-1 (default board)
Processor: ESP32-S3

[led] HIGH_IS_ON pin=2
[bbx] NONE
[rcl] CRSF ser=0
[bar] BMP280 i2c=0 adr=0x76
[mag] NONE
[bat] NONE
[rdr] NONE
[ofl] NONE
[gps] UBLOX ser=1

IMU: MPU9250 installed on I2C1
GPS: UBLOX installed on Serial1
RCL: CRSF installed on Serial0

Setup completed, CLI started - Type 'help' for help, or 'diff' to debug
```

### Trạng thái LED

| Trạng thái LED | Ý nghĩa |
|----------------|---------|
| Xanh dương sáng liên tục | Đang khởi động / calibrate gyro |
| Nhấp nháy xanh lá (dài tắt, ngắn bật) | **DISARMED** — bình thường |
| Nhấp nháy đỏ (dài bật, ngắn tắt) | **ARMED** — cẩn thận! |
| Nhấp nháy rất nhanh | Có lỗi — xem Serial Monitor |
| Khoảng nhấp nháy > 1 giây | `imu_loop()` chạy quá chậm |

---

## 7. Calibration

### 7.1 Calibrate Gyro (tự động)

Gyro **tự động calibrate** khi khởi động. Chỉ cần đặt máy bay **đứng yên hoàn toàn** trong 6 giây đầu (khi LED xanh sáng).

### 7.2 Calibrate IMU/Accelerometer

Dùng khi bay ở chế độ **ANGLE mode** (trở về nằm ngang tự động).

Gõ trong Serial Monitor:
```
calimu
```
Đặt máy bay **nằm phẳng**, đứng yên, chờ đến khi báo xong. Sau đó:
```
save
```

### 7.3 Calibrate Radio (`calradio`)

Gõ lệnh:
```
calradio
```

Làm theo **7 bước** hiện trên Serial Monitor:

| Bước | Hành động |
|------|-----------|
| **Step 1** | Center tất cả stick + ARM switch về DISARMED → Enter |
| **Step 2** | Gạt **ARM switch** sang ARMED (hệ thống tự nhận kênh) |
| **Step 3** | **Throttle**: kéo xuống thấp → đẩy lên cao → về giữa → Enter hoặc toggle arm |
| **Step 4** | **Pitch**: kéo về phía người → đẩy ra trước → về giữa → Enter hoặc toggle arm |
| **Step 5** | **Roll**: đẩy trái → đẩy phải → về giữa → Enter hoặc toggle arm |
| **Step 6** | **Yaw**: đẩy trái → đẩy phải → về giữa → Enter hoặc toggle arm |
| **Step 7** | **Flight Mode switch**: min → max → Enter hoặc `q` nếu không có |

Sau khi hoàn tất:
```
prcl          ← kiểm tra lại giá trị kênh
save          ← lưu vào EEPROM (BẮT BUỘC)
```

> **Lưu ý:** Nhấn `q` bất kỳ lúc nào để hủy bỏ calibration.

### 7.4 Kiểm tra RC Input

```
prcl
```
Di chuyển stick trên tay cầm, giá trị channel sẽ thay đổi.

---

## 8. Lệnh CLI hữu ích

Kết nối Serial Monitor (115200 baud), gõ các lệnh sau:

| Lệnh | Tác dụng |
|------|---------|
| `help` | Danh sách tất cả lệnh |
| `diff` | Hiển thị các tham số đã thay đổi so với mặc định |
| `calradio` | Calibrate radio/receiver |
| `calimu` | Calibrate accelerometer |
| `calgyro` | Calibrate gyro thủ công |
| `prcl` | In giá trị RC channels hiện tại |
| `pimu` | In thông tin IMU và thời gian xử lý |
| `pahr` | In dữ liệu AHRS (roll, pitch, yaw) |
| `i2cscan` | Quét và liệt kê thiết bị I2C |
| `save` | Lưu config vào EEPROM |
| `reboot` | Khởi động lại board |
| `set param value` | Thay đổi một tham số config |

---

## 9. Xử lý sự cố

### IMU không nhận (`IMU install failed`)

1. Gõ `i2cscan` — kiểm tra địa chỉ `0x68` có xuất hiện không
2. Kiểm tra ADO: GPIO12 phải được kéo xuống LOW trong `setup()`
3. Kiểm tra NCS: GPIO10 phải được kéo lên HIGH trong `setup()`
4. Kiểm tra đấu dây SDA=11, SCL=13 đúng chưa

### Receiver không kết nối (`RCL: not connected`)

1. Kiểm tra type gizmo trong `madflight_config.h`: `rcl_gizmo = CRSF`
2. Kiểm tra pin: GPIO18 (RX) nhận signal từ Receiver TX
3. Bật receiver trước khi khởi động ESP32
4. Kiểm tra receiver đã bind với transmitter chưa

### GPS không nhận (`GPS: not installed`)

1. Kiểm tra pin: GPIO3 (RX) nhận từ GPS TX, GPIO46 (TX) gửi đến GPS RX
2. `gps_baud = 0` (auto-detect) — không cần đặt cố định
3. GPS cần vài phút để lock tín hiệu vệ tinh ở lần đầu

### Motor không quay / quay sai chiều

> ⚠️ **THÁO CÁNH QUẠT TRƯỚC KHI TEST MOTOR!**

1. Kiểm tra ESC đã được calibrate chưa (throttle min/max)
2. Nếu motor quay **sai chiều**: đổi 2 dây pha bất kỳ của motor đó
3. Kiểm tra thứ tự motor đúng theo sơ đồ X-frame

### Board không vào chế độ flash

1. Giữ **BOOT** → nhấn **RESET** → thả **BOOT** → Upload
2. Hoặc dùng cáp USB trực tiếp (không qua hub)

### Lỗi PSRAM / crash ngay khi boot

- Kiểm tra chọn đúng **OPI PSRAM** (không phải Quad PSRAM) trong board settings
- N16R8 bắt buộc phải chọn OPI

---

## Tham khảo

- Tài liệu madflight: https://madflight.com
- ESP32-S3 datasheet: https://www.espressif.com/en/products/socs/esp32-s3
- MPU-9250 register map: https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/
- Forum madflight: https://github.com/qqqlab/madflight/discussions
