# madflight — Quadcopter ESP32-S3 N16R8

Firmware flight controller dựa trên [madflight v2.3.1-DEV](https://madflight.com) — MIT License.
Đã customize cho bộ phần cứng cụ thể: **ESP32-S3 N16R8 + MPU-9265 + BME/BMP280 + GPS NEO-6M + ELRS + 4x BLDC A2212 930KV**.

> **DEV Version** — bản phát triển, chưa được kiểm tra bay.
> Dùng [release version](https://github.com/qqqlab/madflight/releases) nếu cần bản đã flight tested.

---

## Phần cứng

| Thiết bị | Model | Giao tiếp | Nguồn | Pin ESP32-S3 |
|----------|-------|-----------|-------|-------------|
| MCU | ESP32-S3 N16R8 | — | **Vin 5V** | 16MB Flash QIO, 8MB OPI PSRAM |
| IMU | MPU-9265 (driver MPU9250) | I2C Bus 1 | 3.3V từ ESP | SDA=11, SCL=13, INT=14, NCS=10, ADO=12 |
| Barometer | BME/BMP280 (driver BMP280) | I2C Bus 0 | 3.3V từ ESP | SDA=8, SCL=9 |
| GPS | NEO-6M V2 | Serial Bus 1 | 3.3V từ ESP | ESP_RX=3, ESP_TX=46 |
| Receiver | Radiomaster XR1 Nano ELRS (CRSF) | Serial Bus 0 | 3.3V từ ESP | ESP_RX=18, ESP_TX=17 |
| Motor 1-4 | BLDC A2212 930KV + ESC 40A | PWM 400Hz | từ ESC | M1=GPIO4, M2=GPIO5, M3=GPIO6, M4=GPIO7 |
| LED | Built-in | GPIO 2 | — | — |

### Sơ đồ motor (Betaflight X-frame)

```
        TRƯỚC (mũi)
  M4[GPIO7]       M2[GPIO5]
    (CW)    \   /   (CCW)
             \ /
              X
             / \
    (CCW)   /   \   (CW)
  M3[GPIO6]       M1[GPIO4]
        SAU
```

---

## Cấu trúc thư mục

```
madflight-main/
├── HUONG_DAN_NHUNG.md          # Hướng dẫn đấu dây, calibration, CLI, xử lý sự cố
├── plan.md                     # STATE SNAPSHOT cho agent
├── platformio.ini              # Cấu hình PlatformIO (env ESP32-S3-N16R8)
├── library.properties          # Arduino library metadata
├── src/                        # Thư viện madflight
│   ├── madflight.h             # Header chính
│   ├── madflight_modules.h     # Include tất cả module
│   └── <module>/               # ahr, alt, bar, bat, bbx, brd, cfg, cli, gps,
│                               # hal, imu, led, lua, mag, nav, ofl, out, pid,
│                               # rcl, rdr, tbx, veh
└── examples/
    └── 10.Quadcopter/          # Entry logic chính
        ├── 10.Quadcopter.ino   # Mở file này nếu dùng Arduino IDE
        ├── main.cpp            # Code chính (setup, imu_loop, PID, mixer)
        └── madflight_config.h  # Cấu hình pin và hardware
```

---

## Hướng dẫn nhúng firmware

### Cách 1: VS Code + PlatformIO (khuyến nghị)

1. Cài [VS Code](https://code.visualstudio.com) → Extensions → cài **PlatformIO IDE** → khởi động lại
2. **File → Open Folder** → chọn thư mục `madflight-main/` (chứa `platformio.ini`)
3. Build: `Ctrl+Alt+B`
4. Upload: `Ctrl+Alt+U`
5. Serial Monitor: `Ctrl+Alt+S` (115200 baud)

Nếu cần chỉ định COM port, thêm vào `platformio.ini`:
```ini
upload_port = COM3
monitor_port = COM3
```

### Cách 2: Arduino IDE

1. Cài [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. File → Preferences → thêm URL:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Board Manager → cài **esp32 by Espressif Systems**
4. Manage Libraries → cài **madflight**
5. Board settings:
   ```
   Board:            ESP32S3 Dev Module
   Flash Size:       16MB (128Mb)
   PSRAM:            OPI PSRAM          ← BẮT BUỘC đúng cho N16R8
   Flash Mode:       QIO 80MHz
   USB CDC On Boot:  Enabled
   Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
   ```
6. Mở `examples/10.Quadcopter/10.Quadcopter.ino` → Upload (`Ctrl+U`)

### Board không tự vào boot mode?

Giữ **BOOT** → nhấn **RESET** → thả **BOOT** → nhấn Upload trong IDE.

---

## Setting ban đầu để bay

### Bước 1 — Kiểm tra khởi động

Mở Serial Monitor (115200 baud). **Đặt board đứng yên 6 giây đầu** (gyro tự calibrate).

Output mong đợi:
```
madflight v2.3.1-DEV starting ...
Board: ESP32-S3 DevKitC-1
IMU: MPU9250 installed on I2C1
GPS: UBLOX installed on Serial1
RCL: CRSF installed on Serial0
Setup completed, CLI started - Type 'help' for help
```

Nếu `IMU install failed` → gõ `i2cscan` kiểm tra địa chỉ `0x68`.

### Bước 2 — Calibrate Accelerometer

```
calimu          ← đặt máy bay nằm phẳng, đứng yên, chờ xong
save            ← lưu vào EEPROM
```

### Bước 3 — Calibrate Radio

```
calradio
```

Làm theo 7 bước trên Serial Monitor:

| Bước | Hành động |
|------|-----------|
| 1 | Center tất cả stick + ARM switch → DISARMED → Enter |
| 2 | Gạt ARM switch sang ARMED (hệ thống tự nhận kênh) |
| 3 | **Throttle**: min → max → giữa → Enter |
| 4 | **Pitch**: kéo về → đẩy ra → giữa → Enter |
| 5 | **Roll**: trái → phải → giữa → Enter |
| 6 | **Yaw**: trái → phải → giữa → Enter |
| 7 | **Flight Mode switch**: min → max → Enter (hoặc `q` nếu không có) |

```
prcl            ← kiểm tra giá trị kênh RC
save            ← BẮT BUỘC lưu
```

### Bước 4 — Kiểm tra trước bay

> **THÁO CÁNH QUẠT TRƯỚC KHI TEST MOTOR!**

```
prcl            ← di chuyển stick, giá trị channel phải thay đổi
pimu            ← xem tốc độ vòng lặp IMU
pahr            ← nghiêng board, roll/pitch/yaw phải đúng chiều
```

ARM bằng switch (hoặc stick: pull both sticks toward you, yaw full right, roll full left) → đẩy throttle nhẹ → kiểm tra:
- 4 motor phải quay
- Chiều quay đúng theo sơ đồ X-frame (CW/CCW)
- Motor sai chiều → đổi 2 dây pha bất kỳ

DISARM ngay sau khi kiểm tra xong.

### Bước 5 — Chọn Flight Mode

Mặc định: **RATE** (stick giữa = giữ tốc độ quay hiện tại, phổ biến cho FPV).

Đổi sang **ANGLE** (stick giữa = tự về ngang) — trong `main.cpp`:
```cpp
//#define FLIGHTMODE_RATE
#define FLIGHTMODE_ANGLE    // ← bật dòng này
```
Bắt buộc chạy `calimu` + `save` trước khi bay ANGLE mode. Build lại + Upload.

---

## Checklist trước bay đầu tiên

- [ ] Serial Monitor: IMU, RCL, GPS — không lỗi
- [ ] `calradio` + `save` xong
- [ ] `calimu` + `save` xong (nếu ANGLE mode)
- [ ] `prcl` — stick phản hồi đúng
- [ ] `pahr` — nghiêng board, roll/pitch/yaw đúng chiều
- [ ] Motor quay đúng chiều (đã test không cánh quạt)
- [ ] Cánh quạt lắp đúng (CW/CCW khớp motor)
- [ ] Pin đầy, kết nối chắc chắn
- [ ] **Bay thử ngoài trời, thoáng, không có người xung quanh**

---

## Lệnh CLI hữu ích

| Lệnh | Tác dụng |
|------|---------|
| `help` | Danh sách tất cả lệnh |
| `diff` | Param đã thay đổi so với default |
| `i2cscan` | Quét thiết bị I2C |
| `calradio` | Calibrate radio/receiver |
| `calimu` | Calibrate accelerometer |
| `pimu` | Hiệu suất IMU loop |
| `pahr` | Dữ liệu AHRS (roll, pitch, yaw) |
| `prcl` | Giá trị RC channels |
| `set param value` | Thay đổi tham số config |
| `save` | Lưu config vào EEPROM |
| `reboot` | Khởi động lại board |

---

## Xử lý sự cố

| Lỗi | Kiểm tra |
|-----|----------|
| `IMU install failed` | `i2cscan` → có `0x68`? Kiểm tra dây SDA=11, SCL=13, ADO=12 (LOW), NCS=10 (HIGH) |
| `RCL: not connected` | `rcl_gizmo = CRSF`? Pin RX=18? Receiver đã bind + bật trước ESP? |
| `GPS: not installed` | Pin RX=3, TX=46? `gps_baud = 0` (auto)? GPS cần vài phút lock vệ tinh |
| Motor không quay | ESC calibrate? Đã ARM? Throttle > 0? |
| Motor sai chiều | Đổi 2 dây pha bất kỳ của motor đó |
| Crash khi boot | PSRAM phải chọn **OPI** (không phải Quad) cho N16R8 |
| Board không flash | Giữ BOOT → nhấn RESET → thả BOOT → Upload |

---

## Tham khảo

- madflight: https://madflight.com
- ESP32-S3 datasheet: https://www.espressif.com/en/products/socs/esp32-s3
- MPU-9250 register map: https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/
- Forum: https://github.com/qqqlab/madflight/discussions

---

## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. Use and modify at your own risk.

MIT License — Copyright (c) 2023-2026 https://madflight.com
