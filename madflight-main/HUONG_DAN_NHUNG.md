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
| IMU | MPU-9265 / MPU-9250 (Gyro + Accel + Mag) | SPI Bus 0 |
| Barometer | BMP280 | I2C Bus 0 |
| GPS | NEO-6M V2 | Serial Bus 1 |
| Receiver | ELRS (CRSF protocol) | Serial Bus 0 |
| Motor | 4x BLDC A2212 930KV + ESC 40A | PWM 400Hz |

**Nguồn:** Cấp **5V** vào chân **Vin/VBUS** của ESP32-S3. Board tự tạo 3.3V cho cảm biến.

---

## 2. Sơ đồ đấu dây

### IMU MPU-9265 — SPI Bus 0

| Chân MPU-9265 | GPIO ESP32-S3 | Ghi chú |
|---------------|---------------|---------|
| VCC | 3.3V | |
| GND | GND | |
| SCL | **GPIO 13** | Trong SPI đây là **SCLK** |
| SDA | **GPIO 11** | Trong SPI đây là **SDI/MOSI** (ESP -> IMU) |
| ADO | **GPIO 12** | Trong SPI đây là **SDO/MISO** (IMU -> ESP) |
| NCS | **GPIO 10** | Chip Select (**CS**) |
| INT | **GPIO 14** | Interrupt bắt buộc |

> **Quan trọng:** Board thường chỉ in nhãn SCL/SDA/ADO/NCS, không in MISO/MOSI.  
> Map đúng là: `SCL->SCLK`, `SDA->MOSI`, `ADO->MISO`, `NCS->CS`.
> Không kéo cứng ADO/NCS như chế độ I2C.

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
| 10 | MPU-9265 NCS/CS (SPI) |
| 11 | MPU-9265 SDA/SDI/MOSI (SPI) |
| 12 | MPU-9265 ADO/SDO/MISO (SPI) |
| 13 | MPU-9265 SCL/SCLK (SPI) |
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
   ├── main.cpp            ← Code chính (setup, control loop, mixer)
        └── madflight_config.h  ← Cấu hình pin và hardware
```

### Các file đã chỉnh sửa

**`madflight_config.h`** — Cấu hình phần cứng:
- Dùng `default_ESP32-S3.h` làm nền
- Chạy IMU qua SPI (`imu_bus_type = SPI`, `imu_spi_bus = 0`, `pin_imu_cs = 10`)
- `imu_gizmo = MPU9250`, `bar_gizmo = BMP280`, `gps_gizmo = UBLOX`, `rcl_gizmo = CRSF`

**`main.cpp`** — Code chính:
- Không ép chân ADO/NCS kiểu I2C để tránh xung đột khi dùng SPI IMU

**`platformio.ini`** — Build config:
- Thêm env `ESP32-S3-N16R8` với 16MB Flash + OPI PSRAM
- `src_dir = examples/10.Quadcopter`
- `lib_extra_dirs = src`

---

## 4. Nhúng bằng Arduino IDE

Mục này hướng dẫn chi tiết theo workflow thực tế cho **ESP32-S3 N16R8**, ưu tiên dùng đúng mã nguồn local trong repo hiện tại.

### 4.1 Chuẩn bị môi trường

#### Bước 1: Cài Arduino IDE 2.x

1. Tải và cài Arduino IDE 2.x từ [arduino.cc/en/software](https://www.arduino.cc/en/software)
2. Mở IDE, vào:
   - **File -> Preferences**
   - hoặc phím tắt `Ctrl + ,`
3. Ở ô **Additional boards manager URLs**, thêm URL:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Nhấn **OK**

Minh họa text:

```text
File
 └─ Preferences
    └─ Additional boards manager URLs:
       https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

#### Bước 2: Cài core ESP32

1. Vào **Tools -> Board -> Boards Manager**
2. Tìm từ khóa `esp32`
3. Cài gói **esp32 by Espressif Systems**

Khuyến nghị phiên bản:
- Dùng bản stable mới nhất
- Nếu gặp lỗi lạ sau khi update, có thể pin về bản stable cũ hơn

### 4.2 Cài thư viện madflight local (chi tiết)

Có 3 cách. Bạn chỉ cần chọn 1 cách, không làm đồng thời.

#### Cách A (khuyến nghị): Cài local bằng Add .ZIP Library

Phù hợp khi muốn Arduino IDE nhận repo như một library chuẩn.

1. Đóng Arduino IDE
2. Nén thư mục `madflight-main/` thành file zip
   - Bên trong zip phải có `library.properties` và thư mục `src/`
3. Mở Arduino IDE
4. Vào **Sketch -> Include Library -> Add .ZIP Library...**
5. Chọn file zip vừa tạo

Sau khi cài, kiểm tra nhanh:
- Trong sketchbook sẽ có thư mục `libraries/madflight-main` (hoặc tên tương tự)
- Bên trong có `library.properties`

#### Cách B: Cài local thủ công vào thư mục libraries

Phù hợp khi bạn muốn quản lý phiên bản local ổn định, dễ kiểm soát.

1. Xác định thư mục sketchbook của Arduino IDE:
   - **File -> Preferences -> Sketchbook location**
2. Tạo thư mục:
   ```
   <Sketchbook>/libraries/madflight
   ```
3. Copy các thành phần sau từ repo vào thư mục trên:
   - `library.properties`
   - toàn bộ thư mục `src/`
4. Khởi động lại Arduino IDE

Checklist cấu trúc đúng:

```text
<Sketchbook>/libraries/madflight/
 ├─ library.properties
 └─ src/
    ├─ madflight.h
    └─ ...
```

#### Cách C: Dùng bản release từ Library Manager

1. **Sketch -> Include Library -> Manage Libraries**
2. Tìm `madflight`
3. Install `madflight by qqqlab`

Lưu ý quan trọng:
- Nếu dùng bản custom trong repo này thì không nên dùng Cách C
- Tránh xung đột bản local và bản release cùng lúc

### 4.3 Mở sketch đúng cách

1. Vào **File -> Open...**
2. Mở file:
   ```
   madflight-main/examples/10.Quadcopter/10.Quadcopter.ino
   ```
3. Sau khi mở, IDE sẽ hiện nhiều tab liên quan cùng sketch

Minh họa text:

```text
10.Quadcopter.ino   |   main.cpp   |   madflight_config.h
   entry sketch     | logic chính  | pin/config phần cứng
```

### 4.4 Cấu hình Tools cho ESP32-S3 N16R8 (bắt buộc)

Vào **Tools** và đặt theo bảng dưới.

| Mục (Tools ->) | Giá trị khuyến nghị | Ghi chú |
|----------------|---------------------|---------|
| Board | `ESP32S3 Dev Module` | Dòng S3 generic của Espressif |
| Port | `COMx` | Chọn đúng cổng serial của board |
| USB CDC On Boot | `Enabled` | Bắt buộc để dùng Serial Monitor qua USB |
| USB Mode | `Hardware CDC and JTAG` | Giá trị ổn định cho đa số DevKit S3 |
| Flash Size | `16MB (128Mb)` | Khớp N16R8 |
| Flash Mode | `QIO 80MHz` | Khuyến nghị |
| PSRAM | `OPI PSRAM` | Bắt buộc với N16R8 |
| Partition Scheme | `16M Flash (3MB APP/9.9MB FATFS)` | Dư cho firmware + dữ liệu |
| Upload Speed | `921600` | Nếu lỗi upload, hạ xuống 460800 |
| Core Debug Level | `None` | Đổi `Verbose` khi cần debug |

Minh họa text:

```text
Tools
 ├─ Board: ESP32S3 Dev Module
 ├─ Flash Size: 16MB
 ├─ PSRAM: OPI PSRAM
 ├─ USB CDC On Boot: Enabled
 └─ Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
```

Cảnh báo quan trọng:
- Chọn sai PSRAM (Quad/QSPI) có thể gây reboot loop ngay sau boot
- Chọn sai Flash Size có thể gây lỗi nạp hoặc runtime bất thường

### 4.5 Verify và Upload

#### Verify

1. Nhấn **Verify** (nút check) hoặc `Ctrl+R`
2. Nếu compile thành công, IDE sẽ in thống kê RAM/Flash

#### Upload

1. Cắm cáp USB data tốt
2. Chọn đúng **Tools -> Port**
3. Nhấn **Upload** hoặc `Ctrl+U`
4. Thành công khi thấy dòng tương tự:
   ```
   Leaving...
   Hard resetting via RTS pin...
   ```

### 4.6 Trường hợp không vào boot mode tự động

Thao tác thủ công:

1. Giữ **BOOT**
2. Nhấn nhả **RESET**
3. Thả **BOOT**
4. Bấm **Upload** ngay

Nếu vẫn lỗi:
- Đổi cáp USB khác (phải là cáp data)
- Cắm trực tiếp vào cổng USB máy tính, không qua hub
- Hạ Upload Speed xuống `460800` hoặc `115200`

### 4.7 Mở Serial Monitor và kiểm tra lần đầu

1. **Tools -> Serial Monitor** (`Ctrl+Shift+M`)
2. Baud rate: `115200`
3. Đặt board đứng yên hoàn toàn 6 giây đầu để gyro calibration
4. Kiểm tra log khởi động theo mục 6

### 4.8 Lỗi thường gặp với Arduino IDE + ESP32-S3 và cách xử lý

| Triệu chứng/lỗi | Nguyên nhân thường gặp | Cách xử lý nhanh |
|-----------------|------------------------|------------------|
| `madflight.h: No such file or directory` | Chưa cài library local đúng cấu trúc | Làm lại mục 4.2 Cách A/B, kiểm tra có `library.properties` và `src/madflight.h` |
| `Multiple libraries were found for "madflight.h"` | Trùng bản local và bản Library Manager | Gỡ bớt 1 bản, chỉ giữ 1 nguồn thư viện |
| `A fatal error occurred: Failed to connect to ESP32-S3` | Không vào bootloader hoặc COM sai | Làm mục 4.6, kiểm tra Port, cáp, driver |
| Upload xong nhưng board reboot liên tục | Sai lựa chọn PSRAM | Đặt lại `PSRAM = OPI PSRAM` |
| Không thấy COM port | Cáp chỉ sạc, driver thiếu, cổng USB lỗi | Đổi cáp data, đổi cổng USB, cài driver CP210x/CH34x |
| Serial Monitor không có log | USB CDC chưa bật hoặc baud sai | `USB CDC On Boot = Enabled`, baud `115200`, nhấn RESET |
| `Sketch too large` | Partition/flash chọn không đúng | Chọn lại `16MB` + partition `16M Flash (3MB APP/9.9MB FATFS)` |
| Compile lỗi ngẫu nhiên sau khi đổi board package | Cache build cũ | Đóng IDE, mở lại, verify lại; nếu cần gỡ và cài lại esp32 core |

### 4.9 Checklist trước khi ra bãi test

1. Verify pass, Upload pass
2. Serial log khởi động đầy đủ
3. IMU nhận đúng, GPS nhận đúng, RCL nhận đúng
4. `prcl` đọc được kênh RC
5. Đã tháo cánh khi test motor lần đầu

> Khuyến nghị: Arduino IDE dùng được tốt cho nạp nhanh và kiểm tra cơ bản. Nếu cần quản lý build/release ổn định hơn, chuyển sang PlatformIO ở mục 5.

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

IMU: MPU9250 installed on SPI0
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

1. Kiểm tra `imu_bus_type` đang là `SPI` và `imu_spi_bus = 0` trong `madflight_config.h`
2. Kiểm tra chân SPI đúng: SCLK=13, MOSI=11, MISO=12, CS=10, INT=14
3. Đảm bảo **không** có code nào set GPIO12 làm OUTPUT LOW (sẽ khóa MISO)
4. Kiểm tra dây CS/INT chắc chắn, không lỏng

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
