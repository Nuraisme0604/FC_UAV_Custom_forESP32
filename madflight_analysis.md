# Phân tích Repo: madflight v2.3.1-DEV

## Tổng quan

**madflight** là một thư viện/framework flight controller mã nguồn mở dành cho các vi điều khiển nhỏ, cho phép xây dựng bộ điều khiển bay (flight controller) DIY chi phí thấp (<$10) với Arduino IDE hoặc PlatformIO.

- **License**: MIT
- **Version**: v2.3.1-DEV (đây là bản phát triển, chưa được kiểm tra bay)
- **Website**: https://madflight.com
- **Ngôn ngữ**: C++ (Arduino framework)

---

## Hardware được hỗ trợ

### Vi xử lý (MCU)
| MCU | Ví dụ board |
|-----|-------------|
| ESP32-S3 / ESP32 | Espressif ESP32/ESP32-S3 DevKitC |
| RP2350 / RP2040 | Raspberry Pi Pico / Pico2 |
| STM32 | Black Pill, etc. |

### Cảm biến bắt buộc
- **IMU**: MPUXXXX (MPU6000/6050/6500/9250/9265), BMI270, ICM426XX (ICM42688P...), ICM45686, LSM6DSO, LSM6DSV, LSM6DSV16B, Auto
- **RC Receiver**: CRSF (ELRS), SBUS, DSM, PPM, IBUS, MAVLink

### Cảm biến tùy chọn
- GPS: UBLOX, NMEA
- Barometer I2C: BMP280, BMP390, BMP580, DPS310, HP203B, MS5611
- Magnetometer I2C: QMC5883L, QMC5883P, QMC6309, RM3100, MMC5603, BMM150, IMU (internal AK8963)
- Battery Monitor: ADC, INA226, INA228
- Radar/Lidar/Sonar: LD2411S, LD2413, USD1, SR04, DTS6012M (I2C + UART), VL53L3CX
- Optical Flow: PMW3901, PMW3901U

---

## Cấu trúc thư mục

```
madflight-main/
├── HUONG_DAN_NHUNG.md          # Hướng dẫn đấu dây, nhúng, calibration, CLI, xử lý sự cố
├── plan.md                     # STATE SNAPSHOT cho agent (working memory)
├── platformio.ini              # Cấu hình PlatformIO (env ESP32-S3-N16R8 duy nhất)
├── library.properties          # Arduino library metadata
├── src/                        # Thư viện chính
│   ├── madflight.h             # Entry point chính - khởi tạo toàn bộ system
│   ├── madflight_modules.h     # Include tất cả module
│   ├── ahr/    # AHRS - Attitude and Heading Reference System
│   ├── alt/    # Altitude Estimator
│   ├── bar/    # Barometer
│   ├── bat/    # Battery Monitor
│   ├── bbx/    # Black Box (SDCARD logger)
│   ├── brd/    # Board definitions (default.h, default_ESP32-S3.h)
│   ├── cfg/    # Configuration & parameters
│   ├── cli/    # Command Line Interface
│   ├── gps/    # GPS
│   ├── hal/    # Hardware Abstraction Layer (ESP32, RP2040, STM32)
│   ├── imu/    # IMU sensor drivers
│   ├── led/    # LED controller
│   ├── lua/    # Lua scripting engine
│   ├── mag/    # Magnetometer
│   ├── nav/    # Navigation
│   ├── ofl/    # Optical Flow
│   ├── out/    # Outputs (motors, servos - PWM/Oneshot/DShot)
│   ├── pid/    # PID Controller
│   ├── rcl/    # RC Link (Radio Control)
│   ├── rdr/    # Radar/Lidar/Sonar
│   ├── tbx/    # Toolbox (MsgBroker, Scheduler, etc.)
│   └── veh/    # Vehicle info (MAVLink vehicle type)
└── examples/
    └── 10.Quadcopter/          # ✅ Entry logic chính (duy nhất trong repo hiện tại)
        ├── 10.Quadcopter.ino   # Mở file này nếu dùng Arduino IDE
        ├── main.cpp            # Code chính (setup, imu_loop, PID, mixer)
        └── madflight_config.h  # Cấu hình pin và hardware cho ESP32-S3 N16R8
```

> **Lưu ý:** Repo gốc madflight (upstream) có thêm `00.HelloWorld`, `11.QuadcopterAdvanced`, `20.Plane`, `extras/`. Bản repo này chỉ giữ lại `10.Quadcopter` và đã customize cho phần cứng cụ thể.

---

## Kiến trúc phần mềm

### 1. Luồng khởi động (Startup Flow)

```
setup() →
  madflight_setup() →
    hal_startup()         # HAL khởi động
    cfg.begin()           # Load config từ EEPROM/Flash
    led.setup()           # LED
    bbx.setup()           # Black Box (SDCARD)
    hal_usb_setup()       # USB
    Serial.begin()        # Serial console
    cli_task (RTOS)       # CLI task trên core0
    [6 giây delay]
    rcl.setup()           # RC Link
    rcl_task (RTOS)       # RC + output watchdog task
    bar/mag/bat/rdr/ofl/gps.setup() # Các sensor phụ
    sensor_task (RTOS)    # Task đọc sensor + logging
    alt.setup()           # Altitude estimator
    ahr.setup()           # AHRS
    imu.setup()           # IMU (10 lần thử nếu fail)
    imu.onUpdate = imu_loop  # Đăng ký callback IMU
    calibrate_gyro()      # Hiệu chỉnh gyro
    lua.begin()           # Lua scripting
```

### 2. Kiến trúc RTOS (FreeRTOS)

Hệ thống chạy 3 RTOS tasks song song:
| Task | Core | Chức năng |
|------|------|-----------|
| `cli_task` | Core 0 | Xử lý lệnh CLI qua Serial |
| `rcl_task` | Core 0 | Nhận RC commands + motor timeout watchdog |
| `sensor_task` | Core 0 | Đọc barometer/mag/gps/bat/radar/ofl + logging BBX |
| **imu_loop** | **Core ISR** | **Main flight control loop** (interrupt-driven) |

### 3. Vòng lặp điều khiển bay (imu_loop)

IMU được kết nối qua interrupt (pin_imu_int). Mỗi khi có data IMU mới (~1000Hz):

```
imu_loop() [~1kHz, interrupt-driven]:
  led_Blink()             # Nhấp nháy LED để chỉ thị trạng thái
  ahr.update()            # Sensor fusion → roll, pitch, yaw (Mahony/Madgwick/VQF)
  control_Rate() hoặc
  control_Angle()         # PID controller
  out_KillSwitchAndFailsafe()  # Kiểm tra arm/disarm
  out_Mixer()             # Mix và gửi lệnh đến motor
```

### 4. Hệ thống Module ("Gizmo" Pattern)

Mỗi module (ahr, imu, bar, etc.) theo mẫu:
- **Config struct**: chứa tham số cấu hình
- **State struct**: output/data của module
- **Gizmo class (abstract)**: driver cụ thể (polymorphism)
- **Main class**: wraps config + state + gizmo

Ví dụ:
```cpp
imu.config.gizmo = mf_BMI270;   // chọn chip
imu.setup();                     // khởi tạo với driver tương ứng
imu.gx, imu.gy, imu.gz          // đọc angular velocity
```

### 5. Hệ thống Config (cfg)

- Tất cả tham số được định nghĩa qua macro `MF_PARAM_LIST`
- Lưu trữ trong EEPROM/Flash (persistent)
- Có thể thay đổi qua CLI (`set param_name value`) hoặc MAVLink
- Cấu trúc: `madflight_config.h` trong từng example → `cfg.load_madflight()`
- Tham số pin, gizmo, hiệu chỉnh sensor, RC mapping...
- **Config format:** Dùng C++ raw string literal `R""(...)""` trong `madflight_config.h`, mỗi dòng là `key value` (không dùng `=`). Ví dụ:
  ```cpp
  const char madflight_config[] = R""(
  imu_gizmo     MPU9250
  pin_i2c1_sda  11
  pin_i2c1_scl  13
  )"";
  ```

### 6. Hệ thống MsgBroker (Pub/Sub)

`tbx` cung cấp message broker nội bộ để share data giữa tasks:
- `MsgTopic<T>`: publisher
- `MsgSubscription<T>`: subscriber với lock-free pull
- Ví dụ: `ahr.topic` publish AhrState, `sensor_task` subscribe để logging

---

## AHRS (Attitude Estimation)

Module `ahr` hỗ trợ 4 thuật toán sensor fusion:
| Gizmo | Mô tả |
|-------|-------|
| **MAHONY** (default) | Mahony complementary filter |
| **MAHONY_BF** | Betaflight variant |
| **MADGWICK** | Madgwick filter |
| **VQF** | Versatile Quaternion-based Filter |

Output: `ahr.roll`, `ahr.pitch`, `ahr.yaw` (degrees), quaternion `ahr.q[4]`

---

## Bộ điều khiển PID

Hai chế độ bay:
- **FLIGHTMODE_RATE**: Ổn định angular rate (deg/sec) - phổ biến cho FPV
- **FLIGHTMODE_ANGLE**: Ổn định góc tuyệt đối (degrees) - cần calibrate IMU trước

PID kép cho Angle mode: outer loop (angle → rate setpoint) + inner loop (rate PID)

Motor mixing (Betaflight order, X-frame):
```
    M4  M2   (front)
     \ /
      X
     / \
    M3  M1   (back)

M1: -pitch -roll -yaw (Back Right, CW)
M2: +pitch -roll +yaw (Front Right, CCW)
M3: -pitch +roll +yaw (Back Left, CCW)
M4: +pitch +roll -yaw (Front Left, CW)
```

---

## Outputs (out module)

Hỗ trợ nhiều chuẩn ESC protocol:
- **PWM** (400Hz, 950-2000µs)
- **Oneshot125** (2000Hz, 125-250µs)
- **DShot300/600/BiDir**
- **Brushed motors** (PWM 0-100%)
- Servo outputs

---

## CLI (Command Line Interface)

Kết nối qua Serial (USB CDC), các lệnh:
- `help`: danh sách lệnh
- `diff`: hiển thị các param đã thay đổi so với default
- `set param value`: thay đổi param
- `save`: lưu vào EEPROM
- `calimu`, `calgyro`: hiệu chỉnh cảm biến
- `pimu`: in IMU performance profile

---

## Lua Scripting

Module `lua` cho phép chạy script Lua từ SDCARD (`/madflight.lua`), mở ra khả năng lập trình mission tùy chỉnh mà không cần recompile.

---

## Cấu hình phần cứng hiện tại (ESP32-S3 N16R8)

Repo này đã được customize cho bộ phần cứng cụ thể:

| Thiết bị | Model | Gizmo / Protocol | Bus | Nguồn | Pin ESP32-S3 |
|----------|-------|-------------------|-----|-------|-------------|
| MCU | ESP32-S3 N16R8 | — | — | **Vin 5V** | 16MB Flash QIO, 8MB OPI PSRAM |
| IMU | MPU-9265 | MPU9250 (tương thích) | I2C Bus 1 | 3.3V từ ESP | SDA=11, SCL=13, INT=14, NCS=10, ADO=12 |
| Barometer | BME/BMP280 | BMP280 | I2C Bus 0 | 3.3V từ ESP | SDA=8, SCL=9 |
| GPS | NEO-6M V2 | UBLOX | Serial Bus 1 | 3.3V từ ESP | ESP_RX=3 ← GPS_TX, ESP_TX=46 → GPS_RX |
| Receiver | ELRS | CRSF | Serial Bus 0 | 3.3V từ ESP | ESP_RX=18 ← ELRS_TX, ESP_TX=17 → ELRS_RX |
| Motor 1-4 | BLDC 2212 + ESC 40A | PWM 400Hz, 950-2000µs | GPIO | từ ESC/Pin | M1=4, M2=5, M3=6, M4=7 |
| LED | Built-in | HIGH_IS_ON | GPIO 2 | — | — |

> **Ghi chú BME/BMP280:** Nếu dùng BME280 (có humidity), driver `BMP280` trong madflight vẫn đọc áp suất/nhiệt độ bình thường — hai chip tương thích register cho phần barometer.

**AHRS:** Mahony (default) · **Flight mode:** RATE · **armed_min_throttle:** 0.20

**Đặc biệt trong `setup()`:**
- GPIO12 (ADO) → OUTPUT LOW → I2C addr 0x68
- GPIO10 (NCS) → OUTPUT HIGH → I2C mode (không phải SPI)

**Thứ tự motor (Betaflight X-frame):**
```
      TRƯỚC (mũi)
 M4[GPIO7]     M2[GPIO5]
   (CW)    \ /    (CCW)
            X
   (CCW)   / \    (CW)
 M3[GPIO6]     M1[GPIO4]
      SAU
```

> Chi tiết đấu dây, calibration, CLI: xem `HUONG_DAN_NHUNG.md`
> Bản gốc cấu hình phần cứng: xem `hwconfig.md`

---

## Tóm tắt nhanh

| Thuộc tính | Chi tiết |
|-----------|---------|
| Mục đích | DIY flight controller firmware |
| Targets | ESP32, RP2040/RP2350, STM32 (HAL có sẵn; repo này chỉ cấu hình ESP32-S3) |
| Framework | Arduino + FreeRTOS |
| Kiến trúc | Modular, interrupt-driven, multi-task |
| Vòng lặp chính | `imu_loop()` @ ~1kHz (interrupt) |
| Sensor fusion | Mahony / Madgwick / VQF |
| Config | EEPROM-backed, CLI + MAVLink |
| Logging | SDCARD Black Box (BBX) |
| Scripting | Lua |
| Build env | `ESP32-S3-N16R8` (duy nhất trong `platformio.ini`) |
| License | MIT |
