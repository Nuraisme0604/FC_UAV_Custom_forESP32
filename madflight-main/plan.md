# PLAN — madflight Quadcopter ESP32-S3 N16R8

> File này là STATE SNAPSHOT cho agent, cập nhật mỗi phiên làm việc.
> Tuân theo cấu trúc WORKING MEMORY từ `agent_rule.md`.

---

## [PROJECT]

- **Name:** madflight
- **Goal:** Firmware flight controller hiệu năng cao cho ESP32-S3 N16R8, cấu hình Quadcopter với MPU-9265 + BME/BMP280 + GPS NEO-6M + ELRS + 4x BLDC 2212.
- **Stack:**
  - C++ trên Arduino framework
  - PlatformIO (env `ESP32-S3-N16R8`)
  - Mã nhúng thời gian thực, kiến trúc module
- **Architecture:**
  - `src/madflight.h` → header chính
  - `src/madflight_modules.h` → gom toàn bộ modules
  - `src/<module>/` → implementation theo module (ahr, alt, bar, bat, bbx, brd, cfg, cli, gps, hal, imu, led, lua, mag, nav, ofl, out, pid, rcl, rdr, tbx, veh)
  - `examples/10.Quadcopter/` → entry logic chính (main.cpp + madflight_config.h)
  - `platformio.ini` → build config, `src_dir = examples/10.Quadcopter`
- **Tài liệu chính:** `HUONG_DAN_NHUNG.md`, `README.md`, `hwconfig.md`

---

## [CURRENT TASK]

- **Objective:** Thiết kế và triển khai 2 chức năng mới:
  1. **Altitude Hold** (giữ độ cao) — dùng barometer + IMU
  2. **Anti-Drift / Position Hold** (chống trôi) — dùng GPS
- **Input:** Codebase hiện có + phần cứng đã cấu hình (BMP280 + GPS NEO-6M)
- **Expected output:** Code mới trong `main.cpp`, có thể bật/tắt qua flight mode switch hoặc define
- **Done definition:**
  - Altitude hold giữ được độ cao ±1m khi thả throttle về giữa
  - Position hold giảm drift khi không có input roll/pitch
  - Không ảnh hưởng RATE/ANGLE mode khi tắt
  - Build hợp lệ cho env `ESP32-S3-N16R8`
  - Có thể tắt hoàn toàn bằng `#define`

---

## [CONSTRAINTS]

### Kỹ thuật
- MCU: ESP32-S3 N16R8 (16MB Flash QIO, 8MB OPI PSRAM)
- Tài nguyên hạn chế — tránh cấp phát động trong vòng lặp nóng
- Vòng lặp điều khiển (`imu_loop()`) phải ổn định, không block
- Build target duy nhất hiện tại: `[env:ESP32-S3-N16R8]`

### Chất lượng
- Ưu tiên ổn định runtime hơn tối ưu vi mô
- Tương thích ngược với cấu trúc module madflight hiện tại
- Giữ convention code hiện có (naming, comment style, file layout)

### An toàn bay
- ⚠️ **RỦI RO BAY** — cả 2 chức năng đều can thiệp trực tiếp vào throttle và roll/pitch
- Phải có cơ chế fallback khi sensor fail (bar mất data, GPS mất fix)
- Pilot phải luôn có thể override bằng stick
- Không thay đổi mapping pin hoặc bus

---

## [KNOWN STATE]

### Phần cứng đã cấu hình (madflight_config.h)
| Thiết bị | Gizmo | Bus | Pin chính |
|----------|-------|-----|-----------|
| IMU | MPU9250 (≡MPU-9265) | I2C Bus 1 | SDA=11, SCL=13, INT=14, NCS=10, ADO=12 |
| Barometer | BME/BMP280 | I2C Bus 0 | SDA=8, SCL=9 |
| GPS | UBLOX (NEO-6M) | Serial Bus 1 | RX=3, TX=46 |
| Receiver | CRSF (ELRS) | Serial Bus 0 | RX=18, TX=17 |
| Motors | PWM 400Hz | GPIO | M1=4, M2=5, M3=6, M4=7 |
| LED | HIGH_IS_ON | GPIO 2 | — |

### Infrastructure đã có sẵn cho feature mới

| Component | Vị trí | API | Ghi chú |
|-----------|--------|-----|---------|
| Altitude Estimator | `src/alt/` | `alt.getH()` (m), `alt.getV()` (m/s) | Kalman3 fuse bar+IMU, đã hoạt động |
| Barometer data | `src/bar/` | `bar.alt` (m), `bar.press` (Pa) | 100Hz, đã cấu hình BMP280 |
| GPS velocity | `src/gps/` | `gps.veln`, `gps.vele` (mm/s NED) | 5-10Hz, có `gps.fix`, `gps.hacc` |
| GPS position | `src/gps/` | `gps.lat`, `gps.lon` (1E-7 deg) | `gps.alt` (mm MSL) |
| Generic PID class | `src/pid/PIDController.h` | `control(desired, actual, dt)` → `out` | Có saturation, reset, derivative |
| PID output container | `src/pid/pid.h` | `pid.roll`, `pid.pitch`, `pid.yaw`, `pid.throttle` | `pid.throttle` tồn tại nhưng **CHƯA DÙNG** |
| Coordinate convert | `src/nav/Nav_Relative.h` | `to_m(lat, lon)` → (x_m, y_m) | Lat/lon sang mét |
| Scheduler | `src/tbx/MF_Schedule.h` | `schedule.interval(us)` → bool | Chạy code theo interval |
| Low-pass filter | `src/pid/FilterLowPass.h` | `filter.update(input, dt)` | Dùng lọc GPS noise |

### Code chính (main.cpp) — luồng hiện tại
```
imu_loop() @ 1000Hz:
  led_Blink()
  ahr.update()                    ← attitude fusion
  control_Rate() / control_Angle() ← PID roll/pitch/yaw → pid.roll, pid.pitch, pid.yaw
  out_KillSwitchAndFailsafe()     ← arm/disarm logic
  out_Mixer()                     ← thr = armed_min + (1-armed_min) * rcl.throttle
                                    motor = thr ± pid.pitch ± pid.roll ± pid.yaw
```

**Điểm chèn (injection points):**
- Altitude hold → chỉnh `thr` trong `out_Mixer()` hoặc dùng `pid.throttle`
- Position hold → chỉnh `pid.roll` / `pid.pitch` sau `control_Rate()`/`control_Angle()`

---

## [PLAN] — Altitude Hold + Anti-Drift

### Tổng quan kiến trúc

```
                    ┌─────────────────────────────────────────────────┐
                    │              imu_loop() @ 1000Hz                │
                    │                                                 │
  rcl.throttle ─────┤  ahr.update()                                   │
  rcl.roll ─────────┤  control_Rate() / control_Angle()               │
  rcl.pitch ────────┤      → pid.roll, pid.pitch, pid.yaw            │
  rcl.yaw ──────────┤                                                 │
                    │  ┌─ NEW ──────────────────────────────────┐     │
                    │  │ control_AltHold()     @ 100Hz          │     │
                    │  │   alt.getH(), alt.getV() → pid.throttle│     │
                    │  │                                        │     │
                    │  │ control_PosHold()     @ 50Hz           │     │
                    │  │   gps.veln/vele → Δpid.roll, Δpid.pitch│     │
                    │  └────────────────────────────────────────┘     │
                    │                                                 │
                    │  out_KillSwitchAndFailsafe()                    │
                    │  out_Mixer()  ← thr + pid.throttle (NEW)       │
                    │               ← pid.roll, pid.pitch, pid.yaw   │
                    └─────────────────────────────────────────────────┘
```

---

### Phase 1: Altitude Hold (Giữ độ cao)

**Nguyên lý:** Khi bật alt hold, throttle stick giữa = giữ nguyên độ cao hiện tại. Đẩy stick lên/xuống = tăng/giảm target altitude.

**Sensor pipeline (đã có sẵn, không cần code):**
```
bar.alt (100Hz) ──→ alt.updateBarAlt()  ──→ alt.getH() = altitude (m)
imu.az  (1000Hz) ──→ alt.updateAccelUp() ──→ alt.getV() = climb rate (m/s)
                     (Kalman3 fusion)
```

**Code mới cần viết trong `main.cpp`:**

#### 1.1 Khai báo (phần USER-SPECIFIED VARIABLES)
```cpp
// === ALTITUDE HOLD ===
#define FEATURE_ALT_HOLD          // comment để tắt hoàn toàn

const float alt_hold_deadband = 0.1;    // Stick deadband quanh center (0.0-1.0)
const float alt_hold_climb_rate = 2.0;  // Max climb/descend rate (m/s)
const float alt_hold_hover = 0.45;      // Throttle hover ước tính (0.0-1.0)

// PID altitude hold
const float Kp_alt = 0.8;    // P-gain: error (m) → target climb rate
const float Ki_alt = 0.15;   // I-gain: tích lũy error
const float Kd_alt = 0.3;    // D-gain: dùng alt.getV() làm derivative

// PID velocity vertical
const float Kp_vz = 0.5;     // P-gain: velocity error → throttle correction
const float Ki_vz = 0.2;     // I-gain
const float Kd_vz = 0.05;    // D-gain
```

#### 1.2 Hàm `control_AltHold()`
```
Logic:
1. Chạy @ 100Hz (dùng MF_Schedule, bỏ qua 9/10 lần gọi trong imu_loop 1000Hz)
2. Nếu throttle nằm trong deadband (giữa ± alt_hold_deadband):
   - Lần đầu vào deadband → ghi nhớ alt_target = alt.getH()
   - alt_error = alt_target - alt.getH()
   - vz_target = Kp_alt * alt_error  (giới hạn ±alt_hold_climb_rate)
3. Nếu throttle ngoài deadband:
   - Pilot đang climb/descend thủ công
   - Liên tục cập nhật alt_target = alt.getH()
   - vz_target = (rcl.throttle - 0.5) * 2 * alt_hold_climb_rate
4. Velocity PID: vz_error = vz_target - alt.getV()
   - pid.throttle = Kp_vz * vz_error + Ki_vz * integral + Kd_vz * derivative
   - Giới hạn pid.throttle trong [-0.3, +0.3]
5. Nếu disarmed hoặc throttle == 0 → reset tất cả integrator
```

#### 1.3 Sửa `out_Mixer()` — ⚠️ RỦI RO BAY
```cpp
// TRƯỚC (hiện tại):
float thr = armed_min_throttle + (1 - armed_min_throttle) * rcl.throttle;

// SAU (khi FEATURE_ALT_HOLD bật):
float thr;
#ifdef FEATURE_ALT_HOLD
  thr = alt_hold_hover + pid.throttle;                    // hover base + correction
  thr = constrain(thr, armed_min_throttle, 1.0f);         // clamp
#else
  thr = armed_min_throttle + (1 - armed_min_throttle) * rcl.throttle;  // giữ nguyên
#endif
```

**Rủi ro & mitigation:**
| Rủi ro | Mitigation |
|--------|-----------|
| Barometer noise → oscillation | alt/ đã dùng Kalman3 lọc, thêm D-gain thấp |
| Integrator windup | Clamp ±0.3 cho pid.throttle, reset khi disarmed |
| Mất barometer | Fallback: nếu `bar.dt > 1s` → tắt alt hold, chuyển về manual throttle |
| `alt_hold_hover` sai → rơi/bay lên | Pilot luôn có thể override bằng stick ngoài deadband |

---

### Phase 2: Anti-Drift / Position Hold (Chống trôi)

**Nguyên lý:** Khi bật pos hold và pilot không input roll/pitch, dùng GPS velocity để tạo roll/pitch correction chống drift.

**Sensor pipeline:**
```
gps.veln (mm/s, North) ──→ FilterLowPass ──→ vel_north (m/s)
gps.vele (mm/s, East)  ──→ FilterLowPass ──→ vel_east  (m/s)
ahr.yaw (degrees)       ──→ chuyển NED velocity sang body frame
```

**Code mới cần viết trong `main.cpp`:**

#### 2.1 Khai báo
```cpp
// === POSITION HOLD (ANTI-DRIFT) ===
#define FEATURE_POS_HOLD          // comment để tắt hoàn toàn

const float pos_hold_deadband = 0.05;   // Stick deadband roll/pitch
const float pos_hold_max_angle = 10.0;  // Max correction angle (degrees)
const int   pos_hold_min_sats = 6;      // Số vệ tinh tối thiểu

// PID velocity → angle
const float Kp_pos = 3.0;    // velocity error (m/s) → angle (deg)
const float Ki_pos = 0.5;    // tích lũy
const float Kd_pos = 0.2;    // derivative
```

#### 2.2 Hàm `control_PosHold()`
```
Logic:
1. Chạy @ 50Hz (dùng MF_Schedule)
2. Kiểm tra điều kiện: gps.fix >= FIX_3D && gps.sat >= pos_hold_min_sats
   - Nếu không đủ → không can thiệp, return
3. Chuyển GPS velocity NED → body frame:
   - yaw_rad = ahr.yaw * DEG_TO_RAD
   - vel_forward = veln*cos(yaw) + vele*sin(yaw)     (m/s)
   - vel_right   = -veln*sin(yaw) + vele*cos(yaw)    (m/s)
4. Nếu roll/pitch stick nằm trong deadband:
   - Target velocity = 0 (muốn đứng yên)
   - pitch_correction = PID(0 - vel_forward)   (giới hạn ±pos_hold_max_angle)
   - roll_correction  = PID(0 - vel_right)     (giới hạn ±pos_hold_max_angle)
   - Cộng vào pid.pitch, pid.roll (scaled by 0.01 như convention hiện có)
5. Nếu stick ngoài deadband:
   - Pilot đang bay thủ công → reset PID integrator
   - Không can thiệp
6. Nếu disarmed → reset tất cả
```

#### 2.3 Injection vào control loop — ⚠️ RỦI RO BAY
```cpp
// Trong imu_loop(), SAU control_Rate()/control_Angle(), TRƯỚC out_Mixer():
#ifdef FEATURE_POS_HOLD
  control_PosHold();  // cộng thêm correction vào pid.roll, pid.pitch
#endif
```

**Rủi ro & mitigation:**
| Rủi ro | Mitigation |
|--------|-----------|
| GPS lag (~200ms) → oscillation | Low-pass filter trên GPS velocity, D-gain thấp |
| GPS mất fix → correction sai | Check `gps.fix >= FIX_3D` && `gps.sat >= 6`, tắt khi fail |
| Correction quá mạnh → lật | `pos_hold_max_angle = 10°` (thấp hơn nhiều so với `maxRoll=30°`) |
| Conflict với ANGLE mode | ANGLE mode set roll/pitch target, pos hold thêm offset — tương thích |
| GPS indoor không có fix | Feature tự tắt, không ảnh hưởng bay manual |

---

### Phase 3: Tích hợp Flight Mode Switch

**Mục tiêu:** Dùng RC flight mode switch để chuyển giữa các mode:

```
Switch position 0 (low):    RATE only        (hiện tại)
Switch position 1 (mid):    RATE + ALT HOLD
Switch position 2 (high):   ANGLE + ALT HOLD + POS HOLD
```

**Implementation:**
```cpp
// Đọc flight mode channel từ rcl
// rcl.ch[flight_mode_ch] → 0.0 / 0.5 / 1.0
// Map sang enum: MODE_RATE, MODE_ALT_HOLD, MODE_FULL_HOLD
```

---

### Thứ tự triển khai

| Step | Nội dung | File | Ưu tiên |
|------|----------|------|---------|
| **1** | `control_AltHold()` — PID altitude + velocity vertical | `main.cpp` | **Cao** |
| **2** | Sửa `out_Mixer()` — inject `pid.throttle` | `main.cpp` | **Cao** |
| **3** | Fallback logic — tắt alt hold khi bar timeout | `main.cpp` | **Cao** |
| **4** | Test alt hold riêng (buộc dây, bay thấp) | — | **Cao** |
| **5** | `control_PosHold()` — PID GPS velocity → roll/pitch | `main.cpp` | Trung bình |
| **6** | NED→body frame transform | `main.cpp` | Trung bình |
| **7** | GPS fallback logic — tắt khi mất fix | `main.cpp` | Trung bình |
| **8** | Test pos hold (bay ngoài trời, có GPS fix) | — | Trung bình |
| **9** | Flight mode switch integration | `main.cpp` | Thấp |
| **10** | Tuning guide + tài liệu | `README.md` | Thấp |

---

### Verification Checklist

#### Alt Hold
- [ ] `alt.getH()` trả về giá trị hợp lý (so sánh với `bar.alt`)
- [ ] `alt.getV()` phản hồi khi nâng/hạ board bằng tay
- [ ] `pid.throttle` output ổn định, không oscillation
- [ ] Throttle stick giữa → quad giữ độ cao ±1m
- [ ] Throttle stick lên/xuống → quad climb/descend mượt
- [ ] Thả throttle = 0 → motor dừng (safety)
- [ ] Mất barometer → fallback về manual throttle

#### Pos Hold
- [ ] GPS fix 3D với ≥6 sats trước khi bật
- [ ] Quad không drift khi hover không có gió
- [ ] Roll/pitch stick override hoạt động mượt
- [ ] Mất GPS fix → pos hold tự tắt, không giật
- [ ] Correction angle không vượt quá 10°

---

## [NEXT ACTION]

- **Chờ user xác nhận plan** trước khi triển khai code
- Khi được duyệt → bắt đầu từ Step 1: `control_AltHold()` trong `main.cpp`
- Giả định: `alt.getH()` và `alt.getV()` đã hoạt động (Kalman3 + BMP280 đã cấu hình)

---

## [CHANGELOG]

| Ngày | Thay đổi |
|------|----------|
| 2026-04-03 | Tạo plan.md ban đầu, ghi nhận trạng thái repo và hardware config |
| 2026-04-03 | Lên plan Altitude Hold + Anti-Drift (Position Hold) — chờ duyệt |
