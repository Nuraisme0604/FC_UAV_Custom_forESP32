/*========================================================================================================================
                                              MADFLIGHT CONFIG
==========================================================================================================================

  Target hardware:
    MCU:        ESP32-S3 N16R8 (16MB Flash QIO, 8MB OPI PSRAM)
    IMU:        MPU-9265 / MPU-9250 (I2C, I2C Bus 1: SDA=GPIO11, SCL=GPIO13)
    Barometer:  BMP280          (I2C, I2C Bus 0: SDA=GPIO8,  SCL=GPIO9)
    GPS:        NEO-6M V2       (Serial Bus 1: ESP_RX=GPIO3,  ESP_TX=GPIO46)
    Receiver:   ELRS/CRSF       (Serial Bus 0: ESP_RX=GPIO18, ESP_TX=GPIO17)
    Motors:     4x BLDC 2212 via ESC 40A (PWM: GPIO4, GPIO5, GPIO6, GPIO7)

  NOTE về ADO và NCS của MPU-9265:
    - ADO (GPIO12): Setup code sẽ kéo xuống LOW → địa chỉ I2C = 0x68
    - NCS (GPIO10): madflight tự kéo lên HIGH khi setup I2C mode

========================================================================================================================*/

// Dùng default ESP32-S3 board làm nền, sau đó override các pin khác bên dưới
#define MF_BOARD "brd/default_ESP32-S3.h"

const char madflight_config[] = R""(

// ===========================================================
//  Override pins cho ESP32-S3 N16R8
//  PHẦN LỚN đã đúng với default_ESP32-S3.h, chỉ cần override I2C1 và các gizmo
// ===========================================================

// --- LED ---
// GPIO2 là LED built-in trên hầu hết ESP32-S3 DevKit
led_gizmo     HIGH_IS_ON
pin_led       2

// --- I2C Bus 0 --- Barometer BMP280 ---
// SDA=8, SCL=9 → trùng với default, không cần override
// pin_i2c0_sda  8   ← default đã có
// pin_i2c0_scl  9   ← default đã có

// --- I2C Bus 1 --- IMU MPU-9265 ---
// OVERRIDE: default dùng SDA=21, SCL=47 → đổi sang SDA=11, SCL=13
pin_i2c1_sda  11
pin_i2c1_scl  13

// --- IMU: MPU-9265 (tương thích MPU-9250) ---
imu_gizmo     MPU9250   // MPU-9265 tương thích hoàn toàn với driver MPU9250
imu_bus_type  I2C
imu_i2c_bus   1
imu_i2c_adr   0         // 0 = dùng địa chỉ mặc định (0x68 khi ADO=LOW)
pin_imu_int   14        // trùng default
pin_imu_cs    10        // NCS pin: madflight kéo HIGH để enable I2C mode

// --- Barometer: BMP280 ---
bar_gizmo     BMP280
bar_i2c_bus   0
bar_i2c_adr   0         // 0 = địa chỉ mặc định (0x76 khi SDO=GND)

// --- Magnetometer: AK8963 bên trong MPU-9250 ---
// Được kết nối tự động qua IMU driver, không cần config thêm
mag_gizmo     NONE

// --- GPS: NEO-6M V2 ---
// ESP_RX=GPIO3, ESP_TX=GPIO46 → trùng với pin_ser1 trong default board
// pin_ser1_rx   3    ← default đã có
// pin_ser1_tx  46    ← default đã có
gps_gizmo     UBLOX
gps_ser_bus   1         // trùng default
gps_baud      0         // 0 = auto-detect baud rate

// --- RC Receiver: ELRS (CRSF protocol) ---
// Receiver_TX=GPIO18 (ESP nhận), Receiver_RX=GPIO17 (ESP gửi telemetry)
// → trùng với pin_ser0 trong default board
// pin_ser0_rx  18    ← default đã có
// pin_ser0_tx  17    ← default đã có
rcl_gizmo     CRSF
rcl_ser_bus   0         // trùng default
rcl_num_ch    8         // số kênh RC

// --- Motor Outputs (ESC 40A) ---
// GPIO4, 5, 6, 7 → trùng với default (pin_out0..3 = 4,5,6,7)
// Không cần override, default đã đúng

// --- AHRS ---
ahr_gizmo     MAHONY    // Mahony filter (ổn định, mặc định)
imu_align     CW0       // Không xoay (nếu IMU gắn đúng hướng đầu máy bay)

)""; // End of madflight_config


//========================================================================================================================//
//                                               COMPILER OPTIONS
//========================================================================================================================//

// Xóa EEPROM config về mặc định (uncomment, upload, chạy xong, comment lại rồi upload lại)
//#define MF_CONFIG_CLEAR

// Giảm startup delay xuống 1.2s (dùng khi debug, TẮT khi bay thật)
//#define MF_DEBUG
