/*#########################################################################################################################

Quadcopter Flight Controller - ESP32-S3 N16R8
Based on madflight - https://madflight.com

###########################################################################################################################

Hardware Configuration:

    MCU:        ESP32-S3 N16R8 (16MB Flash QIO, 8MB OPI PSRAM)
    IMU:        MPU-9265 / MPU-9250 (I2C Bus 1)
                  VCC -> 3.3V  |  GND -> GND
                  SCL -> GPIO13 | SDA -> GPIO11
                  INT -> GPIO14 | ADO -> GND (via GPIO12, driven LOW in setup)
                  NCS -> 3.3V  (via GPIO10, madflight drives HIGH for I2C mode)
    Barometer:  BMP280 (I2C Bus 0)
                  VCC -> 3.3V  | GND -> GND
                  SCL -> GPIO9 | SDA -> GPIO8
    GPS:        NEO-6M V2 (Serial Bus 1)
                  VCC -> 3.3V  | GND -> GND
                  TX  -> GPIO3 | RX  -> GPIO46
    Receiver:   ELRS/CRSF (Serial Bus 0)
                  VCC -> 3.3V  | GND -> GND
                  TX  -> GPIO18 | RX  -> GPIO17
    Motors:     4x BLDC 2212 via ESC 40A
                  Motor1 (back-right  CW)  -> GPIO4  (pin_out0)
                  Motor2 (front-right CCW) -> GPIO5  (pin_out1)
                  Motor3 (back-left   CCW) -> GPIO6  (pin_out2)
                  Motor4 (front-left  CW)  -> GPIO7  (pin_out3)

Arming/disarming with dedicated switch

    Arm: Set throttle low, then flip arm switch from DISARMED to ARMED.
    Disarm: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch".

Arming/disarming with sticks (when no arm switch is defined, i.e. cfg.rcl_arm_ch == 0)

    Arm: Pull both sticks toward you, yaw full right, and roll full left
    Disarm: Pull both sticks toward you, yaw full left, and roll full right

LED State                              Meaning
---------                              -------
OFF                                    Not powered
ON (blue)                              Startup (don't move, running gyro calibration)
Blinking long OFF short ON (green)     DISARMED
Blinking long ON short OFF (red)       ARMED
Blink interval longer than 1 second    imu_loop() is taking too much time
Fast blinking                          Something is wrong, connect USB serial for info

MIT license - Copyright (c) 2023-2026 https://madflight.com
##########################################################################################################################*/

#include "madflight_config.h" //Edit this header file to setup the pins, hardware, radio, etc. for madflight
#include <madflight.h>

//prototypes (for PlatformIO, not needed for Arduino IDE)
void led_Blink();
float degreeModulus(float v);
void control_Angle(bool zero_integrators);
void control_Rate(bool zero_integrators);
void out_KillSwitchAndFailsafe();
void out_Mixer();
void altEstimator_Feed();
void control_FlightMode();
void control_AltHold(bool zero_integrators);
void control_PosHold(bool zero_integrators);

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

//IMPORTANT: This is a safety feature which keeps props spinning when armed, and hopefully reminds the pilot to disarm!!! 
const float armed_min_throttle = 0.20; //Minimum throttle when armed, set to a value between ~0.10 and ~0.25 which keeps the props spinning at minimum speed.

//Flight Mode for mode 0 (default, no alt/pos hold): Uncommment only one
#define FLIGHTMODE_RATE   //control rate - stick centered will keep current roll/pitch angle
//#define FLIGHTMODE_ANGLE  //control angle - stick centered will return to horizontal - IMPORTANT: execute CLI 'calimu' and 'save' before using this!!!

//========================================================================================================================//
//                                     ALTITUDE HOLD & POSITION HOLD CONFIG                                               //
//========================================================================================================================//

//Feature enable/disable - comment out to completely remove feature from build
#define FEATURE_ALT_HOLD
#define FEATURE_POS_HOLD

//Flight mode mapping: rcl.flightmode values from RC switch (0-5)
//  Mode 0: RATE or ANGLE only (selected by FLIGHTMODE_RATE/ANGLE above)
//  Mode 1: RATE + ALT_HOLD
//  Mode 2: ANGLE + ALT_HOLD + POS_HOLD (IMPORTANT: execute CLI 'calimu' and 'save' before using!)
const uint8_t FM_RATE      = 0;
const uint8_t FM_ALT_HOLD  = 1;
const uint8_t FM_POS_HOLD  = 2;

#ifdef FEATURE_ALT_HOLD
//Altitude Hold parameters
const float alt_hold_deadband   = 0.10;   //Throttle stick deadband around center (±0.10 means 0.40-0.60 of stick range)
const float alt_hold_max_vz     = 2.0;    //Maximum climb/descent rate when stick outside deadband [m/s]
const float alt_hold_hover_thr  = 0.45;   //Estimated hover throttle (0.0-1.0) - TUNE to your quad's weight!
const float alt_hold_thr_max_correction = 0.30; //Maximum throttle correction from PID (±0.30)
const uint32_t bar_stale_timeout_us = 1000000;  //Barometer stale timeout [us] - disable alt hold if no update for 1s

//PID Altitude (outer loop): altitude error [m] -> target vertical velocity [m/s]
const float Kp_alt   = 1.0;
const float Ki_alt   = 0.15;
const float Kd_alt   = 0.0;     //Set 0: inner velocity loop provides damping
const float imax_alt = 2.0;     //Integrator limit [m/s]

//PID Vertical Velocity (inner loop): velocity error [m/s] -> throttle correction
const float Kp_vz    = 0.25;
const float Ki_vz    = 0.10;
const float Kd_vz    = 0.01;
const float imax_vz  = 0.20;    //Integrator limit (fraction of throttle)
#endif

#ifdef FEATURE_POS_HOLD
//Position Hold (Anti-Drift) parameters
const float pos_hold_deadband    = 0.05;   //Roll/pitch stick deadband (±0.05)
const float pos_hold_max_angle   = 10.0;   //Maximum correction angle [degrees] - must be less than maxRoll/maxPitch
const int   pos_hold_min_sats    = 6;      //Minimum GPS satellites required
const uint32_t gps_stale_timeout_ms = 500; //Maximum allowed age of GPS solution [ms]
const float pos_hold_gps_lpf_freq = 2.0;   //Low-pass filter cutoff for GPS velocity [Hz]

//PID Position (velocity-based): GPS velocity error [m/s] -> correction angle [degrees]
const float Kp_pos   = 5.0;     //5 deg per 1 m/s of drift
const float Ki_pos   = 0.5;     //Compensate sustained wind
const float Kd_pos   = 0.2;
const float imax_pos = 8.0;     //Integrator limit [degrees]
#endif

//Controller parameters (take note of defaults before modifying!): 
const float i_limit        = 25.0;      //Integrator saturation level, mostly for safety (default 25.0)
const float maxRoll        = 30.0;      //Max roll angle in degrees for angle mode (maximum ~70 degrees)
const float maxPitch       = 30.0;      //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
const float maxRollRate    = 60.0;      //Max roll rate in deg/sec for rate mode 
const float maxPitchRate   = 60.0;      //Max pitch rate in deg/sec for rate mode
const float maxYawRate     = 160.0;     //Max yaw rate in deg/sec for angle and rate mode

//PID Angle Mode 
const float Kp_ro_pi_angle  = 0.2;      //Roll/Pitch P-gain
const float Ki_ro_pi_angle  = 0.1;      //Roll/Pitch I-gain
const float Kd_ro_pi_angle  = 0.05;     //Roll/Pitch D-gain
const float Kp_yaw_angle    = 0.6;      //Yaw P-gain
const float Kd_yaw_angle    = 0.1;      //Yaw D-gain

//PID Rate Mode 
const float Kp_ro_pi_rate   = 0.15;     //Roll/Pitch rate P-gain
const float Ki_ro_pi_rate   = 0.2;      //Roll/Pitch rate I-gain
const float Kd_ro_pi_rate   = 0.0002;   //Roll/Pitch rate D-gain (be careful when increasing too high, motors will begin to overheat!)
const float Kp_yaw_rate     = 0.3;      //Yaw rate P-gain
const float Ki_yaw_rate     = 0.05;     //Yaw rate I-gain
const float Kd_yaw_rate     = 0.00015;  //Yaw rate D-gain (be careful when increasing too high, motors will begin to overheat!)

//Yaw to keep in ANGLE mode when yaw stick is centered
float yaw_desired = 0;

//========================================================================================================================//
//                                     ALTITUDE HOLD & POSITION HOLD STATE                                                //
//========================================================================================================================//

#ifdef FEATURE_ALT_HOLD
PIDController pid_alt_outer;   //Altitude PID: error [m] -> target vz [m/s]
PIDController pid_alt_inner;   //Velocity PID: error [m/s] -> throttle correction
float alt_target = 0;          //Target altitude [m]
bool alt_hold_active = false;  //True when alt hold PID is actually controlling throttle
bool alt_hold_engaged = false; //True when throttle stick is in deadband (holding altitude)
uint32_t bar_ts_prev = 0;     //Previous bar.ts to detect new barometer samples
#endif

#ifdef FEATURE_POS_HOLD
PIDController pid_pos_forward; //Forward velocity PID: error [m/s] -> pitch correction [deg]
PIDController pid_pos_right;   //Right velocity PID: error [m/s] -> roll correction [deg]
FilterLowPass lpf_vel_n;      //Low-pass filter for GPS velocity north
FilterLowPass lpf_vel_e;      //Low-pass filter for GPS velocity east
bool pos_hold_active = false;  //True when position hold is actually correcting
bool pos_hold_engaged = false; //True when roll/pitch sticks are in deadband
#endif

uint8_t flightmode_prev = 255; //Previous flight mode for transition detection (init to invalid)

//========================================================================================================================//
//                                                       SETUP()                                                          //
//========================================================================================================================//

void setup() {
  // =========================================================
  // Hardware-specific pin init BEFORE madflight_setup()
  // =========================================================

  // MPU-9265 ADO pin (GPIO12): kéo xuống LOW để chọn I2C address 0x68
  // ADO=LOW -> 0x68 (default), ADO=HIGH -> 0x69
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  // MPU-9265 NCS pin (GPIO10): kéo lên HIGH để enable I2C mode (thay vì SPI)
  // madflight cũng làm điều này nhưng chủ động set sớm để đảm bảo
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  // =========================================================
  // Setup madflight modules, start madflight RTOS tasks
  // =========================================================
  madflight_setup();

  // Motor outputs: {0,1,2,3} tương ứng pin_out0..3 = GPIO4..7
  int motor_outputs[] = {0, 1, 2, 3}; // back-right, front-right, back-left, front-left

  // Chọn ĐÚNG MỘT protocol ESC (uncomment dòng phù hợp với ESC của bạn):
  bool success = out.setup_motors(4, motor_outputs, 400, 950, 2000);   // Standard PWM: 400Hz, 950-2000us ← ESC 40A thông thường
  //bool success = out.setup_motors(4, motor_outputs, 2000, 125, 250); // Oneshot125: 2000Hz, 125-250us
  //bool success = out.setup_dshot(4, motor_outputs, 300);             // DShot300 (ESC phải hỗ trợ DShot)
  //bool success = out.setup_dshot_bidir(4, motor_outputs, 300);       // DShot300 Bi-Directional (RPM telemetry)

  out.print(); // In thông tin cấu hình motor ra Serial
  if(!success) madflight_panic("Motor init failed.");

  // Lưu yaw hiện tại làm setpoint ban đầu (dùng cho ANGLE mode)
  yaw_desired = ahr.yaw;

  //Initialize altitude hold PIDs
  #ifdef FEATURE_ALT_HOLD
    pid_alt_outer.begin(Kp_alt, Ki_alt, Kd_alt, imax_alt);
    pid_alt_inner.begin(Kp_vz, Ki_vz, Kd_vz, imax_vz);
    alt_target = alt.getH();
  #endif

  //Initialize position hold PIDs and filters
  #ifdef FEATURE_POS_HOLD
    pid_pos_forward.begin(Kp_pos, Ki_pos, Kd_pos, imax_pos);
    pid_pos_right.begin(Kp_pos, Ki_pos, Kd_pos, imax_pos);
    lpf_vel_n.begin(50.0, pos_hold_gps_lpf_freq); //50Hz schedule rate, 2Hz cutoff
    lpf_vel_e.begin(50.0, pos_hold_gps_lpf_freq);
  #endif

  Serial.println("Setup completed, CLI started - Type 'help' for help, or 'diff' to debug");
}

//========================================================================================================================//
//                                                            LOOP()                                                      //
//========================================================================================================================//

void loop() {
  // Nothing to do here for madflight, you can add your code here.
  delay(1000); //this delay() prevents empty loop wasting processor time, give this time to other tasks
}

//========================================================================================================================//
//                                                   IMU UPDATE LOOP                                                      //
//========================================================================================================================//

//This is __MAIN__ function of this program. It is called when new IMU data is available.
void imu_loop() {
  //Blink LED
  led_Blink();

  //Sensor fusion: update ahr.roll, ahr.pitch, and ahr.yaw angle estimates (degrees) from IMU data
  ahr.update(); 

  //Feed altitude estimator with IMU vertical accel + new barometer samples (every cycle)
  altEstimator_Feed();

  //PID Controller - dispatched by runtime flight mode (rcl.flightmode)
  control_FlightMode();

  //Updates out.arm, the output armed flag
  out_KillSwitchAndFailsafe(); //Cut all motor outputs if DISARMED or failsafe triggered.

  //Actuator mixing
  out_Mixer(); //Mixes PID outputs and sends command pulses to the motors, if mot.arm == true
}

//========================================================================================================================
//                      IMU UPDATE LOOP FUNCTIONS - in same order as they are called from imu_loop()
//========================================================================================================================

void led_Blink() {
  //Blink LED once per second, if LED blinks slower then the loop takes too much time, use CLI 'pimu' to investigate.
  //DISARMED: green long off, short on, ARMED: red long on, short off
  uint32_t modulus = imu.update_cnt % imu.getSampleRate();
  if( modulus == 0) led.color( (out.armed() ? 0 : 0x00ff00) ); //start of pulse - armed: off, disarmed: green
  if( modulus == imu.getSampleRate() / 10)  led.color( (out.armed() ? 0xff0000 : 0) ); //end of pulse - armed: red, disarmed: off
}

//returns angle in range -180 to 180
float degreeModulus(float v) {
  if(v >= 180) {
    return fmod(v + 180, 360) - 180;
  }else if(v < -180.0) {
    return fmod(v - 180, 360) + 180;
  }
  return v;
}

void control_Angle(bool zero_integrators) {
  //DESCRIPTION: Computes control commands based on angle error
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des. Error
   * is simply the desired state minus the actual state (ex. roll_des - ahr.roll). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle... saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables pid.roll, pid.pitch, and pid.yaw which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in out_Mixer().
   */ 

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: pid.roll, pid.pitch, pid.yaw

  //desired values
  float roll_des = rcl.roll * maxRoll; //Between -maxRoll and +maxRoll
  float pitch_des = rcl.pitch * maxPitch; //Between -maxPitch and +maxPitch
  float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate roll_PIDand +maxYawRate

  //state vars
  static float integral_roll, integral_pitch, error_yawRate_prev, integral_yawRate;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yawRate = 0;
  }

  //Roll PID
  float error_roll = roll_des - ahr.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahr.gx;
  pid.roll = 0.01 * (Kp_ro_pi_angle*error_roll + Ki_ro_pi_angle*integral_roll - Kd_ro_pi_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch PID
  float error_pitch = pitch_des - ahr.pitch;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = ahr.gy; 
  pid.pitch = 0.01 * (Kp_ro_pi_angle*error_pitch + Ki_ro_pi_angle*integral_pitch - Kd_ro_pi_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw PID
  if(-0.02 < rcl.yaw && rcl.yaw < 0.02) {
    //on reset, set desired yaw to current yaw
    if(zero_integrators) yaw_desired = ahr.yaw; 

    //Yaw stick centered: hold yaw_desired
    float error_yaw = degreeModulus(yaw_desired - ahr.yaw);
    float desired_yawRate = error_yaw / 0.5; //set desired yawRate such that it gets us to desired yaw in 0.5 second
    float derivative_yaw = desired_yawRate - ahr.gz;
    pid.yaw = 0.01 * (Kp_yaw_angle*error_yaw + Kd_yaw_angle*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //update yaw rate controller
    error_yawRate_prev = 0;
  }else{
    //Yaw stick not centered: stablize on rate from GyroZ
    float error_yawRate = yawRate_des - ahr.gz;
    integral_yawRate += error_yawRate * imu.dt;
    integral_yawRate = constrain(integral_yawRate, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
    float derivative_yawRate = (error_yawRate - error_yawRate_prev) / imu.dt; 
    pid.yaw = 0.01 * (Kp_yaw_rate*error_yawRate + Ki_yaw_rate*integral_yawRate + Kd_yaw_rate*derivative_yawRate); //Scaled by .01 to bring within -1 to 1 range

    //Update derivative variables
    error_yawRate_prev = error_yawRate;

    //update yaw controller: 
    yaw_desired = ahr.yaw; //set desired yaw to current yaw, the yaw angle controller will hold this value
  }
}

void control_Rate(bool zero_integrators) {
  //Computes control commands based on state error (rate)
  //See explanation for control_Angle(). Everything is the same here except the error is now: desired rate - raw gyro reading.

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: pid.roll, pid.pitch, pid.yaw

  //desired values
  float rollRate_des = rcl.roll * maxRollRate; //Between -maxRoll and +maxRoll
  float pitchRate_des = rcl.pitch * maxPitchRate; //Between -maxPitch and +maxPitch
  float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate and +maxYawRate 
  
  //state vars
  static float integral_roll, error_roll_prev;
  static float integral_pitch, error_pitch_prev;
  static float integral_yaw, error_yaw_prev;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll
  float error_roll = rollRate_des - ahr.gx;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (error_roll - error_roll_prev) / imu.dt;
  pid.roll = 0.01 * (Kp_ro_pi_rate*error_roll + Ki_ro_pi_rate*integral_roll + Kd_ro_pi_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  float error_pitch = pitchRate_des - ahr.gy;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = (error_pitch - error_pitch_prev) / imu.dt;   
  pid.pitch = 0.01 * (Kp_ro_pi_rate*error_pitch + Ki_ro_pi_rate*integral_pitch + Kd_ro_pi_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  float error_yaw = yawRate_des - ahr.gz;
  integral_yaw += error_yaw * imu.dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt; 
  pid.yaw = 0.01 * (Kp_yaw_rate*error_yaw + Ki_yaw_rate*integral_yaw + Kd_yaw_rate*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update derivative variables
  error_roll_prev = error_roll;
  error_pitch_prev = error_pitch;
  error_yaw_prev = error_yaw;
}

//========================================================================================================================//
//                              ALTITUDE HOLD & POSITION HOLD FUNCTIONS                                                   //
//========================================================================================================================//

void altEstimator_Feed() {
  //DESCRIPTION: Feed the Kalman altitude estimator every IMU cycle.
  //Barometer update is only fed when bar.ts changes (new sample available), to avoid feeding stale data.
  //getAccelUp() returns earth-frame vertical acceleration in m/s^2 (gravity removed by ahrs quaternion rotation).
  #ifdef FEATURE_ALT_HOLD
  alt.updateAccelUp(ahr.getAccelUp(), imu.ts);
  if(bar.ts != bar_ts_prev) {
    bar_ts_prev = bar.ts;
    alt.updateBarAlt(bar.alt, bar.ts);
  }
  #endif
}

void control_FlightMode() {
  //DESCRIPTION: Runtime flight mode dispatcher based on rcl.flightmode RC switch value.
  //Mode transitions: reset all new PID integrators to prevent sudden jumps.
  //Mode 0 = RATE or ANGLE (compile-time selection, original behavior)
  //Mode 1 = RATE + ALT_HOLD
  //Mode 2 = ANGLE + ALT_HOLD + POS_HOLD

  bool zero_int = (rcl.throttle == 0);

  //Detect mode transition: reset new PIDs, re-capture targets
  if(rcl.flightmode != flightmode_prev) {
    #ifdef FEATURE_ALT_HOLD
      pid_alt_outer.reset();
      pid_alt_inner.reset();
      alt_target = alt.getH();           //Re-capture current altitude as target on mode entry
      alt_hold_active   = false;
      alt_hold_engaged  = false;
    #endif
    #ifdef FEATURE_POS_HOLD
      pid_pos_forward.reset();
      pid_pos_right.reset();
      pos_hold_active  = false;
      pos_hold_engaged = false;
    #endif
    flightmode_prev = rcl.flightmode;
  }

  #ifdef FEATURE_ALT_HOLD
  if(rcl.flightmode == FM_ALT_HOLD) {
    //--- Mode 1: RATE + Altitude Hold ---
    control_Rate(zero_int);
    control_AltHold(zero_int);
    return;
  }
  if(rcl.flightmode == FM_POS_HOLD) {
    //--- Mode 2: ANGLE + Altitude Hold + Position Hold ---
    control_Angle(zero_int);
    control_AltHold(zero_int);
    #ifdef FEATURE_POS_HOLD
    control_PosHold(zero_int);
    #endif
    return;
  }
  #endif

  //--- Mode 0 (default): original RATE or ANGLE compile-time selection ---
  #ifdef FEATURE_ALT_HOLD
    alt_hold_active  = false;
  #endif
  #ifdef FEATURE_POS_HOLD
    pos_hold_active  = false;
  #endif
  #ifdef FLIGHTMODE_ANGLE
    control_Angle(zero_int);
  #else
    control_Rate(zero_int);
  #endif
}

#ifdef FEATURE_ALT_HOLD
void control_AltHold(bool zero_integrators) {
  //DESCRIPTION: Cascaded altitude hold PID controller running at 100Hz via MF_Schedule.
  //Outer loop: altitude error [m] -> target vertical velocity [m/s]
  //Inner loop: velocity error [m/s] -> throttle correction (stored in pid.throttle)
  //
  //Throttle stick deadband:
  //  Center (±alt_hold_deadband): hold current altitude
  //  Outside deadband: climb/descend proportionally, then re-capture altitude when stick returns to center
  //
  //Safety: if barometer goes stale (no update for bar_stale_timeout_us), disable alt hold and let pilot control

  static MF_Schedule alt_schedule;
  static float target_vz = 0;  //Current target vertical velocity [m/s]

  //Run at 100Hz (every 10ms = 10000us)
  if(!alt_schedule.interval(10000)) return;

  //dt for this PID loop (nominally 0.01s)
  const float dt = 0.01f;

  //Safety: reset integrators when throttle is idle
  if(zero_integrators) {
    pid_alt_outer.reset();
    pid_alt_inner.reset();
    pid.throttle   = 0;
    alt_hold_active   = false;
    alt_hold_engaged  = false;
    alt_target = alt.getH();
    return;
  }

  //Safety: check barometer freshness. bar.ts is in microseconds.
  bool bar_ok = (micros() - bar.ts < bar_stale_timeout_us);
  if(!bar_ok) {
    //Barometer stale: gracefully disengage alt hold, give throttle back to pilot
    pid_alt_outer.reset();
    pid_alt_inner.reset();
    pid.throttle  = 0;
    alt_hold_active  = false;
    alt_hold_engaged = false;
    return;
  }

  //Throttle stick position: normalised 0.0-1.0, center ~0.5
  float stick_center_dist = rcl.throttle - 0.5f;  //Positive = up, negative = down

  if(fabsf(stick_center_dist) <= alt_hold_deadband) {
    //Stick in deadband: hold altitude
    if(!alt_hold_engaged) {
      //Just entered deadband: capture current altitude as new target
      alt_target       = alt.getH();
      alt_hold_engaged = true;
      target_vz        = 0;
    }
    //Outer PID: altitude error -> target vz (D-term uses measured climb rate)
    target_vz = pid_alt_outer.controlActualDerivative(alt_target, alt.getH(), dt, alt.getV());
    target_vz = constrain(target_vz, -alt_hold_max_vz, alt_hold_max_vz);
  } else {
    //Stick outside deadband: proportional climb/descent rate
    alt_hold_engaged = false;
    pid_alt_outer.reset();  //Don't wind up while pilot is controlling climb rate
    //Scale deadband-corrected stick throw to ±alt_hold_max_vz
    float throw_normalized = (stick_center_dist - (stick_center_dist > 0 ? alt_hold_deadband : -alt_hold_deadband));
    target_vz = (throw_normalized / (0.5f - alt_hold_deadband)) * alt_hold_max_vz;
    target_vz = constrain(target_vz, -alt_hold_max_vz, alt_hold_max_vz);
    //Re-capture altitude continuously so when stick returns to center, we hold the last altitude
    alt_target = alt.getH();
  }

  //Inner PID: velocity error -> throttle correction
  //Use measured vertical acceleration (getAccelUp) as actual derivative of vertical speed
  float accel_up    = ahr.getAccelUp();  //m/s^2, earth-frame up
  pid.throttle = pid_alt_inner.controlActualDerivative(target_vz, alt.getV(), dt, accel_up);
  pid.throttle = constrain(pid.throttle, -alt_hold_thr_max_correction, alt_hold_thr_max_correction);
  alt_hold_active = true;
}
#else
void control_AltHold(bool zero_integrators) { (void)zero_integrators; }
#endif  //FEATURE_ALT_HOLD

#ifdef FEATURE_POS_HOLD
void control_PosHold(bool zero_integrators) {
  //DESCRIPTION: GPS-based position hold (anti-drift) running at 50Hz via MF_Schedule.
  //Uses GPS velocity (NED mm/s) rotated to body frame via yaw angle.
  //PID: target velocity = 0, actual = body-frame velocity -> correction angle added to pid.pitch & pid.roll.
  //Limited to ±pos_hold_max_angle degrees; gate requires GPS 3D fix + enough satellites.
  //
  //Safety: stick override (|roll| or |pitch| outside deadband -> disengage), GPS lost -> reset, no dynamic allocation.

  static MF_Schedule pos_schedule;

  //Run at 50Hz (every 20ms = 20000us)
  if(!pos_schedule.interval(20000)) return;

  const float dt = 0.02f;

  //Safety: reset integrators when throttle is idle
  if(zero_integrators) {
    pid_pos_forward.reset();
    pid_pos_right.reset();
    pos_hold_active  = false;
    pos_hold_engaged = false;
    return;
  }

  //Gate: require fresh 3D GPS fix with minimum satellites
  bool gps_ok = (gps.fix >= GPS_OK_FIX_3D) &&
                (gps.sat >= pos_hold_min_sats) &&
                ((millis() - gps.last_gps_time_ms) < gps_stale_timeout_ms);
  if(!gps_ok) {
    if(pos_hold_active) {
      pid_pos_forward.reset();
      pid_pos_right.reset();
      pos_hold_active  = false;
      pos_hold_engaged = false;
    }
    return;
  }

  //Stick override: if roll or pitch stick outside deadband, let pilot control
  if(fabsf(rcl.roll) > pos_hold_deadband || fabsf(rcl.pitch) > pos_hold_deadband) {
    pid_pos_forward.reset();
    pid_pos_right.reset();
    pos_hold_active  = false;
    pos_hold_engaged = false;
    return;
  }
  pos_hold_engaged = true;

  //GPS velocity NED [m/s] (gps.veln/vele are int32_t mm/s)
  float vel_n = lpf_vel_n.update((float)gps.veln * 0.001f, dt);  //North positive
  float vel_e = lpf_vel_e.update((float)gps.vele * 0.001f, dt);  //East  positive

  //Rotate NED velocity to body frame using yaw angle (degrees -> radians)
  float yaw_rad    = ahr.yaw * (float)(M_PI / 180.0);
  float cos_yaw    = cosf(yaw_rad);
  float sin_yaw    = sinf(yaw_rad);
  float vel_fwd    =  vel_n * cos_yaw + vel_e * sin_yaw;  //Forward velocity in body frame
  float vel_right  = -vel_n * sin_yaw + vel_e * cos_yaw;  //Right velocity in body frame

  //PID: desired velocity = 0, actual = body-frame drift velocity -> correction angle [degrees]
  //Positive forward drift -> negative pitch (nose down to brake) -> correct sign in mixer
  float pitch_corr = pid_pos_forward.control(0, vel_fwd, dt) * 0.01f;  //output in degrees, scaled to pid units
  float roll_corr  = pid_pos_right.control(0, vel_right, dt) * 0.01f;

  //Clamp to safety limit (convert degrees limit to pid scale: 0.01 factor, so limit = max_angle * 0.01)
  float max_corr = pos_hold_max_angle * 0.01f;
  pitch_corr = constrain(pitch_corr, -max_corr, max_corr);
  roll_corr  = constrain(roll_corr, -max_corr, max_corr);

  //Add correction to existing pid.pitch and pid.roll (pilot stick + attitude PID already in there)
  //PID output is negative when drifting forward (error = 0 - positive_vel < 0), so adding it reduces pid.pitch = pitch backward = brake
  pid.pitch += pitch_corr;
  pid.roll  += roll_corr;
  pid.pitch = constrain(pid.pitch, -1.0f, 1.0f);
  pid.roll  = constrain(pid.roll,  -1.0f, 1.0f);
  pos_hold_active = true;
}
#else
void control_PosHold(bool zero_integrators) { (void)zero_integrators; }
#endif  //FEATURE_POS_HOLD

void out_KillSwitchAndFailsafe() {
  //Change to ARMED when rcl is armed (by switch or stick command)
  if (!out.armed() && rcl.armed) {
    out.set_armed(true);
    Serial.println("OUT: ARMED");
  }

  //Change to DISARMED when rcl is disarmed, or if radio lost connection
  if (out.armed() && (!rcl.armed || !rcl.connected())) {
    out.set_armed(false);
    if(!rcl.armed) {
      Serial.println("OUT: DISARMED");
    }else{
      Serial.println("OUT: DISARMED due to lost radio connection");
    }
  }
}

void out_Mixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes pid.roll, pid.pitch, and pid.yaw computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +pid.roll while the right two motors
   * should have -pid.roll. Front two should have +pid.pitch and the back two should have -pid.pitch etc... every motor has
   * normalized (0 to 1) rcl.throttle command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * rcl.xxx variables are to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *rcl.throtle - direct thottle control
   *pid.roll, pid.pitch, pid.yaw - stabilized axis variables
   *rcl.roll, rcl.pitch, rcl.yaw - direct unstabilized command passthrough
   */
/*
Motor order diagram (Betaflight order)

      front
 CW -->   <-- CCW
     4     2 
      \ ^ /
       |X|
      / - \
     3     1 
CCW -->   <-- CW

                                        M1234
Pitch up (stick back)   (front+ back-)   -+-+
Roll right              (left+ right-)   --++
Yaw right               (CCW+ CW-)       -++-
*/

  // IMPORTANT: This is a safety feature to remind the pilot to disarm.
  // Set motor outputs to at least armed_min_throttle, to keep at least one prop spinning when armed. The [out] module will disable motors when out.armed() == false
  //Compute base throttle: use alt hold estimated hover + pid correction, or direct stick control
  float thr;
  #ifdef FEATURE_ALT_HOLD
  if(alt_hold_active && rcl.throttle > 0) {
    //Alt hold active: use estimated hover throttle + PID correction.
    //pid.throttle is constrained to ±alt_hold_thr_max_correction by control_AltHold().
    thr = constrain(alt_hold_hover_thr + pid.throttle, armed_min_throttle, 1.0f);
  } else {
    thr = armed_min_throttle + (1 - armed_min_throttle) * rcl.throttle; //shift motor throttle range from [0.0 .. 1.0] to [armed_min_throttle .. 1.0]
  }
  #else
  thr = armed_min_throttle + (1 - armed_min_throttle) * rcl.throttle; //shift motor throttle range from [0.0 .. 1.0] to [armed_min_throttle .. 1.0]
  #endif

  if(rcl.throttle == 0) {
    //if throttle idle, then run props at low speed without applying PID. This allows for stick commands for arm/disarm.
    out.set_output(0, thr);
    out.set_output(1, thr);
    out.set_output(2, thr);
    out.set_output(3, thr);
  }else{
    // Quad mixing
    out.set_output(0, thr - pid.pitch - pid.roll - pid.yaw); //M1 Back Right CW
    out.set_output(1, thr + pid.pitch - pid.roll + pid.yaw); //M2 Front Right CCW
    out.set_output(2, thr - pid.pitch + pid.roll + pid.yaw); //M3 Back Left CCW
    out.set_output(3, thr + pid.pitch + pid.roll - pid.yaw); //M4 Front Left CW
  }
}
