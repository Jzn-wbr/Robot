#include <Arduino.h>
#include <Wire.h>

constexpr unsigned long BAUDRATE = 115200;

// PWM
constexpr uint8_t CHANNEL_0 = 0;
constexpr uint8_t CHANNEL_1 = 1;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr uint8_t PWM_FORWARD = 150; // Test and adjust
constexpr uint8_t PWM_TURN = 80;     // Test and adjust
constexpr uint8_t PWM_BACKWARD = 80; // Test and adjust

// MPU6050
constexpr int MPU_ADRESS = 0x68;
constexpr int MPU_POWER_MANAGMENT_REGISTER = 0x6B;

// ESP32 pins
namespace PIN
{
  // Driver
  constexpr uint8_t STBY = 26;

  constexpr uint8_t PWMA = 25; // Left motor
  constexpr uint8_t AIN1 = 33;
  constexpr uint8_t AIN2 = 32;

  constexpr uint8_t PWMB = 27; // Right motor
  constexpr uint8_t BIN1 = 16;
  constexpr uint8_t BIN2 = 17;

  // HC-SR04
  constexpr uint8_t TRIG = 23;
  constexpr uint8_t ECHO = 34;

  // MPU 6050
  constexpr uint8_t SDA = 21;
  constexpr uint8_t SCL = 22;
  constexpr uint8_t MPU_INT = 35;
}

enum class Robot_state
{
  FREE_WHEEL,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  TURN_RIGHT,
  TURN_LEFT,
  BRAKE
};

enum class Motor_mode
{
  FREEWHEEL,
  FORWARD,
  BACKWARD,
  BRAKE
};

// Represents one DC motor controlled through an H‑bridge driver.
// The ESP32 sends two logic signals (IN1/IN2) to set current direction,
// and a PWM signal to control motor power.
class Motor
{
  Motor_mode mode;
  uint8_t pwm;
  const uint8_t channel;
  const uint8_t IN1, IN2, STBY;

public:
  Motor(uint8_t channel, uint8_t IN1, uint8_t IN2, uint8_t STBY) : mode{Motor_mode::FREEWHEEL},
                                                                   pwm{0}, channel{channel},
                                                                   IN1{IN1}, IN2{IN2},
                                                                   STBY{STBY} {}

  void set_pwm(uint8_t pwm)
  {
    this->pwm = pwm;
    ledcWrite(channel, pwm);
  }

  void set_mode(Motor_mode new_mode)
  {
    if (mode == new_mode)
      return;

    mode = new_mode;

    switch (mode)
    {
    case Motor_mode::FREEWHEEL:
      digitalWrite(STBY, LOW);
      break;

    case Motor_mode::FORWARD:
      ledcWrite(channel, 0);
      digitalWrite(STBY, HIGH);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(channel, pwm);
      break;

    case Motor_mode::BACKWARD:
      ledcWrite(channel, 0);
      digitalWrite(STBY, HIGH);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      ledcWrite(channel, pwm);
      break;

    case Motor_mode::BRAKE:
      digitalWrite(STBY, HIGH);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      break;
    }
  }

  uint8_t get_pwm() const { return pwm; }
  Motor_mode get_mode() const { return mode; }
};

struct Mpu_data
{
  int16_t ax, ay, az; // Acceleration
  int16_t gx, gy, gz; // Angular velocity
  int16_t temperature;
};

// IMU sensor (accelerometer + gyroscope) communicating over the I2C bus.
// SDA/SCL carry digital data between ESP32 and the chip.
// The INT pin can generate a interrupt when new data is ready.
class MPU6050
{
  Mpu_data last_data;

public:
  MPU6050() : last_data{} {}

  void read()
  {
    // I2C communication
    Wire.beginTransmission(MPU_ADRESS);
    Wire.write(0x3B);

    if (Wire.endTransmission(false))
      return;
    uint8_t n = Wire.requestFrom(MPU_ADRESS, 14, true);
    if (Wire.available() != 14 || n != 14)
      return;

    // Read data by byte
    uint8_t ax_h = Wire.read();
    uint8_t ax_l = Wire.read();
    last_data.ax = (ax_h << 8) | ax_l;
    uint8_t ay_h = Wire.read();
    uint8_t ay_l = Wire.read();
    last_data.ay = (ay_h << 8) | ay_l;
    uint8_t az_h = Wire.read();
    uint8_t az_l = Wire.read();
    last_data.az = (az_h << 8) | az_l;

    uint8_t temp_h = Wire.read();
    uint8_t temp_l = Wire.read();
    last_data.temperature = (temp_h << 8) | temp_l;

    uint8_t gx_h = Wire.read();
    uint8_t gx_l = Wire.read();
    last_data.gx = (gx_h << 8) | gx_l;
    uint8_t gy_h = Wire.read();
    uint8_t gy_l = Wire.read();
    last_data.gy = (gy_h << 8) | gy_l;
    uint8_t gz_h = Wire.read();
    uint8_t gz_l = Wire.read();
    last_data.gz = (gz_h << 8) | gz_l;
  }

  Mpu_data get_data() const { return last_data; }
};

// Ultrasonic distance sensor.
// TRIG: ESP32 sends a 10 µs pulse to start an ultrasonic burst.
// ECHO: sensor outputs a HIGH pulse whose duration equals the echo
//       return time.
// TODO: ignore absurd data and noise
class HCSR04
{
  volatile uint32_t echo_time_rise_us;
  volatile uint32_t echo_total_duration_us; // linked to a ISR, time between
                                            // ECHO rising and descending edge
                                            // in micro second
  float distance_cm;
  volatile bool new_measure_ready;

public:
  HCSR04() : echo_time_rise_us{0}, echo_total_duration_us{0}, distance_cm{0.f},
             new_measure_ready{false} {}

  void request_scan()
  {
    digitalWrite(PIN::TRIG, LOW);
    delayMicroseconds(3);
    digitalWrite(PIN::TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN::TRIG, LOW);
  }

  void calculate_distance() { distance_cm = 0.0343 * echo_total_duration_us / 2.f; }

  float get_distance_cm() const { return distance_cm; }

  void set_echo_time_rise_us(uint32_t time) { echo_time_rise_us = time; }
  uint32_t get_echo_time_rise_us() const { return echo_time_rise_us; }
  void set_echo_total_duration_us(uint32_t time) { echo_total_duration_us = time; }

  void set_new_measure_flag(bool flag) { new_measure_ready = flag; }
  bool get_new_measure_flag() const { return new_measure_ready; }
};

class Robot
{
  Motor left_motor;
  Motor right_motor;
  MPU6050 mpu;
  HCSR04 radar;

public:
  Robot() : left_motor{CHANNEL_0, PIN::AIN1, PIN::AIN2, PIN::STBY},
            right_motor{CHANNEL_1, PIN::BIN1, PIN::BIN2, PIN::STBY} {}

  void apply_state(Robot_state state)
  {
    switch (state)
    {
    case Robot_state::FREE_WHEEL:
      left_motor.set_mode(Motor_mode::FREEWHEEL);
      right_motor.set_mode(Motor_mode::FREEWHEEL);
      break;

    case Robot_state::MOVE_FORWARD:
      left_motor.set_mode(Motor_mode::FORWARD);
      right_motor.set_mode(Motor_mode::FORWARD);
      left_motor.set_pwm(PWM_FORWARD);
      right_motor.set_pwm(PWM_FORWARD);
      break;

    case Robot_state::MOVE_BACKWARD:
      left_motor.set_mode(Motor_mode::BACKWARD);
      right_motor.set_mode(Motor_mode::BACKWARD);
      left_motor.set_pwm(PWM_BACKWARD);
      right_motor.set_pwm(PWM_BACKWARD);
      break;

    case Robot_state::TURN_RIGHT:
      left_motor.set_mode(Motor_mode::FORWARD);
      right_motor.set_mode(Motor_mode::BACKWARD);
      left_motor.set_pwm(PWM_TURN);
      right_motor.set_pwm(PWM_TURN);
      break;

    case Robot_state::TURN_LEFT:
      left_motor.set_mode(Motor_mode::BACKWARD);
      right_motor.set_mode(Motor_mode::FORWARD);
      left_motor.set_pwm(PWM_TURN);
      right_motor.set_pwm(PWM_TURN);
      break;

    case Robot_state::BRAKE:
      left_motor.set_mode(Motor_mode::BRAKE);
      right_motor.set_mode(Motor_mode::BRAKE);
      break;
    }
  }

  void read_mpu() { mpu.read(); }
  void request_radar_scan() { radar.request_scan(); }
  void update_radar_distance() { radar.calculate_distance(); }

  float get_distance_cm() const { return radar.get_distance_cm(); }
  Mpu_data get_mpu_data() const { return mpu.get_data(); }

  void on_echo_rise(uint32_t time) { radar.set_echo_time_rise_us(time); }
  void on_echo_fall(uint32_t time)
  {
    radar.set_echo_total_duration_us(time - radar.get_echo_time_rise_us());
    radar.set_new_measure_flag(true);
  }
};

// Read MPU at 100 Hz
// Trigger radar at 10 Hz
// Calculate radar distance after "ECHO" signal is received
class Scheduler
{
  Robot &robot;

  uint32_t last_fetch_accel_ms;
  uint32_t last_scan_radar_ms;

  uint32_t accel_period_ms; // 100 Hz
  uint32_t radar_period_ms; // 10 Hz

public:
  Scheduler(Robot &robot) : robot{robot}, last_fetch_accel_ms{0}, last_scan_radar_ms{0},
                            accel_period_ms{10}, radar_period_ms{100} {}

  void update(uint32_t now_ms);
};

class Controller
{
  Robot &robot;
  Robot_state state;
  uint32_t entered_current_state_at_ms;

public:
  Controller(Robot &robot) : robot{robot}, state{Robot_state::FREE_WHEEL},
                             entered_current_state_at_ms{0} {}

  void think_and_update_robot_state_if_needed(uint32_t now_ms);
};

// Instances
Robot robot;
Scheduler scheduler{robot};
Controller controller{robot};

// ISR
void IRAM_ATTR echo_change()
{
  uint32_t now = micros();
  int v = digitalRead(PIN::ECHO);

  if (v)
  {
    robot.on_echo_rise(now);
  }
  else
  {
    robot.on_echo_fall(now);
  }
}

void gpio_init()
{
  pinMode(PIN::STBY, OUTPUT);
  pinMode(PIN::AIN1, OUTPUT);
  pinMode(PIN::AIN2, OUTPUT);
  pinMode(PIN::BIN1, OUTPUT);
  pinMode(PIN::BIN2, OUTPUT);
  pinMode(PIN::TRIG, OUTPUT);
  pinMode(PIN::ECHO, INPUT);

  // Safe state
  digitalWrite(PIN::STBY, LOW);
  digitalWrite(PIN::AIN1, LOW);
  digitalWrite(PIN::AIN2, LOW);
  digitalWrite(PIN::BIN1, LOW);
  digitalWrite(PIN::BIN2, LOW);
  digitalWrite(PIN::TRIG, LOW);

  // PWM
  ledcSetup(CHANNEL_0, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcAttachPin(PIN::PWMA, CHANNEL_0);
  ledcWrite(CHANNEL_0, 0);
  ledcSetup(CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcAttachPin(PIN::PWMB, CHANNEL_1);
  ledcWrite(CHANNEL_1, 0);

  // ISR
  attachInterrupt(digitalPinToInterrupt(PIN::ECHO), echo_change, CHANGE);
}

// Run once
void setup()
{
  Serial.begin(BAUDRATE);
  delay(100);

  gpio_init();

  Wire.begin(PIN::SDA, PIN::SCL);
  Wire.beginTransmission(MPU_ADRESS);
  Wire.write(MPU_POWER_MANAGMENT_REGISTER);
  Wire.write(0x00); // Wake up
  Wire.endTransmission();
}

// Run continously
void loop()
{
}
