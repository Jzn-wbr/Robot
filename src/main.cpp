#include <Arduino.h>
#include <Wire.h>

constexpr unsigned long BAUDRATE = 115200;

// PWM
constexpr uint8_t CHANNEL_0 = 0;
constexpr uint8_t CHANNEL_1 = 1;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;

// ESP32 pins
namespace PIN
{
  // Driver
  constexpr uint8_t STBY = 26;
  constexpr uint8_t PWMA = 25;
  constexpr uint8_t AIN1 = 33;
  constexpr uint8_t AIN2 = 32;
  constexpr uint8_t PWMB = 27;
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

public:
  Motor() : mode{Motor_mode::FREEWHEEL}, pwm{0} {}

  void set_pwm(uint8_t pwm);
  void set_mode(Motor_mode mode);

  uint8_t get_pwm() const;
  Motor_mode get_mode() const;
};

struct Mpu_data
{
  int16_t ax, ay, az; // Acceleration
  int16_t gx, gy, gz; // Angular velocity
};

// IMU sensor (accelerometer + gyroscope) communicating over the I2C bus.
// SDA/SCL carry digital data between ESP32 and the chip.
// The INT pin can generate a interrupt when new data is ready.
class MPU6050
{
  Mpu_data last_data;

public:
  MPU6050() : last_data{} {}

  void read();

  Mpu_data get_data() const;
};

// Ultrasonic distance sensor.
// TRIG: ESP32 sends a 10 µs pulse to start an ultrasonic burst.
// ECHO: sensor outputs a HIGH pulse whose duration equals the echo
//       return time.
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

  void request_scan();

  void calculate_distance(); // TODO: ignore absurd data and noise

  float get_distance_cm() const;

  void set_echo_time_rise_us(uint32_t time);
  uint32_t get_echo_time_rise_us() const;
  void set_echo_total_duration_us(uint32_t time);

  void set_new_measure_flag();
  bool get_new_measure_flag() const;
};

class Robot
{
  Motor left_motor;
  Motor right_motor;
  MPU6050 mpu;
  HCSR04 radar;

public:
  void free_wheel();
  void move_forward();
  void move_backward();
  void turn_right();
  void turn_left();
  void brake();

  void read_mpu();
  void request_radar_scan();
  void update_radar_distance();

  float get_distance_cm() const;
  Mpu_data get_mpu_data() const;

  void on_echo_rise(uint32_t time);
  void on_echo_fall(uint32_t time);
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
void IRAM_ATTR echo_change();

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
}

// Run continously
void loop()
{
}
