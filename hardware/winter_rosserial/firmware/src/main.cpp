#include "robot_config.h"
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

// Fixed buffers and arrays: no application heap allocation in the control loop.
ros::NodeHandle_<ArduinoHardware, 1, 5, 512, 512> nh;
std_msgs::UInt32 heartbeat;
std_msgs::Float32MultiArray wheel_cmd;
std_msgs::String status;
float wheel_data[4] = {}; // Left/right m/s, then signed electrical PWM duty counts.
ros::Publisher heartbeat_pub("/mcu/heartbeat", &heartbeat);
ros::Publisher wheel_pub("/mcu/wheel_cmd", &wheel_cmd);
ros::Publisher status_pub("/mcu/status", &status);
uint32_t last_cmd_ms = 0;
bool have_command = false;
char ready_text[] = "READY";
char timeout_text[] = "CMD_TIMEOUT";
constexpr int PWM_MAX = (1UL << PWM_RESOLUTION_BITS) - 1;
static_assert(PWM_RESOLUTION_BITS > 0 && PWM_RESOLUTION_BITS <= 16, "Invalid PWM resolution");
static_assert(PWM_MIN_DUTY <= PWM_MAX, "Minimum PWM exceeds full duty");
static_assert(MAX_WHEEL_SPEED_MPS > 0 && TRACK_WIDTH_M > 0, "Invalid chassis geometry/speed");
static_assert(DRIVER_TYPE == DRIVER_PWM_DIR || DRIVER_TYPE == DRIVER_L298N, "Unknown driver");

// A CHANGE gives two counts per A cycle. B determines direction; not closed-loop control.
#if USE_ENCODERS
volatile uint32_t left_ticks = 0, right_ticks = 0; // Unsigned arithmetic wraps predictably.
int32_t tick_data[2] = {};
std_msgs::Int32MultiArray encoder_ticks;
ros::Publisher encoder_pub("/mcu/encoder_ticks", &encoder_ticks);
void leftEncoder() {
  left_ticks += (digitalRead(LEFT_ENCODER_A_PIN) == digitalRead(LEFT_ENCODER_B_PIN)) ? 1U : UINT32_MAX;
}
void rightEncoder() {
  right_ticks += (digitalRead(RIGHT_ENCODER_A_PIN) == digitalRead(RIGHT_ENCODER_B_PIN)) ? 1U : UINT32_MAX;
}
#endif

float clampValue(float value, float low, float high) {
  return fminf(high, fmaxf(low, value));
}

// Remove PWM before changing direction. Zero duty disables drive (coasting depends on driver).
int driveMotor(float speed, uint32_t pwm, uint32_t dir, uint32_t dir2, bool invert) {
  int duty = 0;
  if (fabsf(speed) > PWM_DEADBAND_MPS) {
    duty = lroundf(fabsf(speed) / MAX_WHEEL_SPEED_MPS * PWM_MAX);
    duty = constrain(duty, PWM_MIN_DUTY, PWM_MAX);
  }
  const bool reverse = (speed < 0) != invert;
  analogWrite(pwm, 0);
  digitalWrite(dir, duty && !reverse ? HIGH : LOW);
#if DRIVER_TYPE == DRIVER_L298N
  digitalWrite(dir2, duty && reverse ? HIGH : LOW);
#else
  (void)dir2;
#endif
  analogWrite(pwm, duty);
  return reverse ? -duty : duty;
}

void setWheels(float left, float right) {
  wheel_data[0] = left;
  wheel_data[1] = right;
  wheel_data[2] = driveMotor(left, LEFT_PWM_PIN, LEFT_DIR_PIN, LEFT_DIR2_PIN, LEFT_MOTOR_INVERT);
  wheel_data[3] = driveMotor(right, RIGHT_PWM_PIN, RIGHT_DIR_PIN, RIGHT_DIR2_PIN, RIGHT_MOTOR_INVERT);
}

void stopMotors() {
  have_command = false;
  setWheels(0, 0);
}

void commandCallback(const geometry_msgs::Twist &msg) {
  // Reject nonfinite values in every field, including unused axes.
  if (!nh.connected() || !isfinite(msg.linear.x) || !isfinite(msg.linear.y) ||
      !isfinite(msg.linear.z) || !isfinite(msg.angular.x) ||
      !isfinite(msg.angular.y) || !isfinite(msg.angular.z)) {
    stopMotors();
    return;
  }
  const float v = clampValue(msg.linear.x, ALLOW_REVERSE ? -MAX_LINEAR_MPS : 0, MAX_LINEAR_MPS);
  const float w = clampValue(msg.angular.z, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS);
  float left = v - w * TRACK_WIDTH_M / 2;
  float right = v + w * TRACK_WIDTH_M / 2;
  const float peak = fmaxf(fabsf(left), fabsf(right));
  // Preserve curvature when either side reaches its full-PWM speed.
  if (peak > MAX_WHEEL_SPEED_MPS) {
    left *= MAX_WHEEL_SPEED_MPS / peak;
    right *= MAX_WHEEL_SPEED_MPS / peak;
  }
  setWheels(left, right);
  last_cmd_ms = millis();
  have_command = true;
}
ros::Subscriber<geometry_msgs::Twist> command_sub("/cmd_vel", commandCallback);

void setup() {
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
#if DRIVER_TYPE == DRIVER_L298N
  pinMode(LEFT_DIR2_PIN, OUTPUT);
  pinMode(RIGHT_DIR2_PIN, OUTPUT);
#endif
  pinMode(STATUS_LED_PIN, OUTPUT);
  analogWriteResolution(PWM_RESOLUTION_BITS);
  analogWriteFrequency(PWM_FREQUENCY_HZ);
  stopMotors(); // Safe before starting serial negotiation.
#if USE_ENCODERS
  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), rightEncoder, CHANGE);
  encoder_ticks.data = tick_data;
  encoder_ticks.data_length = 2;
#endif
  wheel_cmd.data = wheel_data;
  wheel_cmd.data_length = 4;
  nh.getHardware()->setBaud(SERIAL_BAUD); // Serial defaults to ST-Link VCP on Nucleo.
  nh.initNode();
  nh.setSpinTimeout(2); // Bound parsing work so the watchdog runs frequently.
  nh.subscribe(command_sub);
  nh.advertise(heartbeat_pub);
  nh.advertise(wheel_pub);
  nh.advertise(status_pub);
#if USE_ENCODERS
  nh.advertise(encoder_pub);
#endif
}

void loop() {
  static uint32_t last_heartbeat_ms = 0, last_status_ms = 0;
  nh.spinOnce();
  const uint32_t now = millis(); // Unsigned subtraction also works across millis rollover.
  // Stop once on timeout or link loss; motors are already zero when there is no active command.
  if (have_command && (!nh.connected() || now - last_cmd_ms >= CMD_VEL_TIMEOUT_MS)) {
    stopMotors();
  }
  if (now - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS) {
    last_heartbeat_ms = now;
    digitalWrite(STATUS_LED_PIN, (++heartbeat.data & 1U) ? HIGH : LOW);
    if (nh.connected()) {
      heartbeat_pub.publish(&heartbeat);
      wheel_pub.publish(&wheel_cmd);
#if USE_ENCODERS
      noInterrupts(); // Copy both counters atomically; publish outside the interrupt mask.
      tick_data[0] = static_cast<int32_t>(left_ticks);
      tick_data[1] = static_cast<int32_t>(right_ticks);
      interrupts();
      encoder_pub.publish(&encoder_ticks);
#endif
    }
  }
  if (nh.connected() && now - last_status_ms >= 500) {
    last_status_ms = now;
    status.data = have_command ? ready_text : timeout_text;
    status_pub.publish(&status);
  }
}
