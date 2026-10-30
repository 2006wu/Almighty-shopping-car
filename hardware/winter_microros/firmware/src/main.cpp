#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int64_multi_array.h>
#include "robot_config.h"

// Cleanup calls may fail when the agent is already gone; their result is intentionally unused.
#define IGNORE_RC(call) do { rcl_ret_t ignored_rc = (call); (void)ignored_rc; } while (0)

// micro-ROS/newlib may reference printf-style output; discard it because Serial carries XRCE frames.
extern "C" int _write(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  return len;
}

constexpr uint32_t PWM_MAX = (1UL << PWM_RESOLUTION_BITS) - 1;
static_assert(PWM_RESOLUTION_BITS > 0 && PWM_RESOLUTION_BITS <= 16, "Invalid PWM resolution");
static_assert(MAX_WHEEL_SPEED_MPS > 0 && TRACK_WIDTH_M > 0, "Invalid chassis geometry/speed");
static_assert(PWM_MIN_DUTY <= PWM_MAX, "Invalid minimum PWM");
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t cmd_sub;
rcl_publisher_t hb_pub, status_pub, wheel_pub;
rclc_executor_t executor;
// Track partial initialization so failed creation can also be cleaned up.
unsigned created = 0;
geometry_msgs__msg__Twist command = {};
std_msgs__msg__UInt32 heartbeat = {};
std_msgs__msg__String status = {};
std_msgs__msg__Float32MultiArray wheel = {};
char status_buffer[16];
float wheel_data[4] = {};
bool command_fresh = false;
uint32_t last_cmd = 0, last_ping = 0, last_hb = 0, last_status = 0, last_blink = 0;
bool led = false;
#if USE_ENCODERS
rcl_publisher_t encoder_pub;
std_msgs__msg__Int64MultiArray encoder = {};
volatile int64_t left_ticks = 0, right_ticks = 0;
int64_t encoder_data[2];
// Count only A rising edges (1x); swapping A/B reverses the raw tick sign.
void left_encoder() { left_ticks += digitalRead(LEFT_ENCODER_B_PIN) ? -1 : 1; }
void right_encoder() { right_ticks += digitalRead(RIGHT_ENCODER_B_PIN) ? -1 : 1; }
#endif

// Remove PWM before changing direction to avoid a powered polarity change.
uint32_t drive_motor(float speed, uint32_t pwm, uint32_t dir, uint32_t dir2, bool invert) {
  uint32_t duty = 0;
  if (fabsf(speed) >= PWM_DEADBAND_MPS && speed != 0) {
    duty = (uint32_t)lroundf(fabsf(speed) / MAX_WHEEL_SPEED_MPS * PWM_MAX);
    if (duty < PWM_MIN_DUTY) duty = PWM_MIN_DUTY;
    if (duty > PWM_MAX) duty = PWM_MAX;
  }
  bool forward = (speed >= 0) != invert;
  analogWrite(pwm, 0);
  digitalWrite(dir, duty && forward ? HIGH : LOW);
  if (DRIVER_TYPE == IN1_IN2_PWM) digitalWrite(dir2, duty && !forward ? HIGH : LOW);
  analogWrite(pwm, duty);
  return duty;
}

void stop_motors() {
  drive_motor(0, LEFT_PWM_PIN, LEFT_DIR_PIN, LEFT_DIR2_PIN, LEFT_MOTOR_INVERT);
  drive_motor(0, RIGHT_PWM_PIN, RIGHT_DIR_PIN, RIGHT_DIR2_PIN, RIGHT_MOTOR_INVERT);
  memset(wheel_data, 0, sizeof(wheel_data));
  command_fresh = false;
}

void check_timeout() {
  // Unsigned subtraction remains correct across millis() wraparound.
  if (command_fresh && (uint32_t)(millis() - last_cmd) >= CMD_VEL_TIMEOUT_MS) stop_motors();
}

void command_callback(const void *input) {
  const auto &msg = *static_cast<const geometry_msgs__msg__Twist *>(input);
  const double values[] = {msg.linear.x, msg.linear.y, msg.linear.z,
                           msg.angular.x, msg.angular.y, msg.angular.z};
  for (double value : values) {
    if (!isfinite(value)) { stop_motors(); return; }
  }
  // This chassis accepts planar commands only; malformed commands stop it.
  if (msg.linear.y != 0 || msg.linear.z != 0 || msg.angular.x != 0 || msg.angular.y != 0) {
    stop_motors(); return;
  }
  // Clamp in double precision before converting, including very large finite input.
  float v = fmax(ALLOW_REVERSE ? -MAX_LINEAR_MPS : 0.0, fmin(msg.linear.x, MAX_LINEAR_MPS));
  float w = fmax(-MAX_ANGULAR_RADPS, fmin(msg.angular.z, MAX_ANGULAR_RADPS));
  float left = v - w * TRACK_WIDTH_M / 2;
  float right = v + w * TRACK_WIDTH_M / 2;
  // When reverse is disabled, also prevent inner-wheel reverse during turns.
  if (!ALLOW_REVERSE) { left = fmaxf(0, left); right = fmaxf(0, right); }
  float peak = fmaxf(fabsf(left), fabsf(right));
  if (peak > MAX_WHEEL_SPEED_MPS) {
    left *= MAX_WHEEL_SPEED_MPS / peak;
    right *= MAX_WHEEL_SPEED_MPS / peak;
  }
  wheel_data[0] = left;
  wheel_data[1] = right;
  // Echo logical speeds and unsigned duty magnitudes; inversion affects GPIO only.
  wheel_data[2] = drive_motor(left, LEFT_PWM_PIN, LEFT_DIR_PIN, LEFT_DIR2_PIN, LEFT_MOTOR_INVERT);
  wheel_data[3] = drive_motor(right, RIGHT_PWM_PIN, RIGHT_DIR_PIN, RIGHT_DIR2_PIN, RIGHT_MOTOR_INVERT);
  last_cmd = millis();
  command_fresh = true;
}

// No message-buffer allocation: static sequences have an empty layout.
// Entity/executor memory is allocated only during session initialization.
bool create_entities() {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&options, allocator) != RCL_RET_OK) return false;
  rcl_ret_t rc = rcl_init_options_set_domain_id(&options, ROS_DOMAIN_ID);
  if (rc == RCL_RET_OK) rc = rclc_support_init_with_options(&support, 0, nullptr, &options, &allocator);
  IGNORE_RC(rcl_init_options_fini(&options));
  if (rc != RCL_RET_OK) return false;
  created = 1;
#define INIT(call, stage) do { if ((call) != RCL_RET_OK) return false; created = stage; } while (0)
  node = rcl_get_zero_initialized_node();
  cmd_sub = rcl_get_zero_initialized_subscription();
  hb_pub = rcl_get_zero_initialized_publisher();
  status_pub = rcl_get_zero_initialized_publisher();
  wheel_pub = rcl_get_zero_initialized_publisher();
  executor = rclc_executor_get_zero_initialized_executor();
  rmw_qos_profile_t reliable = rmw_qos_profile_default;
  reliable.depth = 1;
  rmw_qos_profile_t telemetry = rmw_qos_profile_sensor_data;
  telemetry.depth = 5;
  INIT(rclc_node_init_default(&node, NODE_NAME, "", &support), 2);
  INIT(rclc_subscription_init(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel", &reliable), 3);
  INIT(rclc_publisher_init(&hb_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/mcu/heartbeat", &reliable), 4);
  INIT(rclc_publisher_init(&status_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/mcu/status", &reliable), 5);
  INIT(rclc_publisher_init(&wheel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/mcu/wheel_cmd", &telemetry), 6);
#if USE_ENCODERS
  encoder_pub = rcl_get_zero_initialized_publisher();
  INIT(rclc_publisher_init(&encoder_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray), "/mcu/encoder_ticks", &telemetry), 7);
#endif
  // Bound reliable publication waits so the motor timeout is checked often.
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&hb_pub), 5) != RMW_RET_OK ||
      rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&status_pub), 5) != RMW_RET_OK) return false;
  INIT(rclc_executor_init(&executor, &support.context, 1, &allocator), 8);
  if (rclc_executor_add_subscription(&executor, &cmd_sub, &command, command_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  // Prepare the wait set now, rather than allocating on the first control spin.
  if (rclc_executor_prepare(&executor) != RCL_RET_OK) return false;
#undef INIT
  last_hb = last_status = last_ping = millis();
  return true;
}

void destroy_entities() {
  // Do not wait for an absent agent to acknowledge deletion.
  if (created >= 1) {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }
  if (created >= 8) IGNORE_RC(rclc_executor_fini(&executor));
#if USE_ENCODERS
  if (created >= 7) IGNORE_RC(rcl_publisher_fini(&encoder_pub, &node));
#endif
  if (created >= 6) IGNORE_RC(rcl_publisher_fini(&wheel_pub, &node));
  if (created >= 5) IGNORE_RC(rcl_publisher_fini(&status_pub, &node));
  if (created >= 4) IGNORE_RC(rcl_publisher_fini(&hb_pub, &node));
  if (created >= 3) IGNORE_RC(rcl_subscription_fini(&cmd_sub, &node));
  if (created >= 2) IGNORE_RC(rcl_node_fini(&node));
  if (created >= 1) IGNORE_RC(rclc_support_fini(&support));
  created = 0;
}

bool publish_telemetry() {
  uint32_t now = millis();
  if ((uint32_t)(now - last_hb) >= HEARTBEAT_PERIOD_MS) {
    last_hb = now;
    heartbeat.data++;
    led = !led;
    digitalWrite(STATUS_LED_PIN, led);
    if (rcl_publish(&hb_pub, &heartbeat, nullptr) != RCL_RET_OK) return false;
    check_timeout();
    if (rcl_publish(&wheel_pub, &wheel, nullptr) != RCL_RET_OK) return false;
#if USE_ENCODERS
    // Cortex-M4 cannot atomically read a 64-bit counter: snapshot with IRQs masked.
    noInterrupts();
    encoder_data[0] = left_ticks;
    encoder_data[1] = right_ticks;
    interrupts();
    if (rcl_publish(&encoder_pub, &encoder, nullptr) != RCL_RET_OK) return false;
#endif
  }
  if ((uint32_t)(now - last_status) >= 500) {
    last_status = now;
    const char *value = command_fresh ? "READY" : "CMD_TIMEOUT";
    status.data.size = strlen(value);
    memcpy(status_buffer, value, status.data.size + 1);
    if (rcl_publish(&status_pub, &status, nullptr) != RCL_RET_OK) return false;
  }
  return true;
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  analogWriteResolution(PWM_RESOLUTION_BITS);
  analogWriteFrequency(PWM_FREQUENCY_HZ);
  const uint32_t pins[] = {LEFT_PWM_PIN, RIGHT_PWM_PIN, LEFT_DIR_PIN, RIGHT_DIR_PIN};
  for (uint32_t pin : pins) pinMode(pin, OUTPUT);
  if (DRIVER_TYPE == IN1_IN2_PWM) {
    pinMode(LEFT_DIR2_PIN, OUTPUT);
    pinMode(RIGHT_DIR2_PIN, OUTPUT);
  }
  stop_motors();
#if USE_ENCODERS
  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), right_encoder, RISING);
  encoder.data.data = encoder_data;
  encoder.data.size = encoder.data.capacity = 2;
#endif
  status.data.data = status_buffer;
  status.data.capacity = sizeof(status_buffer);
  wheel.data.data = wheel_data;
  wheel.data.size = wheel.data.capacity = 4;
  Serial.begin(SERIAL_BAUD);
  set_microros_serial_transports(Serial); // USART2 via ST-Link virtual COM; never print here.
}

void loop() {
  check_timeout(); // Always run, including while waiting or recovering a session.
  uint32_t now = millis();
  switch (state) {
    case WAITING_AGENT:
      if ((uint32_t)(now - last_blink) >= 100) {
        last_blink = now;
        led = !led;
        digitalWrite(STATUS_LED_PIN, led);
      }
      if ((uint32_t)(now - last_ping) >= 500) {
        last_ping = now;
        if (rmw_uros_ping_agent(20, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      }
      break;
    case AGENT_AVAILABLE:
      stop_motors();
      state = create_entities() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      break;
    case AGENT_CONNECTED: {
      if ((uint32_t)(now - last_ping) >= 200) {
        last_ping = now;
        if (rmw_uros_ping_agent(20, 1) != RMW_RET_OK) {
          stop_motors();
          state = AGENT_DISCONNECTED;
          break;
        }
      }
      check_timeout();
      rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      check_timeout();
      if ((rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) || !publish_telemetry()) {
        stop_motors();
        state = AGENT_DISCONNECTED;
      }
      break;
    }
    case AGENT_DISCONNECTED:
      stop_motors();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
  check_timeout();
}
