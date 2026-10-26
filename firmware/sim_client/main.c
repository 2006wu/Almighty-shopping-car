/* Linux-only XRCE-DDS simulation; no device, GPIO or motor access. */
#include <arpa/inet.h>
#include <netdb.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include "model.h"

#define CHECK(call) do { if ((call) != RCL_RET_OK) { \
  fprintf(stderr, "micro-ROS failure at line %d\n", __LINE__); return 1; } } while (0)
static Model model;
static volatile sig_atomic_t running = 1;
static void stop(int sig) { (void)sig; running = 0; }
static double monotonic(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec+ts.tv_nsec/1e9;
}
static void command_cb(const void *input) {
  const geometry_msgs__msg__Twist *msg = input;
  double c[] = {msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z};
  model_command(&model, c, monotonic());
}
static void heartbeat_cb(const void *input) {
  model_heartbeat(&model, ((const std_msgs__msg__UInt32 *)input)->data, monotonic());
}
static void drop_cb(const void *input) {
  float seconds = ((const std_msgs__msg__Float32 *)input)->data;
  if (isfinite(seconds) && seconds >= 0 && seconds <= 60)
    model.drop_commands_until = monotonic()+seconds;
}
static const char *env(const char *name, const char *fallback) {
  const char *value = getenv(name);
  return value && *value ? value : fallback;
}
int main(void) {
  signal(SIGINT, stop);
  signal(SIGTERM, stop);
  model_init(&model);
  /* Resolve Compose DNS explicitly: the XRCE transport expects an IPv4 address. */
  struct addrinfo hints = {0}, *address = NULL;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  const char *host = env("MICROROS_AGENT_HOST", "microros_agent");
  if (getaddrinfo(host, NULL, &hints, &address)) {
    fprintf(stderr, "Cannot resolve agent %s\n", host);
    return 1;
  }
  char ip[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &((struct sockaddr_in *)address->ai_addr)->sin_addr, ip, sizeof(ip));
  freeaddrinfo(address);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  CHECK(rcl_init_options_init(&options, allocator));
  char *end;
  unsigned long domain = strtoul(env("ROS_DOMAIN_ID", "28"), &end, 10);
  if (*end || domain > 232) return 1;
  CHECK(rcl_init_options_set_domain_id(&options, domain));
  rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&options);
  CHECK(rmw_uros_options_set_udp_address(ip, env("MICROROS_AGENT_PORT", "8888"), rmw_options));
  /* Compose restarts this process after init/session failure; no stale model survives. */
  if (rmw_uros_ping_agent_options(200, 10, rmw_options) != RMW_RET_OK) return 1;
  rclc_support_t support;
  CHECK(rclc_support_init_with_options(&support, 0, NULL, &options, &allocator));
  CHECK(rcl_init_options_fini(&options));
  rcl_node_t node = rcl_get_zero_initialized_node();
  CHECK(rclc_node_init_default(&node, "micro_ros_sim_client", "", &support));
  rcl_publisher_t wheel_pub = rcl_get_zero_initialized_publisher();
  rcl_publisher_t hb_pub = rcl_get_zero_initialized_publisher();
  rcl_publisher_t status_pub = rcl_get_zero_initialized_publisher();
  rcl_publisher_t marker_pub = rcl_get_zero_initialized_publisher();
  rmw_qos_profile_t reliable = rmw_qos_profile_default;
  reliable.depth = MCU_MCU_HEARTBEAT_DEPTH;
  rmw_qos_profile_t wheel_qos = rmw_qos_profile_sensor_data;
  wheel_qos.depth = MCU_WHEEL_DEPTH;
  CHECK(rclc_publisher_init(&wheel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), MCU_WHEEL_TOPIC, &wheel_qos));
  CHECK(rclc_publisher_init(&hb_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), MCU_MCU_HEARTBEAT_TOPIC, &reliable));
  CHECK(rclc_publisher_init(&status_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), MCU_STATUS_TOPIC, &reliable));
  CHECK(rclc_publisher_init(&marker_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/mcu/sim/xrce_active", &reliable));
  CHECK(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&hb_pub), 10));
  CHECK(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&status_pub), 10));
  CHECK(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&marker_pub), 10));
  rcl_subscription_t cmd_sub = rcl_get_zero_initialized_subscription();
  rcl_subscription_t hb_sub = rcl_get_zero_initialized_subscription();
  rcl_subscription_t drop_sub = rcl_get_zero_initialized_subscription();
  CHECK(rclc_subscription_init(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), MCU_CMD_TOPIC, &reliable));
  CHECK(rclc_subscription_init(&hb_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), MCU_PC_HEARTBEAT_TOPIC, &reliable));
  CHECK(rclc_subscription_init(&drop_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), MCU_DROP_COMMANDS_TOPIC, &reliable));
  geometry_msgs__msg__Twist command = {0};
  std_msgs__msg__UInt32 pc_hb = {0}, hb = {0};
  std_msgs__msg__Float32 drop = {0};
  std_msgs__msg__Bool marker = {.data = true};
  sensor_msgs__msg__JointState wheel = {0};
  std_msgs__msg__String status = {0};
  /* Allocate bounded sequences/strings once, before executing control callbacks. */
  if (!sensor_msgs__msg__JointState__init(&wheel) || !std_msgs__msg__String__init(&status) ||
      !rosidl_runtime_c__String__assign(&status.data, MCU_STATUS_INVALID_COMMAND) ||
      !rosidl_runtime_c__String__assign(&wheel.header.frame_id, MCU_WHEEL_FRAME) ||
      !rosidl_runtime_c__String__Sequence__init(&wheel.name, 4) ||
      !rosidl_runtime_c__double__Sequence__init(&wheel.position, 4) ||
      !rosidl_runtime_c__double__Sequence__init(&wheel.velocity, 4)) return 1;
  const char *names[] = MCU_WHEEL_NAMES;
  for (int i = 0; i < 4; ++i)
    if (!rosidl_runtime_c__String__assign(&wheel.name.data[i], names[i])) return 1;
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  CHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  CHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &command, command_cb, ON_NEW_DATA));
  CHECK(rclc_executor_add_subscription(&executor, &hb_sub, &pc_hb, heartbeat_cb, ON_NEW_DATA));
  CHECK(rclc_executor_add_subscription(&executor, &drop_sub, &drop, drop_cb, ON_NEW_DATA));
  double next_wheel = 0, next_hb = 0, next_marker = 0, next_ping = monotonic()+1;
  while (running) {
    rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) return 1;
    double now = monotonic();
    model_step(&model, now);
    if (now >= next_wheel) {
      next_wheel = now+1.0/MCU_WHEEL_RATE;
      struct timespec stamp;
      clock_gettime(CLOCK_REALTIME, &stamp);
      wheel.header.stamp.sec = (int32_t)stamp.tv_sec;
      wheel.header.stamp.nanosec = (uint32_t)stamp.tv_nsec;
      memcpy(wheel.position.data, model.position, sizeof(model.position));
      memcpy(wheel.velocity.data, model.velocity, sizeof(model.velocity));
      CHECK(rcl_publish(&wheel_pub, &wheel, NULL));
    }
    if (now >= next_hb) {
      next_hb = now+1.0/MCU_MCU_HEARTBEAT_RATE;
      hb.data++;
      /* All status values fit the allocation reserved above. */
      status.data.size = strlen(model.status);
      memcpy(status.data.data, model.status, status.data.size+1);
      CHECK(rcl_publish(&hb_pub, &hb, NULL));
      CHECK(rcl_publish(&status_pub, &status, NULL));
    }
    if (now >= next_marker) {
      next_marker = now+1;
      CHECK(rcl_publish(&marker_pub, &marker, NULL));
    }
    if (now >= next_ping) {
      next_ping = now+1;
      if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) return 1;
    }
  }
  /* Process exit releases host resources; the next session always starts stopped. */
  return 0;
}
