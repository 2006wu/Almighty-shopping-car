#pragma once
#include <Arduino.h>

// ALL hardware placeholders live here. Verify every TODO before motor power-on.
// TODO(hardware): Board selection also must match platformio.ini: nucleo_f446re.
enum DriverType { PWM_DIR, IN1_IN2_PWM };
constexpr DriverType DRIVER_TYPE = PWM_DIR;  // TODO(hardware): Select the driver interface.
constexpr uint32_t LEFT_PWM_PIN = D3;   // TODO(hardware): PWM-capable Arduino header pin.
constexpr uint32_t RIGHT_PWM_PIN = D5;  // TODO(hardware): PWM-capable Arduino header pin.
constexpr uint32_t LEFT_DIR_PIN = D4;   // TODO(hardware): DIR or IN1.
constexpr uint32_t RIGHT_DIR_PIN = D7;  // TODO(hardware): DIR or IN1.
constexpr uint32_t LEFT_DIR2_PIN = D8;  // TODO(hardware): IN2, unused with PWM_DIR.
constexpr uint32_t RIGHT_DIR2_PIN = D9; // TODO(hardware): IN2, unused with PWM_DIR.
#define USE_ENCODERS 0  // TODO(hardware): Enable only after wiring both encoders.
constexpr uint32_t LEFT_ENCODER_A_PIN = D2;   // TODO(hardware): Interrupt-capable input.
constexpr uint32_t LEFT_ENCODER_B_PIN = D6;   // TODO(hardware): Quadrature direction input.
constexpr uint32_t RIGHT_ENCODER_A_PIN = D10; // TODO(hardware): Interrupt-capable input.
constexpr uint32_t RIGHT_ENCODER_B_PIN = D11; // TODO(hardware): Quadrature direction input.
constexpr float ENCODER_TICKS_PER_REV = 360; // TODO(hardware): A rising edges/output-wheel revolution (1x).
constexpr bool LEFT_MOTOR_INVERT = false;   // TODO(hardware): Match positive speed to forward.
constexpr bool RIGHT_MOTOR_INVERT = false;  // TODO(hardware): Match positive speed to forward.
constexpr float WHEEL_RADIUS_M = 0.05f;     // TODO(hardware): Measure; reserved for future encoder conversion.
constexpr float TRACK_WIDTH_M = 0.30f;      // TODO(hardware): Measure left-to-right wheel spacing.
constexpr float MAX_WHEEL_SPEED_MPS = 1.0f; // TODO(hardware): Estimate speed at full PWM; open-loop only.
constexpr float MAX_LINEAR_MPS = 1.0f;      // TODO(hardware): Choose a safe bring-up limit.
constexpr float MAX_ANGULAR_RADPS = 2.0f;   // TODO(hardware): Choose a safe turning limit.
constexpr bool ALLOW_REVERSE = true;       // TODO(hardware): Bring-up permits reverse, including pivot turns.
constexpr uint32_t PWM_FREQUENCY_HZ = 20000; // TODO(hardware): Check driver frequency rating.
constexpr uint8_t PWM_RESOLUTION_BITS = 8;  // TODO(hardware): Check core/driver PWM support.
constexpr uint32_t PWM_MIN_DUTY = 0;        // TODO(hardware): Calibrate stiction only with wheels lifted.
constexpr float PWM_DEADBAND_MPS = 0.01f;   // TODO(hardware): Speeds below this magnitude produce zero PWM.
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500; // TODO(hardware): Measure actual stop latency.
constexpr uint32_t HEARTBEAT_PERIOD_MS = 100; // TODO(hardware): Keep 10 Hz for this test.
constexpr uint32_t SERIAL_BAUD = 115200;     // TODO(hardware): Match agent baud; ST-Link VCP uses USART2.
constexpr uint32_t STATUS_LED_PIN = LED_BUILTIN; // TODO(hardware): Verify board LED pin.
constexpr size_t ROS_DOMAIN_ID = 28;        // TODO(hardware): Match the isolated PC domain.
constexpr char NODE_NAME[] = "winter_base"; // TODO(hardware): Keep unique within the test domain.
