#pragma once
#include <Arduino.h>

// ALL hardware placeholders live here. Confirm every TODO before motor power.
// TODO(hardware): Confirm the assumed Nucleo-G431RB; board selection is in platformio.ini.
// TODO(hardware): Confirm Serial uses LPUART1 PA2/PA3 through ST-LINK VCP.
// TODO(hardware): Reserve PA2/PA3 for LPUART1 VCP and PA13/PA14 for SWD.
// TODO(hardware): Keep D14/D15 PB9/PB8 free for a future BNO085 I2C IMU; PB8 is also BOOT0.
// TODO(hardware): Keep spare D0/D1 PC5/PC4 (USART1) free.
#define DRIVER_PWM_DIR 0
#define DRIVER_L298N 1
#define DRIVER_TYPE DRIVER_PWM_DIR  // TODO(hardware): Select the driver's input mode.

constexpr uint32_t LEFT_PWM_PIN = D6; // TODO(hardware): PB10, TIM2_CH3 PWM.
constexpr uint32_t RIGHT_PWM_PIN = D9; // TODO(hardware): PC7, TIM3_CH2 PWM.
constexpr uint32_t LEFT_DIR_PIN = D4; // TODO(hardware): PB5, GPIO DIR or IN1.
constexpr uint32_t RIGHT_DIR_PIN = D7; // TODO(hardware): PA8, GPIO DIR or IN1.
constexpr uint32_t LEFT_DIR2_PIN = D8; // TODO(hardware): PA9, GPIO IN2 for IN1/IN2 mode only.
constexpr uint32_t RIGHT_DIR2_PIN = D3; // TODO(hardware): PB3, GPIO IN2 for IN1/IN2 mode only; SWO unused with SWD without trace.
#define USE_ENCODERS 0                 // TODO(hardware): Enable only after wiring.
constexpr uint32_t LEFT_ENCODER_A_PIN = D2; // TODO(hardware): PA10, EXTI10 interrupt input.
constexpr uint32_t LEFT_ENCODER_B_PIN = D12; // TODO(hardware): PA6, GPIO direction input.
constexpr uint32_t RIGHT_ENCODER_A_PIN = D11; // TODO(hardware): PA7, EXTI7 interrupt input.
constexpr uint32_t RIGHT_ENCODER_B_PIN = D5; // TODO(hardware): PB4, GPIO direction input; Arduino core disables UCPD dead-battery pull-down at boot.
constexpr int32_t ENCODER_TICKS_PER_REV = 1024; // TODO(hardware): Output-wheel ticks, A CHANGE (x2).
constexpr bool LEFT_MOTOR_INVERT = false;  // TODO(hardware): Check physical forward.
constexpr bool RIGHT_MOTOR_INVERT = false; // TODO(hardware): Check physical forward.
constexpr float WHEEL_RADIUS_M = 0.05f; // TODO(hardware): Measure loaded wheel radius.
constexpr float TRACK_WIDTH_M = 0.30f;  // TODO(hardware): Measure left/right wheel spacing.
constexpr float MAX_WHEEL_SPEED_MPS = 1.0f; // TODO(hardware): Calibrate speed at full PWM.
constexpr float MAX_LINEAR_MPS = 1.0f;     // TODO(hardware): Lower for initial testing.
constexpr float MAX_ANGULAR_RADPS = 2.0f;  // TODO(hardware): Confirm safe turning limit.
constexpr bool ALLOW_REVERSE = true;      // TODO(hardware): Bring-up allows reverse (1).
constexpr uint32_t PWM_FREQUENCY_HZ = 20000; // TODO(hardware): Check driver frequency rating.
constexpr uint8_t PWM_RESOLUTION_BITS = 8;  // TODO(hardware): Confirm duty resolution.
constexpr uint16_t PWM_MIN_DUTY = 0;        // TODO(hardware): Minimum nonzero duty, 0..PWM_MAX.
constexpr float PWM_DEADBAND_MPS = 0.01f;   // TODO(hardware): Commands below this magnitude stop.
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500; // TODO(hardware): Verify stopping delay on bench.
constexpr uint32_t HEARTBEAT_PERIOD_MS = 100; // TODO(hardware): Verify 10 Hz serial budget.
constexpr uint32_t SERIAL_BAUD = 115200;     // TODO(hardware): Match host SERIAL_BAUD.
constexpr uint32_t STATUS_LED_PIN = LED_BUILTIN; // TODO(hardware): PA5, D13 GPIO user LED LD2.
