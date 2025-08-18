/*
Configuration file for the servo control system
This file contains constants and parameters for controlling MG90 servos

Make sure before using this file, you set all servos to the home position (90 degrees),
and attach all servo horns either parallel or perpendicular to the servo body,
*/
#include <cstdint>

const float SERVO_FREQ = 50.0;  // Analog servos run at ~50 Hz updates
const uint32_t OSCILLATOR = 25000000;  // 25 MHz

// MG90 servo pulse width and sweep parameters
const int MG90_MIN_ANGLE = 0;
const int MG90_MAX_ANGLE = 180;
const int MG90_MIN_PULSE_US = 500;
const int MG90_MAX_PULSE_US = 2500;
const int MG90_STEP = 2;
const int MG90_DELAY_MS = 15;

const uint8_t UP_MAX = 90+50;
const uint8_t DOWN_MAX = 90-50;
const uint8_t LEFT_MAX = 90+50;
const uint8_t RIGHT_MAX = 90-50;
const uint8_t _BOT_OPEN = 45;
const uint8_t _BOT_CLOSED = 30;
const uint8_t _TOP_OPEN = 35;
const uint8_t _TOP_CLOSED = 35;

