//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

#include <Arduino.h>
// #include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <zenoh-pico.h>
#include "config.hpp"

#if Z_FEATURE_SUBSCRIPTION == 1
// WiFi-specific parameters
#define SSID "nectocaris"
#define PASS "idontknowithinkitswrittendown?"

// Client mode values (comment/uncomment as needed)
#define MODE "client"
#define LOCATOR ""  // If empty, it will scout
// Peer mode values (comment/uncomment as needed)
// #define MODE "peer"
// #define LOCATOR "udp/224.0.0.225:7447#iface=en0"

#define KEYEXPR "myhome/kitchen/temp"

#define LED_BUILTIN 2

z_owned_session_t s;
z_owned_subscriber_t sub;

// Calculated angles for eyelids
const uint8_t UP_DOWN_CENTER = (UP_MAX + DOWN_MAX) / 2;
const uint8_t LEFT_RIGHT_CENTER = (LEFT_MAX + RIGHT_MAX) / 2;

const uint8_t BOT_LEFT_CLOSED = 90 + _BOT_CLOSED;
const uint8_t BOT_LEFT_OPEN = 90 - _BOT_OPEN;
const uint8_t BOT_RIGHT_CLOSED = 90 - _BOT_CLOSED;
const uint8_t BOT_RIGHT_OPEN = 90 + _BOT_OPEN;

const uint8_t TOP_LEFT_CLOSED = 90 - _TOP_CLOSED;
const uint8_t TOP_LEFT_OPEN = 90 + _TOP_OPEN;
const uint8_t TOP_RIGHT_CLOSED = 90 + _TOP_CLOSED;
const uint8_t TOP_RIGHT_OPEN = 90 - _TOP_OPEN;

// Calculate max angles in eye's frame of reference
const int UP_DOWN_MAX_ANGLE = UP_MAX - UP_DOWN_CENTER;
const int LEFT_RIGHT_MAX_ANGLE =  LEFT_MAX - LEFT_RIGHT_CENTER;

// PCA9685 PWM resolution and timing
const uint16_t PCA9685_STEPS = 4096;           // 12-bit resolution
const float PCA9685_PWM_PERIOD_US = 20000.0;   // 20ms period for 50Hz
const float PCA9685_STEP_US = PCA9685_PWM_PERIOD_US / PCA9685_STEPS; // ≈4.88us per step

// Servo channel definitions
const uint8_t servo_driver_address = 0x40;  // Default I2C address for PCA9685
const uint8_t look_ud = 0;
const uint8_t look_lr = 1;
const uint8_t lid_bl = 2;
const uint8_t lid_tl = 3;
const uint8_t lid_br = 4;
const uint8_t lid_tr = 5;


class Eyes {
public:
    Eyes() {
        // Initialize the PWM driver
        pwm = Adafruit_PWMServoDriver(servo_driver_address);
        pwm.begin();
        pwm.setPWMFreq(SERVO_FREQ);
        pwm.setOscillatorFrequency(OSCILLATOR);
        delay(10);

        // Intitialize parameters
        left_eyelid_separation = 30;  // [deg]
        right_eyelid_separation = 30; // [deg]
        blink_time = 100;  // [ms]

        // Initialize eyelid angles
        set_separation(left_eyelid_separation, right_eyelid_separation);
    }

    void test_eyes() {
        blink();
        delay(1000);

        set_ud(UP_DOWN_MAX_ANGLE);
        delay(1000);

        set_ud(-UP_DOWN_MAX_ANGLE);
        delay(1000);

        set_ud(0);
        set_lr(LEFT_RIGHT_MAX_ANGLE);
        delay(1000);

        set_lr(-LEFT_RIGHT_MAX_ANGLE);
        delay(1000);

        set_lr(0);
        blink();
        delay(MG90_DELAY_MS);
    }

    void set_separation(uint8_t left_sep, uint8_t right_sep) {
        int left_half_sep = left_sep / 2;
        int right_half_sep = right_sep / 2;
        int old_left_half_sep = left_eyelid_separation / 2;
        int old_right_half_sep = right_eyelid_separation / 2;

        // Update eyelid angles based on new separation
        last_tl_angle =+ left_half_sep - old_left_half_sep;
        last_tr_angle =+ right_half_sep - old_right_half_sep;
        last_bl_angle =+ left_half_sep - old_left_half_sep;
        last_br_angle =+ right_half_sep - old_right_half_sep;

        left_eyelid_separation = left_sep;
        right_eyelid_separation = right_sep;

        // Set eyelids to the new angles
        set_tl(last_tl_angle);
        set_tr(last_tr_angle);
        set_bl(last_bl_angle);
        set_br(last_br_angle);
    }

    /**
     * @brief Set the left-right angle of the eyes.
     * 
     * @param angle Angle in degrees, where 0 is center, 
     * positive (+) is left, and negative (-) is right.
     */
    void set_lr(int angle) {  
        angle = constrain(angle + LEFT_RIGHT_CENTER , LEFT_MAX, RIGHT_MAX);
        setServo(look_lr, angle);
    }

    /**
     * @brief 0 is center, + is up, - is down
     * 
     * @param angle Angle in degrees, where 0 is center, 
     * positive (+) is up, and negative (-) is down.
     */
    void set_ud(int angle) {
        uint32_t servo_angle = angle + UP_DOWN_CENTER;  // Convert to 0-180 range
        angle = constrain(angle, UP_MAX, DOWN_MAX);
        setServo(look_ud, angle);

        // Set eyelids to follow eye movement while maintaining separation
        uint32_t left_half_sep = left_eyelid_separation / 2;
        uint32_t right_half_sep = right_eyelid_separation / 2;

        if (abs(angle) > left_half_sep) {
            uint32_t displacement = abs(angle) - left_half_sep;

            if (angle > 0) {
                last_tl_angle = left_half_sep + displacement;
                last_bl_angle = left_half_sep - displacement;
            } else {
                last_tl_angle = left_half_sep - displacement;
                last_bl_angle = left_half_sep + displacement;
            }         

            set_tl(last_tl_angle);
            set_bl(last_bl_angle);
        }     

        if (abs(angle) > right_half_sep) {
            uint32_t displacement = abs(angle) - right_half_sep;

            if (angle > 0) {
                last_tr_angle = right_half_sep + displacement;
                last_br_angle = right_half_sep - displacement;
            } else {
                last_tr_angle = right_half_sep - displacement;
                last_br_angle = right_half_sep + displacement;
            }         

            set_tr(last_tr_angle);
            set_br(last_br_angle);
        }
    }

    void blink() {
        // Close eyelids
        set_bl(0);
        set_br(0);
        set_tl(0);
        set_tr(0);

        delay(blink_time);

        // Open eyelids
        set_bl(last_bl_angle);
        set_br(last_br_angle);
        set_tl(last_tl_angle);
        set_tr(last_tr_angle);
    }

private:
    void setServo(uint8_t servo, int angle) {
        // Constrain the angle to the servo's range
        angle = constrain(angle, MG90_MIN_ANGLE, MG90_MAX_ANGLE);
        uint16_t pulse = angleToPulse(angle);
        pwm.setPWM(servo, 0, pulse);
    }

    /**
     * @brief Set the bottom-left eyelid servo angle.
     * 
     * @param angle Closed is 0, open is positive (+)
     */
    void set_tr(uint32_t angle) {
        uint32_t servo_angle = TOP_RIGHT_CLOSED - angle;
        servo_angle = constrain(servo_angle, TOP_RIGHT_OPEN, TOP_RIGHT_CLOSED);
        setServo(lid_tr, angle); 
    }

    void set_tl(uint32_t angle) {
        uint32_t servo_angle = TOP_LEFT_CLOSED + angle;
        servo_angle = constrain(servo_angle, TOP_LEFT_CLOSED, TOP_LEFT_OPEN);
        setServo(lid_tl, angle); 
    }

    void set_br(uint32_t angle) {
        uint32_t servo_angle = BOT_LEFT_CLOSED - angle;
        servo_angle = constrain(servo_angle, BOT_LEFT_OPEN, BOT_LEFT_CLOSED);
        setServo(lid_bl, angle); 
    }

    void set_bl(uint32_t angle) {
        uint32_t servo_angle = BOT_RIGHT_CLOSED + angle;
        servo_angle = constrain(servo_angle, BOT_RIGHT_CLOSED, BOT_RIGHT_OPEN);
        setServo(lid_br, angle);
    }

    Adafruit_PWMServoDriver pwm;
    uint8_t left_eyelid_separation;  // [deg]
    uint8_t right_eyelid_separation;  // [deg]
    uint32_t blink_time;  // [ms]

    int last_tl_angle;
    int last_tr_angle;
    int last_bl_angle;
    int last_br_angle;

     /**
     * @brief Helper function to convert angle (0-180) to PWM pulse length for MG90 servos
     * Converts angle to microseconds, then to PCA9685 steps
     * 
     * @param angle angle to convert
     * @return Pulse width in PCA9685 steps
     */
    static uint16_t angleToPulse(
        int angle,
        int minAngle = MG90_MIN_ANGLE,
        int maxAngle = MG90_MAX_ANGLE,
        int minPulse = MG90_MIN_PULSE_US,
        int maxPulse = MG90_MAX_PULSE_US
    ) {
        int us = map(angle, minAngle, maxAngle, minPulse, maxPulse);
        // Divide by PCA9685_STEP_US to get the number of steps for the PWM driver
        return us / PCA9685_STEP_US;
    }
};

// void home() {
//     // Set all servos to 90 degrees
//     uint16_t pulse90 = angleToPulse(90);

//     setServo(look_ud, 90);
//     setServo(look_lr, 90);
//     setServo(lid_bl, 90);
//     setServo(lid_br, 90);
//     setServo(lid_tl, 90);
//     setServo(lid_tr, 90);
//     delay(MG90_DELAY_MS);
// }

void set_tr(int angle) {  // 0 is center, 

}

Eyes eyes;

void setup() {
    eyes.test_eyes();  // Test the eyes functionality
    // ...
}


void loop() {
    // ...
}


void data_handler(z_loaned_sample_t *sample, void *arg) {
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);
    z_owned_string_t value;
    z_bytes_to_string(z_sample_payload(sample), &value);

    Serial.print(" >> [Subscription listener] Received (");
    Serial.write(z_string_data(z_view_string_loan(&keystr)), z_string_len(z_view_string_loan(&keystr)));
    Serial.print(", ");
    Serial.write(z_string_data(z_string_loan(&value)), z_string_len(z_string_loan(&value)));
    Serial.println(")");

    // Convert the value to an integer
    int intValue = atoi(z_string_data(z_string_loan(&value)));
    Serial.print("Converted integer value: ");
    Serial.println(intValue);

    // Check if the value is odd and blink the LED
    if (intValue % 2 != 0) { // Check if the value is odd
        digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
        delay(500);                      // Wait for 500ms
        digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
        delay(500);                      // Wait for 500ms
    } 
    z_string_drop(z_string_move(&value));
}

void _setup() {
    // Initialize Serial for debug
    Serial.begin(115200);
    while (!Serial) {
        delay(1000);
    }

    // Initialize the built-in LED pin
    pinMode(LED_BUILTIN, OUTPUT);

    // Set WiFi in STA mode and trigger attachment
    Serial.print("Connecting to WiFi...");
    // WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    Serial.println("OK");

    // Initialize Zenoh Session and other parameters
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
    if (strcmp(LOCATOR, "") != 0) {
        if (strcmp(MODE, "client") == 0) {
            zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, LOCATOR);
        } else {
            zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_LISTEN_KEY, LOCATOR);
        }
    }

    // Open Zenoh session
    Serial.print("Opening Zenoh Session...");
    if (z_open(&s, z_config_move(&config), NULL) < 0) {
        Serial.println("Unable to open session!");
        while (1) {
            ;
        }
    }
    Serial.println("OK");

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 || zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0) {
        Serial.println("Unable to start read and lease tasks\n");
        z_session_drop(z_session_move(&s));
        while (1) {
            ;
        }
    }

    // Declare Zenoh subscriber
    Serial.print("Declaring Subscriber on ");
    Serial.print(KEYEXPR);
    Serial.println(" ...");
    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, data_handler, NULL, NULL);
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
    if (z_declare_subscriber(z_session_loan(&s), &sub, z_view_keyexpr_loan(&ke), z_closure_sample_move(&callback),
                             NULL) < 0) {
        Serial.println("Unable to declare subscriber.");
        while (1) {
            ;
        }
    }
    Serial.println("OK");
    Serial.println("Zenoh setup finished!");

    delay(300);
}

void _loop() { delay(1000); }

#else
void setup() {
    Serial.println("ERROR: Zenoh pico was compiled without Z_FEATURE_SUBSCRIPTION but this example requires it.");
    return;
}
void loop() {}
#endif