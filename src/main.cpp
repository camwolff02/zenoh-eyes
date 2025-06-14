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
const uint8_t BOT_OPEN = 45;
const uint8_t BOT_CLOSED = 30;
const uint8_t TOP_OPEN = 35;
const uint8_t TOP_CLOSED = 35;


// PCA9685 PWM resolution and timing
const uint16_t PCA9685_STEPS = 4096;           // 12-bit resolution
const float PCA9685_PWM_PERIOD_US = 20000.0;   // 20ms period for 50Hz
const float PCA9685_STEP_US = PCA9685_PWM_PERIOD_US / PCA9685_STEPS; // ≈4.88us per step

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const uint look_ud = 0;
const uint look_lr = 1;
const uint lid_bl = 2;
const uint lid_tl = 3;
const uint lid_br = 4;
const uint lid_tr = 5;

// Helper function to convert angle (0-180) to PWM pulse length for MG90 servos
// Converts angle to microseconds, then to PCA9685 steps
uint16_t angleToPulse(
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

void blink() {
    pwm.setPWM(lid_bl, 0, angleToPulse(90+BOT_CLOSED));
    pwm.setPWM(lid_br, 0, angleToPulse(90-BOT_CLOSED));
    pwm.setPWM(lid_tl, 0, angleToPulse(90-TOP_CLOSED));
    pwm.setPWM(lid_tr, 0, angleToPulse(90+TOP_CLOSED));
    delay(100);
    pwm.setPWM(lid_bl, 0, angleToPulse(90-BOT_OPEN));
    pwm.setPWM(lid_br, 0, angleToPulse(90+BOT_OPEN));
    pwm.setPWM(lid_tl, 0, angleToPulse(90+TOP_OPEN));
    pwm.setPWM(lid_tr, 0, angleToPulse(90-TOP_OPEN));
}

void setup() {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    pwm.setOscillatorFrequency(OSCILLATOR);
    delay(10);

    // Set all servos to 90 degrees
    uint16_t pulse90 = angleToPulse(90);
    pwm.setPWM(look_ud, 0, pulse90);
    pwm.setPWM(look_lr, 0, pulse90);
    pwm.setPWM(lid_bl, 0, pulse90);
    pwm.setPWM(lid_tl, 0, pulse90);
    pwm.setPWM(lid_br, 0, pulse90);
    pwm.setPWM(lid_tr, 0, pulse90);
    delay(1000);

    // pwm.setPWM(look_ud, 0, angleToPulse(UP_MAX));
    // delay(1000);
    // pwm.setPWM(look_ud, 0, angleToPulse(DOWN_MAX));
    // delay(1000);

    // pwm.setPWM(look_ud, 0, pulse90);
    // blink();
}


void loop() {
    int step = 1;

    // Sweep left and right
    for (int angle = 90; angle <= LEFT_MAX; angle += step) {
        pwm.setPWM(look_lr, 0, angleToPulse(angle));
        delay(MG90_DELAY_MS);
    }
    delay(1000);
    for (int angle = LEFT_MAX; angle >= RIGHT_MAX; angle -= step) {
        pwm.setPWM(look_lr, 0, angleToPulse(angle));
        delay(MG90_DELAY_MS);
    }
    delay(1000);
    for (int angle = RIGHT_MAX; angle <= 90; angle += step) {
        pwm.setPWM(look_lr, 0, angleToPulse(angle));
        delay(MG90_DELAY_MS);
    }
    delay(1000);


    // Up and down
    // for (int angle = 90; angle <= UP_MAX; angle += step) {
    //     pwm.setPWM(look_ud, 0, angleToPulse(angle));
    //     delay(500);
    // }
    // delay(1000);
    // for (int angle = UP_MAX; angle >= DOWN_MAX; angle -= step) {
    //     pwm.setPWM(look_ud, 0, angleToPulse(angle));
    //     delay(500);
    // }
    // delay(1000);
    // for (int angle = DOWN_MAX; angle <= 90; angle += step) {
    //     pwm.setPWM(look_ud, 0, angleToPulse(angle));
    //     delay(500);
    // }
    // delay(1000);
 
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