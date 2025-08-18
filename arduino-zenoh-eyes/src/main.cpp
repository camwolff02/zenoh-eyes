// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#include <Arduino.h>
#include <WiFi.h>
#include <zenoh-pico.h>

#include <flatbuffers/flatbuffers.h>
#include "electric_eyes_generated.h"   // generated from schemas/electric_eyes.fbs

#if Z_FEATURE_SUBSCRIPTION == 1
#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // Most ESP32 dev boards use GPIO2 for the on-board LED
#endif

// ---------- Wi-Fi ----------
#define WIFI_SSID "Unsecured Network"
#define WIFI_PASS "OA@9191.com"

// Use client mode and connect directly to your zenoh router.
#define Z_LOCATOR "tcp/192.168.68.136:7447"

// Subscribe to your control channel
#define KEYEXPR   "robot/eye_command"

// ==== GLOBALS =================================================================
z_owned_session_t g_session;
z_owned_subscriber_t g_sub;

// Keep callback work minimal: set a flag and let loop() blink the LED.
volatile bool g_blink = false;

void data_handler(z_loaned_sample_t *sample, void *arg) {
  using namespace electric_eyes;

  // Get payload bytes (binary-safe). We reuse the string helper to access data+len.
  z_owned_string_t payload;
  z_bytes_to_string(z_sample_payload(sample), &payload);
  const uint8_t* buf = reinterpret_cast<const uint8_t*>(
      z_string_data(z_string_loan(&payload)));
  size_t len = z_string_len(z_string_loan(&payload));

  // Verify it’s an EyeCommand flatbuffer
  flatbuffers::Verifier verifier(buf, len);
  if (!VerifyEyeCommandBuffer(verifier)) {
    Serial.println(">> [sub] Received payload, but not a valid EyeCommand FlatBuffer.");
    z_string_drop(z_string_move(&payload));
    return;
  }

  // Parse
  const EyeCommand* cmd = GetEyeCommand(buf);

  // Read fields
  int16_t look_ud = cmd->look_ud();    // [-90..90]
  int16_t look_lr = cmd->look_lr();    // [-90..90]
  uint16_t eye_sep = cmd->eye_sep();   // [0..90]
  bool blink = cmd->blink();

  // Debug print
  Serial.print(">> [EyeCommand] ud=");
  Serial.print(look_ud);
  Serial.print(" lr=");
  Serial.print(look_lr);
  Serial.print(" sep=");
  Serial.print(eye_sep);
  Serial.print(" blink=");
  Serial.println(blink ? "true" : "false");

  // Blink LED when commanded
  if (blink) g_blink = true;

  // In your real app, drive servos here using look_ud/look_lr/eye_sep.

  z_string_drop(z_string_move(&payload));
}

void connect_wifi() {
  Serial.print("Connecting to WiFi ... ");
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println("\nWiFi connect timeout, resetting...");
      ESP.restart();
    }
  }
  Serial.print(" OK  IP=");
  Serial.println(WiFi.localIP());
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  connect_wifi();

  // === Configure zenoh-pico session (matches Arduino example pattern) ========
  z_owned_config_t conf;
  z_config_default(&conf);

  // mode
  zp_config_insert(z_config_loan_mut(&conf), Z_CONFIG_MODE_KEY, "client");

  if (strcmp(Z_LOCATOR, "") != 0) {
    // NOTE: If you ever see z_open() fail here, switch to:
    // zp_config_insert_json5(..., Z_CONFIG_CONNECT_KEY, "[\"tcp/<IP>:7447\"]");
    zp_config_insert(z_config_loan_mut(&conf), Z_CONFIG_CONNECT_KEY, Z_LOCATOR);
  }

  Serial.print("Opening Zenoh session ... ");
  if (z_open(&g_session, z_config_move(&conf), NULL) < 0) {
    Serial.println("FAILED (z_open). Halting.");
    for (;;) delay(1000);
  }
  Serial.println("OK");

  // Start read & lease tasks (required on Arduino, per example)
  if (zp_start_read_task(z_session_loan_mut(&g_session), NULL) < 0 ||
      zp_start_lease_task(z_session_loan_mut(&g_session), NULL) < 0) {
    Serial.println("FAILED to start read/lease tasks. Halting.");
    z_session_drop(z_session_move(&g_session));
    for (;;) delay(1000);
  }

  // === Declare the subscriber =================================================
  Serial.print("Subscribing to " KEYEXPR " ... ");
  z_owned_closure_sample_t cb;
  z_closure_sample(&cb, data_handler, NULL, NULL);

  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);

  if (z_declare_subscriber(z_session_loan(&g_session), &g_sub,
                           z_view_keyexpr_loan(&ke),
                           z_closure_sample_move(&cb), NULL) < 0) {
    Serial.println("FAILED (z_declare_subscriber). Halting.");
    for (;;) delay(1000);
  }
  Serial.println("OK");
  Serial.println("Zenoh subscriber ready.");
}

void loop() {
  // Blink the LED once when commanded
  if (g_blink) {
    g_blink = false;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(120);
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(1);
}

#else
void setup() {
    Serial.println("ERROR: Zenoh pico was compiled without Z_FEATURE_SUBSCRIPTION but this example requires it.");
    return;
}
void loop() {}
#endif
