// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#include <Arduino.h>
#include <WiFi.h>
#include <zenoh-pico.h>

#include <cstring>
#include <flatbuffers/flatbuffers.h>
#include "electric_eyes_generated.h"  // generated from schemas/electric_eyes.fbs

// ---------- Feature guard ----------
#if Z_FEATURE_SUBSCRIPTION != 1
void setup(){ Serial.begin(115200); Serial.println("Z_FEATURE_SUBSCRIPTION missing"); }
void loop(){}
#else

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

// Control loop rate (Hz)
#define CTRL_HZ 2
#define CTRL_PERIOD_MS (1000 / CTRL_HZ)

// ==== Zenoh globals ===========================================================
z_owned_session_t g_session;
z_owned_subscriber_t g_sub;

// ==== Shared buffer (written by callback, read by control task) ===============
static constexpr size_t kCmdBufMax = 128;  // plenty for this schema
static uint8_t g_cmd_buf[kCmdBufMax];
static size_t  g_cmd_len = 0;

// Mutex to protect g_cmd_buf / g_cmd_len
SemaphoreHandle_t g_cmd_mtx = nullptr;

// Latest parsed state (owned by control task)
struct EyeCommandState {
  int16_t  look_ud = 0;
  int16_t  look_lr = 0;
  uint16_t eye_sep = 90;
  bool     blink   = false;
  uint32_t seq     = 0;   // increments whenever a new valid buffer is parsed
};
static EyeCommandState g_state;

// ============== Subscriber callback (minimal work) ============================
static void data_handler(z_loaned_sample_t *sample, void * /*arg*/) {
  // Copy payload bytes into the shared buffer; parsing happens in the control task.
  z_owned_string_t payload;
  z_bytes_to_string(z_sample_payload(sample), &payload);
  const uint8_t* src = reinterpret_cast<const uint8_t*>(z_string_data(z_string_loan(&payload)));
  const size_t   len = z_string_len(z_string_loan(&payload));

  if (len > kCmdBufMax) {
    // Too big — drop (schema is tiny; this should never happen)
    Serial.printf(">> [sub] payload too large (%u > %u), dropping\n", (unsigned)len, (unsigned)kCmdBufMax);
    z_string_drop(z_string_move(&payload));
    return;
  }

  if (xSemaphoreTake(g_cmd_mtx, portMAX_DELAY) == pdTRUE) {
    memcpy(g_cmd_buf, src, len);
    g_cmd_len = len;
    xSemaphoreGive(g_cmd_mtx);
  }
  z_string_drop(z_string_move(&payload));
}

// ======================= Control task (2 Hz) ==================================
void control_task(void *) {
  using namespace electric_eyes;

  uint8_t local_buf[kCmdBufMax];
  size_t  local_len = 0;

  for (;;) {
    // Copy latest buffer snapshot (non-blocking for long)
    if (xSemaphoreTake(g_cmd_mtx, portMAX_DELAY) == pdTRUE) {
      local_len = g_cmd_len;
      if (local_len) memcpy(local_buf, g_cmd_buf, local_len);
      xSemaphoreGive(g_cmd_mtx);
    }

    // If we have something, verify & parse it into g_state
    if (local_len) {
      flatbuffers::Verifier verifier(local_buf, local_len);
      if (VerifyEyeCommandBuffer(verifier)) {
        const EyeCommand* cmd = GetEyeCommand(local_buf);

        g_state.look_ud = cmd->look_ud();
        g_state.look_lr = cmd->look_lr();
        g_state.eye_sep = cmd->eye_sep();
        g_state.blink   = cmd->blink();
        g_state.seq++;

        // (Optional) consume once per parse; keep printing the latest state each tick either way.
        // local_len = 0;  // uncomment to only parse once per incoming message
      } else {
        Serial.println("[ctrl] Received buffer failed EyeCommand verification");
      }
    }

    // "Simulate" servo control by printing current state
    Serial.printf("[ctrl #%lu] ud=%d lr=%d sep=%u blink=%s\n",
                  (unsigned long)g_state.seq,
                  (int)g_state.look_ud, (int)g_state.look_lr, (unsigned)g_state.eye_sep,
                  g_state.blink ? "true" : "false");

    // Optional: tiny LED tick to show loop alive
    digitalWrite(LED_BUILTIN, HIGH); delay(10); digitalWrite(LED_BUILTIN, LOW);

    vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
  }
}

// ============================= Wi-Fi / setup =================================
static void connect_wifi() {
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
  Serial.print(" OK  IP="); Serial.println(WiFi.localIP());
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Create mutex before network starts
  g_cmd_mtx = xSemaphoreCreateMutex();

  connect_wifi();

  // === Configure zenoh-pico session ==========================================
  z_owned_config_t conf;
  z_config_default(&conf);
  zp_config_insert(z_config_loan_mut(&conf), Z_CONFIG_MODE_KEY, "client");

  if (strcmp(Z_LOCATOR, "") != 0) {
    // For robustness you can switch this to *_insert_json5 with a JSON array if needed
    zp_config_insert(z_config_loan_mut(&conf), Z_CONFIG_CONNECT_KEY, Z_LOCATOR);
  }

  Serial.print("Opening Zenoh session ... ");
  if (z_open(&g_session, z_config_move(&conf), NULL) < 0) {
    Serial.println("FAILED (z_open). Halting.");
    for(;;) delay(1000);
  }
  Serial.println("OK");

  if (zp_start_read_task(z_session_loan_mut(&g_session), NULL) < 0 ||
      zp_start_lease_task(z_session_loan_mut(&g_session), NULL) < 0) {
    Serial.println("FAILED to start read/lease tasks. Halting.");
    z_session_drop(z_session_move(&g_session));
    for(;;) delay(1000);
  }

  // Subscriber (callback is lightweight)
  Serial.print("Subscribing to " KEYEXPR " ... ");
  z_owned_closure_sample_t cb;
  z_closure_sample(&cb, data_handler, NULL, NULL);

  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);

  if (z_declare_subscriber(z_session_loan(&g_session), &g_sub,
                           z_view_keyexpr_loan(&ke),
                           z_closure_sample_move(&cb), NULL) < 0) {
    Serial.println("FAILED (z_declare_subscriber). Halting.");
    for(;;) delay(1000);
  }
  Serial.println("OK");

  // Start control task at 2 Hz
  xTaskCreatePinnedToCore(control_task, "eye_ctrl", 4096, nullptr, 1, nullptr, 1);
}

void loop() {
  // Nothing: all the work is in tasks (zenoh read/lease tasks + control task).
  vTaskDelay(1);
}

#endif  // Z_FEATURE_SUBSCRIPTION
