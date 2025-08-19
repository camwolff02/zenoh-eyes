// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#include <Arduino.h>
#include <WiFi.h>
#include <zenoh-pico.h>

#include <cstring>
#include <flatbuffers/flatbuffers.h>
#include "electric_eyes_generated.h"  // generated from schemas/electric_eyes.fbs

// FreeRTOS (from ESP32 Arduino core; no external lib needed)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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

// Control loop rate (Hz) — now 100 Hz
#define CTRL_HZ 100
#define CTRL_PERIOD_MS (1000 / CTRL_HZ)

// ==== Zenoh globals ===========================================================
z_owned_session_t g_session;
z_owned_subscriber_t g_sub;

// ==== Shared buffer (written by callback, read by rx task) ====================
static constexpr size_t kCmdBufMax = 128;  // plenty for this schema
static uint8_t g_cmd_buf[kCmdBufMax];
static size_t  g_cmd_len = 0;

// Mutex to protect g_cmd_buf / g_cmd_len
static SemaphoreHandle_t g_cmd_mtx = nullptr;

// Event to notify a new buffer arrival
static SemaphoreHandle_t g_cmd_evt = nullptr;

// Latest parsed state (owned by RX task)
struct EyeCommandState {
  int16_t  look_ud = 0;
  int16_t  look_lr = 0;
  uint16_t eye_sep = 90;
  bool     blink   = false;
  uint32_t seq     = 0;   // increments whenever a new valid buffer is parsed
};

// Current & previous states for change detection (RX task scope)
static EyeCommandState g_state;       // current
static EyeCommandState g_prev_state;  // previous

// ---------- Debug printing (factored out) -------------------------------------
static void debug_print(const EyeCommandState& s) {
  Serial.printf("[cmd #%lu] ud=%d lr=%d sep=%u blink=%s\n",
                (unsigned long)s.seq,
                (int)s.look_ud, (int)s.look_lr, (unsigned)s.eye_sep,
                s.blink ? "true" : "false");
}

// ---------- Change detection helper -------------------------------------------
static bool different(const EyeCommandState& a, const EyeCommandState& b) {
  return a.look_ud != b.look_ud ||
         a.look_lr != b.look_lr ||
         a.eye_sep != b.eye_sep ||
         a.blink   != b.blink;
}

// ============== Subscriber callback (minimal work) ============================
static void data_handler(z_loaned_sample_t *sample, void * /*arg*/) {
  // Copy payload bytes into the shared buffer; parsing/logging happens elsewhere.
  z_owned_string_t payload;
  z_bytes_to_string(z_sample_payload(sample), &payload);
  const uint8_t* src = reinterpret_cast<const uint8_t*>(z_string_data(z_string_loan(&payload)));
  const size_t   len = z_string_len(z_string_loan(&payload));

  if (len > kCmdBufMax) {
    // Too big — drop (schema is tiny; this should never happen)
    Serial.printf(">> [sub] payload too large (%u > %u), dropping\n",
                  (unsigned)len, (unsigned)kCmdBufMax);
    z_string_drop(z_string_move(&payload));
    return;
  }

  if (xSemaphoreTake(g_cmd_mtx, portMAX_DELAY) == pdTRUE) {
    memcpy(g_cmd_buf, src, len);
    g_cmd_len = len;
    xSemaphoreGive(g_cmd_mtx);
  }
  z_string_drop(z_string_move(&payload));

  // Notify RX task that a new buffer arrived
  if (g_cmd_evt) xSemaphoreGive(g_cmd_evt);
}

// ======================= RX task (parse & debug on change) ====================
static void rx_task(void *) {
  using namespace electric_eyes;

  uint8_t local_buf[kCmdBufMax];
  size_t  local_len = 0;

  for (;;) {
    // Wait until the callback signals a new buffer
    if (xSemaphoreTake(g_cmd_evt, portMAX_DELAY) != pdTRUE) continue;

    // Snapshot the buffer under mutex
    if (xSemaphoreTake(g_cmd_mtx, portMAX_DELAY) == pdTRUE) {
      local_len = g_cmd_len;
      if (local_len) memcpy(local_buf, g_cmd_buf, local_len);
      xSemaphoreGive(g_cmd_mtx);
    }

    if (!local_len) continue;

    // Verify & parse
    flatbuffers::Verifier verifier(local_buf, local_len);
    if (!VerifyEyeCommandBuffer(verifier)) {
      Serial.println("[rx ] Received buffer failed EyeCommand verification");
      continue;
    }
    const EyeCommand* cmd = GetEyeCommand(local_buf);

    // Fill prospective new state
    EyeCommandState new_state = g_state;
    new_state.look_ud = cmd->look_ud();
    new_state.look_lr = cmd->look_lr();
    new_state.eye_sep = cmd->eye_sep();
    new_state.blink   = cmd->blink();

    // If changed, commit and print
    if (different(new_state, g_state)) {
      g_prev_state = g_state;
      g_state = new_state;
      g_state.seq++;
      debug_print(g_state);
    }

    // prepare for next loop
    local_len = 0;
  }
}

// ======================= Control task (100 Hz, placeholder) ===================
static void control_task(void *) {
  // Intentionally empty for now; this runs at 100 Hz and is where servo control would live.
  for (;;) {
    // Example: cheap heartbeat
    // digitalWrite(LED_BUILTIN, HIGH); delay(1); digitalWrite(LED_BUILTIN, LOW);

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

  // Create sync primitives before network starts
  g_cmd_mtx = xSemaphoreCreateMutex();
  g_cmd_evt = xSemaphoreCreateBinary();

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

  // Tasks:
  // - RX task: waits for new messages, parses, prints on change
  xTaskCreatePinnedToCore(rx_task, "eye_rx", 4096, nullptr, 2, nullptr, 1);

  // - Control task: 100 Hz placeholder for future servo code
  xTaskCreatePinnedToCore(control_task, "eye_ctrl", 4096, nullptr, 1, nullptr, 1);
}

void loop() {
  // Nothing: zenoh read/lease tasks + rx_task + control_task do the work.
  vTaskDelay(1);
}

#endif  // Z_FEATURE_SUBSCRIPTION
