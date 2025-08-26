// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#include "network.hpp"

// Define static members
SemaphoreHandle_t EyeNetworkInterface::eye_command_mtx = nullptr;
const eye_interface::EyeCommand *EyeNetworkInterface::eye_command = nullptr;
uint8_t EyeNetworkInterface::eye_command_buf[EyeNetworkInterface::BUF_MAX] = {};
size_t EyeNetworkInterface::eye_command_buf_len = 0;

z_owned_session_t EyeNetworkInterface::g_session = {};
z_owned_subscriber_t EyeNetworkInterface::g_sub = {};

// ---------- Debug printing (factored out) -------------------------------------
void EyeNetworkInterface::debug_print(const eye_interface::EyeCommand *s)
{
  Serial.printf("[cmd] ud=%d lr=%d sep=%u blink=%s\n",
                (int)s->look_ud(), (int)s->look_lr(), (unsigned)s->eye_sep(),
                s->blink() ? "true" : "false");
}

// ---------- Change detection helper -------------------------------------------
bool EyeNetworkInterface::different(const eye_interface::EyeCommand *a, const eye_interface::EyeCommand *b)
{
  return a->look_ud() != b->look_ud() ||
         a->look_lr() != b->look_lr() ||
         a->eye_sep() != b->eye_sep() ||
         a->blink() != b->blink();
}

// ============== Subscriber callback (minimal work) ============================
void EyeNetworkInterface::data_handler(z_loaned_sample_t *sample, void * /*arg*/)
{
  // Copy payload bytes into the shared buffer; parsing/logging happens elsewhere.
  z_owned_string_t payload;
  z_bytes_to_string(z_sample_payload(sample), &payload);
  const uint8_t *src = reinterpret_cast<const uint8_t *>(z_string_data(z_string_loan(&payload)));
  const size_t len = z_string_len(z_string_loan(&payload));

  // Make sure message isn't too big
  if (len > BUF_MAX)
  {
    // Too big — drop (schema is tiny; this should never happen)
    Serial.printf(">> [sub] payload too large (%u > %u), dropping\n",
                  (unsigned)len, (unsigned)BUF_MAX);
    z_string_drop(z_string_move(&payload));
    return;
  }

  // Verify message is an EyeCommand
  flatbuffers::Verifier verifier(src, len);
  if (!eye_interface::VerifyEyeCommandBuffer(verifier))
  {
    Serial.println("Received buffer failed EyeCommand verification");
    z_string_drop(z_string_move(&payload));
    return;
  }

  // Save buffer if they are different
  if (xSemaphoreTake(eye_command_mtx, portMAX_DELAY) == pdTRUE)
  {
    if (!eye_command || different(eye_interface::GetEyeCommand(src), eye_command))
    {
      memcpy(eye_command_buf, src, len);                           // move the received bytes to the saved buffer
      eye_command_buf_len = len;                                   // save the new buffer length
      eye_command = eye_interface::GetEyeCommand(eye_command_buf); // create a pointer to the new buffer
      // Serial.println("New dat!!! wooooo");
      debug_print(eye_command);
    }

    xSemaphoreGive(eye_command_mtx); // Yield mutex
  }

  z_string_drop(z_string_move(&payload));
}

void EyeNetworkInterface::create_semaphores()
{
  // Create sync primitives before network starts
  eye_command_mtx = xSemaphoreCreateMutex();
}

void EyeNetworkInterface::connect_wifi()
{
  // ============================= Wi-Fi / setup =================================
  Serial.print("Connecting to WiFi ... ");
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    Serial.print(".");
    if (millis() - start > 20000)
    {
      Serial.println("\nWiFi connect timeout, resetting...");
      ESP.restart();
    }
  }
  Serial.print(" OK  IP=");
  Serial.println(WiFi.localIP());
}

void EyeNetworkInterface::connect_zenoh()
{
  // === Configure zenoh-pico session ==========================================
  z_owned_config_t conf;
  z_config_default(&conf);
  zp_config_insert(z_config_loan_mut(&conf), Z_CONFIG_MODE_KEY, "client");

  if (strcmp(Z_LOCATOR, "") != 0)
  {
    // For robustness you can switch this to *_insert_json5 with a JSON array if needed
    zp_config_insert(z_config_loan_mut(&conf), Z_CONFIG_CONNECT_KEY, Z_LOCATOR);
  }

  Serial.print("Opening Zenoh session ... ");
  if (z_open(&g_session, z_config_move(&conf), NULL) < 0)
  {
    Serial.println("FAILED (z_open). Halting.");
    while (1)
      delay(1000);
  }
  Serial.println("OK");

  if (zp_start_read_task(z_session_loan_mut(&g_session), NULL) < 0 ||
      zp_start_lease_task(z_session_loan_mut(&g_session), NULL) < 0)
  {
    Serial.println("FAILED to start read/lease tasks. Halting.");
    z_session_drop(z_session_move(&g_session));
    while (1)
      delay(1000);
  }

  // Subscriber (callback is lightweight)
  Serial.print("Subscribing to " KEYEXPR " ... ");
  z_owned_closure_sample_t cb;
  z_closure_sample(&cb, data_handler, NULL, NULL);

  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);

  if (z_declare_subscriber(z_session_loan(&g_session), &g_sub,
                           z_view_keyexpr_loan(&ke),
                           z_closure_sample_move(&cb), NULL) < 0)
  {
    Serial.println("FAILED (z_declare_subscriber). Halting.");
    while (1)
      delay(1000);
  }
  Serial.println("OK");
}

// TODO add mechanism to ensure this can only be called once
void EyeNetworkInterface::add_control_task(TaskFunction_t task)
{
  while (!eye_command)
    vTaskDelay(1);
  
  // Wait for an eye command to be received to schedule control task
  xTaskCreatePinnedToCore(task, "eye_ctrl", 4096, nullptr, 1, nullptr, 1);
  Serial.println("Started control task");
}