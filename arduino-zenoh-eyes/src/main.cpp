#include <Arduino.h>
#include <Wire.h>
#include "controller.hpp"
#include "network.hpp"

EyeController eye_controller;

static void control_task(void *);
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.begin(115200);
    eye_controller.begin();

    delay(5000);
    // Networking
    EyeNetworkInterface::create_semaphores();
    EyeNetworkInterface::connect_wifi();
    EyeNetworkInterface::connect_zenoh();
    EyeNetworkInterface::add_control_task(control_task);
}

void loop() { vTaskDelay(1); }

// ======================= Control task (100 Hz) ===================
static void control_task(void *)
{
    while (1)
    {
        eye_controller.set_lr(EyeNetworkInterface::eye_command->look_lr());
        eye_controller.set_ud(EyeNetworkInterface::eye_command->look_ud());
        eye_controller.set_separation(EyeNetworkInterface::eye_command->eye_sep());
        if (EyeNetworkInterface::eye_command->blink()) {
            eye_controller.blink();
        }

        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}
