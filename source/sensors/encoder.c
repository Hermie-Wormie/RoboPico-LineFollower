#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sensors.h"
#include "queue.h"

// Encoder pulse data
volatile uint32_t pulse_width_L = 0;
volatile uint32_t pulse_width_R = 0;
volatile uint32_t last_pulse_time_L = 0;
volatile uint32_t last_pulse_time_R = 0;
uint32_t encoder_counter = 0;

extern QueueHandle_t encoder_queue;

// From ultrasonic.c
extern volatile float distance;

// From wifi.c
extern ip_addr_t telemetry_ip;
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port);

float total_distance = 0;
float current_speed = 0;

// ----------------------------------------------
// Encoder Interrupt Service Routines
// ----------------------------------------------

void encoder_callback_L(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();

    if (last_pulse_time_L == 0) {
        last_pulse_time_L = current_time;
        return;
    }

    pulse_width_L = current_time - last_pulse_time_L;

    if (pulse_width_L >= MINIMUM_DEBOUCE) {
        last_pulse_time_L = current_time;
    }
}

void encoder_callback_R(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();

    if (last_pulse_time_R == 0) {
        last_pulse_time_R = current_time;
        return;
    }

    pulse_width_R = current_time - last_pulse_time_R;

    if (pulse_width_R >= MINIMUM_DEBOUCE) {
        last_pulse_time_R = current_time;
    }

    // Increment global count for distance tracking
    encoder_counter += 1;
}

// ----------------------------------------------
// Utility Functions
// ----------------------------------------------

void reset_encoder() {
    pulse_width_L = 0;
    pulse_width_R = 0;
    last_pulse_time_L = 0;
    last_pulse_time_R = 0;
}

void reset_counter() {
    encoder_counter = 0;
}

// ----------------------------------------------
// Telemetry Task (send distance + speed over UDP)
// ----------------------------------------------

void telemetry_task() {
    static const float distance_per_notch = 0.3318f / 20.0f;  // wheel circumference / encoder ticks
    float time_per_notch = 0;
    char buffer[32];

    while (1) {
        total_distance = encoder_counter * distance_per_notch;
        time_per_notch = pulse_width_R * 1e-6f; // µs → s

        if (pulse_width_R == 0) {
            current_speed = 0;
        } else {
            current_speed = distance_per_notch / time_per_notch;
        }

        snprintf(buffer, sizeof(buffer), "%.3f-%.3f-%.3f\n",
                 total_distance, current_speed, distance);

        send_udp_packet(buffer, &telemetry_ip, 2004);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
