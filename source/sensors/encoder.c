#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sensors.h"
#include "queue.h"

// ----------------------------------------------
// Debounce threshold (µs)
// ----------------------------------------------
#define MINIMUM_DEBOUNCE 500 // XY-C206 recommended ~500–1000 µs

// Encoder pulse data
volatile uint32_t pulse_width_L = 0;
volatile uint32_t pulse_width_R = 0;
volatile uint32_t last_pulse_time_L = 0;
volatile uint32_t last_pulse_time_R = 0;
uint32_t encoder_counter = 0;
extern uint32_t encoder_counter;

extern QueueHandle_t encoder_queue;

// From ultrasonic.c
extern volatile float distance;

// From wifi.c
extern ip_addr_t telemetry_ip;
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port);

float total_distance = 0;
float current_speed = 0;

void init_encoders(void)
{

    // Motor 1 (left) encoder – GP6
    gpio_init(LEFT_ENCODER);
    gpio_set_dir(LEFT_ENCODER, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &encoder_callback_L);

    // Motor 2 (right) encoder – GP4
    gpio_init(RIGHT_ENCODER);
    gpio_set_dir(RIGHT_ENCODER, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &encoder_callback_R);
}

// ----------------------------------------------
// Encoder Interrupt Service Routines
// ----------------------------------------------

void encoder_callback_L(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (last_pulse_time_L == 0)
    {
        last_pulse_time_L = current_time;
        return;
    }

    uint32_t delta = current_time - last_pulse_time_L;

    if (delta >= MINIMUM_DEBOUNCE)
    {
        pulse_width_L = delta;
        last_pulse_time_L = current_time;
    }
}

void encoder_callback_R(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (last_pulse_time_R == 0)
    {
        last_pulse_time_R = current_time;
        return;
    }

    uint32_t delta = current_time - last_pulse_time_R;

    if (delta >= MINIMUM_DEBOUNCE)
    {
        pulse_width_R = delta;
        last_pulse_time_R = current_time;
        encoder_counter += 1;
    }
}

// ----------------------------------------------
// Utility Functions
// ----------------------------------------------

void reset_encoder()
{
    pulse_width_L = 0;
    pulse_width_R = 0;
    last_pulse_time_L = 0;
    last_pulse_time_R = 0;
}

void reset_counter()
{
    encoder_counter = 0;
}

// ----------------------------------------------
// compute speed function lives HERE
// ----------------------------------------------
float compute_actual_speed(uint32_t pulse_width_us)
{
    if (pulse_width_us < 400 || pulse_width_us > 10000000) {
        return 0.0f;
    }

    const float circumference = 0.3318f;
    const float pulses_per_rev = 20.0f;

    float pulse_interval_seconds = (float)pulse_width_us * 1e-6f;
    float time_per_rev = pulse_interval_seconds * pulses_per_rev;
    if (time_per_rev <= 0.0f) return 0.0f;

    return circumference / time_per_rev;
}


// ----------------------------------------------
// Telemetry Task
// ----------------------------------------------

void telemetry_task()
{
    static const float distance_per_notch = 0.3318f / 20.0f; // 20 pulses per wheel revolution
    char buffer[32];

    while (1)
    {
        total_distance = encoder_counter * distance_per_notch;
        current_speed  = compute_actual_speed(pulse_width_R);

        snprintf(buffer, sizeof(buffer), "%.3f-%.3f-%.3f\n",
                 total_distance, current_speed, distance);

        send_udp_packet(buffer, &telemetry_ip, 2004);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void encoder_debug_task(void *pv)
{
    while (1)
    {
        float sL = compute_actual_speed(pulse_width_L);
        float sR = compute_actual_speed(pulse_width_R);
        printf("L=%6u µs (%.3f m/s)   R=%6u µs (%.3f m/s)   Cnt=%u\n",
               pulse_width_L, sL,
               pulse_width_R, sR,
               encoder_counter);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
