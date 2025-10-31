#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
#include "semphr.h"
#include "timers.h"

// External functions and variables
void update_motor(float speed_motor1, float speed_motor2, bool clockwise_motor1, bool clockwise_motor2);
void update_motor_fast(uint16_t speed_motor1, uint16_t speed_motor2);

// From infrared.c
extern volatile bool black_detected;

// From ultrasonic.c
extern SemaphoreHandle_t UltrasonicWarn_BinarySemaphore;
float get_distance();

// From main.c
extern TaskHandle_t LineFollowing_T;

// From wifi.c
extern ip_addr_t telemetry_ip;
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port);

TimerHandle_t xOffTrackTimer;
#define OFFTRACKDELAY 300
#define SAMPLES 5

// ------- Simple motor helper functions -------

// Slowly speed up to overcome static friction
void ramp_up()
{

    float motor1 = 0;
    float motor2 = 0;

    for (int i = 40; i <= 80; i += 2)
    {

        motor1 = i;
        motor2 = i + 1;
        update_motor_fast(motor1, motor2);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Sweep to find minimum obstacle distance after losing line.
 * The car moves slightly left-right to get multiple ultrasonic readings,
 * keeping the smallest as the closest obstacle distance.
 */
void GetFinalObstacleDistance()
{
    float distanceReadings[10];
    float minDistance = 999;
    bool break_on_next = false;

    for (int i = 0; i < 10; i++)
    {
        // Jerk left
        update_motor_fast(0, 60);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Stop and measure
        update_motor_fast(0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        float currentDistance = get_distance();
        distanceReadings[i] = currentDistance;

        if (currentDistance < minDistance)
        {
            minDistance = currentDistance;
        }
        else
        {
            if (!break_on_next)
            {
                break_on_next = true;
            }
            else
            {
                break;
            }
        }
    }

    minDistance += 1; // small offset
    printf("Min Distance: %.2f cm\n", minDistance);

    char packet[16];
    snprintf(packet, sizeof(packet), "%.2f", minDistance);
    send_udp_packet(packet, &telemetry_ip, 2004);
}

// Triggered if car goes off-track (line lost)
void vOffTrackTimerCallback(TimerHandle_t xTimer)
{
    update_motor_fast(0, 0);
    xTaskNotify(LineFollowing_T, true, eSetValueWithOverwrite);
    printf("Off Track! Performing distance scan...\n");
    GetFinalObstacleDistance();
}

// Core line-following logic
void line_following_task(void *pvParameters) {
    printf("[LineFollowing] Task started.\n");

    // Tunable parameters
    const float MAX_SPEED = 40.0f;
    const float MIN_SPEED = 10.0f;
    const float BASE_SPEED = 20.0f;    // cruising forward speed
    const float TURN_GAIN = 0.35f;     // how strong the turn effect is (lower = smoother)
    const float SMOOTHING = 0.2f;      // 0.1–0.3 recommended; lower = slower acceleration

    uint32_t debug_counter = 0;
    float left_speed = BASE_SPEED;
    float right_speed = BASE_SPEED;
    float target_left = BASE_SPEED;
    float target_right = BASE_SPEED;

    printf("=== LINE FOLLOWING AUTO-START (Smoothed) ===\n");

    while (1) {

        // --- Safety check ---
        if (xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0) == pdTRUE) {
            update_motor_fast(0, 0);
            printf("Distance Warning, ending line following\n");
            break;
        }

        // --- Read IR ---
        int line_state = black_detected;  // 1 = black under sensor, 0 = white

        // --- Determine target turn ---
        if (line_state == 1) {
            // on black: slightly reduce left speed, increase right
            target_left = BASE_SPEED * (1.0f - TURN_GAIN);
            target_right = BASE_SPEED * (1.0f + TURN_GAIN);
        } else {
            // on white: slightly increase left speed, reduce right
            target_left = BASE_SPEED * (1.0f + TURN_GAIN);
            target_right = BASE_SPEED * (1.0f - TURN_GAIN);
        }

        // --- Clamp target range ---
        if (target_left > MAX_SPEED) target_left = MAX_SPEED;
        if (target_left < MIN_SPEED) target_left = MIN_SPEED;
        if (target_right > MAX_SPEED) target_right = MAX_SPEED;
        if (target_right < MIN_SPEED) target_right = MIN_SPEED;

        // --- Smooth transition (low-pass filter) ---
        left_speed = left_speed + SMOOTHING * (target_left - left_speed);
        right_speed = right_speed + SMOOTHING * (target_right - right_speed);

        // --- Apply to motors ---
        update_motor_fast((uint16_t)left_speed, (uint16_t)right_speed);

        // --- Debug print every ~500ms ---
        debug_counter++;
        if (debug_counter >= 50) {
            printf("IR:%d | L:%.1f R:%.1f\n", line_state, left_speed, right_speed);
            debug_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz update rate
    }
}
