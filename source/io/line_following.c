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
    for (int i = 40; i <= 80; i += 2)
    {
        update_motor_fast(i, i + 1);
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
                break_on_next = true;
            else
                break;
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
void line_following_task(void *pvParameters)
{
    printf("Line Following Started\n");
    const uint8_t INCREMENT = 5;
    const uint8_t DECREMENT = 5;
    uint32_t start_command = false;

    xOffTrackTimer = xTimerCreate("OffTrackResetTimer", pdMS_TO_TICKS(OFFTRACKDELAY), pdFALSE, 0, vOffTrackTimerCallback);

    while (1)
    {
        // Main line-following loop
        static int8_t left_momentum = 20;
        static int8_t right_momentum = 20;

        // Stop if ultrasonic detects obstacle too close
        if (xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0) == pdTRUE)
        {
            update_motor_fast(0, 0);
            printf("Distance Warning! Stopping.\n");
            break;
        }

        // IR-based steering logic
        if (black_detected)
        {
            xTimerStart(xOffTrackTimer, 0);
            left_momentum = (left_momentum > 60) ? 60 : left_momentum;
            right_momentum = (right_momentum > 60) ? 60 : right_momentum;
            update_motor_fast(left_momentum, right_momentum);
            left_momentum -= DECREMENT;
            right_momentum += INCREMENT;
        }
        else
        {
            left_momentum = (left_momentum > 60) ? 60 : left_momentum;
            right_momentum = (right_momentum > 60) ? 60 : right_momentum;
            update_motor_fast(left_momentum, right_momentum);
            left_momentum += INCREMENT;
            right_momentum -= DECREMENT;
        }

        // Check for task stop notification
        if (xTaskNotifyWait(0x00, ULONG_MAX, &start_command, 0) == pdPASS)
        {
            update_motor_fast(0, 0);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
