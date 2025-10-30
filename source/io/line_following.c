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
extern ip_addr_t remote_ip;
extern ip_addr_t telemetry_ip;
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port);

TimerHandle_t xOffTrackTimer;
#define OFFTRACKDELAY 300
#define SAMPLES 5

// ------- Define some simple motor macros -------

// Slowly speed up to overcome static friction
void ramp_up() {

    float motor1 = 0;
    float motor2 = 0;

    for (int i = 40; i <= 80; i += 2) {

        motor1 = i;
        motor2 = i+1;
        update_motor_fast(motor1, motor2);
        vTaskDelay(pdMS_TO_TICKS(20));
        
    }
}

/* Small script to backtrack and find distance
We assume the object's true distance will always be the shortest distance.
So once the end of track is found, we sweep the opposite direction until the distance increases*/
void GetFinalObstacleDistance() {

    // Array to store the distances
    float distanceReadings[10];
    float minDistance = 999;
    bool break_on_next = false;

    // Car was moving right before the offtrack stop. Sweep left.
    for (int i = 0; i < 10; i++) {

        // Jerk left
        update_motor_fast(0, 60);

        // Small delay to allow the car to perform the movement
        vTaskDelay(pdMS_TO_TICKS(100));

        // Stop
        update_motor_fast(0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Get the distance from the ultrasonic sensor
        float currentDistance = get_distance();

        // Store the reading
        distanceReadings[i] = currentDistance;

        // Check if the current distance is smaller
        if (currentDistance < minDistance) {
            minDistance = currentDistance;
        } else {

            // Give one more reading grace to check.
            if(!break_on_next){
                break_on_next = true;
            }else{
                break;
            }
        }
    }

    minDistance = minDistance + 1; // Sensor offset

    printf("Min Distance: %f\n", minDistance);

    char packet[8];
    snprintf(packet, 8, "-%.5g", minDistance);

    // Ensure the packet is null-terminated
    packet[7] = '\0';

    send_udp_packet(packet, &remote_ip, 2004);
    send_udp_packet(packet, &telemetry_ip, 2004);

    // Return or store the shortest distance as needed
    //return minDistance; // If this function is designed to return a value
}

// If this happens, the car has gone off track or end of line found
void vOffTrackTimerCallback(TimerHandle_t xTimer){

    // Stop motors.
    update_motor_fast(0, 0);

    // Notify line following to stop
    xTaskNotify(LineFollowing_T, true, eSetValueWithOverwrite);

    printf("Off Track!\n");

    GetFinalObstacleDistance();

}

// Simple line following algorithm, setup for IR mounted on the left. Ie. we skirt along the left edge.
void line_following_task(void *pvParameters){

    const uint8_t INCREMENT = 15;
    const uint8_t DECREMENT = 15;
    uint32_t start_command = false; // Needs to be int32 for task notifcation

    // Create the timer with 1 sec reset
    xOffTrackTimer = xTimerCreate("OffTrackResetTimer", pdMS_TO_TICKS(OFFTRACKDELAY), pdFALSE, 0, vOffTrackTimerCallback);


    while(1){
        
        // Wait until task manager explicitly tells it to start. The same notification is used to kick the task out to idle.
        while (1) {
            if (xTaskNotifyWait(0x00, ULONG_MAX, &start_command, portMAX_DELAY) == pdPASS) {
                if (start_command) {
                    break;
                }
                // Notified but did not ask task to start, continue to wait.
            }
        }

        /* Not needed anymore
        // Crawl forward till black line is reached
        forward();
        while(black_detected != 1){

                // Exit checking
                if (xTaskNotifyWait(0x00, ULONG_MAX, &start_command, 0) == pdPASS) {
                    update_motor_fast(0, 0);
                    break;
                }

            vTaskDelay(pdMS_TO_TICKS(1));

        }
        */

        // Enter main loop
        while(1){

            static int8_t left_momentum = 50;
            static int8_t right_momentum = 50;
            static bool turning_left = false;
            static bool turning_right = false;

            // Check if ultrasonic gave any warnings. Reset speed to zero if so.
            if (xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0) == pdTRUE) {
                update_motor_fast(0, 0);
                printf("Distance Warning, ending line following\n");
                break;
            } 

            // Start Line follow Logic Here -------------------
            // We assume hugging inner edge
    
            // When detected black, turn left, means just crossed the boundary, car canted right. Continuing will result in overshoot to right
            if(black_detected == 1){

                xTimerStart(xOffTrackTimer, 0); // Reset timer everytime black line was detected
                
                // First direction change starting values
                /*
                if (!turning_left){
                    //left_momentum = 0;
                    right_momentum = 60;
                    turning_left = true;
                    turning_right = false;
                }*/

                // Bound
                if (left_momentum > 60){
                    left_momentum = 60;
                } else if (left_momentum < 0){
                    left_momentum = 0;
                }

                if (right_momentum > 60){
                    right_momentum = 60;
                } else if (right_momentum < 0){
                    right_momentum = 0;
                }

                // Update speed
                update_motor_fast(left_momentum, right_momentum);

                left_momentum -= DECREMENT;
                right_momentum += INCREMENT;

            } else{ // when detected white, turn right

                // First direction change starting values
                /*
                if (!turning_right){
                    left_momentum = 60;
                    //right_momentum = 0;
                    turning_left = false;
                    turning_right = true;
                }*/

                // Bound
                if (left_momentum > 60){
                    left_momentum = 60;
                } else if (left_momentum < 0){
                    left_momentum = 0;
                }

                if (right_momentum > 60){
                    right_momentum = 60;
                } else if (right_momentum < 0){
                    right_momentum = 0;
                }

                // Update speed
                update_motor_fast(left_momentum, right_momentum);

                left_momentum += INCREMENT;
                right_momentum -= DECREMENT;

            }

            // End Line follow Logic Here -------------------
            
            // Exit checking
            if (xTaskNotifyWait(0x00, ULONG_MAX, &start_command, 0) == pdPASS) {
                update_motor_fast(0, 0);
                break;
            }

            vTaskDelay(1);

        }
    }

}