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

// Set 1 to print
#if 1
#define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...) // Nothing happens if DEBUG is 0
#endif

#define TRIGGER_DISTANCE 15
#define RIGHT_COUNTER 8
#define FORWARD_COUNTER 5//89

// External functions and variables
void update_motor(float speed_motor1, float speed_motor2, bool clockwise_motor1, bool clockwise_motor2);

// motor.c
extern float target_speed_motor1;
extern float target_speed_motor2;
extern bool clockwise_motor1;
extern bool clockwise_motor2;
extern volatile bool APPLY_PID;
void reset_PID();

// encoder.c
void reset_encoder();
void reset_counter();
void start_counting(bool state);
void set_direction(bool state);
extern QueueHandle_t encoder_queue;

// ultrasonic.c
float get_distance();

// io_handler.c
void buzzer(int count);

void station_1_task(){

    float distance = 0;
    uint16_t right_starting_counter;
    uint16_t forward_starting_counter;
    uint16_t right_final_counter;
    uint16_t forward_final_counter;
    uint16_t right_encoder_counter;
    uint16_t forward_encoder_counter;

    // Task Loop
    while(1){

            distance = 0;
            right_starting_counter = 0;
            forward_starting_counter = 0;
            right_final_counter = 0;
            forward_final_counter = 0;
            right_encoder_counter = 0;
            forward_encoder_counter = 0;

        // Main Loop
        while(1){

            // Wait until task manager tells it to start
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            reset_counter();
            set_direction(true);

            // Move forward
            reset_encoder();
            start_counting(false);
            clockwise_motor1 = true;
            clockwise_motor2 = true;                     
            target_speed_motor1 = 95;
            target_speed_motor2 = 95;
            APPLY_PID = true;


            vTaskDelay((pdMS_TO_TICKS(100))); // Short delay for motor to spin 

            // Keep moving forward untill ultrasonic is 10cm
            distance = get_distance();
            //DEBUG_PRINT("Distance: %f\n", distance);

            while(distance >= TRIGGER_DISTANCE){

                vTaskDelay(pdMS_TO_TICKS(100)); // Poll sensor at 10Hz
                distance = get_distance();
                //DEBUG_PRINT("Distance: %f\n", distance);

            }

            // Stop
            target_speed_motor1 = 0;
            target_speed_motor2 = 0;
            APPLY_PID = false;
            reset_encoder();
            reset_PID();        

            reset_counter();
            vTaskDelay((pdMS_TO_TICKS(1000))); // Short delay

            // Turn Right
            start_counting(true);
            clockwise_motor1 = true;
            clockwise_motor2 = false;
            target_speed_motor1 = 0;
            target_speed_motor2 = 70;
            APPLY_PID = false;
            reset_PID();

            // Wait till encoder alerts distance is reached
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Stop
            target_speed_motor1 = 0;
            target_speed_motor2 = 0;
            APPLY_PID = false;
            reset_encoder();
            reset_PID();        

            reset_counter();
            vTaskDelay((pdMS_TO_TICKS(2000))); // Short delay

            // Move forward
            set_direction(false);
            reset_encoder();
            clockwise_motor1 = true;
            clockwise_motor2 = true;                     
            target_speed_motor1 = 95;
            target_speed_motor2 = 95;
            APPLY_PID = true;

            // Wait till encoder alerts distance is reached
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Stop
            target_speed_motor1 = 0;
            target_speed_motor2 = 0;
            APPLY_PID = false;
            reset_encoder();
            reset_PID();
            start_counting(false);

            printf("Station 1 Task Completed \n");
            break;

        }
        
    }

}