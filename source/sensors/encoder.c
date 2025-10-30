#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sensors.h"
#include "queue.h"

volatile uint32_t pulse_width_L = 0;
volatile uint32_t pulse_width_R = 0;
volatile uint32_t last_pulse_time_L = 0;
volatile uint32_t last_pulse_time_R = 0;
uint32_t encoder_counter = 0;

extern QueueHandle_t encoder_queue;
extern TaskHandle_t Station1_T;

float total_distance = 0;
float current_speed = 0;

uint8_t RIGHT_TURN_COUNT = 15;
uint8_t FORWARD_COUNT = 80;
volatile bool START_COUNTING = false;
volatile bool DIRECTION = true; //true is right, false is forward

// From ultrasonic.c
extern volatile float distance;

void encoder_callback_L(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();

    if (last_pulse_time_L == 0) {
        last_pulse_time_L = current_time;
        return;
    }

    pulse_width_L = current_time - last_pulse_time_L;

    if (pulse_width_L >= MINIMUM_DEBOUCE){
        last_pulse_time_L = current_time;
    }
    
}

// Needs to double duty and also count
void encoder_callback_R_Station1(uint gpio, uint32_t events) {

    uint32_t current_time = time_us_32();

    if (last_pulse_time_R == 0) {
        last_pulse_time_R = current_time;
        return;
    }

    pulse_width_R = current_time - last_pulse_time_R;

    if (pulse_width_R >= MINIMUM_DEBOUCE){
        last_pulse_time_R = current_time;


        if(START_COUNTING){

            encoder_counter += 1;

            if (DIRECTION){

                if (encoder_counter >= RIGHT_TURN_COUNT){
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Needed for context switching
                    vTaskNotifyGiveFromISR(Station1_T, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            } 
            
            else{
                if (encoder_counter >= FORWARD_COUNT){
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Needed for context switching
                    vTaskNotifyGiveFromISR(Station1_T, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            }
        }

    }
}

void encoder_callback_R(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();

    if (last_pulse_time_R == 0) {
        last_pulse_time_R = current_time;
        return;
    }

    pulse_width_R = current_time - last_pulse_time_R;

    if (pulse_width_R >= MINIMUM_DEBOUCE){
        last_pulse_time_R = current_time;
    }

    encoder_counter += 1;

}

void reset_encoder() {

    // Reset encoder variables
    pulse_width_L = 0;
    pulse_width_R = 0;
    last_pulse_time_L = 0;
    last_pulse_time_R = 0;
    
}

void reset_counter(){
    encoder_counter = 0;
}

void set_direction(bool state){
    DIRECTION = state;
}

void start_counting(bool state){
    START_COUNTING = state;
}

void telemetry_task(){

    static const float distance_per_notch = 0.3318 / 20;
    float time_per_notch = 0;
    char buffer[16];

    while(1){

        total_distance = encoder_counter * distance_per_notch;
        time_per_notch = pulse_width_R * 0.000001; // convert to seconds

        if (pulse_width_R == 0) {
            current_speed = 0;
        }else{
            current_speed = distance_per_notch / time_per_notch;
        }
        
        sprintf(buffer, "%.3f-%.3f-%.3f\n", total_distance, current_speed, distance);
        send_udp_packet(buffer, &telemetry_ip, 2004);
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
    
}