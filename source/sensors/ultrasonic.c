#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "sensors.h"
#include "semphr.h"

volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_end = 0;
uint16_t POLLING_DELAY = 250;
SemaphoreHandle_t Ultrasonic_BinarySemaphore;
SemaphoreHandle_t UltrasonicWarn_BinarySemaphore; //extern'd

// For debug
volatile float distance = 0;

// Gets called from interrupt_dispatcher from interrupt.c
void ultrasonic_callback(uint gpio, uint32_t events){

    if (events & GPIO_IRQ_EDGE_RISE) {
        // Rising edge detected: Start timing
        pulse_start = time_us_32(); // Record start time in microseconds

    } else {
        // Falling edge detected
        pulse_end = time_us_32();

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Release semaphore now that falling edge has been detected (ie. ready to calculate)
        xSemaphoreGiveFromISR(Ultrasonic_BinarySemaphore, &xHigherPriorityTaskWoken);

        // Yield to a higher prioriy task after completion (if there is, currently does nothing)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

}

// Function to measure distance with HC-SR04
float get_distance() {

    // Clear the semaphore before starting
    xSemaphoreTake(Ultrasonic_BinarySemaphore, 0);

    // Send trigger pulse
    gpio_put(TRIG_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(0.01));  // Wait for 10 ms
    gpio_put(TRIG_PIN, 0);

    // Wait for measurement to complete
    if (xSemaphoreTake(Ultrasonic_BinarySemaphore, pdMS_TO_TICKS(TIMEOUT)) == pdTRUE) {
        // Measurement complete
        uint32_t pulse_duration = pulse_end - pulse_start;
        float distance = (pulse_duration * 0.01724f);  // Conversion factor to get cm

        // Transform with predefined calibration values
        distance = (distance*CALIBRATION_SCALING) + CALIBRATION_OFFSET;
        return distance;
        
    } else {
        // Timeout or error
        return 999.0f;  // Indicate error
    }
}

// Make sure you make the semaphores first before any task uses it or it will crash!!
void create_semaphores(){

    Ultrasonic_BinarySemaphore = xSemaphoreCreateBinary();
    UltrasonicWarn_BinarySemaphore = xSemaphoreCreateBinary();

    if (Ultrasonic_BinarySemaphore == NULL) {
        while(1){
            printf("Failed to create Ultrasonic_BinarySemaphore\n");
            sleep_ms(500);
        }
    }

    if (UltrasonicWarn_BinarySemaphore == NULL) {
        while(1){
            printf("Failed to create UltrasonicWarn_BinarySemaphore\n");
            sleep_ms(500);
        }
    }
}

void ultrasonic_task(void *pvParameters){

    bool warning_already_raised = false;

    // Clear the semaphore before starting
    xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0);

    // For debug
    distance = 0;

    vTaskDelay(pdMS_TO_TICKS(3000));
    printf("Starting Ultrasonic Task\n");

    while(1){
            
        distance = get_distance();
        if(distance <= MIN_DISTANCE){

            printf("Distance: %.4f\n",distance);

            if (!warning_already_raised){
                warning_already_raised = true;
                xSemaphoreGive(UltrasonicWarn_BinarySemaphore);
            }
            
        } else{
            if (warning_already_raised){
                warning_already_raised = false;
                xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0);
                disable_warning();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY));

    }

}

void set_ultrasonic_polldelay(uint16_t delay){
    POLLING_DELAY = delay;
}