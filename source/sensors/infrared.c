#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "sensors.h"
#include "queue.h"

// Line Following
volatile bool black_detected = false;
extern QueueHandle_t barcodes_queue;

void ir_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        black_detected = true; // Rising edge detected - black detected

    } else if (events & GPIO_IRQ_EDGE_FALL) {
        black_detected = false; // Falling edge detected - white detected
    }
}

void ir_callback_barcode(uint gpio, uint32_t events) {

    static uint32_t last_timestamp = 0; // Static to retain value between ISR calls
    uint32_t current_time = time_us_32();
    uint32_t elapsed_time;

    if(last_timestamp != 0) {

        elapsed_time = current_time - last_timestamp;

        if(elapsed_time >= MIN_DEBOUNCE) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(barcodes_queue, &elapsed_time, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    } else {
        // First event, no previous timestamp to compare
    }

    // Update last_timestamp for the next ISR call
    last_timestamp = current_time;
}