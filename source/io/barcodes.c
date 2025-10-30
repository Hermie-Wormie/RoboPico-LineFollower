#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "barcodes.h"
#include "queue.h"
#include "timers.h"
#include "wifi.h" // for send_udp_packet and telemetry_ip

// Set 1 to print debug
#if 1
#define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...)
#endif

uint32_t valid_pulses_array[ARRAY_SIZE] = {0};
uint32_t temp_sort_array[ARRAY_SIZE] = {0};
uint8_t array_index = 0;

TimerHandle_t xBarcodeResetTimer;

extern ip_addr_t telemetry_ip;  // only telemetry now

// Reverse last 9 bits of a 16-bit value
uint16_t reverse_bits(uint16_t binary_code) {
    uint16_t last_9_bits = binary_code & 0x01FF;
    uint16_t reversed_9_bits = 0;

    for (int i = 0; i < 9; i++) {
        reversed_9_bits <<= 1;
        reversed_9_bits |= (last_9_bits & 1);
        last_9_bits >>= 1;
    }

    return (binary_code & 0xFE00) | reversed_9_bits;
}

// Decode binary barcode pattern to character
char decode_binary(uint16_t binary_code) {
    uint8_t index = 255;

    DEBUG_PRINT("Now Decoding: 0x%X\n", binary_code);

    for (int i = 0; i < 43; i++) {
        if (binary_code == barcodes[i]) {
            index = i;
            break;
        }
    }

    return (index != 255) ? barcode_chars[index] : '^';
}

// Print binary value (debug)
void print_bits(uint16_t value) {
    for (int i = 15; i >= 0; i--) printf("%d", (value >> i) & 1);
    printf("\n");
}

// Convert pulse widths to binary pattern
uint16_t process_pulses(uint32_t pulses[], int size) {
    uint32_t min_pulse = UINT32_MAX;

    for (uint8_t j = 1; j < size; j++) {
        if (pulses[j] < CALCULATION_IGNORE_THRESHOLD) continue;
        if (pulses[j] < min_pulse) min_pulse = pulses[j];
    }

    float threshold = min_pulse * MULTIPLIER;
    DEBUG_PRINT("\nDynamic Threshold: %.2f\n", threshold);
    DEBUG_PRINT("Full:   ");

    for (int j = 0; j < size; j++) {
        DEBUG_PRINT(pulses[j] > threshold ? "1" : "0");
    }
    DEBUG_PRINT("\n");

    uint16_t barcode_binary = 0;

    for (int j = 10; j < size - 10; j++) {
        barcode_binary <<= 1;
        if (pulses[j] > threshold) barcode_binary |= 1;
    }

    DEBUG_PRINT("uint16: ");
    print_bits(barcode_binary);
    return barcode_binary;
}

// Reset buffers and queue
void reset() {
    for (uint8_t i = 0; i < ARRAY_SIZE; i++) valid_pulses_array[i] = 0;
    xQueueReset(barcodes_queue);
    printf("Array and Queue reset\n");
}

void vTimerCallback(TimerHandle_t xTimer) {
    if (array_index > 0 && array_index < ARRAY_SIZE) {
        printf("Barcode Timer Over\n");
        reset();
        array_index = 0;
    }
}

// Main barcode decoding + telemetry task
void barcode_width_processor(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    printf("Barcode Task Ready\n");

    uint32_t timestamp = 0;
    uint16_t processed_barcode = 0;
    uint16_t reversed_barcode = 0;
    char character = '@';
    char character_reversed = '@';

    xBarcodeResetTimer = xTimerCreate("BarcodeResetTimer", pdMS_TO_TICKS(3000), pdFALSE, 0, vTimerCallback);

    while (1) {
        if (xQueueReceive(barcodes_queue, &timestamp, portMAX_DELAY) == pdTRUE) {
            if (array_index == 0)
                xTimerStart(xBarcodeResetTimer, 0);
            else
                xTimerReset(xBarcodeResetTimer, 0);

            if (timestamp <= WIDTH_IGNORE) {
                DEBUG_PRINT("Valid Pulse: %u\n", timestamp);
                valid_pulses_array[array_index++] = timestamp;
            }

            if (array_index == ARRAY_SIZE) {
                xTimerStop(xBarcodeResetTimer, 0);
                array_index = 0;

                processed_barcode = process_pulses(valid_pulses_array, ARRAY_SIZE);
                reversed_barcode = reverse_bits(processed_barcode);

                character = decode_binary(processed_barcode);
                character_reversed = decode_binary(reversed_barcode);

                if (character == '^' && character_reversed == '^') {
                    send_udp_packet("Fail", &telemetry_ip, 2004);
                    printf("Failed to decode barcode\n");
                } else {
                    char packet[8];
                    sprintf(packet, "%c%c", character, character_reversed);
                    send_udp_packet(packet, &telemetry_ip, 2004);
                    printf("Matched barcode: %c or %c\n", character, character_reversed);
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
                reset();
            }
        }
    }
}
