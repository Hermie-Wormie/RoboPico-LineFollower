#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "barcodes.h"
#include "queue.h"
#include "timers.h"

// Set 1 to print
#if 1
#define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...) // Nothing happens if DEBUG is 0
#endif

uint32_t valid_pulses_array[ARRAY_SIZE] = {0};
uint32_t temp_sort_array[ARRAY_SIZE] = {0};
uint8_t array_index = 0;

TimerHandle_t xBarcodeResetTimer;

// Just the last 9 bits only
uint16_t reverse_bits(uint16_t binary_code) {

    uint16_t last_9_bits = binary_code & 0x01FF; // Mask out the last 9 bits (0b0000000111111111)
    uint16_t reversed_9_bits = 0;

    // Reverse the last 9 bits
    for (int i = 0; i < 9; i++) {
        reversed_9_bits <<= 1;
        reversed_9_bits |= (last_9_bits & 1);
        last_9_bits >>= 1;
    }

    // Combine the reversed 9 bits with the remaining bits in binary_code
    uint16_t result = (binary_code & 0xFE00) | reversed_9_bits;

    return result;
}

char decode_binary(uint16_t binary_code){
    uint8_t index = 255; // Preset to an invalid number

    DEBUG_PRINT("Now Decoding: 0x%X\n", binary_code);

    // Search through the array using both original and reversed codes
    for (int i = 0; i < 43; i++) {
        if (binary_code == barcodes[i]) {
            index = i;
            break;
        }
    }

    // If index is not 255, character was found. return it
    if (index != 255){
        return barcode_chars[index];
    } else {
        return '^'; // If not found, return a value not found in code39
    }
}

// TESTING FUNCTION
void print_bits(uint16_t value) {
    // Loop over each bit from the most significant to the least significant
    for (int i = 15; i >= 0; i--) {
        // Print the bit at the current position
        printf("%d", (value >> i) & 1);
    }
    printf("\n"); // Newline for better readability
}

// Function to process an array of pulse widths to short or long bars in binary
uint16_t process_pulses(uint32_t pulses[], int size) {

    uint32_t min_pulse = UINT32_MAX; // Start off with the max value! Not 0 lol.

    for (uint8_t j = 1; j < size; j++) { // Ignore first value, as it is usually an outlier
        if (pulses[j] < CALCULATION_IGNORE_THRESHOLD) {
            continue;  // Ignore pulses below threshold for calculating threshold
        }
        if (pulses[j] < min_pulse) {
            min_pulse = pulses[j];
        }
    }

    // Calculate the dynamic threshold using 2nd lowest
    float threshold = min_pulse * MULTIPLIER;
    DEBUG_PRINT("\nDynamic Threshold: %.2f\n", threshold);
    DEBUG_PRINT("Full:   ");

        // Classify each pulse as short or long
    for (int j = 0; j < size; j++) { // Start from the 2nd pulse and end one before the last
    
    // Seems wrong. But is correct to shift left before we write anything. If we shifted after writing
    // We would get a hanging 0 at the back.

        if (pulses[j] > threshold) {
                DEBUG_PRINT("1");
            } else {
                DEBUG_PRINT("0");
            }
    }
    DEBUG_PRINT("\n");

    DEBUG_PRINT("Middle: ");

    uint16_t barcode_binary = 0; // Holds the binary representation of the barcode

    // Classify each pulse as short or long
    for (int j = 10; j < size - 10; j++) { // Start from the 2nd pulse and end one before the last
    
    // Seems wrong. But is correct to shift left before we write anything. If we shifted after writing
    // We would get a hanging 0 at the back.
    barcode_binary <<= 1; 

        if (pulses[j] > threshold) {
                barcode_binary |= 1;
                DEBUG_PRINT("1");
            } else {
                DEBUG_PRINT("0");
            }
    }

    DEBUG_PRINT("\nuint16: ");
    print_bits(barcode_binary);
    return barcode_binary;
}

// Reset array and queues for repeated testing. 
void reset(){

    for (uint8_t i = 0; i < ARRAY_SIZE; i++) {
        valid_pulses_array[i] = 0;
    }

    xQueueReset(barcodes_queue);
    printf("Array and Queue reset\n");
}

void vTimerCallback(TimerHandle_t xTimer){

    if (array_index > 0 && array_index < ARRAY_SIZE) {
        printf("Barcode Timer Over\n");
        reset();
        array_index = 0;
    }
}

// Sample task
void barcode_width_processor(void *pvParameters) {

    vTaskDelay(pdMS_TO_TICKS(3000));
    printf("Barcode Task Ready\n");

    uint32_t timestamp = 0;
    uint16_t processed_barcode = 0;
    uint16_t reversed_barcode = 0;
    char character = '@';
    char character_reversed = '@';

    // Create the timer with 5 sec reset
    xBarcodeResetTimer = xTimerCreate("BarcodeResetTimer", pdMS_TO_TICKS(3000), pdFALSE, 0, vTimerCallback);

    while (1) {

        // Wait for the first timestamp from ISR
        if (xQueueReceive(barcodes_queue, &timestamp, portMAX_DELAY) == pdTRUE) {

            // Start or reset the timer on the first pulse
            if (array_index == 0) {
                xTimerStart(xBarcodeResetTimer, 0);
            } else {
                xTimerReset(xBarcodeResetTimer, 0); // Reset timer on every pulse
            }

            // Ensures environmental values don't get past
            if (timestamp <= WIDTH_IGNORE) {
                DEBUG_PRINT("Valid Pulse: %u\n", timestamp);
                valid_pulses_array[array_index] = timestamp;
                array_index++;
            }

            // When array size is full, start process
            if (array_index == ARRAY_SIZE) {

                // Stop the timer since array is full
                xTimerStop(xBarcodeResetTimer, 0);

                // Reset index for next run
                array_index = 0;

                // Process barcode into binary
                processed_barcode = process_pulses(valid_pulses_array, ARRAY_SIZE);
                reversed_barcode = reverse_bits(processed_barcode);

                // Check if it matches known barcodes
                character = decode_binary(processed_barcode);
                character_reversed = decode_binary(reversed_barcode);

                if (character == '^' && character_reversed == '^') {

                    // Send failure message to remote
                    send_udp_packet("-Fail", &remote_ip, 2004);
                    send_udp_packet("Fail", &telemetry_ip, 2004);
                    printf("Failed\n");
                } else {

                    // Send character message to remote
                    char packet[4];
                    sprintf(packet, "-%c%c", character, character_reversed);
                    send_udp_packet(packet, &remote_ip, 2004);
                    send_udp_packet(packet, &telemetry_ip, 2004);
                    printf("Matched with %c or %c\n", character, character_reversed);
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
                reset();
            }
        }
    }
}
