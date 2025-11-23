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
#include "motor.h" // for motor_stop()
#include "telemetry.h"

// Set 1 to print debug
#if 1
#define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...)
#endif

uint32_t valid_pulses_array[ARRAY_SIZE] = {0}; // Buffer to store raw pulse widths (time duration in ms) for one barcode
uint32_t temp_sort_array[ARRAY_SIZE] = {0}; // Temp array for sorting pulse widths for unit width
uint8_t array_index = 0; // Index for tracking current fill level of valid_pulses_array

TimerHandle_t xBarcodeResetTimer; // FreeRTOS timer handle used to detect if barcode scanning times out

// Array to store sequence of decoded char
#define MAX_BARCODES 50
char decoded_chars_array[MAX_BARCODES];
uint8_t barcode_count = 0;

extern ip_addr_t telemetry_ip;  // External IP address for UDP telemetry (defined in wifi.c)

// Reverse last 9 bits of a 16-bit value
uint16_t reverse_bits(uint16_t binary_code) {
    uint16_t last_9_bits = binary_code & 0x01FF;
    uint16_t reversed_9_bits = 0;

    for (int i = 0; i < 9; i++) {
        reversed_9_bits <<= 1;
        reversed_9_bits |= (last_9_bits & 1);
        last_9_bits >>= 1;
    }

    //return (binary_code & 0xFE00) | reversed_9_bits;
    return reversed_9_bits;
}

// Decode binary barcode pattern to character
char decode_binary(uint16_t binary_code) {
    uint8_t index = 255;
    uint8_t min_distance = 255;
    uint8_t best_match_index = 255;

    DEBUG_PRINT("Now Decoding: 0x%X\n", binary_code);

    for (int i = 0; i < 44; i++) { // Loop through all 44 barcodes
        uint8_t distance = calculate_hamming_distance(binary_code, barcodes[i]);

        if (distance < min_distance) {
            min_distance = distance;
            best_match_index = i;
        }
    }

    if (min_distance <= 2) {
        // If HD is 0, 1, or 2, we accept it as a valid character
        DEBUG_PRINT("Matched Code 0x%X with HD=%u.\n", barcodes[best_match_index], min_distance);
        return barcode_chars[best_match_index];
    } else {
        // If HD is 3 or more, it's too different from any valid code.
        DEBUG_PRINT("Failed: Minimum Hamming Distance was %u (too high).\n", min_distance);
        return '^'; // Return failure character
    }

    // for (int i = 0; i < 43; i++) {
    //     if (binary_code == barcodes[i]) {
    //         index = i;
    //         break;
    //     }
    // }

    // return (index != 255) ? barcode_chars[index] : '^';
}

// Print binary value (debug)
void print_bits(uint16_t value) {
    for (int i = 15; i >= 0; i--) printf("%d", (value >> i) & 1);
    printf("\n");
}

// Convert pulse widths to binary pattern
// return uint16_t The resulting binary barcode pattern.
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

    for (uint8_t i = 0; i < MAX_BARCODES; i++) decoded_chars_array[i] = '\0';
    barcode_count = 0;
    printf("Array and Queue reset\n");
}

// FreeRTOS Timer Callback function. Called when the barcode sensor times out.
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
            if (array_index == 0) {
                xTimerStart(xBarcodeResetTimer, 0);
                SCANNING_BARCODE = true;
            }else {
                xTimerReset(xBarcodeResetTimer, 0);
            }
            if (timestamp <= WIDTH_IGNORE) {
                DEBUG_PRINT("[%u] Valid Pulse: %u\n", array_index, timestamp);
                valid_pulses_array[array_index++] = timestamp;
            }

            if (array_index == ARRAY_SIZE) {
                xTimerStop(xBarcodeResetTimer, 0);
                array_index = 0;

                // processed_barcode = process_pulses(valid_pulses_array, ARRAY_SIZE);
                // reversed_barcode = reverse_bits(processed_barcode);

                // character = decode_binary(processed_barcode);
                // character_reversed = decode_binary(reversed_barcode);

                // if (character == '^' && character_reversed == '^') {
                //     send_udp_packet("Fail", &telemetry_ip, 2004);
                //     printf("Failed to decode barcode\n");
                // } else {
                //     char packet[8];
                //     sprintf(packet, "%c%c", character, character_reversed);
                //     send_udp_packet(packet, &telemetry_ip, 2004);
                //     printf("Matched barcode: %c or %c\n", character, character_reversed);
                // }

                //char decoded_char = '^';
                char decoded_char = decode_character_normalized(valid_pulses_array); // New logic                
                // if (character != '^') {
                //     decoded_char = character;
                // } else if (character_reversed != '^') {
                //     decoded_char = character_reversed;
                // }

                //if (decoded_char != '\0') { // new logic
                if (decoded_char != '^' && barcode_count < MAX_BARCODES) {
                    decoded_chars_array[barcode_count++] = decoded_char;
                    printf("Decoded Character #%u: %c\n", barcode_count, decoded_char);

                    char decision_char = decoded_chars_array[barcode_count - 1];

                    g_telemetry_data.last_barcode_command = decision_char;

                    // Check if the character is a turn command
                    if (decision_char >= 'A' || decision_char <= 'Z') {
                        
                        char turn_command_char;
                        // Calculate position: A=1, B=2, C=3, etc.
                        // Odd letters (A, C, E, ...): Move Right
                        // Even letters (B, D, F, ...): Move Left
                        if ((decision_char - 'A' + 1) % 2 != 0) {
                            // Odd letter: Move Right
                            turn_command_char = 'R'; 
                            printf("DECISION: Odd letter '%c' -> Move RIGHT.\n", decision_char);
                        } else {
                            // Even letter: Move Left
                            turn_command_char = 'L';
                            printf("DECISION: Even letter '%c' -> Move LEFT.\n", decision_char);
                        }
                        
                        motor_stop(); // Halt the robot immediately
                        vTaskDelay(pdMS_TO_TICKS(1000));

                        if (xQueueSend(turn_command_queue, &decision_char, 0) != pdPASS) {
                            printf("ERROR: Failed to send turn command to queue!\\n");
                        }
                        
                        printf("ACTION: Barcode Read: %c. Turn command sent to line follower.\\n", decision_char);
                        printf("****************************************\\n");

                        // Reset barcode process and let LineTask handle the movement
                        vTaskDelay(pdMS_TO_TICKS(100));
                        reset(); // Reset barcode sensor logic
                        g_telemetry_data.last_barcode_command = '\0';
                        continue; // Skip the rest of the loop and wait for the next pulse
                    } else {
                        printf("ACTION: Unknown command (%c). Proceeding...\\n", decision_char);
                        printf("****************************************\\n");
                        
                        // After making the decision, you likely want to reset and look for the next path
                        vTaskDelay(pdMS_TO_TICKS(100));
                        reset();
                        g_telemetry_data.last_barcode_command = '\0';

                        continue; // Skip the rest of the loop and wait for the next pulse
                    }
                    
                    // After successfully reading one character (but not the 5th),
                    // clear the pulse array but NOT the sequence array/count, 
                    // and wait for the start of the next character's pulse train.
                    for (uint8_t i = 0; i < ARRAY_SIZE; i++) valid_pulses_array[i] = 0;
                    array_index = 0;
                    xQueueReset(barcodes_queue);
                    xTimerStart(xBarcodeResetTimer, 0); // Restart the timeout timer for the next character
                    
                } else {
                    printf("Failed to decode character or max barcodes reached. Resetting...\n");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    reset();
                }
                SCANNING_BARCODE = false;
                vTaskDelay(pdMS_TO_TICKS(1000));
                reset();
            }
        }
    }
}

// Helper Function
// Sorts an array of pulse widths in ascending order (using simple bubble sort).
// param: arr Array of pulse widths, size Number of elements
void sort_pulses(uint32_t arr[], uint8_t size) {
    uint8_t i, j;
    uint32_t temp;
    for (i = 0; i< size - 1; i++) {
        for (j = 0; j < size - i - 1; j++) {
            // Only sort non-zero pulses
            if (arr[j] > arr[j + 1] && arr[j+1] != 0) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

// --- Pulse Array to Character Decoder (Replace your existing inline logic) ---
// return char Decoded character, or '\0' or '^' on failure
char decode_character_normalized(uint32_t *pulses_array) {
    uint8_t i;
    uint32_t sum_narrow = 0;
    uint8_t narrow_count = 0;
    
    // STEP 1: Copy and Sort to find the narrow unit width 
    // Uses global temp_sort_array (defined in your file)
    memcpy(temp_sort_array, pulses_array, sizeof(uint32_t) * ARRAY_SIZE);
    sort_pulses(temp_sort_array, ARRAY_SIZE);

    // STEP 2: Calculate the Narrow Unit Width (T_unit) 
    // Average the narrowest 5 pulses (T-widths) for robustness.
    for (i = 0; i < ARRAY_SIZE; i++) {
        if (temp_sort_array[i] > CALCULATION_IGNORE_THRESHOLD) {//MIN_DEBOUNCE) { 
            if (narrow_count < 5) { 
                sum_narrow += temp_sort_array[i];
                narrow_count++;
            }
        }
    }

    if (narrow_count == 0) return '\0';
    
    // T_unit is the average of the narrowest pulses
    uint32_t T_unit = sum_narrow / narrow_count;
    
    // Dynamic Threshold: Halfway between T (Narrow) and 3T (Wide), which is 2T.
    //uint32_t DYNAMIC_THRESHOLD = (T_unit * 20) / 10; //2; 
    uint32_t DYNAMIC_THRESHOLD = (uint32_t)((float)T_unit * MULTIPLIER);

    DEBUG_PRINT("T_unit: %lu us, Dynamic Threshold: %lu us\n", T_unit, DYNAMIC_THRESHOLD);

    // STEP 3: Translate Pulses to 9-bit Binary Code
    uint16_t binary_code = 0;
    uint8_t bit_index = 0; 
    
    // Code 39 bars/spaces are at odd indices (1, 3, 5, ... 17)
    for (i = 10; i <= 18; i++) { // Iterate only through the 9 bar/space elements
        
        binary_code <<= 1;

        if (pulses_array[i] >= DYNAMIC_THRESHOLD) {
            // Wide element (3T) -> Set bit to 1
            //binary_code |= (1 << bit_index);
            binary_code |= 1;
        } else {
            // Narrow element (T) -> Bit is 0
        }
        
        bit_index++;
    }

    // You will need to keep your existing reverse_bits and match_code_to_char functions.
    //binary_code = reverse_bits(binary_code); 
    
    return decode_binary(binary_code); 
}

// Calculates hamming distance (number of differing bits) between two 9-bit codes
// code1 First binary code
// code2 second bin code
// returns uint8_t Hamming dist.
uint8_t calculate_hamming_distance(uint16_t code1, uint16_t code2) {
    uint16_t xor_result = code1 ^ code2;
    uint8_t distance = 0;
    
    // Count the number of set bits (1s) in the XOR result
    for (int i = 0; i < 9; i++) {
        if ((xor_result >> i) & 1) {
            distance++;
        }
    }
    return distance;
}