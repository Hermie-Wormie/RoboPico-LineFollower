#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
#include "header.h"

// From infrared.c
extern volatile bool black_detected;

// Flags
bool BUZZER_INIT = false;
bool HANDSHAKE = false; // Remote connected state
uint8_t CURRENT_MODE = 1;

// Temp for week 10
void swap_interrupts_for_station1(bool state);

void heartbeat_task(void *pvParameters){
    while(1){
        if(HANDSHAKE){
            char pckt[10];
            sprintf(pckt, "%d", total_packets_received);
            send_udp_packet(pckt, &remote_ip, 2004);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void reply_handshake(uint8_t message){
    if (message == (uint8_t) 0xFF){
        send_udp_packet("Pico-Hello", &remote_ip, 2004);
        total_packets_received -= 1; // Remove handshake packet count
        HANDSHAKE = true;
        buzzer(2);
    }
}

void message_handler(void *pvParameters){
    uint8_t message = 1;
    uint8_t previous_command = 1;

    while(1){
        led_off();
        xQueueReceive(received_queue, &message, portMAX_DELAY);
        led_on();
        //printf("main.c: %s total: %d\n", message, total_packets_received);

        // No remote has been connected
        if (!HANDSHAKE){
            reply_handshake(message);
        
        // Remote has already been connected
        } else {

            // Check for special command messsages
            if (message == 0xF1 || message == 0xF2) {
                buzzer(1);

                // Notify the mode_switcher_task of the mode change
                xTaskNotify(TaskManager_T, message, eSetValueWithOverwrite);
            }

            // Check for special command messsages
            if (message == 0xF4) {

                // Allow line follower to take over remote at any time
                xTaskNotifyGive(AutoTaskManager_T);
            }
            
            // Else the message should be a motor command, so send over to core_1
            else if (previous_command != message){
                
                // If latest command received matches the previus command, ignore
                previous_command = message;

                // Only pass motor commands to commands queue if the selected mode is remote control
                if (CURRENT_MODE == 1){

                    // Directly send message to command_queue
                    if (xQueueSend(commands_queue, &message, 0U) != pdPASS) {
                        printf("Command Queue is full!\n"); // You should never see this, hopefully
                    }

                }
                
            }
        }
    }
}

// Quickly read the state of a GPIO pin
bool get_connection_mode(void){

    gpio_init(MODE_SELECT_BUTTON);
    gpio_set_dir(MODE_SELECT_BUTTON, GPIO_IN);
    gpio_pull_up(MODE_SELECT_BUTTON);

    bool state = gpio_get(MODE_SELECT_BUTTON);
    return state;

}

// Auxilary Function
void buzzer(int count){

    if (!BUZZER_INIT){
        
        gpio_init(BUZZER_PIN);
        gpio_set_dir(BUZZER_PIN, GPIO_OUT);

        uint32_t divider = CLOCK_FREQUENCY / (FREQUENCY * 65536);

        gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
        pwm_set_clkdiv(slice, divider);
        pwm_set_wrap(slice, 65535);
        pwm_set_enabled(slice, true);

        BUZZER_INIT = true;

    }

    for (int i = 0; i < count; i++) {

        pwm_set_gpio_level(BUZZER_PIN, 65535 / 2);
        vTaskDelay(pdMS_TO_TICKS(200));
        pwm_set_gpio_level(BUZZER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(200));

    }
}

void auto_task_switcher(void *pvParameters){

    while(1){

        // Wait for notificaiton
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Wait for black
        while(black_detected != 1){
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Tell remote to hold up
        char packet[] = "+\0";  
        send_udp_packet(packet, &remote_ip, 2004);

        // Black detected, prepare to handover control to line follower
        // Purge all pending commands
        xQueueReset(received_queue);
        xQueueReset(commands_queue);

        DEBUG_PRINT("Reset Queues\n");

        // Stop all motor movements
        reset_motor();

        vTaskDelay(pdMS_TO_TICKS(20)); // Small delay

        // Task Management
        vTaskSuspend(Command_T);
        vTaskSuspend(Motor_T);
        vTaskResume(BarcodesPulse_T);

        DEBUG_PRINT("Tasks Set\n");

        // Disable wheel encoders
        swap_interrupts_for_station1(false);
        disable_encoder_interrupts();
        enable_IR_interrupts();

        DEBUG_PRINT("Interrupts Set\n");

        // Reset queue again in case the driver spams while the car is still moving
        xQueueReset(received_queue);
        xQueueReset(commands_queue);

        // Reset to default 4Hz poll
        set_ultrasonic_polldelay(250);

        // Explicitly signal to line following function to proceed with true command.
        xTaskNotify(LineFollowing_T, true, eSetValueWithOverwrite);
        DEBUG_PRINT("Auto Line Following Enabled\n");

    }

}

void task_manager(void *pvParameters) {

    /* Mode Selection
    [1] = Mannual
    [2] = Auto (Line Following)
    [3] = Station 1 */

    uint32_t REQUESTED_MODE = 0; // Can't use uint8 here
    uint8_t FINAL_MODE = 0;      // Cast it later. Guarenteed to be 8 bits.

    // Main Task Loop
    while (1) {

        /* Task Selection Loop. Since the button increments by one, user has to click twice to go from 1 > 3. Or maybe they misclicked.
        With this loop they get a 1 second grace peroid to confirm their selection*/
        while (1) {

            // Wait for a notification indefinitely and retrieve it into REQUESTED_MODE
            if (xTaskNotifyWait(0x00, ULONG_MAX, &REQUESTED_MODE, portMAX_DELAY) == pdPASS) {
                printf("Task Change to: %d requested\n", REQUESTED_MODE);
                FINAL_MODE = (uint8_t)REQUESTED_MODE;
            }

            while(1){

                // Delay for a specified time before checking for new notifications again
                vTaskDelay(pdMS_TO_TICKS(1000));

                // Try to take another message. 
                if (xTaskNotifyWait(0x00, ULONG_MAX, &REQUESTED_MODE, 0) == pdPASS) {

                    // Able to take a message, means the user has requested a change before the timeout
                    FINAL_MODE = (uint8_t)REQUESTED_MODE;
                    printf("New mode %d requested before selection timeout\n", FINAL_MODE);

                } else{

                    // Nothing to take, user did not request a new mode. Break loop and continue with task switching
                    //printf("Nothing new, continuing with %d\n", FINAL_MODE);
                    break;
                }

            }

            break;

        }

        switch (FINAL_MODE) {

            case REMOTE_MODE:

                // Purge all pending commands
                xQueueReset(received_queue);
                xQueueReset(commands_queue);

                DEBUG_PRINT("Reset Queues\n");

                // Stop all motor movements
                reset_motor();

                // Disable line following function, even if it was in progress
                xTaskNotify(LineFollowing_T, false, eSetValueWithOverwrite);

                vTaskDelay(pdMS_TO_TICKS(50)); // Small delay

                // Manage Interrupts
                swap_interrupts_for_station1(false);
                enable_encoder_interrupts();
                enable_IR_interrupts();

                DEBUG_PRINT("Interrupts Enabled\n");

                vTaskDelay(pdMS_TO_TICKS(50)); // Small delay

                // Purge again pending commands (in case user keeps spamming)
                xQueueReset(received_queue);
                xQueueReset(commands_queue);

                // Task Management
                vTaskResume(Command_T);
                vTaskResume(Motor_T);
                vTaskResume(BarcodesPulse_T);

                DEBUG_PRINT("Motor Tasks Resumed\n");

                // Reset to default 4Hz poll
                set_ultrasonic_polldelay(250);

                // Update currently selected mode
                CURRENT_MODE = 1;
                printf("Remote Operation Enabled\n");

                break;

            case AUTOMATIC_MODE:

                // Purge all pending commands
                xQueueReset(received_queue);
                xQueueReset(commands_queue);

                DEBUG_PRINT("Reset Queues\n");

                // Stop all motor movements
                reset_motor();

                // Task Management
                vTaskSuspend(Command_T);
                vTaskSuspend(Motor_T);
                vTaskResume(BarcodesPulse_T);

                DEBUG_PRINT("Tasks Set\n");

                // Disable wheel encoders
                swap_interrupts_for_station1(false);
                disable_encoder_interrupts();
                enable_IR_interrupts();

                DEBUG_PRINT("Interrupts Set\n");

                vTaskDelay(pdMS_TO_TICKS(20)); // Small delay

                // Reset queue again in case the driver spams while the car is still moving
                xQueueReset(received_queue);
                xQueueReset(commands_queue);

                // Reset to default 4Hz poll
                set_ultrasonic_polldelay(250);

                // Update current selected mode
                CURRENT_MODE = 2;

                // Explicitly signal to line following function to proceed with true command.
                xTaskNotify(LineFollowing_T, true, eSetValueWithOverwrite);
                printf("Line Following Enabled\n");

                break;

            case STATION_1:
                break;

            default:
                break;
        }
    }
}