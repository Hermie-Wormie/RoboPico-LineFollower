#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "header.h"
#include "motor.h"
#include "telemetry.h"

// Queues
QueueHandle_t barcodes_queue = NULL; 
QueueHandle_t encoder_queue = NULL;
QueueHandle_t turn_command_queue = NULL;

// Task handles
TaskHandle_t LED_T = NULL;
TaskHandle_t GPIO_T = NULL;
TaskHandle_t UDP_T = NULL;
TaskHandle_t Heartbeat_T = NULL;
TaskHandle_t Motor_T = NULL;
TaskHandle_t LineFollowing_T = NULL;
TaskHandle_t BarcodesPulse_T = NULL;
TaskHandle_t Ultrasonic_T = NULL;
TaskHandle_t TestHandle_1 = NULL;
TaskHandle_t TestHandle_2 = NULL;

volatile bool IN_MANEUVER = false;
volatile bool SCANNING_BARCODE = false; // Flag to signal reduced speed

telemetry_data_t g_telemetry_data = {0};

// Optional telemetry task
void telemetry_task();

bool HOTSPOT = true;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    while (1)
    {
        printf("STACK OVERFLOW from task: %s (Handle: %p)\n", pcTaskName, (void *)xTask);
        sleep_ms(500);
    }
}

void vApplicationMallocFailedHook(void)
{
    while (1)
    {
        printf("MALLOC FAILED\n");
        sleep_ms(500);
    }
}

// void barcode_debug_trigger_task(void *pvParameters) {
//     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 5 seconds after startup
    
//     printf("--- BARCODE DEBUG TRIGGER STARTED: Sending 'A' (Left) every 15s ---\n");
//     char command_A = 'A';

//     while(1) {
//         printf("DEBUG: Sending 'A' (Left Turn) command to queue.\n");
//         // 'A' for Left Turn
//         xQueueSend(turn_command_queue, &command_A, portMAX_DELAY);

//         vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 15 seconds for the next trigger
//     }
// }

int main(void)
{
    // Initialise everything
    stdio_init_all();
    cyw43_arch_init(); // MQTT & Wi-Fi
    motor_init();
    hmc5883l_init(); // IMU init
    setup_interrupts();
    create_semaphores();
    servo_init(); // Init Ultrasonic servo

    // Create queues
    barcodes_queue = xQueueCreate(40, sizeof(uint32_t));
    encoder_queue = xQueueCreate(10, sizeof(uint16_t));
    turn_command_queue = xQueueCreate(1, sizeof(char));

    printf("Free heap before scheduler: %u bytes\n", xPortGetFreeHeapSize());

    // ====== SET PID TARGETS HERE ======
    target_speed_motor1 = 20; // %
    target_speed_motor2 = 20; // %
    motor_heading_lock_arm();
    APPLY_PID = false;
    // ==================================

    // ---------------- TASK CREATION ----------------

    // xTaskCreate(blink, "LEDTask", 512, NULL, 1, &LED_T);

    // int GPIO_PIN = 28;
    // xTaskCreate(GPIO_blink, "GPIOTask", 512, &GPIO_PIN, 1, &GPIO_T);

    // // Wi-Fi (Access Point) + UDP Telemetry
    //xTaskCreate(start_UDP_server_ap, "UDPTask", 1024, NULL, 3, &UDP_T);
    xTaskCreate(start_UDP_server_hotspot, "UDPTask", 1024, NULL, 3, &UDP_T);

    xTaskCreate(compass_debug_task, "HMCdbg", 512, NULL, 1, NULL);

    // Motor control
    xTaskCreate(motor_task, "MotorTask", 1024, NULL, 3, &Motor_T);

    // Line following
    xTaskCreate(line_following_task, "LineTask", 1024, NULL, 2, &LineFollowing_T);

    // // Ultrasonic sensor
    xTaskCreate(ultrasonic_task, "UltrasonicTask", 1024, NULL, 2, &Ultrasonic_T);

    // // Barcode decoding
    xTaskCreate(barcode_width_processor, "BarcodeTask", 2048, NULL, 3, &BarcodesPulse_T);
    //1xTaskCreate(barcode_debug_trigger_task, "BarcodeDbg", 512, NULL, 1, NULL);    // // Optional telemetry heartbeat

    xTaskCreate(telemetry_task, "TelemetryTask", 1024, NULL, 1, NULL);

    xTaskCreate(encoder_debug_task, "EncDbg", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1)
    {}
}

