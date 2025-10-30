#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// Motor Pin Definitions (from your motor.h)
#define MOTOR1_PWM_PIN 15
#define MOTOR1_DIR_PIN1 14
#define MOTOR1_DIR_PIN2 13
#define MOTOR2_PWM_PIN 10
#define MOTOR2_DIR_PIN1 11
#define MOTOR2_DIR_PIN2 12

// Encoder Pin Definitions (from your sensors.h)
#define LEFT_ENCODER 2
#define RIGHT_ENCODER 6

// PWM Configuration
#define CLOCK_FREQUENCY 125000000.0f
#define FREQUENCY 100.0f

// Encoder tracking
volatile uint32_t left_encoder_count = 0;
volatile uint32_t right_encoder_count = 0;
volatile uint32_t last_pulse_time_L = 0;
volatile uint32_t last_pulse_time_R = 0;

// Encoder ISR callbacks
void encoder_callback_L(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();
    if (current_time - last_pulse_time_L > 40000) { // Debounce
        left_encoder_count++;
        last_pulse_time_L = current_time;
    }
}

void encoder_callback_R(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();
    if (current_time - last_pulse_time_R > 40000) { // Debounce
        right_encoder_count++;
        last_pulse_time_R = current_time;
    }
}

void interrupt_dispatcher(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENCODER) {
        encoder_callback_L(gpio, events);
    } else if (gpio == RIGHT_ENCODER) {
        encoder_callback_R(gpio, events);
    }
}

void setup_encoders() {
    gpio_init(LEFT_ENCODER);
    gpio_init(RIGHT_ENCODER);
    gpio_set_dir(LEFT_ENCODER, GPIO_IN);
    gpio_set_dir(RIGHT_ENCODER, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER);
    gpio_pull_up(RIGHT_ENCODER);
    
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, true, &interrupt_dispatcher);
    gpio_set_irq_enabled(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
}

void setup_gpio_motor() {
    gpio_init(MOTOR1_DIR_PIN1);
    gpio_set_dir(MOTOR1_DIR_PIN1, GPIO_OUT);
    gpio_init(MOTOR1_DIR_PIN2);
    gpio_set_dir(MOTOR1_DIR_PIN2, GPIO_OUT);
    
    gpio_init(MOTOR2_DIR_PIN1);
    gpio_set_dir(MOTOR2_DIR_PIN1, GPIO_OUT);
    gpio_init(MOTOR2_DIR_PIN2);
    gpio_set_dir(MOTOR2_DIR_PIN2, GPIO_OUT);
}

void setup_pwm_motor() {
    uint32_t divider = CLOCK_FREQUENCY / (FREQUENCY * 65536);
    
    // PWM setup for Motor 1
    gpio_set_function(MOTOR1_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    pwm_set_clkdiv(slice_num1, divider);
    pwm_set_wrap(slice_num1, 65535);
    pwm_set_enabled(slice_num1, true);
    
    // PWM setup for Motor 2
    gpio_set_function(MOTOR2_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    pwm_set_clkdiv(slice_num2, divider);
    pwm_set_wrap(slice_num2, 65535);
    pwm_set_enabled(slice_num2, true);
}

void set_motor_speed(uint8_t motor1_speed, uint8_t motor2_speed, bool forward) {
    // Set direction
    if (forward) {
        gpio_put(MOTOR1_DIR_PIN1, 1);
        gpio_put(MOTOR1_DIR_PIN2, 0);
        gpio_put(MOTOR2_DIR_PIN1, 1);
        gpio_put(MOTOR2_DIR_PIN2, 0);
    } else {
        gpio_put(MOTOR1_DIR_PIN1, 0);
        gpio_put(MOTOR1_DIR_PIN2, 1);
        gpio_put(MOTOR2_DIR_PIN1, 0);
        gpio_put(MOTOR2_DIR_PIN2, 1);
    }
    
    // Set PWM speed
    uint16_t pwm_value1 = motor1_speed * 655; // 0-100 -> 0-65535
    uint16_t pwm_value2 = motor2_speed * 655;
    pwm_set_gpio_level(MOTOR1_PWM_PIN, pwm_value1);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, pwm_value2);
}

void stop_motors() {
    set_motor_speed(0, 0, true);
}

void straight_line_test_task(void *pvParameters) {
    printf("\n===== MOTOR STRAIGHT LINE TEST =====\n");
    printf("This test will run motors at different speeds\n");
    printf("Watch encoder counts to see if they're balanced\n\n");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test speeds to try
    uint8_t test_speeds[] = {50, 60, 70, 80, 90, 100};
    int num_tests = sizeof(test_speeds) / sizeof(test_speeds[0]);
    
    for (int i = 0; i < num_tests; i++) {
        uint8_t speed = test_speeds[i];
        
        printf("\n--- Test %d: Speed %d%% ---\n", i + 1, speed);
        
        // Reset encoder counts
        left_encoder_count = 0;
        right_encoder_count = 0;
        
        // Run motors forward
        printf("Running forward...\n");
        set_motor_speed(speed, speed, true);
        
        // Run for 3 seconds
        for (int sec = 0; sec < 3; sec++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            printf("  Time: %ds | Left: %lu | Right: %lu | Diff: %ld\n", 
                   sec + 1, 
                   left_encoder_count, 
                   right_encoder_count,
                   (int32_t)left_encoder_count - (int32_t)right_encoder_count);
        }
        
        // Stop motors
        stop_motors();
        printf("Final counts - Left: %lu | Right: %lu | Difference: %ld\n",
               left_encoder_count, 
               right_encoder_count,
               (int32_t)left_encoder_count - (int32_t)right_encoder_count);
        
        // Calculate percentage difference
        if (left_encoder_count > 0 && right_encoder_count > 0) {
            float diff_percent = ((float)abs((int32_t)left_encoder_count - (int32_t)right_encoder_count) / 
                                  (float)((left_encoder_count + right_encoder_count) / 2)) * 100.0f;
            printf("Difference: %.2f%%\n", diff_percent);
            
            if (diff_percent < 5.0f) {
                printf("✓ GOOD: Motors are well balanced!\n");
            } else if (diff_percent < 10.0f) {
                printf("⚠ OK: Minor imbalance, might need slight adjustment\n");
            } else {
                printf("✗ BAD: Significant imbalance detected!\n");
                if (left_encoder_count > right_encoder_count) {
                    printf("  -> Left motor is faster. Consider reducing Motor 1 speed\n");
                } else {
                    printf("  -> Right motor is faster. Consider reducing Motor 2 speed\n");
                }
            }
        }
        
        // Wait before next test
        printf("Waiting 2 seconds before next test...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    printf("\n===== TEST COMPLETE =====\n");
    printf("Summary: Check the differences above.\n");
    printf("Adjust motor speeds in your code if needed.\n");
    
    // Continuous monitoring mode
    printf("\n--- Entering continuous monitoring mode ---\n");
    printf("Motors at 70%% speed. Press Ctrl+C to stop.\n\n");
    
    left_encoder_count = 0;
    right_encoder_count = 0;
    set_motor_speed(70, 70, true);
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("Left: %lu | Right: %lu | Diff: %ld\n",
               left_encoder_count,
               right_encoder_count,
               (int32_t)left_encoder_count - (int32_t)right_encoder_count);
    }
}

int main(void) {
    stdio_init_all();
    cyw43_arch_init();
    
    printf("\n\n===== INITIALIZING MOTOR TEST =====\n");
    
    setup_gpio_motor();
    setup_pwm_motor();
    setup_encoders();
    
    printf("Hardware initialized successfully\n");
    
    // Create test task
    xTaskCreate(straight_line_test_task, "StraightTest", 1024, NULL, 1, NULL);
    
    // Start scheduler
    vTaskStartScheduler();
    
    while (1) {};
}