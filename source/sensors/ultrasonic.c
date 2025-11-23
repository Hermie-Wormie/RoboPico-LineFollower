#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "sensors.h"
#include "semphr.h"
#include "telemetry.h"
#include <math.h>

void motor_start_avoidance(void);

volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_end = 0;
uint16_t POLLING_DELAY = 250;
SemaphoreHandle_t Ultrasonic_BinarySemaphore;
SemaphoreHandle_t UltrasonicWarn_BinarySemaphore; //extern'd

// For debug
volatile float distance = 0;

// Servo
#define PULSE_MIN_US 500  // ~0 degrees (0.5ms)
#define PULSE_MAX_US 2500 // ~180 degrees (2.5ms)

// The target angles for scan (Central, -30 deg Left, +30 deg Right)
// Assuming 90 deg is the 'center' (forward) position.
#define ANGLE_CENTER 25.0f
#define ANGLE_LEFT_SCAN (ANGLE_CENTER - 45.0f) // 60 deg
#define ANGLE_RIGHT_SCAN (ANGLE_CENTER + 45.0f) // 120 deg

#define MIN_ABSOLUTE_CLEARANCE_CM 20.0f
#define PATH_ADVANTAGE_CM 5.0f

#define DETECTION_THRESHOLD_CM 30.0f
#define MAX_SCAN_HALF_ANGLE_RAD (45.0f * M_PI / 180.0f)

static uint slice_num;

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
    sleep_us(10);
    //vTaskDelay(pdMS_TO_TICKS(0.01));  // Wait for 10 ms
    gpio_put(TRIG_PIN, 0);

    // Wait for measurement to complete
    if (xSemaphoreTake(Ultrasonic_BinarySemaphore, pdMS_TO_TICKS(TIMEOUT)) == pdTRUE) {
        // Measurement complete
        uint32_t pulse_duration = pulse_end - pulse_start;
        // *** ACCURACY CONSIDERATION: Standard conversion for cm (speed of sound/2) ***
        // 343 m/s = 34300 cm/s. 
        // 1/34300 = 0.00002915 s/cm.
        // pulse_duration (us) * 1e-6 (s/us) * 34300 (cm/s) / 2 
        // = pulse_duration * 0.01715
        float distance = (pulse_duration * 0.01715f);
        //float distance = (pulse_duration * 0.01724f);  // Conversion factor to get cm

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

float calculate_object_width_cm(float center_dist) {
    if (center_dist < DETECTION_THRESHOLD_CM && center_dist > 0.1f) {
        // Simple trigonometric approximation: Width ≈ 2 * Distance * tan(Half_Angle)
        return 2.0f * center_dist * tanf(MAX_SCAN_HALF_ANGLE_RAD); 
    }
    return 0.0f;
}

// task for periodically reading the ultrasonic sensor and checking for warnings.
// Runs frequently to provide the current obstacle distance and sets the global 
// DistanceWarning flag if an object is too close
void ultrasonic_task(void *pvParameters){

    bool warning_already_raised = false;
    //float current_distance = 0;

    // Clear the semaphore before starting
    xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0);

    // For debug
    distance = 0;

    vTaskDelay(pdMS_TO_TICKS(3000));
    printf("Starting Ultrasonic Task\n");

    while(1){
            
        // For  telemetry
        float center_distance = get_distance();
        // Object dist
        g_telemetry_data.object_distance_cm = center_distance;
        // Object is detected
        g_telemetry_data.object_detected = (center_distance < DETECTION_THRESHOLD_CM && center_distance > 0.1f);
        // Approximate width of object
        g_telemetry_data.object_width_cm = calculate_object_width_cm(center_distance);

        distance = get_distance();
        //printf("[Debug] Distance: %.4f\n",distance);
        if(distance <= MIN_DISTANCE){

            printf("Distance: %.4f\n",distance);

            if (!warning_already_raised){
                warning_already_raised = true;
                xSemaphoreGive(UltrasonicWarn_BinarySemaphore);
                //turn_away();
                motor_start_avoidance();
                //vTaskSuspend(NULL);
            }
            
        } else{
            if (warning_already_raised){
                warning_already_raised = false;
                xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0);
                //disable_warning();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY));

    }

}

void set_ultrasonic_polldelay(uint16_t delay){
    POLLING_DELAY = delay;
}

// ==========================================
// Servo Section

// Maps angle (0-180) to PWM pulse width in microseconds
static uint16_t angle_to_pulse(float angle) {
    // Clamp angle to 0-180
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // Linearly map angle to pulse width
    return (uint16_t)(PULSE_MIN_US + (angle / 180.0f) * (PULSE_MAX_US - PULSE_MIN_US));
}

void servo_set_angle(float angle) {
    uint16_t pulse_us = angle_to_pulse(angle);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(SERVO_PIN), pulse_us);
}

void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    // Set PWM for 50Hz
    pwm_set_clkdiv_int_frac(slice_num, (uint8_t)CLOCK_DIV, 0); // 125 div
    pwm_set_wrap(slice_num, PWM_WRAP_50HZ);
    pwm_set_enabled(slice_num, true);

    // Set to center angle initially
    servo_set_angle(ANGLE_CENTER);
}

// Scans the environment and determines the best path
// Returns 1 for LEFT path, -1 for RIGHT path, 0 for BLOCKED
int servo_scan_path(float *left_dist, float *right_dist) {
    
    // 1. Move to Left Scan Position
    servo_set_angle(ANGLE_LEFT_SCAN);
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for servo to move
    *left_dist = get_distance();
    printf("Scan Left: %.2f cm\n", *left_dist);

    // 2. Move back to Center (optional, but good for stability)
    servo_set_angle(ANGLE_CENTER);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Move to Right Scan Position
    servo_set_angle(ANGLE_RIGHT_SCAN);
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for servo to move
    *right_dist = get_distance();
    printf("Scan Right: %.2f cm\n", *right_dist);
    
    // 4. Return to Center
    servo_set_angle(ANGLE_CENTER);
    vTaskDelay(pdMS_TO_TICKS(100)); 

    // Dynamic Path Selection
    float left = *left_dist;
    float right = *right_dist;

    // A. If both sides are unsafe (too close to anything)
    if (left < MIN_ABSOLUTE_CLEARANCE_CM && right < MIN_ABSOLUTE_CLEARANCE_CM) {
        return 0; // Both blocked/Unsafe
    }

    // B. Check for a significant advantage
    // If Left is significantly clearer AND it's above the safety clearance
    if (left > right + PATH_ADVANTAGE_CM && left >= MIN_ABSOLUTE_CLEARANCE_CM) {
        return 1; // Left path is clear (Preferred)
    }

    // If Right is significantly clearer AND it's above the safety clearance
    if (right > left + PATH_ADVANTAGE_CM && right >= MIN_ABSOLUTE_CLEARANCE_CM) {
        return -1; // Right path is clear
    }

    if (left > right && left >= MIN_ABSOLUTE_CLEARANCE_CM) {
        return 1; // Left is slightly better
    }
    
    if (right > left && right >= MIN_ABSOLUTE_CLEARANCE_CM) {
        return -1; // Right is slightly better
    }

    // Fallback: If all else fails, or both are exactly equal and safe
    if (left >= MIN_ABSOLUTE_CLEARANCE_CM) {
        return 1; // Default to Left if safe
    }
    if (right >= MIN_ABSOLUTE_CLEARANCE_CM) {
        return -1; // Default to Right if safe
    }

    return 0;
}