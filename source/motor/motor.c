#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "semphr.h"
#include "motor.h"

// ------------------ GLOBAL STATE ------------------
float target_speed_motor1 = 0;  // Target speed percentage (0-100)
float target_speed_motor2 = 0;  // Target speed percentage (0-100)
bool clockwise_motor1 = true;
bool clockwise_motor2 = true;

bool DistanceWarning = false;
bool ReverseOverride = false;

// ------------------ PID PARAMETERS ------------------
float Kp_motor1 = 1.05;
float Ki_motor1 = 0.125;
float Kd_motor1 = 0.05;

float Kp_motor2 = 1.00;
float Ki_motor2 = 0.10;
float Kd_motor2 = 0.05;

bool APPLY_PID = false;

const float wheel_circumference = 0.3318;
const uint8_t pulses_per_revolution = 20;
const float max_speed_motor1 = 0.5;
const float max_speed_motor2 = 0.5;

const float motor_factor = max_speed_motor1 * 0.01f;
const float pulse_to_seconds = 1e-6f;

float integral_motor1 = 0, previous_error_motor1 = 0;
float integral_motor2 = 0, previous_error_motor2 = 0;

float integral_max = 100, integral_min = -100;

// ------------------ EXTERNALS ------------------
extern volatile uint32_t pulse_width_L;
extern volatile uint32_t pulse_width_R;
extern SemaphoreHandle_t UltrasonicWarn_BinarySemaphore;

// ------------------ BASIC HELPERS ------------------

void reset_PID() {
    integral_motor1 = 0;
    previous_error_motor1 = 0;
    integral_motor2 = 0;
    previous_error_motor2 = 0;
    reset_encoder();
}

float compute_actual_speed(uint32_t pulse_width) {
    if (pulse_width == 0) return 0;
    float pulse_interval_seconds = pulse_width * pulse_to_seconds;
    float time_per_revolution = pulse_interval_seconds * pulses_per_revolution;
    return wheel_circumference / time_per_revolution;  // m/s
}

float percent_to_speed(int percent) {
    return percent * motor_factor;
}

// ===================================================
// MOTOR CONTROL — UPDATED FOR ROBO PICO (GP8–GP11)
// ===================================================

void motor_init(void) {
    // Direction pins
    gpio_init(MOTOR1_DIR_PIN);
    gpio_init(MOTOR2_DIR_PIN);
    gpio_set_dir(MOTOR1_DIR_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR2_DIR_PIN, GPIO_OUT);

    // PWM pins
    gpio_set_function(MOTOR1_PWM_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR2_PWM_PIN, GPIO_FUNC_PWM);

    uint slice1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    uint slice2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);

    pwm_set_wrap(slice1, PWM_WRAP);
    pwm_set_wrap(slice2, PWM_WRAP);
    pwm_set_enabled(slice1, true);
    pwm_set_enabled(slice2, true);
}

// Set speed directly (percent)
void motor_set_speed(int left_speed, int right_speed) {
    // Clamp range
    left_speed = left_speed > 255 ? 255 : (left_speed < -255 ? -255 : left_speed);
    right_speed = right_speed > 255 ? 255 : (right_speed < -255 ? -255 : right_speed);

    gpio_put(MOTOR1_DIR_PIN, left_speed >= 0 ? 0 : 1);
    gpio_put(MOTOR2_DIR_PIN, right_speed >= 0 ? 0 : 1);

    pwm_set_gpio_level(MOTOR1_PWM_PIN, abs(left_speed));
    pwm_set_gpio_level(MOTOR2_PWM_PIN, abs(right_speed));
}

void motor_stop(void) {
    pwm_set_gpio_level(MOTOR1_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, 0);
}

// ===================================================
// PID + TASK LOGIC
// ===================================================

void motor_task(void *params) {
    const float dt = 0.025f;
    const float reciprocal_dt = 40.0f;

    while (1) {
        // Ultrasonic safety
        if (xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0) == pdTRUE) {
            DistanceWarning = true;
            printf("Distance Warning\n");
        }

        if (APPLY_PID && !DistanceWarning) {
            float actual_speed_motor1 = compute_actual_speed(pulse_width_L);
            float actual_speed_motor2 = compute_actual_speed(pulse_width_R);

            float target1 = percent_to_speed(target_speed_motor1);
            float target2 = percent_to_speed(target_speed_motor2);

            float error1 = target1 - actual_speed_motor1;
            float error2 = target2 - actual_speed_motor2;

            integral_motor1 += error1 * dt;
            integral_motor1 = fmaxf(fminf(integral_motor1, integral_max), integral_min);

            integral_motor2 += error2 * dt;
            integral_motor2 = fmaxf(fminf(integral_motor2, integral_max), integral_min);

            float derivative1 = (error1 - previous_error_motor1) * reciprocal_dt;
            float derivative2 = (error2 - previous_error_motor2) * reciprocal_dt;

            float output1 = Kp_motor1 * error1 + Ki_motor1 * integral_motor1 + Kd_motor1 * derivative1;
            float output2 = Kp_motor2 * error2 + Ki_motor2 * integral_motor2 + Kd_motor2 * derivative2;

            previous_error_motor1 = error1;
            previous_error_motor2 = error2;

            float duty1 = fmaxf(fminf((output1 / max_speed_motor1) * 255.0f, 255.0f), 0.0f);
            float duty2 = fmaxf(fminf((output2 / max_speed_motor2) * 255.0f, 255.0f), 0.0f);

            motor_set_speed((int)duty1, (int)duty2);
        } else {
            // Stop if obstacle detected (unless reversing)
            if (DistanceWarning && !ReverseOverride) {
                target_speed_motor1 = 0;
                target_speed_motor2 = 0;
            }

            // Apply manual control (non-PID)
            motor_set_speed(
                (int)((clockwise_motor1 ? 1 : -1) * (target_speed_motor1 * 2.55f)),
                (int)((clockwise_motor2 ? 1 : -1) * (target_speed_motor2 * 2.55f))
            );
        }

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}


// ===================================================
// DEBUG UTILITIES
// ===================================================
void encoder_debug_task(void *params) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    while (1) {
        float s1 = compute_actual_speed(pulse_width_L);
        float s2 = compute_actual_speed(pulse_width_R);
        printf("Speed L: %.2f  |  Speed R: %.2f\n", s1, s2);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void disable_warning() {
    DistanceWarning = false;
    printf("Obstacle Cleared\n");
}

// Used by IO handler for emergency stop
void reset_motor() {
    target_speed_motor1 = 0;
    target_speed_motor2 = 0;
    clockwise_motor1 = true;
    clockwise_motor2 = true;
    APPLY_PID = false;
    motor_stop();
    reset_PID();
}

// Used by line_following.c for direct PWM updates
void update_motor_fast(uint16_t speed_motor1, uint16_t speed_motor2) {
    // Convert percentage (0–100) to 8-bit PWM (0–255)
    if (speed_motor1 > 100) speed_motor1 = 100;
    if (speed_motor2 > 100) speed_motor2 = 100;

    uint16_t pwm1 = (speed_motor1 * 255) / 100;
    uint16_t pwm2 = (speed_motor2 * 255) / 100;

    pwm_set_gpio_level(MOTOR1_PWM_PIN, pwm1);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, pwm2);
}