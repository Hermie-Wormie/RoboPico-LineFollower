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

// ==== Practical motor constants (tune per robot) ====
#define MIN_PWM_LEFT    38     // minimal PWM that actually moves LEFT wheel
#define MIN_PWM_RIGHT   42     // minimal PWM that actually moves RIGHT wheel
#define MAX_PWM         255

// Low-pass filter for measured speed (to calm jitter)
#define SPEED_ALPHA     0.6f   // 0..1; higher = smoother but slower

// Clamp helper
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Make sure we can call the implementation that lives in encoder.c
extern float compute_actual_speed(uint32_t pulse_width_us);

// ------------------ GLOBAL STATE ------------------
float target_speed_motor1 = 0; // Target speed percentage (0-100)
float target_speed_motor2 = 0; // Target speed percentage (0-100)
bool  clockwise_motor1     = true;
bool  clockwise_motor2     = true;

bool DistanceWarning = false;
bool ReverseOverride = false;

// ------------------ PID PARAMETERS ------------------
// Baseline gains to start tuning (more stable)
float Kp_motor1 = 0.60f;
float Ki_motor1 = 0.15f;
float Kd_motor1 = 0.00f;

float Kp_motor2 = 0.60f;
float Ki_motor2 = 0.15f;
float Kd_motor2 = 0.00f;

bool APPLY_PID = false;

// Map target % → m/s (max ~0.5 m/s; adjust if your robot can do more)
const float max_speed_motor1 = 0.5f;
const float max_speed_motor2 = 0.5f;
const float motor_factor     = 0.01f * max_speed_motor1; // percent_to_speed()

float integral_motor1 = 0, previous_error_motor1 = 0; // re-used as previous filtered measurement
float integral_motor2 = 0, previous_error_motor2 = 0;

float integral_max = 100, integral_min = -100;

// filtered speeds for smoothing encoder jitter
static float vL_f = 0.0f;
static float vR_f = 0.0f;

// ------------------ EXTERNALS ------------------
extern volatile uint32_t pulse_width_L;
extern volatile uint32_t pulse_width_R;
extern SemaphoreHandle_t UltrasonicWarn_BinarySemaphore;

// ------------------ BASIC HELPERS ------------------
static inline float percent_to_speed(int percent) {
    return percent * motor_factor;   // m/s
}

void reset_PID(void) {
    integral_motor1 = 0;
    integral_motor2 = 0;
    previous_error_motor1 = 0;
    previous_error_motor2 = 0;
    vL_f = vR_f = 0.0f;
    reset_encoder();
}

// ===================================================
// MOTOR CONTROL — ROBO PICO (GP8–GP11)
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

// Set speed directly using raw PWM (-255..255)
void motor_set_speed(int left_speed, int right_speed) {
    // Clamp range
    left_speed  = left_speed  >  255 ?  255 : (left_speed  < -255 ? -255 : left_speed);
    right_speed = right_speed >  255 ?  255 : (right_speed < -255 ? -255 : right_speed);

    gpio_put(MOTOR1_DIR_PIN, left_speed  >= 0 ? 0 : 1);
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
    printf("[MOTOR] Task started!\n");
    const float dt            = 0.025f;   // 25 ms loop
    const float reciprocal_dt = 40.0f;    // 1/dt

    while (1) {
        // // Optional ultrasonic safety
        // if (xSemaphoreTake(UltrasonicWarn_BinarySemaphore, 0) == pdTRUE) {
        //     DistanceWarning = true;
        //     printf("Distance Warning\n");
        // }

        if (APPLY_PID && !DistanceWarning) {
            // 1) Measure (and smooth) actual speeds
            float vL = compute_actual_speed(pulse_width_L);
            float vR = compute_actual_speed(pulse_width_R);
            vL_f = SPEED_ALPHA * vL_f + (1.0f - SPEED_ALPHA) * vL;
            vR_f = SPEED_ALPHA * vR_f + (1.0f - SPEED_ALPHA) * vR;

            // 2) Targets in m/s from percentage
            float target1 = percent_to_speed((int)target_speed_motor1);
            float target2 = percent_to_speed((int)target_speed_motor2);

            // 3) Errors
            float error1 = target1 - vL_f;
            float error2 = target2 - vR_f;

            // 4) Integrators (with anti-windup)
            integral_motor1 += error1 * dt;
            integral_motor2 += error2 * dt;
            integral_motor1 = clampf(integral_motor1, integral_min, integral_max);
            integral_motor2 = clampf(integral_motor2, integral_min, integral_max);

            // 5) Derivative on measurement (less noise)
            float derivative1 = (previous_error_motor1 - vL_f) * reciprocal_dt; // using previous filtered measurement
            float derivative2 = (previous_error_motor2 - vR_f) * reciprocal_dt;

            // 6) PID outputs in m/s-equivalent
            float pid1 = Kp_motor1 * error1 + Ki_motor1 * integral_motor1 + Kd_motor1 * derivative1;
            float pid2 = Kp_motor2 * error2 + Ki_motor2 * integral_motor2 + Kd_motor2 * derivative2;

            previous_error_motor1 = vL_f;
            previous_error_motor2 = vR_f;

            // 7) Map to duty (feedforward + PID correction)
            float ff1 = target_speed_motor1 * 2.55f;  // 0..255
            float ff2 = target_speed_motor2 * 2.55f;

            float mps_to_pwm1 = 255.0f / max_speed_motor1;
            float mps_to_pwm2 = 255.0f / max_speed_motor2;

            float duty1 = ff1 + pid1 * mps_to_pwm1;
            float duty2 = ff2 + pid2 * mps_to_pwm2;

            // 8) Apply min-PWM floors when target > 0 (overcome stiction)
            if (target1 > 0.0f) duty1 = fmaxf(duty1, (float)MIN_PWM_LEFT);
            if (target2 > 0.0f) duty2 = fmaxf(duty2, (float)MIN_PWM_RIGHT);

            // 9) Clamp and drive
            duty1 = clampf(duty1, 0.0f, (float)MAX_PWM);
            duty2 = clampf(duty2, 0.0f, (float)MAX_PWM);

            motor_set_speed((int)duty1, (int)duty2);
        } else {
            // Stop if obstacle detected (unless reversing)
            if (DistanceWarning && !ReverseOverride) {
                target_speed_motor1 = 0;
                target_speed_motor2 = 0;
            }

            // Apply manual control (non-PID) using % → PWM
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
void disable_warning(void) {
    DistanceWarning = false;
    printf("Obstacle Cleared\n");
}

void reset_motor(void) {
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
