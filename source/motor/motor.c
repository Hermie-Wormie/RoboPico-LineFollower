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
// motor1 = RIGHT
// motor2 = LEFT
#define MIN_PWM_RIGHT 36 // RIGHT wheel minimum
#define MIN_PWM_LEFT 45  // LEFT wheel minimum (needs more torque)
#define MAX_PWM 255

// Low-pass filter for measured speed (to calm jitter)
#define SPEED_ALPHA 0.6f // 0..1; higher = smoother but slower

static float TRIM_L = 1.00f; // left gets 60% of commanded percent
static float TRIM_R = 1.00f; // right stays full

// Clamp helper
static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

// Make sure we can call the implementation that lives in encoder.c
extern float compute_actual_speed(uint32_t pulse_width_us);

// ------------------ GLOBAL STATE ------------------
float target_speed_motor1 = 13; // Target speed percentage (0-100)
float target_speed_motor2 = 13; // Target speed percentage (0-100)
bool clockwise_motor1 = true;
bool clockwise_motor2 = true;

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
const float motor_factor = 0.01f * max_speed_motor1; // percent_to_speed()

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
static inline float percent_to_speed(int percent)
{
    return percent * motor_factor; // m/s
}

void reset_PID(void)
{
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
void motor_init(void)
{
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
void motor_set_speed(int motor1_speed, int motor2_speed) // motor1=RIGHT, motor2=LEFT
{
    motor1_speed = motor1_speed > 255 ? 255 : (motor1_speed < -255 ? -255 : motor1_speed);
    motor2_speed = motor2_speed > 255 ? 255 : (motor2_speed < -255 ? -255 : motor2_speed);

    // Motor1 (RIGHT)
    gpio_put(MOTOR1_DIR_PIN, motor1_speed >= 0 ? 0 : 1);
    pwm_set_gpio_level(MOTOR1_PWM_PIN, abs(motor1_speed));

    // Motor2 (LEFT)
    gpio_put(MOTOR2_DIR_PIN, motor2_speed >= 0 ? 0 : 1);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, abs(motor2_speed));
}

void motor_stop(void)
{
    pwm_set_gpio_level(MOTOR1_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, 0);
}

// ===================================================
// PID + TASK LOGIC
// ===================================================
void motor_task(void *params)
{
    printf("[MOTOR] Task started!\n");
    const float dt = 0.025f;
    const float reciprocal_dt = 40.0f;

    static bool boot_kick_done = false;

    while (1)
    {
        // ==== BOOT START KICK ==== (just once)
        if (!boot_kick_done)
        {
            // kick both wheels at min pwm to generate encoder pulses
            motor_set_speed(MIN_PWM_RIGHT, MIN_PWM_LEFT);
            vTaskDelay(pdMS_TO_TICKS(150)); // ~0.15s push
            boot_kick_done = true;
            // now continue to next loop, PID will start next cycle
            continue;
        }

        if (APPLY_PID && !DistanceWarning)
        {
            // 1) speed measurement
            float vL = compute_actual_speed(pulse_width_L); // LEFT
            float vR = compute_actual_speed(pulse_width_R); // RIGHT
            vL_f = SPEED_ALPHA * vL_f + (1.0f - SPEED_ALPHA) * vL;
            vR_f = SPEED_ALPHA * vR_f + (1.0f - SPEED_ALPHA) * vR;

            // 2) targets (with trim)
            float target_R = percent_to_speed((int)(target_speed_motor1 * TRIM_R)); // motor1 = RIGHT
            float target_L = percent_to_speed((int)(target_speed_motor2 * TRIM_L)); // motor2 = LEFT

            // 3) error
            float error_R = target_R - vR_f;
            float error_L = target_L - vL_f;

            // 4) integrator
            integral_motor1 += error_R * dt;
            integral_motor2 += error_L * dt;
            integral_motor1 = clampf(integral_motor1, integral_min, integral_max);
            integral_motor2 = clampf(integral_motor2, integral_min, integral_max);

            // 5) derivative on measurement
            float derivative_R = (previous_error_motor1 - vR_f) * reciprocal_dt;
            float derivative_L = (previous_error_motor2 - vL_f) * reciprocal_dt;
            previous_error_motor1 = vR_f;
            previous_error_motor2 = vL_f;

            // 6) PID outputs
            float pid_R = Kp_motor1 * error_R + Ki_motor1 * integral_motor1 + Kd_motor1 * derivative_R;
            float pid_L = Kp_motor2 * error_L + Ki_motor2 * integral_motor2 + Kd_motor2 * derivative_L;

            float ff_R = (target_speed_motor1 * TRIM_R) * 2.55f;
            float ff_L = (target_speed_motor2 * TRIM_L) * 2.55f;

            static int warmup = 0;
            if (warmup < 15)
            { // suppress feedforward for ~0.35s
                ff_R = 0;
                ff_L = 0;
                warmup++;
            }

            // smooth FF return (slew limit)
            static float ff_R_prev = 0.0f;
            static float ff_L_prev = 0.0f;
            const float FF_MAX_STEP = 4.0f; // max PWM increase per cycle (~4*40Hz = 160 PWM/s)

            ff_R = clampf(ff_R, ff_R_prev - FF_MAX_STEP, ff_R_prev + FF_MAX_STEP);
            ff_L = clampf(ff_L, ff_L_prev - FF_MAX_STEP, ff_L_prev + FF_MAX_STEP);

            ff_R_prev = ff_R;
            ff_L_prev = ff_L;

            float duty_R = ff_R + pid_R * (255.0f / max_speed_motor1);
            float duty_L = ff_L + pid_L * (255.0f / max_speed_motor2);

            // balance correction small
            float dv = (vL_f - vR_f);
            duty_R += dv * 50.0f * -1;
            duty_L += dv * 50.0f * +1;

            // floor
            if (target_R > 0.0f)
                duty_R = fmaxf(duty_R, MIN_PWM_RIGHT);
            if (target_L > 0.0f)
                duty_L = fmaxf(duty_L, MIN_PWM_LEFT);

            // clamp
            duty_R = clampf(duty_R, 0.0f, 255.0f);
            duty_L = clampf(duty_L, 0.0f, 255.0f);

            // apply
            motor_set_speed((int)duty_R, (int)duty_L);
        }
        else
        {
            motor_set_speed(
                (int)((clockwise_motor1 ? 1 : -1) * (target_speed_motor1 * 2.55f)),
                (int)((clockwise_motor2 ? 1 : -1) * (target_speed_motor2 * 2.55f)));
        }

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

// ===================================================
// DEBUG UTILITIES
// ===================================================
void disable_warning(void)
{
    DistanceWarning = false;
    printf("Obstacle Cleared\n");
}

void reset_motor(void)
{
    target_speed_motor1 = 0;
    target_speed_motor2 = 0;
    clockwise_motor1 = true;
    clockwise_motor2 = true;
    APPLY_PID = false;
    motor_stop();
    reset_PID();
}

// Used by line_following.c for direct PWM updates
void update_motor_fast(uint16_t speed_motor1, uint16_t speed_motor2)
{
    // Convert percentage (0–100) to 8-bit PWM (0–255)
    if (speed_motor1 > 100)
        speed_motor1 = 100;
    if (speed_motor2 > 100)
        speed_motor2 = 100;

    uint16_t pwm1 = (speed_motor1 * 255) / 100;
    uint16_t pwm2 = (speed_motor2 * 255) / 100;

    pwm_set_gpio_level(MOTOR1_PWM_PIN, pwm1);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, pwm2);
}
