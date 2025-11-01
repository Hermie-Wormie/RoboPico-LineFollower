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
#include "hmc5883l.h"

// ==== Practical motor constants (tune per robot) ====
// motor1 = RIGHT
// motor2 = LEFT
#define MIN_PWM_RIGHT 36
#define MIN_PWM_LEFT 45
#define MAX_PWM 255

// Low-pass filter for measured speed (to calm jitter)
#define SPEED_ALPHA 0.6f // 0..1; higher = smoother but slower

static float TRIM_L = 1.00f;
static float TRIM_R = 1.00f;

// ---- Compass (HMC5883L) ----
static float target_heading_deg = 0.0f; // set once at start of a straight run

// helpers
static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline float wrap180(float a)
{ // wrap degrees to [-180,180]
    while (a > 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}

// Make sure we can call the implementation that lives in encoder.c
extern float compute_actual_speed(uint32_t pulse_width_us);

// ------------------ GLOBAL STATE ------------------
float target_speed_motor1 = 13; // % (RIGHT)
float target_speed_motor2 = 13; // % (LEFT)
bool clockwise_motor1 = true;
bool clockwise_motor2 = true;

bool DistanceWarning = false;
bool ReverseOverride = false;

// ------------------ PID PARAMETERS ------------------
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

float integral_motor1 = 0, previous_error_motor1 = 0; // reuse as prev filtered measurement (RIGHT)
float integral_motor2 = 0, previous_error_motor2 = 0; // reuse as prev filtered measurement (LEFT)

float integral_max = 100, integral_min = -100;

// filtered speeds for smoothing encoder jitter
static float vL_f = 0.0f; // LEFT measured m/s (filtered)
static float vR_f = 0.0f; // RIGHT measured m/s (filtered)

// ------------------ EXTERNALS ------------------
extern volatile uint32_t pulse_width_L; // LEFT encoder period us
extern volatile uint32_t pulse_width_R; // RIGHT encoder period us
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
// PUBLIC: call this once before a straight segment
// ===================================================
void motor_heading_lock_arm(void)
{
    hmc5883l_read_heading(&target_heading_deg);
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
// NOTE: motor1=RIGHT on MOTOR1_* pins, motor2=LEFT on MOTOR2_* pins
void motor_set_speed(int motor1_speed, int motor2_speed)
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
    const float dt = 0.025f;           // 25 ms loop
    const float reciprocal_dt = 40.0f; // 1/dt

    static bool boot_kick_done = false;

    while (1)
    {
        // ==== BOOT START KICK ==== (just once to wake encoders)
        if (!boot_kick_done)
        {
            motor_set_speed(MIN_PWM_RIGHT, MIN_PWM_LEFT);
            vTaskDelay(pdMS_TO_TICKS(150));
            boot_kick_done = true;
            // capture initial heading for straight hold
            hmc5883l_read_heading(&target_heading_deg);

            continue;
        }

        if (APPLY_PID && !DistanceWarning)
        {
            // 1) measure & smooth (LEFT uses L, RIGHT uses R)
            float vL = compute_actual_speed(pulse_width_L);
            float vR = compute_actual_speed(pulse_width_R);
            vL_f = SPEED_ALPHA * vL_f + (1.0f - SPEED_ALPHA) * vL;
            vR_f = SPEED_ALPHA * vR_f + (1.0f - SPEED_ALPHA) * vR;

            // 2) targets (with trims)
            float target_R = percent_to_speed((int)(target_speed_motor1 * TRIM_R)); // RIGHT
            float target_L = percent_to_speed((int)(target_speed_motor2 * TRIM_L)); // LEFT

            // 3) PID errors (RIGHT uses vR_f, LEFT uses vL_f)
            float error_R = target_R - vR_f;
            float error_L = target_L - vL_f;

            // 4) Integrators
            integral_motor1 += error_R * dt; // RIGHT
            integral_motor2 += error_L * dt; // LEFT
            integral_motor1 = clampf(integral_motor1, integral_min, integral_max);
            integral_motor2 = clampf(integral_motor2, integral_min, integral_max);

            // 5) Derivative (on measurement)
            float derivative_R = (previous_error_motor1 - vR_f) * reciprocal_dt;
            float derivative_L = (previous_error_motor2 - vL_f) * reciprocal_dt;
            previous_error_motor1 = vR_f;
            previous_error_motor2 = vL_f;

            // 6) PID outputs
            float pid_R = Kp_motor1 * error_R + Ki_motor1 * integral_motor1 + Kd_motor1 * derivative_R;
            float pid_L = Kp_motor2 * error_L + Ki_motor2 * integral_motor2 + Kd_motor2 * derivative_L;

            // 7) Feedforward (with warmup + slew limiting)
            float ff_R = (target_speed_motor1 * TRIM_R) * 2.55f;
            float ff_L = (target_speed_motor2 * TRIM_L) * 2.55f;

            static int warmup = 0;
            if (warmup < 15)
            { // ~0.35s
                ff_R = 0;
                ff_L = 0;
                warmup++;
            }
            static float ff_R_prev = 0.0f, ff_L_prev = 0.0f;
            const float FF_MAX_STEP = 4.0f; // PWM per cycle
            ff_R = clampf(ff_R, ff_R_prev - FF_MAX_STEP, ff_R_prev + FF_MAX_STEP);
            ff_L = clampf(ff_L, ff_L_prev - FF_MAX_STEP, ff_L_prev + FF_MAX_STEP);
            ff_R_prev = ff_R;
            ff_L_prev = ff_L;

            // 8) Duty compose
            float duty_R = ff_R + pid_R * (255.0f / max_speed_motor1);
            float duty_L = ff_L + pid_L * (255.0f / max_speed_motor2);

            // 9) Encoder balance (tiny)
            // If LEFT faster than RIGHT (vL_f > vR_f), dv > 0 → slow LEFT a bit, speed RIGHT a bit.
            float dv = (vL_f - vR_f);
            const float BALANCE_K = 50.0f; // PWM per (m/s) difference
            duty_R += (+1.0f) * dv * BALANCE_K * 1.0f;
            duty_L += (-1.0f) * dv * BALANCE_K * 1.0f;

            // 10) Compass heading correction (small)
            // Only when moving forward
            if (target_L > 0.0f && target_R > 0.0f)
            {
                float current_heading;
                hmc5883l_read_heading(&current_heading);

                float heading_error = wrap180(current_heading - target_heading_deg);
                const float HEADING_K = 0.05f; // tune 0.10–0.20
                const float DEAD_DEG = 12.0f;  // ignore tiny noise
                if (fabsf(heading_error) > DEAD_DEG)
                {
                    // Positive error = yawed right → slow RIGHT, boost LEFT
                    duty_R -= heading_error * HEADING_K;
                    duty_L += heading_error * HEADING_K;
                }
            }

            if (duty_R > 200)
                duty_R = 200;
            if (duty_L > 200)
                duty_L = 200;

            // 11) floors to overcome stiction
            if (target_R > 0.0f)
                duty_R = fmaxf(duty_R, (float)MIN_PWM_RIGHT);
            if (target_L > 0.0f)
                duty_L = fmaxf(duty_L, (float)MIN_PWM_LEFT);

            // 12) clamp & apply
            duty_R = clampf(duty_R, 0.0f, (float)MAX_PWM);
            duty_L = clampf(duty_L, 0.0f, (float)MAX_PWM);
            motor_set_speed((int)duty_R, (int)duty_L);
        }
        else
        {
            // Manual % control path (no PID)
            if (DistanceWarning && !ReverseOverride)
            {
                target_speed_motor1 = 0;
                target_speed_motor2 = 0;
            }
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
    if (speed_motor1 > 100)
        speed_motor1 = 100;
    if (speed_motor2 > 100)
        speed_motor2 = 100;

    uint16_t pwm1 = (speed_motor1 * 255) / 100;
    uint16_t pwm2 = (speed_motor2 * 255) / 100;

    pwm_set_gpio_level(MOTOR1_PWM_PIN, pwm1);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, pwm2);
}

// =================== COMPASS DEBUG TASK ======================
void compass_debug_task(void *p)
{
    while (1)
    {
        float heading;
        if (hmc5883l_read_heading(&heading))
            printf("Heading: %.1f deg\n", heading);
        else
            printf("HMC read FAIL\n");

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
