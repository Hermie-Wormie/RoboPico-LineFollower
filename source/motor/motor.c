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
#include "hardware/i2c.h"

// ==== Practical motor constants (tune per robot) ====
// motor1 = RIGHT
// motor2 = LEFT
#define MIN_PWM_RIGHT 50
#define MIN_PWM_LEFT 50
#define MAX_PWM 255

// Low-pass filter for measured speed (to calm jitter)
#define SPEED_ALPHA 0.6f // 0..1; higher = smoother but slower

static float TRIM_L = 1.00f;
static float TRIM_R = 1.00f;

#define HMC5883L_ADDR 0x1E
#define HMC_I2C      i2c1

// ---- Compass (HMC5883L) ----
static float target_heading_deg = 0.0f; // set once at start of a straight run

// For obstacle avoiding
#define TURN_DURATION_MS 500
// for detouring
#define MAT_DISTANCE_CM 27.0f
#define AVOID_DRIVE_SPEED_PERCENT 15.0f
// Linear Speed = 0.20 * 0.5 m/s = 0.1 m/s = 10 cm/s
// Travel Time (1 Mat, 27 cm) = 27 cm / 10 cm/s = 2.7 s
#define ONE_MAT_TIME_MS 350

// Travel time (1.5 mats) 1.5 * 2700 ms = 4050 ms
// To modify based on how big space is (1.5 used at home)
#define LONG_MAT_TIME_MS 300

// Tuning constant for turn duration: must be tuned to achieve 90 degrees
#define TURN_90_DURATION_MS 350 // TUNE THIS VALUE FOR YOUR ROBOT (currently 900ms)
#define TURN_SPEED_PERCENT 38.0f // Lower speed for precision turns

#define PIVOT_45_DURATION_MS 350 // *
#define FORWARD_1S_DURATION_MS 1000
#define FORWARD_1S_SPEED_PWM 120 // *

extern QueueHandle_t turn_command_queue;
extern volatile bool IN_MANEUVER;

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
//static bool post_avoidance_turn = false; // Once car move past obstacle
static float avoidance_start_heading = 0.0f; // Store heading (IMU)
static bool is_avoidance_active = false; // State flag
static bool take_left_path = true; // Path decision

// ------------------ PID PARAMETERS ------------------
float Kp_motor1 = 0.60f;
float Ki_motor1 = 0.15f;
float Kd_motor1 = 0.00f;

float Kp_motor2 = 0.60f;
float Ki_motor2 = 0.15f;
float Kd_motor2 = 0.00f;

bool APPLY_PID = true;

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

extern int servo_scan_path(float *left_dist, float *right_dist);

extern TaskHandle_t LineFollowing_T;
extern TaskHandle_t Ultrasonic_T;
TaskHandle_t ObstacleAvoidance_T = NULL; 

float getDistance();

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
                const float HEADING_K = 2.0f; // tune 0.10–0.20
                const float DEAD_DEG = 1.0f;  // ignore tiny noise
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

            // // Convert target speed % to raw PWM level
            // int raw_pwm_R = (int)(target_speed_motor1 * 2.55f);
            // int raw_pwm_L = (int)(target_speed_motor2 * 2.55f);
            
            // // --- Apply Minimum PWM Floor ---
            // if (raw_pwm_R > 0)
            //     raw_pwm_R = fmaxf(raw_pwm_R, (float)MIN_PWM_RIGHT);
            // if (raw_pwm_L > 0)
            //     raw_pwm_L = fmaxf(raw_pwm_L, (float)MIN_PWM_LEFT);

            // // Apply direction and speed
            // motor_set_speed(
            //     (clockwise_motor1 ? raw_pwm_R : -raw_pwm_R),
            //     (clockwise_motor2 ? raw_pwm_L : -raw_pwm_L)
            // );


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

void disable_warning(void) {
    if (!is_avoidance_active) {
        // If not in avoidance state, resume PID line-following
        DistanceWarning = false;
        ReverseOverride = false;
        APPLY_PID = true;
        printf("Obstacle Cleared. Resuming Line Follow.\n");
        return;
    }

    // if in avoidance state, trigger rejoining line to start
    const float REJOIN_SPEED_PERCENT = 20.0f;
    const uint32_t REJOIN_DURATION_MS = 1000; // Time to perform the rejoin turn/drive

    printf("Obstacle cleared. Starting REJOIN maneuver...\n");

    target_speed_motor1 = REJOIN_SPEED_PERCENT; // Right Motor
    target_speed_motor2 = REJOIN_SPEED_PERCENT; // Left Motor

    if (take_left_path) {
        // Was driving LEFT side of obstacle. Need to turn LEFT to rejoin line.
        clockwise_motor1 = true;  // Right wheel FORWARD
        clockwise_motor2 = false; // Left wheel REVERSE
        printf("Performing LEFT turn to rejoin...\n");
    } else {
        // Was driving RIGHT side of obstacle. Need to turn RIGHT to rejoin line.
        clockwise_motor1 = false; // Right wheel REVERSE
        clockwise_motor2 = true;  // Left wheel FORWARD
        printf("Performing RIGHT turn to rejoin...\n");
    }
    
    ReverseOverride = true; // Keep manual control override
    
    // Perform the rejoin turn
    vTaskDelay(pdMS_TO_TICKS(REJOIN_DURATION_MS));
    
    motor_finish_avoidance();
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
    // if (pwm1 > 0) {
    //     pwm1 = fmaxf(pwm1, (float)MIN_PWM_RIGHT);
    // }
    // if (pwm2 > 0) {
    //     pwm2 = fmaxf(pwm2, (float)MIN_PWM_LEFT);
    // }

    pwm_set_gpio_level(MOTOR1_PWM_PIN, pwm1);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, pwm2);
}

// // =================== COMPASS DEBUG TASK ======================
void compass_debug_task(void *p)
{
    while (1)
    {
        float heading;
        if (hmc5883l_read_heading(&heading))
            //printf("Heading: %.1f deg\n", heading);
            printf("");
        else
            printf("HMC read FAIL\n");

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


// Obstacle Avidance Detour Maneuver
void avoidance_detour_maneuver(bool take_left_path) {

    ReverseOverride = true;

    target_speed_motor1 = TURN_SPEED_PERCENT;
    target_speed_motor2 = TURN_SPEED_PERCENT;

    printf("Step 1: Turning 90 degrees %s...\n", take_left_path ? "LEFT" : "RIGHT");

    // 1. Turn 90 deg to chosen path (left or right)
    if (take_left_path) {
        // Turn LEFT
        clockwise_motor1 = true;  // Right wheel FORWARD
        clockwise_motor2 = false; // Left wheel REVERSE
    } else {
        // Turn RIGHT
        clockwise_motor1 = false; // Right wheel REVERSE
        clockwise_motor2 = true;  // Left wheel FORWARD
    }

    vTaskDelay(pdMS_TO_TICKS(TURN_90_DURATION_MS));

    // 2. Travel 1 mat distance (measured 27cm)
    printf("Step 2: Traveling 1 mat distance (27cm)...\n");
    clockwise_motor1 = true;  // FORWARD
    clockwise_motor2 = true;  // FORWARD
    vTaskDelay(pdMS_TO_TICKS(ONE_MAT_TIME_MS));

    // 3. Turn back 90 deg to face forward
    printf("Step 3: Turning 90 degrees back to face forward...\n");
    if (take_left_path) {
        // Turn RIGHT (Back to original heading)
        clockwise_motor1 = false; // Right wheel REVERSE
        clockwise_motor2 = true;  // Left wheel FORWARD
    } else {
        // Turn LEFT (Back to original heading)
        clockwise_motor1 = true;  // Right wheel FORWARD
        clockwise_motor2 = false; // Left wheel REVERSE
    }
    vTaskDelay(pdMS_TO_TICKS(TURN_90_DURATION_MS));

    // 4. Travel 1.5 mat distance (40.5cm)
    printf("Step 4: Traveling 1.5 mat distance (%.1f cm)...\n", MAT_DISTANCE_CM * 1.5f);
    // *** DISTANCE MODIFICATION POINT: Change LONG_MAT_TIME_MS to adjust travel distance ***
    clockwise_motor1 = true;  // FORWARD
    clockwise_motor2 = true;  // FORWARD
    vTaskDelay(pdMS_TO_TICKS(LONG_MAT_TIME_MS));
    // *** END STEP 4 MODIFICATION POINT ***

    // 5. Turn 90 deg back opposite direction
    printf("Step 5: Turning 90 degrees opposite direction...\n");
    if (take_left_path) {
        // Was LEFT path, now turn RIGHT (to point back towards line)
        clockwise_motor1 = false; // Right wheel REVERSE
        clockwise_motor2 = true;  // Left wheel FORWARD
    } else {
        // Was RIGHT path, now turn LEFT (to point back towards line)
        clockwise_motor1 = true;  // Right wheel FORWARD
        clockwise_motor2 = false; // Left wheel REVERSE
    }
    vTaskDelay(pdMS_TO_TICKS(TURN_90_DURATION_MS));

    // 6. Travel back 1 mat distance
    printf("Step 6: Traveling 1 mat distance back...\n");
    clockwise_motor1 = true;  // FORWARD
    clockwise_motor2 = true;  // FORWARD
    vTaskDelay(pdMS_TO_TICKS(ONE_MAT_TIME_MS));

    // 7. Stop 
    printf("Step 7: Detour complete. Stopping...\n");
    motor_stop();
}

// Stops PID/LineFollower, executes a scan, and starts the dedicated avoidance task.
void motor_start_avoidance(void) {
    if (is_avoidance_active) return;
    is_avoidance_active = true;
    
    // Stop car
    APPLY_PID = false;
    target_speed_motor1 = 0;
    target_speed_motor2 = 0;
    motor_stop();

    printf("Obstacle detected. Stopping and starting scan...\n");

    // Capture current heading (IMU)
    // This heading is the line's direction.
    hmc5883l_read_heading(&avoidance_start_heading);
    printf("Line Heading: %.1f deg\n", avoidance_start_heading);

    vTaskSuspend(LineFollowing_T);
    //vTaskSuspend(Ultrasonic_T);

    xTaskCreate(avoidance_task, "AVOID_T", configMINIMAL_STACK_SIZE, 
                NULL, tskIDLE_PRIORITY + 2, // Give it a high priority
                &ObstacleAvoidance_T);
}

// Cleans up flags, resumes suspended tasks, and prepares to resume line following.
void motor_finish_avoidance(void)
{
    DistanceWarning = false;
    ReverseOverride = false;
    //post_avoidance_turn = false;
    is_avoidance_active = false;

    motor_stop(); 

    APPLY_PID = true;
    motor_heading_lock_arm();

    target_heading_deg = avoidance_start_heading;

    vTaskResume(LineFollowing_T);
    vTaskResume(Ultrasonic_T);
    
    printf("Avoidance maneuver complete. Resuming previous task.\n");
}

// Task that executes the obstacle avoidance logic (Scan -> Choose Path -> Detour).
void avoidance_task(void *pvParameters) {
    
    // --- 1. Scan Environment ---
    float left_dist, right_dist;
    int path_choice = servo_scan_path(&left_dist, &right_dist); // This now runs in its own context
    
    // --- 2. Choose Path  ---
    const float CLEAR_DISTANCE_CM = 30.0f;
    bool take_left = (path_choice == 1);
    
    if (path_choice == 0) {
        printf("Both sides blocked. Waiting...\n");
        // Clean up and notify motor task to resume previous state or standby
        // For now, just stop and delete the task.
        motor_stop();
        motor_finish_avoidance();
    
        vTaskDelete(NULL); 
    }
    
    printf("Path chosen: %s\n", take_left ? "LEFT" : "RIGHT");
    
    avoidance_detour_maneuver(take_left);
    
    // --- 6. Maneuver Complete ---
    motor_finish_avoidance(); // Resumes line follower, cleans up flags
    
    // Delete this temporary task
    vTaskDelete(NULL); 
}

//=================== COMPASS/IMU DEBUG TASK ======================
// void compass_debug_task(void *p)
// {
//     while (1)
//     {
//         // --- 1. Get Magnetometer Data (from hmc5883l logic) ---
//         uint8_t reg_mag = 0x03;
//         uint8_t raw_mag[6];
//         int16_t mx, my, mz;
//         if (i2c_write_blocking(HMC_I2C, HMC5883L_ADDR, &reg_mag, 1, true) >= 0 &&
//             i2c_read_blocking(HMC_I2C, HMC5883L_ADDR, raw_mag, 6, false) >= 0)
//         {
//             // Assuming the HMC5883L/LSM303 magnetomter output order (X, Z, Y)
//             mx = (raw_mag[0] << 8) | raw_mag[1];
//             mz = (raw_mag[2] << 8) | raw_mag[3];
//             my = (raw_mag[4] << 8) | raw_mag[5];
//         } else {
//             mx = my = mz = 0; // Read failed
//         }
//         int16_t ax, ay, az;
//         if (!imu_read_accel(&ax, &ay, &az))
//         {
//              ax = ay = az = 0; // Read failed
//         }    
//         // --- 3. Get Filtered Heading (Tilt-Compensated) ---
//         float heading;
//         hmc5883l_read_heading(&heading); // Use the function that returns the filtered value
//         // --- 4. Print Results ---
//         printf("IMU Debug | H: %.1f | ", heading);
//         printf("MAG: [X:%d Y:%d Z:%d] | ", mx, my, mz);
//         printf("ACCEL: [X:%d Y:%d Z:%d]\n", ax, ay, az);       
//         vTaskDelay(pdMS_TO_TICKS(100)); // Print 10 times per second
//     }
// }

// BARCODE MANEUVER FUNCTIONS

// Executes a fast pivot turn (pure rotation).
// left_turn True for left pivot, False for right pivot.
void motor_pivot_turn(bool left_turn) {
    printf("MANEUVER: Executing pivot turn: %s\n", left_turn ? "LEFT" : "RIGHT");
    int speed = TURN_90_DURATION_MS;

    if (left_turn) {
        // Left Turn: M1 (Right) FWD, M2 (Left) REV (or opposite signs for motor_set_speed)
        // motor_set_speed(Right Motor PWM, Left Motor PWM)
        motor_set_speed(speed, -speed); 
    } else {
        // Right Turn: M1 (Right) REV, M2 (Left) FWD
        motor_set_speed(-speed, speed); 
    }

    // Wait for the estimated rotation duration
    vTaskDelay(pdMS_TO_TICKS(PIVOT_45_DURATION_MS));
    motor_stop(); // Stop the pivot
    vTaskDelay(pdMS_TO_TICKS(50)); // Small brake time
}

// Drives motors straight forward for a set duration and speed.
void motor_drive_forward_time(uint32_t duration_ms, int speed_pwm) {
    printf("MANEUVER: Driving forward for %lu ms\n", duration_ms);
    
    // Drive both motors FWD
    motor_set_speed(speed_pwm, speed_pwm); 

    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motor_stop(); // Stop after driving
    vTaskDelay(pdMS_TO_TICKS(50)); // Small brake time
}

// Executes a full turn sequence based on the decoded barcode command.
// command 'L' for Left Turn, 'R' for Right Turn.
void motor_execute_barcode_turn(char command) {
    // 1. Set the global flag to pause line following
    IN_MANEUVER = true;
    printf("MANEUVER: Starting turn sequence for command '%c'.\n", command);

    // 2. Stop car (Ensure car is stopped before pivot)
    motor_stop();
    APPLY_PID = false; // Ensure PID doesn't fight manual control

    bool left_turn = (command == 'L');

    // 3. Rotate car 45 degrees left/right
    motor_pivot_turn(left_turn);

    // 4. Travel forward for 1 second
    motor_drive_forward_time(FORWARD_1S_DURATION_MS, FORWARD_1S_SPEED_PWM);

    // 5. Resume line following
    APPLY_PID = true; // Resume PID/Line Following control
    IN_MANEUVER = false;
    printf("MANEUVER: Turn sequence complete. Line following resumed.\n");
}