#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "queue.h"
#include "semphr.h"

// ===========================================================
// CYTRON ROBO PICO MOTOR PIN DEFINITIONS
// ===========================================================
// Motor 1 (Right Motor)
#define MOTOR1_PWM_PIN 8     // GP8  - PWM speed control
#define MOTOR1_DIR_PIN 9     // GP9  - Direction control

// Motor 2 (Left Motor)
#define MOTOR2_PWM_PIN 10    // GP10 - PWM speed control
#define MOTOR2_DIR_PIN 11    // GP11 - Direction control

// PWM SETTINGS
#define PWM_WRAP        255         // 8-bit resolution
#define MOTOR_MAX_SPEED 255         // Speed range: 0–255
#define MOTOR_MIN_SPEED 0

// ===========================================================
// EXTERNAL REFERENCES (FROM OTHER MODULES)
// ===========================================================

// From encoder.c
extern volatile uint32_t pulse_width_L;
extern volatile uint32_t pulse_width_R;
void reset_encoder(void);

extern float target_speed_motor1;
extern float target_speed_motor2;
extern bool  APPLY_PID;

// From ultrasonic.c
extern SemaphoreHandle_t UltrasonicWarn_BinarySemaphore;

// ===========================================================
// FROM motor.c (needed for PID + encoder debug)
// ===========================================================
float compute_actual_speed(uint32_t pulse_width_us);

// ===========================================================
// FUNCTION PROTOTYPES
// ===========================================================
void motor_init(void);
void motor_set_speed(int left_speed, int right_speed);
void motor_stop(void);
void disable_warning(void);
void compass_debug_task(void *p);

#endif // MOTOR_H
