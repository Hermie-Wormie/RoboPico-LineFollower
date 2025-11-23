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
//void turn_away(void);

// Obstacle Detection
extern int servo_scan_path(float *left_dist, float *right_dist);
extern void servo_init(void);
extern void motor_finish_avoidance(void); 
void motor_start_avoidance(void); 
void avoidance_task(void *pvParameters);
void drive_fixed_distance_time(uint32_t duration_ms, bool cw1, bool cw2);
void avoidance_detour_maneuver(bool take_left_path);

// Barcode Turn Logic (NEW)
void motor_execute_barcode_turn(char command); // High-level function
void motor_pivot_turn(bool left_turn);
void motor_drive_forward_time(uint32_t duration_ms, int speed_pwm);

extern volatile bool IN_MANEUVER;
extern volatile bool SCANNING_BARCODE; // extern for the speed flag

extern QueueHandle_t turn_command_queue;

typedef enum {
    TURN_NONE = 0,
    TURN_LEFT_45,
    TURN_RIGHT_45
} TurnCommand_t;

extern volatile TurnCommand_t global_turn_command;
void motor_start_turn_45(TurnCommand_t command);

void execute_45_degree_turn(TurnCommand_t command);

#endif // MOTOR_H
