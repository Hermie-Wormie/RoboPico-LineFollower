#include "queue.h"
#include "semphr.h"

// GPIO Pin Definitions
#define MOTOR1_PWM_PIN 15     // PWM pin for Motor 1 speed control (GP2)
#define MOTOR1_DIR_PIN1 14    // Direction control pin for Motor 1 (GP1, IN1)
#define MOTOR1_DIR_PIN2 13    // Direction control pin for Motor 1 (GP0, IN2)

#define MOTOR2_PWM_PIN 10     // PWM pin for Motor 2 speed control (GP5)
#define MOTOR2_DIR_PIN1 11    // Direction control pin for Motor 2 (GP4, IN3)
#define MOTOR2_DIR_PIN2 12   // Direction control pin for Motor 2 (GP3, IN4)

// PWM Definitions (Eliminate PWM Hum)
#define CLOCK_FREQUENCY 125000000.0f
#define FREQUENCY 100.0f

/* Motor Pinout
White   (EN_A)  > GP15
Grey    (IN_1)  > GP14
Purple  (IN_2)  > GP13
Blue    (IN_3)  > GP12
Green   (IN_4)  > GP11
Yellow  (EN_B)  > GP10
*/

// From encoder.c
extern volatile uint32_t pulse_width_L;
extern volatile uint32_t pulse_width_R;
void reset_encoder();

// From ultrasonic.c
extern SemaphoreHandle_t UltrasonicWarn_BinarySemaphore;

// From main.c
extern QueueHandle_t commands_queue;

// Set 1 to print PID values
#if 0
#define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...) // Nothing happens if DEBUG is 0
#endif