#include "lwip/udp.h"

// encoder.c
#define LEFT_ENCODER 4  //Motor 2
#define RIGHT_ENCODER 6 //Motor 1
#define MINIMUM_DEBOUCE 40000
void encoder_callback_L(uint gpio, uint32_t events);
void encoder_callback_R(uint gpio, uint32_t events);
void encoder_callback_R_Station1(uint gpio, uint32_t events);

// ultrasonic.c
#define TRIG_PIN 16
#define ECHO_PIN 17
//#define MIN_DISTANCE 20
#define MIN_DISTANCE 20.0f
#define CALIBRATION_OFFSET 0
#define CALIBRATION_SCALING 1
#define TIMEOUT 100
void ultrasonic_callback(uint gpio, uint32_t events);
//void debug_ultrasonic_serial(){};

// servo.c
#define PWM_WRAP_50HZ 20000 // 125MHz / 50Hz / 125 (clock divider) = 20000
#define SERVO_PIN 15
#define CLOCK_DIV 125
static uint16_t angle_to_pulse(float angle);
void servo_set_angle(float angle);
void servo_init(void);
int servo_scan_path(float *left_dist, float *right_dist);

// infrared.c
#define IR_SENSOR 1
#define BARCODE_IR_SENSOR 7//28
void ir_callback(uint gpio, uint32_t events);
void ir_callback_barcode(uint gpio, uint32_t events);

// motor.c
void disable_warning();

// barcode.c
#define MIN_DEBOUNCE 100    // Minimum value to count as a valid interrupt. Might have to be tuned for indiviudal sensors

// From wifi.c
extern ip_addr_t remote_ip;
extern ip_addr_t telemetry_ip;
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port);

// Set 1 to print
#if 1
#define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...) // Nothing happens if DEBUG is 0
#endif