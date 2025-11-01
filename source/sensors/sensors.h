#include "lwip/udp.h"

// encoder.c
#define LEFT_ENCODER 4
#define RIGHT_ENCODER 6
#define MINIMUM_DEBOUCE 40000
void encoder_callback_L(uint gpio, uint32_t events);
void encoder_callback_R(uint gpio, uint32_t events);
void encoder_callback_R_Station1(uint gpio, uint32_t events);

// ultrasonic.c
#define TRIG_PIN 16
#define ECHO_PIN 17
#define MIN_DISTANCE 20
#define CALIBRATION_OFFSET 0
#define CALIBRATION_SCALING 1
#define TIMEOUT 100
void ultrasonic_callback(uint gpio, uint32_t events);

// infrared.c
#define IR_SENSOR 1
#define BARCODE_IR_SENSOR 28
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