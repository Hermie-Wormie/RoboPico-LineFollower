#include "queue.h" //Fixes issue with importing hardware_pwm conflict

// Networking Stuff
#define WIFI_CHANNEL 11
#define UDP_RECV_PORT 2004

// ---- HOTSPOT ----
#define HOTSPOT_SSID "CY"
#define HOTSPOT_PASSWORD "wifipassword"

#define HOTSPOT_REMOTE_IP_1 172
#define HOTSPOT_REMOTE_IP_2 20
#define HOTSPOT_REMOTE_IP_3 10
#define HOTSPOT_REMOTE_IP_4 2

#define HOTSPOT_TELE_IP_1 172
#define HOTSPOT_TELE_IP_2 20
#define HOTSPOT_TELE_IP_3 10
#define HOTSPOT_TELE_IP_4 4

// ---- DIRECT CONNECTION (AP) ----
#define AP_SSID "PicoW-P3A"
#define AP_PASSWORD "12345678"

#define AP_REMOTE_IP_1 192
#define AP_REMOTE_IP_2 168
#define AP_REMOTE_IP_3 4
#define AP_REMOTE_IP_4 2

#define AP_TELE_IP_1 192
#define AP_TELE_IP_2 168
#define AP_TELE_IP_3 4
#define AP_TELE_IP_4 3

// Enables debug print statements
#define DEBUG 0
#if DEBUG
#define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...) // Nothing happens if DEBUG is 0
#endif

// ---- Queues ----
extern QueueHandle_t received_queue;

// blink.c LED functions
void flash(int count, bool mode);