#include "queue.h" //Fixes issue with importing hardware_pwm conflict

// Networking Stuff
#define WIFI_CHANNEL 6
#define UDP_RECV_PORT 2004

// ---- HOTSPOT ----
#define HOTSPOT_SSID "Pixel_8749"// "CY"
#define HOTSPOT_PASSWORD "fset3wpa3bic9p6" // "wifipassword"

#define HOTSPOT_REMOTE_IP_1 172
#define HOTSPOT_REMOTE_IP_2 20
#define HOTSPOT_REMOTE_IP_3 10
#define HOTSPOT_REMOTE_IP_4 2

#define HOTSPOT_TELE_IP_1 10//172
#define HOTSPOT_TELE_IP_2 34//20
#define HOTSPOT_TELE_IP_3 93//10
#define HOTSPOT_TELE_IP_4 78//4

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
#define MQTTSN_PUBLISH          0x0C
#define TELEMETRY_TOPIC_ID      0x0001 // Assuming this ID is used for the telemetry topic
#define MQTTSN_PORT             1883   // UDP_PORT from gateway.py
#endif

// ---- Queues ----
extern QueueHandle_t received_queue;
extern QueueHandle_t turn_command_queue;

// blink.c LED functions
void flash(int count, bool mode);
void send_udp_raw_packet(const uint8_t *data, size_t len, const ip_addr_t *client_ip, uint16_t client_port);
void send_mqttsn_publish(const char *payload, size_t payload_len);
void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);