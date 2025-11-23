// Pico
#include <string.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
// Networking
#include "lwip/udp.h"
#include "lwip/netifapi.h"
// Our Headers
#include "wifi.h"

#define MQTTSN_PUBLISH 0x0C
#define TELEMETRY_TOPIC_ID 0x0001
#define GATEWAY_PORT 1883 // The UDP_PORT defined in gateway.py

// Global UDP control block
static struct udp_pcb *udp_pcb = NULL;

// Destination IP for telemetry
ip_addr_t telemetry_ip;

void start_udp(void) {
    udp_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (udp_pcb == NULL) {
        printf("Failed to create UDP PCB\n");
        return;
    }

    if (udp_bind(udp_pcb, IP_ANY_TYPE, UDP_RECV_PORT) != ERR_OK) {
        printf("Failed to bind UDP PCB\n");
        udp_remove(udp_pcb);
        udp_pcb = NULL;
        return;
    }

    udp_recv(udp_pcb, udp_recv_callback, NULL);
    printf("UDP initialized and listening on port %d (telemetry only)\n", UDP_RECV_PORT);
}

// Function to send UDP telemetry packets
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port) {
    if (udp_pcb == NULL) {
        printf("UDP PCB is not initialized\n");
        return;
    }

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(data), PBUF_RAM);
    if (p == NULL) {
        printf("Failed to allocate pbuf for UDP\n");
        return;
    }

    memcpy(p->payload, data, strlen(data));
    err_t err = udp_sendto(udp_pcb, p, client_ip, client_port);
    if (err != ERR_OK) {
        printf("Failed to send UDP packet: %d\n", err);
    }

    pbuf_free(p);
}

// Wi-Fi Access Point Mode
void start_UDP_server_ap(void *pvParameters) {
    flash(10, true);
    vTaskDelay(pdMS_TO_TICKS(3000));

    cyw43_arch_enable_ap_mode(AP_SSID, AP_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
    cyw43_wifi_ap_set_channel(&cyw43_state, WIFI_CHANNEL);
    printf("AP_channel: %d\n", cyw43_state.ap_channel);

    start_udp();

    IP4_ADDR(&telemetry_ip,
             AP_TELE_IP_1,
             AP_TELE_IP_2,
             AP_TELE_IP_3,
             AP_TELE_IP_4);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Wi-Fi Hotspot (Station) Mode
void start_UDP_server_hotspot(void *pvParameters) {
    flash(10, false);
    vTaskDelay(pdMS_TO_TICKS(3000));

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(HOTSPOT_SSID, HOTSPOT_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Failed to connect to Wi-Fi\n");
        return;
    }

    printf("Connected to Wi-Fi network\n");

    const ip4_addr_t *ip = &cyw43_state.netif[0].ip_addr;
    for (int i = 0; i < 10; i++) {
        if (!ip4_addr_isany_val(*ip)) break;
        cyw43_arch_poll();
        sleep_ms(500);
    }

    if (!ip4_addr_isany_val(*ip)) {
        printf("IP Address assigned: %s\n", ip4addr_ntoa(ip));
    } else {
        printf("Failed to obtain IP address.\n");
    }

    start_udp();

    IP4_ADDR(&telemetry_ip,
             HOTSPOT_TELE_IP_1,
             HOTSPOT_TELE_IP_2,
             HOTSPOT_TELE_IP_3,
             HOTSPOT_TELE_IP_4);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void send_mqttsn_publish(const char *payload, size_t payload_len) {
    // Total packet length: 7 bytes (Header + Fixed Fields) + Payload Length
    size_t packet_len = 7 + payload_len;

    // We use a local buffer to construct the packet
    // Max size of the buffer is defined by your largest possible packet
    uint8_t buffer[256]; 

    // 1. Length (1 byte)
    buffer[0] = (uint8_t)packet_len; 
    
    // 2. Message Type (1 byte): 0x0C for PUBLISH
    buffer[1] = MQTTSN_PUBLISH; 

    // 3. Flags (1 byte): QoS 0 (0x00). Use 0x01 if you were using Topic ID Type (not needed here)
    // The structure is: DUP | QoS | Retain | Will | CleanSession | TopicIdType
    // For QoS 0, Topic ID: Flags = 0b00000010 = 0x02
    buffer[2] = 0x04; // QoS 0, not retained, Topic Name ID type (short topic ID) 

    // 4. Topic ID (2 bytes): e.g., 0x0001 (high byte first)
    buffer[3] = (uint8_t)((TELEMETRY_TOPIC_ID >> 8) & 0xFF);
    buffer[4] = (uint8_t)(TELEMETRY_TOPIC_ID & 0xFF);

    // 5. Message ID (2 bytes): 0x0000 for QoS 0
    buffer[5] = 0x00;
    buffer[6] = 0x00;

    // 6. Data/Payload (N bytes)
    memcpy(&buffer[7], payload, payload_len);

    // --- ADD THIS DEBUG BLOCK ---
    printf("\n[TX] Final Packet Hex (Len: %d): ", packet_len);
    for (size_t i = 0; i < 10; i++) { // Print first 10 bytes of header + payload start
        printf("%02X ", buffer[i]);
    }
    printf("\n");
    // --- Send the UDP packet ---
    //send_udp_packet((const char*)buffer, &telemetry_ip, GATEWAY_PORT); 
    send_udp_raw_packet(buffer, packet_len, &telemetry_ip, GATEWAY_PORT);

}
void send_udp_raw_packet(const uint8_t *data, size_t len, const ip_addr_t *client_ip, uint16_t client_port) {
    if (udp_pcb == NULL || len == 0) return;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
    if (p == NULL) {
        printf("Failed to allocate pbuf for raw UDP packet\n");
        return;
    }

    // Copy the raw byte buffer into the pbuf payload
    memcpy(p->payload, data, len);

    // Send the data
    err_t err = udp_sendto(udp_pcb, p, client_ip, client_port);
    if (err != ERR_OK) {
        printf("UDP send failed with error %d\n", err);
    }
    
    // Free the pbuf
    pbuf_free(p);
}

void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        // We only process packets if they are coming from the expected GATEWAY PORT (1883)
        // and from the telemetry IP address.
        if (port == GATEWAY_PORT && ip_addr_cmp(addr, &telemetry_ip)) {
            
            // Your gateway is likely sending a *standard* MQTT-SN PUBLISH packet.
            // The format is: [Length, MsgType, Flags, TopicID_H, TopicID_L, MsgID_H, MsgID_L, PAYLOAD...]
            // The barcode command (the character) will be the first byte of the PAYLOAD (at index 7)

            if (p->len >= 8 && ((uint8_t*)p->payload)[1] == MQTTSN_PUBLISH) {
                // Assuming the command is the first character of the payload (at index 7)
                char command = ((char*)p->payload)[7]; 

                // Safely send the command to the FreeRTOS queue (must use FromISR variant!)
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                
                if (xQueueSendFromISR(turn_command_queue, &command, &xHigherPriorityTaskWoken) == pdPASS) {
                    printf("RX Barcode Command: %c\n", command);
                }
                
                // If a higher priority task was woken, yield from the ISR
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        
        // IMPORTANT: The pbuf must be freed after use to prevent memory leaks
        pbuf_free(p);
    }
}