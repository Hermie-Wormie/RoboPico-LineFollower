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

// Variables to keep track of the UDP PCB and client IP addresses
static struct udp_pcb *udp_pcb = NULL;
uint16_t total_packets_received = 0;

ip_addr_t remote_ip;
ip_addr_t telemetry_ip;

// Function prototype for the UDP receive callback
void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port);

void start_udp(void) {
    // Create a new UDP PCB
    udp_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (udp_pcb == NULL) {
        printf("Failed to create UDP PCB\n");
        return;
    }

    // Bind the UDP PCB to the specified port
    if (udp_bind(udp_pcb, IP_ANY_TYPE, UDP_RECV_PORT) != ERR_OK) {
        printf("Failed to bind UDP PCB\n");
        udp_remove(udp_pcb);
        udp_pcb = NULL;
        return;
    }

    // Set the receive callback function
    udp_recv(udp_pcb, udp_receive_callback, NULL);

    printf("UDP initialized and listening on port %d\n", UDP_RECV_PORT);
}

// UDP receive callback function
void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port) {
    if (p == NULL) {
        return;
    }

    uint8_t buffer;
    pbuf_copy_partial(p, &buffer, 1, 0);  // Copy a single byte. look at all them bytes saved lol

    // Print the received message
    //DEBUG_PRINT("Received UDP packet from %s:%d\n", ipaddr_ntoa(addr), port);
    DEBUG_PRINT("Received UDP packet from %s:%d, data (hex): 0x%02X\n", ipaddr_ntoa(addr), port, buffer);
    total_packets_received += 1;
    
    // Add to queue
    if (xQueueSend(received_queue, &buffer, 0U) != pdPASS) {
        printf("Queue is full! Dumping!\n");
        xQueueReset(received_queue);
    } 

    // Free the pbuf
    pbuf_free(p);
}

// Function to send UDP packets to a specific client
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port) {
    if (udp_pcb == NULL) {
        printf("UDP PCB is not initialized\n");
        return;
    }

    if (client_ip->addr == IPADDR_ANY || client_port == 0) {
        printf("Invalid client IP or port\n");
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
    } else {
        //printf("UDP packet sent to %s:%d\n", ipaddr_ntoa(client_ip), client_port);
    }
    pbuf_free(p);
}


// Main function to start the UDP server
void start_UDP_server_ap(void *pvParameters) {

    flash(10, true); // Flash to show connection type

    // Wait for 3 seconds for setup (optional)
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Enable Access Point mode
    cyw43_arch_enable_ap_mode(AP_SSID, AP_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

    // Set Wi-Fi channel to reduce interference
    cyw43_wifi_ap_set_channel(&cyw43_state, WIFI_CHANNEL);
    printf("AP_channel: %d\n", cyw43_state.ap_channel);

    // Initialize UDP
    start_udp();

    // Set the remote IP for AP mode
    IP4_ADDR(&remote_ip,
            AP_REMOTE_IP_1,
            AP_REMOTE_IP_2,
            AP_REMOTE_IP_3,
            AP_REMOTE_IP_4);

    
    IP4_ADDR(&telemetry_ip,
             AP_TELE_IP_1,
             AP_TELE_IP_2,
             AP_TELE_IP_3,
             AP_TELE_IP_4);
    

    // Main loop
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // This is apparently a better alterative
        //vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void start_UDP_server_hotspot(void *pvParameters) {

    flash(10, false); // Flash to show connection type

    // Wait for 3 seconds for setup
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Enable Station mode
    cyw43_arch_enable_sta_mode();
    
    // Connect to a Wi-Fi network (replace ACCESS_POINT_NAME and PASSWORD)
    if (cyw43_arch_wifi_connect_timeout_ms(HOTSPOT_SSID, HOTSPOT_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Failed to connect to Wi-Fi\n");
        return;
    }
    printf("Connected to Wi-Fi network\n");

    // Wait for IP address assignment
    const ip4_addr_t *ip = &cyw43_state.netif[0].ip_addr;
    for (int i = 0; i < 10; i++) {
        if (!ip4_addr_isany_val(*ip)) {
            break;
        }
        cyw43_arch_poll();
        sleep_ms(500);
    }

    // Check if IP address is assigned
    if (!ip4_addr_isany_val(*ip)) {
        printf("IP Address assigned: %s\n", ip4addr_ntoa(ip));
    } else {
        printf("Failed to obtain IP address.\n");
    }

    // Initialize UDP
    start_udp();

    // Set the remote IP for Hotspot mode
    IP4_ADDR(&remote_ip,
            HOTSPOT_REMOTE_IP_1,
            HOTSPOT_REMOTE_IP_2,
            HOTSPOT_REMOTE_IP_3,
            HOTSPOT_REMOTE_IP_4);
    
    IP4_ADDR(&telemetry_ip,
             HOTSPOT_TELE_IP_1,
             HOTSPOT_TELE_IP_2,
             HOTSPOT_TELE_IP_3,
             HOTSPOT_TELE_IP_4);
    
   
    // Main loop
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // This is apparently a better alterative
        //vTaskDelay(pdMS_TO_TICKS(1000));
    }
}