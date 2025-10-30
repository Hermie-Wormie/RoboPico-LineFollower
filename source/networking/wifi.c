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
