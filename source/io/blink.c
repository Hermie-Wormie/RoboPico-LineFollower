#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"

#define BLINK_DELAY 1000
volatile bool UDP_FLAG = false;

// Simple Task to constantly blink the built in LED
void blink(void *pvParameters)
{   
    while(1){
        while (!UDP_FLAG) {
            cyw43_arch_gpio_put(0, 1);
            vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY));
            cyw43_arch_gpio_put(0, 0);
            vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY));
        }
    }
}

// Additional sample task to blink a GPIO LED for testing SMP
void GPIO_blink(void *param) {
    int GPIO_PIN = *((int *)param);

    gpio_init(GPIO_PIN);
    gpio_set_dir(GPIO_PIN, GPIO_OUT);

    while (true) {
        gpio_put(GPIO_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY));
        gpio_put(GPIO_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY));
    }
}

void led_on()
{   //UDP_FLAG = true;
    cyw43_arch_gpio_put(0, 1);
}

void led_off()
{   
    cyw43_arch_gpio_put(0, 0);
    //UDP_FLAG = false;
}

// Simple flashing program to denote which connection mode is active
void flash(int count, bool mode) {

    if (mode){ // Constant blinking
        for (int i = 0; i < count; i++) {
            cyw43_arch_gpio_put(0, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            cyw43_arch_gpio_put(0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } else{ // Flashes twice quickly
        for (int i = 0; i < 3; i++) {
            cyw43_arch_gpio_put(0, 1);
            vTaskDelay(pdMS_TO_TICKS(70));
            cyw43_arch_gpio_put(0, 0);
            vTaskDelay(pdMS_TO_TICKS(70));
            cyw43_arch_gpio_put(0, 1);
            vTaskDelay(pdMS_TO_TICKS(70));
            cyw43_arch_gpio_put(0, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

}