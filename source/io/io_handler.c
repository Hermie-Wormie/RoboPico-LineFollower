#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>
#include "header.h"

// From infrared.c
extern volatile bool black_detected;

// Flags
bool BUZZER_INIT = false;
uint8_t CURRENT_MODE = 1;

// Temp for week 10
void swap_interrupts_for_station1(bool state);

void heartbeat_task(void *pvParameters) {
    while (1) {
        printf("Heartbeat OK\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Quickly read the state of a GPIO pin
bool get_connection_mode(void){

    gpio_init(MODE_SELECT_BUTTON);
    gpio_set_dir(MODE_SELECT_BUTTON, GPIO_IN);
    gpio_pull_up(MODE_SELECT_BUTTON);

    bool state = gpio_get(MODE_SELECT_BUTTON);
    return state;

}

// Auxilary Function
void buzzer(int count){

    if (!BUZZER_INIT){
        
        gpio_init(BUZZER_PIN);
        gpio_set_dir(BUZZER_PIN, GPIO_OUT);

        uint32_t divider = CLOCK_FREQUENCY / (FREQUENCY * 65536);

        gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
        pwm_set_clkdiv(slice, divider);
        pwm_set_wrap(slice, 65535);
        pwm_set_enabled(slice, true);

        BUZZER_INIT = true;

    }

    for (int i = 0; i < count; i++) {

        pwm_set_gpio_level(BUZZER_PIN, 65535 / 2);
        vTaskDelay(pdMS_TO_TICKS(200));
        pwm_set_gpio_level(BUZZER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(200));

    }
}
