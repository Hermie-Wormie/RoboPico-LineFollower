#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "sensors.h"

// Unified interrupt dispatcher for all sensors
void interrupt_dispatcher(uint gpio, uint32_t events) {
    switch (gpio) {
        case LEFT_ENCODER:
            encoder_callback_L(gpio, events);
            break;
        case RIGHT_ENCODER:
            encoder_callback_R(gpio, events);
            break;
        case BARCODE_IR_SENSOR:
            ir_callback_barcode(gpio, events);
            break;
        case IR_SENSOR:
            ir_callback(gpio, events);
            break;
        default:
            ultrasonic_callback(gpio, events);
            break;
    }
}

void setup_interrupts() {
    // ---------- Ultrasonic ----------
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interrupt_dispatcher);

    // ---------- Wheel Encoders ----------
    gpio_init(LEFT_ENCODER);
    gpio_init(RIGHT_ENCODER);
    gpio_set_dir(LEFT_ENCODER, GPIO_IN);
    gpio_set_dir(RIGHT_ENCODER, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER);
    gpio_pull_up(RIGHT_ENCODER);
    gpio_set_irq_enabled(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, true);

    // ---------- Infrared Sensors ----------
    gpio_init(IR_SENSOR);
    gpio_init(BARCODE_IR_SENSOR);
    gpio_set_pulls(BARCODE_IR_SENSOR, false, true); // pull-down
    gpio_set_dir(IR_SENSOR, GPIO_IN);
    gpio_set_dir(BARCODE_IR_SENSOR, GPIO_IN);
    gpio_set_irq_enabled(BARCODE_IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Encoder interrupt control
void enable_encoder_interrupts(void) {
    gpio_set_irq_enabled(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
}

void disable_encoder_interrupts(void) {
    gpio_set_irq_enabled(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, false);
}

// IR interrupt control
void enable_IR_interrupts(void) {
    gpio_set_irq_enabled(BARCODE_IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void disable_IR_interrupts(void) {
    gpio_set_irq_enabled(BARCODE_IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
}
