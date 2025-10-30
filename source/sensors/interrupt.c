#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "sensors.h"

bool station1_flag = false;

void interrupt_dispatcher(uint gpio, uint32_t events) {
    
    // Organised in ascending pins, most likely to least likely
    switch (gpio) {
        case LEFT_ENCODER:
            encoder_callback_L(gpio, events);
            break;
        case RIGHT_ENCODER:
            if (station1_flag){
                encoder_callback_R_Station1(gpio, events);
            }else{
                encoder_callback_R(gpio, events);
            }
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

void setup_interrupts(){

    // ---------- Ultrasonic Stuff ---------- 
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    
    // Remember that echo needs to catch both rising and falling
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interrupt_dispatcher);

    // ----------  Wheel Encoder Stuff ---------- 
    gpio_init(LEFT_ENCODER);
    gpio_init(RIGHT_ENCODER);
    gpio_set_dir(LEFT_ENCODER, GPIO_IN);
    gpio_set_dir(RIGHT_ENCODER, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER);
    gpio_pull_up(RIGHT_ENCODER);

    // Set up interrupts to trigger only on the rising edge
    gpio_set_irq_enabled(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, true);

    // ---------- Infrared Sensor Stuff ---------- 
    gpio_init(IR_SENSOR);
    gpio_init(BARCODE_IR_SENSOR);
    gpio_set_pulls(BARCODE_IR_SENSOR, false, true); // Pull down
    gpio_set_dir(IR_SENSOR, GPIO_IN);
    gpio_set_dir(BARCODE_IR_SENSOR, GPIO_IN);

    // Set up interrupts to trigger on both edge
    gpio_set_irq_enabled(BARCODE_IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

}

void enable_encoder_interrupts(void){
    gpio_set_irq_enabled(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
    //gpio_set_irq_enabled(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
}

void disable_encoder_interrupts(void){
    gpio_set_irq_enabled(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, false);
    //gpio_set_irq_enabled(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, false);
}

void enable_IR_interrupts(void){
    gpio_set_irq_enabled(BARCODE_IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
}

void disable_IR_interrupts(void){
    gpio_set_irq_enabled(BARCODE_IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(IR_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void swap_interrupts_for_station1(bool state){
    station1_flag = state;
}