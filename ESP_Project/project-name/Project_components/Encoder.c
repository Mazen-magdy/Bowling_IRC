#include "./Encoder.h"
#include <driver/gpio.h>
void encoder_init(struct Encoder* encoder, long pin_A, long pin_B, float CPR, float circumference) {
    encoder->pin_A = pin_A;
    encoder->pin_B = pin_B;
    encoder->counts = 0;
    encoder->CPR = CPR;
    encoder->CPC = CPR / circumference; // Counts per centimeter
    encoder->circumference = circumference;
    encoder->target_distance = 0.0f;

    // Initialize GPIO pins
    gpio_set_direction(pin_A, GPIO_MODE_INPUT);
    gpio_set_direction(pin_B, GPIO_MODE_INPUT);
}
void encoder_update(struct Encoder* encoder) { // will be ISR for pin A change
    if(digitalRead(encoder->pin_A) == 0 &&  digitalRead(encoder->pin_B) == 1) {
        encoder->counts++;
    } 
    else if(digitalRead(encoder->pin_A) == 1 &&  digitalRead(encoder->pin_B) == 0) {
        encoder->counts--;
    }
}
long encoder_get_counts(struct Encoder* encoder) {
    return encoder->counts;
}
void encoder_set_counts(struct Encoder* encoder, long counts) {
    encoder->counts = counts;
}
void encoder_reset(struct Encoder* encoder) {
    encoder->counts = 0;
    encoder->target_distance = 0.0f;
}
float encoder_get_distance(struct Encoder* encoder) {
    return (float)encoder->counts / encoder->CPC;
}
float encoder_get_error_distance(struct Encoder* encoder) {
    return encoder->target_distance - encoder_get_distance(encoder);
}

void encoder_attach_isr(struct Encoder* encoder, void (*isr_handler)(void)) {
    gpio_set_intr_type(encoder->pin_A, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(encoder->pin_A, (gpio_isr_t)isr_handler, (void*) encoder);
}
