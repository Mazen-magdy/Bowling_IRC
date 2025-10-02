#include "./Encoder.h"
#include "driver/gpio.h"

void encoder_init(struct Encoder* encoder, int pin_A, int pin_B, float CPR, float circumference) {
    encoder->pin_A = pin_A;
    encoder->pin_B = pin_B;
    encoder->counts = 0;
    encoder->CPR = CPR;
    encoder->CPC = CPR / circumference; // Counts per centimeter
    encoder->circumference = circumference;
    encoder->target_distance = 0.0f;

    // Initialize GPIO pins with pull-up resistors
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pin_A) | (1ULL << pin_B),
        .pull_down_en = 0,
        .pull_up_en = 1,  // Enable pull-up resistors
    };
    gpio_config(&io_conf);
}
void encoder_update(struct Encoder* encoder) { // will be ISR for pin A change
    if(gpio_get_level(encoder->pin_A) == 0 &&  gpio_get_level(encoder->pin_B) == 1) {
        encoder->counts++;
    } 
    else if(gpio_get_level(encoder->pin_A) == 1 &&  gpio_get_level(encoder->pin_B) == 0) {
        encoder->counts--;
    }   
}
int32_t encoder_get_counts(struct Encoder* encoder) {
    return encoder->counts;
}
void encoder_set_counts(struct Encoder* encoder, int32_t counts) {
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
    // Configure pin A for interrupt
    gpio_set_intr_type(encoder->pin_A, GPIO_INTR_ANYEDGE);
    
    // Install ISR service only once
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        isr_service_installed = true;
    }
    
    // Add ISR handler for pin A
    gpio_isr_handler_add(encoder->pin_A, (gpio_isr_t)isr_handler, (void*) encoder);
}
