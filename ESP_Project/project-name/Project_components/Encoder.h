#ifndef ENCODER_H
#define ENCODER_H
#include <driver/gpio.h>
#include <stdint.h>

struct Encoder {
    int pin_A;
    int pin_B;
    int32_t counts;
    float CPR;
    float CPC;
    float circumference;
    float target_distance;
};
void encoder_init(struct Encoder* encoder, int pin_A, int pin_B, float CPR, float circumference);
void encoder_update(struct Encoder* encoder);
int32_t encoder_get_counts(struct Encoder* encoder);
void encoder_set_counts(struct Encoder* encoder, int32_t counts);
void encoder_reset(struct Encoder* encoder);
float encoder_get_distance(struct Encoder* encoder);
float encoder_get_error_distance(struct Encoder* encoder);
void encoder_attach_isr(struct Encoder* encoder, void (*isr_handler)(void));
#endif // ENCODER_H