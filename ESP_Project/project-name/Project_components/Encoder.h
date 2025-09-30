#ifndef ENCODER_H
#define ENCODER_H

struct Encoder {
    long pin_A;
    long pin_B;
    long counts;
    float CPR;
    float CPC;
    float circumference;
    float target_distance;
};
void encoder_init(struct Encoder* encoder, long pin_A, long pin_B, float CPR, float circumference);
void encoder_update(struct Encoder* encoder);
long encoder_get_counts(struct Encoder* encoder);
void encoder_set_counts(struct Encoder* encoder, long counts);
void encoder_reset(struct Encoder* encoder);
float encoder_get_distance(struct Encoder* encoder);
float encoder_get_error_distance(struct Encoder* encoder);
#endif // ENCODER_H