#include "helpers.h"
#include <string.h>

void floatToBytes(float value, uint8_t *bytes) {

    union {
        float f;
        uint8_t b[4];
    } u;

    u.f = value;
    
    for (int i = 0; i < 4; i++) {
        bytes[i] = u.b[i];
    }
}

void bytesToFloat(uint8_t *bytes, float *value) {

    union {
        float f;
        uint8_t b[4];
    } u;

    memcpy(u.b, bytes, 4);

    *value = u.f;
}