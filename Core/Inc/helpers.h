#ifndef HELPERS_H
#define HELPERS_H

#include <stdint.h>

void floatToBytes(float value, uint8_t *bytes);
void bytesToFloat(uint8_t *bytes, float *value);

#endif