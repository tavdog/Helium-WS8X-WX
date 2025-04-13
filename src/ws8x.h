#ifndef WS8x_H
#define WS8x_H

#include <Arduino.h>

void ws8x_init();
void ws8x_checkSerial();
void ws8x_populate_lora_buffer(uint8_t* buffer, int size);
void ws8x_reset_counters();

#endif