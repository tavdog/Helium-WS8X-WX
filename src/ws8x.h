#ifndef WS8x_H
#define WS8x_H
#include <LoRaWan-RAK4630.h>
void ws8x_init();
void ws8x_checkSerial();
void ws8x_populate_lora_buffer(lmh_app_data_t *m_lora_app_data, int size);
void ws8x_reset_counters();
extern unsigned long send_interval_ms; // main uses this

#endif