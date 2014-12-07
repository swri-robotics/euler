#ifndef __vetexcanopen_h__
#define __vetexcanopen_h__

#include <stdint.h>

void vetex_initialize(void);
void vetex_terminate(void);
void vetex_disable_movement(void);
void vetex_enable_movement(void);
void vetex_set_x_position(uint16_t ticks);
void vetex_set_y_position(uint16_t ticks);
void vetex_set_z_position(uint16_t ticks);
void vetex_set_x_percentage(int8_t percent);
void vetex_set_y_percentage(int8_t percent);
void vetex_set_z_percentage(int8_t percent);
void vetex_set_all_percentages(int8_t x_percent, int8_t y_percent, int8_t z_percent);

#endif

