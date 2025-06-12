/*
 * sensor.h
 *
 * Created: 10.08.2024 15:29:08
 *  Author: H.Dani
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

#include "config.h"

void setup_ext_sensors(void);

/*
void update_encoder(volatile uint8_t* last_state, volatile int32_t* position,
					volatile uint8_t* port_a, uint8_t pin_a,
					volatile uint8_t* port_b, uint8_t pin_b);*/

void ADC_init(void);

uint16_t ADC_read(uint8_t channel);

float read_current(uint8_t channel, uint8_t sample_size);

#endif /* SENSOR_H_ */