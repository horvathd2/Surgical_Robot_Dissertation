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

void ADC_init(void);

uint16_t ADC_read(uint8_t channel);

float read_current(uint8_t channel, uint8_t sample_size);

#endif /* SENSOR_H_ */