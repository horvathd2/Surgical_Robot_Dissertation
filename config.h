/*
 * config.h
 *
 * Created: 7/22/2024 1:54:02 PM
 *  Author: H.Daniel
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_


#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void set_output(volatile uint8_t *ddr, uint8_t pin);
void set_input(volatile uint8_t *ddr, uint8_t pin);

void pin_high(volatile uint8_t *port, uint8_t pin);
void pin_low(volatile uint8_t *port, uint8_t pin);

uint32_t micros();

#endif /* CONFIG_H_ */