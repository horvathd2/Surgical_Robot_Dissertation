/*
 * config.c
 *
 * Created: 28.01.2025 18:10:56
 *  Author: H.Dani
 */ 


#include "config.h"

volatile uint64_t timer3_overflow_count = 0;

ISR(TIMER3_OVF_vect){
	timer3_overflow_count++;
}

void set_output(volatile uint8_t *ddr, uint8_t pin) {
	*ddr |= (1 << pin);
}

void set_input(volatile uint8_t *ddr, uint8_t pin) {
	*ddr &= ~(1 << pin);
}

void pin_high(volatile uint8_t *port, uint8_t pin) {
	*port |= (1 << pin);
}

void pin_low(volatile uint8_t *port, uint8_t pin) {
	*port &= ~(1 << pin);
}

void setup_micros(void){
	cli();

	TCCR3B |= (1 << CS30);
	TIMSK3 |= (1 << TOIE3);

	sei();
}

uint32_t micros() {
	uint32_t overflows, timer_value;

	cli(); // Disable interrupts to read consistent values
	overflows = timer3_overflow_count;
	timer_value = TCNT3;
	sei(); // Enable interrupts

	// Calculate total microseconds
	return ((overflows * 65536UL) + timer_value) / (F_CPU / 1000000UL);
}