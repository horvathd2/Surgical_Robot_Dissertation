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

void setup_micros(void);
uint32_t micros();

#endif /* CONFIG_H_ */

// SETUP DIRECTION & PWM PINS FOR BASE MOTORS
//DDRH |= (1 << DDH3); //B2 OC4A MOTOR2
//DDRH |= (1 << DDH4); //B1 OC4B MOTOR2

//DDRH |= (1 << DDH5); //A2 OC4C MOTOR1
//DDRH |= (1 << DDH6); //A1 OC2B MOTOR1

//MICRO MOTOR PWM PINS
//DDRB |= (1 << DDB6); //PWMB OC1B MOTOR3
//DDRB |= (1 << DDB7); //PWMA OC0A MOTOR4

//MICRO MOTOR DIRECTION PINS
//DDRL |= (1 << DDL0); //A1 MOTOR4
//DDRL |= (1 << DDL1); //A2 MOTOR4
//DDRL |= (1 << DDL2); //B1 MOTOR3
//DDRL |= (1 << DDL3); //B2 MOTOR3

//BASE MOTOR 1
//OCR4C = 0;		//A2 NEGATIVE
//OCR2B = 200;		//A1 POSITIVE

//BASE MOTOR 2
//OCR4A = 0;		//B2 NEGATIVE
//OCR4B = 200;		//B1 POSITIVE