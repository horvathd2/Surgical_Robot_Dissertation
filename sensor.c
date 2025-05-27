/*
 * sensor.c
 *
 * Created: 10.08.2024 15:24:42
 *  Author: H.Dani
 */ 

#include "sensor.h"

float		sample_voltage	= 0;
float		total_voltage	= 0;
float		avg_voltage		= 0;
float		amp				= 0;
uint16_t	adcvalue		= 0;

void setup_ext_sensors(void){
	cli();

	//CONFIGURE ENCODER A PINS AND SENSOR IN PINS AS INPUTS
	DDRD &= ~((1 << DDD0) | (1 << DDD1) | (1 << DDD2) | (1 << DDD3) | (1 << DDD6) | (1 << DDD7));
	DDRE &= ~((1 << DDE4) | (1 << DDE5) | (1 << DDE6) | (1 << DDE7));
	DDRB &= ~((1 << DDB4) | (1 << DDB5));

	//DISABLE PULL-UP RESISTORS
	PORTD &= ~((1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7));
	PORTB &= ~((1 << PB4) | (1 << PB5));

	//ENABLE PULL-UPS FOR EXTERNAL SENSORS
	PORTE |=  ((1 << PE4) | (1 << PE5) | (1 << PE6) | (1 << PE7));

	//SETUP EXTERNAL INTERRUPTS
	EICRA |= (1 << ISC00) | (1 << ISC01) | (1 << ISC10) | (1 << ISC11) | (1 << ISC20) | (1 << ISC21) | (1 << ISC30) | (1 << ISC31); // rising edge detect
	EICRB |= (1 << ISC40) | (1 << ISC50) | (1 << ISC60) | (1 << ISC70);																// any logical change
	EIMSK |= (1 << INT0)  | (1 << INT1)  | (1 << INT2)  | (1 << INT3)  | (1 << INT4)  | (1 << INT5)  | (1 << INT6)  | (1 << INT7);  // Enable interrupts

	sei();
}

void ADC_init(void){														//PF6 ADC6 / PF7 ADC7
	ADCSRA	|= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Enable the ADC
	ADMUX	|= (1 << REFS0);												// Use AVCC for reference
	ADMUX	&= ~(1 << REFS1);
	ADCSRB	&= ~(1 << MUX5);
}

uint16_t ADC_read(uint8_t channel){
	//MUX1 MUX2			- ACD6
	//MUX0 MUX1 MUX2	- ACD7
	if(channel == 6)														// Select sampling channel for reading
		ADMUX = (ADMUX & 0xF0) | 0x06;  // ADC6
	else if (channel == 7)
		ADMUX = (ADMUX & 0xF0) | 0x07;  // ADC7
	else return 0;
			
	ADCSRA |= (1 << ADSC);													// Start ADC conversion
	while (ADCSRA & (1 << ADSC));
	return ADC;																// Return converted result
}

float read_current(uint8_t channel, uint8_t sample_size){
	for (int i = 0; i < sample_size; i++) {
		adcvalue = ADC_read(channel);
		sample_voltage = (adcvalue * 5000.0) / 1024.0;
		total_voltage += sample_voltage;
	}
												// ADD HISTERESHYS!!!!!!!!!!!!!!!!
	avg_voltage = total_voltage/sample_size;
	amp = (avg_voltage - 2510) / 181;
	total_voltage = 0;
	return amp;
}