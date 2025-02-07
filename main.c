/*
 * TEST_ATMEL.c
 *
 * Created: 6/28/2024 10:40:55 AM
 * Author : H.Daniel
 */

#include "config.h"
#include "uart.h"
#include "motor.h"

volatile int32_t motor1_currentPos = 0;
volatile int32_t motor2_currentPos = 0;
volatile int32_t motor3_currentPos = 0;
volatile int32_t motor4_currentPos = 0;

volatile char read[100];
volatile uint8_t rx_index = 0;         // Buffer position
volatile uint8_t data_ready = 0;       // Flag: 1 when a full string is received

char tobesent[100];
int32_t setpoint = 3500;
uint8_t connected = 0;

//-------- ENCODERS ---------
ISR(INT0_vect){ // A2 MOTOR 2 (PH3 PH4) - DIRECTION & PWM PINS
	if (PINB & (1 << PB4)) {
		motor2_currentPos++;  // If B is high while A changes, increment ticks
	} else {
		motor2_currentPos--;  // If B is low while A changes, decrement ticks
	}
}

ISR(INT1_vect){ // A1 MOTOR 1 (PH5 PH6) - DIRECTION & PWM PINS
	if (PINB & (1 << PB5)) {
		motor1_currentPos++;  // If B is high while A changes, increment ticks
	} else {
		motor1_currentPos--;  // If B is low while A changes, decrement ticks
	}
}

ISR(INT2_vect){ // A4 MOTOR 4 (PL0 PL1) - DIRECTION & PB6 - PWM
	if (PIND & (1 << PD6)) {
		motor4_currentPos--;  // If B is high while A changes, increment ticks
	} else {
		motor4_currentPos++;  // If B is low while A changes, decrement ticks
	}
}

ISR(INT3_vect){ // A3 MOTOR 3 (PL2 PL3) - DIRECTION & PB7 - PWM
	if (PIND & (1 << PD7)) {
		motor3_currentPos++;  // If B is high while A changes, increment ticks
	} else {
		motor3_currentPos--;  // If B is low while A changes, decrement ticks
	}
}

//-------- SENSORS ---------
ISR(INT4_vect){ // SENSOR 4
	if (PINE & (1 << PINE4)) {
		PORTA |= (1 << PA1);
	} else {
		PORTA &= ~(1 << PA1);
	}
}

ISR(INT5_vect){ // SENSOR 3
	if (PINE & (1 << PINE5)) {
		PORTA |= (1 << PA1);
	} else {
		PORTA &= ~(1 << PA1);
	}
}
 
ISR(INT6_vect){ // SENSOR 2
	if (PINE & (1 << PINE6)) {
		PORTA |= (1 << PA1);
	} else {
		PORTA &= ~(1 << PA1);
	}
}

ISR(INT7_vect){ // SENSOR 1
	if (PINE & (1 << PINE7)) {
		PORTA |= (1 << PA1);
	} else {
		PORTA &= ~(1 << PA1);
	}
}

// Interrupt Service Routine (ISR) for USART Receive Complete
ISR(USART0_RX_vect) {
	char c = UDR0;  // Read received character

	// If Enter key is received, mark the string as complete
	if (c == '\n' || c == '\r') {
		read[rx_index] = '\0';  // Null-terminate the string
		data_ready = 1;  // Set flag indicating a complete string is available
		rx_index = 0;    // Reset buffer index for the next message
	}
	else if (rx_index < 100 - 1) {
		read[rx_index++] = c;  // Store character in buffer
	}
}

int main(void)
{
	//CONFIGURE ENCODER A PINS AND SENSOR IN PINS AS INPUTS
	DDRD &= ~((1 << DDD2) | (1 << DDD3) | (1 << DDD6) | (1 << DDD7));
	DDRE &= ~((1 << DDE4) | (1 << DDE5) | (1 << DDE6) | (1 << DDE7));
	DDRB &= ~((1 << DDB4) | (1 << DDB5));

	//DISABLE PULL-UP RESISTORS
	PORTD &= ~((1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7));
	PORTE &= ~((1 << PE4) | (1 << PE5) | (1 << PE6) | (1 << PE7));
	PORTB &= ~((1 << PB4) | (1 << PB5));

	//SETUP EXTERNAL INTERRUPTS
	cli();

	EICRA |= (1 << ISC00) | (1 << ISC01) | (1 << ISC10) | (1 << ISC11) | (1 << ISC20) | (1 << ISC21) | (1 << ISC30) | (1 << ISC31); //rising edge detect
	EICRB |= (1 << ISC40) | (1 << ISC50) | (1 << ISC60) | (1 << ISC70); // any logical change for induction sensor external interrupts
	EIMSK |= (1 << INT2)  | (1 << INT3)  | (1 << INT4);  // Enable interrupts

	//INITIALIZE SERIAL COMMUNICATION
	USART0_init();

	//Motor basemotor1 = init_motor(2, PORTH, OCR4C, OCR2B, DDRH, DDH5, DDH6, motor1_currentPos, &DDRH, DDH5, &OCR2B, 255);
	//init_pid(&basemotor1, 1.5, 0, 0);
	//Motor basemotor2 = init_motor(2, PORTH, OCR4A, OCR4B, DDRH, DDH3, DDH4, motor2_currentPos, &DDRH, DDH3, &OCR4B, 255);
	//init_pid(&basemotor2, 1.5, 0, 0);

	Motor micromotor3 = init_motor(1, &PORTL, PL2, PL3, &DDRL, DDL2, DDL3, motor3_currentPos, &DDRB, DDB6, &OCR1B, 255);
	init_pid(&micromotor3, 1, 0.05, 0.0);
	Motor micromotor4 = init_motor(1, &PORTL, PL0, PL1, &DDRL, DDL0, DDL1, motor4_currentPos, &DDRB, DDB7, &OCR0A, 255);
	init_pid(&micromotor4, 1.2, 0, 0);

	//SET THE TIMER COUNTER CONTROL REGISTERS TO FAST PWM MODE WITH CLEAR ON COMPARE MATCH
	//MICRO MOTOR PWM SETUP
	TCCR0A |= (1 << COM0A1) | (1 << WGM01)  | (1 << WGM00);
	TCCR0B |=  (1 << CS00)  | (1 << WGM02);
	
	TCCR1A |= (1 << COM1B1) | (1 << WGM10); 
	TCCR1B |=  (1 << CS10)  | (1 << WGM12);

	//BASE MOTOR PWM SETUP
	TCCR2A |= (1 << COM2B1) | (1 << WGM21)  | (1 << WGM20);
	TCCR2B |=  (1 << CS20)  | (1 << WGM22);

	TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM40); 
	TCCR4B |=  (1 << CS40)  | (1 << WGM42);

	//SETUP TIMER 3 FOR MICROSECOND COUNTING FROM BOOT
	TCCR3B |= (1 << CS30);
	TIMSK3 |= (1 << TOIE3);

	sei();

    while (1) 
    {
		//BASE MOTOR 1
		//OCR4C = 200;
		//OCR2B = 0;

		//BASE MOTOR 2
		//OCR4A = 200;
		//OCR4B = 0;

		if(strcmp(read, "ST") == 0) connected = 1;
		else if(strcmp(read, "SO") == 0) connected = 0;

		if(connected){ // IMPLEMENT DATA READY CHECKING
			sscanf(read, "%ld", &setpoint);
			//setpoint = atoi(read);
			move_abs(&micromotor3, setpoint, motor3_currentPos);

			sprintf(tobesent,"%ld", micromotor3.pid.us_time);
			strcat(tobesent, "\n");
			USART0_send_string(tobesent);
		}//else stop(&micromotor3, micromotor3.portdirpin, micromotor3.dirpin1, micromotor3.dirpin2);

		_delay_ms(10);
    }
	return(0);
}

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