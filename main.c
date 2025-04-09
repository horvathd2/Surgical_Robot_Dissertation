/*
 * TEST_ATMEL.c
 *
 * Created: 6/28/2024 10:40:55 AM
 * Author : H.Daniel
 */


#include "config.h"
#include "uart.h"
#include "motor.h"
#include "sensor.h"

volatile int32_t motor1_currentPos = 0;
volatile int32_t motor2_currentPos = 0;
volatile int32_t motor3_currentPos = 0;
volatile int32_t motor4_currentPos = 0;

volatile char read[100];
volatile uint8_t rx_index	= 0;         // Buffer position
volatile uint8_t data_ready	= 0;       // Flag: 1 when a full string is received

char response[100];
char command[100];
char *token;

int32_t setpoint1	= 0;
int32_t setpoint2	= 0;
int32_t setpoint3	= 0;
uint8_t start_s		= 0;
uint8_t connected	= 0;
uint8_t homing		= 0;
uint8_t homed1		= 0;
uint8_t homed2		= 0;

volatile uint8_t limit1		= 0;
volatile uint8_t limit2		= 0;
volatile uint8_t limit3		= 0;

float current_motor1 = 0;
float current_motor2 = 0;

Motor basemotor1;
Motor basemotor2;
Motor micromotor3;
Motor micromotor4;

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
		motor1_currentPos++;  
	} else {
		motor1_currentPos--;  
	}
}

ISR(INT2_vect){ // A4 MOTOR 4 (PL0 PL1) - DIRECTION & PB6 - PWM
	if (PIND & (1 << PD6)) {
		motor4_currentPos++;  
	} else {
		motor4_currentPos--;  
	}
}

ISR(INT3_vect){ // A3 MOTOR 3 (PL2 PL3) - DIRECTION & PB7 - PWM
	if (PIND & (1 << PD7)) {
		motor3_currentPos++;  
	} else {
		motor3_currentPos--;  
	}
}

//-------- SENSORS ---------
ISR(INT4_vect){ // SENSOR 4
	limit1 = 1;
}

ISR(INT5_vect){ // SENSOR 3
	limit2 = 1;
}
 
ISR(INT6_vect){ // SENSOR 2
	limit2 = 1;
}

ISR(INT7_vect){ // SENSOR 1
	limit3 = 1;
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
	//SETUP TIMER 3 FOR MICROSECOND COUNTING FROM BOOT & TIMER 3 INTERRUPT
	setup_micros();

	//SETUP LIMIT SENSORS
	setup_ext_sensors();

	//INITIALIZE SERIAL COMMUNICATION
	USART0_init();

	//INITIALIZE CURRENT SENSORS
	ADC_init();

	//INITIALIZE MOTORS
	basemotor1 = init_motor(2, &PORTH, PH5, PH6, &DDRH, DDH5, DDH6, motor1_currentPos, &DDRH, DDH5, &OCR2B, &OCR4C, 200);
	init_pid(&basemotor1, 1.0, 0.006, 0.0);
	basemotor2 = init_motor(2, &PORTH, PH3, PH4, &DDRH, DDH3, DDH4, motor2_currentPos, &DDRH, DDH3, &OCR4B, &OCR4A, 200);
	init_pid(&basemotor2, 1.0, 0.006, 0.0); //0000008

	micromotor3 = init_motor(1, &PORTL, PL2, PL3, &DDRL, DDL2, DDL3, motor3_currentPos, &DDRB, DDB6, &OCR1B, NULL, 255);
	init_pid(&micromotor3, 1.2, 0.05, 0.0000008);
	//micromotor4 = init_motor(1, &PORTL, PL0, PL1, &DDRL, DDL0, DDL1, motor4_currentPos, &DDRB, DDB7, &OCR0A, NULL, 255);
	//init_pid(&micromotor4, 1.2, 0.05, 0.0000008);

	//INITIALIZE MOTORS PWM
	init_pwm();

	//SETUP RELAY PIN
	DDRF  |= (1 << DDF7);

    while (1) 
    {
		//*************************************************
		// FOR CURRENT SENSING AND LIMITING

		// EITHER RUN EVERY CYCLE WITHOUT FOR (5 SAMPLES = 5 CYCLES = 50ms) SAMPLES++;
		// IF SAMPLES == 100 THEN GET AVERAGE VALUE OF CURRENT FOR PWM
		// AND THEN COMPARE TO CHECK IF CURRENT IS GREATER THAN LIMIT
		// FINALLY RESET SAMPLES = 0

		// ADD HISTERESHYS!!!!!!!!!!!!!!!!

		// MAY BE BETTER TO USE JUST A FOR() LOOP
		// MORE EFFICIENT AND CAN USE MORE SAMPLES PER CYCLE

		//*************************************************
		
		if(!connected) if(strcmp(read, "conn") == 0){connected = 1; USART0_send_string("ackc\n");}
		if(connected){
			if(strcmp(read, "disc") == 0){connected = 0; USART0_send_string("ackd\n");}

			token = strtok(read, "/");
			if (token != NULL) strcpy(command, token);

			token = strtok(NULL, "/");
			if (token != NULL) setpoint1 = atoi(token);

			token = strtok(NULL, "/");
			if (token != NULL) setpoint2 = atoi(token);

			token = strtok(NULL, "/");
			if (token != NULL) setpoint3 = atoi(token);
			
			if(strcmp(command, "HM") == 0){homing = 1; start_s = 0;}
			else if(strcmp(command, "HS") == 0) homing = 0;

			if(strcmp(command, "ST") == 0){start_s = 1; homing = 0;}
			else if(strcmp(command, "SO") == 0) start_s = 0;

			if(homing){ //MODIFY!!!!!
				home_motor(&basemotor1);
				if(homed1){
					home_motor(&basemotor2);
					if(homed2){
						USART0_send_string("ackhomed\n");
						homing = 0;
					}
				}
			}

			if(start_s){
				// IMPLEMENT DATA READY CHECKING

				if(PINE & (1 << PE4)) {limit1 = 0; move_abs(&basemotor1, setpoint1, motor1_currentPos);}
				else stop(&basemotor1);

				if((PINE & (1 << PE5)) && (PINE & (1 << PE6))) {limit2 = 0; move_abs(&basemotor2, setpoint2, motor2_currentPos);}
				else stop(&basemotor2);

				move_abs(&micromotor3, setpoint3, motor3_currentPos);

				current_motor1 = read_current(6,400);
				//sprintf(response,"%ld", motor1_currentPos);
				//strcat(response, "- pos1 \n");
				//USART0_send_string(response);

				dtostrf(current_motor1, 6, 4, response);
				strcat(response, "\n");
				USART0_send_string(response);
			}else{
				stop(&basemotor1);
				stop(&basemotor2);
				stop(&micromotor3);
			}
		}
		
		_delay_ms(5);
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

	//BASE MOTOR 1
	//OCR4C = 200;
	//OCR2B = 0;

	//BASE MOTOR 2
	//OCR4A = 200;
	//OCR4B = 0;