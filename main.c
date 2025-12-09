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

volatile int16_t motor1_currentPos = 0;
volatile int16_t motor2_currentPos = 0;
volatile int16_t motor3_currentPos = 0;
volatile int16_t motor4_currentPos = 0;

#define PACKET_SIZE 11

typedef struct __attribute__((packed)) {
	uint8_t  str;
	uint8_t  cmd;
	int16_t  sp1;
	int16_t  sp2;
	int16_t  sp3;
	int16_t  sp4;
	uint8_t  chksum;
} UART_packet;

typedef enum {
	CMD_NOP     = 0x00,   // do nothing
	CMD_CONN	= 0x01,   // start motion loop
	CMD_DISC    = 0x02,   // stop motion loop
	CMD_START   = 0x03,   // start motion loop
	CMD_STOP    = 0x04,   // stop motion loop
	CMD_HOME    = 0x05,   // begin homing routine
	CMD_ZERO    = 0x06,   // zero encoder
	CMD_SETSP   = 0x07    // update setpoints only
} CommandType;

UART_packet active_packet;

volatile uint8_t rx_buffer[PACKET_SIZE];
volatile uint8_t rx_index	= 0;       // Buffer position
volatile uint8_t data_ready	= 0;       // Flag: 1 when a full string is received

char response[100];
uint8_t received[100];
uint8_t command[100];

uint8_t start_s		= 0;
uint8_t connected	= 0;
uint8_t homing		= 0;

volatile uint8_t homing1 = 1;
volatile uint8_t homing2 = 0;

volatile uint8_t limit1	= 0;
volatile uint8_t limit2	= 0;
volatile uint8_t limit3	= 0;

float current_motor1 = 0.0;
float current_motor2 = 0.0;

Motor basemotor1;
Motor basemotor2;
Motor micromotor3;
//Motor micromotor4;

//-------- ENCODERS ---------
ISR(INT0_vect){ // A4 MOTOR 4 (PL0 PL1) - DIRECTION & PB6 - PWM
	if (PIND & (1 << PD6)) {
		motor4_currentPos++;  // If B is high while A changes, increment ticks
	} else {
		motor4_currentPos--;  // If B is low while A changes, decrement ticks
	}
}

ISR(INT1_vect){ // A3 MOTOR 3 (PL2 PL3) - DIRECTION & PB7 - PWM
	if (PIND & (1 << PD7)) {
		motor3_currentPos++;  
	} else {
		motor3_currentPos--;  
	}
}

ISR(INT2_vect){ // A2 MOTOR 2 (PH3 PH4) - DIRECTION & PWM PINS
	if (PINB & (1 << PB5)) {
		motor2_currentPos++;
	} else {
		motor2_currentPos--;
	}
}

ISR(INT3_vect){ // A1 MOTOR 1 (PH5 PH6) - DIRECTION & PWM PINS
	if (PINB & (1 << PB4)) {
		motor1_currentPos++;
	} else {
		motor1_currentPos--;
	}
}

//-------- SENSORS ---------
ISR(INT4_vect){ // SENSOR 4
	limit1 = 1;
	if(homing){
		homing1 = 0;
		homing2 = 1;
	}
}

ISR(INT5_vect){ // SENSOR 3
	limit2 = 1;
	if(homing){
		homing2 = 0;
	}
}
 
ISR(INT6_vect){ // SENSOR 2
	limit2 = 1;
	if(homing){
		homing2 = 0;
	}
}

ISR(INT7_vect){ // SENSOR 1
	limit3 = 1;
}

ISR(USART0_RX_vect) {
	uint8_t b = UDR0;

	if (rx_index == 0) {
		if (b == 0xAA) {
			rx_buffer[rx_index++] = b;
			PORTK |= (1 << PK0);
		}
		return;
	}

	rx_buffer[rx_index++] = b;

	if (rx_index == PACKET_SIZE) {
		rx_index = 0;
		data_ready = 1;
	}
}

uint8_t compute_checksum(uint8_t *data, uint8_t len) {
	uint8_t c = 0;
	for (uint8_t i = 0; i < len; i++)
	c ^= data[i];
	return c;
}

uint8_t validate_packet(uint8_t *buf) {
	return compute_checksum(buf, PACKET_SIZE - 1) == buf[PACKET_SIZE - 1];
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
	basemotor1 = init_motor(2, &PORTH, PH5, PH6, &DDRH, DDH5, DDH6, &DDRH, DDH5, &OCR2B, &OCR4C, 60);
	init_pid(&basemotor1, 1.0, 0.006, 0.0); //0.006, 0.00000008
	basemotor2 = init_motor(2, &PORTH, PH3, PH4, &DDRH, DDH3, DDH4, &DDRH, DDH3, &OCR4B, &OCR4A, 60);
	init_pid(&basemotor2, 1.0, 0.006, 0.0); //0000008

	micromotor3 = init_motor(1, &PORTL, PL2, PL3, &DDRL, DDL2, DDL3, &DDRB, DDB6, &OCR1B, NULL, 100);
	init_pid(&micromotor3, 1.2, 0.05, 0.0); //0.0000008
	//micromotor4 = init_motor(1, &PORTL, PL0, PL1, &DDRL, DDL0, DDL1, &DDRB, DDB7, &OCR0A, NULL, 255);
	//init_pid(&micromotor4, 1.2, 0.05, 0.0000008);

	//INITIALIZE MOTORS PWM
	init_pwm();

	//DEBUGGING LED
	DDRK |= (1 << DDK0);
	PORTK &= ~(1 << PK0);

    while (1) 
    {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			basemotor1.pid.current_pos	= motor1_currentPos;
			basemotor2.pid.current_pos	= motor2_currentPos;
			micromotor3.pid.current_pos = motor3_currentPos;
			//micromotor4.pid.current_pos = motor4_currentPos;
		}

		if (data_ready) {
			cli();         
			data_ready = 0;

			uint8_t temp_buf[PACKET_SIZE];
			memcpy(temp_buf, (void*)rx_buffer, PACKET_SIZE);
			sei();                  

			//if (validate_packet(temp_buf)) {
				memcpy(&active_packet, temp_buf, PACKET_SIZE);
			//}
		}
		
		if(!connected) if(active_packet.cmd == CMD_CONN){connected = 1; USART0_send_string("ackc\n");}
		if(connected){
			if(active_packet.cmd == CMD_DISC){connected = 0; USART0_send_string("ackd\n");}

			if(active_packet.cmd == CMD_HOME){homing = 1; start_s = 0;}
			else if(active_packet.cmd == CMD_STOP) homing = 0;

			if(active_packet.cmd == CMD_START){start_s = 1; homing = 0;}
			else if(active_packet.cmd == CMD_STOP) start_s = 0;

			if(homing){
				set_max_speed(&basemotor1, 60);
				set_max_speed(&basemotor2, 60);

				if(homing1) move_abs(&basemotor1, -20000, current_motor1);	
				else stop(&basemotor1); motor1_currentPos = -200;
				if(homing2) move_abs(&basemotor2, -20000, current_motor2);
				else stop(&basemotor2); motor2_currentPos = -340; 
				if(!homing1 && !homing2){ 
					homing = 0; 
					USART0_send_string("ackh\n"); 
					homing1 = 1; homing2 = 0; 
				}
			}

			if(start_s){
				//current_motor1 = read_current(6, 300);
				//current_motor2 = read_current(7, 300);
						
				move_abs(&basemotor1, active_packet.sp1, current_motor1);
				move_abs(&basemotor2, active_packet.sp2, current_motor2);
				move_abs(&micromotor3, active_packet.sp3, 0.0);
			}
					
			if(!start_s && !homing){
				stop(&basemotor1);
				stop(&basemotor2);
				stop(&micromotor3);
			}
		}

		_delay_ms(10);
    }
	return(0);
}

//FOR DEBUGGING PLACE IN INF LOOP
//sprintf(response,"%ld motor 1\n", pos1);
//PRINT FLOATS AS STRINGS
//dtostrf(current_motor2, 6, 4, response);
//USART0_send_string(response);

//sprintf(response,"%d motor 2\n", basemotor2.pid.current_pos);
//USART0_send_string(response);
				
//dtostrf(basemotor2.pid.ctrl_signal, 6, 4, response);
//USART0_send_string(response);
//USART0_send_string("\r\n");

/*
if(PINE & (1 << PE4) && basemotor1.moving_bwd){ //switch to moving_fwd if wrong
	limit1 = 0; 
	move_abs(&basemotor1, setpoint1, current_motor1);
} else stop(&basemotor1);

if(((PINE & (1 << PE5) == 0) && basemotor2.moving_bwd) || 
	((PINE & (1 << PE6) == 0) && basemotor2.moving_fwd)) //switch to moving_fwd if wrong
	stop(&basemotor2);
else{
	limit2 = 0;
	move_abs(&basemotor2, setpoint2, current_motor2);
}

if((PINE & (1 << PE5)) && (PINE & (1 << PE6))) {
	limit2 = 0; 
	move_abs(&basemotor2, setpoint2, current_motor2);
} else stop(&basemotor2);*/