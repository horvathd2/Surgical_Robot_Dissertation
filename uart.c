/*
 * uart.c
 *
 * Created: 7/22/2024 11:20:12 AM
 *  Author: H.Daniel
 */ 

#include "uart.h"

void USART0_init(void){
	//USART BAUD RATE SETUP
	UBRR0H = (unsigned char) (BRC >> 8);
	UBRR0L = (unsigned char) BRC;
	
	//RX TX ENABLE
	//UCSR0A = (1 << U2X0);
	UCSR0B = (1 << RXEN0)  | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART0_send(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void USART0_send_string(char *ptr){
	while (*ptr) {
		USART0_send(*ptr++);
	}
}

unsigned char USART0_receive(void) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void USART0_read_string(char *array, uint8_t bufsize){
	if(UCSR0A & (1 << RXC0)){	
		volatile uint8_t i = 0;
		char c;
	
		// Receive characters until newline or buffer limit
		while (i < bufsize - 1) {
			c = USART0_receive();
			if (c == '\n' || c == '\r') {
				break;
			}
			array[i++] = c;
		}

		array[i] = '\0';
	}
}