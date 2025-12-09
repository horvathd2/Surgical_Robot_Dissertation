/*
 * uart.h
 *
 * Created: 7/22/2024 11:19:56 AM
 *  Author: H.Daniel
 */ 


#ifndef UART_H_
#define UART_H_

#include "config.h"

#define BAUDR			9600
#define BRC				((F_CPU/8/BAUDR)-1)
#define RX_BUFFER_SIZE	128

void USART0_init(void);

void USART0_send(unsigned char data);
void USART0_send_string(char *ptr);

unsigned char USART0_receive(void);
void USART0_read_string(char *array, uint8_t bufsize);

#endif /* UART_H_ */