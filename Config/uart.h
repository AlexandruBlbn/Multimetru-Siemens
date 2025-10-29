/*
 * uart.h
 *
 * Created: 10/21/2025 2:27:25 PM
 *  Author: Alexandru
 */ 


#ifndef UART_H_
#define UART_H_



#define F_CPU 16000000UL
#define BAUD 115200
#define UBRR ((F_CPU / (8UL * BAUD)) - 1)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

static inline void uart_init(void){
	UBRR0H = (UBRR >> 8);
	UBRR0L = (UBRR & 0xFF);
	
	UCSR0A = (1 << U2X0);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static inline void uart_putc(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

static inline char uart_getc(void) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

static inline void uart_puts(const char *str) {
	while (*str) {
		uart_putc(*str++);
	}
}

static inline void uart_putInt(int32_t val) {
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "%ld", val);
	uart_puts(buffer);
}

static inline void uart_putUInt32(uint32_t val) {
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "%lu", val);
	uart_puts(buffer);
}

static inline void uart_putFloat(float val, uint8_t decimals) {
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "%.*f", decimals, val);
	uart_puts(buffer);
}

static inline char uartCheck(char data) {
	uart_putc(data);
	return uart_getc();
}



#endif /* UART_H_ */