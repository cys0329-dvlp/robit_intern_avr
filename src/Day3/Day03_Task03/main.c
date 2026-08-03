#define F_CPU 16000000UL // Define clock speed (16 MHz)
#include <avr/io.h>
#include <util/delay.h>

// Initialize UART
void uart_init() {
	unsigned int ubrr = F_CPU / 16 / 9600 - 1;  // Set baud rate to 9600
	UBRR0H = (unsigned char)(ubrr >> 8);  // Set baud rate high byte
	UBRR0L = (unsigned char)ubrr;         // Set baud rate low byte
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);  // Enable receiver and transmitter
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // Set 8 data bits, no parity, 1 stop bit
}

// Transmit data over UART
void uart_transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty transmit buffer
	UDR0 = data;  // Send data
}

// Send string over UART
void uart_send_string(const char* str) {
	while (*str) {
		uart_transmit(*str++);
	}
}

int main(void) {
	uart_init();  // Initialize UART
	while (1) {
		uart_send_string("Hello, World!\r\n");  // Send "Hello, World!" over UART
		_delay_ms(1000);  // Wait for 1 second before sending again
	}
}