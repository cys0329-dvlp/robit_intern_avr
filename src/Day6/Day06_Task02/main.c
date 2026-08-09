#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	// PB0 = IN1
	// PB1 = IN2
	// PB2 = IN3
	// PB3 = IN4
	// PB5 = ENA
	// PB6 = ENB

	DDRB = 0x6F;   // PB0~PB3, PB5, PB6 출력

	// 처음에는 전부 LOW
	PORTB = 0x00;

	// ENA, ENB 항상 활성화
	PORTB |= (1 << PB5) | (1 << PB6);

	while (1)
	{
		// =========================
		// 정회전
		// =========================

		// 모터 A : IN1=1, IN2=0
		PORTB |=  (1 << PB0);
		PORTB &= ~(1 << PB1);

		// 모터 B : IN3=1, IN4=0
		PORTB |=  (1 << PB2);
		PORTB &= ~(1 << PB3);

		_delay_ms(3000);


		// =========================
		// 정지
		// =========================

		PORTB &= ~((1 << PB0) |
		(1 << PB1) |
		(1 << PB2) |
		(1 << PB3));

		_delay_ms(1000);


		// =========================
		// 역회전
		// =========================

		// 모터 A : IN1=0, IN2=1
		PORTB &= ~(1 << PB0);
		PORTB |=  (1 << PB1);

		// 모터 B : IN3=0, IN4=1
		PORTB &= ~(1 << PB2);
		PORTB |=  (1 << PB3);

		_delay_ms(3000);


		// =========================
		// 정지
		// =========================

		PORTB &= ~((1 << PB0) |
		(1 << PB1) |
		(1 << PB2) |
		(1 << PB3));

		_delay_ms(1000);
	}
}