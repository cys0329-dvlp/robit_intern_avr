#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// ============================================================
// MG995 Servo
// Signal : PB7 (OC1C)
// ============================================================

void Servo_Init(void)
{
	// PB7 = OC1C 출력
	DDRB |= (1 << PB7);

	// Timer1 Fast PWM Mode 14
	// TOP = ICR1
	// 비반전 PWM
	TCCR1A = (1 << COM1C1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

	// 16 MHz / 8 = 2 MHz
	// 20 ms = 40000 counts
	ICR1 = 40000;

	// 초기 위치 약 0도
	OCR1C = 2000;
}

void Servo_SetAngle(uint16_t angle)
{
	uint16_t pulse;

	// 0도   = 1.0 ms
	// 90도  = 1.5 ms
	// 180도 = 2.0 ms
	pulse = 2000 + ((uint32_t)angle * 2000 / 180);

	OCR1C = pulse;
}

int main(void)
{
	Servo_Init();

	while (1)
	{
		// 0도
		Servo_SetAngle(0);
		_delay_ms(1000);

		// 90도
		Servo_SetAngle(90);
		_delay_ms(1000);

		// 180도
		Servo_SetAngle(180);
		_delay_ms(1000);
	}
}
