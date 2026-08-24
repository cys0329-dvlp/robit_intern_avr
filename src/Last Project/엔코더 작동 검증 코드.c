#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

// =====================================================
// UART 설정
// =====================================================

#define BAUD        9600
#define UBRR_VALUE  ((F_CPU / 16UL / BAUD) - 1)

// =====================================================
// 모터 PWM 설정
// =====================================================

#define PWM_TOP     255
#define MOTOR_SPEED 150

// =====================================================
// 엔코더 카운트
// =====================================================

volatile int32_t left_encoder_count  = 0;
volatile int32_t right_encoder_count = 0;

uint8_t left_last_a;
uint8_t right_last_a;


// =====================================================
// UART 초기화
// =====================================================

void UART0_Init(void)
{
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)UBRR_VALUE;

	UCSR0A = 0x00;

	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}


// =====================================================
// UART 문자 전송
// =====================================================

void UART0_SendChar(char data)
{
	while (!(UCSR0A & (1 << UDRE0)));

	UDR0 = data;
}


// =====================================================
// UART 문자열 전송
// =====================================================

void UART0_SendString(const char *str)
{
	while (*str)
	{
		UART0_SendChar(*str++);
	}
}


// =====================================================
// UART 숫자 출력
// =====================================================

void UART0_PrintInt(int32_t value)
{
	char buffer[15];

	sprintf(buffer, "%ld", (long)value);

	UART0_SendString(buffer);
}


// =====================================================
// 엔코더 초기화
// =====================================================

void Encoder_Init(void)
{
	// PD4~PD7 입력
	DDRD &= ~(
	(1 << PD4) |
	(1 << PD5) |
	(1 << PD6) |
	(1 << PD7)
	);

	// 내부 풀업 사용
	PORTD |=
	(1 << PD4) |
	(1 << PD5) |
	(1 << PD6) |
	(1 << PD7);

	// 초기 A상 상태 저장
	left_last_a =
	(PIND & (1 << PD4)) ? 1 : 0;

	right_last_a =
	(PIND & (1 << PD6)) ? 1 : 0;
}


// =====================================================
// 엔코더 업데이트
// A상 변화 감지
//
// 왼쪽:
// A = PD4
// B = PD5
//
// 오른쪽:
// A = PD6
// B = PD7
// =====================================================

void Encoder_Update(void)
{
	uint8_t left_a;
	uint8_t left_b;

	uint8_t right_a;
	uint8_t right_b;


	// 현재 엔코더 상태 읽기
	left_a  = (PIND & (1 << PD4)) ? 1 : 0;
	left_b  = (PIND & (1 << PD5)) ? 1 : 0;

	right_a = (PIND & (1 << PD6)) ? 1 : 0;
	right_b = (PIND & (1 << PD7)) ? 1 : 0;


	// =============================================
	// 왼쪽 A상 변화 감지
	// =============================================

	if (left_a != left_last_a)
	{
		// A와 B가 같으면 정방향
		if (left_a == left_b)
		{
			left_encoder_count++;
		}
		else
		{
			left_encoder_count--;
		}

		left_last_a = left_a;
	}


	// =============================================
	// 오른쪽 A상 변화 감지
	// =============================================

	if (right_a != right_last_a)
	{
		// A와 B가 같으면 정방향
		if (right_a == right_b)
		{
			right_encoder_count++;
		}
		else
		{
			right_encoder_count--;
		}

		right_last_a = right_a;
	}
}


// =====================================================
// 모터 초기화
// =====================================================

void Motor_Init(void)
{
	// 방향 핀 출력
	DDRB |=
	(1 << PB0) |
	(1 << PB1) |
	(1 << PB2) |
	(1 << PB3) |
	(1 << PB5) |
	(1 << PB6);


	// -------------------------------
	// Timer1 Fast PWM 8bit
	//
	// PB5 = OC1A
	// PB6 = OC1B
	// -------------------------------

	TCCR1A =
	(1 << COM1A1) |
	(1 << COM1B1) |
	(1 << WGM10);

	TCCR1B =
	(1 << WGM12) |
	(1 << CS11);

	OCR1A = 0;
	OCR1B = 0;
}


// =====================================================
// 모터 속도 설정
// =====================================================

void Motor_SetSpeed(uint8_t left_speed,
uint8_t right_speed)
{
	OCR1A = left_speed;
	OCR1B = right_speed;
}


// =====================================================
// 전진
// =====================================================

void Motor_Forward(void)
{
	// 왼쪽 모터 전진
	PORTB |= (1 << PB0);
	PORTB &= ~(1 << PB1);

	// 오른쪽 모터 전진
	PORTB |= (1 << PB2);
	PORTB &= ~(1 << PB3);
}


// =====================================================
// 후진
// =====================================================

void Motor_Backward(void)
{
	// 왼쪽 모터 후진
	PORTB &= ~(1 << PB0);
	PORTB |= (1 << PB1);

	PORTB &= ~(1 << PB2);
	PORTB |= (1 << PB3);
}


// =====================================================
// 정지
// =====================================================

void Motor_Stop(void)
{
	OCR1A = 0;
	OCR1B = 0;
}


// =====================================================
// 메인
// =====================================================

int main(void)
{
	uint16_t print_timer = 0;

	UART0_Init();
	Motor_Init();
	Encoder_Init();

	UART0_SendString("\r\n");
	UART0_SendString("================================\r\n");
	UART0_SendString("TT ENCODER MOTOR TEST START\r\n");
	UART0_SendString("Left: PD4(A), PD5(B)\r\n");
	UART0_SendString("Right: PD6(A), PD7(B)\r\n");
	UART0_SendString("================================\r\n");


	// 모터 전진
	Motor_Forward();

	// 약 60% 속도
	Motor_SetSpeed(MOTOR_SPEED, MOTOR_SPEED);


	while (1)
	{
		// 엔코더 상태 계속 확인
		Encoder_Update();

		// 약 100ms마다 UART 출력
		if (++print_timer >= 100)
		{
			print_timer = 0;

			UART0_SendString("L: ");
			UART0_PrintInt(left_encoder_count);

			UART0_SendString("   R: ");
			UART0_PrintInt(right_encoder_count);

			UART0_SendString("\r\n");
		}

		_delay_ms(1);
	}
}
