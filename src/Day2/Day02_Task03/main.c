#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "LCD_Text.h"

// ===== 스위치 핀 설정 =====
// SW1 : PD2 (A 증가)
// SW2 : PD3 (연산자 변경)
// SW3 : PE4 (B 증가)
// SW4 : PE5 (계산 실행)

#define SW1 PD2
#define SW2 PD3
#define SW3 PE4
#define SW4 PE5

// 계산 함수
int calculate(int a, int b, char op)
{
	switch (op)
	{
		case '+':
		return a + b;

		case '-':
		return a - b;

		case '*':
		return a * b;

		case '/':
		if (b != 0)
		return a / b;
		else
		return 0;
	}

	return 0;
}

int main(void)
{
	// ===== PD2, PD3 입력 + 내부 풀업 =====
	DDRD &= ~((1 << SW1) | (1 << SW2));
	PORTD |= (1 << SW1) | (1 << SW2);

	// ===== PE4, PE5 입력 + 내부 풀업 =====
	DDRE &= ~((1 << SW3) | (1 << SW4));
	PORTE |= (1 << SW3) | (1 << SW4);

	// LCD 초기화
	lcdInit();
	lcdClear();

	// 계산기 변수
	int A = 1;
	int B = 1;

	char opList[4] = {'+', '-', '*', '/'};
	unsigned char opIndex = 0;

	// 이전 스위치 상태 저장
	unsigned char prev1 = 1;
	unsigned char prev2 = 1;
	unsigned char prev3 = 1;
	unsigned char prev4 = 1;

	// 현재 스위치 상태
	unsigned char cur1, cur2, cur3, cur4;

	char buffer[17];

	// 최초 화면 출력
	sprintf(buffer, "%d%c%d", A, opList[opIndex], B);
	lcdString(0, 0, buffer); //0행 0열에 출력

	while (1)
	{
		// 현재 스위치 상태 읽기
		// SW1 (PD2)
		if (PIND & (1 << SW1))
		{
			cur1 = 1;   // 안 눌림 (Pull-up 때문에 HIGH)
		}
		else
		{
			cur1 = 0;   // 눌림 (LOW)
		}

		// SW2 (PD3)
		if (PIND & (1 << SW2))
		{
			cur2 = 1;
		}
		else
		{
			cur2 = 0;
		}

		// SW3 (PE4)
		if (PINE & (1 << SW3))
		{
			cur3 = 1;
		}
		else
		{
			cur3 = 0;
		}

		// SW4 (PE5)
		if (PINE & (1 << SW4))
		{
			cur4 = 1;
		}
		else
		{
			cur4 = 0;
		}

		// ==========================
		// SW1 : A 증가
		// ==========================
		if (prev1 == 1 && cur1 == 0)
		{
			_delay_ms(20);

			A++;

			lcdClear();
			sprintf(buffer, "%d%c%d", A, opList[opIndex], B);
			lcdString(0, 0, buffer);
		}

		// ==========================
		// SW2 : 연산자 변경
		// ==========================
		if (prev2 == 1 && cur2 == 0)
		{
			_delay_ms(20);

			opIndex = (opIndex + 1) % 4;

			lcdClear();
			sprintf(buffer, "%d%c%d", A, opList[opIndex], B);
			lcdString(0, 0, buffer);
		}

		// ==========================
		// SW3 : B 증가
		// ==========================
		if (prev3 == 1 && cur3 == 0)
		{
			_delay_ms(20);

			B++;

			lcdClear();
			sprintf(buffer, "%d%c%d", A, opList[opIndex], B);
			lcdString(0, 0, buffer);
		}

		// ==========================
		// SW4 : 계산 실행
		// ==========================
		if (prev4 == 1 && cur4 == 0)
		{
			_delay_ms(20);

			int C = calculate(A, B, opList[opIndex]);

			lcdClear();
			sprintf(buffer, "%d%c%d=%d", A, opList[opIndex], B, C);
			lcdString(0, 0, buffer);
		}

		// 이전 상태 저장
		prev1 = cur1;
		prev2 = cur2;
		prev3 = cur3;
		prev4 = cur4;

		_delay_ms(10);
	}

	return 0;
}