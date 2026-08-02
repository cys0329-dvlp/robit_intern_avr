#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define Bit_Delay() _delay_us(104) // 16,000,000 / 9600 ≈ 1667 cycle → 약 104us

void Putch(char c);
void TX_Start(void);

int main(void)
{
	DDRD |= (1 << PD3);   // PD3(TX)만 출력으로 설정
	PORTD |= (1 << PD3);  // 기본 상태 = HIGH

	while (1)
	{
		TX_Start();
		_delay_ms(1000); // 1초마다 보내기
	}
}

void TX_Start(void)
{
	Putch('H');
	Putch('e');
	Putch('l');
	Putch('l');
	Putch('o');
	Putch(' ');
	Putch('W');
	Putch('o');
	Putch('r');
	Putch('l');
	Putch('d');
	Putch('!');
}

void Putch(char str)
{
	PORTD &= ~(1 << PD3); // START BIT: LOW
	Bit_Delay(); 

	// 데이터 8비트
	for (int i = 0; i < 8; i++)
	{
		if (str & 0x01)
		{
			PORTD |= (1 << PD3);
		}
		else
		{
			PORTD &= ~(1 << PD3);
		}

		Bit_Delay();
		str >>= 1; // 비트 한칸씩 옮기기(str = str >>1 이랑 똑같음)
	}

	// 스톱 비트: HIGH
	PORTD |= (1 << PD3);
	Bit_Delay();
}