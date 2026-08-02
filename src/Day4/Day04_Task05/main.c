#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>

int Uart_Getint(void);
void Servo_Move(int angle);
void UART_transmit_string(char *str);
void Uart_Putch(unsigned char PutData);

int main(void)
{
	UBRR0L = 16;    // Baud Rate : 57600bps
	UBRR0H = 0;
	UCSR0B = 0x18;  // 송신, 수신 기능 활성화 (RXEN0, TXEN0)
	UCSR0C = 0x06;  // START 1비트 / DATA 8비트 / STOP 1비트
	
	DDRB|=0x20;
	PORTB|=0x20;
	
	// PB7(OC1C)을 출력으로 설정
	DDRB |= (1 << PB7);

	// Fast PWM, TOP = ICR1, 8분주
	// 16MHz / 8 = 2MHz → 1틱 = 0.5us
	// ICR1 = 40000 → 주기 = 40000 * 0.5us = 20ms
	ICR1 = 39999;

	TCCR1A = (1 << COM1C1) | (1 << WGM11); // 비반전 모드, Fast PWM
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // 8분주

	OCR1C = 3000; // 초기값 (1.5ms, 중립 위치)
	Servo_Move(90); // 처음 시작하거나 리셋하면 초기 자세로
	while (1)
	{
		int input_data = Uart_Getint(); //숫자 받기
		
		if(0<=input_data && input_data <=180)
		{
			Servo_Move(input_data);
			_delay_ms(1000);
		}
		else if(input_data < 0 || 180 < input_data )
		{
			UART_transmit_string("ERROR. angle can be between 0 and 180\r");
		}
		else
		{
			UART_transmit_string("ERROR. angle can be between 0 and 180\r");
		}
	}
}

int Uart_Getint(void)
{
	unsigned char ch;
	int result = 0;
	int started = 0;  // 숫자를 하나라도 받았는지 체크

	while (1)
	{
		while(!(UCSR0A & (1 << RXC0))); // 데이터 수신 대기
		ch = UDR0;

		if (ch == '\r' || ch == '\n')
		{
			if (started)
			{
				break;      // 숫자 입력 후 엔터 -> 정상 종료
			}
			else
			{
				continue;   // 숫자 없이 들어온 개행문자는 그냥 무시(잔여 \n 처리)
			}
			
		}
		else if (ch >= '0' && ch <= '9')
		{
			result = result * 10 + (ch - '0'); // ch 아스키값에서 0의 아스키 값인 48을 빼서 진짜 입력한 숫자로 인식되게 함
			started = 1;
		}
		else
		{
			return -1;
			break;
		}
	}

	return result;
}

void Servo_Move(int angle) 
{
	// 0도: 1ms = 2000틱, 180도: 2ms = 4000틱
	OCR1C = 2000 + ((long)angle * 2000 / 180);
}

void Uart_Putch(unsigned char PutData)
{
	while(!(UCSR0A & (1 << UDRE0))); // UCSR0A의 Bit5 활성화 -> 새로운 Data를 입력받을 준비가 됐다는 flag (Datasheet 189P 참고)
	UDR0 = PutData; //받은 값을 입력 buffer에 넣음
}

void UART_transmit_string(char *str)
{
	while(*str != '\0') // \0이면 while 문 중지
	{
		Uart_Putch(*str);
		str++;
	}
}

