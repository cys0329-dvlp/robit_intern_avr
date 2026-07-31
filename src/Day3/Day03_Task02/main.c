#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char Uart_Getch(void);
void Uart_Putch(unsigned char PutData);
void UART_transmit_string(char *str);

int main(void)
{
	UBRR0L = 16;    // Baud Rate : 57600bps
	UBRR0H = 0;
	UCSR0B = 0x18;  // 송신, 수신 기능 활성화 (RXEN0, TXEN0)
	UCSR0C = 0x06;  // START 1비트 / DATA 8비트 / STOP 1비트

	DDRA = 0XFF; //A포트 모두 출력
	DDRD = 0X00; //D포트 모두 입력
	
	//INT3 활성화
	EIMSK |= 0X08;
	EICRA |= 0X80;
	
	
	PORTA = 0XFF; //처음 시작 모두 끄기
	
	sei();
	
	while (1)
	{
		unsigned char input_data = Uart_Getch(); //숫자 받기
		
		if(input_data== '0')
		{
			PORTA = 0X7F;
			UART_transmit_string("0 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '1')
		{
			PORTA = 0XBF;
			UART_transmit_string("1 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '2')
		{
			PORTA = 0XDF;
			UART_transmit_string("2 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '3')
		{
			PORTA = 0XEF;
			UART_transmit_string("3 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '4')
		{
			PORTA = 0XF7;
			UART_transmit_string("4 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '5')
		{
			PORTA = 0XFB;
			UART_transmit_string("5 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '6')
		{
			PORTA = 0XFD;
			UART_transmit_string("6 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '7')
		{
			PORTA = 0XFE;
			UART_transmit_string("7 LED on\r");
			_delay_ms(100);
		}
		/*
		예를 들어 1111 1011 인데 오른쪽으로 한칸 옮기면 0111 1101이 되기 때문에 왼쪽 오른쪽으로 움직일 때마다
		양쪽에 1을 다시 덮어 씌워줘야함. 
		동영상 촬영 방향에 따라 다르게 보일 수 있습니다.
		*/
		else if(input_data == '8')
		{
			PORTA = (PORTA >> 1)| 0b10000000;
			UART_transmit_string("LEFT\r");
			_delay_ms(100);
		}
		else if(input_data == '9')
		{
			PORTA = (PORTA << 1)| 0b00000001; 
			UART_transmit_string("RIGHT\r");
			_delay_ms(100);
		}
		else // 0~9 숫자 말고 다른 문자 혹은 범위를 초과하는 수를 넣었을 때 오류 처리
		{
			UART_transmit_string("Please Enter Numbers between 0 and 9\r"); 
		}
		
	}
}

unsigned char Uart_Getch(void)
{
	while(!(UCSR0A & (1 << RXC0))); // UCSR0A의 Bit7 활성화 -> 데이터를 입력 받을 준비가 됐다는 flag(Datasheet 189P 참고)
	return UDR0; // 입력 buffer
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

ISR(INT3_vect) //좌측 이동했다가 우측 이동
{
	PORTA = 0XFF; // 초기에 다 꺼진 상태로 돌아감
	for(int i = 0; i < 9; i++)
	{
		UART_transmit_string("\r"); //캐리지 리턴으로 줄바꿈 많이해서 리셋된거처럼 보이기
	}
	UART_transmit_string("RESET\r");
}


