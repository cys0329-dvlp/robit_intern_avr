
#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
/*
INT3 핀은 PD3만 사용가능하다고 해서 불가피하게 영상에서 스위치를 위에서부터 
순서대로 사용하지못했습니다. 
맨 위부터 순서대로 SW1, INT3, SW2, INT4입니다.
LED는 왼쪽부터 0번입니다.
*/
int main(void)
{
	DDRA = 0XFF; //A포트 모두 출력
	DDRD = 0X00; //D포트 모두 입력
	DDRE = 0x00;
	
	//INT3 활성화
	EIMSK |= 0X08;
	EICRA = 0X80;
	
	//INT4 활성화
	EIMSK |= 0X10; // | = 전에 정의된 INT 초기화시키지않고 새로 선언
	EICRB = 0X02;
	
	sei();
	
	//0이면 LED 가 켜지고 1이면 꺼지므로 조건문 앞에 모두 !를 붙여주었습니다.
	while(1)
	{
		if((!(PIND &(1<<PIND2))) && (!(PINE & (1<<PINE5))))
		{
			PORTA = 0X00;
			_delay_ms(100);
		}
		else if (!(PINE & (1<<PINE5))) //SW1 눌렀을 떄 0~3 출력
		{
			PORTA = 0X0F;
			_delay_ms(100);
		}
		else if (!(PIND &(1<<PIND2))) // SW2 눌렀을 때 4~7 출력
		{
			PORTA = 0XF0;
			_delay_ms(100);
		}
		else
		{
			PORTA = 0XFF; //모두 끔
			_delay_ms(500);
			PORTA = 0X00; //모두 킴
			_delay_ms(500);
		}
		
	}
}

ISR(INT3_vect)
{
	PORTA = 0X7F;
	_delay_ms(100);
	
	PORTA = 0XBF;
	_delay_ms(100);
	
	PORTA = 0XDF;
	_delay_ms(100);
	
	PORTA = 0XEF;
	_delay_ms(100);
	
	PORTA = 0XF7;
	_delay_ms(100);
	
	PORTA = 0XFB;
	_delay_ms(100);
	
	PORTA = 0XFD;
	_delay_ms(100);
	
	PORTA = 0XFE;
	_delay_ms(100);
}

ISR(INT4_vect)
{
		PORTA = 0XFE;
		_delay_ms(100);
		
		PORTA = 0XFD;
		_delay_ms(100);
		
		PORTA = 0XFB;
		_delay_ms(100);
		
		PORTA = 0XF7;
		_delay_ms(100);
		
		PORTA = 0XEF;
		_delay_ms(100);
		
		PORTA = 0XDF;
		_delay_ms(100);
		
		PORTA = 0XBF;
		_delay_ms(100);
		
		PORTA = 0X7F;
		_delay_ms(100);
	
}