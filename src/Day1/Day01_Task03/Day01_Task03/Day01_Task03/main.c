
#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
/*
PD2,3 PE4,5 를 사용하므로 INT0~3이 아닌 
INT2~5를 사용했습니다. 
*/
int count = 0xFF; //INT5에서도 써야하기 때문에 전역 변수로 설정

int main(void)
{
	DDRA = 0XFF; //A포트 모두 출력
	DDRD = 0X00; //D포트 모두 입력
	DDRE = 0x00;
	
	//INT2 활성화
	EIMSK |= 0X04;
	EICRA |= 0X20;
	
	//INT3 활성화
	EIMSK |= 0X08;
	EICRA |= 0X80;
	
	//INT4 활성화
	EIMSK |= 0X10;
	EICRB |= 0X02;
	
	//INT5 활성화
	EIMSK |= 0X20; // | = 전에 정의된 INT 초기화시키지않고 새로 선언
	EICRB |= 0X08;
	
	sei();


	while(1)
	{
		PORTA = count;   // count의 각 비트를 그대로 LED에 반영
		_delay_ms(100);
		if(count == 0X00)
		{
			count = 0XFF; //인터럽트에서는 무한루프 걸리면 안되기 때문에 한번 다 돌면 나가기
		}
		else
		{
			count--; //0XFF로 다 채운 상태에서 1씩 감소
		}
	}
}


ISR(INT2_vect) //3개씩 좌측 이동
{
	for (int i = 0; i < 8; i++)
	{
		PORTA = ~(1 << i) & ~(1 << (i+1)) & ~(1 << (i+2));
		_delay_ms(300);
		if((i+2) > 8)
		{
			break;
		}
	}
}

ISR(INT4_vect) // 3개씩 우측 이동
{
	for (int i = 7; i >=0; i--)
	{
		PORTA = ~(1 << (i-2)) & ~(1 << (i-1)) & ~(1 << i);
		_delay_ms(300);
		if(i <0)
		{
			break;
		}
	}
}
				
ISR(INT3_vect) //좌측 이동했다가 우측 이동
{
	//PA는 안써도 <avr/io.h>에 이미 PA0 = 0, PA1 = 1.. 로 정의되어있음
	
	for (int i = 0; i < 8; i++)
	{
		PORTA = ~(1 << i);
		_delay_ms(100);
	}
	for (int i = 0; i < 8; i++)
	{
		PORTA = ~(1 << (7 - i));  //0이 켜지고 1이 꺼지므로 ~로 반전시켜야함
		_delay_ms(100);
	}
}
	
ISR(INT5_vect) // 2진 카운터 초기화
{
	count = 0XFF; //count만 처음으로 초기화 시킨 뒤 main에서 다시 돌리기
}

