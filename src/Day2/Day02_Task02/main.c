#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "LCD_Text.h"

// LED 회로 : PA0~PA7 각 핀 - LED(캐소드) - LED(애노드) - 330옴 저항 - 5V
// => PA핀이 LOW(0)일 때 전류가 흘러 LED가 켜지는 active-low 구조
#define LED_PORT PORTA
#define LED_DDR  DDRA

int main(void)
{
	// ===== 초기화 =====
	DDRF   = 0x00;   // PF0(ADC0)를 입력으로 설정 (가변저항 입력용)
	ADMUX  = 0x40;   // 기준전압 AVCC(5V), 채널 ADC0 선택
	ADCSRA = 0x87;   // ADC Enable, 단일변환모드, 분주비 128

	LED_DDR = 0xFF;  // LED 포트 전체 출력으로 설정

	lcdInit();       // I2C LCD 초기화 (LCD_Text.c 내부에서 TWI도 함께 초기화됨)
	lcdClear();

	while (1)
	{
		unsigned int adcValue = 0;

		// ===== 1. ADC 값 읽기 =====
		ADMUX  = 0x40 | 0x00;          // 채널 0 유지
		ADCSRA |= 0x40;                // ADSC=1 : 변환 시작
		while ((ADCSRA & 0x10) == 0);  // ADIF=1(변환 완료) 될 때까지 대기
		adcValue = ADC;                 // 결과값(0~1023) 읽기

		// ===== 2. 전압 계산 (voltage*10, 소수 1자리까지 정수 연산으로 처리) =====
		unsigned int voltage_x10 = (unsigned long)adcValue * 50 / 1024;

		// ===== 3. LCD 출력 =====
		lcdClear();                              // 이전 값 잔상 방지
		lcdString(0, 0, "21th_CYS ");                  // 1행 : 이니셜
		lcdNumber(1, 6, adcValue);                // 1행 : ADC 원시값(0~1023)

		lcdNumber(1, 0, voltage_x10 / 10);        // 2행 : 전압 정수부
		lcdString(1, 1, ".");
		lcdNumber(1, 2, voltage_x10 % 10);        // 2행 : 전압 소수1자리
		lcdString(1, 3, "V");

		// ===== 4. 가변저항 값에 따라 LED 누적 점등 (막대그래프 방식) =====
		// adcValue(0~1023)를 켜야 할 LED 개수(0~8)로 환산
		unsigned char ledCount = (unsigned long)adcValue * 8 / 1023;

		// (1 << ledCount) - 1  : 하위 ledCount개 비트를 전부 1로 만듦
		//                        예) ledCount=3 -> 0000 0111
		// ~(...)               : 비트 반전 -> 하위 ledCount개는 0(켜짐), 나머지는 1(꺼짐)
		//                        예) ledCount=3 -> 1111 1000
		//                        PA0부터 순서대로 ledCount개의 LED가 "함께" 켜짐
		LED_PORT = ~((1 << ledCount) - 1);

		_delay_ms(100);
	}
}