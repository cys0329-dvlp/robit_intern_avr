#ifndef F_CPU
#define F_CPU 16000000UL   // main.c에서 정의 안 했을 경우를 대비한 기본값
#endif

#include "LCD_Text.h"
#include <stdlib.h>   // itoa

// PCF8574 각 비트가 LCD의 어느 핀에 연결되는지 (일반적인 I2C LCD 백팩 배선)
#define LCD_RS        0x01   // P0
#define LCD_RW        0x02   // P1
#define LCD_EN        0x04   // P2
#define LCD_BACKLIGHT 0x08   // P3 (백라이트 상시 ON 용)
// P4~P7 : LCD D4~D7 (데이터 4비트)

// ================= TWI(I2C) 저수준 함수 =================

static void twiInit(void)
{
	TWSR = 0x00;                            // 프리스케일러 = 1
	TWBR = ((F_CPU / 100000UL) - 16) / 2;   // SCL 클럭 100kHz로 설정
	TWCR = (1 << TWEN);                     // TWI 하드웨어 활성화
}

static void twiStart(void)
{
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));         // START 조건 전송 완료까지 대기
}

static void twiStop(void)
{
	TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
	_delay_us(20);
}

static void twiWrite(unsigned char data)
{
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));         // 1바이트 전송 완료까지 대기
}

// PCF8574로 1바이트(8개 핀 상태)를 통째로 전송
static void i2cSendByte(unsigned char data)
{
	twiStart();
	twiWrite(LCD_I2C_ADDR << 1);            // SLA+W : 주소를 왼쪽으로 1비트 밀고 마지막 비트=0(쓰기)
	twiWrite(data);
	twiStop();
}

// ================= LCD 저수준 제어 =================

// EN 핀을 0->1->0으로 흔들어 PCF8574가 latch한 데이터를 LCD가 읽어가게 함
static void lcdEnablePulse(unsigned char data)
{
	i2cSendByte(data | LCD_EN);
	_delay_us(1);
	i2cSendByte(data & ~LCD_EN);
	_delay_us(50);
}

// 4비트(니블) 하나를 RS 상태를 유지한 채 I2C로 전송
static void lcdWriteNibble(unsigned char nibble, unsigned char rs)
{
	unsigned char data = (nibble & 0xF0) | LCD_BACKLIGHT | rs;
	i2cSendByte(data);       // 데이터 먼저 세팅
	lcdEnablePulse(data);    // EN 펄스로 확정
}

void lcdCommand(unsigned char cmd)
{
	lcdWriteNibble(cmd & 0xF0, 0);          // RS=0 : 명령어 모드, 상위 4비트
	lcdWriteNibble((cmd << 4) & 0xF0, 0);   // 하위 4비트
	_delay_ms(2);
}

void lcdData(unsigned char data)
{
	lcdWriteNibble(data & 0xF0, LCD_RS);        // RS=1 : 데이터 모드, 상위 4비트
	lcdWriteNibble((data << 4) & 0xF0, LCD_RS); // 하위 4비트
	_delay_ms(2);
}

void lcdInit(void)
{
	twiInit();
	_delay_ms(50);   // 전원 안정화 대기

	// 4비트 모드 진입 시퀀스 (HD44780 표준 초기화, PPT와 동일한 흐름)
	lcdWriteNibble(0x30, 0);
	_delay_ms(5);
	lcdWriteNibble(0x30, 0);
	_delay_us(200);
	lcdWriteNibble(0x30, 0);
	_delay_us(200);
	lcdWriteNibble(0x20, 0);   // 4비트 모드로 전환
	_delay_us(200);

	lcdCommand(0x28); // Function Set : 4bit, 2행, 5x7 dot
	lcdCommand(0x0C); // Display ON, Cursor OFF, Blink OFF
	lcdCommand(0x06); // Entry Mode : 커서 우측 이동, Shift 없음
	lcdCommand(0x01); // Display Clear
	_delay_ms(2);
}

void lcdClear(void)
{
	lcdCommand(0x01);
	_delay_ms(2);
}

// row(0/1), col(0~15)을 DDRAM 주소로 변환
static void lcdGotoXY(unsigned char row, unsigned char col)
{
	unsigned char addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
	lcdCommand(addr);
}

void lcdString(unsigned char row, unsigned char col, char *str)
{
	lcdGotoXY(row, col);
	while (*str)
	{
		lcdData(*str);
		str++;
	}
}

void lcdNumber(unsigned char row, unsigned char col, int num)
{
	char buffer[7];
	itoa(num, buffer, 10);
	lcdString(row, col, buffer);
}