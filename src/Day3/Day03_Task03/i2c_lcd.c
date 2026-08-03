#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "i2c_lcd.h"

/* PCF8574 슬레이브 주소 및 제어 비트 정의 */
#define PCF_ADDR        0x27
#define BIT_RS          (1 << 0)
#define BIT_RW          (1 << 1)
#define BIT_EN          (1 << 2)
#define BIT_BL          (1 << 3)

/* ------------------- 하드웨어 I2C(TWI) 저수준 함수 ------------------- */

static void twi_setup(void)
{
	/* 분주비 1, 약 100kHz 클럭이 나오도록 TWBR 설정 */
	TWSR = 0;
	TWBR = 72;
	TWCR = (1 << TWEN);
}

static void twi_start_condition(void)
{
	TWCR = (1 << TWEN) | (1 << TWSTA) | (1 << TWINT);
	while ((TWCR & (1 << TWINT)) == 0)
	{
		/* START 조건이 걸릴 때까지 대기 */
	}
}

static void twi_stop_condition(void)
{
	TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWINT);
	while (TWCR & (1 << TWSTO))
	{
		/* STOP 조건 완료 대기 */
	}
}

static void twi_put_byte(uint8_t byte_val)
{
	TWDR = byte_val;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while ((TWCR & (1 << TWINT)) == 0)
	{
		/* 전송 완료 대기 */
	}
}

/* PCF8574 확장 포트로 1바이트를 그대로 출력 */
static void expander_write(uint8_t port_val)
{
	twi_start_condition();
	twi_put_byte((uint8_t)(PCF_ADDR << 1));
	twi_put_byte(port_val);
	twi_stop_condition();
}

/* ------------------- LCD 4비트 프로토콜 계층 ------------------- */

/* EN 라인을 토글하여 확장 포트에 실린 값을 LCD가 래치하도록 함 */
static void latch_nibble(uint8_t byte_with_data)
{
	expander_write(byte_with_data | BIT_EN);
	_delay_us(1);
	expander_write((uint8_t)(byte_with_data & (uint8_t)~BIT_EN));
	_delay_us(50);
}

/* 상위 니블만 사용하여 4비트 데이터 한 번 전송 (mode: 0=명령, BIT_RS=데이터) */
static void push_nibble(uint8_t nibble_high, uint8_t mode)
{
	uint8_t frame = (uint8_t)((nibble_high & 0xF0) | BIT_BL | mode);
	latch_nibble(frame);
}

/* 8비트 값을 상위 니블 -> 하위 니블 순서로 전송 */
static void push_byte(uint8_t value, uint8_t mode)
{
	push_nibble(value, mode);
	push_nibble((uint8_t)(value << 4), mode);
}

void lcd_send_cmd(uint8_t cmd_byte)
{
	push_byte(cmd_byte, 0);

	/* Clear Display / Return Home 명령은 처리 시간이 더 필요함 */
	if (cmd_byte == 0x01 || cmd_byte == 0x02)
	{
		_delay_ms(2);
	}
}

void lcd_send_char(uint8_t char_byte)
{
	push_byte(char_byte, BIT_RS);
}

void lcd_module_begin(void)
{
	twi_setup();
	_delay_ms(50);

	/* HD44780 초기화 시퀀스 (8비트로 3회 웨이크업 후 4비트 모드 전환) */
	push_nibble(0x30, 0);
	_delay_ms(5);

	push_nibble(0x30, 0);
	_delay_us(150);

	push_nibble(0x30, 0);
	_delay_us(150);

	push_nibble(0x20, 0);
	_delay_us(150);

	lcd_send_cmd(0x28); /* 4-bit, 2 line, 5x8 dots */
	lcd_send_cmd(0x08); /* display off */
	lcd_send_cmd(0x01); /* clear display */
	lcd_send_cmd(0x06); /* entry mode: increment, no shift */
	lcd_send_cmd(0x0C); /* display on, cursor off, blink off */
}

void lcd_module_clear_screen(void)
{
	lcd_send_cmd(0x01);
}

void lcd_set_cursor(uint8_t line_no, uint8_t col_no)
{
	uint8_t ddram_addr = (line_no == 0) ? col_no : (uint8_t)(0x40 + col_no);
	lcd_send_cmd((uint8_t)(0x80 | ddram_addr));
}

void lcd_print_text(const char *text_ptr)
{
	while (*text_ptr)
	{
		lcd_send_char((uint8_t)*text_ptr);
		text_ptr++;
	}
}
