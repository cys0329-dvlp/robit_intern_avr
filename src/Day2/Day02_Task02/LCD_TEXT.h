#ifndef LCD_TEXT_H_
#define LCD_TEXT_H_

#include <avr/io.h>
#include <util/delay.h>

// PCF8574 I2C LCD 백팩 모듈의 슬레이브 주소
// 보드에 따라 0x27 또는 0x3F 인 경우가 많음 (동작 안 하면 이 값 바꿔서 테스트)
#define LCD_I2C_ADDR 0x27

// ===== 함수 원형 =====
void lcdInit(void);
void lcdClear(void);
void lcdCommand(unsigned char cmd);
void lcdData(unsigned char data);
void lcdString(unsigned char row, unsigned char col, char *str);
void lcdNumber(unsigned char row, unsigned char col, int num);

#endif /* LCD_TEXT_H_ */