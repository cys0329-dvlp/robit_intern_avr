#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#include <stdint.h>

/* PCF8574 기반 I2C LCD(4비트 모드) 제어 인터페이스 */

void lcd_module_begin(void);
void lcd_module_clear_screen(void);
void lcd_send_cmd(uint8_t cmd_byte);
void lcd_send_char(uint8_t char_byte);
void lcd_set_cursor(uint8_t line_no, uint8_t col_no);
void lcd_print_text(const char *text_ptr);

#endif
