/*
 * 과제 1 - ATmega128 (Microchip Studio)
 * -------------------------------------------------
 * - IR 센서 6개 : PF2 ~ PF7 (ADC channel 2~7)
 * - LED 6개    : PA0 ~ PA5 (IR 센서와 1:1 대응)
 * - USART1 (PD2/PD3) : 9600bps, 터미널 출력
 * - I2C LCD (PCF8574, 16x2)   : SCL = PD0, SDA = PD1
 *
 * [가정 사항 - 필요시 수정하세요]
 * 1) MAF(이동평균필터) 윈도우 크기 = 5
 * 2) min / max 는 전원 인가 후 누적(계속 갱신)되는 값
 * 3) I2C LCD 주소 = 0x27 (PCF8574T 기본값, 모듈에 따라 0x3F일 수 있음)
 * 4) PCF8574 <-> LCD 배선 순서 : D7 D6 D5 D4 BL EN RW RS (가장 흔한 배선)
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

/* ------------------- 설정값 ------------------- */
#define IR_COUNT   6          // IR 센서 개수 (PF2~PF7)
#define MAF_SIZE   5           // 이동평균필터 윈도우 크기
#define LED_PORT   PORTA
#define LED_DDR    DDRA
#define NORM_THRESHOLD 0.8f

#define LCD_ADDR   0x27        // I2C LCD(PCF8574) 주소, 모듈에 따라 0x3F로 변경

/* ================================================
 *                USART1 (9600bps)
 * ================================================ */
void UART1_init(unsigned int ubrr)
{
    UBRR1H = (unsigned char)(ubrr >> 8);
    UBRR1L = (unsigned char)ubrr;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8N1
}

void UART1_transmit(unsigned char data)
{
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = data;
}

void UART1_print(const char *str)
{
    while (*str) UART1_transmit(*str++);
}

/* ================================================
 *                     ADC
 * ================================================ */
void ADC_init(void)
{
    ADMUX  = (1 << REFS0);                                   // AVCC 기준전압
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 분주비 128
}

uint16_t ADC_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

/* ================================================
 *                 I2C (TWI) 드라이버
 * ================================================ */
void I2C_init(void)
{
    TWSR = 0x00;   // 프리스케일러 1
    TWBR = 72;     // 16MHz 기준 100kHz -> TWBR=72
    TWCR = (1 << TWEN);
}

void I2C_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(10);
}

void I2C_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

/* ================================================
 *         I2C LCD (PCF8574 + HD44780, 4bit)
 *   비트배치 : D7 D6 D5 D4 BL EN RW RS
 * ================================================ */
#define LCD_BACKLIGHT 0x08
#define LCD_EN        0x04
#define LCD_RW        0x02
#define LCD_RS        0x01

void LCD_writeI2C(uint8_t data)
{
    I2C_start();
    I2C_write(LCD_ADDR << 1);
    I2C_write(data);
    I2C_stop();
}

void LCD_pulseEnable(uint8_t data)
{
    LCD_writeI2C(data | LCD_EN);
    _delay_us(1);
    LCD_writeI2C(data & ~LCD_EN);
    _delay_us(50);
}

void LCD_sendNibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    LCD_writeI2C(data);
    LCD_pulseEnable(data);
}

void LCD_sendByte(uint8_t byte, uint8_t mode)
{
    LCD_sendNibble(byte & 0xF0, mode);
    LCD_sendNibble((byte << 4) & 0xF0, mode);
}

void LCD_command(uint8_t cmd)  { LCD_sendByte(cmd, 0); }
void LCD_data(uint8_t data)    { LCD_sendByte(data, LCD_RS); }

void LCD_init(void)
{
    _delay_ms(50);
    LCD_sendNibble(0x30, 0); _delay_ms(5);
    LCD_sendNibble(0x30, 0); _delay_us(150);
    LCD_sendNibble(0x30, 0); _delay_us(150);
    LCD_sendNibble(0x20, 0); _delay_us(150); // 4bit mode 진입

    LCD_command(0x28); // 4bit, 2line, 5x8 font
    LCD_command(0x0C); // display on, cursor off
    LCD_command(0x06); // entry mode set
    LCD_command(0x01); // clear display
    _delay_ms(2);
}

void LCD_setCursor(uint8_t col, uint8_t row)
{
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_command(addr);
}

void LCD_print(const char *str)
{
    while (*str) LCD_data(*str++);
}

/* ================================================
 *                      MAIN
 * ================================================ */
int main(void)
{
    uint16_t raw[IR_COUNT];
    static uint16_t maf_buf[IR_COUNT][MAF_SIZE];
    uint8_t  maf_idx = 0;
    uint8_t  maf_filled = 0;

    uint16_t filtered[IR_COUNT];
    uint16_t ir_min[IR_COUNT];
    uint16_t ir_max[IR_COUNT];
    float    norm[IR_COUNT];

    char buf[64];
    char normStr[8];

    UART1_init(103);   // 16MHz, 9600bps -> UBRR = 103
    ADC_init();
    I2C_init();
    LCD_init();

    LED_DDR  = 0x3F;   // PA0~PA5 출력 설정
    LED_PORT = 0x00;

    for (uint8_t i = 0; i < IR_COUNT; i++) {
        ir_min[i] = 1023;
        ir_max[i] = 0;
    }

    while (1)
    {
        for (uint8_t i = 0; i < IR_COUNT; i++)
        {
            /* 1) 원본 값 읽기 : PF2~PF7 -> ADC channel 2~7 */
            raw[i] = ADC_read(i + 2);

            /* 2) 이동평균필터(MAF) 적용 */
            maf_buf[i][maf_idx] = raw[i];
            uint32_t sum = 0;
            uint8_t  cnt = maf_filled ? MAF_SIZE : (maf_idx + 1);
            for (uint8_t j = 0; j < cnt; j++) sum += maf_buf[i][j];
            filtered[i] = (uint16_t)(sum / cnt);

            /* 3) 누적 min / max 갱신 */
            if (filtered[i] < ir_min[i]) ir_min[i] = filtered[i];
            if (filtered[i] > ir_max[i]) ir_max[i] = filtered[i];

            /* 4) 정규화 */
            if (ir_max[i] > ir_min[i])
                norm[i] = (float)(filtered[i] - ir_min[i]) / (float)(ir_max[i] - ir_min[i]);
            else
                norm[i] = 0.0f;

            /* 5) LED 제어 : 정규화값 0.8 이상이면 ON, 이하면 OFF */
            if (norm[i] >= NORM_THRESHOLD)
                LED_PORT |= (1 << i);
            else
                LED_PORT &= ~(1 << i);
        }

        maf_idx++;
        if (maf_idx >= MAF_SIZE) { maf_idx = 0; maf_filled = 1; }

        /* ---------- USART1 로 값 출력 ---------- */
        UART1_print("       original / filter(MAF) / min / max / norm\r\n");
        for (uint8_t i = 0; i < IR_COUNT; i++)
        {
            dtostrf(norm[i], 4, 2, normStr);
            sprintf(buf, "IR %d : %4u        %4u          %4u %4u   %s\r\n",
                    i, raw[i], filtered[i], ir_min[i], ir_max[i], normStr);
            UART1_print(buf);
        }
        UART1_print("\r\n");

        /* ---------- LCD 에 정규화 값 6개 출력 (1줄에 3개씩) ---------- */
        LCD_setCursor(0, 0);
        LCD_print("                "); // 라인 클리어
        LCD_setCursor(0, 0);
        for (uint8_t i = 0; i < 3; i++) {
            dtostrf(norm[i], 4, 2, normStr);
            sprintf(buf, "%d:%s ", i, normStr);
            LCD_print(buf);
        }

        LCD_setCursor(0, 1);
        LCD_print("                "); // 라인 클리어
        LCD_setCursor(0, 1);
        for (uint8_t i = 3; i < 6; i++) {
            dtostrf(norm[i], 4, 2, normStr);
            sprintf(buf, "%d:%s ", i, normStr);
            LCD_print(buf);
        }

        _delay_ms(200);
    }
}