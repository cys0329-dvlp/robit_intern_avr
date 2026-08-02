/*
 * ============================================================
 *  과제 2 : 달력, 시계 만들기  (ATmega128 / Atmel Studio / avr-gcc)
 * ============================================================
 *  - 가변저항(PF0, ADC0) 값 + SW1(PD0) 으로 날짜/시간 세팅
 *      (가변저항 값에 따라 값이 바뀌고 -> SW1 눌러서 확정 -> 다음 항목)
 *      순서 : 연도 -> 월 -> 일 -> 시 -> 분 -> 초
 *  - LCD(16x2, 4bit, PC0=RS, PC1=RW, PC2=E, PC4~PC7=D4~D7)에
 *      "YYMMDD"  / "HH:MM:SS.CC"  형태로 출력
 *      (ex : 190722 / 10:50:48.34   , CC = 1/100초)
 *  - SW2(PD1) 누르면 시간이 흐르기 시작 (Timer1, 10ms 인터럽트)
 *  - 월별 일수, 윤년, 초/분/시/일/월/연 캐리(자리올림) 예외처리 포함
 *
 *  회로 매핑 (업로드된 회로도 기준)
 *   PF0        : 가변저항 신호 (ADC0)
 *   PD0        : SW1 (세팅용, 확정/다음항목)
 *   PD1        : SW2 (시계 시작)
 *   PC0/1/2    : LCD RS/RW/E
 *   PC4~PC7    : LCD D4~D7 (4bit 모드)
 *   PA0~PA7    : LED (이번 과제에서는 사용하지 않음, 필요시 확장)
 * ============================================================
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* ---------------- LCD 핀 정의 (PORTC) ---------------- */
#define LCD_PORT   PORTC
#define LCD_DDR    DDRC
#define LCD_RS     0   // PC0
#define LCD_RW     1   // PC1
#define LCD_EN     2   // PC2
// 데이터라인 D4~D7 : PC4~PC7 (상위 4비트)

/* ---------------- 스위치 핀 정의 (PORTD) ---------------- */
#define SW_PIN     PIND
#define SW_DDR     DDRD
#define SW1        0   // PD0 : 세팅 확정 / 다음 항목
#define SW2        1   // PD1 : 시계 시작

/* ---------------- 전역 시간 변수 (ISR에서 갱신) ---------------- */
volatile uint8_t g_year = 0;   // 00~99  (20YY)
volatile uint8_t g_mon  = 1;   // 1~12
volatile uint8_t g_day  = 1;   // 1~31
volatile uint8_t g_hour = 0;   // 0~23
volatile uint8_t g_min  = 0;   // 0~59
volatile uint8_t g_sec  = 0;   // 0~59
volatile uint8_t g_cs   = 0;   // 0~99 (센티초, 1/100초)
volatile uint8_t g_running = 0; // 0=정지/세팅중, 1=시간 흐름

/* ================================================================
 *  달/윤년 관련 함수
 * ================================================================ */
static uint8_t is_leap_year(uint8_t yy)
{
    uint16_t y = 2000 + yy;
    return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

static uint8_t days_in_month(uint8_t mon, uint8_t yy)
{
    static const uint8_t dim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (mon < 1) mon = 1;
    if (mon > 12) mon = 12;
    if (mon == 2 && is_leap_year(yy))
        return 29;
    return dim[mon - 1];
}

/* ================================================================
 *  LCD 4bit 드라이버
 * ================================================================ */
static void lcd_pulse_enable(void)
{
    LCD_PORT |= (1 << LCD_EN);
    _delay_us(1);
    LCD_PORT &= ~(1 << LCD_EN);
    _delay_us(100);
}

static void lcd_write4(uint8_t nibble)
{
    LCD_PORT = (LCD_PORT & 0x0F) | (nibble & 0xF0);
    lcd_pulse_enable();
}

static void lcd_send(uint8_t value, uint8_t is_data)
{
    if (is_data) LCD_PORT |= (1 << LCD_RS);
    else         LCD_PORT &= ~(1 << LCD_RS);

    LCD_PORT &= ~(1 << LCD_RW); // 항상 쓰기 모드

    lcd_write4(value & 0xF0);          // 상위 니블
    lcd_write4((value << 4) & 0xF0);   // 하위 니블
    _delay_us(50);
}

static void lcd_command(uint8_t cmd) { lcd_send(cmd, 0); }
static void lcd_data(uint8_t dat)    { lcd_send(dat, 1); }

static void lcd_init(void)
{
    LCD_DDR = 0xFF;      // PC0~PC7 전부 출력
    _delay_ms(50);

    LCD_PORT &= ~(1 << LCD_RS);
    LCD_PORT &= ~(1 << LCD_RW);

    lcd_write4(0x30); _delay_ms(5);
    lcd_write4(0x30); _delay_us(150);
    lcd_write4(0x30); _delay_us(150);
    lcd_write4(0x20); _delay_us(150);   // 4bit mode

    lcd_command(0x28); // 4bit, 2 line, 5x8 font
    lcd_command(0x0C); // display on, cursor off
    lcd_command(0x06); // entry mode : increment
    lcd_command(0x01); // clear
    _delay_ms(2);
}

static void lcd_goto(uint8_t col, uint8_t row)
{
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_command(0x80 | addr);
}

static void lcd_string(const char *s)
{
    while (*s) lcd_data(*s++);
}

static void lcd_print2(uint8_t val) // 0~99 -> 두자리
{
    if (val > 99) val = 99;
    lcd_data('0' + (val / 10));
    lcd_data('0' + (val % 10));
}

/* ================================================================
 *  ADC (가변저항, PF0 = ADC0)
 * ================================================================ */
static void adc_init(void)
{
    ADMUX  = (1 << REFS0);              // AVCC 기준, 채널 0
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 분주비 128
}

static uint16_t adc_read(uint8_t ch)
{
    ADMUX = (ADMUX & 0xE0) | (ch & 0x1F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// ADC 값(0~1023)을 [lo, hi] 범위로 매핑
static uint8_t adc_map(uint16_t adc, uint8_t lo, uint8_t hi)
{
    uint16_t range = (uint16_t)(hi - lo + 1);
    uint16_t val = lo + (uint16_t)(((uint32_t)adc * range) / 1024);
    if (val > hi) val = hi;
    return (uint8_t)val;
}

/* ================================================================
 *  스위치 (디바운스 포함, 눌렀다 뗄 때까지 대기)
 * ================================================================ */
static uint8_t switch_pressed(uint8_t pin)
{
    if (!(SW_PIN & (1 << pin)))          // LOW = 눌림
    {
        _delay_ms(20);                   // 디바운스
        if (!(SW_PIN & (1 << pin)))
        {
            while (!(SW_PIN & (1 << pin))); // 뗄 때까지 대기
            _delay_ms(20);
            return 1;
        }
    }
    return 0;
}

/* ================================================================
 *  Timer1 : 10ms 마다 인터럽트 (100Hz) -> 센티초 카운트
 * ================================================================ */
static void timer1_init(void)
{
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, prescaler 64
    OCR1A = 2499;              // 16MHz/64/100Hz - 1 = 2499  -> 10ms
    TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
    if (!g_running) return;

    g_cs++;
    if (g_cs >= 100)
    {
        g_cs = 0;
        g_sec++;
        if (g_sec >= 60)
        {
            g_sec = 0;
            g_min++;
            if (g_min >= 60)
            {
                g_min = 0;
                g_hour++;
                if (g_hour >= 24)
                {
                    g_hour = 0;
                    g_day++;
                    if (g_day > days_in_month(g_mon, g_year))
                    {
                        g_day = 1;
                        g_mon++;
                        if (g_mon > 12)
                        {
                            g_mon = 1;
                            g_year++;
                            if (g_year > 99) g_year = 0;
                        }
                    }
                }
            }
        }
    }
}

/* ================================================================
 *  화면 출력
 * ================================================================ */
static void display_datetime(void)
{
    uint8_t yy, mo, dd, hh, mi, se, cs;

    cli();
    yy = g_year; mo = g_mon; dd = g_day;
    hh = g_hour; mi = g_min; se = g_sec; cs = g_cs;
    sei();

    lcd_goto(0, 0);
    lcd_print2(yy);
    lcd_print2(mo);
    lcd_print2(dd);
    lcd_string("        "); // 남은 자리 지우기

    lcd_goto(0, 1);
    lcd_print2(hh); lcd_data(':');
    lcd_print2(mi); lcd_data(':');
    lcd_print2(se); lcd_data('.');
    lcd_print2(cs);
    lcd_string("    ");
}

// 세팅 모드에서 현재 조정중인 항목을 보여줌
static void display_setting(uint8_t idx)
{
    const char *label[6] = {"YEAR", "MON ", "DAY ", "HOUR", "MIN ", "SEC "};

    lcd_goto(0, 0);
    lcd_string("SET ");
    lcd_string(label[idx]);
    lcd_string(" :");
    switch (idx)
    {
        case 0: lcd_print2(g_year); break;
        case 1: lcd_print2(g_mon);  break;
        case 2: lcd_print2(g_day);  break;
        case 3: lcd_print2(g_hour); break;
        case 4: lcd_print2(g_min);  break;
        case 5: lcd_print2(g_sec);  break;
    }
    lcd_string("   ");

    lcd_goto(0, 1);
    lcd_print2(g_year); lcd_print2(g_mon); lcd_print2(g_day);
    lcd_data(' ');
    lcd_print2(g_hour); lcd_data(':'); lcd_print2(g_min); lcd_data(':'); lcd_print2(g_sec);
}

/* ================================================================
 *  MAIN
 * ================================================================ */
int main(void)
{
    SW_DDR &= ~((1 << SW1) | (1 << SW2)); // PD0, PD1 입력 (회로에 풀업 이미 존재)

    lcd_init();
    adc_init();
    timer1_init();
    sei();

    lcd_command(0x01);
    _delay_ms(2);

    /* ---------------- 1) 날짜/시간 세팅 단계 ---------------- */
    uint8_t idx = 0; // 0:연 1:월 2:일 3:시 4:분 5:초
    while (idx < 6)
    {
        uint16_t adc = adc_read(0);

        switch (idx)
        {
            case 0: g_year = adc_map(adc, 0, 99); break;
            case 1: g_mon  = adc_map(adc, 1, 12); break;
            case 2:
            {
                uint8_t maxd = days_in_month(g_mon, g_year); // 예외처리: 월/윤년에 맞는 최대일
                g_day = adc_map(adc, 1, maxd);
                break;
            }
            case 3: g_hour = adc_map(adc, 0, 23); break;
            case 4: g_min  = adc_map(adc, 0, 59); break;
            case 5: g_sec  = adc_map(adc, 0, 59); break;
        }

        display_setting(idx);

        if (switch_pressed(SW1))
        {
            idx++;
        }
        _delay_ms(30);
    }

    /* 세팅 완료, SW2 대기 */
    g_cs = 0;
    lcd_command(0x01);
    lcd_goto(0, 0);
    lcd_string("SET DONE");
    lcd_goto(0, 1);
    lcd_string("SW2:START");

    while (!switch_pressed(SW2));

    /* ---------------- 2) 시계 동작 단계 ---------------- */
    lcd_command(0x01);
    g_running = 1;

    while (1)
    {
        display_datetime();
        _delay_ms(30);
    }
}