#define F_CPU 16000000UL
#define I2C_ADDR 0x27 // 화면 안 뜨면 0x3F로 바꿔볼것

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// PCF8574 -> LCD 핀 매핑 (백팩 기본 배선)
// P0=RS, P1=RW, P2=EN, P3=백라이트, P4~P7=D4~D7
#define RS_BIT   0x01
#define RW_BIT   0x02
#define EN_BIT   0x04
#define BL_BIT   0x08 // 백라이트 항상 켜두려고 씀

// 스위치는 PD0/PD1(SCL,SDA)이 I2C가 먹고있어서 PD2,PD3으로 옮김
#define SW_PIN     PIND
#define SW_DDR     DDRD
#define SW1        2   // PD2 : 값 확정하고 다음 항목으로 넘어가는 버튼
#define SW2        3   // PD3 : 이거 누르면 시계 시작

// 시간값들 (인터럽트에서 계속 바뀜)
volatile uint16_t g_year = 2000; // 0~2999 그대로 저장 (2000년으로 시작값 잡음)
volatile uint8_t g_mon  = 1;
volatile uint8_t g_day  = 1;
volatile uint8_t g_hour = 0;
volatile uint8_t g_min  = 0;
volatile uint8_t g_sec  = 0;
volatile uint8_t g_cs   = 0;   // 1/100초 단위 (센티초)
volatile uint8_t g_running = 0; // 0이면 세팅중, 1이면 시계 진짜로 흐르는중

// 윤년 계산 (4로 나눠지고 100으로 안나눠지거나, 400으로 나눠지면 윤년)
static uint8_t is_leap_year(uint16_t y)
{
	return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

// 그 달이 며칠까지 있는지 (2월이고 윤년이면 29일까지)
static uint8_t days_in_month(uint8_t mon, uint16_t y)
{
	static const uint8_t dim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
	if (mon < 1) mon = 1;
	if (mon > 12) mon = 12;
	if (mon == 2 && is_leap_year(y))
	return 29;
	return dim[mon - 1];
}

// ------------------- I2C(TWI) 기본 함수 -------------------
void twi_init(void) {
	TWSR = 0x00;                          // 프리스케일러 1로
	TWBR = ((F_CPU / 100000UL) - 16) / 2; // SCL 100kHz 나오게 계산
	TWCR = (1 << TWEN);
}

void twi_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))); // 전송 끝날때까지 대기
}

void twi_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	_delay_us(50);
}

void twi_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

// PCF8574로 1바이트 보내기
void pcf8574_write(uint8_t data) {
	twi_start();
	twi_write(I2C_ADDR << 1); // 쓰기모드 주소
	twi_write(data | BL_BIT); // 백라이트 비트 항상 켜서 같이 보냄
	twi_stop();
}

// ------------------- LCD 4비트 통신 -------------------
void lcd_pulse_enable(uint8_t data) {
	pcf8574_write(data | EN_BIT);
	_delay_us(1);
	pcf8574_write(data & ~EN_BIT); // EN 내려서 데이터 래치
	_delay_us(50);
}

void lcd_send_nibble(uint8_t nibble, uint8_t rs) {
	uint8_t data = (nibble & 0xF0) | (rs ? RS_BIT : 0);
	pcf8574_write(data);
	lcd_pulse_enable(data);
}

void lcd_send_byte(uint8_t value, uint8_t rs) {
	lcd_send_nibble(value & 0xF0, rs);        // 상위 4비트 먼저
	lcd_send_nibble((value << 4) & 0xF0, rs); // 하위 4비트 나중
}

void lcd_command(uint8_t cmd) {
	lcd_send_byte(cmd, 0);
	_delay_ms(2);
}

void lcd_data(uint8_t data) {
	lcd_send_byte(data, 1);
	_delay_ms(1);
}

void lcd_string(const char *str) {
	while (*str) {
		lcd_data(*str++);
	}
}

// 숫자 두자리로 찍기 (07, 23 이런식)
static void lcd_print2(uint8_t val)
{
	if (val > 99) val = 99;
	lcd_data('0' + (val / 10));
	lcd_data('0' + (val % 10));
}

// 연도용, 네자리로 찍기 (앞자리 0 채워서 0007, 2024 이런식)
static void lcd_print4(uint16_t val)
{
	if (val > 2999) val = 2999;
	lcd_data('0' + (val / 1000) % 10);
	lcd_data('0' + (val / 100)  % 10);
	lcd_data('0' + (val / 10)   % 10);
	lcd_data('0' + (val)        % 10);
}

// LCD 4bit 모드 초기화 (데이터시트에 나오는 순서 그대로 따라간거)
void lcd_init(void) {
	_delay_ms(50); // 전원켜지고 좀 기다려줘야함

	lcd_send_nibble(0x30, 0);
	_delay_ms(5);
	lcd_send_nibble(0x30, 0);
	_delay_us(150);
	lcd_send_nibble(0x30, 0);
	_delay_us(150);
	lcd_send_nibble(0x20, 0); // 이제부터 4비트 모드로 쓰겠다는 뜻

	lcd_command(0x28); // 4bit, 2줄, 5x8폰트
	lcd_command(0x0C); // 화면 켜고 커서는 안보이게
	lcd_command(0x06); // 글자 쓰면 커서 오른쪽으로 이동
	lcd_command(0x01); // 화면 지우기
	_delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
	lcd_command(addr);
}

// ------------------- ADC (가변저항, PF0=ADC0) -------------------
static void adc_init(void)
{
	ADMUX  = (1 << REFS0); // 기준전압 AVCC, 채널0
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 분주비 128
}

static uint16_t adc_read(uint8_t ch)
{
	ADMUX = (ADMUX & 0xE0) | (ch & 0x1F);
	ADCSRA |= (1 << ADSC); // 변환 시작
	while (ADCSRA & (1 << ADSC)); // 끝날때까지 대기
	return ADC;
}

// ADC값(0~1023)을 원하는 범위(lo~hi)로 바꿔주는 함수
// 연도처럼 큰 범위(0~2999)도 되게 16비트로 계산함
static uint16_t adc_map(uint16_t adc, uint16_t lo, uint16_t hi)
{
	uint32_t range = (uint32_t)(hi - lo + 1);
	uint32_t val = lo + ((uint32_t)adc * range) / 1024;
	if (val > hi) val = hi; // 혹시 넘으면 최대값으로 clamp
	return (uint16_t)val;
}

// ------------------- 스위치 (디바운스, 뗄때까지 대기) -------------------
static uint8_t switch_pressed(uint8_t pin)
{
	if (!(SW_PIN & (1 << pin))) // 눌리면 LOW로 들어옴
	{
		_delay_ms(20); // 채터링 방지용으로 잠깐 대기
		if (!(SW_PIN & (1 << pin)))
		{
			while (!(SW_PIN & (1 << pin))); // 손 뗄때까지 계속 대기 (한번 누르면 한번만 처리되게)
			_delay_ms(20);
			return 1;
		}
	}
	return 0;
}

static void timer1_init(void)
{
	TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC모드, 분주비 64
	OCR1A = 2499; // 16,000,000 / 64 / 100Hz - 1 = 2499  -> 10ms마다 인터럽트
	TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
	if (!g_running) return; // 세팅중일땐 그냥 무시

	g_cs++;
	if (g_cs >= 100) // 1초 다 채움
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
					if (g_day > days_in_month(g_mon, g_year)) // 예외처리 : 그 달 마지막날 넘으면 다음달로
					{
						g_day = 1;
						g_mon++;
						if (g_mon > 12) // 예외처리 : 12월 넘으면 다음해로
						{
							g_mon = 1;
							g_year++;
							if (g_year > 2999) g_year = 0; // 2999 넘으면 0년으로 롤오버
						}
					}
				}
			}
		}
	}
}


// 실제 시계 돌아갈때 화면 (1행 : YYYYMMDD, 2행 : HH:MM:SS.CC)
static void display_datetime(void)
{
	uint16_t yy;
	uint8_t mo, dd, hh, mi, se, cs;

	// 인터럽트 도중에 값 튀는거 막으려고 잠깐 끄고 복사해옴
	cli();
	yy = g_year; mo = g_mon; dd = g_day;
	hh = g_hour; mi = g_min; se = g_sec; cs = g_cs;
	sei();

	lcd_set_cursor(0, 0);
	lcd_print4(yy);
	lcd_print2(mo);
	lcd_print2(dd);
	lcd_string("      "); // 남은 칸 지우려고 공백 채움

	lcd_set_cursor(1, 0);
	lcd_print2(hh); lcd_data(':');
	lcd_print2(mi); lcd_data(':');
	lcd_print2(se); lcd_data('.');
	lcd_print2(cs);
	lcd_string("    ");
}

// 세팅 모드일때, 지금 뭐 조정중인지 보여주는 화면
static void display_setting(uint8_t idx)
{
	const char *label[6] = {"YEAR", "MON ", "DAY ", "HOUR", "MIN ", "SEC "};

	lcd_set_cursor(0, 0);
	lcd_string("SET ");
	lcd_string(label[idx]);
	lcd_string(" :");
	switch (idx) // 지금 몇번째 항목 조정중인지에 따라 값 찍기
	{
		case 0: lcd_print4(g_year); break;
		case 1: lcd_print2(g_mon);  break;
		case 2: lcd_print2(g_day);  break;
		case 3: lcd_print2(g_hour); break;
		case 4: lcd_print2(g_min);  break;
		case 5: lcd_print2(g_sec);  break;
	}
	lcd_string("  ");

	// 2행엔 지금까지 정해진 날짜+시간 전체 요약 (16자리 딱 맞춤, 8+8)
	lcd_set_cursor(1, 0);
	lcd_print4(g_year); lcd_print2(g_mon); lcd_print2(g_day);
	lcd_print2(g_hour); lcd_data(':'); lcd_print2(g_min); lcd_data(':'); lcd_print2(g_sec);
}

int main(void)
{
	SW_DDR &= ~((1 << SW1) | (1 << SW2)); // PD2,PD3 입력으로
	PORTD |= (1 << SW1) | (1 << SW2); // 내부 풀업 걸어둠 (외부 풀업 있어도 상관없음)

	twi_init();
	lcd_init();
	adc_init();
	timer1_init();
	sei();

	lcd_command(0x01);
	_delay_ms(2);

	// 순서 : 연 -> 월 -> 일 -> 시 -> 분 -> 초
	uint8_t idx = 0;
	while (idx < 6)
	{
		uint16_t adc = adc_read(0); // 가변저항 값 읽기

		switch (idx)
		{
			case 0: g_year = adc_map(adc, 0, 2999); break;
			case 1: g_mon  = adc_map(adc, 1, 12); break;
			case 2:
			{
				uint8_t maxd = days_in_month(g_mon, g_year); // 예외처리 : 이미 정한 월/윤년 기준으로 최대 일수 잡기
				g_day = adc_map(adc, 1, maxd);
				break;
			}
			case 3: g_hour = adc_map(adc, 0, 23); break;
			case 4: g_min  = adc_map(adc, 0, 59); break;
			case 5: g_sec  = adc_map(adc, 0, 59); break;
		}

		display_setting(idx);

		if (switch_pressed(SW1)) // SW1 누르면 확정하고 다음 항목으로
		{
			idx++;
		}
		_delay_ms(30);
	}

	// 세팅 다 끝났으니 SW2 누를때까지 대기
	g_cs = 0;
	lcd_command(0x01);
	lcd_set_cursor(0, 0);
	lcd_string("SET DONE");
	lcd_set_cursor(1, 0);
	lcd_string("SW2:START");

	while (!switch_pressed(SW2));
	
	lcd_command(0x01);
	g_running = 1; // 이제부터 인터럽트에서 시간 실제로 흘러감

	while (1)
	{
		display_datetime();
		_delay_ms(30);
	}
}
