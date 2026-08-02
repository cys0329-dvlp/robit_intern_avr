/*==============================================================================
 *   2026 로봇게임단 신입생 교육 - MCU 과제 3 : PSD 기반 거리 측정 시스템
 *
 *   [동작]
 *     PF1(ADC1)의 PSD 센서 값을 일정 주기로 읽어서
 *     룩업테이블 + 선형보간으로 cm 단위 거리로 환산하고
 *     UART로 PC 시리얼 터미널에 출력한다.
 *
 *   [센서] SHARP GP2Y0A02YK0F  (각인 : 2Y0A02)
 *     측정범위 20 ~ 150cm
 *     ★ 20cm 미만에서는 출력전압이 거꾸로 떨어진다(비단조).
 *       즉 1.3V 라는 값 하나가 5cm 일 수도, 45cm 일 수도 있어서
 *       ADC값만으로는 구분이 불가능하다. 이 구간은 반드시 예외처리해야 한다.
 *
 *   [예외처리]
 *     1. ADC가 거의 0        -> 센서 미연결 / 배선 이상
 *     2. ADC > 20cm 지점     -> 20cm 미만. 값이 두 거리로 해석되므로 거리 계산 거부
 *     3. ADC < 150cm 지점    -> 측정범위 초과(너무 멀거나 물체 없음)
 *     4. 튀는 값(스파이크)   -> 5회 샘플의 중앙값(median)을 사용해 제거
 *
 *   [포트 할당]
 *     PF0 : 가변저항 (이번 과제에서는 사용 안 함)
 *     PF1 : PSD 센서        <- ADC1
 *     PF2 ~ PF7 : IR 센서 (이후 과제)
 *
 *   [UART]
 *     UART0 (PE0/PE1), 57600bps 8-N-1
 *     MAX485의 DE/RE(PE2)는 LOW로 유지해서 485 버스로 나가지 않게 한다.
 *==============================================================================*/

//------------------------------------------------------------------------------
//  F_CPU는 반드시 모든 #include 보다 위에 있어야 한다.
//------------------------------------------------------------------------------
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

//==============================================================================
//                                사용자 설정
//==============================================================================
#define PSD_ADC_CH          1           // PF1 = ADC1

//  측정 주기 / 출력 주기 (ms)
//    센서 응답시간이 약 38ms 이므로 그보다 너무 빠르게 읽어봐야 의미가 없다.
#define SAMPLE_PERIOD_MS    20          // 샘플 1회 취득 주기
#define PRINT_PERIOD_MS     200         // PC로 출력하는 주기

#define MEDIAN_N            5           // 중앙값 필터에 쓸 샘플 개수 (홀수)

//  UART
#define UART0_BAUD          57600UL
#define UBRR0_VAL           ((((F_CPU) + 4UL * (UART0_BAUD)) / (8UL * (UART0_BAUD))) - 1)

//  MAX485 방향제어 : LOW로 두면 485 버스로 송신되지 않는다
#define DIR_PIN             PE2

//  유효 판정 경계 (아래 룩업테이블의 양 끝과 같다)
#define ADC_DISCONNECT      30          // 이보다 작으면 센서 미연결로 간주
#define ADC_FAR_LIMIT       92          // 0.45V = 150cm. 이보다 작으면 범위 초과
#define ADC_NEAR_LIMIT      512         // 2.50V =  20cm. 이보다 크면 20cm 미만

//==============================================================================
//                        PSD 특성 룩업테이블
//   ADC값(오름차순) -> 거리(0.1cm 단위)
//   ADC가 커질수록 거리는 가까워진다.
//   0.1cm 단위 정수로 저장해서 부동소수점 연산을 피한다.
//==============================================================================
typedef struct {
    uint16_t adc;
    uint16_t dist;      // 0.1cm 단위 (1500 = 150.0cm)
} PsdPoint;

static const PsdPoint psd_lut[] PROGMEM = {
    {   92, 1500 },   //  0.45V  150cm
    {   98, 1400 },   //  0.48V  140cm
    {  106, 1300 },   //  0.52V  130cm
    {  115, 1200 },   //  0.56V  120cm
    {  123, 1100 },   //  0.60V  110cm
    {  133, 1000 },   //  0.65V  100cm
    {  147,  900 },   //  0.72V   90cm
    {  164,  800 },   //  0.80V   80cm
    {  180,  700 },   //  0.88V   70cm
    {  205,  600 },   //  1.00V   60cm
    {  235,  500 },   //  1.15V   50cm
    {  262,  450 },   //  1.28V   45cm
    {  297,  400 },   //  1.45V   40cm
    {  327,  350 },   //  1.60V   35cm
    {  368,  300 },   //  1.80V   30cm
    {  440,  250 },   //  2.15V   25cm
    {  512,  200 },   //  2.50V   20cm
};

#define LUT_SIZE  (sizeof(psd_lut) / sizeof(psd_lut[0]))

//==============================================================================
//                                전역 변수
//==============================================================================
static volatile uint16_t g_ms = 0;          // 1ms 카운터 (타이머가 증가)
static volatile uint8_t  g_do_sample = 0;   // 샘플 취득 시점 신호
static volatile uint8_t  g_do_print  = 0;   // 출력 시점 신호

//  측정 결과 상태
enum {
    PSD_OK = 0,
    PSD_TOO_CLOSE,      // 20cm 미만 (값이 두 거리로 해석됨)
    PSD_TOO_FAR,        // 150cm 초과 또는 물체 없음
    PSD_DISCONNECTED    // 센서 미연결
};

//==============================================================================
//                                  UART0
//==============================================================================
static void uart0_init(void)
{
    DDRE |=  (1 << PE1);        // TXD0 출력
    DDRE &= ~(1 << PE0);        // RXD0 입력
    PORTE |= (1 << PE0);        // 풀업 (플로팅 방지)

    DDRE  |=  (1 << DIR_PIN);   // MAX485 방향제어 출력
    PORTE &= ~(1 << DIR_PIN);   // LOW = 485 송신 비활성 (PC로만 나간다)

    UBRR0H = (uint8_t)(UBRR0_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR0_VAL);

    UCSR0A = (1 << U2X0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // 8-N-1
    UCSR0B = (1 << TXEN0);                      // 송신만 사용
}

static void uart0_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

static void uart0_puts(const char *s)
{
    while (*s) uart0_putchar(*s++);
}

//==============================================================================
//                                   ADC
//==============================================================================
static void adc_init(void)
{
    // PF1을 입력으로, 내부 풀업 OFF
    DDRF  &= ~(1 << PF1);
    PORTF &= ~(1 << PF1);

    ADMUX  = (1 << REFS0) | (PSD_ADC_CH & 0x1F);        // AVCC 기준, ADC1
    ADCSRA = (1 << ADEN)
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128분주 -> 125kHz
}

static uint16_t adc_read(void)
{
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADCW;
}

//==============================================================================
//                        중앙값(median) 필터
//   PSD 센서는 순간적으로 값이 크게 튀는 특성이 있다.
//   평균을 쓰면 튄 값이 결과에 섞이지만, 중앙값은 아예 무시된다.
//==============================================================================
static uint16_t median_of(uint16_t *src, uint8_t n)
{
    uint16_t buf[MEDIAN_N];
    uint8_t  i, j;
    uint16_t key;

    for (i = 0; i < n; i++) buf[i] = src[i];

    // 삽입정렬 (n이 5개뿐이라 이걸로 충분하다)
    for (i = 1; i < n; i++)
    {
        key = buf[i];
        j = i;
        while (j > 0 && buf[j - 1] > key) { buf[j] = buf[j - 1]; j--; }
        buf[j] = key;
    }

    return buf[n / 2];
}

//==============================================================================
//                        ADC -> 거리 환산 (선형보간)
//   반환 : 상태값 (PSD_OK 등)
//   dist : 0.1cm 단위 거리 (PSD_OK 일 때만 유효)
//==============================================================================
static uint8_t psd_adc_to_dist(uint16_t adc, uint16_t *dist)
{
    uint8_t  i;
    uint16_t a0, a1, d0, d1;

    *dist = 0;

    // ---- 예외 1 : 센서가 아예 응답하지 않음 ----
    if (adc < ADC_DISCONNECT) return PSD_DISCONNECTED;

    // ---- 예외 2 : 측정범위보다 멀거나 물체가 없음 ----
    if (adc < ADC_FAR_LIMIT)  return PSD_TOO_FAR;

    // ---- 예외 3 : 20cm 미만 ----
    //   이 구간은 전압이 거꾸로 떨어져서 같은 ADC값이 두 거리를 가리킨다.
    //   추정 자체가 불가능하므로 거리값을 내놓지 않는다.
    if (adc > ADC_NEAR_LIMIT) return PSD_TOO_CLOSE;

    // ---- 정상 : 테이블에서 구간을 찾아 선형보간 ----
    for (i = 0; i < LUT_SIZE - 1; i++)
    {
        a0 = pgm_read_word(&psd_lut[i].adc);
        a1 = pgm_read_word(&psd_lut[i + 1].adc);

        if (adc >= a0 && adc <= a1)
        {
            d0 = pgm_read_word(&psd_lut[i].dist);
            d1 = pgm_read_word(&psd_lut[i + 1].dist);

            // 거리는 ADC가 커질수록 작아지므로 d0 > d1 이다
            //   dist = d0 - (d0 - d1) * (adc - a0) / (a1 - a0)
            *dist = (uint16_t)(d0 - (uint16_t)(((uint32_t)(d0 - d1) * (adc - a0))
                                               / (a1 - a0)));
            return PSD_OK;
        }
    }

    return PSD_TOO_FAR;     // 여기 오면 안 되지만 방어 코드
}

//==============================================================================
//                        Timer1 : 1ms 주기 인터럽트
//   CTC 모드, 분주비 64
//   16,000,000 / 64 = 250,000Hz  ->  250,000 / 1000Hz = 250  ->  OCR1A = 249
//==============================================================================
static void timer1_init(void)
{
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A  = 249;                       // 1ms
    TCNT1  = 0;
    TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
    static uint16_t sample_cnt = 0;
    static uint16_t print_cnt  = 0;

    g_ms++;

    if (++sample_cnt >= SAMPLE_PERIOD_MS) { sample_cnt = 0; g_do_sample = 1; }
    if (++print_cnt  >= PRINT_PERIOD_MS)  { print_cnt  = 0; g_do_print  = 1; }
}

//==============================================================================
int main(void)
{
	char     line[48];
	uint16_t samples[MEDIAN_N];
	uint8_t  idx    = 0;
	uint8_t  filled = 0;
	uint16_t adc, dist;
	uint8_t  status;

	uart0_init();
	adc_init();
	timer1_init();
	sei();

	_delay_ms(50);              // 센서 기동 대기
	adc_read();                 // 첫 변환은 버린다

	uart0_puts("\r\n===== PSD Distance Meter =====\r\n");
	uart0_puts("Sensor : GP2Y0A02YK0F (20-150cm)\r\n");
	sprintf(line, "Period : sample %dms / print %dms\r\n",
	SAMPLE_PERIOD_MS, PRINT_PERIOD_MS);
	uart0_puts(line);
	uart0_puts("==============================\r\n");

	while (1)
	{
		//----------------------------------------------------------------------
		//  주기적으로 샘플을 모은다
		//----------------------------------------------------------------------
		if (g_do_sample)
		{
			g_do_sample = 0;

			samples[idx] = adc_read();
			idx = (uint8_t)((idx + 1) % MEDIAN_N);

			if (filled < MEDIAN_N) filled++;
		}

		//----------------------------------------------------------------------
		//  주기적으로 결과를 출력한다
		//----------------------------------------------------------------------
		if (g_do_print)
		{
			g_do_print = 0;

			if (filled < MEDIAN_N) continue;        // 아직 샘플이 덜 모임

			adc    = median_of(samples, MEDIAN_N);  // 스파이크 제거
			status = psd_adc_to_dist(adc, &dist);

			switch (status)
			{
				case PSD_OK:
				sprintf(line, "ADC:%4u  Distance: %3u.%1u cm\r\n",
				adc, (uint16_t)(dist / 10), (uint16_t)(dist % 10));
				break;

				case PSD_TOO_CLOSE:
				sprintf(line, "ADC:%4u  [WARN] too close (<20cm)\r\n", adc);
				break;

				case PSD_TOO_FAR:
				sprintf(line, "ADC:%4u  [WARN] out of range / no object\r\n", adc);
				break;

				case PSD_DISCONNECTED:
				default:
				sprintf(line, "ADC:%4u  [ERROR] sensor not connected\r\n", adc);
				break;
			}

			uart0_puts(line);
		}
	}

	return 0;
}