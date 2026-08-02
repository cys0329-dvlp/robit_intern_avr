#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>

/*
	PSD 센서(GP2Y0A02YK0F)로 거리 재서 UART로 출력하는 코드
	20cm 미만에서는 전압이 거꾸로 떨어져서 하나의 ADC값이 두 거리를 가리킴
	-> 이 구간은 계산을 포기하고 경고만 띄움
	5개 샘플 중앙값으로 스파이크(순간 튀는 값) 제거

	[과제 반영]
	필터링 미적용 원시데이터(RAW)와 중앙값 필터를 적용한 데이터(FILTERED)를
	같은 줄에 동시에 UART로 출력한다.
	예) RAW: 412 | FILTERED: 405 | DISTANCE: 15.2cm

	포트 : PF1 = PSD 센서(ADC1), UART0(PE0/PE1) 57600bps
	MAX485 DIR핀(PE2)은 LOW로 고정해서 485버스로는 안 나가게 함
*/

#define PSD_ADC_CH        1      //PF1 = ADC1

#define SAMPLE_PERIOD_MS  20     //샘플 취득 주기, 센서 응답속도가 38ms라 이보다 빠르게 읽어도 의미없음
#define PRINT_PERIOD_MS   200    //UART 출력 주기

#define MEDIAN_N          5      //중앙값 필터에 쓸 샘플 개수(홀수)

#define UART0_BAUD        57600
#define UBRR0_VAL         ((F_CPU / (8 * UART0_BAUD)) - 1)

#define DIR_PIN           PE2    //MAX485 방향제어

//유효 판정 경계값 (아래 룩업테이블 양끝값과 같음)
#define ADC_DISCONNECT    30     //이보다 작으면 센서 미연결
#define ADC_FAR_LIMIT     92     //0.45V = 150cm, 이보다 작으면 범위초과
#define ADC_NEAR_LIMIT    512    //2.50V = 20cm, 이보다 크면 20cm 미만(계산 불가 구간)

//PSD 특성 룩업테이블 : ADC값 -> 거리(0.1cm 단위), 오름차순
//부동소수점 안쓰려고 0.1cm 단위 정수로 저장
typedef struct
{
	unsigned int adc;
	unsigned int dist; //0.1cm 단위 (1500 = 150.0cm)
}PsdPoint;

const PsdPoint psd_lut[] PROGMEM =
{
	{   92, 1500 },  //0.45V 150cm
	{   98, 1400 },  //0.48V 140cm
	{  106, 1300 },  //0.52V 130cm
	{  115, 1200 },  //0.56V 120cm
	{  123, 1100 },  //0.60V 110cm
	{  133, 1000 },  //0.65V 100cm
	{  147,  900 },  //0.72V  90cm
	{  164,  800 },  //0.80V  80cm
	{  180,  700 },  //0.88V  70cm
	{  205,  600 },  //1.00V  60cm
	{  235,  500 },  //1.15V  50cm
	{  262,  450 },  //1.28V  45cm
	{  297,  400 },  //1.45V  40cm
	{  327,  350 },  //1.60V  35cm
	{  368,  300 },  //1.80V  30cm
	{  440,  250 },  //2.15V  25cm
	{  512,  200 },  //2.50V  20cm
};

#define LUT_SIZE (sizeof(psd_lut) / sizeof(psd_lut[0]))

//거리 계산 결과 상태
enum
{
	PSD_OK = 0,
	PSD_TOO_CLOSE,     //20cm 미만 (값 두개가 겹치는 구간)
	PSD_TOO_FAR,       //150cm 초과 또는 물체 없음
	PSD_DISCONNECTED   //센서 미연결
};

volatile unsigned int g_ms = 0;         //1ms마다 증가하는 카운터(타이머에서 증가시킴)
volatile unsigned char g_do_sample = 0; //샘플 뽑을 타이밍 신호
volatile unsigned char g_do_print  = 0; //출력할 타이밍 신호

unsigned char Uart0_Getch(void);
void Uart0_Putch(unsigned char PutData);
void UART_transmit_string(char *str);

void Uart0_Init(void)
{
	DDRE |= (1 << PE1);        //TXD0 출력
	DDRE &= ~(1 << PE0);       //RXD0 입력
	PORTE |= (1 << PE0);       //풀업(플로팅 방지)
	
	DDRE |= (1 << DIR_PIN);    //MAX485 방향제어 출력
	PORTE &= ~(1 << DIR_PIN);  //LOW = 485 송신 비활성(PC로만 나감)
	
	UBRR0H = (unsigned char)(UBRR0_VAL >> 8);
	UBRR0L = (unsigned char)(UBRR0_VAL);
	
	UCSR0A = (1 << U2X0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  //8비트 / 패리티없음 / STOP 1비트
	UCSR0B = (1 << TXEN0);                   //송신만 사용
}

unsigned char Uart0_Getch(void)
{
	while(!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void Uart0_Putch(unsigned char PutData)
{
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = PutData;
}

void UART_transmit_string(char *str)
{
	while(*str != '\0')
	{
		Uart0_Putch(*str);
		str++;
	}
}

void Adc_Init(void)
{
	DDRF &= ~(1 << PF1);   //PF1 입력
	PORTF &= ~(1 << PF1);  //내부 풀업 OFF
	
	ADMUX = (1 << REFS0) | (PSD_ADC_CH & 0X1F);  //AVCC 기준, ADC1
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  //128분주 -> 125kHz
}

unsigned int Adc_Read(void)
{
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	return ADCW;
}

//PSD가 순간적으로 값이 크게 튀어서(스파이크) 평균 대신 중앙값을 씀
//평균은 튄 값이 섞이지만 중앙값은 그냥 무시됨
unsigned int Median_Of(unsigned int *src, unsigned char n)
{
	unsigned int buf[MEDIAN_N];
	unsigned char i, j;
	unsigned int key;
	
	for(i = 0; i < n; i++) buf[i] = src[i];
	
	//삽입정렬, 어차피 5개뿐이라 이걸로 충분함
	for(i = 1; i < n; i++)
	{
		key = buf[i];
		j = i;
		while(j > 0 && buf[j-1] > key)
		{
			buf[j] = buf[j-1];
			j--;
		}
		buf[j] = key;
	}
	
	return buf[n/2];
}

//ADC값을 거리로 환산(선형보간), 반환값은 상태(PSD_OK 등), dist는 PSD_OK일때만 유효
unsigned char Psd_AdcToDist(unsigned int adc, unsigned int *dist)
{
	unsigned char i;
	unsigned int a0, a1, d0, d1;
	
	*dist = 0;
	
	if(adc < ADC_DISCONNECT) return PSD_DISCONNECTED;  //센서가 응답 자체를 안함
	
	if(adc < ADC_FAR_LIMIT) return PSD_TOO_FAR;  //너무 멀거나 물체가 없음
	
	//20cm 미만 구간은 전압이 거꾸로 떨어져서 같은 ADC값이 두 거리를 가리킴
	//구분이 안되니까 계산을 포기함
	if(adc > ADC_NEAR_LIMIT) return PSD_TOO_CLOSE;
	
	//테이블에서 구간 찾아서 보간
	for(i = 0; i < LUT_SIZE - 1; i++)
	{
		a0 = pgm_read_word(&psd_lut[i].adc);
		a1 = pgm_read_word(&psd_lut[i+1].adc);
		
		if(adc >= a0 && adc <= a1)
		{
			d0 = pgm_read_word(&psd_lut[i].dist);
			d1 = pgm_read_word(&psd_lut[i+1].dist);
			
			//ADC가 커질수록 거리는 작아지므로 d0 > d1
			*dist = (unsigned int)(d0 - (unsigned int)(((unsigned long)(d0-d1) * (adc-a0)) / (a1-a0)));
			return PSD_OK;
		}
	}
	
	return PSD_TOO_FAR;  //여기 오면 안되지만 방어코드
}

void Timer1_Init(void)  //1ms마다 인터럽트, CTC모드, 분주비 64
{
	TCCR1A = 0X00;
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A = 249;  //16000000/64/1000 - 1 = 249
	TCNT1 = 0;
	TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
	static unsigned int sample_cnt = 0;
	static unsigned int print_cnt = 0;
	
	g_ms++;
	
	if(++sample_cnt >= SAMPLE_PERIOD_MS)
	{
		sample_cnt = 0;
		g_do_sample = 1;
	}
	if(++print_cnt >= PRINT_PERIOD_MS)
	{
		print_cnt = 0;
		g_do_print = 1;
	}
}

int main(void)
{
	char line[64];
	unsigned int samples[MEDIAN_N];
	unsigned char idx = 0;
	unsigned char filled = 0;
	unsigned int raw_adc, filtered_adc, dist;
	unsigned char status;
	
	Uart0_Init();
	Adc_Init();
	Timer1_Init();
	
	sei();
	
	_delay_ms(50);  //센서 기동 대기
	Adc_Read();     //첫 변환은 버림
	
	UART_transmit_string("\r\n===== PSD Distance Meter =====\r\n");
	UART_transmit_string("Sensor : GP2Y0A02YK0F (20-150cm)\r\n");
	UART_transmit_string("Filter : Median filter (N=5)\r\n");
	sprintf(line, "Period : sample %dms / print %dms\r\n", SAMPLE_PERIOD_MS, PRINT_PERIOD_MS);
	UART_transmit_string(line);
	UART_transmit_string("==============================\r\n");
	
	while(1)
	{
		//주기적으로 샘플 모으기 (raw 데이터를 그대로 버퍼에 저장)
		if(g_do_sample)
		{
			g_do_sample = 0;
			
			raw_adc = Adc_Read();               //필터를 거치지 않은 원시값
			samples[idx] = raw_adc;
			idx = (unsigned char)((idx+1) % MEDIAN_N);
			
			if(filled < MEDIAN_N) filled++;
		}
		
		//주기적으로 결과 출력
		if(g_do_print)
		{
			g_do_print = 0;
			
			if(filled < MEDIAN_N) continue;  //아직 샘플이 다 안모임
			
			//가장 최근에 읽은 raw 샘플 (필터 적용 전)
			raw_adc = samples[(unsigned char)((idx + MEDIAN_N - 1) % MEDIAN_N)];
			
			//5개 샘플에 중앙값 필터를 적용한 값 (필터 적용 후)
			filtered_adc = Median_Of(samples, MEDIAN_N);
			
			//거리 계산은 스파이크가 제거된 filtered 값을 기준으로 함
			status = Psd_AdcToDist(filtered_adc, &dist);
			
			switch(status)
			{
				case PSD_OK:
					sprintf(line, "RAW:%4u | FILTERED:%4u | DISTANCE: %3u.%1u cm\r\n",
							raw_adc, filtered_adc, (unsigned int)(dist/10), (unsigned int)(dist%10));
					break;
				case PSD_TOO_CLOSE:
					sprintf(line, "RAW:%4u | FILTERED:%4u | [WARN] too close (<20cm)\r\n",
							raw_adc, filtered_adc);
					break;
				case PSD_TOO_FAR:
					sprintf(line, "RAW:%4u | FILTERED:%4u | [WARN] out of range / no object\r\n",
							raw_adc, filtered_adc);
					break;
				case PSD_DISCONNECTED:
				default:
					sprintf(line, "RAW:%4u | FILTERED:%4u | [ERROR] sensor not connected\r\n",
							raw_adc, filtered_adc);
					break;
			}
			
			UART_transmit_string(line);
		}
	}
}