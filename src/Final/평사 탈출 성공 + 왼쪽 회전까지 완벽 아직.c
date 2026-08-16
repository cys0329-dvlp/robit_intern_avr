#define F_CPU 16000000UL
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD) - 1)

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>


// ============================================================
// 모터 설정
// ============================================================

#define TOP_VALUE       256
#define BASE_SPEED      0.4

#define KICKSTART_DUTY  256
#define KICKSTART_MS    40


// ============================================================
// 함수 선언
// ============================================================

void System_Init(void);
void ADC_init(void);
void UART1_init(unsigned int ubrr);
void Motor_init(void);

// 버튼
int Button_Edge(void);
void Button_Process(void);

// 상태별 처리
void Raw_Process(void);
void Calibration_Process(void);
void Driving_Process(void);

// 센서
int ADC_read(int channel);
void Sensor_Update(void);
void ADC_Fix(void);

// 출력
void LED_Update(void);

// 모터
void Motor_SetSpeed(int left_duty, int right_duty);

// 주행
void SLine_Update(void);
void Kickstart_Process(void);
void Line8_Update(void);
void Parallelogram_Update(void);
void Bar_Update(void);

// 기타
void UART1_print(const char *str);


// ============================================================
// 전역 변수
// ============================================================

char buf[32];

// ============================================================
// 평행사변형 좌/우 끝단 번갈이 카운트 (코너 탈출용)
// ============================================================

int LR_Edge_Count = 0;   // 좌/우 끝 LED가 번갈아 켜진 횟수
int LR_Last_Side  = 0;   // 0 = 없음, 1 = 왼쪽, 2 = 오른쪽ㅍ

int Parallelogram_TurnDone = 0;   // state2 진입 시 1회 회전 완료 여부
// ============================================================
// ADC 데이터
// ============================================================

int raw[6];

int filtered[6];

int raw_max[6] = {
	0, 0, 0, 0, 0, 0
};

int raw_min[6] = {
	1023, 1023, 1023,
	1023, 1023, 1023
};


// ============================================================
// 시스템 상태
// ============================================================

// 0 = Raw
// 1 = Calibration
// 2 = Line Trace

int state = 0;

int button_last = 1;


// ============================================================
// 센서 상태
// ============================================================

int on_line[6];

const int weight[6] = {
	-40, -10, -5,
	5, 10, 40
};


// ============================================================
// 마지막 주행 PWM
// ============================================================

int last_left_duty = 0;
int last_right_duty = 0;


// ============================================================
// 킥스타트
// ============================================================

int kickstart_pending = 1;


// ============================================================
// 8자 라인트레이싱 상태
// ============================================================

// 0 = 아직 8자 진입 전
// 1 = 첫 번째 6센서 구간 -> 오른쪽 회전
// 2 = 첫 번째 회전 완료 -> 일반 라인 추적
// 3 = 두 번째 6센서 구간 -> 직진
// 4 = 두 번째 직진 완료 -> 일반 라인 추적
// 5 = 세 번째 6센서 구간 -> 정지
// 6 = 주행 종료

int Line8_State = 0; //아무것도 안하는 상태


// ============================================================
// 6개 ON 구간 이탈 확인
// ============================================================

int Line8_LeaveAllOn = 0;


// ============================================================
// 평행사변형
// ============================================================

int Parallelogram_state = 0;
int LineFind_EdgeCount = 0;
int LineFind_Last = 0;

// ============================================================
// Bar(PSD) 관련 상태
// ============================================================

int Bar_Trigger = 0;   // State 3 -> 4 전환 시 1로 설정되는 활성화 플래그
int psd_value   = 0;   // PSD 센서 Raw 값 (PF0 / ADC0)


// ============================================================
// MAIN
// ============================================================

int main(void)
{
	System_Init();

	while (1)
	{
		Button_Process();

		switch (state)
		{
			case 0:
			Raw_Process();
			break;

			case 1:
			Calibration_Process();
			break;

			case 2:
			Driving_Process();
			break;
		}
	}
}


// ============================================================
// 시스템 초기화
// ============================================================

void System_Init(void)
{
	// PE5 버튼 입력 + 내부 풀업
	DDRE &= ~(1 << PE5);
	PORTE |= (1 << PE5);


	// PORTA LED 출력
	DDRA = 0xFF;
	PORTA = 0xFF;


	// UART
	UART1_init(UBRR_VALUE);

	// ADC
	ADC_init();

	// Motor
	Motor_init();
}


// ============================================================
// 버튼 처리
// ============================================================

void Button_Process(void)
{
	if (!Button_Edge())
	return;


	state = (state + 1) % 3;


	// ========================================================
	// Calibration
	// ========================================================

	if (state == 1)
	{
		for (int i = 0; i < 6; i++)
		{
			raw_max[i] = 0;
			raw_min[i] = 1023;
		}

		Motor_SetSpeed(0, 0);

		UART1_print("Calibration start\r\n");
	}


	// ========================================================
	// Line Trace 시작
	// ========================================================

	else if (state == 2)
	{
		UART1_print("Calibration complete\r\n");

		for (int i = 0; i < 6; i++)
		{
			sprintf(
			buf,
			"%d:%d~%d ",
			i,
			raw_min[i],
			raw_max[i]
			);

			UART1_print(buf);
		}

		UART1_print("\r\n");


		// 라인트레이싱 시작 시 킥스타트 허용
		kickstart_pending = 1;


		// 8자 상태 초기화
		Line8_State = 0;

		Line8_LeaveAllOn = 0;


		// 마지막 PWM 초기화
		last_left_duty = 0;
		last_right_duty = 0;
		
		Parallelogram_state = 0;
		Parallelogram_TurnDone = 0;
		LR_Edge_Count = 0;
		Bar_Trigger = 0;
	}


	// ========================================================
	// Raw
	// ========================================================

	else
	{
		Motor_SetSpeed(0, 0);

		UART1_print("Raw mode\r\n");
	}
}


// ============================================================
// RAW 모드
// ============================================================

void Raw_Process(void)
{
	int count = 0;

	for (int i = 0; i < 6; i++)
	{
		raw[i] = ADC_read(i + 2);

		sprintf(
		buf,
		"%d ",
		raw[i]
		);

		UART1_print(buf);

		count++;

		if (count == 6)
		{
			UART1_print("\r\n");
		}
	}

	_delay_ms(500);
}


// ============================================================
// Calibration 모드
// ============================================================

void Calibration_Process(void)
{
	for (int i = 0; i < 6; i++)
	{
		raw[i] = ADC_read(i + 2);

		if (raw[i] > raw_max[i])
		raw_max[i] = raw[i];

		if (raw[i] < raw_min[i])
		raw_min[i] = raw[i];
	}

	UART1_print("Calibrating...\r\n");

	_delay_ms(50);
}


// ============================================================
// 주행 모드
// ============================================================

void Driving_Process(void)
{
	// 센서값 읽기
	Sensor_Update();

	// LED 출력
	LED_Update();

	// 8자 상태 확인
	Line8_Update();


	// ========================================================
	// 8자 특수 주행 상태가 아닐 때만
	// S라인 라인트레이싱 실행
	// ========================================================

	if (Line8_State == 0 ||
	Line8_State == 2 ||
	Line8_State == 4||
	Line8_State == 6||
	Parallelogram_state ==2 ||
	Bar_Trigger == 1)
	{
		SLine_Update();
	}
	Parallelogram_Update();
	
	Bar_Update();
}


// ============================================================
// 센서 업데이트
// ============================================================

void Sensor_Update(void)
{
	for (int i = 0; i < 6; i++)
	{
		raw[i] = ADC_read(i + 2);

		if (raw_max[i] - raw_min[i] != 0)
		{
			filtered[i] =
			(int)(
			((float)(raw[i] - raw_min[i]) /
			(float)(raw_max[i] - raw_min[i])) * 100
			);


			// 0~100 범위 제한

			if (filtered[i] < 0)
			filtered[i] = 0;

			if (filtered[i] > 100)
			filtered[i] = 100;
		}
	}


	// 4/5번 센서 크로스토크 보정
	ADC_Fix();


	// 검정색 라인 감지
	for (int i = 0; i < 6; i++)
	{
		on_line[i] = (filtered[i] < 30);
	}
}


// ============================================================
// LED 출력
// ============================================================

void LED_Update(void)
{
	if (filtered[0] < 30)
	PORTA &= ~(1 << PA0);
	else
	PORTA |= (1 << PA0);


	if (filtered[1] < 30)
	PORTA &= ~(1 << PA1);
	else
	PORTA |= (1 << PA1);


	if (filtered[2] < 30)
	PORTA &= ~(1 << PA2);
	else
	PORTA |= (1 << PA2);


	if (filtered[3] < 30)
	PORTA &= ~(1 << PA3);
	else
	PORTA |= (1 << PA3);


	if (filtered[4] < 30)
	PORTA &= ~(1 << PA4);
	else
	PORTA |= (1 << PA4);


	if (filtered[5] < 30)
	PORTA &= ~(1 << PA5);
	else
	PORTA |= (1 << PA5);
}


// ============================================================
// 버튼 Edge 검출
// ============================================================

int Button_Edge(void)
{
	int pressed = !(PINE & (1 << PE5));

	if (pressed && !button_last)
	{
		_delay_ms(20);

		if (!(PINE & (1 << PE5)))
		{
			button_last = pressed;
			return 1;
		}
	}

	button_last = pressed;

	return 0;
}


// ============================================================
// ADC 초기화
// ============================================================

void ADC_init(void)
{
	ADMUX = (1 << REFS0);

	ADCSRA =
	(1 << ADEN) |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);
}


// ============================================================
// ADC 읽기
// ============================================================

int ADC_read(int channel)
{
	ADMUX =
	(ADMUX & 0xE0) |
	(channel & 0x1F);

	ADCSRA |= (1 << ADSC);

	while (ADCSRA & (1 << ADSC));

	return ADC;
}


// ============================================================
// UART 초기화
// ============================================================

void UART1_init(unsigned int ubrr)
{
	UBRR1H = (unsigned char)(ubrr >> 8);
	UBRR1L = (unsigned char)ubrr;


	UCSR1B =
	(1 << RXEN1) |
	(1 << TXEN1);


	UCSR1C =
	(1 << UCSZ11) |
	(1 << UCSZ10);
}


// ============================================================
// UART 전송
// ============================================================

void UART1_transmit(unsigned char data)
{
	while (!(UCSR1A & (1 << UDRE1)));

	UDR1 = data;
}


void UART1_print(const char *str)
{
	while (*str)
	{
		UART1_transmit(*str++);
	}
}


// ============================================================
// 모터 초기화
// ============================================================

void Motor_init(void)
{
	// PWM 출력
	// 방향 출력

	DDRB |=
	(1 << PB5) |
	(1 << PB6) |
	(1 << PB0) |
	(1 << PB1) |
	(1 << PB2) |
	(1 << PB3);


	// ========================================================
	// 왼쪽 모터 전진
	// IN1 = 0
	// IN2 = 1
	// ========================================================

	PORTB |= (1 << PB1);
	PORTB &= ~(1 << PB0);


	// ========================================================
	// 오른쪽 모터 전진
	// IN3 = 0
	// IN4 = 1
	// ========================================================

	PORTB |= (1 << PB3);
	PORTB &= ~(1 << PB2);


	// ========================================================
	// Timer1 Fast PWM
	// TOP = ICR1
	// 비반전 PWM
	// 64분주
	// ========================================================

	TCCR1A =
	(1 << COM1A1) |
	(1 << COM1B1) |
	(1 << WGM11);


	TCCR1B =
	(1 << WGM13) |
	(1 << WGM12) |
	(1 << CS11) |
	(1 << CS10);


	ICR1 = TOP_VALUE;

	OCR1A = 0;
	OCR1B = 0;
}


// ============================================================
// 모터 속도 설정
// ============================================================

void Motor_SetSpeed(int left_duty, int right_duty)
{
	if (left_duty < 0)
	left_duty = 0;

	if (left_duty > TOP_VALUE)
	left_duty = TOP_VALUE;


	if (right_duty < 0)
	right_duty = 0;

	if (right_duty > TOP_VALUE)
	right_duty = TOP_VALUE;


	OCR1A = left_duty;
	OCR1B = right_duty;
}


// ============================================================
// 킥스타트
// ============================================================

void Kickstart_Process(void)
{
	if (!kickstart_pending)
	return;


	// 양쪽 모터 전진

	PORTB &= ~(1 << PB0);
	PORTB |=  (1 << PB1);

	PORTB &= ~(1 << PB2);
	PORTB |=  (1 << PB3);


	// 킥스타트
	Motor_SetSpeed(
	KICKSTART_DUTY,
	KICKSTART_DUTY
	);


	_delay_ms(KICKSTART_MS);


	// 1회만 실행
	kickstart_pending = 0;
}


// ============================================================
// S자 라인트레이싱
// ============================================================

void SLine_Update(void)
{
	// ========================================================
	// 최초 1회 킥스타트
	// ========================================================

	Kickstart_Process();


	int base_duty =
	(int)(BASE_SPEED * TOP_VALUE);

	int correction = 0;


	// ========================================================
	// 센서별 보정량
	// ========================================================

	if (on_line[0])
	correction -= 120;

	if (on_line[1])
	correction -= 10;

	if (on_line[2])
	correction -= 3;

	if (on_line[3])
	correction += 3;

	if (on_line[4])
	correction += 10;

	if (on_line[5])
	correction += 120;


	// ========================================================
	// 모든 센서가 선을 놓친 경우
	// ========================================================

	if (!on_line[0] &&
	!on_line[1] &&
	!on_line[2] &&
	!on_line[3] &&
	!on_line[4] &&
	!on_line[5])
	{
		Motor_SetSpeed(last_left_duty, last_right_duty);

		return;
	}


	// ========================================================
	// 좌우 PWM 계산
	// ========================================================

	int left_duty =
	base_duty + correction;

	int right_duty =
	base_duty - correction;


	// ========================================================
	// PWM 범위 제한
	// ========================================================

	if (left_duty < 0)
	left_duty = 0;

	if (left_duty > TOP_VALUE)
	left_duty = TOP_VALUE;


	if (right_duty < 0)
	right_duty = 0;

	if (right_duty > TOP_VALUE)
	right_duty = TOP_VALUE;


	// ========================================================
	// 양쪽 모터 전진
	// ========================================================

	PORTB &= ~(1 << PB0);
	PORTB |=  (1 << PB1);

	PORTB &= ~(1 << PB2);
	PORTB |=  (1 << PB3);


	// ========================================================
	// PWM 적용
	// ========================================================

	Motor_SetSpeed(
	left_duty,
	right_duty
	);


	// ========================================================
	// 마지막 주행값 저장
	// ========================================================

	last_left_duty = left_duty;
	last_right_duty = right_duty;
}


// ============================================================
// 8자 라인트레이싱
// ===========================================================

void Line8_Update(void)
{
	if(Line8_State ==0)
	{
		if((on_line[0] && on_line[1] &&on_line[2] && on_line[3] &&on_line[4] && on_line[5]) ||
		(on_line[0] && on_line[1] &&on_line[2] && on_line[3] &&on_line[4] && !on_line[5]))
		{
			Line8_State = 1;
			// 아직 6개 ON 구간을 벗어나지 않음
			Line8_LeaveAllOn = 0;
		}
	}
	// ========================================================
	// State 1
	// 오른쪽 회전
	// ========================================================

	if (Line8_State == 1)
	{
		// 왼쪽 모터 전진

		PORTB &= ~(1 << PB0);
		PORTB |= (1 << PB1);


		// 오른쪽 모터 정지

		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);


		Motor_SetSpeed(150, 0);

		if (!on_line[0] && !on_line[1] && !on_line[2] && !on_line[3] && !on_line[4] &&!on_line[5])
		{
			Line8_LeaveAllOn = 1;
		}
		// 중앙 라인을 다시 찾으면
		// 일반 라인트레이싱

		if ((Line8_LeaveAllOn) && (on_line[2] || on_line[3]))
		{
			Line8_State = 2;
			
			Line8_LeaveAllOn = 0;
		}
	}


	// ========================================================
	// State 2
	// 일반 라인트레이싱
	// ========================================================
	
	else if(Line8_State == 2)
	{
		if((on_line[0] && on_line[1] &&on_line[2] && on_line[3] &&on_line[4] && on_line[5]) || (on_line[0] && on_line[1] &&on_line[2] && on_line[3] &&on_line[4] && !on_line[5])) // 0~4까지 켜지면
		{
			Line8_State = 3;
			// 아직 6개 ON 구간을 벗어나지 않음
			Line8_LeaveAllOn = 0;
		}
	}
	else if(Line8_State == 3)
	{
		// 왼쪽 모터 전진

		PORTB &= ~(1 << PB0);
		PORTB |=  (1 << PB1);


		// 오른쪽 모터 정지

		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);


		Motor_SetSpeed(150, 0);

		if (!on_line[0] && !on_line[1] && !on_line[2] && !on_line[3] && !on_line[4] &&!on_line[5])
		{
			Line8_LeaveAllOn = 1;
		}
		// 중앙 라인을 다시 찾으면
		// 일반 라인트레이싱

		if ((Line8_LeaveAllOn) && (on_line[2] || on_line[3]))
		{
			Line8_State = 4;
			
			Line8_LeaveAllOn = 0;
		}
	}
	else if (Line8_State == 4)
	{
		// 두 번째 ALL ON 진입
		
		if (
		(!on_line[0] &&
		on_line[1] &&
		!on_line[2] &&
		on_line[3] &&
		on_line[4] &&
		!on_line[5])||
		
		(on_line[0] &&
		on_line[1] &&
		!on_line[2] &&
		on_line[3] &&
		!on_line[4] &&
		!on_line[5]) ||
		
		(on_line[0] &&
		on_line[1] &&
		on_line[2] &&
		on_line[3] &&
		!on_line[4] &&
		!on_line[5])||
		
		(!on_line[0] &&
		on_line[1] &&
		on_line[2] &&
		on_line[3] &&
		on_line[4] &&
		!on_line[5])
		)
		{
			Line8_State = 5;
			
		}
	}


	// ========================================================
	// State 3
	// ALL ON 이후 직진
	// ========================================================

	else if (Line8_State == 5)
	{
		
		// 왼쪽 모터 전진

		PORTB &= ~(1 << PB0);
		PORTB |=  (1 << PB1);


		// 오른쪽 모터 정지

		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);


		Motor_SetSpeed(100, 0);
		
		
		
		if (!on_line[0] && !on_line[1] && !on_line[2] && !on_line[3] && !on_line[4] &&!on_line[5])
		{
			Line8_LeaveAllOn = 1;
		}
		// 중앙 라인을 다시 찾으면
		// 일반 라인트레이싱

		if ((Line8_LeaveAllOn) && (on_line[2] || on_line[3]))
		{
			Line8_State = 6;
			
			Line8_LeaveAllOn = 0;
		}
	}
	else if(Line8_State == 6)
	{
		if((!on_line[0] &&
		on_line[1] &&
		on_line[2] &&
		on_line[3] &&
		on_line[4] &&
		!on_line[5]) ||
		
		(on_line[0] &&
		on_line[1] &&
		!on_line[2] &&
		!on_line[3] &&
		on_line[4] &&
		!on_line[5])||
		
		(on_line[0] &&
		on_line[1] &&
		on_line[2] &&
		!on_line[3] &&
		on_line[4] &&
		!on_line[5]) ||
		
		(on_line[0] &&
		on_line[1] &&
		on_line[2] &&
		on_line[3] &&
		on_line[4] &&
		on_line[5])
		)
		{
			// 왼쪽 모터 정지

			PORTB &= ~(1 << PB0);
			PORTB &= ~(1 << PB1);


			// 오른쪽 모터 정지

			PORTB &= ~(1 << PB2);
			PORTB &= ~(1 << PB3);


			Motor_SetSpeed(0, 0);
			
			_delay_ms(1000);
			Line8_State = 7; //다른 State로 넘겨줘야 반복 안됨
			Parallelogram_state = 1; // 평행사변형 시작
		}
	}

}


// ============================================================
// 평행사변형
// ============================================================

// ============================================================
// 평행사변형
// ============================================================

void Parallelogram_Update(void)
{
	int LR_count = 0;


	// ========================================================
	// State 1
	// 좌우 지그재그로 코너 탈출 (10회 반복)
	// ========================================================

	if (Parallelogram_state == 1)
	{
		while (1)
		{
			Sensor_Update();

			if (on_line[0] && !on_line[1] && !on_line[2] && !on_line[3] && !on_line[4] && !on_line[5])
			{
				// 왼쪽 모터 후진
				PORTB |= (1 << PB0);
				PORTB &= ~(1 << PB1);

				// 오른쪽 모터 후진
				PORTB |= (1 << PB2);
				PORTB &= ~(1 << PB3);

				Motor_SetSpeed(150, 150);

				_delay_ms(200);

				// 왼쪽 모터 전진
				PORTB &= ~(1 << PB0);
				PORTB |= (1 << PB1);

				// 오른쪽 모터 후진
				PORTB |= (1 << PB2);
				PORTB &= ~(1 << PB3);

				Motor_SetSpeed(150, 150);

				_delay_ms(200);

				LR_count++;
				LR_Edge_Count = 1;
			}
			else if (!on_line[0] && !on_line[1] && !on_line[2] && !on_line[3] && !on_line[4] && on_line[5])
			{
				// 왼쪽 모터 후진
				PORTB |= (1 << PB0);
				PORTB &= ~(1 << PB1);

				// 오른쪽 모터 후진
				PORTB |= (1 << PB2);
				PORTB &= ~(1 << PB3);

				Motor_SetSpeed(150, 150);

				_delay_ms(200);

				// 왼쪽 모터 후진
				PORTB |= (1 << PB0);
				PORTB &= ~(1 << PB1);

				// 오른쪽 모터 전진
				PORTB &= ~(1 << PB2);
				PORTB |= (1 << PB3);

				Motor_SetSpeed(150, 150);

				_delay_ms(200);

				LR_count++;
				LR_Edge_Count = 2;
			}
			else
			{
				// 왼쪽 모터 전진
				PORTB &= ~(1 << PB0);
				PORTB |= (1 << PB1);

				// 오른쪽 모터 전진
				PORTB &= ~(1 << PB2);
				PORTB |= (1 << PB3);

				Motor_SetSpeed(150, 150);
			}

			if (LR_count == 10)
			{
				Parallelogram_state = 2;
				Parallelogram_TurnDone = 0;
				break;
			}
		}
	}


	// ========================================================
	// State 2
	// LR_Edge_Count 값에 따라 회전->직진 or 직진->회전 (원본 유지)
	// ========================================================

	else if (Parallelogram_state == 2)
	{
		if (!Parallelogram_TurnDone)
		{
			// ====================================================
			// 1회성 회전 (LR_Edge_Count에 따라 방향 결정)
			// ====================================================

			if (LR_Edge_Count == 1)
			{
				// 왼쪽 모터 후진
				PORTB |= (1 << PB0);
				PORTB &= ~(1 << PB1);

				// 오른쪽 모터 전진
				PORTB &= ~(1 << PB2);
				PORTB |= (1 << PB3);

				Motor_SetSpeed(90, 90);

				_delay_ms(200);
				
				// 왼쪽 모터 전진
				PORTB &= ~(1 << PB0);
				PORTB |= (1 << PB1);

				// 오른쪽 모터 전진
				PORTB &= ~(1 << PB2);
				PORTB |= (1 << PB3);

				Motor_SetSpeed(90, 90);

				_delay_ms(500);

				// 왼쪽 모터 후진
				PORTB |= (1 << PB0);
				PORTB &= ~(1 << PB1);

				// 오른쪽 모터 전진
				PORTB &= ~(1 << PB2);
				PORTB |= (1 << PB3);

				Motor_SetSpeed(90, 90);

				_delay_ms(500);
			}
			else if (LR_Edge_Count == 2)
			{
				// 왼쪽 모터 후진
				PORTB |= (1 << PB0);
				PORTB &= ~(1 << PB1);

				// 오른쪽 모터 전진
				PORTB &= ~(1 << PB2);
				PORTB |= (1 << PB3);

				Motor_SetSpeed(90, 90);

				_delay_ms(200);

				// 왼쪽 모터 전진
				PORTB &= ~(1 << PB0);
				PORTB |= (1 << PB1);

				// 오른쪽 모터 전진
				PORTB &= ~(1 << PB2);
				PORTB |= (1 << PB3);

				Motor_SetSpeed(90, 90);

				_delay_ms(500);
			}

			Parallelogram_TurnDone = 1;   // 1회 회전 완료 표시
			
			 Sensor_Update();
			if(on_line[0] && on_line[1])
			{
				PORTA = 1<<PA7;
				Parallelogram_state = 3;
			}
		}
	}


	// ========================================================
	// State 3
	// 계속 좌회전 유지 (조건 없이) -> 라인 재감지 시 다음 단계
	// ========================================================

	else if (Parallelogram_state == 3)
	{
		
			// 왼쪽 모터 후진
			PORTB |= (1 << PB0);
			PORTB &= ~(1 << PB1);
					
			// 오른쪽 모터 전진
			PORTB &= ~(1 << PB2);
			PORTB |= (1 << PB3);
					
			Motor_SetSpeed(150, 150);
		

		if (!on_line[0] && !on_line[1] && !on_line[2] && !on_line[3] && !on_line[4] && !on_line[5])
		{
			Line8_LeaveAllOn = 1;
		}

		if (Line8_LeaveAllOn && (on_line[0]||on_line[1]||on_line[2] || on_line[3]||on_line[4] || on_line[5]))
		{
			Parallelogram_state = 4;
			Line8_LeaveAllOn = 0;
			Bar_Trigger = 1;
		}
	}
}

// ============================================================
// PSD 센서(PF0 / ADC0) 기반 정지 판단
// 155~165 범위면 정지, 그 외에는 계속 라인트레이싱
// ============================================================

void Bar_Update(void)
{
	if(Bar_Trigger == 1)
	{
		// PSD 센서 값 읽기 (PF0 = ADC 채널 0)
		psd_value = ADC_read(0);
		
		// 라인 센서도 최신 값으로 갱신
		Sensor_Update();
		
		// PSD 값이 155~165 사이면 정지
		if (psd_value >= 150 && psd_value <= 170)
		{
			PORTB &= ~(1 << PB0);
			PORTB &= ~(1 << PB1);
			
			PORTB &= ~(1 << PB2);
			PORTB &= ~(1 << PB3);
			
			Motor_SetSpeed(0, 0);
			
			return;
		}
		else
		{
			// 그 외에는 기존 S라인 로직으로 계속 주행
			SLine_Update();
		}
		
		if(on_line[0] && on_line[1] && on_line[2] && !on_line[3] && !on_line[4] && !on_line[5])
		{
			//왼쪽 모터
			PORTB &= ~(1 << PB0);
			PORTB &= ~(1 << PB1);
			
			PORTB &= ~(1 << PB2);
			PORTB &= ~(1 << PB3);
			
			Motor_SetSpeed(0, 0);
		}
	}
}

// ============================================================
// ADC 크로스토크 보정
// ============================================================

void ADC_Fix(void)
{
	if (
	filtered[3] <= 75 &&
	50 <= filtered[3]
	)
	{
		filtered[4] = 20;
	}

	else if (
	10 <= filtered[3] &&
	filtered[3] <= 15
	)
	{
		filtered[4] = 75;
	}
}
