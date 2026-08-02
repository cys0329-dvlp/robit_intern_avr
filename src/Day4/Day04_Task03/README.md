# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 08/02

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러와 PSD 센서의 작동 원리를 공부하여 
ADC 값을 cm로 환산하여 출력한다.

### 핵심 목표
* PSD센서의 아날로그 값을 디지털 값으로 환산
* ADC 환산 결과를 cm로 다시 환산

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 / UART|
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, PSD 센서|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
 PORTF (PF1)   ----->   PSD signal pin
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **주의사항:**
	ISP 다운로드 시 SPI 핀 타겟 전원 및 리셋 회로 간섭 주의
	atmelstudio chip 프로그램과 시리얼통신 1.9b 프로그램을 동시에 띄워놓으면 컴파일 혹은 Connect 에러 발생

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── Day04_Task03/
   ├── Day04_Task03.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day04_Task03.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### ADC 초기화 및 읽기
```c
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

```

### 타이머 인터럽트로 샘플링/출력 타이밍 만들기
```c
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

```

### 스파이크 제거를 위한 중앙값 필터
```c
unsigned int Median_Of(unsigned int *src, unsigned char n)
{
	unsigned int buf[MEDIAN_N];
	unsigned char i, j;
	unsigned int key;
	
	for(i = 0; i < n; i++) buf[i] = src[i];
	
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
```

### ADC값을 거리로 환산 (예외처리 + 선형보간)
```c
unsigned char Psd_AdcToDist(unsigned int adc, unsigned int *dist)
{
	unsigned char i;
	unsigned int a0, a1, d0, d1;
	
	*dist = 0;
	
	if(adc < ADC_DISCONNECT) return PSD_DISCONNECTED;
	if(adc < ADC_FAR_LIMIT) return PSD_TOO_FAR;
	if(adc > ADC_NEAR_LIMIT) return PSD_TOO_CLOSE;
	
	for(i = 0; i < LUT_SIZE - 1; i++)
	{
		a0 = pgm_read_word(&psd_lut[i].adc);
		a1 = pgm_read_word(&psd_lut[i+1].adc);
		
		if(adc >= a0 && adc <= a1)
		{
			d0 = pgm_read_word(&psd_lut[i].dist);
			d1 = pgm_read_word(&psd_lut[i+1].dist);
			*dist = (unsigned int)(d0 - (unsigned int)(((unsigned long)(d0-d1) * (adc-a0)) / (a1-a0)));
			return PSD_OK;
		}
	}
	
	return PSD_TOO_FAR;
}
```
---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. UART 통신 연결 시 ADC 출력 값과 cm 값 출력

### 동작 사진 / 영상

| 정면 동작 모습 | 
|https://drive.google.com/file/d/1m1t8kp5E1JLhBhN6H7uMSqVJi4YNbkVF/view?usp=drive_link | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **claude** | 룩업테이블 + 정수 선형보간 | - ADC 값을 거리로 변환할 때, 센서 특성이 선형이 아니라서 몇 개의 기준점(룩업테이블)만 가지고 있고 그 사이 값은 보간해서 구해야 했습니다. 처음에는 부동소수점 나눗셈으로 비율을 계산하려 했는데, AVR 환경에서는 float 연산 비용이 크다는 점이 걸려서 정수 연산만으로 처리하는 방법을 Claude에게 질문했습니다.
| **claude** | 타이머 인터럽트로 두 개의 주기 동시 처리 | -샘플링 주기(20ms)와 출력 주기(200ms)가 서로 다른데, _delay_ms로 블로킹하면 두 주기를 동시에 관리할 수 없다는 문제가 있었습니다. 하나의 타이머 인터럽트로 여러 주기를 어떻게 동시에 처리하는지 원리를 Claude에게 질문했습니다. 답변을 통해 1ms마다 발생하는 인터럽트 안에서 카운터 변수 여러 개를 각각 증가시키고, 각 카운터가 목표값에 도달할 때만 플래그를 세우는 방식으로 여러 주기를 독립적으로 만들 수 있다는 원리를 이해했습니다. 또한 인터럽트 안에서는 시간이 오래 걸리는 작업(출력 등)을 직접 하지 않고, 메인 루프에서 플래그를 확인해서 처리하는 구조가 왜 필요한지도 파악했습니다. 이 원리를 바탕으로 실제 카운터 변수 설계, 플래그 처리 로직, 메인 루프 폴링 구조는 직접 작성했습니다.
| **claude** | 중앙값 필터를 이용한 스파이크 제거 | - 센서 값을 그대로 쓰면 순간적으로 튀는 값(스파이크) 때문에 거리 출력이 불안정한 문제가 있었습니다. 평균을 내는 방법을 먼저 시도했는데, 튄 값이 평균 계산에 섞여서 결과가 같이 흔들리는 문제가 있어 다른 방법을 Claude에게 질문했습니다.


### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함
