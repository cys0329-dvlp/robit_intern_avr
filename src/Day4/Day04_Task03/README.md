# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 08/02

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러와 UART를 활용하여 시리얼 통신 과정을 공부하고 제어하며 키보드 입력에 따라 변화하는 서보모터의 모습을 확인하기 위한 과제이더.

### 핵심 목표
* ATmega128 레지스터 설정과 UART 통신을 통한 LED 제어
* UART 활성화 방법과 송수신 방법, UART를 통한 서보모터 제어

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 / UART|
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드 |

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
1. 기본 상태: servo 90도 상태
2. 시리얼 통신에 입력하는 값(0~180)에 servo angle 변경
3. 시리얼 통신에 입력하는 값이 0 미만, 180 초과 혹은 문자라면 오류 메시지 출력
5. reset botton 누르면 90도 초기 상태로 초기화

### 동작 사진 / 영상

| 정면 동작 모습 | 
|https://drive.google.com/file/d/1E4AfsIyCXFuxQcfJjwJicNj6d98FCgY6/view?usp=drive_link | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **claude** | 문자열 -> 정수형 변환 | - 문자 입력받고 작동까지하도록 코드 작성했는데 문자가 한자리수까지만 입력돼서 10의 자리, 100의 자리까지 어떻게 입력받아야할지 질문하여
아이디어를 참고하여 작성했습니다. 
| **Chat GPT** | 시리얼 입력값 중복 문제 | - 처음엔 아래 코드와 같이 작성했습니다. GPT에게 정답을 절대 알려주지말고 힌트를 달라고했더니 함수 반환값을 문자와 여러번 비교하면 정상작동은하지만 원하는 결과가 출력되지 않을 것이라는 답변을 받았습니다. 


### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함

   
참고) 
```c
void Servo_Move(int angle) 
```
는 google 자료에 있는 예제 참고했습니다.
