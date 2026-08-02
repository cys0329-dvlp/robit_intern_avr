# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 08/02

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러와 PSD 센서의 작동 원리를 공부하여 
ADC 값을 cm로 환산하여 출력한다.

### 핵심 목표
* PSD센서의 아날로그 값을 디지털 값으로 환산, 디지털값 필터링
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

### 핵심 코드 및 레지스터 설정
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
if(g_do_sample)
{
	g_do_sample = 0;
	
	raw_adc = Adc_Read();               //필터를 거치지 않은 원시값
	samples[idx] = raw_adc;
	idx = (unsigned char)((idx+1) % MEDIAN_N);
	
	if(filled < MEDIAN_N) filled++;
}
```

### RAW / FILTERED 동시 출력 — 최근 raw 값 추출 + 중앙값 필터 적용
```c
//가장 최근에 읽은 raw 샘플 (필터 적용 전)
raw_adc = samples[(unsigned char)((idx + MEDIAN_N - 1) % MEDIAN_N)];

//5개 샘플에 중앙값 필터를 적용한 값 (필터 적용 후)
filtered_adc = Median_Of(samples, MEDIAN_N);

//거리 계산은 스파이크가 제거된 filtered 값을 기준으로 함
status = Psd_AdcToDist(filtered_adc, &dist);
```

### RAW / FILTERED 동시 출력 — UART 출력 포맷
```c
case PSD_OK:
	sprintf(line, "RAW:%4u | FILTERED:%4u | DISTANCE: %3u.%1u cm\r\n",
			raw_adc, filtered_adc, (unsigned int)(dist/10), (unsigned int)(dist%10));
	break;
```

### RAW / FILTERED 동시 출력 — 예외 상황에서도 raw/filtered 병기
```c
case PSD_TOO_CLOSE:
	sprintf(line, "RAW:%4u | FILTERED:%4u | [WARN] too close (<20cm)\r\n",
			raw_adc, filtered_adc);
	break;
```
---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. UART 통신 연결 시 ADC 출력 값(Raw), 필터링된 값(Filtered) 과 cm 값 출력

### 동작 사진 / 영상

| 정면 동작 모습 | 
| https://drive.google.com/file/d/1Ort1VuNEh2HA40mo1iCX9ei1nvN7qSY4/view?usp=drive_link | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **claude** | RAW/FILTERED 동시 출력 기능 구현 | - 기존에는 중앙값 필터를 적용한 값만 출력하고 있었는데, 필터링 전/후 값을 비교해서 보여주고 싶어 질문함. 필터 적용 전 원시값을 별도 변수에 저장해뒀다가 필터링된 값과 나란히 출력하는 구조를 제안받아 참고하여 직접 코드에 반영했습니다.


### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함
