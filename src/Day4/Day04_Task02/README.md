# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 08/02

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러와 LCD를 활용하여 전류를 보내며 가변저항으로 그 양을 조절하고 그 양에 따라 시계를 만든다.

### 핵심 목표
* ATmega128 레지스터 설정과 가변저항 값에 따른 LCD에 값 출력
* 년도, 월, 일, 시간, 분, 초를 설정한 후 sw2를 누르면 시계가 시작

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 / UART|
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, LCD(I2C) |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
 PORTD (PD1)   ----->   SDA
PORTD(PD0) ---------> SCL
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **주의사항:**
	ISP 다운로드 시 SPI 핀 타겟 전원 및 리셋 회로 간섭 주의
  I2C 가변저항에 따라 출력이 안될 수 있으니 가변저항 값 잘 맞추기
---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── Day04_Task02/
   ├── Day04_Task02.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day04_Task02.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### 가변저항 값(ADC)을 원하는 범위로 매핑하는 함수
```c
	// ADC값(0~1023)을 원하는 범위(lo~hi)로 바꿔주는 함수
// 연도처럼 큰 범위(0~2999)도 되게 16비트로 계산함
static uint16_t adc_map(uint16_t adc, uint16_t lo, uint16_t hi)
{
    uint32_t range = (uint32_t)(hi - lo + 1);
    uint32_t val = lo + ((uint32_t)adc * range) / 1024;
    if (val > hi) val = hi; // 혹시 넘으면 최대값으로 clamp
    return (uint16_t)val;
}
```

### 가변저항 + SW1로 연/월/일/시/분/초 순서로 세팅하는 로직
```c
// ---------------- 1) 날짜/시간 세팅 단계 ----------------
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
            uint8_t maxd = days_in_month(g_mon, g_year); // 예외처리 : 이미 정한 월/윤년 기준 최대 일수
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

```

### 오류 메시지 출력을 위한 함수
```c
void Uart_Putch(unsigned char PutData)
{
	while(!(UCSR0A & (1 << UDRE0))); // UCSR0A의 Bit5 활성화 -> 새로운 Data를 입력받을 준비가 됐다는 flag (Datasheet 189P 참고)
	UDR0 = PutData; //받은 값을 입력 buffer에 넣음
}

void UART_transmit_string(char *str)
{
	while(*str != '\0') // \0이면 while 문 중지
	{
		Uart_Putch(*str);
		str++;
	}
}
```

### 윤년/월말 자리올림 예외처리 (Timer1 10ms 인터럽트 내부)
```c
TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC모드, 분주비 64
OCR1A = 2499; // 16,000,000 / 64 / 100Hz - 1 = 2499  -> 10ms마다 인터럽트

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
                            if (g_year > 2999) g_year = 0;
                        }
                    }
                }
            }
        }
    }
}
```
### I2C(TWI) 통신속도 계산 및 LCD 4bit 초기화
```c
TWSR = 0x00;                          // 프리스케일러 1로
TWBR = ((F_CPU / 100000UL) - 16) / 2; // SCL 100kHz 나오게 계산
TWCR = (1 << TWEN);
```
---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 기본 상태: 선택 화면
2. 가변 저항을 돌린 후 sw1을 눌러 확정
3. 확정되면 다음 선택으로 넘어감(년, 월, 일, 시간, 분, 초)
5. 모두 확정된 후 sw2누르면 시계 시작

### 동작 사진 / 영상

| 정면 동작 모습 | 
|https://drive.google.com/file/d/1Nd9RvHEn2LbSkVNHIG-wE136nkN0bCuk/view?usp=drive_link | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **claude** | 문자열 -> 정수형 변환 | - 문자 입력받고 작동까지하도록 코드 작성했는데 문자가 한자리수까지만 입력돼서 10의 자리, 100의 자리까지 어떻게 입력받아야할지 질문하여
아이디어를 참고하여 작성했습니다. 


### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함
