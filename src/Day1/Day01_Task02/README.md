# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 (학부명)**  
> **작성자:** (이름)
> **제출일:** (날짜)

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러를 활용하여 주요 주변장치(Peripherals)를 제어하고 센서 데이터를 수신/처리하는 시스템을 구현하는 것을 목표로 함.

### 핵심 목표
* ATmega128 레지스터 설정을 통한 주변장치 제어
* 센서 및 외부 모듈과의 통신 (USART / SPI / I2C 등) 및 데이터 처리
* 타이머/카운터를 활용한 PWM 출력 및 인터럽트 제어

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, DC/STEP 모터, ADC 센서 모듈 |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
 PORTA (PA0 ~ PA7)   ----->   8-Bit LED
 PORTB Pin 4 (PB4)   ----->   PWM Motor Control (OC0)
 PE0 (RXD0) / PE1    ----->   UART Serial Communication
 ADC0 (PF0)          ----->   Analog Sensor Input
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **주의사항:** ISP 다운로드 시 SPI 핀 타겟 전원 및 리셋 회로 간섭 주의

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── Day00_Task00/
│   ├── main.c # 메인 제어 루프 및 시스템 초기화
│   ├── timer. # 타이머/카운터 및 PWM 설정
│   ├── uart.c # 시리얼 통신(UART) 드라이버
│   └── adc.c # ADC 데이터 수신 드라이버
├── include/
│   ├── timer.h
│   ├── uart.h
│   └── adc.h
├── docs/
│   └── schematic.pdf # 회로도 파일
└── README.md
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### 타이머/카운터 및 PWM 초기화 예시 (`timer.c`)
```c
#include <avr/io.h>
#include <avr/interrupt.h>

void Timer0_PWM_Init(void) {
    // Fast PWM Mode, Non-inverting Mode 설정
    TCCR0 |= (1 << WGM00) | (1 << WGM01); // Fast PWM Mode
    TCCR0 |= (1 << COM01); // Non-inverting Mode
    TCCR0 |= (1 << CS02) | (1 << CS01); // 분주비 256 설정
    
    DDRB |= (1 << PB4); // OC0 핀 출력 설정
}
```

---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 시스템 전원 인가 시 ATmega128 주변장치(UART, Timer, ADC) 초기화함
2. 센서 입력 값 및 인터럽트 신호 수신함
3. PWM 제어를 통한 액추에이터 제어 및 UART 시리얼 데이터 출력함

### 동작 사진 / 영상

| 정면 동작 모습 | 센서 측정 및 시리얼 출력 |
| :---: | :---: |
| ![Hardware Setup](개인_구글드라이브_링크_첨부) | ![Serial Monitor](개인_구글드라이브_링크_첨부) |

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **ChatGPT / Claude** | 코드 디버깅 & 리팩토링 | - 빌드 에러 및 문법 오류 원인 분석<br>- 레지스터 설정 주석 작성 및 가독성 개선 |
| **Gemini** | 개념 정리 & 모듈 설계 | - ATmega128 Timer/Counter 및 ADC 인터럽트 동작 원리 검토<br>- 전체 프로젝트 파일/디렉토리 구조 설계 참고 |

### AI 활용 및 검증 원칙
1. **코드 검증:** AI가 생성한 레지스터 설정 및 함수 코드는 데이터시트(ATmega128 Datasheet)와 비교 검증한 후 실제 오실로스코프/시리얼 모니터링을 거쳐 직접 수정 및 테스트하였습니다.
2. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 보조 도구(디버깅, 문서화)로만 활용하였습니다.
