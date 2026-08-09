# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 08/09

---

## 1. 개요 (Overview)
본 과제는 모터 두개를 작동시키는 과제입니다.  

### 핵심 목표
* PWM 제어를 통해 두개의 모터를 제어한다

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500|
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, LCD(I2C), L298N|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
ENA -----------------------> PB5
ENB -----------------------> PB6
IN1 -----------------------> PB0
IN2 -----------------------> PB1
IN3 -----------------------> PB2
IN4 -----------------------> PB3
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
├── Day06_Task02/
   ├── Day06_Task02.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day06_Task02.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### 초기 설정
```c
// PB0~PB3 : IN1~IN4, PB5,PB6 : ENA,ENB
DDRB = 0x6F;   // PB0~PB3, PB5, PB6 출력
PORTB = 0x00;  // 처음에는 전부 LOW

// ENA, ENB 항상 활성화 (모터 드라이버 인에이블)
PORTB |= (1 << PB5) | (1 << PB6);
```

### 정회전 제어
```c
// 모터 A : IN1=1, IN2=0
PORTB |=  (1 << PB0);
PORTB &= ~(1 << PB1);
// 모터 B : IN3=1, IN4=0
PORTB |=  (1 << PB2);
PORTB &= ~(1 << PB3);
_delay_ms(3000);

```

### 역회전 제어
```c
// 모터 A : IN1=0, IN2=1
PORTB &= ~(1 << PB0);
PORTB |=  (1 << PB1);
// 모터 B : IN3=0, IN4=1
PORTB &= ~(1 << PB2);
PORTB |=  (1 << PB3);
_delay_ms(3000);
```


### 동작 사진 / 영상

| 정면 동작 모습 | 
|https://drive.google.com/file/d/1zckwp2CxCFD-FS4CZ-umM2-0FKddpawx/view?usp=drive_link| 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **claude** | 전체적인 예제 형식 작성 요청 | - 회로 연결 정보를 주어 간단한 코드 틀 작성을 요청했습니다.
| **claude** | 모터 드라이버에 출력되는 전압 측정 | - 전원을 연결했을 때 각 모터에 몇 V가 찍혀야 정상인지 질문했습니다

### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함
