# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 7/30

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러를 활용하여 주요 주변장치(Peripherals)를 제어하고 스위치 입력에 따라 변화하는 LED의 모습을 확인하기 위한 과제이다.

### 핵심 목표
* ATmega128 레지스터 설정을 통한 주변장치 제어
* 스위치 INPUT과 LED OUTPUT의 회로 연결 확인 및 활용

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 |
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, LED(8개), push botton(4개) |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
 PORTA (PA0 ~ PA7)   ----->   8-Bit LED
 PORTD (PD2, PD3)  ----->   pushbotton 1,2
 PORTE (PE4, PE5)    ----->   pushbotton 3,4
```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **주의사항:** ISP 다운로드 시 SPI 핀 타겟 전원 및 리셋 회로 간섭 주의

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── Day00_Task00/
   ├── Day01_Task02.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day01_Task02.componentinfo.xml #IDE가 해당 디바이스 지원 팩을 프로젝트에 올바르게 연결·관리하도록 해주는 메타데이터 파일
   ├── Day01_Task02.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### 0.5S마다 깜빡임 + 내부 인터럽트 
```c
while(1)
	{
		if((!(PIND &(1<<PIND2))) && (!(PINE & (1<<PINE5))))
		{
			PORTA = 0X00;
			_delay_ms(100);
		}
		else if (!(PINE & (1<<PINE5))) //SW1 눌렀을 떄 0~3 출력
		{
			PORTA = 0X0F;
			_delay_ms(100);
		}
		else if (!(PIND &(1<<PIND2))) // SW2 눌렀을 때 4~7 출력
		{
			PORTA = 0XF0;
			_delay_ms(100);
		}
		else
		{
			PORTA = 0XFF; //모두 끔
			_delay_ms(500);
			PORTA = 0X00; //모두 킴
			_delay_ms(500);
		}
		
	}
```
### 외부인터럽트3 -> 왼쪽에서 오른쪽으로 LED 이동(PD3 사용)
```c
ISR(INT3_vect)
{
	PORTA = 0X7F;
	_delay_ms(100);
	
	PORTA = 0XBF;
	_delay_ms(100);
	
	PORTA = 0XDF;
	_delay_ms(100);
	
	PORTA = 0XEF;
	_delay_ms(100);
	
	PORTA = 0XF7;
	_delay_ms(100);
	
	PORTA = 0XFB;
	_delay_ms(100);
	
	PORTA = 0XFD;
	_delay_ms(100);
	
	PORTA = 0XFE;
	_delay_ms(100);
}

```

### 외부인터럽트4 -> 오른쪽에서 왼쪽으로 LED 이동(PE4 사용)
```c
ISR(INT4_vect)
{
		PORTA = 0XFE;
		_delay_ms(100);
		
		PORTA = 0XFD;
		_delay_ms(100);
		
		PORTA = 0XFB;
		_delay_ms(100);
		
		PORTA = 0XF7;
		_delay_ms(100);
		
		PORTA = 0XEF;
		_delay_ms(100);
		
		PORTA = 0XDF;
		_delay_ms(100);
		
		PORTA = 0XBF;
		_delay_ms(100);
		
		PORTA = 0X7F;
		_delay_ms(100);
	
}
```
---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 기본 상태: 0.5S마다 깜빡임
2. PD2 눌리면 LED4~7 출력, PE5 눌리면 LED0~3 출력 (내부 인터럽트)
3. PD3 눌리면 왼쪽에서 오른쪽으로 LED 하나 씩 이동 (외부 인터럽트)
4. PE4 눌리면 오른쪽에서 왼쪽으로 LED 하나 씩 이동 (외부 인터럽트)

### 동작 사진 / 영상

| 정면 동작 모습 | 
| (https://drive.google.com/file/d/1tHC01KBqHPHXJxw1AKZOmwz98_phY6b1/view?usp=drive_link) | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 스위치 동시 눌림 해결 | - 스위치 동시에 누를 때와 각각 누를 때 동시 입력이 안되는 문제점을 if문 제일 위에 동시 입력 조건을 넣어야한다고 피드백 받음

### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함
