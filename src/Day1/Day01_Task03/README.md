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
├── Day01_Task03/
   ├── Day01_Task03.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day01_Task03.componentinfo.xml #IDE가 해당 디바이스 지원 팩을 프로젝트에 올바르게 연결·관리하도록 해주는 메타데이터 파일
   ├── Day01_Task03.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)
**원래 INT0,1,2,3을 사용해야하지만 입력핀을 PD2,3 / PE4,5를 사용했기 때문에 INT2,3,4,5를 사용했습니다.
### 2진 카운트
```c
while(1)
	{
		PORTA = count;   // count의 각 비트를 그대로 LED에 반영
		_delay_ms(100);
		if(count == 0X00)
		{
			count = 0XFF; //인터럽트에서는 무한루프 걸리면 안되기 때문에 한번 다 돌면 나가기
		}
		else
		{
			count--; //0XFF로 다 채운 상태에서 1씩 감소
		}
	}

```
### INT4(3개씩 우측 이동)
```c
ISR(INT4_vect) // 3개씩 우측 이동
{
	for (int i = 7; i >=0; i--)
	{
		PORTA = ~(1 << (i-2)) & ~(1 << (i-1)) & ~(1 << i);
		_delay_ms(300);
		if(i <0)
		{
			break;
		}
	}
}
```

### INT2(3개씩 좌측 이동)
```c
ISR(INT2_vect) //3개씩 좌측 이동
{
	for (int i = 0; i < 8; i++)
	{
		PORTA = ~(1 << i) & ~(1 << (i+1)) & ~(1 << (i+2));
		_delay_ms(300);
		if((i+2) > 8)
		{
			break;
		}
	}
}
```
### INT3(1개씩 좌우 이동)
```c
ISR(INT3_vect) //좌측 이동했다가 우측 이동
{
	//PA는 안써도 <avr/io.h>에 이미 PA0 = 0, PA1 = 1.. 로 정의되어있음
	
	for (int i = 0; i < 8; i++)
	{
		PORTA = ~(1 << i);
		_delay_ms(100);
	}
	for (int i = 0; i < 8; i++)
	{
		PORTA = ~(1 << (7 - i));  //0이 켜지고 1이 꺼지므로 ~로 반전시켜야함
		_delay_ms(100);
	}
}
```
### INT5(2진 카운터 초기화)
```c
ISR(INT5_vect) // 2진 카운터 초기화
{
	count = 0XFF; //count만 처음으로 초기화 시킨 뒤 main에서 다시 돌리기
}
```
---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 0.1s마다 2진 카운터 진행
2. INT2: for문 안 i = 0부터 7까지 i, i+1, i+2의 값들을 하나씩 옆으로 옮김
   -> 만약 i+2가 8보다 크다면 for문 중단 -> 3개씩 좌측 이동
3. INT4: for문 안 i = 7부터 7까지 i-2, i-1, i의 값들을 하나씩 옆으로 옮김
   -> 만약 i가 0보다 작다면 for문 중단 -> 3개씩 우측 이동
4. INT3: 

### 동작 사진 / 영상

| 정면 동작 모습 | 
|(https://drive.google.com/file/d/1_FWkoJnIHqlHSSgurhoePAruJzNyUMhm/view?usp=drive_link) | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 핀 입력 방식 조언 | - 비트연산자를 사용하여 PA의 값들을 하나씩 옮기고 싶었습니다. PA의 포트를 for문 안에서 배열로 받아야하나? 라는 생각에 배열로 선언하는 방식에 대해 물어봤지만 포트는 배열로 선언할 수 없고 <avr/io.h>에 이미 PA0 = 0, PA1 = 1.. 로 정의되어있다는 정보를 얻을 수 있었습니다. 그 후 for문을 직접 작성하였습니다.

### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함

