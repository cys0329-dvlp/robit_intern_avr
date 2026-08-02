# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 08/02

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러와 UART를 활용하여 시리얼 통신 과정을 공부하고 제어하며 스위치 입력과 키보드 입력에 따라 변화하는 LED의 모습을 확인하기 위한 과제이다.

### 핵심 목표
* ATmega128 레지스터 설정과 UART 통신을 통한 LED 제어
* UART 활성화 방법과 송수신 방법, 터미널 출력 방법 등 활용

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 / UART|
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, LED(8개), push botton(4개) |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
 PORTA (PA0 ~ PA7)   ----->   8-Bit LED
 PORTD (PD1)  ----->   SW1

```

### 주요 회로 특징
* **전원:** 5V DC 안정화 전원 공급
* **주의사항:**
	ISP 다운로드 시 SPI 핀 타겟 전원 및 리셋 회로 간섭 주의
	atmelstudio chip 프로그램과 시리얼통신 1.9b 프로그램을 동시에 띄워놓으		면 컴파일 혹은 Connect 에러 발생

---

## 4. 프로젝트 구조 (Directory Structure)
> 구현부(.c), 선언부(.h)만 구조에 표기함.
```text
├── Day03_Task02/
   ├── Day03_Task02.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day03_Task02.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### 입력 받은 값에 따라 LED 출 
```c
if(input_data== '0')
		{
			PORTA = 0X7F;
			UART_transmit_string("0 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '1')
		{
			PORTA = 0XBF;
			UART_transmit_string("1 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '2')
		{
			PORTA = 0XDF;
			UART_transmit_string("2 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '3')
		{
			PORTA = 0XEF;
			UART_transmit_string("3 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '4')
		{
			PORTA = 0XF7;
			UART_transmit_string("4 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '5')
		{
			PORTA = 0XFB;
			UART_transmit_string("5 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '6')
		{
			PORTA = 0XFD;
			UART_transmit_string("6 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '7')
		{
			PORTA = 0XFE;
			UART_transmit_string("7 LED on\r");
			_delay_ms(100);
		}
		else if(input_data == '8')
		{
			PORTA = (PORTA >> 1)| 0b10000000;
			UART_transmit_string("LEFT\r");
			_delay_ms(100);
		}
```
### 기능1. 8입력되면 좌측으로 이동 후 LEFT 출력 
```c
else if(input_data == '8')
		{
			PORTA = (PORTA >> 1)| 0b10000000;
			UART_transmit_string("LEFT\r");
			_delay_ms(100);
		}

```

### 기능2. 9입력되면 우측으로 이동 후 RIGHT 출력 
```c
else if(input_data == '9')
		{
			PORTA = (PORTA << 1)| 0b00000001;
			UART_transmit_string("RIGHT\r");
			_delay_ms(100);
		}

```

### LED 우측, 좌측으로 옮기기
```c
else if(input_data == '8')
		{
			PORTA = (PORTA >> 1)| 0b10000000;
			UART_transmit_string("LEFT\r");
			_delay_ms(100);
		}
		else if(input_data == '9')
		{
			PORTA = (PORTA << 1)| 0b00000001; 
			UART_transmit_string("RIGHT\r");
			_delay_ms(100);
		}
```
---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 기본 상태: LED 모두 꺼짐 상태. 
2. 시리얼 통신에 입력하는 값(0~7)에 따라서 LED 하나씩 켜짐(중첩 안됨)
ex) 0 누르면 0번 째 LED 출력, 4누르면 4번 때 LED 출력
4. 시리얼 통신에 8입력 시 좌측, 9입력 시 LED 하나씩 이동
5. 지정된 스위치를 누르면 INT3 발생시켜 RESET 출력 후 초기상태로 돌아감

### 동작 사진 / 영상

| 정면 동작 모습 | 
| https://drive.google.com/file/d/1eq6jIFhQi9UHLastT5AHoDD_CUi4hLQJ/view?usp=drive_link | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Chat GPT** | 명령어 존재 여부 | - 시리얼 통신 화면 리셋 관련 명령어가 존재하는지 질문했고 존재하지않는다는 답을 받았습니다.
| **Chat GPT** | 시리얼 입력값 중복 문제 | - 처음엔 아래 코드와 같이 작성했습니다. GPT에게 정답을 절대 알려주지말고 힌트를 달라고했더니 함수 반환값을 문자와 여러번 비교하면 정상작동은하지만 원하는 결과가 출력되지 않을 것이라는 답변을 받았습니다. 
```C
while (1)
	{
		char recvData = Uart_Getch();
		if(UART_transmit_string(char *str) == 'a')
		{
			PORTA = 0X07;
			_delay_ms(100);
		}
	}
```

### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함

