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
| **주요 부품** | ATmega128 개발보드, servo motor(SG90)|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
 PORTF (PF7)   ----->   servo motor
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
├── Day04_Task05/
   ├── Day04_Task05.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day04_Task05.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### servo motor 초기화 및 입력 값에 따른 angle 변화
```c
	Servo_Move(90); // 처음 시작하거나 리셋하면 초기 자세로
	while (1)
	{
		int input_data = Uart_Getint(); //숫자 받기
		
		if(0<=input_data && input_data <=180)
		{
			Servo_Move(input_data);
			_delay_ms(1000);
		}
		else if(input_data < 0 || 180 < input_data )
		{
			UART_transmit_string("ERROR. angle can be between 0 and 180\r");
		}
		else
		{
			UART_transmit_string("ERROR. angle can be between 0 and 180\r");
		}
	}

```

### 문자를 숫자로 바꾸어 입력해주는 함수(엔터를 누르면 값 입력 끝)
```c
int Uart_Getint(void)
{
	unsigned char ch;
	int result = 0;
	int started = 0;  // 숫자를 하나라도 받았는지 체크

	while (1)
	{
		while(!(UCSR0A & (1 << RXC0))); // 데이터 수신 대기
		ch = UDR0;

		if (ch == '\r' || ch == '\n')
		{
			if (started)
			{
				break;      // 숫자 입력 후 엔터 -> 정상 종료
			}
			else
			{
				continue;   // 숫자 없이 들어온 개행문자는 그냥 무시(잔여 \n 처리)
			}
			
		}
		else if (ch >= '0' && ch <= '9')
		{
			result = result * 10 + (ch - '0'); // ch 아스키값에서 0의 아스키 값인 48을 빼서 진짜 입력한 숫자로 인식되게 함
			started = 1;
		}
		else
		{
			return -1;
			break;
		}
	}

	return result;
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
