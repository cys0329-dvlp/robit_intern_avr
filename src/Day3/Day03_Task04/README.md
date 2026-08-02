# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 7/30

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러와 UART 통신을 DORTD만 활용하여 Hello worl!를 터미널에 출력하기 위한 과제이다.

### 핵심 목표
* UART 관련 레지스터를 사용하지않고 PORTD만 활용하여 TXD(출력) 설정하기

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 / UART|
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, UART 통신선 |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
PORTD (PD3)  ----->   TXD(출력)
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
├── Day03_Task04/
   ├── Day03_Task04.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day03_Task04.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### PD3 HIGH 설정 및 Hello World! 출력
```c
int main(void)
{
	DDRD |= (1 << PD3);   // PD3(TX)만 출력으로 설정
	PORTD |= (1 << PD3);  // 기본 상태 = HIGH

	while (1)
	{
		TX_Start();
		_delay_ms(1000); // 1초마다 보내기
	}
}
```
### 문자열 출력 함수
```c
void Putch(char str)
{
	PORTD &= ~(1 << PD3); // START BIT: LOW
	Bit_Delay(); 

	// 데이터 8비트
	for (int i = 0; i < 8; i++)
	{
		if (str & 0x01)
		{
			PORTD |= (1 << PD3);
		}
		else
		{
			PORTD &= ~(1 << PD3);
		}

		Bit_Delay();
		str >>= 1; // 비트 한칸씩 옮기기(str = str >>1 이랑 똑같음)
	}

	// STOP BIT: HIGH
	PORTD |= (1 << PD3);
	Bit_Delay();
}
```
---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 시작하면 1초마다 터미널에 Hello World! 출력

### 동작 사진 / 영상

| 정면 동작 모습 | 
| https://drive.google.com/file/d/1WbQJWqZgiRy8n6ml1X8RlfRzZWhX7JGW/view?usp=drive_link | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Claude** | 보드레이트 계산 | - 계산해봤을 때 delay 104로 하면 되겠다해서 했는데 1초마다 작동을 안해서 AI에게 물어본 결과 밀리세컨드가 아니라 마이크로세컨드인 us를 써야한다는 답변을 받았습니다.

### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, 절대 코드를 직접 짜달라고 하지않았습니다. 

