# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 08/03

---

## 1. 개요 (Overview)
본 과제는 ATmega128 마이크로컨트롤러와 UART를 활용하여 시리얼 통신 과정을 공부하고 MAX485와 UART 통신의 연결을 이해한 후
다이나믹셀을 변화하는 가변저항값에 맞추어 다이나믹셀을 제어하는 과제입니다.

### 핵심 목표
* ATmega128 레지스터 설정과 UART와 MAX485를 통한 다이나믹셀 제어
* UART 활성화 방법과 송수신 방법, 터미널 출력 방법 등 활용

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 / UART|
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, DYNAMIXEL MX-64R |

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
PE0 -----------------------> RO
PE2 -----------------------> RE, DE
PE1 -----------------------> DI
VCC -----------------------> 5V
D- ------------------------> DYNAMIXEL DATA-
D+ ------------------------> DYNAMIXEL DATA+
GND -----------------------> GND
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
├── Day03_Task03/
   ├── Day03_Task03.atsln #Atmel Studio(현재는 Microchip Studio) 솔루션 파일
   ├── Day03_Task03.cproj # MSBuild 기반 프로젝트 파일
   ├── main.c # 메인 제어 루프 및 시스템 초기화
   ├── i2c_lcd.c #lcd와 i2c 관련 소스 파일
   ├── i2c_lcd.h #lcd와 i2c 관련 헤 파일
   └── README.md #과제 보고서
```

---

## 5. 핵심 코드 및 레지스터 설정 (Key Implementation)

### CRC16 계산 (Dynamixel Protocol 2.0 무결성 검증)
```c
static uint16_t crc16_dxl_update(uint16_t seed, const uint8_t *buf, uint16_t len)
{
	uint16_t crc = seed;
	uint16_t idx;
	uint8_t bit;

	for (idx = 0; idx < len; idx++)
	{
		crc = (uint16_t)(crc ^ ((uint16_t)buf[idx] << 8));

		for (bit = 0; bit < 8; bit++)
		{
			crc = (crc & 0x8000)
			? (uint16_t)((crc << 1) ^ 0x8005)
			: (uint16_t)(crc << 1);
		}
	}

	return crc;
}
```
### 패킷 조립 + 바이트 스터핑 (Protocol 2.0 핵심)
```c
/* 0xFF 0xFF 0xFD 시퀀스가 나타나면 0xFD를 삽입 (Byte Stuffing) */
for (k = 0; k < inst_len; k++)
{
	stuffed[stuffed_len++] = inst[k];

	if (stuffed_len >= 3 &&
	stuffed[stuffed_len - 3] == 0xFF &&
	stuffed[stuffed_len - 2] == 0xFF &&
	stuffed[stuffed_len - 1] == 0xFD)
	{
		stuffed[stuffed_len++] = 0xFD;
	}
}

```

### ADC 다중 샘플링 평균 (노이즈 제거)
```c
static uint16_t pot_adc_average(void)
{
	uint32_t acc = 0;
	uint8_t n;

	for (n = 0; n < POT_SAMPLES; n++)
	{
		acc += pot_adc_sample();
	}

	return (uint16_t)(acc / POT_SAMPLES);
}
```

### 메인 루프: 이벤트 기반 속도 갱신 + 데드밴드 기반 위치 갱신
```c
if (pc_link_has_data())
{
	uint8_t rx_ch = pc_link_get();

	if (rx_ch >= '0' && rx_ch <= '9')
	{
		uint8_t level = (uint8_t)(rx_ch - '0');
		cur_speed = speed_level_to_value(level);
		dxl_write_u32(REG_PROFILE_VELOCITY, cur_speed);
		resend_pos_flag = 1;
	}
}

cur_pos = pot_adc_average();

if (resend_pos_flag || u16_diff(cur_pos, sent_pos) >= POS_DEADBAND)
{
	dxl_write_u32(REG_GOAL_POSITION, cur_pos);
	sent_pos = cur_pos;
	resend_pos_flag = 0;
}
```
---

## 6. 동작 설명 및 결과 (Results)

### 동작 시나리오
1. 시스템 초기화(기본 속도 100)
2. Reset botton 한번 누르기
3. 시리얼 통신에 0~9 입력 -> 속도 설정
4. 가변 저항 조절 -> 모터 작

### 동작 사진 / 영상

| 정면 동작 모습 | 
| https://drive.google.com/file/d/1wH1yrE4oK9epUxyclKq0NSAmCye04hFY/view?usp=drive_link | 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **Chat GPT** | MAX485 연결 이상 | - 과제2를 진행할 때 MAX 485를 빼면 시리얼 통신이 되고 꽂으면 시리얼 통신이 안되던 이슈가 있어 혹시 회로 연결에 문제가 있는지 물어보았습니다. 
| **Chat GPT** | 전체적인 핵심 코드 구성 | - 이번 과제에서는 AI에게 핵심 코드의 논리 아이디어를 많이 참고하여 작성하였습니다. 핀 설정은 DataSheet를 찾아가며 스스로 설정하였지만 다이나믹셀과 연결시키는 방법에 관해서는 AI의 도움을 받았습니다.  

### AI 활용 및 검증 원칙
1. **학습 주도성:** 이번 과제는 핵심 코드도 AI에게 도움을 받아 작성하였습니다. 

