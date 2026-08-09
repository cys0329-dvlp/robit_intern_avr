# ATmega128 과제 및 프로젝트 템플릿

> **광운대학교 로봇학부**  
> **작성자:** 최윤서
> **제출일:** 08/09

---

## 1. 개요 (Overview)
본 과제는 수발광부와 LCD, LED를 활용하여 수발광부의 ADC값을 LCD에 출력하고 그에 따라 LED를 출력시키는 과제입니다. 

### 핵심 목표
* 정규화 값이 0.8이상일 경우 LED 켬, 이하일 경우 끔
*  LCD에 정규화된 IR센서 값 띄우기

---

## 2. 개발 환경 (Environment)

| 항목 | 내용 |
| :--- | :--- |
| **MCU** | ATmega128A (16MHz External Crystal) |
| **IDE / Compiler** | Microchip Studio 7.0 / Microchip AVR GCC |
| **Flasher Tool** | USBISP / STK500 / UART|
| **언어** | C Language |
| **주요 부품** | ATmega128 개발보드, LCD(I2C), LED, IR센서|

---

## 3. 하드웨어 구성 및 핀 맵 (Hardware Structure)

### Pin Configuration

```text
[ATmega128]                 [Target Component]
IR센서 -----------------> PF2~7
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

### 필터링 + 정규화 계산
```c
	/* 1) 원본 값 읽기 : PF2~PF7 -> ADC channel 2~7 */
raw[i] = ADC_read(i + 2);

/* 2) 이동평균필터(MAF) 적용 */
maf_buf[i][maf_idx] = raw[i];
uint32_t sum = 0;
uint8_t  cnt = maf_filled ? MAF_SIZE : (maf_idx + 1);
for (uint8_t j = 0; j < cnt; j++) sum += maf_buf[i][j];
filtered[i] = (uint16_t)(sum / cnt);

/* 3) 누적 min / max 갱신 */
if (filtered[i] < ir_min[i]) ir_min[i] = filtered[i];
if (filtered[i] > ir_max[i]) ir_max[i] = filtered[i];

/* 4) 정규화 */
if (ir_max[i] > ir_min[i])
    norm[i] = (float)(filtered[i] - ir_min[i]) / (float)(ir_max[i] - ir_min[i]);
else
    norm[i] = 0.0f;
```

### LCD 출력
```c
LCD_setCursor(0, 0);
for (uint8_t i = 0; i < 3; i++) {
    dtostrf(norm[i], 4, 2, normStr);
    sprintf(buf, "%d:%s ", i, normStr);
    LCD_print(buf);
}

LCD_setCursor(0, 1);
for (uint8_t i = 3; i < 6; i++) {
    dtostrf(norm[i], 4, 2, normStr);
    sprintf(buf, "%d:%s ", i, normStr);
    LCD_print(buf);
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

### LED 제어
```c
LCD_setCursor(0, 0);
for (uint8_t i = 0; i < 3; i++) {
    dtostrf(norm[i], 4, 2, normStr);
    sprintf(buf, "%d:%s ", i, normStr);
    LCD_print(buf);
}

LCD_setCursor(0, 1);
for (uint8_t i = 3; i < 6; i++) {
    dtostrf(norm[i], 4, 2, normStr);
    sprintf(buf, "%d:%s ", i, normStr);
    LCD_print(buf);
}
```

### 동작 사진 / 영상

| 정면 동작 모습 | 
|(https://drive.google.com/file/d/19Se1A0pqSLPAQ1b6_P5uL_9S5MNSjgy1/view?usp=drive_link)| 

---

## 7. AI 툴 활용 명시 (AI Tools Declaration)
본 과제 작성 및 구현 과정에서 활용한 AI 도구(Generative AI)의 사용 현황 및 목적은 다음과 같음.

| 도구명 (Tool) | 활용 영역 | 세부 사용 목적 및 내용 |
| :--- | :--- | :--- |
| **claude** | 전체적인 예제 형식 작성 요청 | - 회로 연결 정보를 주어 간단한 코드 틀 작성을 요청했습니다.
| **claude** | LCD 출력 | - IR센서에서 나오는 정규화 값을 LCD에 어떻게 출력해야하는지 질문했습니다.


### AI 활용 및 검증 원칙
1. **학습 주도성:** 코드의 핵심 제어 로직 설계는 직접 작성하였으며, AI는 막히는 문제점에 관한 힌트를 간접적으로 받았고 정답과 코드를 대신 작성해주지 말라고 명령함
