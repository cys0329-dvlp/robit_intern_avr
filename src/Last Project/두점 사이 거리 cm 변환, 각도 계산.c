#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>


// =====================================================
// 설정
// =====================================================

#define BAUD 9600
#define UBRR_VALUE ((F_CPU / 16UL / BAUD) - 1)

#define MAX_ID 6

// 픽셀 → cm 변환값
#define CM_PER_PIXEL 0.278

// 각도 계산용 PI
#define PI 3.14159265359


// =====================================================
// LCD I2C 주소
// =====================================================

#define LCD_I2C_ADDR 0x27


// =====================================================
// LED
// =====================================================

#define LED_ID5 PA5
#define LED_ID6 PA3


// =====================================================
// HuskyLens
// =====================================================

#define HUSKY_HEADER1        0x55
#define HUSKY_HEADER2        0xAA
#define HUSKY_ADDRESS        0x11

#define HUSKY_REQUEST_BLOCKS 0x21
#define HUSKY_RETURN_INFO    0x29
#define HUSKY_RETURN_BLOCK   0x2A

#define UART_TIMEOUT 60000UL


// =====================================================
// ID 데이터
// ID5, ID6만 사용
// =====================================================

uint8_t id_detected[7];

uint16_t id_x[7];
uint16_t id_y[7];


// 픽셀 기준 거리
uint16_t id_distance = 0;


// cm 거리
// 10배 저장
// 예: 123 = 12.3cm
uint16_t distance_cm_x10 = 0;


// ID5 기준 ID6 각도
int16_t id_angle = 0;


// =====================================================
// LED 초기화
// =====================================================

void LED_Init(void)
{
	DDRA |= (1 << LED_ID5);
	DDRA |= (1 << LED_ID6);

	PORTA &= ~(1 << LED_ID5);
	PORTA &= ~(1 << LED_ID6);
}


// =====================================================
// LED 업데이트
// =====================================================

void LED_Update(void)
{
	// ID5
	if (id_detected[5])
	{
		PORTA |= (1 << LED_ID5);
	}
	else
	{
		PORTA &= ~(1 << LED_ID5);
	}


	// ID6
	if (id_detected[6])
	{
		PORTA |= (1 << LED_ID6);
	}
	else
	{
		PORTA &= ~(1 << LED_ID6);
	}
}


// =====================================================
// UART0 초기화
// =====================================================

void UART0_Init(void)
{
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)UBRR_VALUE;

	UCSR0A = 0x00;

	UCSR0B =
	(1 << RXEN0) |
	(1 << TXEN0);

	UCSR0C =
	(1 << UCSZ01) |
	(1 << UCSZ00);
}


// =====================================================
// UART Byte 전송
// =====================================================

void UART0_SendByte(uint8_t data)
{
	while (!(UCSR0A & (1 << UDRE0)));

	UDR0 = data;
}


// =====================================================
// UART Byte 수신
// =====================================================

uint8_t UART0_ReadByteTimeout(uint8_t *data)
{
	uint32_t timeout = UART_TIMEOUT;

	while (!(UCSR0A & (1 << RXC0)))
	{
		if (--timeout == 0)
		{
			return 0;
		}
	}

	*data = UDR0;

	return 1;
}


// =====================================================
// UART 버퍼 초기화
// =====================================================

void UART0_ClearBuffer(void)
{
	while (UCSR0A & (1 << RXC0))
	{
		volatile uint8_t dummy = UDR0;
		(void)dummy;
	}
}


// =====================================================
// HuskyLens BLOCK 요청
// =====================================================

void HuskyLens_RequestBlocks(void)
{
	UART0_SendByte(0x55);
	UART0_SendByte(0xAA);
	UART0_SendByte(0x11);
	UART0_SendByte(0x00);
	UART0_SendByte(0x21);
	UART0_SendByte(0x31);
}


// =====================================================
// HuskyLens 패킷 수신
// =====================================================

uint8_t HuskyLens_ReadPacket(uint8_t *buffer)
{
	uint8_t data;
	uint8_t length;
	uint8_t index = 0;

	uint8_t checksum = 0;


	// Header 0x55 찾기
	while (1)
	{
		if (!UART0_ReadByteTimeout(&data))
		{
			return 0;
		}

		if (data == HUSKY_HEADER1)
		{
			break;
		}
	}

	buffer[index++] = data;


	// Header 0xAA 확인
	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	if (data != HUSKY_HEADER2)
	{
		return 0;
	}

	buffer[index++] = data;


	// Address 확인
	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	if (data != HUSKY_ADDRESS)
	{
		return 0;
	}

	buffer[index++] = data;


	// Length
	if (!UART0_ReadByteTimeout(&length))
	{
		return 0;
	}

	buffer[index++] = length;


	if (length > 30)
	{
		return 0;
	}


	// Command + Data
	for (uint8_t i = 0; i < length + 1; i++)
	{
		if (!UART0_ReadByteTimeout(&data))
		{
			return 0;
		}

		buffer[index++] = data;
	}


	// Checksum 수신
	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	buffer[index++] = data;


	// Checksum 계산
	for (uint8_t i = 0; i < index - 1; i++)
	{
		checksum += buffer[i];
	}


	if (checksum != buffer[index - 1])
	{
		return 0;
	}

	return 1;
}


// =====================================================
// 16bit Little Endian
// =====================================================

uint16_t GetUInt16(uint8_t low, uint8_t high)
{
	return ((uint16_t)high << 8) | low;
}


// =====================================================
// HuskyLens 데이터 업데이트
// ID5, ID6만 저장
// =====================================================

uint8_t HuskyLens_Update(void)
{
	uint8_t packet[40];

	uint16_t block_count;


	// ID5, ID6 검출 상태 초기화
	id_detected[5] = 0;
	id_detected[6] = 0;


	// UART 버퍼 초기화
	UART0_ClearBuffer();


	// BLOCK 요청
	HuskyLens_RequestBlocks();


	// RETURN_INFO 수신
	if (!HuskyLens_ReadPacket(packet))
	{
		return 0;
	}


	if (packet[4] != HUSKY_RETURN_INFO)
	{
		return 0;
	}


	// BLOCK 개수
	block_count =
	GetUInt16(packet[5], packet[6]);


	// BLOCK 데이터 수신
	for (uint16_t i = 0; i < block_count; i++)
	{
		uint16_t id;
		uint16_t x;
		uint16_t y;


		if (!HuskyLens_ReadPacket(packet))
		{
			break;
		}


		// BLOCK인지 확인
		if (packet[4] != HUSKY_RETURN_BLOCK)
		{
			continue;
		}


		// X 좌표
		x = GetUInt16(packet[5], packet[6]);


		// Y 좌표
		y = GetUInt16(packet[7], packet[8]);


		// ID
		id = GetUInt16(packet[13], packet[14]);


		// ID5, ID6만 저장
		if (id == 5 || id == 6)
		{
			id_detected[id] = 1;

			id_x[id] = x;
			id_y[id] = y;
		}
	}

	return 1;
}


// =====================================================
// ID5 - ID6 픽셀 거리 계산
//
// 거리 = √((X6-X5)² + (Y6-Y5)²)
// =====================================================

uint16_t CalculateDistance(void)
{
	int32_t dx;
	int32_t dy;

	uint32_t distance_squared;


	if (!id_detected[5] || !id_detected[6])
	{
		return 0;
	}


	// X축 차이
	dx = (int32_t)id_x[6] - (int32_t)id_x[5];


	// Y축 차이
	dy = (int32_t)id_y[6] - (int32_t)id_y[5];


	// 피타고라스 공식
	distance_squared =
	(uint32_t)(dx * dx) +
	(uint32_t)(dy * dy);


	return (uint16_t)sqrt((double)distance_squared);
}


// =====================================================
// 픽셀 거리 → cm 변환
//
// 1 pixel = 0.278 cm
//
// 결과는 0.1cm 단위
// 예: 123 → 12.3cm
// =====================================================

uint16_t CalculateDistanceCMx10(void)
{
	double distance_cm;


	if (!id_detected[5] || !id_detected[6])
	{
		return 0;
	}


	distance_cm =
	(double)id_distance * CM_PER_PIXEL;


	// 0.1cm 단위로 변환
	return (uint16_t)(distance_cm * 10.0 + 0.5);
}


// =====================================================
// ID5 기준 ID6의 각도 계산
//
// atan2(dy, dx)
//
// 화면 기준:
//
//            270°
//              ↑
//              |
// 180°  ← ----+---- → 0°
//              |
//              ↓
//             90°
//
// ID5를 기준점으로 ID6 방향 계산
// =====================================================

int16_t CalculateAngle(void)
{
    int32_t dx;
    int32_t dy;

    double angle;

    if (!id_detected[5] || !id_detected[6])
    {
        return 0;
    }

    // ID5 → ID6
    dx = (int32_t)id_x[6] - (int32_t)id_x[5];
    dy = (int32_t)id_y[6] - (int32_t)id_y[5];

    /*
        원하는 기준

               90
                ↑
                |
        0 ← ----+---- → 180
                |
                ↓
               270

        화면 좌표는 Y가 아래로 증가하므로
        atan2(-dy, -dx)를 사용
    */

    angle = atan2(
                (double)(-dy),
                (double)(-dx)
            ) * 180.0 / PI;

    // 음수면 0~360도로 변환
    if (angle < 0)
    {
        angle += 360.0;
    }

    return (int16_t)(angle + 0.5);
}


// =====================================================
// I2C / TWI
// =====================================================

#define TWBR_VALUE 72


void I2C_Init(void)
{
	DDRD &= ~(1 << PD0);
	DDRD &= ~(1 << PD1);

	PORTD |= (1 << PD0);
	PORTD |= (1 << PD1);

	TWBR = TWBR_VALUE;

	TWSR = 0x00;

	TWCR = (1 << TWEN);
}


// =====================================================
// I2C START
// =====================================================

uint8_t I2C_Start(uint8_t address)
{
	TWCR =
	(1 << TWINT) |
	(1 << TWSTA) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));


	TWDR = address;


	TWCR =
	(1 << TWINT) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));


	return 1;
}


// =====================================================
// I2C STOP
// =====================================================

void I2C_Stop(void)
{
	TWCR =
	(1 << TWINT) |
	(1 << TWEN) |
	(1 << TWSTO);

	_delay_us(10);
}


// =====================================================
// I2C 데이터 전송
// =====================================================

void I2C_Write(uint8_t data)
{
	TWDR = data;

	TWCR =
	(1 << TWINT) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));
}


// =====================================================
// LCD
// =====================================================

#define LCD_RS 0x01
#define LCD_RW 0x02
#define LCD_EN 0x04
#define LCD_BL 0x08


void LCD_ExpanderWrite(uint8_t data)
{
	I2C_Start((LCD_I2C_ADDR << 1) | 0);

	I2C_Write(data | LCD_BL);

	I2C_Stop();
}


// =====================================================
// LCD Enable
// =====================================================

void LCD_Enable(uint8_t data)
{
	LCD_ExpanderWrite(data | LCD_EN);

	_delay_us(1);

	LCD_ExpanderWrite(data & ~LCD_EN);

	_delay_us(50);
}


// =====================================================
// LCD 4bit
// =====================================================

void LCD_Send4Bit(uint8_t data)
{
	LCD_ExpanderWrite(data);

	LCD_Enable(data);
}


// =====================================================
// LCD Command
// =====================================================

void LCD_SendCommand(uint8_t command)
{
	uint8_t high;
	uint8_t low;


	high = command & 0xF0;

	low =
	(command << 4) & 0xF0;


	LCD_Send4Bit(high);

	LCD_Send4Bit(low);


	if (command == 0x01 ||
	command == 0x02)
	{
		_delay_ms(2);
	}
}


// =====================================================
// LCD Data
// =====================================================

void LCD_SendData(uint8_t data)
{
	uint8_t high;
	uint8_t low;


	high =
	(data & 0xF0) | LCD_RS;


	low =
	((data << 4) & 0xF0) | LCD_RS;


	LCD_Send4Bit(high);

	LCD_Send4Bit(low);
}


// =====================================================
// LCD 초기화
// =====================================================

void LCD_Init(void)
{
	_delay_ms(50);

	LCD_Send4Bit(0x30);
	_delay_ms(5);

	LCD_Send4Bit(0x30);
	_delay_us(150);

	LCD_Send4Bit(0x30);

	LCD_Send4Bit(0x20);

	LCD_SendCommand(0x28);

	LCD_SendCommand(0x0C);

	LCD_SendCommand(0x06);

	LCD_SendCommand(0x01);

	_delay_ms(2);
}


// =====================================================
// LCD Clear
// =====================================================

void LCD_Clear(void)
{
	LCD_SendCommand(0x01);

	_delay_ms(2);
}


// =====================================================
// LCD Cursor
// =====================================================

void LCD_SetCursor(uint8_t row, uint8_t col)
{
	uint8_t address;


	if (row == 0)
	{
		address = 0x80 + col;
	}
	else
	{
		address = 0xC0 + col;
	}


	LCD_SendCommand(address);
}


// =====================================================
// LCD 문자열 출력
// =====================================================

void LCD_Print(const char *str)
{
	while (*str)
	{
		LCD_SendData(*str++);
	}
}


// =====================================================
// LCD 숫자 출력
// =====================================================

void LCD_PrintUInt(uint16_t value)
{
	char buffer[6];

	uint8_t i = 0;


	if (value == 0)
	{
		LCD_SendData('0');

		return;
	}


	while (value > 0)
	{
		buffer[i++] =
		'0' + (value % 10);

		value /= 10;
	}


	while (i > 0)
	{
		LCD_SendData(buffer[--i]);
	}
}


// =====================================================
// LCD 거리 + 각도 출력
//
// 1번째 줄:
// DIST:20.0cm
//
// 2번째 줄:
// ANGLE:45deg
// =====================================================

void LCD_DisplayDistanceAngle(void)
{
	LCD_Clear();


	// -------------------------------------------------
	// ID5, ID6 둘 다 검출
	// -------------------------------------------------

	if (id_detected[5] && id_detected[6])
	{
		// ============================
		// 첫 번째 줄 : 거리
		// ============================

		LCD_SetCursor(0, 0);

		LCD_Print("DIST:");

		// 정수 부분
		LCD_PrintUInt(distance_cm_x10 / 10);

		LCD_SendData('.');

		// 소수 첫째 자리
		LCD_PrintUInt(distance_cm_x10 % 10);

		LCD_Print("cm");


		// ============================
		// 두 번째 줄 : 각도
		// ============================

		LCD_SetCursor(1, 0);

		LCD_Print("ANGLE:");

		LCD_PrintUInt((uint16_t)id_angle);

		LCD_Print("deg");
	}


	// -------------------------------------------------
	// 하나라도 검출되지 않음
	// -------------------------------------------------

	else
	{
		LCD_SetCursor(0, 0);

		LCD_Print("ID5/ID6");

		LCD_SetCursor(1, 0);

		LCD_Print("Searching...");
	}
}


// =====================================================
// MAIN
// =====================================================

int main(void)
{
	// -------------------------------------------------
	// 초기화
	// -------------------------------------------------

	LED_Init();

	UART0_Init();

	I2C_Init();

	LCD_Init();


	// -------------------------------------------------
	// HuskyLens 부팅 대기
	// -------------------------------------------------

	_delay_ms(2000);


	// -------------------------------------------------
	// 시작 화면
	// -------------------------------------------------

	LCD_Clear();

	LCD_SetCursor(0, 0);

	LCD_Print("HuskyLens");

	LCD_SetCursor(1, 0);

	LCD_Print("ID5 -> ID6");

	_delay_ms(1000);


	// =================================================
	// Main Loop
	// =================================================

	while (1)
	{
		// ---------------------------------------------
		// HuskyLens에서 ID5, ID6 좌표 수신
		// ---------------------------------------------

		HuskyLens_Update();


		// ---------------------------------------------
		// 두 ID 사이 픽셀 거리 계산
		// ---------------------------------------------

		id_distance =
		CalculateDistance();


		// ---------------------------------------------
		// 픽셀 → cm 변환
		// ---------------------------------------------

		distance_cm_x10 =
		CalculateDistanceCMx10();


		// ---------------------------------------------
		// ID5 기준 ID6의 각도 계산
		// ---------------------------------------------

		id_angle =
		CalculateAngle();


		// ---------------------------------------------
		// LED 업데이트
		// ---------------------------------------------

		LED_Update();


		// ---------------------------------------------
		// LCD에 거리 + 각도 실시간 출력
		// ---------------------------------------------

		LCD_DisplayDistanceAngle();


		// ---------------------------------------------
		// 0.1초마다 갱신
		// ---------------------------------------------

		_delay_ms(100);
	}
}
