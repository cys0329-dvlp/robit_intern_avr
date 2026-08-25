#define F_CPU 16000000UL
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD) - 1)

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>


// ============================================================
// MPU6050 설정
// ============================================================

#define MPU6050_ADDR       0x68
#define MPU6050_PWR_MGMT1  0x6B
#define MPU6050_GYRO_ZOUT  0x47

// 실제 90도 회전 → 약 30도 측정 보정
#define GYRO_SCALE         6.3


// ============================================================
// 각도 리셋 스위치
// ============================================================

// PD2에 스위치 연결
// 스위치 반대쪽은 GND
#define RESET_SWITCH PD2


// ============================================================
// LCD I2C 설정
// ============================================================

#define LCD_I2C_ADDR 0x27

#define LCD_RS 0x01
#define LCD_RW 0x02
#define LCD_EN 0x04
#define LCD_BL 0x08


// ============================================================
// UART0
// ============================================================

void UART0_Init(void)
{
	UBRR0H = (unsigned char)(UBRR_VALUE >> 8);
	UBRR0L = (unsigned char)UBRR_VALUE;

	UCSR0A = 0x00;

	// 송신 활성화
	UCSR0B = (1 << TXEN0);

	// 8bit, 1 Stop bit, No Parity
	UCSR0C =
	(1 << UCSZ01) |
	(1 << UCSZ00);
}


// ============================================================
// UART 문자 전송
// ============================================================

void UART0_TX(char data)
{
	while (!(UCSR0A & (1 << UDRE0)));

	UDR0 = data;
}


// ============================================================
// UART printf 연결
// ============================================================

int UART0_putchar(char c, FILE *stream)
{
	if (c == '\n')
	{
		UART0_TX('\r');
	}

	UART0_TX(c);

	return 0;
}


FILE UART0_OUTPUT =
FDEV_SETUP_STREAM(
UART0_putchar,
NULL,
_FDEV_SETUP_WRITE
);


// ============================================================
// TWI(I2C) 초기화
// ============================================================

void TWI_Init(void)
{
	// Prescaler = 1
	TWSR = 0x00;

	// 약 100kHz
	TWBR = 72;

	// TWI 활성화
	TWCR = (1 << TWEN);
}


// ============================================================
// TWI START
// ============================================================

uint8_t TWI_Start(void)
{
	TWCR =
	(1 << TWINT) |
	(1 << TWSTA) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));

	uint8_t status = TWSR & 0xF8;

	if (status == 0x08 ||
	status == 0x10)
	{
		return 1;
	}

	return 0;
}


// ============================================================
// TWI STOP
// ============================================================

void TWI_Stop(void)
{
	TWCR =
	(1 << TWINT) |
	(1 << TWEN) |
	(1 << TWSTO);
}


// ============================================================
// TWI 데이터 전송
// ============================================================

uint8_t TWI_Write(uint8_t data)
{
	TWDR = data;

	TWCR =
	(1 << TWINT) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));

	uint8_t status = TWSR & 0xF8;

	if (status == 0x18 ||
	status == 0x28 ||
	status == 0x40)
	{
		return 1;
	}

	return 0;
}


// ============================================================
// TWI 데이터 읽기 ACK
// ============================================================

uint8_t TWI_Read_ACK(void)
{
	TWCR =
	(1 << TWINT) |
	(1 << TWEN) |
	(1 << TWEA);

	while (!(TWCR & (1 << TWINT)));

	return TWDR;
}


// ============================================================
// TWI 데이터 읽기 NACK
// ============================================================

uint8_t TWI_Read_NACK(void)
{
	TWCR =
	(1 << TWINT) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));

	return TWDR;
}


// ============================================================
// MPU6050 데이터 쓰기
// ============================================================

void MPU6050_Write(uint8_t reg, uint8_t data)
{
	TWI_Start();

	// MPU6050 + Write
	TWI_Write(MPU6050_ADDR << 1);

	// 레지스터
	TWI_Write(reg);

	// 데이터
	TWI_Write(data);

	TWI_Stop();
}


// ============================================================
// MPU6050 16bit 데이터 읽기
// ============================================================

int16_t MPU6050_Read16(uint8_t reg)
{
	uint8_t high;
	uint8_t low;


	// START
	TWI_Start();


	// MPU6050 + Write
	TWI_Write(MPU6050_ADDR << 1);


	// 읽을 레지스터
	TWI_Write(reg);


	// Repeated START
	TWI_Start();


	// MPU6050 + Read
	TWI_Write(
	(MPU6050_ADDR << 1) | 0x01
	);


	// High byte
	high = TWI_Read_ACK();


	// Low byte
	low = TWI_Read_NACK();


	TWI_Stop();


	return ((int16_t)high << 8) | low;
}


// ============================================================
// MPU6050 초기화
// ============================================================

void MPU6050_Init(void)
{
	// Sleep Mode 해제
	MPU6050_Write(
	MPU6050_PWR_MGMT1,
	0x00
	);

	_delay_ms(100);
}


// ============================================================
// Gyro Z 영점 보정
// ============================================================

int16_t MPU6050_Calibrate_GyroZ(void)
{
	long sum = 0;

	int16_t gz;


	// 500번 평균
	for (uint16_t i = 0; i < 500; i++)
	{
		gz = MPU6050_Read16(
		MPU6050_GYRO_ZOUT
		);

		sum += gz;

		_delay_ms(5);
	}


	return (int16_t)(sum / 500);
}


// ============================================================
// 각도 리셋 스위치 초기화
//
// PD2 입력
// 내부 풀업 사용
//
// 연결:
//
// PD2 ---- 스위치 ---- GND
//
// 평상시 = HIGH
// 누름    = LOW
// ============================================================

void ResetSwitch_Init(void)
{
	// PD2 입력
	DDRD &= ~(1 << RESET_SWITCH);

	// 내부 풀업 활성화
	PORTD |= (1 << RESET_SWITCH);
}


// ============================================================
// 각도 리셋 스위치 확인
//
// 눌림 = 1
// 안 눌림 = 0
// ============================================================

uint8_t ResetSwitch_Pressed(void)
{
	// 스위치가 눌리면 LOW
	if (!(PIND & (1 << RESET_SWITCH)))
	{
		// 디바운싱
		_delay_ms(20);

		// 다시 확인
		if (!(PIND & (1 << RESET_SWITCH)))
		{
			return 1;
		}
	}

	return 0;
}


// ============================================================
// LCD I2C Expander Write
// ============================================================

void LCD_ExpanderWrite(uint8_t data)
{
	TWI_Start();

	// LCD I2C 주소 + Write
	TWI_Write(
	(LCD_I2C_ADDR << 1) | 0
	);

	TWI_Write(data | LCD_BL);

	TWI_Stop();
}


// ============================================================
// LCD Enable
// ============================================================

void LCD_Enable(uint8_t data)
{
	LCD_ExpanderWrite(
	data | LCD_EN
	);

	_delay_us(1);

	LCD_ExpanderWrite(
	data & ~LCD_EN
	);

	_delay_us(50);
}


// ============================================================
// LCD 4bit 전송
// ============================================================

void LCD_Send4Bit(uint8_t data)
{
	LCD_ExpanderWrite(data);

	LCD_Enable(data);
}


// ============================================================
// LCD Command
// ============================================================

void LCD_SendCommand(uint8_t command)
{
	uint8_t high;
	uint8_t low;


	high =
	command & 0xF0;

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


// ============================================================
// LCD Data
// ============================================================

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


// ============================================================
// LCD 초기화
// ============================================================

void LCD_Init(void)
{
	_delay_ms(50);


	LCD_Send4Bit(0x30);

	_delay_ms(5);


	LCD_Send4Bit(0x30);

	_delay_us(150);


	LCD_Send4Bit(0x30);


	LCD_Send4Bit(0x20);


	// 4bit / 2 Line
	LCD_SendCommand(0x28);

	// Display ON
	LCD_SendCommand(0x0C);

	// Entry Mode
	LCD_SendCommand(0x06);

	// Clear
	LCD_SendCommand(0x01);

	_delay_ms(2);
}


// ============================================================
// LCD Cursor
// ============================================================

void LCD_SetCursor(
uint8_t row,
uint8_t col
)
{
	uint8_t address;


	if (row == 0)
	{
		address =
		0x80 + col;
	}
	else
	{
		address =
		0xC0 + col;
	}


	LCD_SendCommand(address);
}


// ============================================================
// LCD 문자열 출력
// ============================================================

void LCD_Print(const char *str)
{
	while (*str)
	{
		LCD_SendData(*str++);
	}
}


// ============================================================
// LCD 부호 포함 정수 출력
// ============================================================

void LCD_PrintInt(int16_t value)
{
	if (value < 0)
	{
		LCD_SendData('-');

		value = -value;
	}

	LCD_PrintUInt((uint16_t)value);
}


// ============================================================
// LCD 숫자 출력
// ============================================================

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
		LCD_SendData(
		buffer[--i]
		);
	}
}


// ============================================================
// LCD 현재 Z축 각도 출력
//
// 범위:
// -180° ~ +180°
// ============================================================

void LCD_DisplayAngle(float angle)
{
	int16_t angle_int;


	// 정수로 변환
	if (angle >= 0)
	{
		angle_int =
		(int16_t)(angle + 0.5);
	}
	else
	{
		angle_int =
		(int16_t)(angle - 0.5);
	}


	// 첫 번째 줄
	LCD_SetCursor(0, 0);

	LCD_Print("GYRO Z ANGLE");


	// 두 번째 줄
	LCD_SetCursor(1, 0);

	LCD_Print("Angle: ");

	LCD_PrintInt(angle_int);

	LCD_Print(" deg");
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
	int16_t gz_raw;

	int16_t gyro_z_offset;

	float gz_dps;

	float angle_z = 0.0;


	// ========================================================
	// UART 초기화
	// ========================================================

	UART0_Init();

	// printf 연결
	stdout = &UART0_OUTPUT;


	// ========================================================
	// TWI 초기화
	// ========================================================

	TWI_Init();

	_delay_ms(100);


	// ========================================================
	// 각도 리셋 스위치 초기화
	// ========================================================

	ResetSwitch_Init();


	// ========================================================
	// LCD 초기화
	// ========================================================

	LCD_Init();


	LCD_SetCursor(0, 0);
	LCD_Print("MPU6050");


	LCD_SetCursor(1, 0);
	LCD_Print("GYRO Z ANGLE");


	_delay_ms(1000);


	// ========================================================
	// UART 시작 메시지
	// ========================================================

	printf("\n");

	printf("MPU6050 GYRO Z TEST\n");


	// ========================================================
	// MPU6050 초기화
	// ========================================================

	MPU6050_Init();


	printf("Keep sensor still...\n");


	// ========================================================
	// Gyro Z 영점 보정
	// ========================================================

	gyro_z_offset =
	MPU6050_Calibrate_GyroZ();


	printf(
	"Offset: %d\n",
	gyro_z_offset
	);

	printf(
	"Calibration Complete!\n\n"
	);


	// ========================================================
	// MAIN LOOP
	// ========================================================

	while (1)
	{
		// ====================================================
		// PD2 스위치 확인
		//
		// 누르면 현재 방향을 0도로 초기화
		// ====================================================

		if (ResetSwitch_Pressed())
		{
			// 현재 각도 = 0°
			angle_z = 0.0;


			// LCD 즉시 0° 표시
			LCD_DisplayAngle(angle_z);


			// UART 출력
			printf(
			"ANGLE RESET -> 0 deg\n"
			);


			// 스위치를 뗄 때까지 대기
			while (!(PIND & (1 << RESET_SWITCH)))
			{
				_delay_ms(10);
			}
		}


		// ====================================================
		// Z축 자이로 Raw 값 읽기
		// ====================================================

		gz_raw =
		MPU6050_Read16(
		MPU6050_GYRO_ZOUT
		);


		// ====================================================
		// 영점 오차 제거
		// ====================================================

		gz_raw -=
		gyro_z_offset;


		// ====================================================
		// Raw → °/s
		//
		// ±250°/s
		// 131 LSB = 1°/s
		// ====================================================

		gz_dps =
		gz_raw / 131.0;


		// ====================================================
		// 회전각 누적
		//
		// 오른쪽 = +
		// 왼쪽   = -
		//
		// 10ms 주기
		// 실제 측정값 보정 2.7배
		// ====================================================

		angle_z -=
		gz_dps *
		0.01 *
		GYRO_SCALE;


		// ====================================================
		// -180° ~ +180° 범위 유지
		//
		// 예:
		// +270° → -90°
		// -270° → +90°
		// ====================================================

		while (angle_z > 180.0)
		{
			angle_z -= 360.0;
		}


		while (angle_z <= -180.0)
		{
			angle_z += 360.0;
		}


		// ====================================================
		// UART 출력
		// ====================================================

		printf(
		"Angle Z: %d deg\n",
		(int)angle_z
		);


		// ====================================================
		// LCD 출력
		// ====================================================

		LCD_DisplayAngle(angle_z);


		// ====================================================
		// 10ms 대기
		// ====================================================

		_delay_ms(10);
	}
}
