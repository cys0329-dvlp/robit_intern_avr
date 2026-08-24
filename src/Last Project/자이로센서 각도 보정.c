#define F_CPU 16000000UL
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD) - 1)

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>

// ============================================================
// 설정
// ============================================================

#define MPU6050_ADDR       0x68
#define MPU6050_PWR_MGMT1  0x6B
#define MPU6050_GYRO_ZOUT  0x47

// 실제 90도 회전 → 약 30도 측정 보정
#define GYRO_SCALE         2.7


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
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART0_TX(char data)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

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
FDEV_SETUP_STREAM(UART0_putchar, NULL, _FDEV_SETUP_WRITE);


// ============================================================
// TWI(I2C)
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

uint8_t TWI_Start(void)
{
	TWCR = (1 << TWINT) |
	(1 << TWSTA) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));

	uint8_t status = TWSR & 0xF8;

	if (status == 0x08 || status == 0x10)
	return 1;

	return 0;
}

void TWI_Stop(void)
{
	TWCR = (1 << TWINT) |
	(1 << TWEN) |
	(1 << TWSTO);
}

uint8_t TWI_Write(uint8_t data)
{
	TWDR = data;

	TWCR = (1 << TWINT) |
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

uint8_t TWI_Read_ACK(void)
{
	TWCR = (1 << TWINT) |
	(1 << TWEN) |
	(1 << TWEA);

	while (!(TWCR & (1 << TWINT)));

	return TWDR;
}

uint8_t TWI_Read_NACK(void)
{
	TWCR = (1 << TWINT) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));

	return TWDR;
}


// ============================================================
// MPU6050
// ============================================================

void MPU6050_Write(uint8_t reg, uint8_t data)
{
	TWI_Start();

	TWI_Write(MPU6050_ADDR << 1);
	TWI_Write(reg);
	TWI_Write(data);

	TWI_Stop();
}

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
	TWI_Write((MPU6050_ADDR << 1) | 0x01);

	// High byte
	high = TWI_Read_ACK();

	// Low byte
	low = TWI_Read_NACK();

	TWI_Stop();

	return ((int16_t)high << 8) | low;
}

void MPU6050_Init(void)
{
	// Sleep Mode 해제
	MPU6050_Write(MPU6050_PWR_MGMT1, 0x00);

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
		gz = MPU6050_Read16(MPU6050_GYRO_ZOUT);

		sum += gz;

		_delay_ms(5);
	}

	return (int16_t)(sum / 500);
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


	// UART 초기화
	UART0_Init();

	// printf 연결
	stdout = &UART0_OUTPUT;

	// TWI 초기화
	TWI_Init();

	_delay_ms(100);

	printf("\n");
	printf("MPU6050 GYRO Z TEST\n");

	// MPU6050 초기화
	MPU6050_Init();

	printf("Keep sensor still...\n");


	// ========================================================
	// 영점 보정
	// ========================================================

	gyro_z_offset = MPU6050_Calibrate_GyroZ();

	printf("Offset: %d\n", gyro_z_offset);
	printf("Calibration Complete!\n\n");


	// ========================================================
	// MAIN LOOP
	// ========================================================

	while (1)
	{
		// Z축 자이로 Raw 값 읽기
		gz_raw = MPU6050_Read16(MPU6050_GYRO_ZOUT);


		// ----------------------------------------------------
		// 영점 오차 제거
		// ----------------------------------------------------

		gz_raw -= gyro_z_offset;


		// ----------------------------------------------------
		// Raw → °/s 변환
		//
		// ±250°/s 설정
		// 131 LSB = 1°/s
		// ----------------------------------------------------

		gz_dps = gz_raw / 131.0;


		// ----------------------------------------------------
		// 회전각 누적
		//
		// 기본 계산:
		// angle = 각속도 × 시간
		//
		// 실제 90도 → 약 30도 측정되므로
		// 3배 보정
		// ----------------------------------------------------

		angle_z += gz_dps * 0.01 * GYRO_SCALE;


		// ----------------------------------------------------
		// 0 ~ 360도 범위 유지
		//
		// 예:
		// 360 → 0
		// 400 → 40
		// -10 → 350
		// ----------------------------------------------------

		while (angle_z >= 360.0)
		{
			angle_z -= 360.0;
		}

		while (angle_z < 0.0)
		{
			angle_z += 360.0;
		}


		// ----------------------------------------------------
		// UART 출력
		// ----------------------------------------------------

		printf("Angle Z: %d deg\n", (int)angle_z);


		_delay_ms(10);
	}
}
