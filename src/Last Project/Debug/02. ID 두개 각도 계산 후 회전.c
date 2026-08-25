#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>


// ============================================================
// 기본 설정
// ============================================================

#define BAUD 9600
#define UBRR_VALUE ((F_CPU / 16UL / BAUD) - 1)

#define PI 3.14159265359


// ============================================================
// MPU6050
// ============================================================

#define MPU6050_ADDR       0x68
#define MPU6050_PWR_MGMT1  0x6B
#define MPU6050_GYRO_ZOUT  0x47

#define GYRO_SCALE         6.3


// ============================================================
// 각도 리셋 스위치
// ============================================================

#define RESET_SWITCH PD2


// ============================================================
// LCD
// ============================================================

#define LCD_I2C_ADDR 0x27

#define LCD_RS 0x01
#define LCD_RW 0x02
#define LCD_EN 0x04
#define LCD_BL 0x08


// ============================================================
// HuskyLens
// ============================================================

#define HUSKY_HEADER1        0x55
#define HUSKY_HEADER2        0xAA
#define HUSKY_ADDRESS        0x11

#define HUSKY_REQUEST_BLOCKS 0x21
#define HUSKY_RETURN_INFO    0x29
#define HUSKY_RETURN_BLOCK   0x2A

#define UART_TIMEOUT 60000UL


// ============================================================
// 모터
//
// L298N
//
// 왼쪽 모터:
// PWM  PB5
// IN1  PB0
// IN2  PB1
//
// 오른쪽 모터:
// PWM  PB6
// IN3  PB2
// IN4  PB3
// ============================================================

#define LEFT_PWM   PB5
#define RIGHT_PWM  PB6

#define LEFT_IN1   PB0
#define LEFT_IN2   PB1

#define RIGHT_IN1  PB2
#define RIGHT_IN2  PB3


// ============================================================
// 회전 설정
// ============================================================

#define TURN_SPEED 250

#define ANGLE_TOLERANCE 2.0


// ============================================================
// LED 단계 표시
//
// PA0 = 단계 1
// PA1 = 단계 2
// PA2 = 단계 3
// PA3 = 단계 4
// PA4 = 단계 5
// PA5 = 단계 6
// PA6 = 단계 7
// PA7 = 단계 8
// ============================================================

#define STAGE_LED_PORT PORTA
#define STAGE_LED_DDR  DDRA


// ============================================================
// ID 데이터
// ============================================================

uint8_t id_detected[7];

uint16_t id_x[7];
uint16_t id_y[7];


// ============================================================
// ID5 → ID6 거리
// ============================================================

uint16_t id_distance = 0;


// ============================================================
// ID5 → ID6 각도
//
// 위       = 0°
// 오른쪽   = +90°
// 아래     = ±180°
// 왼쪽     = -90°
// ============================================================

int16_t id_angle = 0;


// ============================================================
// 회전 상태
// ============================================================

uint8_t rotation_active = 0;
uint8_t rotation_finished = 0;

float target_angle = 0.0;


// ============================================================
// IMU 현재 각도
// ============================================================

float angle_z = 0.0;


// ============================================================
// LED 단계 표시 함수
//
// stage:
// 1 → PA0
// 2 → PA1
// ...
// 8 → PA7
// ============================================================

void LED_SetStage(uint8_t stage)
{
	STAGE_LED_PORT = 0x00;


	if (stage >= 1 && stage <= 8)
	{
		STAGE_LED_PORT =
		(1 << (stage - 1));
	}
}


// ============================================================
// UART0 초기화
// ============================================================

void UART0_Init(void)
{
	UBRR0H =
	(uint8_t)(UBRR_VALUE >> 8);

	UBRR0L =
	(uint8_t)UBRR_VALUE;

	UCSR0A = 0x00;

	UCSR0B =
	(1 << RXEN0) |
	(1 << TXEN0);

	UCSR0C =
	(1 << UCSZ01) |
	(1 << UCSZ00);
}


// ============================================================
// UART 전송
// ============================================================

void UART0_SendByte(uint8_t data)
{
	while (!(UCSR0A & (1 << UDRE0)));

	UDR0 = data;
}


// ============================================================
// UART 수신
// ============================================================

uint8_t UART0_ReadByteTimeout(uint8_t *data)
{
	uint32_t timeout =
	UART_TIMEOUT;


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


// ============================================================
// UART 버퍼 삭제
// ============================================================

void UART0_ClearBuffer(void)
{
	while (UCSR0A & (1 << RXC0))
	{
		volatile uint8_t dummy =
		UDR0;

		(void)dummy;
	}
}


// ============================================================
// UART printf
// ============================================================

void UART0_TX(char data)
{
	UART0_SendByte(data);
}


int UART0_putchar(
char c,
FILE *stream
)
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
// TWI 초기화
// ============================================================

void TWI_Init(void)
{
	TWSR = 0x00;

	TWBR = 72;

	TWCR =
	(1 << TWEN);
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

	uint8_t status =
	TWSR & 0xF8;


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
// TWI Write
// ============================================================

uint8_t TWI_Write(uint8_t data)
{
	TWDR = data;

	TWCR =
	(1 << TWINT) |
	(1 << TWEN);

	while (!(TWCR & (1 << TWINT)));

	uint8_t status =
	TWSR & 0xF8;


	if (status == 0x18 ||
	status == 0x28 ||
	status == 0x40)
	{
		return 1;
	}

	return 0;
}


// ============================================================
// TWI Read ACK
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
// TWI Read NACK
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
// MPU6050 Write
// ============================================================

void MPU6050_Write(
uint8_t reg,
uint8_t data
)
{
	TWI_Start();

	TWI_Write(
	MPU6050_ADDR << 1
	);

	TWI_Write(reg);

	TWI_Write(data);

	TWI_Stop();
}


// ============================================================
// MPU6050 16bit Read
// ============================================================

int16_t MPU6050_Read16(uint8_t reg)
{
	uint8_t high;
	uint8_t low;


	TWI_Start();

	TWI_Write(
	MPU6050_ADDR << 1
	);

	TWI_Write(reg);

	TWI_Start();

	TWI_Write(
	(MPU6050_ADDR << 1) |
	0x01
	);


	high =
	TWI_Read_ACK();

	low =
	TWI_Read_NACK();

	TWI_Stop();


	return
	((int16_t)high << 8) |
	low;
}


// ============================================================
// MPU6050 초기화
// ============================================================

void MPU6050_Init(void)
{
	MPU6050_Write(
	MPU6050_PWR_MGMT1,
	0x00
	);

	_delay_ms(100);
}


// ============================================================
// Gyro Z Calibration
// ============================================================

int16_t MPU6050_Calibrate_GyroZ(void)
{
	long sum = 0;

	int16_t gz;


	for (uint16_t i = 0;
	i < 500;
	i++)
	{
		gz =
		MPU6050_Read16(
		MPU6050_GYRO_ZOUT
		);

		sum += gz;

		_delay_ms(5);
	}


	return
	(int16_t)(sum / 500);
}


// ============================================================
// Reset Switch 초기화
// ============================================================

void ResetSwitch_Init(void)
{
	DDRD &=
	~(1 << RESET_SWITCH);

	PORTD |=
	(1 << RESET_SWITCH);
}


// ============================================================
// Reset Switch 확인
// ============================================================

uint8_t ResetSwitch_Pressed(void)
{
	if (!(PIND & (1 << RESET_SWITCH)))
	{
		_delay_ms(20);

		if (!(PIND & (1 << RESET_SWITCH)))
		{
			return 1;
		}
	}

	return 0;
}


// ============================================================
// LED 초기화
// ============================================================

void LED_Init(void)
{
	// PA0~PA7 전체 출력
	STAGE_LED_DDR = 0xFF;

	// 모든 LED OFF
	STAGE_LED_PORT = 0x00;
}


// ============================================================
// HuskyLens BLOCK 요청
// ============================================================

void HuskyLens_RequestBlocks(void)
{
	UART0_SendByte(0x55);
	UART0_SendByte(0xAA);
	UART0_SendByte(0x11);
	UART0_SendByte(0x00);
	UART0_SendByte(0x21);
	UART0_SendByte(0x31);
}


// ============================================================
// HuskyLens Packet Read
// ============================================================

uint8_t HuskyLens_ReadPacket(
uint8_t *buffer
)
{
	uint8_t data;
	uint8_t length;
	uint8_t index = 0;

	uint8_t checksum = 0;


	// Header 1
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


	// Header 2
	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	if (data != HUSKY_HEADER2)
	{
		return 0;
	}

	buffer[index++] = data;


	// Address
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
	for (uint8_t i = 0;
	i < length + 1;
	i++)
	{
		if (!UART0_ReadByteTimeout(&data))
		{
			return 0;
		}

		buffer[index++] = data;
	}


	// Checksum
	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	buffer[index++] = data;


	// Checksum 계산
	for (uint8_t i = 0;
	i < index - 1;
	i++)
	{
		checksum += buffer[i];
	}


	if (checksum != buffer[index - 1])
	{
		return 0;
	}


	return 1;
}


// ============================================================
// UInt16 Little Endian
// ============================================================

uint16_t GetUInt16(
uint8_t low,
uint8_t high
)
{
	return
	((uint16_t)high << 8) |
	low;
}


// ============================================================
// HuskyLens 데이터 업데이트
// ============================================================

uint8_t HuskyLens_Update(void)
{
	uint8_t packet[40];

	uint16_t block_count;


	id_detected[5] = 0;
	id_detected[6] = 0;


	UART0_ClearBuffer();


	HuskyLens_RequestBlocks();


	// RETURN_INFO
	if (!HuskyLens_ReadPacket(packet))
	{
		return 0;
	}


	if (packet[4] != HUSKY_RETURN_INFO)
	{
		return 0;
	}


	block_count =
	GetUInt16(
	packet[5],
	packet[6]
	);


	// BLOCK
	for (uint16_t i = 0;
	i < block_count;
	i++)
	{
		uint16_t id;
		uint16_t x;
		uint16_t y;


		if (!HuskyLens_ReadPacket(packet))
		{
			break;
		}


		if (packet[4] != HUSKY_RETURN_BLOCK)
		{
			continue;
		}


		x =
		GetUInt16(
		packet[5],
		packet[6]
		);


		y =
		GetUInt16(
		packet[7],
		packet[8]
		);


		id =
		GetUInt16(
		packet[13],
		packet[14]
		);


		if (id == 5 ||
		id == 6)
		{
			id_detected[id] = 1;

			id_x[id] = x;
			id_y[id] = y;
		}
	}


	return 1;
}


// ============================================================
// ID5 → ID6 거리
// ============================================================

uint16_t CalculateDistance(void)
{
	int32_t dx;
	int32_t dy;

	uint32_t distance_squared;


	if (!id_detected[5] ||
	!id_detected[6])
	{
		return 0;
	}


	dx =
	(int32_t)id_x[6] -
	(int32_t)id_x[5];


	dy =
	(int32_t)id_y[6] -
	(int32_t)id_y[5];


	distance_squared =
	(uint32_t)(dx * dx) +
	(uint32_t)(dy * dy);


	return
	(uint16_t)sqrt(
	(double)distance_squared
	);
}


// ============================================================
// ID5 → ID6 각도
//
// 위       = 0°
// 오른쪽   = +90°
// 아래     = ±180°
// 왼쪽     = -90°
// ============================================================

int16_t CalculateAngle(void)
{
	int32_t dx;
	int32_t dy;

	double angle;


	if (!id_detected[5] ||
	!id_detected[6])
	{
		return 0;
	}


	dx =
	(int32_t)id_x[6] -
	(int32_t)id_x[5];


	dy =
	(int32_t)id_y[6] -
	(int32_t)id_y[5];


	angle =
	atan2(
	(double)dx,
	(double)(-dy)
	)
	* 180.0 / PI;


	if (angle > 180.0)
	{
		angle -= 360.0;
	}


	if (angle < -180.0)
	{
		angle += 360.0;
	}


	if (angle >= 0)
	{
		return
		(int16_t)(angle + 0.5);
	}
	else
	{
		return
		(int16_t)(angle - 0.5);
	}
}


// ============================================================
// LCD Expander
// ============================================================

void LCD_ExpanderWrite(uint8_t data)
{
	TWI_Start();

	TWI_Write(
	(LCD_I2C_ADDR << 1) | 0
	);

	TWI_Write(
	data | LCD_BL
	);

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
// LCD 4bit
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
	((data << 4) & 0xF0) |
	LCD_RS;


	LCD_Send4Bit(high);

	LCD_Send4Bit(low);
}


// ============================================================
// LCD Init
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


	LCD_SendCommand(0x28);

	LCD_SendCommand(0x0C);

	LCD_SendCommand(0x06);

	LCD_SendCommand(0x01);

	_delay_ms(2);
}


// ============================================================
// LCD Clear
// ============================================================

void LCD_Clear(void)
{
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
// LCD 문자열
// ============================================================

void LCD_Print(const char *str)
{
	while (*str)
	{
		LCD_SendData(*str++);
	}
}


// ============================================================
// LCD UInt
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
// LCD Int
// ============================================================

void LCD_PrintInt(int16_t value)
{
	if (value < 0)
	{
		LCD_SendData('-');

		value = -value;
	}

	LCD_PrintUInt(
	(uint16_t)value
	);
}


// ============================================================
// 모터 초기화
// ============================================================

void Motor_Init(void)
{
	DDRB |= (1 << LEFT_PWM);
	DDRB |= (1 << RIGHT_PWM);


	DDRB |= (1 << LEFT_IN1);
	DDRB |= (1 << LEFT_IN2);

	DDRB |= (1 << RIGHT_IN1);
	DDRB |= (1 << RIGHT_IN2);


	// Timer1 Fast PWM
	// TOP = 255
	// 약 976 Hz

	TCCR1A =
	(1 << COM1A1) |
	(1 << COM1B1) |
	(1 << WGM10);


	TCCR1B =
	(1 << WGM12) |
	(1 << CS11) |
	(1 << CS10);


	OCR1A = 0;
	OCR1B = 0;


	Motor_Stop();
}


// ============================================================
// 모터 정지
// ============================================================

void Motor_Stop(void)
{
	OCR1A = 0;
	OCR1B = 0;


	PORTB &= ~(1 << LEFT_IN1);
	PORTB &= ~(1 << LEFT_IN2);

	PORTB &= ~(1 << RIGHT_IN1);
	PORTB &= ~(1 << RIGHT_IN2);
}


// ============================================================
// 오른쪽 회전
// ============================================================

void Motor_TurnRight(uint8_t speed)
{
	// 왼쪽 전진
	PORTB |=
	(1 << LEFT_IN1);

	PORTB &=
	~(1 << LEFT_IN2);


	// 오른쪽 정지
	PORTB &=
	~(1 << RIGHT_IN1);

	PORTB &=
	~(1 << RIGHT_IN2);


	OCR1A = speed;
	OCR1B = 0;
}


// ============================================================
// 왼쪽 회전
// ============================================================

void Motor_TurnLeft(uint8_t speed)
{
	// 왼쪽 정지
	PORTB &=
	~(1 << LEFT_IN1);

	PORTB &=
	~(1 << LEFT_IN2);


	// 오른쪽 전진
	PORTB |=
	(1 << RIGHT_IN1);

	PORTB &=
	~(1 << RIGHT_IN2);


	OCR1A = 0;
	OCR1B = speed;
}


// ============================================================
// IMU 각도 정규화
// ============================================================

void NormalizeAngle(void)
{
	while (angle_z > 180.0)
	{
		angle_z -= 360.0;
	}


	while (angle_z <= -180.0)
	{
		angle_z += 360.0;
	}
}


// ============================================================
// IMU 각도 업데이트
// ============================================================

void MPU6050_UpdateAngle(
int16_t gyro_z_offset
)
{
	int16_t gz_raw;

	float gz_dps;


	gz_raw =
	MPU6050_Read16(
	MPU6050_GYRO_ZOUT
	);


	gz_raw -=
	gyro_z_offset;


	gz_dps =
	gz_raw / 131.0;


	// 오른쪽 = +
	// 왼쪽   = -

	angle_z -=
	gz_dps *
	0.01 *
	GYRO_SCALE;


	NormalizeAngle();
}


// ============================================================
// 회전 시작
// ============================================================

void StartRotation(void)
{
	// 회전 시작 순간을 0°
	angle_z = 0.0;


	target_angle =
	(float)id_angle;


	rotation_active = 1;
	rotation_finished = 0;


	// 단계 6
	// 회전 시작
	LED_SetStage(6);


	printf(
	"TARGET ANGLE: %d deg\n",
	id_angle
	);
}


// ============================================================
// 목표각 도달 여부
// ============================================================

uint8_t RotationReached(void)
{
	float error;


	error =
	target_angle -
	angle_z;


	while (error > 180.0)
	{
		error -= 360.0;
	}


	while (error <= -180.0)
	{
		error += 360.0;
	}


	if (fabs(error) <=
	ANGLE_TOLERANCE)
	{
		return 1;
	}


	return 0;
}


// ============================================================
// 회전 제어
// ============================================================

void Rotation_Control(void)
{
	if (!rotation_active)
	{
		return;
	}


	// ------------------------------------------------
	// 목표각 도달
	// ------------------------------------------------

	if (RotationReached())
	{
		Motor_Stop();

		rotation_active = 0;
		rotation_finished = 1;


		// 단계 7
		// 목표각 도달 → 정지
		LED_SetStage(7);


		printf(
		"ROTATION COMPLETE\n"
		);

		printf(
		"IMU ANGLE: %d deg\n",
		(int)angle_z
		);

		return;
	}


	// ------------------------------------------------
	// 목표각이 +이면 오른쪽
	// ------------------------------------------------

	if (target_angle > 0)
	{
		Motor_TurnRight(
		TURN_SPEED
		);
	}


	// ------------------------------------------------
	// 목표각이 -이면 왼쪽
	// ------------------------------------------------

	else if (target_angle < 0)
	{
		Motor_TurnLeft(
		TURN_SPEED
		);
	}


	// ------------------------------------------------
	// 목표각 = 0
	// ------------------------------------------------

	else
	{
		Motor_Stop();

		rotation_active = 0;
		rotation_finished = 1;

		LED_SetStage(7);
	}
}


// ============================================================
// LCD 상태 표시
// ============================================================

void LCD_DisplayStatus(void)
{
	LCD_Clear();


	// ========================================================
	// 회전 중
	// ========================================================

	if (rotation_active)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("TARGET:");

		LCD_PrintInt(
		(int16_t)target_angle
		);

		LCD_Print("deg");


		LCD_SetCursor(1, 0);

		LCD_Print("IMU:");

		LCD_PrintInt(
		(int16_t)angle_z
		);

		LCD_Print("deg");

		return;
	}


	// ========================================================
	// 회전 완료
	// ========================================================

	if (rotation_finished)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("ROTATION DONE");


		LCD_SetCursor(1, 0);

		LCD_Print("ANGLE:");

		LCD_PrintInt(
		(int16_t)angle_z
		);

		LCD_Print("deg");

		return;
	}


	// ========================================================
	// ID 검출
	// ========================================================

	if (id_detected[5] &&
	id_detected[6])
	{
		LCD_SetCursor(0, 0);

		LCD_Print("ID ANGLE:");

		LCD_PrintInt(
		id_angle
		);

		LCD_Print("deg");


		LCD_SetCursor(1, 0);

		LCD_Print("READY");
	}
	else
	{
		LCD_SetCursor(0, 0);

		LCD_Print("ID5/ID6");


		LCD_SetCursor(1, 0);

		LCD_Print("Searching...");
	}
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
	int16_t gyro_z_offset;


	// ========================================================
	// LED 초기화
	// ========================================================

	LED_Init();


	// ========================================================
	// 단계 1
	// 시스템 초기화
	// ========================================================

	LED_SetStage(1);


	UART0_Init();

	TWI_Init();

	ResetSwitch_Init();

	LCD_Init();

	Motor_Init();


	LCD_Clear();

	LCD_SetCursor(0, 0);

	LCD_Print("HuskyLens + IMU");

	LCD_SetCursor(1, 0);

	LCD_Print("Initializing...");


	printf("\n");
	printf("SYSTEM INITIALIZING\n");


	// ========================================================
	// 단계 2
	// MPU6050 초기화
	// ========================================================

	MPU6050_Init();


	LED_SetStage(2);


	printf(
	"MPU6050 INITIALIZED\n"
	);


	LCD_Clear();

	LCD_SetCursor(0, 0);

	LCD_Print("MPU6050");

	LCD_SetCursor(1, 0);

	LCD_Print("Initialized");


	_delay_ms(500);


	// ========================================================
	// HuskyLens 부팅 대기
	// ========================================================

	_delay_ms(2000);


	// ========================================================
	// 단계 3
	// Gyro Calibration
	// ========================================================

	LED_SetStage(3);


	LCD_Clear();

	LCD_SetCursor(0, 0);

	LCD_Print("GYRO CALIBRATE");

	LCD_SetCursor(1, 0);

	LCD_Print("Keep still");


	printf(
	"GYRO CALIBRATION\n"
	);

	printf(
	"Keep sensor still...\n"
	);


	gyro_z_offset =
	MPU6050_Calibrate_GyroZ();


	printf(
	"Offset: %d\n",
	gyro_z_offset
	);

	printf(
	"Calibration Complete!\n"
	);


	LCD_Clear();

	LCD_SetCursor(0, 0);

	LCD_Print("Calibration");

	LCD_SetCursor(1, 0);

	LCD_Print("Complete!");


	_delay_ms(1000);


	// ========================================================
	// IMU 시작각도
	// ========================================================

	angle_z = 0.0;


	// ========================================================
	// 단계 4
	// HuskyLens ID5 / ID6 탐색
	// ========================================================

	LED_SetStage(4);


	LCD_Clear();

	LCD_SetCursor(0, 0);

	LCD_Print("HuskyLens");

	LCD_SetCursor(1, 0);

	LCD_Print("Searching...");


	printf(
	"HUSKYLENS SEARCHING\n"
	);


	// ========================================================
	// MAIN LOOP
	// ========================================================

	while (1)
	{
		// ====================================================
		// PD2 리셋 스위치
		// ====================================================

		if (ResetSwitch_Pressed())
		{
			angle_z = 0.0;

			Motor_Stop();

			rotation_active = 0;
			rotation_finished = 0;


			printf(
			"IMU RESET -> 0 deg\n"
			);


			// 다시 ID 검색 단계
			LED_SetStage(4);


			LCD_Clear();

			LCD_SetCursor(0, 0);

			LCD_Print("IMU RESET");

			LCD_SetCursor(1, 0);

			LCD_Print("Angle: 0 deg");


			while (!(PIND &
			(1 << RESET_SWITCH)))
			{
				_delay_ms(10);
			}
		}


		// ====================================================
		// IMU 각도 업데이트
		// ====================================================

		MPU6050_UpdateAngle(
		gyro_z_offset
		);


		// ====================================================
		// 아직 회전을 시작하지 않았다면
		// ====================================================

		if (!rotation_active &&
		!rotation_finished)
		{
			// -----------------------------------------------
			// HuskyLens
			// -----------------------------------------------

			HuskyLens_Update();


			// -----------------------------------------------
			// 거리
			// -----------------------------------------------

			id_distance =
			CalculateDistance();


			// -----------------------------------------------
			// 각도
			// -----------------------------------------------

			id_angle =
			CalculateAngle();


			// -----------------------------------------------
			// ID5 + ID6 모두 검출
			// -----------------------------------------------

			if (id_detected[5] &&
			id_detected[6])
			{
				// ===========================================
				// 단계 5
				// 목표각 계산 완료
				// ===========================================

				LED_SetStage(5);


				printf(
				"ID5: X=%d Y=%d\n",
				id_x[5],
				id_y[5]
				);


				printf(
				"ID6: X=%d Y=%d\n",
				id_x[6],
				id_y[6]
				);


				printf(
				"ID ANGLE: %d deg\n",
				id_angle
				);


				LCD_Clear();

				LCD_SetCursor(0, 0);

				LCD_Print("ID ANGLE:");

				LCD_PrintInt(
				id_angle
				);

				LCD_Print("deg");


				LCD_SetCursor(1, 0);

				LCD_Print("TARGET READY");


				_delay_ms(500);


				// ===========================================
				// 회전 시작
				// ===========================================

				StartRotation();
			}
		}


		// ====================================================
		// 회전 제어
		// ====================================================

		Rotation_Control();


		// ====================================================
		// 회전 완료 후 최종 단계
		// ====================================================

		if (rotation_finished)
		{
			// PA7
			LED_SetStage(8);


			LCD_DisplayStatus();


			printf(
			"ALL PROCESS COMPLETE\n"
			);


			// 한 번만 완료 상태 유지
			rotation_finished = 2;
		}
		else
		{
			// =================================================
			// LCD
			// =================================================

			LCD_DisplayStatus();
		}


		// ====================================================
		// 10ms
		// ====================================================

		_delay_ms(10);
	}
}
