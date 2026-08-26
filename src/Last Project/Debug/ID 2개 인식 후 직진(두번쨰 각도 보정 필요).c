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

#define UART_TIMEOUT         60000UL


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
// 그리퍼 서보
//
// 현재 단계에서는 사용하지 않음
// ============================================================

#define GRIPPER_SERVO_PIN PB7

#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500
#define SERVO_PERIOD_MS   20


// ============================================================
// 회전 / 직진 설정
// ============================================================

#define TURN_SPEED          180
#define FORWARD_SPEED       180

#define ANGLE_TOLERANCE     2.0


// ============================================================
// 직진 중 IMU 보정 설정
//
// 회전 완료 시점의 IMU 각도(target_angle)를
// 직진 중에도 계속 유지하기 위한 P 제어
//
// STRAIGHT_KP
// → 각도 오차 1도당 좌우 속도를 얼마나 다르게 줄지
//
// STRAIGHT_MIN_SPEED / STRAIGHT_MAX_SPEED
// → 보정 후 실제 모터에 들어가는 속도 제한
// ============================================================

#define STRAIGHT_KP          6.0

#define STRAIGHT_MIN_SPEED   0
#define STRAIGHT_MAX_SPEED   255


// ============================================================
// 최종 정지 조건
//
// ID5-ID6 거리 45 pixel 이하
// ID5-ID7 거리 45 pixel 이하
// ============================================================

#define TAG_DISTANCE_STOP   45


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
//
// 인덱스 5, 6, 7 사용
// (0~7 전부 담기 위해 배열 크기 8로 확장)
// ============================================================

uint8_t id_detected[8];

uint16_t id_x[8];
uint16_t id_y[8];


// ============================================================
// ID5 → ID6 거리 / 각도
// ============================================================

uint16_t id_distance = 0;

int16_t id_angle = 0;


// ============================================================
// ID5 → ID7 거리 / 각도
//
// ID5-ID6 도달 이후 재계산해서 사용
// ============================================================

uint16_t id57_distance = 0;

int16_t id57_angle = 0;


// ============================================================
// 초기 회전 상태 (ID5 → ID6 기준)
// ============================================================

uint8_t rotation_active = 0;
uint8_t rotation_finished = 0;

float target_angle = 0.0;


// ============================================================
// 태그까지 직진 상태 (ID5 → ID6 기준)
// ============================================================

uint8_t tag_forward_active = 0;
uint8_t tag_forward_finished = 0;


// ============================================================
// ID5-ID6 도달 후 ID7 재탐색 상태
// ============================================================

uint8_t searching_id7 = 0;


// ============================================================
// 두번째 회전 상태 (ID5 → ID7 기준)
// ============================================================

uint8_t second_rotation_active = 0;
uint8_t second_rotation_finished = 0;

float target_angle2 = 0.0;


// ============================================================
// 두번째 직진 상태 (ID5 → ID7 기준)
// ============================================================

uint8_t second_forward_active = 0;
uint8_t second_forward_finished = 0;


// ============================================================
// 최종 정지 상태
//
// ID5-ID7 45 pixel 도달 후 1
// ============================================================

uint8_t process_finished = 0;


// ============================================================
// 그리퍼 상태
// ============================================================

uint8_t gripper_finished = 0;


// ============================================================
// IMU 현재 각도
// ============================================================

float angle_z = 0.0;


// ============================================================
// LED 단계 표시
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
// ADC 초기화
//
// 현재 단계에서는 PSD를 사용하지 않지만
// 기존 하드웨어 초기화는 유지
// ============================================================

void ADC_Init(void)
{
	DDRF &=
	~(1 << PF0);

	PORTF &=
	~(1 << PF0);

	ADMUX =
	(1 << REFS0);

	ADCSRA =
	(1 << ADEN) |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);
}


// ============================================================
// ADC 읽기
// ============================================================

uint16_t ADC_Read(uint8_t channel)
{
	ADMUX =
	(1 << REFS0) |
	(channel & 0x07);

	ADCSRA |=
	(1 << ADSC);

	while (ADCSRA & (1 << ADSC));

	return ADC;
}


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
	STAGE_LED_DDR = 0xFF;

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

	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	if (data != HUSKY_HEADER2)
	{
		return 0;
	}

	buffer[index++] = data;

	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	if (data != HUSKY_ADDRESS)
	{
		return 0;
	}

	buffer[index++] = data;

	if (!UART0_ReadByteTimeout(&length))
	{
		return 0;
	}

	buffer[index++] = length;

	if (length > 30)
	{
		return 0;
	}

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

	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	buffer[index++] = data;

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
//
// ID5, ID6, ID7 모두 갱신
// ============================================================

uint8_t HuskyLens_Update(void)
{
	uint8_t packet[40];

	uint16_t block_count;

	id_detected[5] = 0;
	id_detected[6] = 0;
	id_detected[7] = 0;

	UART0_ClearBuffer();

	HuskyLens_RequestBlocks();

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
		id == 6 ||
		id == 7)
		{
			id_detected[id] = 1;

			id_x[id] = x;
			id_y[id] = y;
		}
	}

	return 1;
}


// ============================================================
// 두 ID 사이 거리
//
// id_a → id_b 순서는 결과(거리)에 영향 없음
// ============================================================

uint16_t CalculateDistanceIDs(
uint8_t id_a,
uint8_t id_b
)
{
	int32_t dx;
	int32_t dy;

	uint32_t distance_squared;

	if (!id_detected[id_a] ||
	!id_detected[id_b])
	{
		return 0;
	}

	dx =
	(int32_t)id_x[id_b] -
	(int32_t)id_x[id_a];

	dy =
	(int32_t)id_y[id_b] -
	(int32_t)id_y[id_a];

	distance_squared =
	(uint32_t)(dx * dx) +
	(uint32_t)(dy * dy);

	return
	(uint16_t)sqrt(
	(double)distance_squared
	);
}


// ============================================================
// id_a → id_b 각도
//
// 위       = 0°
// 오른쪽   = +90°
// 아래     = ±180°
// 왼쪽     = -90°
// ============================================================

int16_t CalculateAngleIDs(
uint8_t id_a,
uint8_t id_b
)
{
	int32_t dx;
	int32_t dy;

	double angle;

	if (!id_detected[id_a] ||
	!id_detected[id_b])
	{
		return 0;
	}

	dx =
	(int32_t)id_x[id_b] -
	(int32_t)id_x[id_a];

	dy =
	(int32_t)id_y[id_b] -
	(int32_t)id_y[id_a];

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
// ID5 → ID6 거리 (기존 호환용 래퍼)
// ============================================================

uint16_t CalculateDistance(void)
{
	return
	CalculateDistanceIDs(5, 6);
}


// ============================================================
// ID5 → ID6 각도 (기존 호환용 래퍼)
// ============================================================

int16_t CalculateAngle(void)
{
	return
	CalculateAngleIDs(5, 6);
}


// ============================================================
// LCD Expander
// ============================================================

void LCD_ExpanderWrite(uint8_t data)
{
	TWI_Start();

	TWI_Write(
	(LCD_I2C_ADDR << 1)
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
	(data & 0xF0) |
	LCD_RS;

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
	DDRB |=
	(1 << LEFT_PWM);

	DDRB |=
	(1 << RIGHT_PWM);

	DDRB |=
	(1 << LEFT_IN1);

	DDRB |=
	(1 << LEFT_IN2);

	DDRB |=
	(1 << RIGHT_IN1);

	DDRB |=
	(1 << RIGHT_IN2);

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

	PORTB &=
	~(1 << LEFT_IN1);

	PORTB &=
	~(1 << LEFT_IN2);

	PORTB &=
	~(1 << RIGHT_IN1);

	PORTB &=
	~(1 << RIGHT_IN2);
}


// ============================================================
// 전진
// ============================================================

void Motor_Forward(uint8_t speed)
{
	PORTB &=
	~(1 << LEFT_IN1);

	PORTB |=
	(1 << LEFT_IN2);

	PORTB &=
	~(1 << RIGHT_IN1);

	PORTB |=
	(1 << RIGHT_IN2);

	OCR1A = speed;
	OCR1B = speed;
}


// ============================================================
// IMU 기반 직진 보정
//
// target_angle(회전 완료 시점의 목표 각도)과
// 현재 angle_z의 오차만큼
// 좌우 모터 속도를 다르게 주어서
// 직진 경로를 유지 (P 제어, 라인트레이싱 방식)
//
// error > 0
// → 로봇이 왼쪽으로 틀어짐 (오른쪽으로 보정 필요)
// → 왼쪽 바퀴를 느리게, 오른쪽 바퀴를 빠르게
//
// error < 0
// → 로봇이 오른쪽으로 틀어짐 (왼쪽으로 보정 필요)
// → 오른쪽 바퀴를 느리게, 왼쪽 바퀴를 빠르게
// ============================================================

void Motor_Forward_Straight(
uint8_t base_speed,
float error
)
{
	int16_t correction;

	int16_t left_speed;
	int16_t right_speed;

	correction =
	(int16_t)(
	STRAIGHT_KP * error
	);

	left_speed =
	(int16_t)base_speed -
	correction;

	right_speed =
	(int16_t)base_speed +
	correction;


	// --------------------------------------------------------
	// 속도 범위 제한
	// --------------------------------------------------------

	if (left_speed >
	STRAIGHT_MAX_SPEED)
	{
		left_speed =
		STRAIGHT_MAX_SPEED;
	}

	if (left_speed <
	STRAIGHT_MIN_SPEED)
	{
		left_speed =
		STRAIGHT_MIN_SPEED;
	}

	if (right_speed >
	STRAIGHT_MAX_SPEED)
	{
		right_speed =
		STRAIGHT_MAX_SPEED;
	}

	if (right_speed <
	STRAIGHT_MIN_SPEED)
	{
		right_speed =
		STRAIGHT_MIN_SPEED;
	}


	// --------------------------------------------------------
	// 전진 방향 설정
	// --------------------------------------------------------

	PORTB &=
	~(1 << LEFT_IN1);

	PORTB |=
	(1 << LEFT_IN2);

	PORTB &=
	~(1 << RIGHT_IN1);

	PORTB |=
	(1 << RIGHT_IN2);

	OCR1A =
	(uint8_t)left_speed;

	OCR1B =
	(uint8_t)right_speed;
}


// ============================================================
// 오른쪽 회전
// ============================================================

void Motor_TurnRight(uint8_t speed)
{
	PORTB |=
	(1 << LEFT_IN1);

	PORTB &=
	~(1 << LEFT_IN2);

	PORTB &=
	~(1 << RIGHT_IN1);

	PORTB |=
	(1 << RIGHT_IN2);

	OCR1A = speed;
	OCR1B = 0;
}


// ============================================================
// 왼쪽 회전
// ============================================================

void Motor_TurnLeft(uint8_t speed)
{
	PORTB &=
	~(1 << LEFT_IN1);

	PORTB |=
	(1 << LEFT_IN2);

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

	angle_z -=
	gz_dps *
	0.01 *
	GYRO_SCALE;

	NormalizeAngle();
}


// ============================================================
// 각도 차이 계산
// ============================================================

float AngleError(
float target,
float current
)
{
	float error;

	error =
	target -
	current;

	while (error > 180.0)
	{
		error -= 360.0;
	}

	while (error <= -180.0)
	{
		error += 360.0;
	}

	return error;
}


// ============================================================
// 초기 회전 시작 (ID5 → ID6)
// ============================================================

void StartRotation(void)
{
	angle_z = 0.0;

	target_angle =
	(float)id_angle;

	rotation_active = 1;
	rotation_finished = 0;

	tag_forward_active = 0;
	tag_forward_finished = 0;

	process_finished = 0;

	LED_SetStage(6);

	printf(
	"INITIAL ROTATION START\n"
	);

	printf(
	"TARGET ANGLE: %d deg\n",
	id_angle
	);
}


// ============================================================
// 초기 회전 제어 (ID5 → ID6)
// ============================================================

void Rotation_Control(void)
{
	float error;

	if (!rotation_active)
	{
		return;
	}

	error =
	AngleError(
	target_angle,
	angle_z
	);

	if (fabs(error) <=
	ANGLE_TOLERANCE)
	{
		Motor_Stop();

		rotation_active = 0;
		rotation_finished = 1;

		tag_forward_active = 1;
		tag_forward_finished = 0;

		LED_SetStage(7);

		printf(
		"INITIAL ROTATION COMPLETE\n"
		);

		printf(
		"IMU ANGLE: %d deg\n",
		(int)angle_z
		);

		printf(
		"TARGET ANGLE (HOLD): %d deg\n",
		(int)target_angle
		);

		printf(
		"TAG DISTANCE FORWARD START\n"
		);

		_delay_ms(300);

		return;
	}

	if (error > 0)
	{
		Motor_TurnRight(
		TURN_SPEED
		);
	}
	else
	{
		Motor_TurnLeft(
		TURN_SPEED
		);
	}
}


// ============================================================
// 태그 거리 도달 확인 (ID5-ID6)
//
// ID5-ID6 거리 45 pixel 이하
// ============================================================

uint8_t TagDistanceReached(void)
{
	if (!id_detected[5] ||
	!id_detected[6])
	{
		return 0;
	}

	if (id_distance <=
	TAG_DISTANCE_STOP)
	{
		return 1;
	}

	return 0;
}


// ============================================================
// 태그 거리 직진 제어 (ID5-ID6)
//
// 회전 완료 시점의 target_angle을
// 그대로 "직진 유지 목표각"으로 사용
//
// 직진 중에도 IMU를 계속 읽어서
// 현재각과 target_angle의 오차만큼
// 좌우 모터 속도를 보정 (라인트레이싱 방식)
//
// 여기서 45 pixel 이하가 되면
// 모터를 정지하고,
// 곧바로 ID5-ID7 재탐색 단계로 넘어감
// (process_finished는 아직 세우지 않음)
// ============================================================

void TagForward_Control(void)
{
	float error;

	if (!tag_forward_active)
	{
		return;
	}

	HuskyLens_Update();

	id_distance =
	CalculateDistance();

	// --------------------------------------------------------
	// 45 pixel 이하 (ID5-ID6)
	// --------------------------------------------------------

	if (TagDistanceReached())
	{
		Motor_Stop();

		tag_forward_active = 0;
		tag_forward_finished = 1;

		LED_SetStage(8);

		printf(
		"================================\n"
		);

		printf(
		"ID5-ID6 DISTANCE <= %d PIXEL\n",
		TAG_DISTANCE_STOP
		);

		printf(
		"ID5-ID6 DISTANCE: %d PIXEL\n",
		id_distance
		);

		printf(
		"MOTOR STOP\n"
		);

		printf(
		"STAGE 1 COMPLETE\n"
		);

		printf(
		"SEARCHING ID5 / ID7\n"
		);

		printf(
		"================================\n"
		);

		// ID7 재탐색 시작 (메인 루프에서 계속 시도)
		searching_id7 = 1;

		return;
	}

	// --------------------------------------------------------
	// 아직 45 pixel보다 큼
	//
	// target_angle 유지하며 직진 (IMU 보정)
	// --------------------------------------------------------

	error =
	AngleError(
	target_angle,
	angle_z
	);

	printf(
	"STRAIGHT ERR: %d deg  IMU: %d deg\n",
	(int)error,
	(int)angle_z
	);

	Motor_Forward_Straight(
	FORWARD_SPEED,
	error
	);
}


// ============================================================
// 두번째 회전 시작 (ID5 → ID7)
//
// ID5-ID6 도달 후 다시 계산한 각도만큼
// 처음 회전했던 것과 동일한 방식으로 회전
// ============================================================

void StartRotation2(void)
{
	angle_z = 0.0;

	target_angle2 =
	(float)id57_angle;

	second_rotation_active = 1;
	second_rotation_finished = 0;

	second_forward_active = 0;
	second_forward_finished = 0;

	LED_SetStage(6);

	printf(
	"SECOND ROTATION START (ID5->ID7)\n"
	);

	printf(
	"TARGET ANGLE: %d deg\n",
	id57_angle
	);
}


// ============================================================
// 두번째 회전 제어 (ID5 → ID7)
// ============================================================

void Rotation_Control2(void)
{
	float error;

	if (!second_rotation_active)
	{
		return;
	}

	error =
	AngleError(
	target_angle2,
	angle_z
	);

	if (fabs(error) <=
	ANGLE_TOLERANCE)
	{
		Motor_Stop();

		second_rotation_active = 0;
		second_rotation_finished = 1;

		second_forward_active = 1;
		second_forward_finished = 0;

		LED_SetStage(7);

		printf(
		"SECOND ROTATION COMPLETE\n"
		);

		printf(
		"IMU ANGLE: %d deg\n",
		(int)angle_z
		);

		printf(
		"TARGET ANGLE (HOLD): %d deg\n",
		(int)target_angle2
		);

		printf(
		"ID5-ID7 FORWARD START\n"
		);

		_delay_ms(300);

		return;
	}

	if (error > 0)
	{
		Motor_TurnRight(
		TURN_SPEED
		);
	}
	else
	{
		Motor_TurnLeft(
		TURN_SPEED
		);
	}
}


// ============================================================
// 태그 거리 도달 확인 (ID5-ID7)
//
// ID5-ID7 거리 45 pixel 이하
// ============================================================

uint8_t TagDistanceReached2(void)
{
	if (!id_detected[5] ||
	!id_detected[7])
	{
		return 0;
	}

	if (id57_distance <=
	TAG_DISTANCE_STOP)
	{
		return 1;
	}

	return 0;
}


// ============================================================
// 태그 거리 직진 제어 (ID5-ID7)
//
// target_angle2를 유지하며 IMU 보정 직진
// (첫번째 구간과 완전히 동일한 방식)
//
// 여기서 45 pixel 이하가 되면
// 모터를 정지하고 전체 과정을 종료
// ============================================================

void TagForward_Control2(void)
{
	float error;

	if (!second_forward_active)
	{
		return;
	}

	HuskyLens_Update();

	id57_distance =
	CalculateDistanceIDs(5, 7);

	// --------------------------------------------------------
	// 45 pixel 이하 (ID5-ID7)
	// --------------------------------------------------------

	if (TagDistanceReached2())
	{
		Motor_Stop();

		second_forward_active = 0;
		second_forward_finished = 1;

		process_finished = 1;

		STAGE_LED_PORT = 0xFF;

		printf(
		"================================\n"
		);

		printf(
		"ID5-ID7 DISTANCE <= %d PIXEL\n",
		TAG_DISTANCE_STOP
		);

		printf(
		"ID5-ID7 DISTANCE: %d PIXEL\n",
		id57_distance
		);

		printf(
		"MOTOR STOP\n"
		);

		printf(
		"FULL PROCESS COMPLETE\n"
		);

		printf(
		"================================\n"
		);

		return;
	}

	// --------------------------------------------------------
	// 아직 45 pixel보다 큼
	//
	// target_angle2 유지하며 직진 (IMU 보정)
	// --------------------------------------------------------

	error =
	AngleError(
	target_angle2,
	angle_z
	);

	printf(
	"STRAIGHT2 ERR: %d deg  IMU: %d deg\n",
	(int)error,
	(int)angle_z
	);

	Motor_Forward_Straight(
	FORWARD_SPEED,
	error
	);
}


// ============================================================
// LCD 상태 표시
// ============================================================

void LCD_DisplayStatus(void)
{
	LCD_Clear();

	// --------------------------------------------------------
	// 초기 회전 (ID5-ID6)
	// --------------------------------------------------------

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


	// --------------------------------------------------------
	// 태그까지 직진 (ID5-ID6, IMU 유지)
	// --------------------------------------------------------

	if (tag_forward_active)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("DIST:");

		LCD_PrintUInt(
		id_distance
		);

		LCD_Print("px");

		LCD_SetCursor(1, 0);

		LCD_Print("IMU:");

		LCD_PrintInt(
		(int16_t)angle_z
		);

		LCD_Print("/");

		LCD_PrintInt(
		(int16_t)target_angle
		);

		LCD_Print("deg");

		return;
	}


	// --------------------------------------------------------
	// ID5-ID6 도달 후 ID7 재탐색 중
	// --------------------------------------------------------

	if (searching_id7)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("ID5/ID7");

		LCD_SetCursor(1, 0);

		LCD_Print("Searching...");

		return;
	}


	// --------------------------------------------------------
	// 두번째 회전 (ID5-ID7)
	// --------------------------------------------------------

	if (second_rotation_active)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("TARGET2:");

		LCD_PrintInt(
		(int16_t)target_angle2
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


	// --------------------------------------------------------
	// 두번째 직진 (ID5-ID7, IMU 유지)
	// --------------------------------------------------------

	if (second_forward_active)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("DIST7:");

		LCD_PrintUInt(
		id57_distance
		);

		LCD_Print("px");

		LCD_SetCursor(1, 0);

		LCD_Print("IMU:");

		LCD_PrintInt(
		(int16_t)angle_z
		);

		LCD_Print("/");

		LCD_PrintInt(
		(int16_t)target_angle2
		);

		LCD_Print("deg");

		return;
	}


	// --------------------------------------------------------
	// 최종 완료 (ID5-ID7 45 pixel 도달)
	// --------------------------------------------------------

	if (process_finished)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("STOPPED");

		LCD_SetCursor(1, 0);

		LCD_Print("DIST7:");

		LCD_PrintUInt(
		id57_distance
		);

		LCD_Print("px");

		return;
	}


	// --------------------------------------------------------
	// 최초 ID5/ID6 검출 대기
	// --------------------------------------------------------

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

		LCD_Print("DIST:");

		LCD_PrintUInt(
		id_distance
		);

		LCD_Print("px");
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
	// 단계 1
	// 시스템 초기화
	// ========================================================

	LED_Init();

	LED_SetStage(1);

	UART0_Init();

	stdout =
	&UART0_OUTPUT;

	TWI_Init();

	ResetSwitch_Init();

	LCD_Init();

	Motor_Init();

	ADC_Init();


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

			tag_forward_active = 0;
			tag_forward_finished = 0;

			searching_id7 = 0;

			second_rotation_active = 0;
			second_rotation_finished = 0;

			second_forward_active = 0;
			second_forward_finished = 0;

			process_finished = 0;

			gripper_finished = 0;

			printf(
			"FULL PROCESS RESET\n"
			);

			LED_SetStage(4);

			LCD_Clear();

			LCD_SetCursor(0, 0);

			LCD_Print("SYSTEM RESET");

			LCD_SetCursor(1, 0);

			LCD_Print("Searching...");


			while (!(PIND &
			(1 << RESET_SWITCH)))
			{
				_delay_ms(10);
			}
		}


		// ====================================================
		// 이미 ID5-ID7 45 pixel에서 정지 완료
		//
		// 이후에는 절대로 모터를 다시 구동하지 않음
		// ====================================================

		if (process_finished)
		{
			Motor_Stop();

			LCD_DisplayStatus();

			_delay_ms(100);

			continue;
		}


		// ====================================================
		// IMU 각도 업데이트
		//
		// 두 구간의 회전 중 + 직진 중(직진 유지 보정용) 모두 사용
		// ====================================================

		if (rotation_active ||
		tag_forward_active ||
		second_rotation_active ||
		second_forward_active)
		{
			MPU6050_UpdateAngle(
			gyro_z_offset
			);
		}


		// ====================================================
		// 초기 회전 전
		// HuskyLens ID5 / ID6 탐색
		// ====================================================

		if (!rotation_active &&
		!rotation_finished &&
		!tag_forward_active &&
		!tag_forward_finished)
		{
			HuskyLens_Update();

			id_distance =
			CalculateDistance();

			id_angle =
			CalculateAngle();


			if (id_detected[5] &&
			id_detected[6])
			{
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
				"ID DISTANCE: %d PIXEL\n",
				id_distance
				);

				printf(
				"ID ANGLE: %d DEG\n",
				id_angle
				);


				LCD_Clear();

				LCD_SetCursor(0, 0);

				LCD_Print("ANGLE:");

				LCD_PrintInt(
				id_angle
				);

				LCD_Print("deg");


				LCD_SetCursor(1, 0);

				LCD_Print("DIST:");

				LCD_PrintUInt(
				id_distance
				);

				LCD_Print("px");


				_delay_ms(500);


				// 초기 회전 시작
				StartRotation();
			}
		}


		// ====================================================
		// 초기 각도 회전 (ID5-ID6)
		// ====================================================

		Rotation_Control();


		// ====================================================
		// 회전 완료 후
		// ID 거리 45 pixel까지 직진 (ID5-ID6)
		//
		// target_angle 유지하며 IMU 보정 직진
		// ====================================================

		TagForward_Control();


		// ====================================================
		// ID5-ID6 도달 후
		// ID5 / ID7 재탐색
		//
		// 처음 ID5/ID6 탐색했던 것과 동일한 방식
		// ====================================================

		if (searching_id7)
		{
			HuskyLens_Update();

			id57_distance =
			CalculateDistanceIDs(5, 7);

			id57_angle =
			CalculateAngleIDs(5, 7);

			if (id_detected[5] &&
			id_detected[7])
			{
				printf(
				"ID5: X=%d Y=%d\n",
				id_x[5],
				id_y[5]
				);

				printf(
				"ID7: X=%d Y=%d\n",
				id_x[7],
				id_y[7]
				);

				printf(
				"ID5-ID7 DISTANCE: %d PIXEL\n",
				id57_distance
				);

				printf(
				"ID5-ID7 ANGLE: %d DEG\n",
				id57_angle
				);


				LCD_Clear();

				LCD_SetCursor(0, 0);

				LCD_Print("ANGLE7:");

				LCD_PrintInt(
				id57_angle
				);

				LCD_Print("deg");


				LCD_SetCursor(1, 0);

				LCD_Print("DIST7:");

				LCD_PrintUInt(
				id57_distance
				);

				LCD_Print("px");


				_delay_ms(500);


				searching_id7 = 0;

				// 두번째 회전 시작
				StartRotation2();
			}
		}


		// ====================================================
		// 두번째 각도 회전 (ID5-ID7)
		// ====================================================

		Rotation_Control2();


		// ====================================================
		// 두번째 회전 완료 후
		// ID5-ID7 거리 45 pixel까지 직진
		//
		// target_angle2 유지하며 IMU 보정 직진
		// ====================================================

		TagForward_Control2();


		// ====================================================
		// 상태 표시
		// ====================================================

		LCD_DisplayStatus();


		// ====================================================
		// 10ms
		// ====================================================

		_delay_ms(10);
	}
}
