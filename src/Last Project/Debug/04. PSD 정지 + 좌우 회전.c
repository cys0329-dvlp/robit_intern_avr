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
// PB7 = Servo Signal
// ============================================================

#define GRIPPER_SERVO_PIN PB7

#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500
#define SERVO_PERIOD_MS   20


// ============================================================
// 회전 / 직진 설정
// ============================================================

#define TURN_SPEED          250
#define FORWARD_SPEED       180

#define ANGLE_TOLERANCE     2.0


// ============================================================
// Delay 기반 PSD 탐색 설정
//
// 필요하면 여기 시간만 조절하면 됨
//
// LEFT_TURN_TIME_MS
// → 왼쪽으로 회전하는 시간
//
// CENTER_TIME_MS
// → 가운데 방향으로 복귀하는 시간
//
// RIGHT_TURN_TIME_MS
// → 오른쪽으로 회전하는 시간
// ============================================================

#define LEFT_TURN_TIME_MS    500
#define CENTER_TIME_MS       500
#define RIGHT_TURN_TIME_MS   500


// ============================================================
// PSD 설정
//
// PF0 = ADC0
//
// PSD 측정 방식:
// 1. ADC 20회 측정
// 2. 최대값 제거
// 3. 최소값 제거
// 4. 나머지 18개 평균
// 5. 이전값 7/8 + 현재값 1/8
//
// 피벗 탐색:
// PSD 값이 320 ~ 330일 때만 정지
//
// 최종 직진:
// PSD 값이 250 이하이면 정지
// ============================================================

#define PF0_PIVOT_MIN        320
#define PF0_PIVOT_MAX        330

#define PF0_FINAL_THRESHOLD  250

#define TAG_DISTANCE_STOP    45


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
// 초기 회전 상태
// ============================================================

uint8_t rotation_active = 0;
uint8_t rotation_finished = 0;

float target_angle = 0.0;


// ============================================================
// 초기 직진 상태
//
// ID 거리 45 pixel까지 이동
// ============================================================

uint8_t tag_forward_active = 0;
uint8_t tag_forward_finished = 0;


// ============================================================
// PSD Delay 탐색 상태
//
// IMU 사용 안 함
//
// 0 = 왼쪽 탐색
// 1 = 가운데 복귀
// 2 = 오른쪽 탐색
// 3 = 가운데 복귀
// ============================================================

uint8_t pivot_active = 0;
uint8_t pivot_finished = 0;

uint8_t pivot_search_state = 0;


// ============================================================
// 최종 직진 상태
// ============================================================

uint8_t final_forward_active = 0;
uint8_t final_finished = 0;


// ============================================================
// 그리퍼 상태
// ============================================================

uint8_t gripper_finished = 0;


// ============================================================
// IMU 현재 각도
// ============================================================

float angle_z = 0.0;


// ============================================================
// PSD 안정화 값
// ============================================================

uint16_t psd_stable_value = 0;


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
// PF0 = ADC0
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

	while (ADCSRA &
	(1 << ADSC));

	return ADC;
}


// ============================================================
// PSD 정밀 측정
//
// 1. ADC 20회 측정
// 2. 최댓값 제거
// 3. 최솟값 제거
// 4. 나머지 18개 평균
// ============================================================

uint16_t PSD_Read_Filtered(uint8_t channel)
{
	uint32_t sum = 0;

	uint16_t value;

	uint16_t min = 1023;
	uint16_t max = 0;

	for (uint8_t i = 0;
	i < 20;
	i++)
	{
		value =
		ADC_Read(channel);

		sum += value;

		if (value < min)
		{
			min = value;
		}

		if (value > max)
		{
			max = value;
		}
	}

	sum -= max;
	sum -= min;

	return
	(uint16_t)(sum / 18);
}


// ============================================================
// PSD 최종 안정화
//
// 이전 값 7/8
// 현재 새 값 1/8
// ============================================================

uint16_t PSD_Read_Stable(uint8_t channel)
{
	uint16_t new_value;

	new_value =
	PSD_Read_Filtered(channel);

	if (psd_stable_value == 0)
	{
		psd_stable_value =
		new_value;
	}
	else
	{
		psd_stable_value =
		(uint16_t)
		(
		(psd_stable_value * 7UL +
		new_value)
		/ 8UL
		);
	}

	return
	psd_stable_value;
}


// ============================================================
// PSD 필터 초기화
// ============================================================

void PSD_ResetFilter(void)
{
	psd_stable_value = 0;
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
	if (!(PIND &
	(1 << RESET_SWITCH)))
	{
		_delay_ms(20);

		if (!(PIND &
		(1 << RESET_SWITCH)))
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

		if (data ==
		HUSKY_HEADER1)
		{
			break;
		}
	}

	buffer[index++] =
	data;

	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	if (data !=
	HUSKY_HEADER2)
	{
		return 0;
	}

	buffer[index++] =
	data;

	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	if (data !=
	HUSKY_ADDRESS)
	{
		return 0;
	}

	buffer[index++] =
	data;

	if (!UART0_ReadByteTimeout(&length))
	{
		return 0;
	}

	buffer[index++] =
	length;

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

		buffer[index++] =
		data;
	}

	if (!UART0_ReadByteTimeout(&data))
	{
		return 0;
	}

	buffer[index++] =
	data;

	for (uint8_t i = 0;
	i < index - 1;
	i++)
	{
		checksum +=
		buffer[i];
	}

	if (checksum !=
	buffer[index - 1])
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

	if (!HuskyLens_ReadPacket(packet))
	{
		return 0;
	}

	if (packet[4] !=
	HUSKY_RETURN_INFO)
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

		if (packet[4] !=
		HUSKY_RETURN_BLOCK)
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
// 기존 오른쪽 피벗
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
// 기존 왼쪽 피벗
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
// 제자리 왼쪽 회전
//
// 왼쪽 모터 후진
// 오른쪽 모터 전진
// ============================================================

void Motor_RotateLeft(uint8_t speed)
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
	OCR1B = speed;
}


// ============================================================
// 제자리 오른쪽 회전
//
// 왼쪽 모터 전진
// 오른쪽 모터 후진
// ============================================================

void Motor_RotateRight(uint8_t speed)
{
	PORTB &=
	~(1 << LEFT_IN1);

	PORTB |=
	(1 << LEFT_IN2);

	PORTB |=
	(1 << RIGHT_IN1);

	PORTB &=
	~(1 << RIGHT_IN2);

	OCR1A = speed;
	OCR1B = speed;
}


// ============================================================
// 그리퍼 서보 초기화
// ============================================================

void GripperServo_Init(void)
{
	DDRB |=
	(1 << GRIPPER_SERVO_PIN);

	PORTB &=
	~(1 << GRIPPER_SERVO_PIN);
}


// ============================================================
// 그리퍼 서보 각도 제어
//
// 0도   = 500us
// 270도 = 2500us
// ============================================================

void GripperServo_SetAngle(uint16_t angle)
{
	uint16_t pulse_us;
	uint16_t low_us;

	if (angle > 270)
	{
		angle = 270;
	}

	pulse_us =
	SERVO_MIN_US +
	(
	(uint32_t)
	(SERVO_MAX_US - SERVO_MIN_US)
	* angle
	/ 270
	);

	low_us =
	(SERVO_PERIOD_MS * 1000UL)
	- pulse_us;

	PORTB |=
	(1 << GRIPPER_SERVO_PIN);

	while (pulse_us--)
	{
		_delay_us(1);
	}

	PORTB &=
	~(1 << GRIPPER_SERVO_PIN);

	while (low_us--)
	{
		_delay_us(1);
	}
}


// ============================================================
// 그리퍼 270도 파지
// ============================================================

void GripperServo_270(void)
{
	uint8_t i;

	printf(
	"GRIPPER 270 DEG START\n"
	);

	for (i = 0;
	i < 50;
	i++)
	{
		GripperServo_SetAngle(270);
	}

	gripper_finished = 1;

	printf(
	"GRIPPER 270 DEG COMPLETE\n"
	);
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
// 초기 회전 시작
//
// 여기서는 기존처럼 IMU 사용
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

	pivot_active = 0;
	pivot_finished = 0;

	pivot_search_state = 0;

	final_forward_active = 0;
	final_finished = 0;

	gripper_finished = 0;

	PSD_ResetFilter();

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
// 초기 회전 제어
//
// 기존 IMU 방식 유지
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
// 태그 거리 도달 확인
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
// 태그 거리 직진 제어
// ============================================================

void TagForward_Control(void)
{
	if (!tag_forward_active)
	{
		return;
	}

	HuskyLens_Update();

	id_distance =
	CalculateDistance();

	if (TagDistanceReached())
	{
		Motor_Stop();

		tag_forward_active = 0;
		tag_forward_finished = 1;

		printf(
		"TAG DISTANCE <= %d PIXEL\n",
		TAG_DISTANCE_STOP
		);

		printf(
		"TAG DISTANCE: %d PIXEL\n",
		id_distance
		);


		// ====================================================
		// Delay 기반 좌우 PSD 탐색 시작
		//
		// IMU는 여기부터 사용하지 않음
		// ====================================================

		pivot_active = 1;
		pivot_finished = 0;

		pivot_search_state = 0;

		PSD_ResetFilter();

		LED_SetStage(7);

		printf(
		"DELAY PIVOT SEARCH START\n"
		);

		printf(
		"LEFT 1 SEC -> CENTER 1 SEC -> RIGHT 1 SEC -> CENTER 1 SEC\n"
		);

		return;
	}

	Motor_Forward(
	FORWARD_SPEED
	);
}


// ============================================================
// PSD 탐색
//
// IMU 미사용
//
// 순서:
//
// 0: 왼쪽 1초
// 1: 오른쪽으로 1초 복귀
// 2: 오른쪽 1초
// 3: 왼쪽으로 1초 복귀
//
// 위 과정을 반복하면서
// PSD 320~330 확인
// ============================================================

void Pivot_Control(void)
{
	uint16_t pf0_raw;

	if (!pivot_active)
	{
		return;
	}


	// --------------------------------------------------------
	// 현재 PSD 확인
	// --------------------------------------------------------

	pf0_raw =
	PSD_Read_Stable(0);

	printf(
	"PIVOT PSD STABLE: %u\n",
	pf0_raw
	);


	// --------------------------------------------------------
	// PSD 320 ~ 330
	// 발견하면 즉시 정지
	// --------------------------------------------------------

	if (pf0_raw >=
	PF0_PIVOT_MIN &&
	pf0_raw <=
	PF0_PIVOT_MAX)
	{
		Motor_Stop();

		pivot_active = 0;
		pivot_finished = 1;

		printf(
		"PSD IN RANGE: %d ~ %d\n",
		PF0_PIVOT_MIN,
		PF0_PIVOT_MAX
		);

		printf(
		"CURRENT PSD: %u\n",
		pf0_raw
		);

		printf(
		"OBJECT FOUND\n"
		);

		printf(
		"PIVOT STOP\n"
		);


		// 최종 직진 전 필터 초기화
		PSD_ResetFilter();

		final_forward_active = 1;
		final_finished = 0;

		LED_SetStage(8);

		printf(
		"FINAL FORWARD START\n"
		);

		return;
	}


	// --------------------------------------------------------
	// 상태 0
	//
	// 왼쪽 1초 회전
	// --------------------------------------------------------

	if (pivot_search_state == 0)
	{
		printf(
		"SEARCH: LEFT TURN\n"
		);

		Motor_RotateLeft(
		TURN_SPEED
		);

		_delay_ms(
		LEFT_TURN_TIME_MS
		);

		Motor_Stop();

		_delay_ms(200);

		pivot_search_state = 1;

		return;
	}


	// --------------------------------------------------------
	// 상태 1
	//
	// 오른쪽으로 1초
	// 가운데 방향 복귀
	// --------------------------------------------------------

	if (pivot_search_state == 1)
	{
		printf(
		"SEARCH: RETURN TO CENTER FROM LEFT\n"
		);

		Motor_RotateRight(
		TURN_SPEED
		);

		_delay_ms(
		CENTER_TIME_MS
		);

		Motor_Stop();

		_delay_ms(200);

		pivot_search_state = 2;

		return;
	}


	// --------------------------------------------------------
	// 상태 2
	//
	// 오른쪽 1초 회전
	// --------------------------------------------------------

	if (pivot_search_state == 2)
	{
		printf(
		"SEARCH: RIGHT TURN\n"
		);

		Motor_RotateRight(
		TURN_SPEED
		);

		_delay_ms(
		RIGHT_TURN_TIME_MS
		);

		Motor_Stop();

		_delay_ms(200);

		pivot_search_state = 3;

		return;
	}


	// --------------------------------------------------------
	// 상태 3
	//
	// 왼쪽으로 1초
	// 가운데 방향 복귀
	// --------------------------------------------------------

	if (pivot_search_state == 3)
	{
		printf(
		"SEARCH: RETURN TO CENTER FROM RIGHT\n"
		);

		Motor_RotateLeft(
		TURN_SPEED
		);

		_delay_ms(
		CENTER_TIME_MS
		);

		Motor_Stop();

		_delay_ms(200);

		pivot_search_state = 0;

		return;
	}
}


// ============================================================
// 최종 직진 제어
//
// PSD <= 250이면 정지
// → 그리퍼 270도
// → 종료
// ============================================================

void FinalForward_Control(void)
{
	uint16_t pf0_raw;

	if (!final_forward_active)
	{
		return;
	}

	pf0_raw =
	PSD_Read_Stable(0);

	printf(
	"FINAL PSD STABLE: %u\n",
	pf0_raw
	);

	if (pf0_raw <=
	PF0_FINAL_THRESHOLD)
	{
		Motor_Stop();

		final_forward_active = 0;
		final_finished = 1;

		printf(
		"FINAL PSD <= %d\n",
		PF0_FINAL_THRESHOLD
		);

		printf(
		"CURRENT PSD: %u\n",
		pf0_raw
		);

		printf(
		"FINAL POSITION REACHED\n"
		);

		GripperServo_270();

		printf(
		"ALL PROCESS COMPLETE\n"
		);

		return;
	}

	Motor_Forward(
	FORWARD_SPEED
	);
}


// ============================================================
// LCD 상태 표시
// ============================================================

void LCD_DisplayStatus(void)
{
	LCD_Clear();


	// 초기 회전
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


	// 태그까지 직진
	if (tag_forward_active)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("TAG DIST:");

		LCD_PrintUInt(
		id_distance
		);

		LCD_Print("px");

		LCD_SetCursor(1, 0);

		LCD_Print("TARGET:45px");

		return;
	}


	// Delay 기반 PSD 탐색
	if (pivot_active)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("PIVOT PSD:");

		LCD_PrintUInt(
		psd_stable_value
		);

		LCD_SetCursor(1, 0);

		if (pivot_search_state == 0)
		{
			LCD_Print("TURN LEFT");
		}
		else if (pivot_search_state == 1)
		{
			LCD_Print("LEFT->CENTER");
		}
		else if (pivot_search_state == 2)
		{
			LCD_Print("TURN RIGHT");
		}
		else
		{
			LCD_Print("RIGHT->CENTER");
		}

		return;
	}


	// 최종 직진
	if (final_forward_active)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("FINAL PSD:");

		LCD_PrintUInt(
		psd_stable_value
		);

		LCD_SetCursor(1, 0);

		LCD_Print("STOP<=250");

		return;
	}


	// 최종 완료
	if (final_finished)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("GRIPPER DONE");

		LCD_SetCursor(1, 0);

		LCD_Print("270 DEG");

		return;
	}


	// ID 검출
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

	GripperServo_Init();

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
	// PSD 필터 초기화
	// ========================================================

	PSD_ResetFilter();


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

			pivot_active = 0;
			pivot_finished = 0;

			pivot_search_state = 0;

			final_forward_active = 0;
			final_finished = 0;

			gripper_finished = 0;

			PSD_ResetFilter();


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
		// IMU 각도 업데이트
		//
		// 초기 회전에만 사용
		// PSD 탐색은 IMU 사용 안 함
		// ====================================================

		if (rotation_active)
		{
			MPU6050_UpdateAngle(
			gyro_z_offset
			);
		}


		// ====================================================
		// 초기 회전 전
		// HuskyLens ID 검색
		// ====================================================

		if (!rotation_active &&
		!rotation_finished &&
		!tag_forward_active &&
		!tag_forward_finished &&
		!pivot_active &&
		!pivot_finished &&
		!final_forward_active &&
		!final_finished)
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


				StartRotation();
			}
		}


		// ====================================================
		// 초기 IMU 회전
		// ====================================================

		Rotation_Control();


		// ====================================================
		// ID 거리 45 pixel까지 직진
		// ====================================================

		TagForward_Control();


		// ====================================================
		// ID 거리 45 pixel 도달 후
		//
		// Delay 기반 좌우 탐색
		//
		// 왼쪽 1초
		// → 가운데 1초
		// → 오른쪽 1초
		// → 가운데 1초
		// → 반복
		// ====================================================

		Pivot_Control();


		// ====================================================
		// PSD 발견 후 최종 직진
		// ====================================================

		FinalForward_Control();


		// ====================================================
		// 최종 완료
		// ====================================================

		if (final_finished)
		{
			LED_SetStage(8);

			LCD_DisplayStatus();

			Motor_Stop();

			_delay_ms(100);
		}
		else
		{
			LCD_DisplayStatus();
		}


		// ====================================================
		// 기본 주기
		// ====================================================

		_delay_ms(10);
	}
}
