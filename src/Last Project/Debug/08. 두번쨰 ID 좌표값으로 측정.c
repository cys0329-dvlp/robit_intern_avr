#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
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
// PWM  PB5 (OC1A)
// IN1  PB0
// IN2  PB1
//
// 오른쪽 모터:
// PWM  PB6 (OC1B)
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
// 그리퍼 서보 (MG995)
//
// Signal : PB7 (OC1C)
//
// 모터(OC1A/OC1B)와 그리퍼(OC1C)는 전부 Timer1 하나를 공유하므로
// Timer1은 Mode 14 (Fast PWM, TOP=ICR1) / 분주 8 로 통일해서 쓴다.
// ICR1 = 40000 → 2MHz 카운트 기준 20ms 주기 (서보 표준 프레임)
//
// 모터는 원래 0~255 스케일(TURN_SPEED, FORWARD_SPEED 등)을 그대로 쓰되
// 실제로 OCR1A/OCR1B에 넣을 때만 MOTOR_DUTY()로 40000 스케일로 환산한다.
// ============================================================

#define GRIPPER_SERVO_PIN PB7

#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500
#define SERVO_PERIOD_MS   20

#define GRIPPER_ANGLE_MAX 270
#define GRIPPER_OPEN_ANGLE   0
#define GRIPPER_CLOSE_ANGLE  270

#define PWM_TOP           40000UL

#define MOTOR_DUTY(x)     ((uint16_t)(((uint32_t)(x) * PWM_TOP) / 255UL))


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
// → 보정 후 실제 모터에 들어가는 속도 제한 (0~255 스케일 기준)
// ============================================================

#define STRAIGHT_KP          6.0

#define STRAIGHT_MIN_SPEED   0
#define STRAIGHT_MAX_SPEED   255


// ============================================================
// 최종 정지 조건 (ID5-ID6 단계 전용)
//
// ID5-ID6 거리 45 pixel 이하
//
// ID5-ID7 단계는 더 이상 이 값을 쓰지 않고
// "ID5 좌표 == ID7 저장 좌표" 방식으로 정지한다.
// (아래 ID5_ReachedTarget7 참고)
// ============================================================

#define TAG_DISTANCE_STOP   45


// ============================================================
// ID5-ID6 도달 후 추가 동작 설정
//
// 정지 -> 0.5초 전진 -> 그리퍼 270도 회전 -> ID7 진행
// ============================================================

#define POST_STOP_FORWARD_MS   500
#define GRIPPER_MOVE_WAIT_MS    1000


// ============================================================
// ID5-ID7 단계 전용 설정
//
// - ID7 좌표는 회전 시작 직전(각도 계산 시점)에 한 번만 저장해두고
//   그 이후로는 갱신하지 않는다 (고정 목표점으로 사용).
// - 직진 중에는 ID5 좌표만 0.2초 주기로 계속 갱신해서
//   저장해둔 ID7 좌표와 비교한다.
// - 카메라 픽셀 좌표가 완전히 똑같이 나오기는 어려우므로
//   POSITION_MATCH_TOLERANCE 픽셀 이내면 "일치"로 판정한다.
//   (필요하면 이 값만 조정하면 됨)
// ============================================================

#define ID5_UPDATE_INTERVAL_MS     200
#define POSITION_MATCH_TOLERANCE   50


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
// 현재 목표 태그 ID
//
// ID5-ID6 단계에서는 6
// ID5-ID6 완료 후 ID5-ID7 단계에서는 7 로 전환
//
// ID5-ID6 / ID5-ID7 모두
// 아래의 동일한 회전 로직을 그대로 재사용한다.
// (직진/정지 로직은 ID7부터 달라짐, TagForward_Control 참고)
// ============================================================

uint8_t current_target_id = 6;


// ============================================================
// ID5 → 목표(ID6 또는 ID7) 거리 / 각도
// ============================================================

uint16_t id_distance = 0;

int16_t id_angle = 0;


// ============================================================
// 회전 상태 (ID5 → 현재 목표 기준)
// ============================================================

uint8_t rotation_active = 0;
uint8_t rotation_finished = 0;

float target_angle = 0.0;


// ============================================================
// 태그까지 직진 상태 (ID5 → 현재 목표 기준)
// ============================================================

uint8_t tag_forward_active = 0;
uint8_t tag_forward_finished = 0;


// ============================================================
// 최종 정지 상태
//
// ID5-ID7 좌표 일치 후 1
// ============================================================

uint8_t process_finished = 0;


// ============================================================
// 그리퍼 상태
// ============================================================

uint8_t gripper_finished = 0;


// ============================================================
// ID5-ID7 단계 전용 상태
//
// target7_x/y   : 회전 시작 직전에 저장해 둔 ID7의 고정 좌표
// target7_saved : 저장 여부
// id5_update_timer_ms : ID5 좌표 갱신 주기(0.2초) 카운터
// ============================================================

uint16_t target7_x = 0;
uint16_t target7_y = 0;
uint8_t target7_saved = 0;

uint16_t id5_update_timer_ms = 0;


// ============================================================
// IMU 현재 각도
// ============================================================

float angle_z = 0.0;


// ============================================================
// 함수 프로토타입 (Motor_Init에서 Servo_SetAngle을 먼저 쓰기 위함)
// ============================================================

void Servo_SetAngle(uint16_t angle_deg);
void Motor_Stop(void);


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
// ID5 → 현재 목표(current_target_id) 거리
// ============================================================

uint16_t CalculateDistance(void)
{
	return
	CalculateDistanceIDs(5, current_target_id);
}


// ============================================================
// ID5 → 현재 목표(current_target_id) 각도
// ============================================================

int16_t CalculateAngle(void)
{
	return
	CalculateAngleIDs(5, current_target_id);
}


// ============================================================
// ID5-ID7 단계 전용
//
// 저장해 둔 target7_x/y (회전 시작 직전 ID7 좌표)와
// 현재 ID5 좌표를 비교해서 "도착"을 판정한다.
//
// 픽셀 좌표가 정확히 똑같이 나오기는 어려우므로
// POSITION_MATCH_TOLERANCE 이내면 일치로 본다.
// ============================================================

uint8_t ID5_ReachedTarget7(void)
{
	int32_t dx;
	int32_t dy;

	if (!id_detected[5] ||
	!target7_saved)
	{
		return 0;
	}

	dx =
	(int32_t)id_x[5] -
	(int32_t)target7_x;

	dy =
	(int32_t)id_y[5] -
	(int32_t)target7_y;

	if (labs(dx) <= POSITION_MATCH_TOLERANCE &&
	labs(dy) <= POSITION_MATCH_TOLERANCE)
	{
		return 1;
	}

	return 0;
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
// 모터 + 그리퍼 서보 공용 Timer1 초기화
//
// OC1A(왼쪽모터) / OC1B(오른쪽모터) / OC1C(그리퍼서보)는
// 전부 Timer1 하나를 공유하므로 레지스터를 한 번에 통일해서 설정한다.
//
// Mode 14 (Fast PWM, TOP=ICR1), 분주 8
// → 16MHz/8 = 2MHz 카운트 → ICR1=40000 → 20ms 주기 (서보 표준 프레임)
//
// 모터 속도는 기존처럼 0~255 스케일(TURN_SPEED 등)을 그대로 쓰고
// 실제 OCR1A/OCR1B에 넣을 때만 MOTOR_DUTY()로 40000 스케일로 환산한다.
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

	DDRB |=
	(1 << GRIPPER_SERVO_PIN);   // OC1C 출력 (그리퍼 서보)

	TCCR1A =
	(1 << COM1A1) |
	(1 << COM1B1) |
	(1 << COM1C1) |
	(1 << WGM11);

	TCCR1B =
	(1 << WGM13) |
	(1 << WGM12) |
	(1 << CS11);        // 분주 8

	ICR1 = PWM_TOP;

	OCR1A = 0;
	OCR1B = 0;

	Motor_Stop();

	Servo_SetAngle(GRIPPER_OPEN_ANGLE);   // 그리퍼 시작 각도 0도
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

	OCR1A = MOTOR_DUTY(speed);
	OCR1B = MOTOR_DUTY(speed);
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
	// 속도 범위 제한 (0~255 스케일 기준)
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
	MOTOR_DUTY((uint8_t)left_speed);

	OCR1B =
	MOTOR_DUTY((uint8_t)right_speed);
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

	OCR1A = MOTOR_DUTY(speed);
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
	OCR1B = MOTOR_DUTY(speed);
}


// ============================================================
// 그리퍼 서보 각도 설정 (0~270도)
//
// SERVO_MIN_US(500us) ~ SERVO_MAX_US(2500us)를 0~270도에 매핑
// 타이머 tick = 0.5us (2MHz) 이므로 us * 2 = tick
//
// 실제 서보의 270도 풀스윙 펄스 범위가 500~2500us가 맞는지는
// 데이터시트/실측으로 한 번 확인 후 필요하면 SERVO_MIN_US/MAX_US만 조정
// ============================================================

void Servo_SetAngle(uint16_t angle_deg)
{
	uint32_t pulse_us;
	uint16_t pulse_ticks;

	if (angle_deg > GRIPPER_ANGLE_MAX)
	{
		angle_deg = GRIPPER_ANGLE_MAX;
	}

	pulse_us =
	SERVO_MIN_US +
	((uint32_t)angle_deg *
	(SERVO_MAX_US - SERVO_MIN_US)) /
	GRIPPER_ANGLE_MAX;

	pulse_ticks =
	(uint16_t)(pulse_us * 2);

	OCR1C = pulse_ticks;
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
// 회전 시작 (ID5 → 현재 목표)
//
// ID5-ID6 단계, ID5-ID7 단계 모두
// 이 함수를 그대로 재사용한다.
// ============================================================

void StartRotation(void)
{
	// --------------------------------------------------------
	// 주의: 여기서 angle_z 를 0으로 리셋하지 않는다.
	//
	// HuskyLens 카메라가 외부(천장) 고정이라 id_angle 은
	// "로봇 기준 상대각"이 아니라 "카메라 화면 기준 절대 방위각"이다.
	//
	// angle_z 는 자이로 캘리브레이션 직후(main 함수) 또는
	// 리셋 스위치를 눌렀을 때만 0으로 잡고,
	// 그 이후로는 여러 구간(ID6 -> ID7)에 걸쳐
	// 계속 누적된 값을 그대로 사용해야
	// 카메라의 절대 방위각 기준과 일치한다.
	//
	// 여기서 다시 0으로 리셋하면, 이전 구간에서 실제로 회전한
	// 각도만큼 기준이 어긋나서 다음 구간이 과회전하게 된다.
	// --------------------------------------------------------

	target_angle =
	(float)id_angle;

	rotation_active = 1;
	rotation_finished = 0;

	tag_forward_active = 0;
	tag_forward_finished = 0;

	LED_SetStage(6);

	printf(
	"ROTATION START (ID5->ID%d)\n",
	current_target_id
	);

	printf(
	"TARGET ANGLE: %d deg\n",
	id_angle
	);
}


// ============================================================
// 회전 제어 (ID5 → 현재 목표)
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

		// ID5 좌표 0.2초 주기 갱신 타이머 리셋
		// (ID6 단계에서는 사용하지 않지만 초기화해 둬도 무해함)
		id5_update_timer_ms = 0;

		LED_SetStage(7);

		printf(
		"ROTATION COMPLETE (ID5->ID%d)\n",
		current_target_id
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
// 태그 거리 도달 확인 (ID5-ID6 단계 전용)
// ============================================================

uint8_t TagDistanceReached(void)
{
	if (!id_detected[5] ||
	!id_detected[current_target_id])
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
// ID5-ID6 도달 후 추가 시퀀스
//
// 1. 0.5초 전진
// 2. 그리퍼 서보 270도 회전
//
// 완료 후 current_target_id 를 7로 전환하는 것은
// 호출부(TagForward_Control)에서 그대로 이어서 처리한다.
// ============================================================

void Gripper_PostStopSequence(void)
{
	printf(
	"EXTRA FORWARD %dms\n",
	POST_STOP_FORWARD_MS
	);

	Motor_Forward(FORWARD_SPEED);

	_delay_ms(POST_STOP_FORWARD_MS);

	Motor_Stop();

	printf(
	"GRIPPER ROTATE TO %ddeg\n",
	GRIPPER_CLOSE_ANGLE
	);

	Servo_SetAngle(GRIPPER_CLOSE_ANGLE);

	_delay_ms(GRIPPER_MOVE_WAIT_MS);

	gripper_finished = 1;
}


// ============================================================
// 태그 거리 직진 제어 (ID5-현재 목표)
//
// 회전 완료 시점의 target_angle을
// 그대로 "직진 유지 목표각"으로 사용
//
// 직진 중에도 IMU를 계속 읽어서
// 현재각과 target_angle의 오차만큼
// 좌우 모터 속도를 보정 (라인트레이싱 방식)
//
// ------------------------------------------------------------
// [ID6 단계] (기존과 동일, 변경 없음)
//   45 pixel 이하가 되면:
//   -> 0.5초 추가 전진 + 그리퍼 270도 회전(Gripper_PostStopSequence)
//   -> 각도(angle_z) 초기화 없이 상태를 맨 처음(ID6 탐색 시작 전)과
//      동일하게 리셋하고 current_target_id 를 7로 바꿔서
//      똑같은 로직(메인 루프의 초기 탐색 -> StartRotation
//      -> Rotation_Control -> TagForward_Control)을
//      그대로 재사용해 ID7을 향해 다시 진행한다.
//
// [ID7 단계] (신규 로직)
//   - ID7 좌표는 회전 시작 직전(각도 계산 시점)에
//     target7_x/y 로 이미 고정 저장되어 있음 (더 이상 갱신 안 함)
//   - ID5 좌표만 0.2초 주기로 계속 갱신
//   - ID5 좌표가 target7_x/y 와 일치(오차범위 이내)하면
//     -> 정지, 그리퍼 0도로 초기화, 전체 과정 종료
// ------------------------------------------------------------

void TagForward_Control(void)
{
	float error;

	if (!tag_forward_active)
	{
		return;
	}


	// ========================================================
	// [ID7 단계] 좌표 일치 방식
	// ========================================================

	if (current_target_id == 7)
	{
		// ID5 좌표는 0.2초(ID5_UPDATE_INTERVAL_MS) 주기로만 갱신
		// 메인 루프가 10ms 마다 한 번씩 이 함수를 호출하므로
		// 그 사이에는 직진(각도 보정)만 계속 유지한다.

		id5_update_timer_ms += 10;

		if (id5_update_timer_ms <
		ID5_UPDATE_INTERVAL_MS)
		{
			error =
			AngleError(
			target_angle,
			angle_z
			);

			Motor_Forward_Straight(
			FORWARD_SPEED,
			error
			);

			return;
		}

		id5_update_timer_ms = 0;

		// ID5 좌표만 필요하지만 HuskyLens 프로토콜상
		// 전체 블록을 갱신해야 한다 (ID7은 더 이상 사용하지 않음).
		HuskyLens_Update();

		printf(
		"ID5 POS: X=%d Y=%d / TARGET7 X=%d Y=%d\n",
		id_x[5],
		id_y[5],
		target7_x,
		target7_y
		);

		if (ID5_ReachedTarget7())
		{
			Motor_Stop();

			tag_forward_active = 0;
			tag_forward_finished = 1;

			process_finished = 1;

			STAGE_LED_PORT = 0xFF;

			printf(
			"================================\n"
			);

			printf(
			"ID5 POSITION == TARGET7 POSITION\n"
			);

			printf(
			"MOTOR STOP\n"
			);

			Servo_SetAngle(
			GRIPPER_OPEN_ANGLE
			);

			printf(
			"GRIPPER RESET TO %ddeg\n",
			GRIPPER_OPEN_ANGLE
			);

			printf(
			"FULL PROCESS COMPLETE\n"
			);

			printf(
			"================================\n"
			);

			return;
		}

		error =
		AngleError(
		target_angle,
		angle_z
		);

		Motor_Forward_Straight(
		FORWARD_SPEED,
		error
		);

		return;
	}


	// ========================================================
	// [ID6 단계] 기존 픽셀 거리 방식 (변경 없음)
	// ========================================================

	HuskyLens_Update();

	id_distance =
	CalculateDistance();

	if (TagDistanceReached())
	{
		Motor_Stop();

		tag_forward_active = 0;
		tag_forward_finished = 1;

		printf(
		"================================\n"
		);

		printf(
		"ID5-ID%d DISTANCE <= %d PIXEL\n",
		current_target_id,
		TAG_DISTANCE_STOP
		);

		printf(
		"ID5-ID%d DISTANCE: %d PIXEL\n",
		current_target_id,
		id_distance
		);

		printf(
		"MOTOR STOP\n"
		);

		printf(
		"STAGE 1 COMPLETE\n"
		);

		Gripper_PostStopSequence();

		printf(
		"RESTART FROM BEGINNING FOR ID7\n"
		);

		printf(
		"================================\n"
		);

		current_target_id = 7;

		// angle_z 는 절대 방위각 기준을 유지해야 하므로
		// 여기서 리셋하지 않는다 (위 StartRotation 주석 참고).

		rotation_active = 0;
		rotation_finished = 0;

		tag_forward_active = 0;
		tag_forward_finished = 0;

		// ID7 단계 좌표 고정 저장 상태 초기화
		// (다음 탐색 블록에서 각도 계산 시점에 다시 저장됨)
		target7_saved = 0;

		id5_update_timer_ms = 0;

		LED_SetStage(4);

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
// LCD 상태 표시
// ============================================================

void LCD_DisplayStatus(void)
{
	LCD_Clear();

	// --------------------------------------------------------
	// 회전 중 (ID5-현재 목표)
	// --------------------------------------------------------

	if (rotation_active)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("TGT");

		LCD_PrintUInt(current_target_id);

		LCD_Print(":");

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
	// 태그까지 직진 중 (ID5-현재 목표, IMU 유지)
	//
	// ID7 단계에서는 픽셀 거리 대신 ID5/TARGET7 좌표를 보여준다.
	// --------------------------------------------------------

	if (tag_forward_active)
	{
		if (current_target_id == 7)
		{
			LCD_SetCursor(0, 0);

			LCD_Print("ID5:");

			LCD_PrintUInt(id_x[5]);

			LCD_Print(",");

			LCD_PrintUInt(id_y[5]);

			LCD_SetCursor(1, 0);

			LCD_Print("TG7:");

			LCD_PrintUInt(target7_x);

			LCD_Print(",");

			LCD_PrintUInt(target7_y);

			return;
		}

		LCD_SetCursor(0, 0);

		LCD_Print("DIST");

		LCD_PrintUInt(current_target_id);

		LCD_Print(":");

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
	// 최종 완료 (ID5-ID7 좌표 일치)
	// --------------------------------------------------------

	if (process_finished)
	{
		LCD_SetCursor(0, 0);

		LCD_Print("STOPPED");

		LCD_SetCursor(1, 0);

		LCD_Print("ID5==TG7");

		return;
	}


	// --------------------------------------------------------
	// 현재 목표(ID6 또는 ID7) 탐색 대기
	// --------------------------------------------------------

	if (id_detected[5] &&
	id_detected[current_target_id])
	{
		LCD_SetCursor(0, 0);

		LCD_Print("ANGLE");

		LCD_PrintUInt(current_target_id);

		LCD_Print(":");

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

		LCD_Print("ID5/ID");

		LCD_PrintUInt(current_target_id);

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

	Motor_Init();   // 모터 + 그리퍼 서보(0도) 초기화 포함

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
	// HuskyLens ID5 / 현재 목표(ID6) 탐색
	// ========================================================

	current_target_id = 6;

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
			// 전체 리셋 시점에는 로봇을 캘리브레이션 때와
			// 동일한 물리적 방향으로 다시 맞춰놓은 뒤
			// 리셋 스위치를 누른다고 가정하고 angle_z를 0으로 잡는다.
			// (카메라가 절대 방위각 기준이므로, 방향을 안 맞추고
			//  리셋하면 다시 기준이 어긋난다.)
			angle_z = 0.0;

			Motor_Stop();

			current_target_id = 6;

			rotation_active = 0;
			rotation_finished = 0;

			tag_forward_active = 0;
			tag_forward_finished = 0;

			process_finished = 0;

			gripper_finished = 0;

			// ID7 단계 좌표 고정 저장 상태도 초기화
			target7_saved = 0;
			id5_update_timer_ms = 0;

			Servo_SetAngle(GRIPPER_OPEN_ANGLE);   // 그리퍼도 0도로 리셋

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
		// 이미 ID5-ID7 정지 완료
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
		// 회전 중 + 직진 중(직진 유지 보정용) 모두 사용
		// ====================================================

		if (rotation_active ||
		tag_forward_active)
		{
			MPU6050_UpdateAngle(
			gyro_z_offset
			);
		}


		// ====================================================
		// 회전 전
		// HuskyLens ID5 / 현재 목표(current_target_id) 탐색
		//
		// ID5-ID6 단계, ID5-ID7 단계 모두
		// 완전히 동일한 이 블록을 그대로 재사용한다.
		//
		// current_target_id == 7 인 경우, 여기서 각도를
		// 계산하는 시점에 ID7 좌표를 target7_x/y 로 고정
		// 저장해 둔다 (이후 직진 중에는 ID7 좌표를 다시
		// 읽지 않고 이 값만 기준으로 사용).
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
			id_detected[current_target_id])
			{
				LED_SetStage(5);

				printf(
				"ID5: X=%d Y=%d\n",
				id_x[5],
				id_y[5]
				);

				printf(
				"ID%d: X=%d Y=%d\n",
				current_target_id,
				id_x[current_target_id],
				id_y[current_target_id]
				);

				printf(
				"ID DISTANCE: %d PIXEL\n",
				id_distance
				);

				printf(
				"ID ANGLE: %d DEG\n",
				id_angle
				);


				// ID5-ID7 단계: ID7 좌표를 여기서 한 번만 고정 저장
				if (current_target_id == 7 &&
				!target7_saved)
				{
					target7_x = id_x[7];
					target7_y = id_y[7];

					target7_saved = 1;

					printf(
					"TARGET7 SAVED: X=%d Y=%d\n",
					target7_x,
					target7_y
					);
				}


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


				// 회전 시작 (ID5-ID6 / ID5-ID7 공용)
				StartRotation();
			}
		}


		// ====================================================
		// 각도 회전 (ID5 → 현재 목표, ID6/ID7 공용)
		// ====================================================

		Rotation_Control();


		// ====================================================
		// 회전 완료 후 직진 (ID5 → 현재 목표)
		//
		// ID6: 픽셀 거리 45 이하에서 정지 (기존과 동일)
		// ID7: ID5 좌표 == 저장된 TARGET7 좌표에서 정지 (신규)
		//
		// ID6 완료 시 Gripper_PostStopSequence() 실행 후
		// current_target_id 가 7로 바뀌고 상태가 전부 리셋되므로,
		// 다음 루프부터는 위의 "회전 전 탐색" 블록이 자동으로
		// ID7을 찾기 시작한다.
		// ====================================================

		TagForward_Control();


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
