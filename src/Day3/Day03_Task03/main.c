#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include "i2c_lcd.h"

/* ------------------------- 하드웨어/프로토콜 상수 ------------------------- */

#define DXL_ID                  1
#define RS485_DIR_PIN           PE2

#define REG_TORQUE_ENABLE       64
#define REG_PROFILE_VELOCITY    112
#define REG_GOAL_POSITION       116

#define POT_SAMPLES             8
#define POS_DEADBAND            3

#define DEFAULT_SPEED_LEVEL     3

#define UBRR_DXL                34
#define UBRR_PC                 103

/* ------------------------- RS485 방향 제어 ------------------------- */

static inline void rs485_set_tx(void)
{
	PORTE |= (1 << RS485_DIR_PIN);
}

static inline void rs485_set_rx(void)
{
	PORTE &= (uint8_t)~(1 << RS485_DIR_PIN);
}

/* ------------------------- USART0: Dynamixel 버스 ------------------------- */

static void dxl_bus_init(void)
{
	DDRE |= (1 << RS485_DIR_PIN);
	rs485_set_rx();

	UCSR0A = (1 << U2X0);              /* 2배속 모드 */
	UBRR0H = (uint8_t)(UBRR_DXL >> 8);
	UBRR0L = (uint8_t)UBRR_DXL;        /* 57600bps */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8N1 */
}

static void dxl_bus_put(uint8_t b)
{
	while ((UCSR0A & (1 << UDRE0)) == 0)
	{
		/* 송신 버퍼 대기 */
	}
	UDR0 = b;
}

static void dxl_bus_drain_rx(void)
{
	while (UCSR0A & (1 << RXC0))
	{
		(void)UDR0;
	}
}

/* ------------------------- USART1: PC 통신 ------------------------- */

static void pc_link_init(void)
{
	UCSR1A = 0;
	UBRR1H = (uint8_t)(UBRR_PC >> 8);
	UBRR1L = (uint8_t)UBRR_PC;        /* 9600bps */
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

static inline uint8_t pc_link_has_data(void)
{
	return (UCSR1A & (1 << RXC1)) ? 1 : 0;
}

static inline uint8_t pc_link_get(void)
{
	return UDR1;
}

/* ------------------------- 가변저항(ADC) ------------------------- */

static void pot_adc_init(void)
{
	DDRF &= (uint8_t)~(1 << DDF0);
	PORTF &= (uint8_t)~(1 << PF0);

	ADMUX = (1 << REFS0);              /* AVCC 기준, ADC0 채널 */
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	_delay_ms(1);

	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
	{
		/* 워밍업 변환 대기 */
	}
	(void)ADC;                          /* 첫 결과는 버림 */
}

static uint16_t pot_adc_sample(void)
{
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
	{
		/* 변환 완료 대기 */
	}
	return ADC;
}

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

/* ------------------------- Dynamixel Protocol 2.0 ------------------------- */

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

/* Write 명령 패킷을 조립하여 전송하고, 응답 Status Packet은 버린다 */
static void dxl_write_reg(uint16_t reg_addr, const uint8_t *payload, uint8_t payload_len)
{
	uint8_t inst[16];
	uint8_t stuffed[20];
	uint8_t frame[32];

	uint8_t inst_len;
	uint8_t stuffed_len = 0;
	uint8_t frame_len;

	uint8_t k;
	uint16_t param_len;
	uint16_t crc_val;

	inst[0] = 0x03;                          /* Instruction: WRITE */
	inst[1] = (uint8_t)(reg_addr & 0xFF);
	inst[2] = (uint8_t)(reg_addr >> 8);

	for (k = 0; k < payload_len; k++)
	{
		inst[3 + k] = payload[k];
	}
	inst_len = (uint8_t)(3 + payload_len);

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

	param_len = (uint16_t)(stuffed_len + 2);   /* +CRC 2바이트 */

	frame[0] = 0xFF;
	frame[1] = 0xFF;
	frame[2] = 0xFD;
	frame[3] = 0x00;
	frame[4] = DXL_ID;
	frame[5] = (uint8_t)(param_len & 0xFF);
	frame[6] = (uint8_t)(param_len >> 8);

	for (k = 0; k < stuffed_len; k++)
	{
		frame[7 + k] = stuffed[k];
	}

	crc_val = crc16_dxl_update(0, frame, (uint16_t)(7 + stuffed_len));
	frame[7 + stuffed_len] = (uint8_t)(crc_val & 0xFF);
	frame[8 + stuffed_len] = (uint8_t)(crc_val >> 8);

	frame_len = (uint8_t)(9 + stuffed_len);

	dxl_bus_drain_rx();

	rs485_set_tx();
	_delay_us(10);
	UCSR0A |= (1 << TXC0);

	for (k = 0; k < frame_len; k++)
	{
		dxl_bus_put(frame[k]);
	}
	while ((UCSR0A & (1 << TXC0)) == 0)
	{
		/* 마지막 바이트가 실제로 나갈 때까지 대기 */
	}

	rs485_set_rx();

	_delay_ms(3);
	dxl_bus_drain_rx();
}

static void dxl_write_u8(uint16_t reg_addr, uint8_t val)
{
	uint8_t buf[1] = { val };
	dxl_write_reg(reg_addr, buf, 1);
}

static void dxl_write_u32(uint16_t reg_addr, uint32_t val)
{
	uint8_t buf[4];
	buf[0] = (uint8_t)(val);
	buf[1] = (uint8_t)(val >> 8);
	buf[2] = (uint8_t)(val >> 16);
	buf[3] = (uint8_t)(val >> 24);
	dxl_write_reg(reg_addr, buf, 4);
}

/* ------------------------- 유틸리티 ------------------------- */

/* PC에서 들어온 0~9 숫자를 속도 0~300 범위로 스케일링 */
static uint32_t speed_level_to_value(uint8_t level)
{
	return ((uint32_t)level * 300UL) / 9UL;
}

static uint16_t u16_diff(uint16_t a, uint16_t b)
{
	return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static void lcd_show_row(uint8_t row, const char *text)
{
	char buf[17];
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		buf[i] = ' ';
	}
	buf[16] = '\0';

	for (i = 0; text[i] != '\0' && i < 16; i++)
	{
		buf[i] = text[i];
	}

	lcd_set_cursor(row, 0);
	lcd_print_text(buf);
}

static void lcd_show_status(uint32_t speed_val, uint16_t pos_val)
{
	char row0[17];
	char row1[17];

	snprintf(row0, sizeof(row0), "SPEED: %3lu", (unsigned long)speed_val);
	snprintf(row1, sizeof(row1), "POSITION: %4u", (unsigned int)pos_val);

	lcd_show_row(0, row0);
	lcd_show_row(1, row1);
}

/* ------------------------- 메인 루프 ------------------------- */

int main(void)
{
	uint32_t cur_speed;
	uint16_t cur_pos;
	uint16_t sent_pos;
	uint8_t resend_pos_flag;

	lcd_module_begin();
	pot_adc_init();
	pc_link_init();
	dxl_bus_init();

	_delay_ms(500);

	cur_speed = speed_level_to_value(DEFAULT_SPEED_LEVEL);
	cur_pos = pot_adc_average();
	sent_pos = cur_pos;

	lcd_show_status(cur_speed, cur_pos);

	dxl_write_u8(REG_TORQUE_ENABLE, 0);
	dxl_write_u32(REG_PROFILE_VELOCITY, cur_speed);
	dxl_write_u8(REG_TORQUE_ENABLE, 1);

	_delay_ms(100);

	dxl_write_u32(REG_GOAL_POSITION, cur_pos);

	resend_pos_flag = 0;

	for (;;)
	{
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

		lcd_show_status(cur_speed, cur_pos);

		_delay_ms(50);
	}

	return 0;
}
