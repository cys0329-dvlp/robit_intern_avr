#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// =====================================================
// UART0
//
// HuskyLens TX -> PE0 (RXD0)
// HuskyLens RX <- PE1 (TXD0)
// =====================================================

#define BAUD 9600
#define UBRR_VALUE ((F_CPU / 16UL / BAUD) - 1)

#define UART_TIMEOUT 60000UL


// =====================================================
// HuskyLens Protocol
// =====================================================

#define HEADER1             0x55
#define HEADER2             0xAA
#define ADDRESS             0x11

#define REQUEST_BLOCKS      0x21
#define RETURN_INFO         0x29
#define RETURN_BLOCK        0x2A


// =====================================================
// LED
//
// PA5 = 태그 인식
// PA3 = 태그 미인식
// =====================================================

void LED_Init(void)
{
    DDRA |= (1 << PA5);
    DDRA |= (1 << PA3);

    // 초기 상태
    PORTA &= ~(1 << PA5);
    PORTA |=  (1 << PA3);
}


void LED_TagDetected(void)
{
    PORTA |=  (1 << PA5);
    PORTA &= ~(1 << PA3);
}


void LED_TagNotDetected(void)
{
    PORTA &= ~(1 << PA5);
    PORTA |=  (1 << PA3);
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

    // 8bit / 1 stop / no parity
    UCSR0C =
        (1 << UCSZ01) |
        (1 << UCSZ00);
}


// =====================================================
// UART 송신
// =====================================================

void UART0_SendByte(uint8_t data)
{
    while (!(UCSR0A & (1 << UDRE0)));

    UDR0 = data;
}


// =====================================================
// UART 수신
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
// UART RX 버퍼 비우기
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
// HuskyLens 요청
//
// 55 AA 11 00 21 31
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
// 패킷 하나 읽기
//
// 반환:
// 1 = 정상 수신
// 0 = timeout / 오류
//
// buffer 구조:
//
// [0] 55
// [1] AA
// [2] 11
// [3] Length
// [4] Command
// [5...] Data
// [마지막] Checksum
// =====================================================

uint8_t HuskyLens_ReadPacket(uint8_t *buffer)
{
    uint8_t data;
    uint8_t length;
    uint8_t index = 0;


    // -------------------------------------------------
    // Header 0x55 찾기
    // -------------------------------------------------

    while (1)
    {
        if (!UART0_ReadByteTimeout(&data))
        {
            return 0;
        }

        if (data == HEADER1)
        {
            break;
        }
    }

    buffer[index++] = data;


    // -------------------------------------------------
    // Header 0xAA
    // -------------------------------------------------

    if (!UART0_ReadByteTimeout(&data))
    {
        return 0;
    }

    if (data != HEADER2)
    {
        return 0;
    }

    buffer[index++] = data;


    // -------------------------------------------------
    // Address
    // -------------------------------------------------

    if (!UART0_ReadByteTimeout(&data))
    {
        return 0;
    }

    if (data != ADDRESS)
    {
        return 0;
    }

    buffer[index++] = data;


    // -------------------------------------------------
    // Data Length
    // -------------------------------------------------

    if (!UART0_ReadByteTimeout(&length))
    {
        return 0;
    }

    buffer[index++] = length;


    // 비정상 길이 방지
    if (length > 30)
    {
        return 0;
    }


    // -------------------------------------------------
    // Command + Data
    // -------------------------------------------------

    for (uint8_t i = 0; i < length + 1; i++)
    {
        if (!UART0_ReadByteTimeout(&data))
        {
            return 0;
        }

        buffer[index++] = data;
    }


    // -------------------------------------------------
    // Checksum
    // -------------------------------------------------

    if (!UART0_ReadByteTimeout(&data))
    {
        return 0;
    }

    buffer[index++] = data;


    return 1;
}


// =====================================================
// HuskyLens Tag 확인
//
// 반환:
// 1 = 태그 인식
// 0 = 태그 없음
// =====================================================

uint8_t HuskyLens_TagDetected(void)
{
    uint8_t packet[40];

    uint16_t detected_count;


    // -------------------------------------------------
    // 이전 데이터 제거
    // -------------------------------------------------

    UART0_ClearBuffer();


    // -------------------------------------------------
    // HuskyLens에 BLOCK 요청
    // -------------------------------------------------

    HuskyLens_RequestBlocks();


    // -------------------------------------------------
    // INFO 패킷 기다림
    // -------------------------------------------------

    if (!HuskyLens_ReadPacket(packet))
    {
        return 0;
    }


    // -------------------------------------------------
    // 반드시 RETURN_INFO인지 확인
    // -------------------------------------------------

    if (packet[4] != RETURN_INFO)
    {
        return 0;
    }


    /*
     * RETURN_INFO 구조
     *
     * packet[5]  = block/arrow 개수 Low
     * packet[6]  = block/arrow 개수 High
     *
     * packet[7]  = 학습된 ID 개수 Low
     * packet[8]  = 학습된 ID 개수 High
     *
     * packet[9]  = frame Low
     * packet[10] = frame High
     *
     * packet[11~14] = reserved
     */


    detected_count =
        ((uint16_t)packet[6] << 8) |
        packet[5];


    // -------------------------------------------------
    // 검출된 객체가 0개
    // -------------------------------------------------

    if (detected_count == 0)
    {
        return 0;
    }


    // -------------------------------------------------
    // 객체가 1개 이상이면
    // 실제 BLOCK 패킷을 확인
    // -------------------------------------------------

    if (!HuskyLens_ReadPacket(packet))
    {
        return 0;
    }


    // -------------------------------------------------
    // 실제 BLOCK인지 확인
    // -------------------------------------------------

    if (packet[4] == RETURN_BLOCK)
    {
        /*
         * BLOCK 데이터
         *
         * packet[5]  = X Low
         * packet[6]  = X High
         *
         * packet[7]  = Y Low
         * packet[8]  = Y High
         *
         * packet[9]  = Width Low
         * packet[10] = Width High
         *
         * packet[11] = Height Low
         * packet[12] = Height High
         *
         * packet[13] = ID Low
         * packet[14] = ID High
         */

        return 1;
    }


    return 0;
}


// =====================================================
// MAIN
// =====================================================

int main(void)
{
    LED_Init();

    UART0_Init();


    // HuskyLens 부팅 대기
    _delay_ms(2000);


    while (1)
    {
        // -------------------------------------------------
        // 태그 인식 여부
        // -------------------------------------------------

        if (HuskyLens_TagDetected())
        {
            // =============================================
            // 태그 인식
            //
            // PA5 ON
            // PA3 OFF
            // =============================================

            LED_TagDetected();
        }
        else
        {
            // =============================================
            // 태그 미인식
            //
            // PA5 OFF
            // PA3 ON
            // =============================================

            LED_TagNotDetected();
        }


        _delay_ms(100);
    }
}
