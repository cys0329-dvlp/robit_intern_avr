# Timer/Counter Register 설정 보고서

※ ATmega128 기준

## 목차

1. 8bit Timer/Counter0 with PWM and Asynchronous Operation
2. 16 bit Timer/Counter (Timer/Counter 1,3)
3. Timer/counter 1,2,3 Prescalers
4. 8 bit Timer/Counter2 with PWM

---

## 1. 8bit Timer/Counter0 with PWM and Asynchronous Operation

### 1-1. 개요

| Timer | 레지스터 |
| :--- | :--- |
| **Timer/Counter0** | TCNT0, OCR0, TCCR0, ASSR, TIMSK, TIFR |
| **Timer/Counter1** | TCNT1H/L, OCR1A/B/C, ICR1, TCCR1A/B/C, TIMSK, TIFR, ETIMSK, ETIFR |
| **Timer/Counter2** | TCNT2, OCR2, TCCR2, TIMSK, TIFR |
| **Timer/Counter3** | TCNT3H/L, OCR3A/B/C, ICR3, TCCR3A/B/C, ETIMSK, ETIFR |

### 1-2. 레지스터 설명

- **TCCR0 (Timer/Counter Control Register0)**

<img width="658" height="92" alt="image" src="https://github.com/user-attachments/assets/e66cf219-a77e-4f60-8172-bf948f9dd9e5"/>

    **Bit7 - FOC0 (Force Output Compare)**:
    - PWM모드가 아닌 경우에만 유효
    - 1로 설정 시 강제로 즉시 OCn 단자에 출력 비교가 매치 된것과 같은 출력 보냄
    - 출력 신호의 동작은 COMn1~0 비트에 의해 결정
    - 단지 OCn 단자에 신호만 출력할 뿐 해당 인터럽트 발생X, CTC모드에서 TCNTn 레지스터 클리어 X

    **Bit3,6 - WGM01~00 (Waveform Generation Mode)**:
    - Timer/Counter의 동작 모드(카운팅 방식, TOP 값, OCR0 갱신 시점)를 결정하는 비트
    - WGM01은 Bit3, WGM00은 Bit6에 위치 (두 비트가 레지스터 상에서 서로 떨어져 있음에 주의)


    **Bit5,4 - COM01~00 (Compare Match Output Mode)**:
    - OC0 핀의 동작을 설정하는 비트
    - PWM 모드가 아닌 경우, Fast PWM인 경우, Phase Correct PWM인 경우 각각 설정해야하는 비트가 달라진다


    **Bit2:0 - CS02~00 (Clock Select)**:
    - Timer/Counter0의 클럭 소스(prescaler 분주비)를 결정
    - Timer0는 비동기 모드 지원을 위해 Timer1,2,3과는 다른 별도의 prescaler 분주비를 가진다 (자세한 내용은 3장 참고)
    - clkT0S: Timer/Counter0의 클럭 소스(내부 clkI/O 또는 TOSC1/TOSC2에 연결된 외부 크리스탈)

- **TCNT0(Timer/Counter Register)**
<img width="634" height="96" alt="image" src="https://github.com/user-attachments/assets/fbf0fb59-7209-40d0-8a62-62eda16784c8" />

    **TCNT0 7:0**:
    - 8비트 카운더 값을 저장하는 레지스터
    - 언제나 Read/Write 가능
    - But, Counter가 동작 중 값 수정 시 문제 발생

- **OCR0(Output Compare Register)**
<img width="656" height="100" alt="image" src="https://github.com/user-attachments/assets/98c15a02-c437-4e7e-a59f-72adac74be27" />

     **OCR0 7:0**:
    - TCNT 값과 비교하여 OCn 단자에 출력 신호를 발생하기 위한 8비트를 저장하는 레지스터
    - TCNT0이 TOP 또는 BOTTOM에 도달했을 때 값이 갱심됨

- **ASSR(Asynchronous Status Register)**
<img width="644" height="96" alt="image" src="https://github.com/user-attachments/assets/185a38da-fa5a-4b5e-b47c-38b30ba57dab" />

    **Bit3 - AS0**:
    - clock 소스를 선택하는 비트
    - AS0 = 0으로 설정 -> 내부 클록이 선택 -> 동기 모드 작동
    - AS0 = 1로 설정 -> 외부 크리스탈 오실레이터로부터 단자에 입력되는 클록 선택 -> 비동기 모드 작동

    **Bit2 - TCN0UB**
    - 만약 AS0 = 1로 설정해서 비동기 모드로 동작할 때

        -> TCNT0 레지스터에 새로운 값을 Write하면 비트가 1로 set

        -> 이 값이 임시 레지스터로부터 TCNT0 레지스터에 옮겨져서 TCNT0의 Write가 완료되면 다시 0이 됨

    **Bit1 - OCR0UB**
    - 만약 AS0 = 1로 설정해서 비동기 모드로 동작할 때

        -> OCR0 레지스터에 새로운 값을 Write하면 비트가 1로 set

        -> 이 값이 임시 레지스터로부터 OCR0 레지스터에 옮겨져서 OCR0의 Write가 완료되면 다시 0이 됨

    **Bit0 - TCR0UB**
    - 만약 AS0 = 1로 설정해서 비동기 모드로 동작할 때

        -> TCCR0 레지스터에 새로운 값을 Write하면 비트가 1로 set

        -> 이 값이 임시 레지스터로부터 TCCR0 레지스터에 옮겨져서 TCCR0의 Write가 완료되면 다시 0이 됨

- **TIMSK (Timer/Counter Interrupt Mask Register)**
<img width="632" height="96" alt="image" src="https://github.com/user-attachments/assets/12f91a57-66af-4a93-abf9-907b929642f4" />

    **Bit7 - OCIE2**:
    - OCIE2 = 1 이고 SREG의 I비트(전역 인터럽트 허용)도 1로 설정된 경우
    - Timer/Counter2의 출력비교 
    인터럽트가 허용된 상태가 되어, TIFR의 OCF2 플래그가 set되면 해당 인터럽트가 실행됨

    **Bit6 - TOIE2**:
    - TOIE2 = 1이고 전역 인터럽트가 허용된 경우
    - Timer/Counter2의 오버플로우 인터럽트가 허용되어, TIFR의 TOV2 플래그가 set되면 해당 인터럽트가 실행됨

    **Bit5 - TICIE1**:
    - TICIE1 = 1이고 전역 인터럽트가 허용된 경우
    - Timer/Counter1의 입력 캡처 인터럽트가 허용되어, TIFR의 ICF1 플래그가 set되면 해당 인터럽트가 실행됨

    **Bit4 - OCIE1A**:
    - Timer/Counter1의 Output Compare A 매치 인터럽트 허용 비트
    - OCIE1A = 1이고 전역 인터럽트가 허용되면 TIFR의 OCF1A 플래그 set 시 인터럽트 실행

    **Bit3 - OCIE1B**:
    - Timer/Counter1의 Output Compare B 매치 인터럽트 허용 비트
    - OCIE1B = 1이고 전역 인터럽트가 허용되면 TIFR의 OCF1B 플래그 set 시 인터럽트 실행

    **Bit2 - TOIE1**:
    - Timer/Counter1의 오버플로우 인터럽트 허용 비트
    - TOIE1 = 1이고 전역 인터럽트가 허용되면 TIFR의 TOV1 플래그 set 시 인터럽트 실행

    **Bit1 - OCIE0**:
    - Timer/Counter0의 Output Compare 매치 인터럽트 허용 비트
    - OCIE0 = 1이고 전역 인터럽트가 허용되면 TIFR의 OCF0 플래그 set 시 인터럽트 실행

    **Bit0 - TOIE0**:
    - Timer/Counter0의 오버플로우 인터럽트 허용 비트
    - TOIE0 = 1이고 전역 인터럽트가 허용되면 TIFR의 TOV0 플래그 set 시 인터럽트 실행

    - 참고: TIMSK는 Timer0·Timer1·Timer2 관련 인터럽트를 모두 관리하며, Timer1의 Output Compare C와 Timer3 관련 인터럽트는 뒤에 나올 **ETIMSK**가 별도로 관리한다

- **TIFR (Timer/Counter Interrupt Flag Register)**
<img width="612" height="88" alt="image" src="https://github.com/user-attachments/assets/34ca36c3-69af-4506-b0dd-c1c8f39dad97" />

    **Bit7 - OCF2**:
    - Timer/Counter2의 TCNT2 값이 OCR2와 일치(Compare Match)하면 1로 set
    - 해당 인터럽트 서비스 루틴이 실행되면 하드웨어가 자동으로 Clear, 혹은 1을 write하여 소프트웨어적으로도 Clear 가능

    **Bit6 - TOV2**:
    - Timer/Counter2가 오버플로우되면 1로 set
    - 인터럽트 실행 시 자동 Clear, 혹은 1을 write하여 Clear 가능

    **Bit5 - ICF1**:
    - Timer/Counter1에서 Input Capture 이벤트(ICP1 핀 또는 아날로그 비교기)가 발생하면 1로 set

    **Bit4 - OCF1A**:
    - TCNT1 값이 OCR1A와 일치하면 1로 set

    **Bit3 - OCF1B**:
    - TCNT1 값이 OCR1B와 일치하면 1로 set

    **Bit2 - TOV1**:
    - Timer/Counter1이 오버플로우(혹은 WGM 설정에 따라 TOP/BOTTOM 도달)되면 1로 set

    **Bit1 - OCF0**:
    - TCNT0 값이 OCR0와 일치하면 1로 set

    **Bit0 - TOV0**:
    - Timer/Counter0이 오버플로우되면 1로 set

---

## 2. 16 bit Timer/Counter (Timer/Counter 1,3)

### 2-1. 개요

- Timer/Counter1과 Timer/Counter3은 동일한 구조를 가진 16비트 타이머
- 8비트 타이머와 달리 Output Compare 유닛이 A/B/C 3개, Input Capture 기능 보유
- 레지스터가 16비트이므로 HIGH/LOW바이트로 나뉘어 있고, 임시 레지스터(Temporary Register)를 거쳐 8비트 버스로 원자적(atomic)으로 처리됨
- Timer3의 각 레지스터는 이름의 "1"이 "3"으로 바뀐 것을 제외하면 Timer1과 완전히 동일하게 동작 → 아래에서는 함께 설명

### 2-2. 레지스터 설명

- **TCCR1A / TCCR3A (Timer/Counter1,3 Control Register A)**
<img width="648" height="220" alt="image" src="https://github.com/user-attachments/assets/ac08b688-7881-4ec4-8020-0d04216bffaa" />

    **Bit7,6 - COM1A1~0**:
    - OC1A 핀 동작 설정

    **Bit5,4 - COM1B1~0**:
    - OC1B 핀 동작 설정

    **Bit3,2 - COM1C1~0**:
    - OC1C 핀 동작 설정 (Timer1,3만 가지는 세 번째 출력비교 채널)

    - COM1x1~0 비트는 Non-PWM/Fast PWM/Phase Correct PWM 모드별로 비트를 다르게 설정해야한다. 
<img width="646" height="268" alt="image" src="https://github.com/user-attachments/assets/a289173c-db24-4b27-b042-5e5ee50f8696" />

<img width="642" height="372" alt="image" src="https://github.com/user-attachments/assets/dc701ee0-0308-408c-882e-1b1e79d76209" />


<img width="664" height="398" alt="image" src="https://github.com/user-attachments/assets/2a3a852b-7296-46a8-962e-4b38f69f2bcf" />

    **Bit1,0 - WGM11~10**:
    - TCCR1B의 WGM13~12와 합쳐서 4비트로 동작 모드를 결정 

- **TCCR1B / TCCR3B (Timer/Counter1,3 Control Register B)**

    **Bit7 - ICNC1 (Input Capture Noise Canceler)**:
    - 1로 설정 시 입력 캡처 노이즈 캔슬러 활성화
    - ICP1 입력이 4번의 연속된 클럭에서 동일한 값이 나올 때만 유효한 edge로 인정 → 노이즈에 의한 오동작 방지, 대신 캡처가 실제 edge보다 4클럭 지연됨

    **Bit6 - ICES1 (Input Capture Edge Select)**:
    - ICES1 = 0: ICP1의 Falling edge에서 캡처
    - ICES1 = 1: ICP1의 Rising edge에서 캡처
    - 캡처 이벤트 발생 시 TCNT1 값이 ICR1로 복사되고 ICF1 플래그 set

    **Bit4,3 - WGM13~12**:
    - TCCR1A의 WGM11~10과 합쳐 아래 표와 같이 16비트 타이머의 동작 모드를 결정

    - Timer/Counter1의 클럭 소스(prescaler) 선택 비트
    - Timer2,3과 prescaler를 공유하며 자세한 분주비 표는 3장에서 다룸

- **TCCR1C / TCCR3C (Timer/Counter1,3 Control Register C)**

    **Bit7 - FOC1A**:
    - PWM 모드가 아닐 때만 유효, 1로 설정 시 강제로 OC1A에 Compare Match와 동일한 출력 발생
    - TCCR0의 FOC0와 동일한 개념 (인터럽트 발생 X, TOP/BOTTOM 값 clear 안 함)

    **Bit6 - FOC1B**:
    - 위와 동일하되 OC1B 대상

    **Bit5 - FOC1C**:
    - 위와 동일하되 OC1C 대상

- **TCNT1H/L, TCNT3H/L (Timer/Counter1,3 Register)**

    **TCNT1 15:0**:
    - 16비트 카운터 값을 저장, TCNT1H(상위 8비트)와 TCNT1L(하위 8비트)로 나뉨
    - CPU가 16비트 레지스터에 접근할 때는 반드시 임시 레지스터를 거쳐 상/하위 바이트가 한 번에 갱신되도록 하여 원자성을 보장 → 8비트 버스로 16비트 레지스터를 안전하게 Read/Write 가능
    - Read 시 항상 TCNT1L을 먼저 읽고, Write 시 항상 TCNT1H를 먼저 써야 함
    - Counter 동작 중 값을 수정하면 Compare Match를 놓치는 등의 문제가 발생할 수 있음

- **OCR1AH/L, OCR1BH/L, OCR1CH/L, OCR3AH/L, OCR3BH/L, OCR3CH/L (Output Compare Register)**

    **OCR1x 15:0**:
    - TCNT1과 비교되어 OC1x 단자에 출력을 만들기 위한 16비트 값을 저장
    - TCNT1과 마찬가지로 임시 레지스터를 거쳐 상/하위 바이트가 동시에 갱신됨
    - Double Buffering 지원: PWM 모드에서는 TOP(혹은 BOTTOM) 시점에 실제 비교값이 갱신되어 글리치 없는 파형 생성 가능

- **ICR1H/L, ICR3H/L (Input Capture Register)**

    **ICR1 15:0**:
    - ICP1(Input Capture Pin) 단자에 지정된 edge(ICES1으로 설정)가 감지되면 그 순간의 TCNT1 값이 자동으로 저장되는 레지스터
    - WGM13:10 설정에 따라 TOP 값으로도 사용 가능 (주파수 가변 PWM 구현 시 활용)
    - 캡처 시 ICF1 플래그가 set되며, ICNC1으로 노이즈 캔슬러 적용 가능

- **ETIMSK (Extended Timer/Counter Interrupt Mask Register)**

    **Bit5 - TICIE3**:
    - Timer/Counter3의 Input Capture 인터럽트 허용 비트
    - TICIE3 = 1이고 전역 인터럽트 허용 시, ETIFR의 ICF3 set되면 인터럽트 실행

    **Bit4 - OCIE3A**:
    - Timer/Counter3의 Output Compare A 인터럽트 허용 비트

    **Bit3 - OCIE3B**:
    - Timer/Counter3의 Output Compare B 인터럽트 허용 비트

    **Bit2 - TOIE3**:
    - Timer/Counter3의 오버플로우 인터럽트 허용 비트

    **Bit1 - OCIE3C**:
    - Timer/Counter3의 Output Compare C 인터럽트 허용 비트

    **Bit0 - OCIE1C**:
    - Timer/Counter1의 Output Compare C 인터럽트 허용 비트 (TIMSK가 아닌 ETIMSK에서 관리됨에 유의)

    - Bit7:6은 사용하지 않는 예약 비트(항상 0)

- **ETIFR (Extended Timer/Counter Interrupt Flag Register)**

    **Bit5 - ICF3**:
    - Timer/Counter3에서 Input Capture 이벤트 발생 시 1로 set

    **Bit4 - OCF3A**:
    - TCNT3 값이 OCR3A와 일치하면 1로 set

    **Bit3 - OCF3B**:
    - TCNT3 값이 OCR3B와 일치하면 1로 set

    **Bit2 - TOV3**:
    - Timer/Counter3가 오버플로우되면 1로 set

    **Bit1 - OCF3C**:
    - TCNT3 값이 OCR3C와 일치하면 1로 set

    **Bit0 - OCF1C**:
    - TCNT1 값이 OCR1C와 일치하면 1로 set

---

## 3. Timer/counter 1,2,3 Prescalers

### 3-1. 개요

- Timer/Counter1, 2, 3은 하나의 10비트 prescaler(분주 회로)를 공유
- Timer/Counter0은 비동기 동작 지원을 위해 별도의 prescaler를 독립적으로 사용 (1장의 CS02~00 참고)

### 3-2. Internal Clock Source

- CSn2:0 (CS12:10 / CS22:20 / CS32:30) 값에 따른 클럭 분주비는 Timer1,2,3 모두 아래의 동일한 표를 따른다

<img width="650" height="288" alt="image" src="https://github.com/user-attachments/assets/2e267a38-9c12-407b-8162-e4ab92f964a9" />


### 3-3. Prescaler Reset

- **SFIOR (Special Function IO Register)**

    **Bit7 - TSM (Timer/Counter Synchronization Mode)**:
    - 1로 설정 시 Timer/Counter 동기화 모드 활성화
    - PSR0, PSR321에 write한 값이 그대로 유지되어 해당 prescaler가 계속 리셋 상태로 정지 → 여러 타이머를 정확히 같은 시점에 동시에 시작시키고 싶을 때 사용
    - TSM = 0으로 clear하면 PSR0, PSR321이 하드웨어에 의해 자동으로 clear되며 각 Timer/Counter가 동시에 카운팅 시작

    **Bit3 - ACME (Analog Comparator Multiplexer Enable)**:
    - 아날로그 비교기 입력에 ADC 멀티플렉서를 연결할지 결정하는 비트 (Timer와 직접 관련 없음)

    **Bit2 - PUD (Pull-up Disable)**:
    - 모든 포트의 내부 Pull-up 저항을 비활성화하는 비트 (Timer와 직접 관련 없음)

    **Bit1 - PSR0 (Prescaler Reset Timer/Counter0)**:
    - 1로 set하면 Timer/Counter0의 prescaler가 리셋됨
    - 일반적으로 하드웨어에 의해 즉시 다시 clear되나, 비동기 모드에서는 실제 리셋이 완료될 때까지 1로 유지됨
    - TSM = 1인 동안에는 clear되지 않음

    **Bit0 - PSR321 (Prescaler Reset Timer/Counter3,2,1)**:
    - 1로 set하면 Timer/Counter1,2,3이 공유하는 prescaler가 리셋됨
    - TSM = 1인 동안에는 clear되지 않고, TSM이 0이 되는 순간 하드웨어가 clear하며 3개 타이머가 동시에 카운팅 시작

### 3-4. External Clock Source

- Timer/Counter1,2,3은 각각 T1, T2, T3 핀을 통해 외부 클럭을 입력받아 카운터 클럭으로 사용할 수 있음 (CSn2:0 = 110 또는 111)
- 외부 클럭은 내부에서 프리스케일링 되지 않고 그대로 카운터 클럭으로 사용됨
- Tn 핀으로 입력된 신호는 동기화 로직을 거쳐 edge가 감지되므로, 안정적인 카운트를 위해 외부 클럭 주파수는 시스템 클럭의 1/2.5 이하로 유지하는 것을 권장

---

## 4. 8 bit Timer/Counter2 with PWM

### 4-1. 개요

- Timer/Counter0과 거의 동일한 구조를 가진 8비트 타이머
- 단, Timer0과 달리 비동기 동작(ASSR)은 지원하지 않으며, 대신 Timer1,3과 함께 공용 prescaler를 사용
- 외부 이벤트 입력 핀을 통한 외부 클럭 입력 지원

### 4-2. 레지스터 설명

- **TCCR2 (Timer/Counter2 Control Register)**

    **Bit7 - FOC2**:
    - PWM 모드가 아닐 때만 유효, 1로 설정 시 강제로 OC2 단자에 Compare Match와 동일한 출력 발생 (TCCR0의 FOC0와 동일 개념)

    **Bit3,6 - WGM21~20**:
    - TCCR0의 WGM01~00과 완전히 동일한 표를 따름 (Normal / PWM Phase Correct / CTC / Fast PWM)
    - WGM21은 Bit3, WGM20은 Bit6에 위치

    **Bit5,4 - COM21~20**:
    - OC2 핀 동작 설정, TCCR0의 COM01~00과 완전히 동일한 표를 따름

    **Bit2:0 - CS22~20**:
    - Timer/Counter2의 클럭 소스 선택
    - Timer1,3과 prescaler를 공유하므로 3장의 Internal Clock Source 표(clk/1, /8, /64, /256, /1024, 외부클럭)를 그대로 따름 (Timer0의 CS02~00과는 분주비가 다름에 유의)

- **TCNT2 (Timer/Counter2 Register)**

    **TCNT2 7:0**:
    - 8비트 카운터 값을 저장, 언제나 Read/Write 가능
    - TCNT0과 마찬가지로 카운터 동작 중 값 수정 시 Compare Match를 한 클럭 놓치는 등의 문제가 발생할 수 있음

- **OCR2 (Output Compare Register2)**

    **OCR2 7:0**:
    - TCNT2 값과 비교되어 OC2 단자에 출력을 만들기 위한 8비트 값을 저장하는 레지스터
    - TCNT2가 TOP 또는 BOTTOM에 도달했을 때 실제 값이 갱신됨(Double Buffering)
