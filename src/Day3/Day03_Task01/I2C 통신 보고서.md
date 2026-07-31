# I2C 통신의 SCL, SDA에 대한 보고서

*수습단원 최윤서*

## I2C 통신 시스템

### 개요)

I2C(Inter Intergrated Circuit)은 TWI(Two-wire-Serial Interface)과 동일한 기능으로
2개의 선을 사용하고 하나는 SCL, SDA입니다.
Master와 Slave Operation 이 지원되며 Device는 Transmitter와 Receiver로 동작할 수 있습니다.

### 구성)

I2C 통신은 SDA, SCL 두 개의 전선으로 구성되며 풀업 저항을 통해 전원에 연결되어있습니다. I2C는 여러 장치가 동일한 선을 공유하는 방식이므로 Open-drain 방식을 사용하여 회로를 연결한다.

- 각 장치의 단자 회로는 Open-drain 혹은 Open-collector 형식으로 구성되어있음
- SDA: 실제 데이터가 전송되는 신호선, SCL에 동기되어 0과 1의 신호를 주고받음
- SCL: 통신 속도와 기준 박자를 맞추는 클럭 신호선, 주로 마스터에서 만들어냄
- Open-drain: Drain이 Output Pin에 연결된 회로 구조

![alt text](image.png)

### Data 통신 방법)

SDA 신호가 LOW로 떨어질 때 시작 신호라고 판단하여 SCL 클럭 신호가 만들어짐
클럭 신호가 HIGH일 때 SDA 신호를 읽고 LOW면 다시 비트신호로 바꾸는 식을
반복하여 SCL이 HIGH일 때 값을 측정한다.
만약 모든 데이터의 전송이 끝난 후 SCL과 SDA가 모두 HIGH라면 Stop 신호를 만들어 정지한다.

![alt text](image-1.png)

### Master와 Slave)

2개 이상의 장치가 데이터를 주고받을 때 다른 장치들에게 명령을 주는 장치를 Master,
명령을 받고 명령을 실행하는 장치를 Slave라고 합니다.
Slave 장치는 어떠한 데이터도 먼저 전송할 수 없고 항상 신호에 응답만 할 수 있습니다.
Master에는 여러개의 Slave를 연결할 수 있는데 이때 Slave는 고유 주소로 구분합니다.

## 2. Data Transfer & Frame Format

### Bit 전송)

각각의 데이터 Bit는 SCL의 Clock pulse에 동기되어 전달됩니다.
Start와 Stop 신호를 받을 때를 제외하고는 데이터가 전송될 때 동기되는 SCL의 값이 안정적으로 유지되어야 Data가 제대로 전달되고 만약 일정하게 유지되지않는다면 Data가 변화했음을 알 수 있습니다.
사진.2를 보면 SDA에 입력되는 Data가 대부분의 영역에서 동기됨을 알 수 있고
Data가 바뀌었을 때 Data상태가 Chage하는 것을 알 수 있습니다.

![alt text](image-2.png)

### Start/Stop 신호)

Master는 데이터 전송 시작 및 종료합니다.
만약 Start와 Stop 사이의 신호라면 Bus는 Busy 상태로 간주되고 Busy 상태일 때는 어떠한 master도 Bus를 제어할 수 없습니다.
특별한 경우 새로운 Start 상태가 Start와 Stop 사이에서 발생하는 것입니다.
이를 Repeated Start라고 정의합니다.
사진.3을 참고하면 두 번째 Start와 Stop 사이에 Start가 한번 더 발생했다면 그 Start는
Repeated Start로 정의됩니다.
Repeated Start가 실행된 후로도 Stop 비트가 나오기 전까지 Bus는 Busy한 상태이고
이는 Start의 기능과 동일한 역할을 하게 됩니다.

![alt text](image-3.png)

### Adress Packet Format)

Adress bit 7개, Read/Write bit 1개, Acknowledge bit 1개로 총 9개의 비트로 구성되어있습니다.

### Data Packet Format)

Data bit 8개, Acknowledge bit 1개로 구성되어있습니다.
Data가 전송되는 동안 Master는 클럭 신호와 Start, Stop신호를 발생시킵니다.

## 3. 전송 Mode

### Master Transmitter(MT))

Master에서 Slave로 Data를 전송하는 Mode(Master -> Slave)
Start 신호가 전송된 다음 SLA+W 신호가 전송되면 Master Transmitter가 시작되고
Start 신호가 전송된 다음 SLA+R신호가 전송되면 Master Receiver가 시작된다.

### Master Receiver(MR))

Master가 Slave에게 Data를 받는 Mode이다.(Master <- Slave)
Start 신호가 전송된 다음 SLA+R 신호가 전송되면 Master Receiver가 시작된다.

### Slave Transmitter(ST))

Master에서 보낸 정보를 Slave가 수신하는 Mode(Slave <- Master)

### Slave Receiver(SR))

Slave가 Master에게 데이터를 전송하는 Mode(Slave ->Master)

## 4. Register 설명

### TWBR)

Bits 7:0
TWBR은 Master Mode에서 SCL 클럭 유동성을 결정하기 위한 bit rate generator Division factor을 선택하는 데에 사용된다.
**bit rate: 1초 동안 전송되거나 처리되는 데이터의 양(비트 수)
![alt text](image-4.png)
### TWCR)

- Bit0: TWI Interrupt Enable, SREG의 I-bit와 TWIE에가 Set된 상태에서 TWINT flag가 High로 되면 TWI Interrupt Request가 활성화 된다.
- Bit1: Reserved Bit, 사용되지않는 예약된 비트이며 항상 0이다.
- Bit2: TWI 작동 활성화, TWI 인터페이스 활성화
- Bit3: TWINT이 Low인 상태에서 TWI Data Register에 Write 하는 경우 발생함
  High인 상태에서는 Write하면 Clear 된다.
- Bit4: Master Mode에서 TWSTO bit에 1 -> STOP신호 생성됨
  STOP 신호가 생성되면 TWSTO bit는 자동 초기화
  Slave Mode라면 Error condition에서 Recover하기위해서 TWSTO bit에 1을 한다.
- Bit5: Master가 되고자할 때 bit에 1을 Write한다.
  Bus가 사용가능한 상태인지 Check -> if free 상태 -> START 신호 생성
- Bit6: Aknowledge pulse 발생을 제어
  if TWEA bit = 1 -> ACK pulse가 발생함
- Bit7: TWI가 현재 작업을 끝내고 제어프로그램의 응답을 기대할 때 하드웨어에 의해 set된다.
![alt text](image-5.png)
### TWSR)

- Bits 7:3: TWI 로직과 I2C Bus의 상태를 반영한다.
- Bit2: Rserved Bit
- Bit 1:0 : Read/Write가 가능하고 Bit Rate Prescaler를 제어한다.
![alt text](image-6.png)
### TWDR)

Bit7:0 : I2C Bus에서 다음에 전송할 Data 또는 최근에 수신한 Data를 저장
Transmit mode -> 다음 전송할 Data 저장
Receive mode -> 최근에 수신한 Data 저장
![alt text](image-7.png)
### TWAR)

Bit7:1 : Slave device의 Slave adress를 저장
Master Mode에서는 사용 X
Bit0: Slave Mode에서 set된다면 general call에 응답함
![alt text](image-8.png)
## 5. Atmega128에서의 활용

### 사용법)

- SCL = PD0, SDA = PD1 핀에 연결
- 무조건 풀업 저항을 달아주어야 신호가 정상작동
- TWBR과 TWSR을 이용해 SCL 클럭 주파수 설정

### 정의)

Atmega128에서 SDA와 SCL은 통신을 위한 두 개의 핵심 신호선으로
각각 Serial Clock, Serial Data를 의미함.

### 원리)

SDA: SCL=Low -> 상태 변경 / SCL = HIGH -> 변화X
