자료 출처 
https://velog.io/@skullant16/ATmega128UART-%ED%86%B5%EC%8B%A0

코드 구조 파악에 사용

2. while (1)
	{
		char recvData = Uart_Getch();
		if(UART_transmit_string(char *str) == 'a')
		{
			PORTA = 0X07;
			_delay_ms(100);
		}
	}

  --

  처음엔 이렇게 썼는데 키보드 입력 값을 어떻게 비교해야할지 몰라 AI에게 힌트 달라고함.
