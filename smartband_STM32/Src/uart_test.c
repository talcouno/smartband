

#include "uart_test.h"

extern int flag;
extern int gyro_onoff;
extern int hr_flag;
extern uint32_t tickstart1;
extern int operation;
extern int fflag;

char  tx_test[100];
char ch, ch2, rx_Test[100];
char rx_buffer[100];
int ind = 0;
char tmp[100];
int dflag;

void UART6_Test(void)
{
#if 1
	while(CRZ_FALSE == UartQueue_Serial_Is_Empty())
	{
		ch = UartQueue_Serial_DeQueue();

		if(ch == '['){
			dflag = 1;
		}
		if(dflag && ch != 91 && ch != 93){
			operation = ch - '0';
			send_operation(operation);
		}
		if(ch == ']'){
			dflag = 0;
		}
	}
#endif



#if 0
		while(CRZ_FALSE == Uart_6_Queue_Comm_Is_Empty())
		{
			ch = Uart_6_Queue_Comm_DeQueue();
			if(ch == '[')
			{
				HAL_Delay(2000);
				ch = Uart_6_Queue_Comm_DeQueue();
				sprintf(rx_Test, "tmp:  %c", ch);
				HAL_UART_Transmit(&huart1, (uint8_t *)rx_Test, strlen(rx_Test), 0xFFFF);

			}
		}
#endif


}

void send_operation(int op)
{
	switch(op) {
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		flag = 2;
		sprintf(tx_test, "[%d,%d,0\n", flag, op);
		HAL_UART_Transmit(&huart6, (uint8_t *)tx_test, strlen(tx_test), 0xFFFF);
		break;
	case 6:
		gyro_onoff = 1;
		flag = 1;
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
		break;
	case 7:
		gyro_onoff = 0;
		flag = 3;
		fflag = 1;
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
		sprintf(tx_test, "[%d,0,0\n",flag);
		HAL_UART_Transmit(&huart6, (uint8_t *)tx_test, strlen(tx_test), 0xFFFF);
		break;
	case 8:
		printf("wait 10 sec...\n");
		hr_flag = 1;
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
		tickstart1 = HAL_GetTick();
		break;
	default:
		printf("operation error\n");
		break;
	}

	operation = 0;

}
