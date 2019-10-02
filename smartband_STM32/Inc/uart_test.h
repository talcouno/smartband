
#ifndef UART_TEST_H_
#define UART_TEST_H_

#include "usart.h"
#include "uart_queue.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


extern char  tx_test[100];
extern char ch, ch2, rx_Test[100];


void UART6_Test(void);
void send_operation(int op);

#endif /* UART_TEST_H_ */
