

#ifndef UART_QUEUE_H_
#define UART_QUEUE_H_
#ifndef __cplusplus
typedef enum {CRZ_FALSE = 0, CRZ_TRUE = !CRZ_FALSE} CRZ_bool;
#endif
#include "usart.h"

#define  UART_QUEUE_BUFFER_SIZE  1000
#define  EVENT_QUEUE_BUFFER_SIZE  200

void print_que();

// UART Queue for Serial

void UartQueue_Serial_Initialize(void);
CRZ_bool UartQueue_Serial_Is_Empty(void);
void UartQueue_Serial_EnQueue(uint16_t data);
uint16_t UartQueue_Serial_DeQueue(void);

// UART 6 Queue for Comm

void Uart_6_Queue_Comm_Initialize(void);
CRZ_bool Uart_6_Queue_Comm_Is_Empty(void);
void Uart_6_Queue_Comm_EnQueue(uint16_t data);
uint16_t Uart_6_Queue_Comm_DeQueue(void);


#endif /* UART_QUEUE_H_ */
