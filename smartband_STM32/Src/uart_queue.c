
#include "uart_queue.h"


/* Static Functions */

__STATIC_INLINE void uart_queue_increase_point_value(uint32_t * data_p)
{
    (* data_p) ++;
    if(UART_QUEUE_BUFFER_SIZE == (* data_p))
    {
        (* data_p) = 0;
    }
}

__STATIC_INLINE void event_queue_increase_point_value(uint32_t * data_p)
{
    (* data_p) ++;
    if(EVENT_QUEUE_BUFFER_SIZE == (* data_p))
    {
        (* data_p) = 0;
    }
}

/* Global Functions */



// UART Queue for Serial

uint16_t  uart_queue_serial_buffer[UART_QUEUE_BUFFER_SIZE];
uint32_t  uart_queue_serial_point_head = 0;
uint32_t  uart_queue_serial_point_tail = 0;

void UartQueue_Serial_Initialize(void)
{
    uart_queue_serial_point_head = uart_queue_serial_point_tail = 0;
}

CRZ_bool UartQueue_Serial_Is_Empty(void)
{
    if(uart_queue_serial_point_head == uart_queue_serial_point_tail)
    {
        return CRZ_TRUE;
    }
    return CRZ_FALSE;
}

void UartQueue_Serial_EnQueue(uint16_t data)
{
    uart_queue_serial_buffer[uart_queue_serial_point_head] = data;
    uart_queue_increase_point_value(&uart_queue_serial_point_head);
}
void print_que()
{
	int i;
	for(i = uart_queue_serial_point_tail; i < uart_queue_serial_point_head; i++)
	{
		printf("%d ",uart_queue_serial_buffer[i]);
	}
	printf("\n");
	return;
}

uint16_t UartQueue_Serial_DeQueue(void)
{
    uint16_t retVal = uart_queue_serial_buffer[uart_queue_serial_point_tail];
    uart_queue_increase_point_value(&uart_queue_serial_point_tail);
    return retVal;
}

// UART 6 Queue for Comm

uint16_t  uart_6_queue_comm_buffer[UART_QUEUE_BUFFER_SIZE];
uint32_t  uart_6_queue_comm_point_head = 0;
uint32_t  uart_6_queue_comm_point_tail = 0;

void Uart_6_Queue_Comm_Initialize(void)
{
    uart_6_queue_comm_point_head = uart_6_queue_comm_point_tail = 0;
}

CRZ_bool Uart_6_Queue_Comm_Is_Empty(void)
{
    if(uart_6_queue_comm_point_head == uart_6_queue_comm_point_tail)
    {
        return CRZ_TRUE;
    }
    return CRZ_FALSE;
}

void Uart_6_Queue_Comm_EnQueue(uint16_t data)
{
    uart_6_queue_comm_buffer[uart_6_queue_comm_point_head] = data;
    uart_queue_increase_point_value(&uart_6_queue_comm_point_head);
}

uint16_t Uart_6_Queue_Comm_DeQueue(void)
{
    uint16_t retVal = uart_6_queue_comm_buffer[uart_6_queue_comm_point_tail];
    uart_queue_increase_point_value(&uart_6_queue_comm_point_tail);
    return retVal;
}


