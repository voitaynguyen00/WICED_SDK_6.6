#ifndef UART_TX_THREAD_H_INCLUDED
#define UART_TX_THREAD_H_INCLUDED

#include "basetype.h"
#include "osobjects.h"

#define MAX_UART_WRITE_SIZE         0x1000

UINT32_t UARTWriteThread(CExecutionThread* pExecThread);

#endif /* UART_TX_THREAD_H_INCLUDED */
