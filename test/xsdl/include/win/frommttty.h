
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <windows.h>



/*-----------------------------------------------------------------------------
    This is a part of the Microsoft Source Code Samples. 
    Copyright (C) 1995 Microsoft Corporation.
    All rights reserved. 
    This source code is only intended as a supplement to 
    Microsoft Development Tools and/or WinHelp documentation.
    See these sources for detailed information regarding the 
    Microsoft samples programs.

    MODULE: MTTTY.h

    PURPOSE: Contains global definitions and variables

-----------------------------------------------------------------------------*/

#ifndef FROMMTTY_H_INCLUDED

#define MAX_STATUS_BUFFER       20000
#define MAX_WRITE_BUFFER        1024
#define MAX_READ_BUFFER         2048
#define READ_TIMEOUT            500
#define STATUS_CHECK_TIMEOUT    500
#define WRITE_CHECK_TIMEOUT     500
#define PURGE_FLAGS             PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR 
#define EVENTFLAGS_DEFAULT      EV_BREAK | EV_CTS | EV_DSR | EV_ERR | EV_RING | EV_RLSD
#define FLAGCHAR_DEFAULT        '\n'

//
// Write request types
//
#define WRITE_CHAR          0x01
#define WRITE_FILE          0x02
#define WRITE_FILESTART     0x03
#define WRITE_FILEEND       0x04
#define WRITE_ABORT         0x05
#define WRITE_BLOCK         0x06

//
// Read states
//
#define RECEIVE_TTY         0x01
#define RECEIVE_CAPTURED    0x02

//
//  Initialization/deinitialization/settings functions
//
HANDLE SetupCommPort( void );
void ChangeConnection( HWND, BOOL );
BOOL BreakDownCommPort( void );
BOOL UpdateConnection( void );
void GlobalInitialize( void );
void DestroyTTYInfo( void );
void GlobalCleanup( void );
void UpdateTTYInfo( void );
BOOL DisconnectOK( void );
BOOL InitTTYInfo( void );
void InitNewFont( LOGFONT, COLORREF );

//
//  Status functions
//

class CUARTTransportData ;

void CheckModemStatus( CUARTTransportData * pData );
void CheckComStat(CUARTTransportData * pData);

#endif