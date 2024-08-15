
// ClientControl.h : main header file for the PROJECT_NAME application
//

#pragma once

#include "ControlComm.h"

#ifndef __AFXWIN_H__
    #error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"        // main symbols


// CClientControlApp:
// See ClientControl.cpp for the implementation of this class
//

class CClientControlApp : public CWinApp
{
public:
    CClientControlApp( );

// Overrides
public:
    virtual BOOL InitInstance( );

// Implementation

    DECLARE_MESSAGE_MAP( )
};

void WriteLog( const char * format ... );

extern CClientControlApp theApp;

extern ComHelper *m_ComHelper;
extern int ComPort;
extern int as32BaudRate[4];
extern int aComPorts[];
extern int ComPortSelected;
extern int BaudRateSelected;
BYTE ProcNibble(char n);
int FindBaudRateIndex(int baud);
DWORD GetHexValue(char *szbuf, LPBYTE buf, DWORD buf_size);
BYTE GetNumElements(BYTE *p_data, USHORT len);
void SendGetCompositionData(USHORT dst, BYTE page);
void SendAddAppKey(USHORT dst, USHORT net_key_idx, USHORT app_key_idx, BYTE *p_key);
void SendBind(USHORT dst, USHORT element_addr, ULONG model_id, USHORT app_key_idx);
void SendSetDeviceKey(USHORT dst);
int  GetModelElementIdx(USHORT company_id, USHORT model_id, BYTE *p_composition_data, USHORT composition_data_len);
#define COMPOSITION_DATA_TYPE_HSL       1
#define COMPOSITION_DATA_TYPE_CTL       2
#define COMPOSITION_DATA_TYPE_XYL       3
#define COMPOSITION_DATA_TYPE_DIMMABLE  4
#define COMPOSITION_DATA_TYPE_ONOFF     5
BOOL CheckCompositionData(BYTE type, BYTE *p_composition_data, USHORT len);
