#ifndef USB_TRAN_DATA_H_INCLUDED
#define USB_TRAN_DATA_H_INCLUDED

#include "trancommon.h"
#include "windows.h"
#include "winbase.h"
#include "frommttty.h"

#define         MAX_USB_RCV_QUEUE_SIZE     0x20000





class CUSBTransportData : public CTransportData 
{
public:
	CUSBTransportData(HANDLE hUSBPortFile = NULL, DWORD dwEventFlags = EVENTFLAGS_DEFAULT);
	virtual ~CUSBTransportData();

    void SetUSBFileHandle (HANDLE hFile) //
    {
        m_hUSBPortFile = hFile;
    }



    void                OutputDebugString (const char * sText); //


public:

    HANDLE              m_hUSBPortFile;
    DWORD               m_dwEventFlags;

    HANDLE              m_hStatusMessageEvent;
    HANDLE              m_hThreadExitEvent;
    HANDLE              m_hTransferCompleteEvent;
    HANDLE              m_hReceivedMoreBytesFromUSBEvent;

    CRITICAL_SECTION    m_csReceivedQueue;


protected:





    FILE *              m_fDebugAndStatusFile;
    CRITICAL_SECTION    m_csDebugFile;

};

#endif //USB_TRAN_DATA_H_INCLUDED