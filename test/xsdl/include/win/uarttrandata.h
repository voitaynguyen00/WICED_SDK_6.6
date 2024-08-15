#ifndef UARTTRANDATA_H_DEFINED

#define UARTTRANDATA_H_DEFINED


#include "trancommon.h"


#include "windows.h"
#include "winbase.h"



#include "frommttty.h"





typedef enum UartParity
{
	UartParityUnknown = 0,
	UartParityOdd,
	UartParityEven,
	UartParityMark,
	UartParitySpace

} UARTPARITY;







class CUARTTransportData : public CTransportData
{
public:
	CUARTTransportData(HANDLE hComPortFile = NULL, DWORD dwEventFlags = EVENTFLAGS_DEFAULT);
	virtual ~CUARTTransportData();


    void                SetComFileHandle (HANDLE hFile)
    {
        m_hComPortFile = hFile;
    }


    
    TRANSPORT_STATUS    WriteBytesForTransmission (UINT8_t * pSrcBuf, UINT32_t nNumBytes); 
    

    void                OutputDebugString (const char * sText);

public:

    HANDLE              m_hComPortFile;
    DWORD               m_dwEventFlags;

    HANDLE              m_hStatusMessageEvent;
    HANDLE              m_hThreadExitEvent;
    HANDLE              m_hWriterEvent;
    HANDLE              m_hTransferCompleteEvent;

    DWORD               m_dwLastEvent;


    HANDLE              m_hReceivedMoreBytesFromUARTEvent;


    // These used to be __declspec(thread) in the original sample
    COMSTAT             ComStatOld ;
    DWORD               dwErrorsOld;
    DWORD               dwOldStatus;




protected:


    FILE *              m_fDebugAndStatusFile;
    CRITICAL_SECTION    m_csDebugFile;
};


#endif