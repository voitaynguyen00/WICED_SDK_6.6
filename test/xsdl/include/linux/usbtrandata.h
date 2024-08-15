#ifndef USBTRANDATA_H_DEFINED
#define USBTRANDATA_H_DEFINED

#include <termios.h>
#include "trancommon.h"

#define FALSE   0
#define TRUE    1

#define         MAX_RECEIVED_QUEUE_SIZE     0x10000
#define         MAX_TRANSMIT_QUEUE_SIZE     0x10000
#define         MAX_READ_BUF_SIZE           0x1000


typedef enum USBTransportStatus
{
    USBTransportStatusOk = 1,
    USBTransportStatusReceiveOverflow,
    USBTransportStatusTransmitOverflow
} USB_TRANSPORT_STATUS;



class CUSBTransportData
{
public:
	CUSBTransportData();
	virtual ~CUSBTransportData();

    USB_TRANSPORT_STATUS    GetStatus ();

    const char *        GetStatusError ()
    {
        return m_sTransportErr;
    }

	unsigned    long    	CountBytesReceivedFromUSB ();
	unsigned    long    	CountReadReceivedBytes ();
	unsigned    long    	CountBytesNotYetRead ();
	USB_TRANSPORT_STATUS    ReadReceivedBytes (UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesReturned);
	USB_TRANSPORT_STATUS    TransferReceivedBytesFromUSB (UINT8_t * pUARTRxDataBuf, UINT32_t nNumBytes);
/*
	unsigned    long    CountWrittenBytes ();
	unsigned    long    CountBytesTransmittedToUART ();
	unsigned    long    CountBytesNotYetTransmitted ();
	TRANSPORT_STATUS    WriteBytesForTransmission (UINT8_t * pSrcBuf, UINT32_t nNumBytes);
	TRANSPORT_STATUS    TransferBytesToBeSentToUART (UINT8_t * pUARTTxDataBuf, UINT32_t nTxBufSize, UINT32_t * nNumBytesTransferred);
*/

	void	OutputDebugString (const char* sText);

	INT32_t            m_nFileDescriptor;
	struct termios     m_oOldTIO;      //place for old and new port settings for serial port

	FUNC_PROCESS_RECEIVED_BYTES    m_pFuncRxBytes;
	void*                          m_pDataForFunc;

protected:

    USB_TRANSPORT_STATUS    m_nStatus;
    char                m_sTransportErr [256];

    unsigned    long    m_nNumBytesReceivedFromUSB;
    unsigned    long    m_nNumBytesRead;

    //unsigned    long    m_nNumBytesWritten;
    //unsigned    long    m_nNumBytesSentToUART;

    UINT8_t             m_arReceivedQueue [MAX_RECEIVED_QUEUE_SIZE];
    //UINT8_t             m_arTransmitQueue [MAX_TRANSMIT_QUEUE_SIZE];
};



#endif /* USBTRANDATA_H_DEFINED */
