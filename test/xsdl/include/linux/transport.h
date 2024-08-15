#ifndef TRANSPORT_H_DEFINED
#define TRANSPORT_H_DEFINED

#include "basetype.h"
#include "osobjects.h"
#include "trancommon.h"
#include "uarttrandata.h"
#include "usbtrandata.h"

class CUARTConnector : public CSerialConnector
{
public:

	CUARTConnector();
	virtual ~CUARTConnector();



    string      Connect (CCommunicationParameters& refParams, FUNC_PROCESS_RECEIVED_BYTES pFunc, void * pSomeDataForFunc);
    string      Disconnect();
    string      TransmitBytes (UINT8_t * pByteBuffer, UINT32_t nNumBytes);
    string      ReadReceivedBytes (UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesReturned);
    string      Reconnect (const char * sModifiedConnectionParams);


    CUARTTransportData* GetData ()
    {
        return m_pData;
    }

protected:
/*
    HANDLE              m_hThreadCommReader;
    HANDLE              m_hThreadCommWriter;
    DWORD               m_dwThreadIdCommReader;
    DWORD               m_dwThreadIdCommWriter;
    HANDLE              m_hComPortFile;

    DWORD               m_dwMaxReadBufferSize;
    DWORD               m_dwMaxWriteBufferSize;
*/

    INT32_t	            m_nFileDescriptor;


    CUARTTransportData*    m_pData;

    UINT8_t             m_arReadingBuf [MAX_READ_BUF_SIZE];

    CExecutionThread*	m_pReadThread;
    CExecutionThread*   m_pWriteThread;

    bool                CreateConnectionViaFile (CCommunicationParameters& refParams);
    void                CleanUp ();

    bool                StartWorkerThreads();
    void				CloseWorkerThreads();
//    DWORD               WaitForThreadsToTerminate (DWORD dwTimeout);

//    void                FillTimeOutStructure (CCommunicationParameters& refParams, COMMTIMEOUTS * pDestTimeouts);


};




class CUSBConnector : public CSerialConnector
{
public:

	CUSBConnector();
	virtual ~CUSBConnector();



    string      Connect (CCommunicationParameters& refParams, FUNC_PROCESS_RECEIVED_BYTES pFunc, void * pSomeDataForFunc);
    string      Disconnect();
    string      TransmitBytes (UINT8_t * pByteBuffer, UINT32_t nNumBytes);
    string      ReadReceivedBytes (UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesReturned);


    CUSBTransportData* GetData ()
    {
        return m_pData;
    }

protected:





    INT32_t	            m_nFileDescriptor;


    CUSBTransportData*    m_pData;

    UINT8_t             m_arReadingBuf [MAX_READ_BUF_SIZE];

    CExecutionThread*	m_pReadThread;
    //CExecutionThread*   m_pWriteThread;

    bool                CreateConnectionViaFile (CCommunicationParameters& refParams);
    void                CleanUp ();

    bool                StartWorkerThreads();
    void		CloseWorkerThreads();

};
#endif	/* TRANSPORT_H_DEFINED */


