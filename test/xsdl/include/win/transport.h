#ifndef TRANSPORT_H_DEFINED
#define TRANSPORT_H_DEFINED


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





    HANDLE              m_hThreadCommReader;
    HANDLE              m_hThreadCommWriter;
    DWORD               m_dwThreadIdCommReader;
    DWORD               m_dwThreadIdCommWriter;
    HANDLE              m_hComPortFile;

    DWORD               m_dwMaxReadBufferSize;
    DWORD               m_dwMaxWriteBufferSize;



    CUARTTransportData *    m_pData;


    COMMTIMEOUTS        m_structOrigTimeOuts;
    COMMTIMEOUTS        m_structCurTimeOuts;


    BYTE                m_arReadingBuf [MAX_READ_BUF_SIZE];


    bool                CreateConnectionViaFile (CCommunicationParameters& refParams);
    void                CleanUp ();
    void                ClosePortFile ();
    void                CloseWorkerThreads ();





    bool                StartWorkerThreads ();
    bool                ReleaseCommPort ();
    DWORD               WaitForThreadsToTerminate (DWORD dwTimeout);

    void                FillTimeOutStructure (CCommunicationParameters& refParams, COMMTIMEOUTS * pDestTimeouts);


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


    string      Reconnect (const char * sModifiedConnectionParams);


    void        SetActiveWriteEndpoint (DWORD ep);

protected:

    HANDLE              m_hThreadUSBEventReader;
    HANDLE              m_hThreadUSBDataReader;
    DWORD               m_dwThreadIdUSBEventReader;
    DWORD               m_dwThreadIdUSBDataReader;
    HANDLE              m_hUSBPortFile;


    DWORD               m_dwActiveWriteEndpoint;

    CUSBTransportData*  m_pUSBData;

    bool                CreateUSBConnectionViaFile (CCommunicationParameters& refParams);
    bool                SetUSBConnectionExclusive ();
    bool                StartWorkerThreads ();

    void                CleanUp ();
    bool                CloseUSB ();
    void                ClosePortFile ();    
    void                CloseWorkerThreads ();
    DWORD               WaitForThreadsToTerminate (DWORD dwTimeout);

};

class CDHDConnector : public CDummyConnector
{
public:
    CDHDConnector () : CDummyConnector ("CDHDConnector")
    {
    }
    
    ~CDHDConnector ()
    {
    }
};
#endif


