#ifndef TRANCOMMON_H_DEFINED


#define TRANCOMMON_H_DEFINED



#include "highertypes.h"

#include "circque.h"


typedef enum PortTypes
{
    PortTypeUnknown = 0,
    PortTypeUART,
    PortTypeUSB
} PORTTYPES;



typedef     UINT32_t    (*FUNC_PROCESS_RECEIVED_BYTES) (void * pSomeDataForFunc, UINT8_t * pByteBuffer, UINT32_t nNumBytes);


class CCommunicationParameters  
{
public:
	CCommunicationParameters();
	virtual ~CCommunicationParameters();



	CCommunicationParameters(const CCommunicationParameters& cp);
	CCommunicationParameters& operator= (const CCommunicationParameters& cp);



    string              ParseCommParams (const char * sCommParams);

    string              GetPortId ();
    // Textual native port id, e.g. COM1 or USB0 on Windows, or /dev/ttyS0 on Linux

    PORTTYPES           GetPortType ();

    string              GetPortTypeName ();


    string              GetParamAsText (const char * sParamName, const char * sDefaultIfMissing);

    UINT32_t            GetParamAsU32 (const char * sParamName, UINT32_t nDefaultIfMissing);


    const char *        GetWholeConnectionString ()
    {
        return m_sConnectionString.c_str();
    }


    CNameValueTable     GetAllParams ()
    {
        return m_oParams;
    }

protected:


    string              m_sConnectionString;

    CNameValueTable     m_oParams;




};




#define         MAX_RECEIVED_QUEUE_SIZE     0x10000
#define         MAX_TRANSMIT_QUEUE_SIZE     0x10000



#define         MAX_READ_BUF_SIZE           0x1000




typedef enum TransportStatus
{
    TransportStatusOk = 1,
    TransportStatusReceiveOverflow,
    TransportStatusTransmitOverflow
} TRANSPORT_STATUS;





// This platform-independent base class will take care of internal Tx and Rx data buffering,
// keeping track of status, and callback function pointers.
// The buffer sizes for every specific case can be controlled via constructor parameters.

class CTransportData  
{
public:





	CTransportData (UINT32_t nMaxRxQueueSize = MAX_RECEIVED_QUEUE_SIZE, 
                    UINT32_t nMaxTxQueueSize = MAX_TRANSMIT_QUEUE_SIZE);

    virtual ~CTransportData();





    TRANSPORT_STATUS    GetStatus ();

    const char *        GetStatusError ()
    {
        return m_sTransportErr;
    }

    UINT32_t CountTotalBytesReceivedFromDevice ()
    {
        return m_nNumBytesReceivedFromDevice;
    }



    UINT32_t    CountReceivedBytesAlreadyProcessed ()
    {
        return m_nNumBytesReadByCaller;
    }


    UINT32_t    CountReceivedBytesNotYetProcessed ()
    {
        return m_nNumBytesReceivedFromDevice - m_nNumBytesReadByCaller;
    }




    TRANSPORT_STATUS    TransferBytesJustReceivedToLocalBuffer (UINT8_t * pRxDataBuf, UINT32_t nNumBytes);
    TRANSPORT_STATUS    FetchReceivedBytesFromLocalBuffer (UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesReturned, bool bPeekOnly = false);


    TRANSPORT_STATUS    StoreBytesForTransmission (UINT8_t * pSrcBuf, UINT32_t nNumBytes);
    TRANSPORT_STATUS    TransmitLocallyQueuedBytesToTxBuffer (UINT8_t * pTxDataBuf, UINT32_t nTxBufSize, UINT32_t * nNumBytesTransferred);


    UINT32_t    CountWrittenBytes ()
    {
        return m_nNumBytesWrittenByCaller;
    }



    UINT32_t    CountBytesTransmittedToDevice ()
    {
        return m_nNumBytesSentToDevice;
    }



    UINT32_t    CountBytesNotYetTransmitted ()
    {
        return m_nNumBytesWrittenByCaller - m_nNumBytesSentToDevice;
    }

    

    void        OutputDebugString (const char * sText)
    {
        // Not implemented in the base class
    }

    void        SetCommParams (CCommunicationParameters* pCommParams)
    {
        m_pCommParams = pCommParams;
    }

    CCommunicationParameters* GetCommParams ()
    {
        return m_pCommParams;
    }

public:


    FUNC_PROCESS_RECEIVED_BYTES m_pFuncRxBytes;
    void *                      m_pDataForFunc;


protected:

    TRANSPORT_STATUS    m_nStatus;
    char                m_sTransportErr [256];


    UINT32_t            m_nNumBytesReceivedFromDevice;
    UINT32_t            m_nNumBytesReadByCaller;

    UINT32_t            m_nNumBytesWrittenByCaller;
    UINT32_t            m_nNumBytesSentToDevice;

    UINT8_t*            m_pReceivedQueue;
    UINT8_t*            m_pTransmitQueue;

    UINT32_t            m_nMaxRxQueueSize;
    UINT32_t            m_nMaxTxQueueSize;

    CCommunicationParameters*   m_pCommParams;

};













class CSerialConnector
{
public:
	CSerialConnector()
    {
        m_pCommParams = NULL;
        m_pTxPrefix = NULL;
    }

	virtual ~CSerialConnector()
    {
        delete m_pCommParams;
        delete m_pTxPrefix;
    }



    virtual string      Connect (CCommunicationParameters& refParams, FUNC_PROCESS_RECEIVED_BYTES pFunc, void * pSomeDataForFunc) = 0;
    virtual string      Disconnect() = 0;
    virtual string      TransmitBytes (UINT8_t * pByteBuffer, UINT32_t nNumBytes) = 0;
    virtual string      ReadReceivedBytes (UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesReturned, bool bPeekOnly = false) = 0;

    virtual string      Reconnect (const char * sModifiedConnectionParams = NULL)
    {
        return "NOT IMPLEMENTED (may not be allowed for some transport/OS combination)";
    }

    virtual bool        WaitForTransmissionToComplete (UINT32_t dwMaxTimeMS)
    {
        // Can be implemented separately for different connector classes to have an ability to tell when transmission of the 
        // current byte sequence has completed
        return true;
    }



    bool        IsConnected ()
    {
        return (m_pCommParams != NULL);
    }

    CCommunicationParameters*   GetConnectionParameters ()
    {
        return m_pCommParams;
    }

    CByteBuffer* GetTxPrefix ()
    {
        return m_pTxPrefix;
    }

    void ClearTxPrefix ()
    {
        delete m_pTxPrefix;
        m_pTxPrefix = NULL;
    }

    void SetTxPrefix (CByteBuffer& refPrefix)
    {
        delete m_pTxPrefix;
        if (refPrefix.GetDataByteSize () > 0)
            m_pTxPrefix = new CByteBuffer (refPrefix);
        else
            m_pTxPrefix = NULL;
    }


protected:

    CCommunicationParameters*   m_pCommParams;

    CByteBuffer*                m_pTxPrefix;
};


class CDummyConnector : public CSerialConnector
{
public:
	CDummyConnector(const char * sName)
    {
        m_sName = (sName ? sName : "NULL");
    }

	~CDummyConnector()
    {
    }



    string      Connect (CCommunicationParameters& refParams, FUNC_PROCESS_RECEIVED_BYTES pFunc, void * pSomeDataForFunc)
    {
        return "Connect not implemented for class " + m_sName;
    }

    string      Disconnect()
    {
        return "Disconnect not implemented for class " + m_sName;
    }

    string      TransmitBytes (UINT8_t * pByteBuffer, UINT32_t nNumBytes)
    {
        return "TransmitBytes not implemented for class " + m_sName;
    }

    string      ReadReceivedBytes (UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesReturned)
    {
        return "ReadReceivedBytes not implemented for class " + m_sName;
    }


    string      Reconnect (const char * sModifiedConnectionParams)
    {
        return "Reconnect not implemented for class " + m_sName;
    }



protected:

    string          m_sName;

};


enum CustomPDUState
{	
    PDUStateUndefined = 0,
	PDUStateTxHostToDevice,
	PDUStateTxCompleted,
	PDUStateTxSkipped,
	PDUStateRxHostFromDevice,
	PDUStateRxCompleted,
	PDUStateRxSkipped,
	PDUStateError
};

class CPDUProcessor
{
public:

    CPDUProcessor ()
    {
        m_nState = PDUStateUndefined;
    }

    virtual ~CPDUProcessor ()
    {
        
    }

	CustomPDUState  GetState ()
    {
        return m_nState;
    }
	
    
    string GetErrorText ()
    {
        return m_sError;
    }


	virtual string TransmitBytesAndReceiveResponse (CSerialConnector * pConnector, 
                                                    UINT8_t * pTxBytes, UINT32_t nNumTxBytes,
                                                    UINT8_t * pRxBytes, UINT32_t nRxBufSize, UINT32_t * nNumRxBytes) = 0;


protected:


	enum CustomPDUState m_nState;

    string              m_sError;
};





#define     MAX_CHUNKED_RX_MSG_SIZE     0x2000

class CChunkingConnector : public CSerialConnector  
{
public:
	CChunkingConnector()
    {
        m_pConnector = NULL;
        m_pChunker = NULL;
        m_nRxMsgSize = 0;

    }
	virtual ~CChunkingConnector();

    string      LinkToPhysicalConnector (CSerialConnector * pOpenConnector, CPDUProcessor * pChunker)
    {
        if (!pOpenConnector)
            return "Need to provide a valid physical CSerialConnector instance!";
        if (!pChunker)
            return "Need to provide a valid CPDUProcessor instance!";

        m_pConnector = pOpenConnector;
        m_pChunker = pChunker;

        return "";
    }



    string      Connect (CCommunicationParameters& refParams, FUNC_PROCESS_RECEIVED_BYTES pFunc, void * pSomeDataForFunc)
    {
        if (!m_pConnector)
            return "NULL Connector!";

        return "";
    
    }



    string      Disconnect()
    {
        if (!m_pConnector)
            return "NULL Connector!";

        return m_pConnector->Disconnect ();
    }



    string      TransmitBytes (UINT8_t * pByteBuffer, UINT32_t nNumBytes)
    {
        string      s;

        if (!m_pConnector)
            return "NULL Connector!";

        m_nRxMsgSize = 0;

	    return m_pChunker->TransmitBytesAndReceiveResponse (m_pConnector, pByteBuffer, nNumBytes,                                                    
                                                            m_arRxMsg, sizeof(m_arRxMsg), &m_nRxMsgSize);

    }




    string      ReadReceivedBytes (UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesReturned)
    {
        if (!m_pConnector)
            return "NULL Connector!";
        if (nBufSize < m_nRxMsgSize)
            return "Rx data size overflow";

        *nNumBytesReturned = m_nRxMsgSize;

        if (m_nRxMsgSize > 0)
        {
            memcpy (pDestBuf, m_arRxMsg, m_nRxMsgSize);
            m_nRxMsgSize = 0;
        }

        return "";
    }




    static 
    CPDUProcessor * CreatePDUChunker (const char * sPDUClass)
    {
        if (!sPDUClass)
            return NULL;

        STRING_TO_UPPER ((char *) sPDUClass);
        if (!strcmp (sPDUClass, "SMBUS"))
        {
            return NULL;//new CSMBusChunker;
        }
        else
            return NULL;
    }



protected:

    CSerialConnector *  m_pConnector;
    CPDUProcessor *     m_pChunker;



    UINT8_t             m_arRxMsg [MAX_CHUNKED_RX_MSG_SIZE];
    UINT32_t            m_nRxMsgSize;
};







// Must be implemented inside the platform-specific module.
// Returns NULL if cannot determine the class from the connection string.
extern  CSerialConnector*   CreateSerialConnector (const char * sConnectionString, string * sOutErr, FUNC_PROCESS_RECEIVED_BYTES pFunc = NULL, void * pSomeDataForFunc = NULL);

extern  bool                IdentifyPortType (const char * sPort, PORTTYPES * nOutType, string * sOutType);


// The following platform-specific functions allow to generate port ids in a loop.
extern  INT32_t             GetMinPortIndex (PORTTYPES nPortType);  // Or -1 if N/A
extern  INT32_t             GetMaxPortIndex (PORTTYPES nPortType);  // Or -1 if N/A
extern  bool                CreatePortId (PORTTYPES nPortType, int nPortIndex, string * sOutPortId, const char * sPortRootName = NULL);

extern  INT32_t             FetchAvailablePortIds (PORTTYPES nPortType, vector<string> * vecOutListOfPorts);





class CCommunicatorWithRxQueue
{

public:

    CCommunicatorWithRxQueue (int nRxQueueSize = 0x10000);

    ~CCommunicatorWithRxQueue ();

    string      Connect (CSerialConnector * pConnector, CCommunicationParameters& refParams);
    string      Disconnect ();



    CppQueueWrapper     GetRxQueue ()
    {
        return (CppQueueWrapper (*m_pRxQueue));
    }



protected:

    CSerialConnector *  m_pConnector;

    DATA_QUEUE*         m_pRxQueue;
};









#endif

