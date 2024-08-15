#ifndef OSCOMMON_H_DEFINED


#define OSCOMMON_H_DEFINED



#include "basetype.h"





#define     INFINITE_SYNCHRONIZATION_WAIT   0x7FFFFFFF



class CSynchronizationMutex
{
public:


    CSynchronizationMutex ()
    {

    }

    virtual ~CSynchronizationMutex ()
    {
    }


    virtual     bool    WaitToLock (UINT32_t nMaxWaitMs = INFINITE_SYNCHRONIZATION_WAIT) = 0;
    virtual     void    Unlock () = 0;


};


class CSynchronizationEvent
{
public:


    CSynchronizationEvent ()
    {

    }

    virtual ~CSynchronizationEvent ()
    {
    }


    virtual     bool    WaitForSignal (UINT32_t nMaxWaitMs = INFINITE_SYNCHRONIZATION_WAIT) = 0;
    virtual     void    SetEvent () = 0;


};






#define     THREAD_STARTED_SUCCESSFULLY     0
#define     THREAD_START_FAILURE            0xFFFFFFFF
#define     THREAD_FUNCTION_NULL            0xFFFFFFFE

#define     THREAD_TERMINATED_SUCCESSFULLY  0
#define     THREAD_HANDLE_NULL              0xFFFFFFFE



typedef enum ThreadStatus
{
	THREADSTATUS_UNINITIALIZED = 0,
	THREADSTATUS_INITIALIZED,
	THREADSTATUS_FAILEDTOSTART,
	THREADSTATUS_EXECUTING,
    THREADSTATUS_REQUEST_TO_TERMINATE,
	THREADSTATUS_COMPLETED
} THREADSTATUS;






class CExecutionThread;



typedef     UINT32_t    (*FUNC_THREAD_EXECUTION)(CExecutionThread * pThreadData);


class CExecutionThread
{

public:


    CExecutionThread (FUNC_THREAD_EXECUTION pThreadFunc, void * pData = NULL)
    {

        m_pThreadFunc = pThreadFunc;
        m_pThreadData = pData;
        m_sDescBuf [0] = '\0';
        m_pMutex = NULL;
        m_dwThreadResult = 0;
        m_nStatus = pThreadFunc ? THREADSTATUS_INITIALIZED : THREADSTATUS_UNINITIALIZED;
    }

    virtual ~CExecutionThread ();


    const char* Description ()
    {
        return m_sDescBuf;
    }


    void    UseMutex (CSynchronizationMutex* pMutex)
    {
        m_pMutex = pMutex;
    }





    virtual UINT32_t    TerminateThread () = 0;


    virtual UINT32_t    RunThread () = 0;



    FUNC_THREAD_EXECUTION       m_pThreadFunc;
    void *                      m_pThreadData;

    char                        m_sDescBuf [64];

    THREADSTATUS                m_nStatus;

    CSynchronizationMutex*      m_pMutex;

    UINT32_t                    m_dwThreadResult;

};






void        PutThreadToSleep (UINT32_t nMilliSeconds);

void        GetCurrentOSTime (DATETIMEHOLDER* oTime);

UINT32_t    GetCurrentOSTimeMS ();

bool        GetMACAddress (UINT8_t * pSixByteBuffer);


#endif

