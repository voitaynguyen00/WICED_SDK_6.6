#ifndef OSOBJECTS_H_DEFINED

#define OSOBJECTS_H_DEFINED


#include "oscommon.h"

#include "windows.h"
#include "winbase.h"



class COSExecutionThread : public CExecutionThread
{

public:


    COSExecutionThread (FUNC_THREAD_EXECUTION pThreadFunc, void * pData = NULL) :
        CExecutionThread (pThreadFunc, pData)
    {
        m_hThread = 0;
        m_dwThreadId = 0;
        m_dwThreadResult = 0;
    }

    ~COSExecutionThread ()
    {
        if (m_hThread)
            ::CloseHandle (m_hThread);
    }


    virtual UINT32_t    RunThread ();
    virtual UINT32_t    TerminateThread ();


    HANDLE              m_hThread;
    DWORD               m_dwThreadId;

};




class COSMutex : public CSynchronizationMutex
{
public:

    COSMutex (); 

    ~COSMutex ();



    bool        WaitToLock (UINT32_t nMaxWaitMs = INFINITE_SYNCHRONIZATION_WAIT);
    void        Unlock ();


    HANDLE      m_hMutex;

};


class COSEvent : public CSynchronizationEvent
{
public:

	COSEvent();

	~COSEvent();

    bool    WaitForSignal(UINT32_t nMaxWaitMs = INFINITE_SYNCHRONIZATION_WAIT);
    void    SetEvent();

	HANDLE      m_hEvent;
};
#endif


