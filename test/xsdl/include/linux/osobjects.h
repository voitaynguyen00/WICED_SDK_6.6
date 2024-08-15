#ifndef OSOBJECTS_H_DEFINED
#define OSOBJECTS_H_DEFINED

#include "oscommon.h"

#include <pthread.h>




class COSExecutionThread : public CExecutionThread
{

public:


    COSExecutionThread (FUNC_THREAD_EXECUTION pThreadFunc, void * pData = NULL) :
        CExecutionThread (pThreadFunc, pData)
    {
        m_pthread = 0;
    }

    virtual ~COSExecutionThread ()
    {
    }



    virtual UINT32_t    RunThread ();
    virtual UINT32_t    TerminateThread();



    pthread_t           m_pthread;

    int                 m_nCreateRetVal;
};




class COSMutex : public CSynchronizationMutex
{
public:

    COSMutex (); 

    virtual ~COSMutex ();



    bool        WaitToLock (UINT32_t nMaxWaitMs = INFINITE_SYNCHRONIZATION_WAIT);
    void        Unlock ();


    pthread_mutex_t     m_pthread_mutex;
};

class COSEvent : public CSynchronizationEvent
{
public:

	COSEvent();

	virtual ~COSEvent();

	bool    WaitForSignal(UINT32_t nMaxWaitMs = INFINITE_SYNCHRONIZATION_WAIT);
	void    SetEvent();

	pthread_cond_t	m_hEvent;
	pthread_mutex_t	m_hMutex;
	bool			m_bSignalled;
};
#endif /* OSOBJECTS_H_DEFINED */


