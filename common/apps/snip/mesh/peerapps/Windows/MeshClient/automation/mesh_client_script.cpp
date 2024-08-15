/****************************************************************************
**
**  Name:       mesh_client_script.c
**
**  Function:   this file contains functions to send and receive L2CAP
**              commands and events.
**
**  Copyright (c) 2018, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include <Windows.h>
#include <string.h>
#include "bt_types.h"
#include "mesh_client_script.h"
#include "wiced_bt_trace.h"
#include "mesh_client.pb.h"

extern "C" void mesh_client_rpc_return_event(tMESH_CLIENT_SCRIPT_EVENT_PARAMS *p_evt);

#include<list>

extern "C" void script_start_wait_timer(UINT32 timeout);
extern "C" void script_stop_wait_timer(void);

/*******************************************************************************
**
** Function         mesh_client_enqueue_and_check_event - enqueue and check for awaited events
**
*******************************************************************************/
class CAutoLock 
{
public:
    CAutoLock(CRITICAL_SECTION& cs) :
        m_cs(cs)
    {
        EnterCriticalSection(&m_cs);
    }
    ~CAutoLock() {
        LeaveCriticalSection(&m_cs);
    }

private:
    CRITICAL_SECTION& m_cs;
};

class SyncObject {
public:
    SyncObject() :
        m_hEvent(CreateSemaphore(NULL, 0, 100, NULL))
    {
        InitializeCriticalSection(&m_cs);
    };
    virtual ~SyncObject()
    {
        if (m_hEvent != INVALID_HANDLE_VALUE)
        {
            CloseHandle(m_hEvent);
            m_hEvent = INVALID_HANDLE_VALUE;
        }
    };

    LONG pushEvent(tMESH_CLIENT_SCRIPT_EVENT_PARAMS *p)
    {
        CAutoLock lock(m_cs);
        LONG prevCount;
        evtList.push_back(p);
        ReleaseSemaphore(m_hEvent, 1, &prevCount);
        return ++prevCount;
    }
    void *getEvent()
    {
        void* ret = NULL;
        DWORD dwWaitResult = WaitForSingleObject(m_hEvent, 1000);
        if (dwWaitResult == WAIT_OBJECT_0)
        {
            CAutoLock lock(m_cs);
            if (evtList.size() > 0)
            {
                ret = (void*)evtList.front();
                evtList.pop_front();
            }
        }
        return ret;
    }
    void reset() {
        CAutoLock lock(m_cs);
        while (evtList.size() > 0)
        {
            free(evtList.front());
            evtList.pop_front();
        }
    }
private:
    HANDLE m_hEvent;
    CRITICAL_SECTION m_cs;
    std::list<tMESH_CLIENT_SCRIPT_EVENT_PARAMS*> evtList;
};

static SyncObject obj;

extern "C" void mesh_client_reset_callback_EvtList()
{
    obj.reset();
}

void mesh_client_enqueue_and_check_event(tMESH_CLIENT_SCRIPT_EVENT event, void *p_params, UINT16 size_of_params)
{

    if (p_params != NULL)
    {
        tMESH_CLIENT_SCRIPT_EVENT_PARAMS *p_evt = NULL;
        if ((p_evt = (tMESH_CLIENT_SCRIPT_EVENT_PARAMS *)malloc(sizeof(tMESH_CLIENT_SCRIPT_EVENT) + size_of_params)) == NULL)
        {
            WICED_BT_TRACE("SCRIPT: mesh_client_enqueue_and_check_event - no buffer - event: %d", event);
            return;
        }
        p_evt->rcvd_event = event;

        memcpy(&p_evt->uu, p_params, size_of_params);

        obj.pushEvent(p_evt);

        WICED_BT_TRACE("SCRIPT: mesh_client_enqueue_and_check_event event: %d", event);
    }
    else
    {
        tMESH_CLIENT_SCRIPT_EVENT_PARAMS *p_evt = NULL;
        
        while (p_evt == NULL)
        {
            p_evt = (tMESH_CLIENT_SCRIPT_EVENT_PARAMS *)obj.getEvent();
            if (p_evt != NULL)
            {
                mesh_client_rpc_return_event(p_evt);
                free(p_evt);
            }
            else
                Sleep(1000);
        }
    }
}


/*******************************************************************************
**
** Function         _wait_mesh_client_event
**
*******************************************************************************/
static DWORD WINAPI WaitThread(LPVOID lpThreadParameter)
{
    static int count = 0;
    mesh_client_enqueue_and_check_event(*(tMESH_CLIENT_SCRIPT_EVENT*) lpThreadParameter, NULL, 0);
    return count++;
}

extern "C" void _wait_mesh_client_event(UINT32 timeout, tMESH_CLIENT_SCRIPT_EVENT wait_event)
{
    static tMESH_CLIENT_SCRIPT_EVENT param = wait_event;
    DWORD dwThreadId = 0;
    HANDLE m_hThreadRead = CreateThread(NULL, 0, WaitThread, (LPVOID)&param, 0, &dwThreadId);

}
