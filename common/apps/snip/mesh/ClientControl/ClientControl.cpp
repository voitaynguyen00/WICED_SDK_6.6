
// ClientControl.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include "stdint.h"
//#include "wiced.h"
#include "ClientControl.h"
#include "ClientControlDlg.h"
#include "resource.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

extern "C" CRITICAL_SECTION cs;
CRITICAL_SECTION cs;

int as32BaudRate[4] =
{
    115200,
    921600,
    3000000,
    4000000
};

int aComPorts[128] = { 0 };
int ComPortSelected = -1;
int BaudRateSelected = -1;

#if 0
uint64_t TickCountInitValue;
#endif

// CClientControlApp

BEGIN_MESSAGE_MAP( CClientControlApp, CWinApp )
	ON_COMMAND( ID_HELP, &CWinApp::OnHelp )
END_MESSAGE_MAP( )


// CClientControlApp construction

CClientControlApp::CClientControlApp( )
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}


// The one and only CClientControlApp object

CClientControlApp theApp;

// CClientControlApp initialization

BOOL CClientControlApp::InitInstance( )
{
    InitializeCriticalSection(&cs);
    // InitCommonControlsEx( ) is required on Windows XP if an application
    // manifest specifies use of ComCtl32.dll version 6 or later to enable
    // visual styles.  Otherwise, any window creation will fail.
    INITCOMMONCONTROLSEX InitCtrls;
    InitCtrls.dwSize = sizeof( InitCtrls );
    // Set this to include all the common control classes you want to use
    // in your application.
    InitCtrls.dwICC = ICC_WIN95_CLASSES;
    InitCommonControlsEx( &InitCtrls );

    CWinApp::InitInstance( );

    // Create the shell manager, in case the dialog contains
    // any shell tree view or shell list view controls.
    CShellManager *pShellManager = new CShellManager;

    // Activate "Windows Native" visual manager for enabling themes in MFC controls
    CMFCVisualManager::SetDefaultManager( RUNTIME_CLASS( CMFCVisualManagerWindows ) );

    CCommandLineInfo cmdInfo;
    ParseCommandLine( cmdInfo );

    // Change the registry key under which our settings are stored
    SetRegistryKey( _T( "Cypress" ) );

    srand((DWORD)time(NULL));

    memset(aComPorts, 0, sizeof(aComPorts));
    int numPorts = 0;
    for (int i = 1; i < 128; i++)
    {
        WCHAR buf[20];
        wsprintf(buf, L"\\\\.\\COM%d", i);
        HANDLE handle = CreateFile(buf, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
        if (handle != INVALID_HANDLE_VALUE)
        {
            CloseHandle(handle);
            aComPorts[numPorts++] = i;
        }
    }
#if 0
    TickCountInitValue = GetTickCount64() / 1000;
#endif

    CClientDialog dlg(_T("Mesh Client Control"));
    m_pMainWnd = &dlg;
    dlg.DoModal();

    // Delete the shell manager created above.
    if ( pShellManager != NULL )
    {
        delete pShellManager;
    }

    // Since the dialog has been closed, return FALSE so that we exit the
    //  application, rather than start the application's message pump.
    return FALSE;
}

IMPLEMENT_DYNAMIC(CClientDialog, CPropertySheet)

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
#include "mesh_automation.h"
CSocketWindow *pSocketWin = NULL;
extern "C" void ods(char * fmt_str, ...)
{
    char buf[1000] = { 0 };
    va_list marker = NULL;

    va_start(marker, fmt_str);

    SYSTEMTIME st;
    GetLocalTime(&st);

    int len = sprintf_s(buf, sizeof(buf), "%02d:%02d:%02d.%03d ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    vsnprintf_s(&buf[len], sizeof(buf) - len, _TRUNCATE, fmt_str, marker);
    va_end(marker);

    if (buf[strlen(buf) - 1] != '\n')
        strcat_s(buf, sizeof(buf), "\n");
    OutputDebugStringA(buf);
}
#endif

CClientDialog::CClientDialog(UINT nIDCaption, CWnd* pParentWnd, UINT iSelectPage)
    :CPropertySheet(nIDCaption, pParentWnd, iSelectPage)
{
    m_psh.dwFlags &= ~PSH_HASHELP;
#ifndef NO_LIGHT_CONTROL
    AddPage(&pageLight);
#endif
    AddPage(&pageMain);
    AddPage(&pageConfig);
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    if (pSocketWin == NULL)
        pSocketWin = new CSocketWindow();
#endif
}

CClientDialog::CClientDialog(LPCTSTR pszCaption, CWnd* pParentWnd, UINT iSelectPage)
    :CPropertySheet(pszCaption, pParentWnd, iSelectPage)
{
    m_psh.dwFlags &= ~PSH_HASHELP;
#ifndef NO_LIGHT_CONTROL
    AddPage(&pageLight);
#endif
    AddPage(&pageMain);
    AddPage(&pageConfig);
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    if (pSocketWin == NULL)
        pSocketWin = new CSocketWindow();
#endif
}

CClientDialog::~CClientDialog()
{
    DeleteCriticalSection(&cs);
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    if (pSocketWin != NULL)
    {
        pSocketWin->DestroyWindow();
        delete pSocketWin;
        pSocketWin = NULL;
    }
#endif
}

BOOL CClientDialog::OnInitDialog()
{
    BOOL ret =  CPropertySheet::OnInitDialog();
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    if (pSocketWin != NULL)
        pSocketWin->Create();
#endif
    return ret;
}

BEGIN_MESSAGE_MAP(CClientDialog, CPropertySheet)
    //{{AFX_MSG_MAP(COptions)
    //ON_COMMAND(ID_APPLY_NOW, OnApplyNow)
    //ON_COMMAND(IDOK, OnOK)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

#include "wiced_timer.h"

wiced_result_t wiced_init_timer(wiced_timer_t* p_timer, wiced_timer_callback_t TimerCb, TIMER_PARAM_TYPE cBackparam, wiced_timer_type_t type)
{
    return WICED_BT_SUCCESS;
}

wiced_result_t wiced_deinit_timer(wiced_timer_t* p)
{
    return WICED_BT_SUCCESS;
}

wiced_result_t wiced_start_timer(wiced_timer_t* wt, uint32_t timeout)
{
    return WICED_ERROR;
}

wiced_result_t wiced_stop_timer(wiced_timer_t* wt)
{
    return WICED_ERROR;
}

extern "C" void wiced_bt_mesh_gatt_client_connection_state_changed(uint16_t conn_id, uint16_t mtu)
{
}

extern "C" void wiced_bt_mesh_remote_provisioning_connection_state_changed(uint16_t conn_id, uint8_t reason)
{
}

extern "C" uint8_t wiced_hci_send(uint16_t opcode, uint8_t *p_buffer, uint16_t length)
{
    return ((uint16_t)m_ComHelper->SendWicedCommand(opcode, p_buffer, length) != 0);
}

/////////////////////////////////////////////////////////////////////////////

extern "C" HANDLE hGatewayEvent;
HANDLE hGatewayEvent;

extern "C" void mible_wiced_init(void)
{
    hGatewayEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
}

extern "C" void mible_wiced_wait_event(uint32_t timeout)
{
    WaitForSingleObject(hGatewayEvent, timeout);
}

extern "C" void mible_wiced_set_event(void)
{
    SetEvent(hGatewayEvent);
}
