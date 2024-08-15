
// SerialGattDlg.cpp : implementation file
//

#include "stdafx.h"
#include <setupapi.h>
#include "SerialGatt.h"
#include "SerialGattDlg.h"
#include "afxdialogex.h"
#include "DeviceSelectAdv.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

extern "C"
{
    typedef void(serial_client_cback_t)(unsigned long arg);

    void ProcessedReceivedData(unsigned char* p_data, unsigned short len);
    void StartTimer(unsigned short duration, serial_client_cback_t* p_timer_callback);
    void StopTimer(void);
    void SetMtu(unsigned short);
    void RegisterForNotifications(void);
    void SendSerialClientData(unsigned char* p_data, unsigned short len);
    void SerilaGattTxComplete(unsigned char result);
    void ods(char *txt);

    void serial_gatt_client_connection_up();
    void serial_gatt_client_connection_down();
    int serial_gatt_client_process_forward_data(unsigned char *p_buffer, unsigned short offset, unsigned short length);
    void serial_gatt_client_process_notification(unsigned short conn_id, unsigned char *p_data, unsigned short len);
}

void Log(WCHAR *fmt, ...)
{
    WCHAR   msg[1001];
    va_list cur_arg;

    memset(msg, 0, sizeof(msg));
    va_start(cur_arg, fmt);
    _vsnwprintf_s(msg, 1000, fmt, cur_arg);
    va_end(cur_arg);

    OutputDebugString(msg);
}


// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
    CAboutDlg();

    // Dialog Data
    enum { IDD = IDD_ABOUTBOX };

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
    DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CSerialGattDlg dialog

CSerialGattDlg *Dlg;

CSerialGattDlg::CSerialGattDlg(CWnd* pParent /*=NULL*/)
    : CDialogEx(CSerialGattDlg::IDD, pParent)
{
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
    m_btInterface = NULL;
    Dlg = this;
    m_hBsgTxComplete = CreateEvent(NULL, TRUE, FALSE, NULL);
    m_bsg_receive_file = NULL;
    InitializeCriticalSection(&m_cs);
    m_bWin8 = FALSE;
    m_bWin10 = FALSE;
}

CSerialGattDlg::~CSerialGattDlg()
{
    delete m_btInterface;
}

void CSerialGattDlg::SetParam(BLUETOOTH_ADDRESS *bth, HMODULE hLib)
{
    if (m_bWin10)
        m_btInterface = new CBtWin10Interface(bth, this);
    else if (m_bWin8)
        m_btInterface = new CBtWin8Interface(bth, hLib, this);
    else
        m_btInterface = new CBtWin7Interface(bth, hLib, this);
}

void CSerialGattDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CSerialGattDlg, CDialogEx)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_MESSAGE(WM_CONNECTED, OnConnected)
    ON_MESSAGE(WM_SERIAL_DATA, OnSerialGatt)
    ON_BN_CLICKED(IDCANCEL, &CSerialGattDlg::OnBnClickedCancel)
    ON_CBN_SELCHANGE(IDC_LONG_CHAR_CLIENT_CFG, &CSerialGattDlg::OnCbnSelchangeSerialGattClientCfg)
    ON_BN_CLICKED(IDC_SEND_FILE_START, &CSerialGattDlg::OnBnClickedSendFileStart)
    ON_BN_CLICKED(IDC_BROWSE_SEND_FILE, &CSerialGattDlg::OnBnClickedBrowseSendFile)
    ON_BN_CLICKED(IDC_BROWSE_RECV_FILE, &CSerialGattDlg::OnBnClickedBrowseRecvFile)
    ON_BN_CLICKED(IDC_RECEIVE_TO_FILE, &CSerialGattDlg::OnBnClickedBSGFileReceive)
END_MESSAGE_MAP()


// CSerialGattDlg message handlers

BOOL CSerialGattDlg::OnInitDialog()
{
    BOOL bConnected = TRUE;  // assume that device is connected which should generally be the case for location sensor

    CDialogEx::OnInitDialog();

    // Add "About..." menu item to system menu.

    // IDM_ABOUTBOX must be in the system command range.
    ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
    ASSERT(IDM_ABOUTBOX < 0xF000);

    CMenu* pSysMenu = GetSystemMenu(FALSE);
    if (pSysMenu != NULL)
    {
        BOOL bNameValid;
        CString strAboutMenu;
        bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
        ASSERT(bNameValid);
        if (!strAboutMenu.IsEmpty())
        {
            pSysMenu->AppendMenu(MF_SEPARATOR);
            pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
        }
    }

    // Set the icon for this dialog.  The framework does this automatically
    //  when the application's main window is not a dialog
    SetIcon(m_hIcon, TRUE);			// Set big icon
    SetIcon(m_hIcon, FALSE);		// Set small icon

    USHORT ClientConfDescrNotify = 0;

    m_btInterface->Init(guidSvcSerialGatt);

    // on Win7 we will receive notification when device is connected and will intialize dialog there
    if (!m_bWin8 && !m_bWin10)
        return TRUE;


    if (m_bWin10)
    {
        CBtWin10Interface *pWin10BtInterface = dynamic_cast<CBtWin10Interface *>(m_btInterface);

        // Assume that we are connected.  Failed attempt to read battery will change that to FALSE.
        pWin10BtInterface->m_bConnected = TRUE;

        if (pWin10BtInterface->m_bConnected)
        {
            pWin10BtInterface->RegisterNotification(&guidSvcSerialGatt, &guidCharSerialGatt);

            SetMtu(23);

            unsigned short mtu = pWin10BtInterface->GetMTUSize();
            if (mtu)
                SetMtu(mtu);

            this->PostMessage(WM_CONNECTED, (WPARAM)1, NULL);
        }
    }
    else if (m_bWin8)
    {
        CBtWin8Interface *pWin8BtInterface = dynamic_cast<CBtWin8Interface *>(m_btInterface);

        // Assume that we are connected.  
        pWin8BtInterface->m_bConnected = TRUE;
        m_bConnected = TRUE;

        pWin8BtInterface->RegisterNotification(guidCharSerialGatt, 0);

        this->PostMessage(WM_CONNECTED, (WPARAM)1, NULL);
    }
    return TRUE;  // return TRUE  unless you set the focus to a control
}

void CSerialGattDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
    if ((nID & 0xFFF0) == IDM_ABOUTBOX)
    {
        CAboutDlg dlgAbout;
        dlgAbout.DoModal();
    }
    else
    {
        CDialogEx::OnSysCommand(nID, lParam);
    }
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CSerialGattDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this); // device context for painting

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // Center icon in client rectangle
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        // Draw the icon
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CDialogEx::OnPaint();
    }
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CSerialGattDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}



void CSerialGattDlg::PostNcDestroy()
{
    CDialogEx::PostNcDestroy();
}


LRESULT CSerialGattDlg::OnConnected(WPARAM bConnected, LPARAM lparam)
{
    SetDlgItemText(IDC_DEVICE_STATE, bConnected ? L"Connected" : L"Disconnected");
    m_bConnected = (int)bConnected;
    EnterCriticalSection(&m_cs);
    if (bConnected)
        serial_gatt_client_connection_up();
    else
        serial_gatt_client_connection_down();
    LeaveCriticalSection(&m_cs);
    return S_OK;
}

BOOL CSerialGattDlg::WriteSerialGatt(LPBYTE pData, DWORD length)
{
    BTW_GATT_VALUE value = { 0 };
    memcpy(value.value, pData, length);
    value.len = (USHORT)length;

    return (m_btInterface->WriteCharacteristic(guidCharSerialGatt, 0, &value));
}

BOOL CSerialGattDlg::WriteSerialGattClientCfg(int ClientCfg)
{
    BTW_GATT_VALUE value = { 0 };
    value.len = 2;
    value.value[0] = ClientCfg;
    value.value[1] = 0;

    return (m_btInterface->SetDescriptorValue(guidCharSerialGatt, 0, BTW_GATT_UUID_DESCRIPTOR_CLIENT_CONFIG, &value));
}

LRESULT CSerialGattDlg::OnSerialGatt(WPARAM Instance, LPARAM lparam)
{
    BTW_GATT_VALUE *pValue = (BTW_GATT_VALUE *)lparam;
    EnterCriticalSection(&m_cs);
    serial_gatt_client_process_notification(1, pValue->value, pValue->len);
    LeaveCriticalSection(&m_cs);
    free(pValue);
    return S_OK;
}

void CSerialGattDlg::OnCbnSelchangeSerialGattClientCfg()
{
    int Sel = ((CComboBox *)GetDlgItem(IDC_LONG_CHAR_CLIENT_CFG))->GetCurSel();
    WriteSerialGattClientCfg(Sel);
}

BYTE ProcNibble(char n)
{
    if ((n >= '0') && (n <= '9'))
    {
        n -= '0';
    }
    else if ((n >= 'A') && (n <= 'F'))
    {
        n = ((n - 'A') + 10);
    }
    else if ((n >= 'a') && (n <= 'f'))
    {
        n = ((n - 'a') + 10);
    }
    else
    {
        n = (char)0xff;
    }
    return (n);
}

DWORD CSerialGattDlg::GetHexValue(DWORD id, LPBYTE buf, DWORD buf_size)
{
    char szbuf[1024];
    char *psz = szbuf;
    BYTE *pbuf = buf;
    DWORD res = 0;

    memset(buf, 0, buf_size);

    GetDlgItemTextA(m_hWnd, id, szbuf, 1024);
    if (strlen(szbuf) == 1)
    {
        szbuf[2] = 0;
        szbuf[1] = szbuf[0];
        szbuf[0] = '0';
    }
    else if (strlen(szbuf) == 3)
    {
        szbuf[4] = 0;
        szbuf[3] = szbuf[2];
        szbuf[2] = szbuf[1];
        szbuf[1] = szbuf[0];
        szbuf[0] = '0';
    }
    for (DWORD i = 0; i < strlen(szbuf); i++)
    {
        if (isxdigit(psz[i]) && isxdigit(psz[i + 1]))
        {
            *pbuf++ = (ProcNibble(psz[i]) << 4) + ProcNibble(psz[i + 1]);
            res++;
            i++;
        }
    }
    return res;
}

DWORD WINAPI SendFileThread(void* param)
{
    return ((CSerialGattDlg *)param)->SendFileThread();
}

#define WICED_BT_RFCOMM_SUCCESS 0
#define WICED_BT_RFCOMM_NO_MEM  5

DWORD CSerialGattDlg::SendFileThread()
{
    FILE *fp = NULL;
    char buf[1030] = { 0 };
    char filename[MAX_PATH];
    GetDlgItemTextA(m_hWnd, IDC_SEND_FILENAME, filename, sizeof(filename));
    fopen_s(&fp, filename, "rb");
    if (!fp)
    {
        WCHAR name[512] = { 0 };
        MultiByteToWideChar(CP_ACP, 0, (const char *)filename, (int)strlen(filename), name, (int)sizeof(name)/sizeof(WCHAR));
        Log(L"Failed to open file %s", name);
        return 0;
    }

    fseek(fp, 0, SEEK_END);
    m_bsg_total_to_send = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    m_bsg_bytes_sent = 0;

    // MTU 512  Win8: 512 - (5 bytes serial gatt header + 3 bytes ATT header)
    // MTU 23   Win7: 23 - (5 bytes serial gatt header + 3 bytes ATT header)
    DWORD dwBytes = 0;

    if (m_bWin10)
    {
        CBtWin10Interface *pWin10BtInterface = dynamic_cast<CBtWin10Interface *>(m_btInterface);
        unsigned short mtu = pWin10BtInterface->GetMTUSize();
        if (mtu)
            dwBytes = (mtu >= 512) ? 504 : (mtu - 8);
        else
            dwBytes = m_bWin10 ? 504 : 15;
    }
    else
    {
        dwBytes = m_bWin8 ? 504 : 15;
    }

    int read_bytes;
    while (m_bConnected &&
        ((txBuffer = (LPBYTE)malloc(dwBytes + 5)) != NULL) &&
        ((read_bytes = (int)fread(&txBuffer[5], 1, dwBytes, fp)) != 0))
    {
        ResetEvent(m_hBsgTxComplete);

        EnterCriticalSection(&m_cs);
        serial_gatt_client_process_forward_data(txBuffer, 5, read_bytes);
        LeaveCriticalSection(&m_cs);

        m_bsg_bytes_sent += read_bytes;

        if (WaitForSingleObject(m_hBsgTxComplete, 10000) != WAIT_OBJECT_0)
        {
            free(Dlg->txBuffer);
            OutputDebugString(L"SendFileThread() - ERROR Wait failed\n");
            break;
        }
        if ((m_bsg_tx_complete_result != 0) ||
            (((CButton*)GetDlgItem(IDC_SEND_FROM_FILE))->GetCheck() != BST_CHECKED))
        {
            m_bsg_total_to_send = 0;
            break;
        }
    }
    fclose(fp);
    GetDlgItem(IDC_SEND_FILE_START)->EnableWindow(TRUE);
    return 0;
}


void CSerialGattDlg::OnBnClickedSendFileStart()
{
    if (((CButton *)GetDlgItem(IDC_SEND_FROM_FILE))->GetCheck() != 1)
    {
        USHORT len;

        m_bsg_total_to_send = 0;
        m_bsg_bytes_sent = 0;

        if ((txBuffer = (LPBYTE)malloc(512 + 5)) != NULL)
        {
            // leave some space for the header
            memset(txBuffer, 0, 512 + 5);
            char szbuf[1024] = { 0 };
            GetDlgItemTextA(m_hWnd, IDC_SEND_TEXT, szbuf, 512);
            len = (USHORT) strlen(szbuf);
            if (len && (len <= 512))
            {
                memcpy(&txBuffer[5], szbuf, len);
                EnterCriticalSection(&m_cs);
                serial_gatt_client_process_forward_data(txBuffer, 5, len);
                LeaveCriticalSection(&m_cs);
            }
            else
            {
                OutputDebugString(L"Enter valid string\n");
            }
        }
    }
    else
    {
        GetDlgItem(IDC_SEND_FILE_START)->EnableWindow(FALSE);
        HANDLE hSppSendThread = CreateThread(NULL, 0, ::SendFileThread, this, 0, NULL);
    }
}

void CSerialGattDlg::OnBnClickedCancel()
{
    CDialogEx::OnCancel();
}

void ProcessedReceivedData(LPBYTE p_data, USHORT len)
{
    WCHAR buf[1024 * 3] = { 0 };

    for (int i = 0; i < len; i++)
        wsprintf(&buf[wcslen(buf)], L"%c", p_data[i]);

    if (Dlg->m_bsg_receive_file)
    {
        fwrite(p_data, len, 1, Dlg->m_bsg_receive_file);
    }
    else
    {
        Dlg->SetDlgItemText(IDC_RECV_TEXT, buf);
    }
}

UINT_PTR nTimerEvent;
serial_client_cback_t* timer_callback = NULL;

VOID CALLBACK SerialGattTimerFunc(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
{
    KillTimer(NULL, nTimerEvent);
    nTimerEvent = NULL;

    if (timer_callback)
        timer_callback(1);
}

void StartTimer(unsigned short duration, serial_client_cback_t* p_timer_callback)
{
    StopTimer();
    timer_callback = p_timer_callback;
    nTimerEvent = SetTimer(NULL, 1, duration, &SerialGattTimerFunc);
}

void StopTimer(void)
{
    if (nTimerEvent)
    {
        KillTimer(NULL, nTimerEvent);
        nTimerEvent = NULL;
    }
}



void RegisterForNotifications(void)
{
    Dlg->WriteSerialGattClientCfg(1);
}

void SendSerialClientData(unsigned char* p_data, unsigned short len)
{
    Dlg->WriteSerialGatt(p_data, len);
}

void SerilaGattTxComplete(unsigned char result)
{
    Dlg->m_bsg_tx_complete_result = result;
    free(Dlg->txBuffer);
    SetEvent(Dlg->m_hBsgTxComplete);
}

void ods(char *txt)
{
    OutputDebugStringA(txt);
}

void CSerialGattDlg::OnBnClickedBrowseSendFile()
{
    CFileDialog dlgFile(TRUE, NULL, NULL, OFN_OVERWRITEPROMPT, NULL);
    if (dlgFile.DoModal() == IDOK)
    {
        SetDlgItemText(IDC_SEND_FILENAME, dlgFile.GetPathName());
    }
}

void CSerialGattDlg::OnBnClickedBrowseRecvFile()
{
    CFileDialog dlgFile(TRUE, NULL, NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, NULL);
    if (dlgFile.DoModal() == IDOK)
    {
        SetDlgItemText(IDC_RECV_FILENAME, dlgFile.GetPathName());

        OnBnClickedBSGFileReceive();
    }
}

void CSerialGattDlg::OnBnClickedBSGFileReceive()
{
    //Log(L"OnBnClickedBSGFileReceive %d", ((CButton*)GetDlgItem(IDC_RECEIVE_TO_FILE))->GetCheck());
    if (m_bsg_receive_file)
    {
        fclose(m_bsg_receive_file);
        m_bsg_receive_file = NULL;
    }

    if (((CButton*)GetDlgItem(IDC_RECEIVE_TO_FILE))->GetCheck())
    {
        TCHAR szName[512];
        USES_CONVERSION;

        char filename[MAX_PATH];
        GetDlgItemTextA(m_hWnd, IDC_RECV_FILENAME, filename, sizeof(filename));
        fopen_s(&m_bsg_receive_file, filename, "wb");

        _tcscpy_s(szName, A2T(filename));

        if (!m_bsg_receive_file)
            Log(L"Error: could not open bsg receive file %s \n", szName);
        else
            Log(L"Opened bsg receive file %s \n", szName);
    }
}
