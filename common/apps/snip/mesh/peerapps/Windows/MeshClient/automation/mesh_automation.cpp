/****************************************************************************
**
**  Name:       mesh_automation.cpp
**
**  Function:   this file contains functions to handle Mesh Automation
**              commands and events.
**
**  Copyright (c) 2018, Cypress Semiconductor Corp., All Rights Reserved.
**  Cypress Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include "afxwin.h"

#include "windows.h"
#include "winsock.h"
#include <string.h>
#include "bt_types.h"
#include "platform.h"
#include "mesh_client_script.h"
#include "wiced_bt_trace.h"

#include "hci_control_api.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "mesh_automation.h"
#include <fcntl.h> /* Added for the nonblocking socket */

static int SOCK_PORT_NUM = 12012;

#define HOST_PKT_TYPE_COMMAND       1
#define HOST_PKT_TYPE_ACL_DATA      2
#define HOST_PKT_TYPE_EVENT         4

#define WICED_APP_SCRIPT_INCLUDED           TRUE

#define WM_SOCKET (WM_USER + 181)

typedef struct
{
    UINT8   pkt_type;
    UINT8   data[1200];
} tSCRIPT_PKT;


static SOCKET  m_ListenSocket = INVALID_SOCKET;
static SOCKET  m_ClientSocket = INVALID_SOCKET;
static int readHostTCPpkt(unsigned char *pPkt);

extern "C" void hci_control_script_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t len);


/*
* Handle received command over UART. Please refer to the WICED Smart Ready
* Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
* HCI UART control protocol.
*/
static uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length)
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t  buffer_processed = WICED_TRUE;

    WICED_BT_TRACE( "hci_control_proc_rx_cmd:%d\n", length );

    if (!p_buffer)
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if ((length < 4) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\n");

        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("hci_control_proc_rx_cmd() - cmd_opcode 0x%04x\n", opcode);

    switch ((opcode >> 8) & 0xff)
    {
#if (WICED_APP_SCRIPT_INCLUDED == TRUE)
    case HCI_CONTROL_GROUP_SCRIPT:
        hci_control_script_handle_command(opcode, p_data, payload_len);
        break;
#endif

    default:
        WICED_BT_TRACE("unknown class code (opcode:%x)\n", opcode);
        break;
    }

    return HCI_CONTROL_STATUS_SUCCESS;
}

// Call WICED HCI API from script
int CallWicedHciApi(UINT8 *data, UINT8 len)
{
    char s[1200] = { 0 };

    for (int i = 0, j = 0; i < len; i++)
    {
        j = i * 3;
        sprintf(&s[j], "x%X,", data[i]);
    }
    WICED_BT_TRACE("packet %s, len %d", s, len);

    // Calling Rx handler
    hci_control_proc_rx_cmd(data, len);

    return 0;
}


// Send return code to script
void SendScriptReturnCode(int iRet)
{
    // Return code to python
    tSCRIPT_PKT temp;
    memset(&temp.data, 0, 1200);
    temp.pkt_type = HCI_WICED_PKT;
    temp.data[0] = 0x1;
    temp.data[1] = HCI_CONTROL_GROUP_SCRIPT;
    temp.data[2] = 5;
    temp.data[3] = iRet;

    if (SOCKET_ERROR == (iRet = send(m_ClientSocket, (char *)&temp, 5, 0)))
    {
        WICED_BT_TRACE("SendScriptReturnCode() Error\n");
    }
}

static int readHostTCPpkt(unsigned char *pPkt)
{
    unsigned int readLen, hdrLen, dataLen;

    if ((readLen = recv(m_ClientSocket, (char *)pPkt, 1, 0)) != 1)
    {
        WICED_BT_TRACE("readHostTCPpkt() Expected 1, got: %d", readLen);
        return (-1);
    }

    // ACL and WICED-HCI share the same basic format
    if ((pPkt[0] == HCI_WICED_PKT) /*|| (pPkt[0] == MPAF_TRAN_PKT_TYPE)*/)
    {
        if ((hdrLen = recv(m_ClientSocket, (char *)&pPkt[1], 4, 0)) != 4)
        {
            WICED_BT_TRACE("readHostTCPpkt() Expected 4, got: %d", readLen);
            return (-1);
        }
        dataLen = pPkt[3] | (pPkt[4] << 8);
    }
    else if (pPkt[0] == HOST_PKT_TYPE_COMMAND)
    {
        if ((hdrLen = recv(m_ClientSocket, (char *)&pPkt[1], 3, 0)) != 3)
        {
            WICED_BT_TRACE("readHostTCPpkt() Expected 3, got: %d", readLen);
            return (-1);
        }
        dataLen = pPkt[3];
    }
    else
    {
        WICED_BT_TRACE("!!!!Unknown Type: %u", pPkt[0]);
        return (-1);
    }

    if (dataLen != 0)
    {
        if ((readLen = recv(m_ClientSocket, (char *)&pPkt[1 + hdrLen], dataLen, 0)) != dataLen)
        {
            WICED_BT_TRACE("readHostTCPpkt() Expected to read datalen of %u, actually got: %d", dataLen, readLen);
            return (-1);
        }
    }

    return (1 + hdrLen + dataLen);
}


int read_script_pct(char *pPkt)
{
    unsigned int readLen, hdrLen, dataLen;

    if ((readLen = recv(m_ClientSocket, (char *)pPkt, 1, 0)) != 1)
    {
        WICED_BT_TRACE("read_script_pct() Expected 1, got: %d", readLen);
        return (-1);
    }

    // ACL and WICED-HCI share the same basic format
    if (pPkt[0] == HCI_WICED_PKT)
    {
        if ((hdrLen = recv(m_ClientSocket, (char *)&pPkt[1], 4, 0)) != 4)
        {
            WICED_BT_TRACE("read_script_pct() Expected 4, got: %d", readLen);
            return (-1);
        }
        dataLen = pPkt[3] | (pPkt[4] << 8);
    }
    else
    {
        WICED_BT_TRACE("!!!!Unknown Type: %u", pPkt[0]);
        return (-1);
    }
    if (dataLen != 0)
    {
        if ((readLen = recv(m_ClientSocket, (char *)&pPkt[1 + hdrLen], dataLen, 0)) != dataLen)
        {
            WICED_BT_TRACE("read_script_pct() Expected to read datalen of %u, actually got: %d", dataLen, readLen);
            return (-1);
        }
    }

    return (1 + hdrLen + dataLen);
}


extern "C" void sendToHost(int type, UINT8 *pData, UINT32 len)
{
    tSCRIPT_PKT    UdpPkt;
    int            iRet;

    if (m_ClientSocket == INVALID_SOCKET)
    {
        WICED_BT_TRACE("!!!!  sendToHost() - no TCP socket to host - dropping data !!!");
        return;
    }

    memset(&UdpPkt, 0, sizeof(tSCRIPT_PKT));
    UdpPkt.pkt_type = HCI_WICED_PKT;
    UdpPkt.data[0] = 0x1;
    UdpPkt.data[1] = HCI_CONTROL_GROUP_SCRIPT;
    UdpPkt.data[2] = len & 0xff;
    UdpPkt.data[3] = (len >> 8) & 0xff;

    if (len)
    {
        memcpy((char*)&UdpPkt.data[4], pData, len);
    }

    len += 5; //Adding len used for header

    if (SOCKET_ERROR == (iRet = send(m_ClientSocket, (char *)&UdpPkt, len, 0)))
        WICED_BT_TRACE("TCP socket send failed: %d", iRet);
}


extern "C" void     script_post_bt_init();

DWORD __stdcall socketRecv(HWND handle)
{
    int                 iResult;
    SOCKADDR_IN         service;
    WSADATA             wsaData;

    WSAStartup(MAKEWORD(2, 2), &wsaData);

    // Create a local SOCKET for incoming connection
    if (INVALID_SOCKET == (m_ListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
    {
        WICED_BT_TRACE("listen socket failed with error: %ld, socket thread exiting\n", WSAGetLastError());
        return 0;
    }

    service.sin_family = AF_INET;
    service.sin_addr.s_addr = inet_addr("127.0.0.1");
    service.sin_port = htons(SOCK_PORT_NUM);

    if (SOCKET_ERROR == (iResult = bind(m_ListenSocket, (SOCKADDR *)& service, sizeof(service))))
    {
        WICED_BT_TRACE("bind failed with error: %d, socket thread exiting\n", WSAGetLastError());
        closesocket(m_ListenSocket);
        return 0;
    }

    {
        WICED_BT_TRACE("Listening for client to connect TCP socket....");

        if (m_ClientSocket != INVALID_SOCKET)
        {
            closesocket(m_ClientSocket);
            m_ClientSocket = INVALID_SOCKET;
        }

        if (WSAAsyncSelect(m_ListenSocket, (HWND)handle, WM_SOCKET, FD_ACCEPT | FD_CLOSE) == 0)
            WICED_BT_TRACE("WSAAsyncSelect() is OK lol!\n");
        else
            WICED_BT_TRACE("WSAAsyncSelect() failed with error code %d\n", WSAGetLastError());

        if (SOCKET_ERROR == (iResult = listen(m_ListenSocket, 1)))
        {
            WICED_BT_TRACE("TCP socket listen failed with error: %d\n", WSAGetLastError());
            return 0;
        }
    }

    script_post_bt_init();

    return 0;
}

///////////////////////////////////////////////////////////////////

void OnFDAccept(HWND handle, WPARAM wParam, LPARAM lParam)
{
    SOCKET sock = (SOCKET)wParam;
    // Accept the client TCP socket
    if (INVALID_SOCKET == (m_ClientSocket = accept(m_ListenSocket, NULL, NULL)))
    {
        WICED_BT_TRACE("Client TCP socket accept failed with error: %d", WSAGetLastError());
        return ;
    }

    if (WSAAsyncSelect(m_ClientSocket, (HWND)handle, WM_SOCKET, FD_READ | FD_WRITE | FD_CLOSE) == 0)
        WICED_BT_TRACE("WSAAsyncSelect() is OK lol!\n");
    else
        WICED_BT_TRACE("WSAAsyncSelect() failed with error code %d\n", WSAGetLastError());
}

void OnFDRead(WPARAM wParam, LPARAM lParam)
{
    tSCRIPT_PKT    pkt;
    int            bytes_rcvd;

    //ods("Before read_script_pct...");
    bytes_rcvd = read_script_pct((char *)&pkt);
    WICED_BT_TRACE("After read_script_pct...");
    if (bytes_rcvd <= 0)
    {
        WICED_BT_TRACE("Client TCP socket recv failed with error: %d    Closing...", WSAGetLastError());
        return;
    }

    WICED_BT_TRACE(">>> TCP Socket >>> Received", &pkt);

    if (pkt.pkt_type == HCI_WICED_PKT)
    {
        int iRet = CallWicedHciApi(pkt.data, bytes_rcvd - 1);
    }
}
IMPLEMENT_DYNAMIC(CSocketWindow, CWnd)

BOOL CSocketWindow::Create()
{
    if (!CWnd::CreateEx(0, AfxRegisterWndClass(0),
        _T("CSocketWindows"),
        WS_OVERLAPPED, 0, 0, 0, 0, NULL, NULL))
    {
        return FALSE;
    }
    return TRUE;
}

 int CSocketWindow::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
    int ret = CWnd::OnCreate(lpCreateStruct);
    socketRecv(GetSafeHwnd());
    return ret;
}

BEGIN_MESSAGE_MAP(CSocketWindow, CWnd)
    //{{AFX_MSG_MAP(COptions)
    //ON_COMMAND(ID_APPLY_NOW, OnApplyNow)
    //ON_COMMAND(IDOK, OnOK)
    //}}AFX_MSG_MAP
    ON_WM_CREATE()
    ON_MESSAGE(WM_SOCKET, &CSocketWindow::OnSocketMessage)
END_MESSAGE_MAP()

extern void OnFDRead(WPARAM wParam, LPARAM lParam);
extern void OnFDAccept(HWND hwnd, WPARAM wParam, LPARAM lParam);

LRESULT CSocketWindow::OnSocketMessage(WPARAM wParam, LPARAM lParam)
{
    SOCKET sock = (SOCKET)wParam;
    int event = WSAGETSELECTEVENT(lParam);
    int WsaErr = WSAGETSELECTERROR(lParam);
    // Determine whether an error occurred on the socket by using the WSAGETSELECTERROR() macro
    if (WsaErr)
    {
        closesocket(sock);
        return 0;
    }
    switch (event)// Determine what event occurred on the socket
    {
    case FD_ACCEPT:
        OnFDAccept(m_hWnd, wParam, lParam);
        break;        // Accept an incoming connection
    case FD_READ:
    {
        OnFDRead(wParam, lParam);
    }
    break;        // Receive data from the socket in wParam
    case FD_WRITE:
        //OnWrite(sock);
        //ods(">>> FD_WRITE >>> Received");
        break;        // The socket in wParam is ready for sending data
    case FD_CLOSE:
        closesocket(sock);
        break;        // The connection is now closed
    }
    return S_OK;

}

