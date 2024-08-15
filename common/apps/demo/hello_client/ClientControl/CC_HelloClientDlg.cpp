
// ClientControlDlg.cpp : implementation file
//

#include "stdafx.h"
#include "CC_HelloClient.h"
#include "CC_HelloClientDlg.h"
#include "afxdialogex.h"
#include "IpcClSocket.h"

IpcClSocket g_ClientSock;

void Log(WCHAR *fmt, ...)
{
    WCHAR   msg[1001];
    va_list cur_arg;

    memset(msg, 0, sizeof(msg));
    va_start(cur_arg, fmt);
    _vsnwprintf_s(msg, 1000, fmt, cur_arg);
    va_end(cur_arg);
}

CClientControlDlg::CClientControlDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CClientControlDlg::IDD, pParent)
{

}

BEGIN_MESSAGE_MAP(CClientControlDlg, CDialogEx)
	ON_BN_CLICKED(IDC_HW_BUTTON, &CClientControlDlg::OnBnClickedHwButton)
	ON_WM_DESTROY()
END_MESSAGE_MAP()


BOOL CClientControlDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	g_ClientSock.Open();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void win_send_data(LPBYTE lpBytes, DWORD dwLen)
{
	g_ClientSock.SendToServer(lpBytes, dwLen);
}

void CClientControlDlg::OnBnClickedHwButton()
{
	// byte 0: unused
	// byte 1: 2 - h/w button presses
	// byte 2: 30 - pin#
	// byte 3: 1|0 - pin state
	BYTE data[4] = { 0, 2, 30, IsDlgButtonChecked(IDC_HW_BUTTON) ? 1 : 0 };

	win_send_data( data, 4 );
}

void CClientControlDlg::OnDestroy()
{
	g_ClientSock.Close();
	CDialogEx::OnDestroy();	
}
