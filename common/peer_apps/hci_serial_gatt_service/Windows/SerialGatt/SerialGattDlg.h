
// SerialGattDlg.h : header file
//

#pragma once
#include "Win7Interface.h"
#include "Win8Interface.h"
#include "Win10Interface.h"

// CSerialGattDlg dialog
class CSerialGattDlg : public CDialogEx
{
    // Construction
public:
    CSerialGattDlg(CWnd* pParent = NULL);   // standard constructor
    virtual ~CSerialGattDlg();

    void SetParam(BLUETOOTH_ADDRESS *bth, HMODULE hLib);

    // Dialog Data
    enum { IDD = IDD_SERIAL_GATT_CLIENT };

    BOOL m_bWin8;
    BOOL m_bWin10;
    //    BLUETOOTH_ADDRESS m_bth;
    HMODULE m_hLib;
    CBtInterface *m_btInterface;
    BOOL WriteSerialGatt(LPBYTE pData, DWORD length);
    BOOL ReadSerialGattClientCfg();
    BOOL WriteSerialGattClientCfg(int ClientCfg);
    DWORD GetHexValue(DWORD id, LPBYTE buf, DWORD buf_size);
    LPBYTE txBuffer;
    DWORD SendFileThread();
    HANDLE  m_hBsgTxComplete;
    BYTE    m_bsg_tx_complete_result;
    int m_bConnected;
    CRITICAL_SECTION m_cs;
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

private:
    HANDLE m_hDevice;

    DWORD   m_bsg_bytes_sent;
    DWORD   m_bsg_total_to_send;
    WORD    m_bsg_sent;
    WORD    m_bsg_acked;
    DWORD   m_uart_tx_size;

public:
    FILE   *m_bsg_receive_file;

    // Implementation
protected:
    HICON m_hIcon;

    // Generated message map functions
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    DECLARE_MESSAGE_MAP()
    virtual void PostNcDestroy();
    LRESULT OnConnected(WPARAM bConnected, LPARAM lparam);
    LRESULT OnSerialGatt(WPARAM op, LPARAM lparam);
public:
    afx_msg void OnBnClickedCancel();
    afx_msg void OnCbnSelchangeSerialGattClientCfg();
    afx_msg void OnBnClickedSendFileStart();
    afx_msg void OnBnClickedBrowseSendFile();
    afx_msg void OnBnClickedBrowseRecvFile();
    afx_msg void OnBnClickedBSGFileReceive();
};
