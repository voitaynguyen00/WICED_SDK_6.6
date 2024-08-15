
// ClientControlDlg.h : header file
//

#pragma once


// CClientControlDlg dialog
class CClientControlDlg : public CDialogEx
{
// Construction
public:
	CClientControlDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_CLIENTCONTROL_DIALOG };


	// Generated message map functions
	virtual BOOL OnInitDialog();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedHwButton();
	afx_msg void OnDestroy();
};
