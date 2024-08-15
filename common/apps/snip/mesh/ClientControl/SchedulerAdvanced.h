#pragma once


// CSchedulerAdvanced dialog

class CSchedulerAdvanced : public CDialogEx
{
	DECLARE_DYNAMIC(CSchedulerAdvanced)

public:
	CSchedulerAdvanced(CWnd* pParent = NULL);   // standard constructor
	virtual ~CSchedulerAdvanced();
    USHORT m_month_selection;
    BYTE m_day_of_week_selection;
    BYTE m_year;
    BYTE m_day;
    BYTE m_hour;
    BYTE m_minute;
    BYTE m_second;
    // Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SCHEDULER_ADVANCED };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
    afx_msg void OnBnClickedOk();
    virtual BOOL OnInitDialog();
};
