// SchedulerAdvanced.cpp : implementation file
//

#include "stdafx.h"
#include "SchedulerAdvanced.h"
#include "afxdialogex.h"
#include "resource.h"

// CSchedulerAdvanced dialog

IMPLEMENT_DYNAMIC(CSchedulerAdvanced, CDialogEx)

CSchedulerAdvanced::CSchedulerAdvanced(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SCHEDULER_ADVANCED, pParent)
{
}

CSchedulerAdvanced::~CSchedulerAdvanced()
{
}

BOOL CSchedulerAdvanced::OnInitDialog()
{
    CDialogEx::OnInitDialog();
    ((CComboBox*)GetDlgItem(IDC_SCHEDULER_YEAR))->SetCurSel(0);
    ((CComboBox*)GetDlgItem(IDC_SCHEDULER_DAY))->SetCurSel(0);
    ((CComboBox*)GetDlgItem(IDC_SCHEDULER_HOUR))->SetCurSel(0);
    ((CComboBox*)GetDlgItem(IDC_SCHEDULER_MINUTE))->SetCurSel(0);
    ((CComboBox*)GetDlgItem(IDC_SCHEDULER_SECOND))->SetCurSel(0);

    for (int i = 0; i < 7; i++)
    {
        if (m_day_of_week_selection & (1 << i))
            ((CButton *)GetDlgItem(IDC_MONDAY + i))->SetCheck(1);
    }
    for (int i = 0; i < 12; i++)
    {
        if (m_month_selection & (1 << i))
            ((CButton *)GetDlgItem(IDC_JANUARY + i))->SetCheck(1);
    }
    return TRUE;  // return TRUE unless you set the focus to a control
}

void CSchedulerAdvanced::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CSchedulerAdvanced, CDialogEx)
    ON_BN_CLICKED(IDOK, &CSchedulerAdvanced::OnBnClickedOk)
END_MESSAGE_MAP()


// CSchedulerAdvanced message handlers


void CSchedulerAdvanced::OnBnClickedOk()
{
    for (int i = 0; i < 7; i++)
    {
        if (((CButton *)GetDlgItem(IDC_MONDAY + i))->GetCheck())
            m_day_of_week_selection |= (1 << i);
    }
    for (int i = 0; i < 12; i++)
    {
        if (((CButton *)GetDlgItem(IDC_JANUARY + i))->GetCheck())
            m_month_selection |= (1 << i);
    }
    m_year = ((CComboBox*)GetDlgItem(IDC_SCHEDULER_YEAR))->GetCurSel();
    m_day = ((CComboBox*)GetDlgItem(IDC_SCHEDULER_DAY))->GetCurSel();
    m_hour = ((CComboBox*)GetDlgItem(IDC_SCHEDULER_HOUR))->GetCurSel();
    m_minute = ((CComboBox*)GetDlgItem(IDC_SCHEDULER_MINUTE))->GetCurSel();
    m_second = ((CComboBox*)GetDlgItem(IDC_SCHEDULER_SECOND))->GetCurSel();
    CDialogEx::OnOK();
}
