
// HelloClientDlg.cpp : implementation file
//

#include "stdafx.h"
#include <setupapi.h>
#include "WsOtaUpgrade.h"
#include "Win7Interface.h"
#include "WsOtaUpgradeDlg.h"

void GattConnectCallback(LPVOID pRefData, BLUETOOTH_ADDRESS *pDevice, BOOL bConnected)
{
    ods ("Connected:%d\n", bConnected);
    PostMessage(((CWsOtaUpgradeDlg *)pRefData)->m_hWnd, WM_CONNECTED, (WPARAM)bConnected, NULL); 
}

DWORD GattRequestCallback(
    LPVOID               pRefData,
    BLUETOOTH_ADDRESS   *pAddress, 
    GUID                *pGuidService, 
    DWORD               dwServiceInstance, 
    GUID                *pGuidChar, 
    DWORD               dwCharInstance,
    GUID                *pGuidDescr, 
    DWORD               security, 
    BTW_GATT_OPTYPE     Op, 
    BTW_GATT_VALUE      *pValue
)
{
    ods ("Op:%d\n", Op);
    if ((*pGuidService == guidSvcWSUpgrade) && (*pGuidChar == guidCharWSUpgradeControlPoint))
    {
        if ((Op == BTW_GATT_OPTYPE_NOTIFICATION) || (Op == BTW_GATT_OPTYPE_INDICATION))
        {
            BTW_GATT_VALUE *p = (BTW_GATT_VALUE *)malloc (sizeof (BTW_GATT_VALUE));
            if (p)
            {
                *p = *pValue;
                PostMessage(((CWsOtaUpgradeDlg *)pRefData)->m_hWnd, WM_NOTIFIED, (WPARAM)Op, (LPARAM)p);
            }
        }
    }
    return ERROR_SUCCESS;
}

CBtWin7Interface::CBtWin7Interface (BLUETOOTH_ADDRESS *bth, HMODULE hLib, LPVOID NotificationContext) 
	: CBtInterface(bth, hLib, NotificationContext, OSVERSION_WINDOWS_7)
    , m_hReg(INVALID_HANDLE_VALUE)
{
}

CBtWin7Interface::~CBtWin7Interface()
{
    if (m_hReg != INVALID_HANDLE_VALUE)
    {
        FP_BtwGattDeregister pDereg = (FP_BtwGattDeregister)GetProcAddress(m_hLib, "BtwGattDeregister");
        if (pDereg)
            (*pDereg)(m_hReg);
    }
}

BOOL CBtWin7Interface::Init()
{
    extern void BtwGuidFromGuid(GUID *pOut, const GUID *pIn);
    if (m_bSecure)
        BtwGuidFromGuid(&guidSvcWSUpgrade, &GUID_OTA_SEC_FW_UPGRADE_SERVICE);
    else
        BtwGuidFromGuid(&guidSvcWSUpgrade, &GUID_OTA_FW_UPGRADE_SERVICE);

    if (m_hLib != NULL)
    {
        FP_BtwGattRegister p = (FP_BtwGattRegister)GetProcAddress(m_hLib, "BtwGattRegister");
        if (p)
        {
            DWORD rc = (*p)(&m_bth, &GattConnectCallback, NULL, &GattRequestCallback, m_NotificationContext, &m_hReg);
            return (rc == ERROR_SUCCESS);
        }
    }
    return FALSE;
}

BOOL CBtWin7Interface::SendWsUpgradeCommand(BYTE Command)
{
    ods("+%S\n", __FUNCTIONW__);
    BTW_GATT_VALUE value = {1, Command};
    DWORD dwResult = 0;
    FP_BtwGattWriteCharacteristic pWriteChar = (FP_BtwGattWriteCharacteristic)GetProcAddress(m_hLib, "BtwGattWriteCharacteristic");
    if (pWriteChar)
    {
        dwResult = (*pWriteChar)(m_hReg, &m_bth, &guidSvcWSUpgrade, 0, &guidCharWSUpgradeControlPoint, 0, 0, &value, TRUE, this);
        ods("-%S %d\n", __FUNCTIONW__, dwResult);
        return (dwResult == ERROR_SUCCESS);
    }
    ods("-%S %d\n", __FUNCTIONW__, dwResult);
    return FALSE;
}

BOOL CBtWin7Interface::SendWsUpgradeCommand(BYTE Command, USHORT sParam)
{
    ods("+%S\n", __FUNCTIONW__);
    BTW_GATT_VALUE value = {3, Command, (BYTE)(sParam & 0xff), (BYTE)((sParam >> 8) & 0xff)};
    DWORD dwResult = 0;
    FP_BtwGattWriteCharacteristic pWriteChar = (FP_BtwGattWriteCharacteristic)GetProcAddress(m_hLib, "BtwGattWriteCharacteristic");
    if (pWriteChar)
    {
        dwResult = (*pWriteChar)(m_hReg, &m_bth, &guidSvcWSUpgrade, 0, &guidCharWSUpgradeControlPoint, 0, 0, &value, TRUE, this);
        ods("-%S %d\n", __FUNCTIONW__, dwResult);
        return (dwResult == ERROR_SUCCESS);
    }
    ods("-%S %d\n", __FUNCTIONW__, dwResult);
    return FALSE;
}

BOOL CBtWin7Interface::SendWsUpgradeCommand(BYTE Command, ULONG lParam)
{
    ods("+%S\n", __FUNCTIONW__);
    BTW_GATT_VALUE value = {5, Command, lParam & 0xff, (lParam >> 8) & 0xff, (lParam >> 16) & 0xff, (lParam >> 24) & 0xff};
    DWORD dwResult = 0;
    FP_BtwGattWriteCharacteristic pWriteChar = (FP_BtwGattWriteCharacteristic)GetProcAddress(m_hLib, "BtwGattWriteCharacteristic");
    if (pWriteChar)
    {
        dwResult = (*pWriteChar)(m_hReg, &m_bth, &guidSvcWSUpgrade, 0, &guidCharWSUpgradeControlPoint, 0, 0, &value, TRUE, this);
        ods("-%S %d\n", __FUNCTIONW__, dwResult);
        return (dwResult == ERROR_SUCCESS);
    }
    ods("-%S %d\n", __FUNCTIONW__, dwResult);
    return FALSE;
}

BOOL CBtWin7Interface::SendWsUpgradeData(BYTE *Data, DWORD len)
{
    ods("+%S\n", __FUNCTIONW__);
    BTW_GATT_VALUE value;

    if (len > sizeof (value.value))
    {
        ods("-%S data too long\n", __FUNCTIONW__);
        return (FALSE);
    }
    value.len = (USHORT)len;
    memcpy (value.value, Data, len);

    DWORD dwResult = 0;
    FP_BtwGattWriteCharacteristic pWriteChar = (FP_BtwGattWriteCharacteristic)GetProcAddress(m_hLib, "BtwGattWriteCharacteristic");
    if (pWriteChar)
    {
        dwResult = (*pWriteChar)(m_hReg, &m_bth, &guidSvcWSUpgrade, 0, &guidCharWSUpgradeData, 0, 0, &value, TRUE, this);
        ods("-%S %d\n", __FUNCTIONW__, dwResult);
        return (dwResult == ERROR_SUCCESS);
    }
    ods("-%S %d\n", __FUNCTIONW__, dwResult);
    return FALSE;
}


BOOL CBtWin7Interface::GetDescriptorValue(USHORT *Value)
{
    ods("+%S\n", __FUNCTIONW__);
    DWORD dwResult;
    BTW_GATT_VALUE value = {0};
    FP_BtwGattReadDescriptor pReadDescr = (FP_BtwGattReadDescriptor)GetProcAddress(m_hLib, "BtwGattReadDescriptor");
    if (pReadDescr)
    {
        value.value[0] = 0;
        value.value[1] = 0;
        value.len = 2;

        dwResult = (*pReadDescr)(m_hReg, &m_bth, &guidSvcWSUpgrade, 0, &guidCharWSUpgradeControlPoint, 0, &guidClntConfigDesc, 0, &value, TRUE, NULL);
        if (dwResult == ERROR_SUCCESS)
        {
            *Value = value.value[0] + (value.value[1] << 8);
            ods("-%S %d\n", __FUNCTIONW__, dwResult);
            return TRUE;
        }
    }
    ods("-%S %d\n", __FUNCTIONW__, dwResult);
    return FALSE;
}

BOOL CBtWin7Interface::SetDescriptorValue(USHORT Value)
{
    ods("+%S\n", __FUNCTIONW__);
    BTW_GATT_VALUE value = {0};
    value.len = 2;
    value.value[0] = (BYTE)(Value & 0xff);
    value.value[1] = (BYTE)((Value >> 8) &0xff);

    FP_BtwGattWriteDescriptor pWriteDescr = (FP_BtwGattWriteDescriptor)GetProcAddress(m_hLib, "BtwGattWriteDescriptor");
    if (pWriteDescr)
    {
        DWORD dwResult = (*pWriteDescr)(m_hReg, &m_bth, &guidSvcWSUpgrade, 0, &guidCharWSUpgradeControlPoint, 0, &guidClntConfigDesc, 0, &value, TRUE, NULL);
        ods("-%S %d\n", __FUNCTIONW__, dwResult);
        return (dwResult == ERROR_SUCCESS);
    }
    ods("-%S\n", __FUNCTIONW__);
    return FALSE;
}

