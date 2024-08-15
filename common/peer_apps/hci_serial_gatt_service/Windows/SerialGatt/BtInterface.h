
// BtInterface.h : header file
//

#pragma once


typedef enum {
    OSVERSION_WINDOWS_7 = 0,
    OSVERSION_WINDOWS_8,
    OSVERSION_WINDOWS_10
}tOSVersion;


class CBtInterface
{
public:
    CBtInterface(BLUETOOTH_ADDRESS *bth, HMODULE hLib, LPVOID NotificationContext, tOSVersion osversion)
    {
        m_bth = *bth;
        m_hLib = hLib;
        m_NotificationContext = NotificationContext;
        m_bWin8 = (osversion == OSVERSION_WINDOWS_8) ? TRUE : FALSE;
        m_bWin10 = (osversion == OSVERSION_WINDOWS_10) ? TRUE : FALSE;
    };

    virtual BOOL Init(GUID uuidServ) = NULL;
    virtual BOOL GetDescriptorValue(GUID  uuidChar, USHORT inx, USHORT uuidDescr, BTW_GATT_VALUE *pValue) = NULL;
    virtual BOOL SetDescriptorValue(GUID uuidChar, USHORT inx, USHORT uuidDescr, BTW_GATT_VALUE *pValue) = NULL;
    virtual BOOL ReadCharacteristic(GUID uuidChar, USHORT inx, BTW_GATT_VALUE *pValue) = NULL;
    virtual BOOL WriteCharacteristic(GUID uuidChar, USHORT inx, BTW_GATT_VALUE *pValue) = NULL;
    BOOL CBtInterface::GetDescriptorClientConfigValue(GUID uuidChar, USHORT inx, USHORT *Value);
    BOOL CBtInterface::SetDescriptorClientConfigValue(GUID uuidChar, USHORT inx, USHORT Value);

    BLUETOOTH_ADDRESS m_bth;
    HMODULE m_hLib;
    LPVOID m_NotificationContext;
    GUID m_guid;
    BOOL m_bWin8;
    BOOL m_bWin10;
    tOSVersion m_osversion;
}; 

