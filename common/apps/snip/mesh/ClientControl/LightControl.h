#pragma once

#include "ClientControl.h"
#include "ControlComm.h"

// CLightControl dialog

class CLightControl : public CPropertyPage
{
	DECLARE_DYNAMIC(CLightControl)

public:
	CLightControl();
	virtual ~CLightControl();
    void OnCancel();
    void SetDlgItemHex(DWORD id, DWORD val);
    DWORD GetHexValue(DWORD id, LPBYTE buf, DWORD buf_size);
    DWORD GetHexValueInt(DWORD id);
    void ProcessData(DWORD opcode, LPBYTE p_data, DWORD len);
    void ProcessEvent(LPBYTE p_data, DWORD len);
    CListBox *m_trace;

    BOOL        m_scan_started;
    BOOL        m_bConnecting;
    BOOL        m_bScanning;
    uint8_t     m_dfuMethod;
    BOOL m_bConnected;

    void DisplayCurrentGroup();
    void ProvisionCompleted();

    WCHAR   m_szCurrentGroup[80];

    BOOL    m_fw_download_active;
    BYTE    m_received_evt[261];
    DWORD   m_received_evt_len;
    HANDLE  m_hHciEvent;
    HANDLE m_event;
#define STATE_IDLE                          0
#define STATE_LOCAL_GET_COMPOSITION_DATA    1
#define STATE_LOCAL_ADD_APPLICATION_KEY     2
#define STATE_LOCAL_BIND_MODELS             3
#define STATE_REMOTE_GET_COMPOSITION_DATA   4
#define STATE_REMOTE_ADD_APPLICATION_KEY    5
#define STATE_REMOTE_BIND_MODELS            6
    int m_state;

    CProgressCtrl m_Progress;
    LRESULT OnProgress(WPARAM completed, LPARAM total);
//    WSDownloader *m_pDownloader;
    LPBYTE m_pPatch;
    DWORD m_dwPatchSize;

    void ProcessUnprovisionedDevice(uint8_t *p_uuid, uint16_t oob, uint8_t *name, uint8_t name_len);
    void LinkStatus(uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt);
    void ProcessUnprovisionedDevice(LPBYTE p_data, DWORD len);
    void ProcessVendorSpecificData(LPBYTE p_data, DWORD len);
    // Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_LIGHT_CONTROL };
#endif

    int GetBaudRateSelection();

public:
    void OnOtaUpgradeContinue();
    uint32_t GetDfuImageSize();
    void GetDfuImageChunk(uint8_t *p_data, uint32_t offset, uint16_t data_len);

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()

    LRESULT OnWsUpgradeCtrlPoint(WPARAM op, LPARAM lparam);
    LRESULT OnMeshDeviceConnected(WPARAM Instance, LPARAM lparam);
    LRESULT OnMeshDeviceDisconnected(WPARAM Instance, LPARAM lparam);
    LRESULT OnMeshDeviceAdvReport(WPARAM Instance, LPARAM lparam);
    LRESULT OnMeshDeviceConnect(WPARAM state, LPARAM param);
    LRESULT OnMeshDeviceDisconnect(WPARAM state, LPARAM param);

public:
    virtual BOOL OnSetActive();
    afx_msg void OnClose();
    afx_msg void OnCbnSelchangeComPort();
    afx_msg void OnBnClickedClearTrace();
    afx_msg void OnBnClickedScanUnprovisioned();
    afx_msg void OnBnClickedDownload();
    afx_msg void OnBnClickedBrowse();
    afx_msg void OnBnClickedProvision();
    afx_msg void OnBnClickedNetworkCreate();
    afx_msg void OnBnClickedNetworkDelete();
    afx_msg void OnBnClickedNetworkOpen();
    afx_msg void OnBnClickedNetworkClose();
    afx_msg void OnBnClickedGroupCreate();
    afx_msg void OnBnClickedGroupDelete();
    afx_msg void OnBnClickedNodeReset();
    afx_msg void OnSelchangeNetwork();
    afx_msg void OnBnClickedConfigureNewName();
    afx_msg void OnSelchangeCurrentGroup();
    afx_msg void OnBnClickedMoveToGroup();
    afx_msg void OnBnClickedConfigurePub();
    afx_msg void OnBnClickedOnOffGet();
    afx_msg void OnBnClickedOnOffSet();
    afx_msg void OnBnClickedLevelGet();
    afx_msg void OnBnClickedLevelSet();
    afx_msg void OnBnClickedLightHslGet();
    afx_msg void OnBnClickedLightHslSet();
    afx_msg void OnBnClickedVsData();
    afx_msg void OnBnClickedLightCtlGet();
    afx_msg void OnBnClickedLightCtlSet();
    afx_msg void OnBnClickedLightnessGet();
    afx_msg void OnBnClickedLightnessSet();

    afx_msg void OnBnClickedOtaUpgradeStart();
    afx_msg void OnBnClickedConnectdisconnect();
    afx_msg void OnBnClickedIdentify();
    afx_msg void OnBnClickedReconfigure();
    afx_msg void OnBnClickedBrowseOta();
    afx_msg void OnCbnSelchangeConfigureControlDevice();
    afx_msg void OnBnClickedNetworkImport();
    afx_msg void OnBnClickedNetworkExport();
    afx_msg void OnBnClickedOtaUpgradeStop();
    afx_msg void OnBnClickedGetComponentInfo();
    afx_msg void OnBnClickedOtaUpgradeStatus();
    afx_msg void OnBnClickedSensorGet();
    afx_msg void OnCbnSelchangeControlDevice();
    afx_msg void OnBnClickedSensorConfigure();
    virtual BOOL OnInitDialog();
};

extern CClientControlApp theApp;
