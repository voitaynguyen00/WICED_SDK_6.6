// LightControl.cpp : implementation file
//

#include "stdafx.h"
#include "LightControl.h"
#include "ClientControlDlg.h"
#include "SensorConfig.h"
#include "afxdialogex.h"
#include "resource.h"
#include "wiced_mesh_client.h"
#include "hci_control_api.h"
#include "wiced_bt_mesh_model_defs.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_bt_mesh_db.h"

int provision_test = 0;
DWORD provision_test_scan_unprovisioned_time;
DWORD provision_test_connect_unprovisioned_time;
DWORD provision_test_provision_start_time;
DWORD provision_test_connect_provisioned_time;
DWORD provision_test_config_start_time;
DWORD provision_test_reset_time;
BOOL  provision_test_bScanning;


char *log_filename = "trace.txt";  // if you add full path make sure that directory exists, otherwise it will crash
// #define MESH_AUTOMATION_ENABLED TRUE
#if defined(MESH_AUTOMATION_ENABLED) && (MESH_AUTOMATION_ENABLED == TRUE)
#include "mesh_client_script.h"
#endif

static WCHAR *szDeviceType[] =
{
    L"Unknown",
    L"On/Off Client",
    L"Level Client",
    L"On/Off Server",
    L"Level Server",
    L"Dimmable Light",
    L"Power Outlet",
    L"HSL Light",
    L"CTL Light",
    L"XYL Light",
    L"Vendor Specific",
};

typedef struct
{
    int is_gatt_proxy;
    int is_friend;
    int is_relay;
    int send_net_beacon;
    int relay_xmit_count;
    int relay_xmit_interval;
    int default_ttl;
    int net_xmit_count;
    int net_xmit_interval;
    int publish_credential_flag;       ///< Value of the Friendship Credential Flag
    int publish_ttl;                   ///< Default TTL value for the outgoing messages
    int publish_retransmit_count;      ///< Number of retransmissions for each published message
    int publish_retransmit_interval;   ///< Interval in milliseconds between retransmissions
} device_config_params_t;

device_config_params_t DeviceConfig = { 1, 1, 1, 1, 3, 100, 63, 3, 100, 0, 63, 0, 500 };

void network_opened(uint8_t status);
/*extern "C" */ void unprovisioned_device(uint8_t *uuid, uint16_t oob, uint8_t *name_len, uint8_t name);
/*extern "C" */ void link_status(uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt);
/*extern "C" */ void node_connect_status(uint8_t is_connected, char *p_device_name);
/*extern "C" */ void provision_status(uint8_t status, uint8_t *p_uuid);
/*extern "C" */ void database_changed(char *mesh_name);
/*extern "C" */ void component_info_status(uint8_t status, char *component_name, char *component_info);
/*extern "C" */ void onoff_status(const char *device_name, uint8_t present, uint8_t target, uint32_t remaining_time);
/*extern "C" */ void level_status(const char *device_name, int16_t present, int16_t target, uint32_t remaining_time);
/*extern "C" */ void lightness_status(const char *device_name, uint16_t present, uint16_t target, uint32_t remaining_time);
/*extern "C" */ void hsl_status(const char *device_name, uint16_t lightness, uint16_t hue, uint16_t saturation, uint32_t remaining_time);
/*extern "C" */ void ctl_status(const char *device_name, uint16_t present_lightness, uint16_t present_temperature, uint16_t target_lightness, uint16_t target_temperature, uint32_t remaining_time);
/*extern "C" */ void sensor_status(const char *device_name, int property_id, uint8_t value_len, uint8_t *value);
/*extern "C" */ void fw_distribution_status(uint8_t status, int current_block_num, int total_blocks);

#define DFU_METHOD_PROXY_TO_ALL                     0
#define DFU_METHOD_PROXY_TO_DEVICE                  1
#define DFU_METHOD_APP_TO_ALL                       2
#define DFU_METHOD_APP_TO_DEVICE                    3

WCHAR *dfuMethods[] = {
    L"Proxy DFU to all",
    L"Proxy DFU to device",
    L"App DFU to all",
    L"App DFU to device",
};

extern wiced_bool_t mesh_adv_scanner_open();
extern void mesh_adv_scanner_close(void);
extern "C" void mesh_client_advert_report(uint8_t *bd_addr, uint8_t addr_type, int8_t rssi, uint8_t *adv_data);

char provisioner_uuid[50];

mesh_client_init_t mesh_client_init_callbacks =
{
    unprovisioned_device,
    provision_status,
    link_status,
    node_connect_status,
    database_changed,
    onoff_status,
    level_status,
    lightness_status,
    hsl_status,
    ctl_status,
    sensor_status,
};

extern void TraceHciPkt(BYTE type, BYTE *buffer, USHORT length);
extern void Log(WCHAR *fmt, ...);
extern int FwDownload(char *sHCDFileName);
void FwDownloadProcessEvent(LPBYTE p_data, DWORD len);
extern "C" void wiced_hci_process_data(uint16_t opcode, uint8_t *p_buffer, uint16_t len);

// CLightControl dialog

IMPLEMENT_DYNAMIC(CLightControl, CPropertyPage)

CLightControl::CLightControl()
	: CPropertyPage(IDD_LIGHT_CONTROL)
{
    m_trace = NULL;
    m_szCurrentGroup[0] = 0;
    m_fw_download_active = FALSE;
    m_hHciEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    m_bScanning = FALSE;

    FILE *fp = fopen("NetParameters.bin", "rb");
    if (fp)
    {
        fread(&DeviceConfig, 1, sizeof(DeviceConfig), fp);
        fclose(fp);
    }
}

CLightControl::~CLightControl()
{
}

void CLightControl::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CLightControl, CPropertyPage)
    ON_WM_CLOSE()
    ON_CBN_SELCHANGE(IDC_COM_PORT, &CLightControl::OnCbnSelchangeComPort)
    ON_CBN_SELCHANGE(IDC_COM_BAUD, &CLightControl::OnCbnSelchangeComPort)
    ON_BN_CLICKED(IDC_CLEAR_TRACE, &CLightControl::OnBnClickedClearTrace)
    ON_BN_CLICKED(IDC_SCAN_UNPROVISIONED, &CLightControl::OnBnClickedScanUnprovisioned)
    ON_BN_CLICKED(IDC_DOWNLOAD, &CLightControl::OnBnClickedDownload)
    ON_BN_CLICKED(IDC_BROWSE, &CLightControl::OnBnClickedBrowse)
    ON_BN_CLICKED(IDC_PROVISION, &CLightControl::OnBnClickedProvision)
    ON_BN_CLICKED(IDC_NETWORK_CREATE, &CLightControl::OnBnClickedNetworkCreate)
    ON_BN_CLICKED(IDC_NETWORK_DELETE, &CLightControl::OnBnClickedNetworkDelete)
    ON_BN_CLICKED(IDC_NETWORK_OPEN, &CLightControl::OnBnClickedNetworkOpen)
    ON_BN_CLICKED(IDC_NETWORK_CLOSE, &CLightControl::OnBnClickedNetworkClose)
    ON_BN_CLICKED(IDC_GROUP_CREATE, &CLightControl::OnBnClickedGroupCreate)
    ON_BN_CLICKED(IDC_GROUP_DELETE, &CLightControl::OnBnClickedGroupDelete)
    ON_BN_CLICKED(IDC_NODE_RESET, &CLightControl::OnBnClickedNodeReset)
    ON_CBN_SELCHANGE(IDC_NETWORK, &CLightControl::OnSelchangeNetwork)
    ON_BN_CLICKED(IDC_CONFIGURE_NEW_NAME, &CLightControl::OnBnClickedConfigureNewName)
    ON_CBN_SELCHANGE(IDC_CURRENT_GROUP, &CLightControl::OnSelchangeCurrentGroup)
    ON_BN_CLICKED(IDC_CONFIGURE_MOVE, &CLightControl::OnBnClickedMoveToGroup)
    ON_BN_CLICKED(IDC_CONFIGURE_PUB, &CLightControl::OnBnClickedConfigurePub)
//    ON_BN_CLICKED(IDC_CONNECTDISCONNECT, &CLightControl::OnBnClickedConnectdisconnect)
    ON_BN_CLICKED(IDC_IDENTIFY, &CLightControl::OnBnClickedIdentify)
    ON_BN_CLICKED(IDC_ON_OFF_GET, &CLightControl::OnBnClickedOnOffGet)
    ON_BN_CLICKED(IDC_ON_OFF_SET, &CLightControl::OnBnClickedOnOffSet)
    ON_BN_CLICKED(IDC_LEVEL_GET, &CLightControl::OnBnClickedLevelGet)
    ON_BN_CLICKED(IDC_LEVEL_SET, &CLightControl::OnBnClickedLevelSet)
    ON_BN_CLICKED(IDC_LIGHT_HSL_GET, &CLightControl::OnBnClickedLightHslGet)
    ON_BN_CLICKED(IDC_LIGHT_HSL_SET, &CLightControl::OnBnClickedLightHslSet)
    ON_BN_CLICKED(IDC_VS_DATA, &CLightControl::OnBnClickedVsData)
    ON_BN_CLICKED(IDC_LIGHT_CTL_GET, &CLightControl::OnBnClickedLightCtlGet)
    ON_BN_CLICKED(IDC_LIGHT_CTL_SET, &CLightControl::OnBnClickedLightCtlSet)
    ON_BN_CLICKED(IDC_LIGHTNESS_GET, &CLightControl::OnBnClickedLightnessGet)
    ON_BN_CLICKED(IDC_LIGHTNESS_SET, &CLightControl::OnBnClickedLightnessSet)
    ON_CBN_SELCHANGE(IDC_CONFIGURE_CONTROL_DEVICE, &CLightControl::OnCbnSelchangeConfigureControlDevice)
    ON_BN_CLICKED(IDC_NETWORK_IMPORT, &CLightControl::OnBnClickedNetworkImport)
    ON_BN_CLICKED(IDC_NETWORK_EXPORT, &CLightControl::OnBnClickedNetworkExport)
    ON_BN_CLICKED(IDC_BROWSE_OTA, &CLightControl::OnBnClickedBrowseOta)
    ON_BN_CLICKED(IDC_OTA_UPGRADE_STOP, &CLightControl::OnBnClickedOtaUpgradeStop)
    ON_BN_CLICKED(IDC_OTA_UPGRADE_STATUS, &CLightControl::OnBnClickedOtaUpgradeStatus)
    ON_BN_CLICKED(IDC_SENSOR_GET, &CLightControl::OnBnClickedSensorGet)
    ON_CBN_SELCHANGE(IDC_CONTROL_DEVICE, &CLightControl::OnCbnSelchangeControlDevice)
    ON_BN_CLICKED(IDC_SENSOR_CONFIGURE, &CLightControl::OnBnClickedSensorConfigure)
END_MESSAGE_MAP()

BOOL CLightControl::OnSetActive()
{
    CPropertyPage::OnSetActive();

    m_trace = (CListBox *)GetDlgItem(IDC_TRACE);
    CClientDialog *pSheet = (CClientDialog *)theApp.m_pMainWnd;
    pSheet->m_active_page = 1;

    CComboBox *m_cbCom = (CComboBox *)GetDlgItem(IDC_COM_PORT);
    if (m_cbCom->GetCount() == 0)
    {
        for (int i = 0; i < 128 && aComPorts[i] != 0; i++)
        {
            WCHAR buf[20];
            wsprintf(buf, L"COM%d", aComPorts[i]);
            m_cbCom->SetItemData(m_cbCom->AddString(buf), aComPorts[i]);
        }
        CComboBox *m_cbBaud = (CComboBox *)GetDlgItem(IDC_COM_BAUD);
        for (int i = 0; i < sizeof(as32BaudRate) / sizeof(as32BaudRate[0]); i++)
        {
            WCHAR acBaud[10];
            wsprintf(acBaud, L"%d", as32BaudRate[i]);

            m_cbBaud->SetItemData(m_cbBaud->AddString(acBaud), i);
        }
        m_cbBaud->SetCurSel(FindBaudRateIndex(3000000));

        if (m_ComHelper == NULL)
        {
            m_ComHelper = new ComHelper(m_hWnd);
        }
    }

    if (ComPortSelected > 0)
        ((CComboBox *)GetDlgItem(IDC_COM_PORT))->SetCurSel(ComPortSelected);

    if (BaudRateSelected > 0)
        ((CComboBox *)GetDlgItem(IDC_COM_BAUD))->SetCurSel(BaudRateSelected);

    SetDlgItemHex(IDC_IDENTITY_DURATION, 1);

    CClientControlDlg *pMainDlg = &pSheet->pageMain;

    CString sHCDFileName = theApp.GetProfileString(L"LightControl", L"HCDFile", L"");
    SetDlgItemText(IDC_FILENAME, sHCDFileName);

    WCHAR szHostName[128];
    DWORD dw = 128;
    GetComputerName(szHostName, &dw);

    SetDlgItemText(IDC_PROVISIONER, szHostName);

    CString sProvisionerUuid = theApp.GetProfileString(L"LightControl", L"ProvisionerUuid", L"");
    if (sProvisionerUuid == "")
    {
        WCHAR sProvisionerUuid[33] = { 0 };
        for (int i = 0; i < 8; i++)
            sprintf(&provisioner_uuid[i * 4], "%04X", rand());
        MultiByteToWideChar(CP_UTF8, 0, provisioner_uuid, 32, sProvisionerUuid, 32);
        theApp.WriteProfileStringW(L"LightControl", L"ProvisionerUuid", sProvisionerUuid);
    }
    else
    {
        WideCharToMultiByte(CP_UTF8, 0, sProvisionerUuid.GetBuffer(), -1, provisioner_uuid, 33, 0, FALSE);
    }

    ((CComboBox *)GetDlgItem(IDC_NETWORK))->ResetContent();

    char *p_networks = mesh_client_get_all_networks();
    char *p = p_networks;
    WCHAR szNetwork[80];
    int num_networks = 0;
    while (p != NULL && *p != NULL)
    {
        MultiByteToWideChar(CP_UTF8, 0, p, strlen(p) + 1, szNetwork, sizeof(szNetwork) / sizeof(WCHAR));
        ((CComboBox *)GetDlgItem(IDC_NETWORK))->AddString(szNetwork);
        p += strlen(p) + 1;
        num_networks++;
    }
    if (num_networks)
    {
        ((CComboBox *)GetDlgItem(IDC_NETWORK))->SetCurSel(0);
    }

    ((CButton *)GetDlgItem(IDC_GATT_PROXY))->SetCheck(DeviceConfig.is_gatt_proxy);
    ((CButton *)GetDlgItem(IDC_FRIEND))->SetCheck(DeviceConfig.is_friend);
    ((CButton *)GetDlgItem(IDC_RELAY))->SetCheck(DeviceConfig.is_relay);
    ((CButton *)GetDlgItem(IDC_NET_BEACON))->SetCheck(DeviceConfig.send_net_beacon);
    SetDlgItemInt(IDC_RELAY_TRANSMIT_COUNT, DeviceConfig.relay_xmit_count);
    SetDlgItemInt(IDC_RELAY_TRANSMIT_INTERVAL, DeviceConfig.relay_xmit_interval);
    SetDlgItemInt(IDC_DEFAULT_TTL, DeviceConfig.default_ttl);
    SetDlgItemInt(IDC_NETWORK_TRANSMIT_COUNT, DeviceConfig.net_xmit_count);
    SetDlgItemInt(IDC_NETWORK_TRANSMIT_INTERVAL, DeviceConfig.net_xmit_interval);

    ((CComboBox *)GetDlgItem(IDC_MODEL_PUB_CREDENTIAL_FLAG))->SetCurSel(DeviceConfig.publish_credential_flag);
    SetDlgItemInt(IDC_PUBLISH_TTL, DeviceConfig.publish_ttl);
    SetDlgItemInt(IDC_MODEL_PUB_RETRANSMIT_COUNT, DeviceConfig.publish_retransmit_count);
    SetDlgItemInt(IDC_MODEL_PUB_RETRANSMIT_INTERVAL, DeviceConfig.publish_retransmit_interval);

    mesh_client_init(&mesh_client_init_callbacks);
    free(p_networks);
    return TRUE;  // return TRUE unless you set the focus to a control
}

void CLightControl::OnClose()
{
    mesh_client_network_close();
    m_ComHelper->ClosePort();
    Sleep(1000);
    CPropertyPage::OnClose();
    // CDialogEx::OnClose();
}

void CLightControl::OnCancel()
{
    mesh_client_network_close();
    m_ComHelper->ClosePort();
    Sleep(1000);
    CPropertyPage::OnCancel();
    // CDialogEx::OnClose();
}

void CLightControl::OnCbnSelchangeComPort()
{
    CComboBox *m_cbCom = (CComboBox *)GetDlgItem(IDC_COM_PORT);

    ComPortSelected = m_cbCom->GetCurSel();
    BaudRateSelected = ((CComboBox *)GetDlgItem(IDC_COM_BAUD))->GetCurSel();

    ComPort = m_cbCom->GetItemData(m_cbCom->GetCurSel());
    int baud = GetBaudRateSelection();

    if (ComPort >= 0)
    {
        m_ComHelper->ClosePort();
        Sleep(1000);

        m_ComHelper->OpenPort(ComPort, baud);
    }
}

int CLightControl::GetBaudRateSelection()
{
    CComboBox *m_cbBaud = (CComboBox *)GetDlgItem(IDC_COM_BAUD);
    int select = m_cbBaud->GetItemData(m_cbBaud->GetCurSel());

    if (select >= 0)
        return as32BaudRate[select];

    return as32BaudRate[0];
}

void CLightControl::OnBnClickedClearTrace()
{
    m_trace->ResetContent();
}

// CLightControl message handlers
void CLightControl::OnBnClickedDownload()
{
    char sHCDFileName[MAX_PATH] = { 0 };
    GetDlgItemTextA(m_hWnd, IDC_FILENAME, sHCDFileName, MAX_PATH);

    if (sHCDFileName[0] == 0)
    {
        Log(L"Specify valid configuration file and Address");
        return;
    }

    FILE *fHCD = NULL;
    LONG  nVeryFirstAddress = 0;

    WCHAR name[512] = { 0 };
    MultiByteToWideChar(CP_ACP, 0, (const char *)sHCDFileName, strlen(sHCDFileName), name, sizeof(name));
    BOOL rc = theApp.WriteProfileString(L"LightControl", L"HCDFile", name);

    m_fw_download_active = TRUE;
    rc = FwDownload(sHCDFileName);
    m_fw_download_active = FALSE;
}

void CLightControl::OnBnClickedBrowse()
{
    static TCHAR BASED_CODE szFilter[] = _T("HCD Files (*.hcd)|*.hcd|");

    CFileDialog dlgFile(TRUE, NULL, NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, szFilter);
    if (dlgFile.DoModal() == IDOK)
    {
        SetDlgItemText(IDC_FILENAME, dlgFile.GetPathName());
    }
}

DWORD CLightControl::GetHexValueInt(DWORD id)
{
    DWORD ret = 0;
    BYTE buf[32];
    DWORD len = GetHexValue(id, buf, sizeof(buf));
    for (DWORD i = 0; i<len; i++)
    {
        ret = (ret << 8) + buf[i];
    }
    return ret;
}

void CLightControl::SetDlgItemHex(DWORD id, DWORD val)
{
    WCHAR buf[10];
    wsprintf(buf, L"%x", val);
    SetDlgItemText(id, buf);
}

DWORD CLightControl::GetHexValue(DWORD id, LPBYTE buf, DWORD buf_size)
{
    char szbuf[1300];
    char *psz = szbuf;
    BYTE *pbuf = buf;
    DWORD res = 0;

    memset(buf, 0, buf_size);

    GetDlgItemTextA(m_hWnd, id, szbuf, sizeof(szbuf));
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

void CLightControl::ProcessEvent(LPBYTE p_data, DWORD len)
{
    if (m_fw_download_active)
        FwDownloadProcessEvent(p_data, len);
    else
    {
        m_received_evt_len = len;
        memcpy(m_received_evt, p_data, len);
        SetEvent(m_hHciEvent);
    }
}

void CLightControl::ProcessData(DWORD opcode, LPBYTE p_data, DWORD len)
{
    if (opcode == HCI_CONTROL_EVENT_WICED_TRACE)
    {
        if (len >= 2)
        {
            if ((len > 2) && (p_data[len - 2] == '\n'))
            {
                p_data[len - 2] = 0;
                len--;
            }
            TraceHciPkt(0, p_data, (USHORT)len);
        }
        //MultiByteToWideChar(CP_ACP, 0, (const char *)p_data, len, trace, sizeof(trace) / sizeof(WCHAR));
        //m_trace->SetCurSel(m_trace->AddString(trace));
        return;
    }
    else if (opcode == HCI_CONTROL_EVENT_HCI_TRACE)
    {
        TraceHciPkt(p_data[0] + 1, &p_data[1], (USHORT)(len - 1));
        return;
    }
    else if (opcode == HCI_CONTROL_MESH_EVENT_VENDOR_STATUS)
    {
        ProcessVendorSpecificData(p_data, len);
    }
    else
    {
        wiced_hci_process_data((uint16_t)opcode, p_data, (uint16_t)len);
    }
}

void CLightControl::ProcessUnprovisionedDevice(uint8_t *p_uuid, uint16_t oob, uint8_t *name, uint8_t name_len)
{
    WCHAR buf[512];
    WCHAR uuid[50] = { 0 };
    WCHAR szName[31] = { 0 };

    for (int i = 0; i < 16; i++)
        wsprintf(&uuid[wcslen(uuid)], L"%02x ", p_uuid[i]);
    SetDlgItemText(IDC_PROVISION_UUID, uuid);

    if (name_len != 0)
    {
        MultiByteToWideChar(CP_UTF8, 0, (char *)name, name_len, szName, sizeof(szName) / sizeof(WCHAR));
        szName[name_len] = 0;
        SetDlgItemText(IDC_NODE_NAME, szName);
    }
    else
        SetDlgItemText(IDC_NODE_NAME, L"");

    wcscpy(buf, L"Unprovisioned Device UUID:");
    wcscat(buf, uuid);

    wsprintf(&buf[wcslen(buf)], L"OOB:%x %s", oob, szName);

    m_trace->SetCurSel(m_trace->AddString(buf));

    if (m_bScanning && provision_test && p_uuid[12] == 0x11 && p_uuid[13] == 0x11 && p_uuid[14] == 0x11 && p_uuid[15] == 0x11)
    {
        PostMessage(WM_COMMAND, IDC_SCAN_UNPROVISIONED, 0);
        PostMessage(WM_COMMAND, IDC_PROVISION, 0);
    }
}

void CLightControl::LinkStatus(uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt)
{
    WCHAR buf[180];

    wsprintf(&buf[0], L"Link Status:%d", is_connected);
    m_trace->SetCurSel(m_trace->AddString(buf));
    m_bConnected = is_connected;
    if (m_bConnected)
        SetDlgItemText(IDC_CONNECTDISCONNECT, L"Disconnect");
    else
        SetDlgItemText(IDC_CONNECTDISCONNECT, L"Connect");
}

void CLightControl::OnBnClickedScanUnprovisioned()
{
    if (!m_bScanning)
    {
        provision_test_scan_unprovisioned_time = GetTickCount();
        SetDlgItemText(IDC_SCAN_UNPROVISIONED, L"Stop Scanning");
        m_bScanning = TRUE;
    }
    else
    {
        SetDlgItemText(IDC_SCAN_UNPROVISIONED, L"Scan Unprovisioned");
        m_bScanning = FALSE;
    }
    WCHAR buf[128];
    wsprintf(buf, L"scan unprovisioned:%d", m_bScanning);
    m_trace->SetCurSel(m_trace->AddString(buf));
    mesh_client_scan_unprovisioned(m_bScanning);
}

void CLightControl::OnBnClickedProvision()
{
    uint8_t identify_duration = (BYTE)GetHexValueInt(IDC_IDENTITY_DURATION);

    uint8_t uuid[16];
    GetHexValue(IDC_PROVISION_UUID, uuid, 16);

    char group_name[80];
    GetDlgItemTextA(m_hWnd, IDC_CURRENT_GROUP, group_name, sizeof(group_name));

    char node_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NODE_NAME, node_name, sizeof(node_name));

    DeviceConfig.is_gatt_proxy = (BYTE)((CButton *)GetDlgItem(IDC_GATT_PROXY))->GetCheck();
    DeviceConfig.is_friend = (BYTE)((CButton *)GetDlgItem(IDC_FRIEND))->GetCheck();
    DeviceConfig.is_relay = (BYTE)((CButton *)GetDlgItem(IDC_RELAY))->GetCheck();
    DeviceConfig.send_net_beacon = (BYTE)((CButton *)GetDlgItem(IDC_NET_BEACON))->GetCheck();
    DeviceConfig.relay_xmit_count = GetDlgItemInt(IDC_RELAY_TRANSMIT_COUNT);
    DeviceConfig.relay_xmit_interval = GetDlgItemInt(IDC_RELAY_TRANSMIT_INTERVAL);
    DeviceConfig.default_ttl = GetDlgItemInt(IDC_DEFAULT_TTL);
    DeviceConfig.net_xmit_count = (BYTE)GetDlgItemInt(IDC_NETWORK_TRANSMIT_COUNT);
    DeviceConfig.net_xmit_interval = (USHORT)GetDlgItemInt(IDC_NETWORK_TRANSMIT_INTERVAL);

    DeviceConfig.publish_credential_flag = (BYTE)((CComboBox *)GetDlgItem(IDC_MODEL_PUB_CREDENTIAL_FLAG))->GetCurSel();
    DeviceConfig.publish_ttl = GetDlgItemInt(IDC_PUBLISH_TTL);
    DeviceConfig.publish_retransmit_count = GetDlgItemInt(IDC_MODEL_PUB_RETRANSMIT_COUNT);
    DeviceConfig.publish_retransmit_interval = GetDlgItemInt(IDC_MODEL_PUB_RETRANSMIT_INTERVAL);

    FILE *fp = fopen("NetParameters.bin", "wb");
    if (fp)
    {
        fwrite(&DeviceConfig, 1, sizeof(DeviceConfig), fp);
        fclose(fp);
    }

    mesh_client_set_device_config(NULL, DeviceConfig.is_gatt_proxy, DeviceConfig.is_friend, DeviceConfig.is_relay, DeviceConfig.send_net_beacon, DeviceConfig.relay_xmit_count, DeviceConfig.relay_xmit_interval, DeviceConfig.default_ttl, DeviceConfig.net_xmit_count, DeviceConfig.net_xmit_interval);
    mesh_client_set_publication_config(DeviceConfig.publish_credential_flag, DeviceConfig.publish_retransmit_count, DeviceConfig.publish_retransmit_interval, DeviceConfig.publish_ttl);

    mesh_client_provision(node_name, group_name, uuid, identify_duration);
}


void CLightControl::OnBnClickedNetworkCreate()
{
    char mesh_name[80], provisioner_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NETWORK, mesh_name, sizeof(mesh_name));
    GetDlgItemTextA(m_hWnd, IDC_PROVISIONER, provisioner_name, sizeof(provisioner_name));
    if (strlen(mesh_name) == 0)
        MessageBoxA(m_hWnd, "Provide mesh name and provisioner name", "Error", MB_ICONERROR);
    else if (mesh_client_network_exists(mesh_name))
        MessageBoxA(m_hWnd, mesh_name, "Network Already Exists", MB_ICONERROR);
    else
    {
        int res = mesh_client_network_create(provisioner_name, provisioner_uuid, mesh_name);
        if (res == MESH_CLIENT_SUCCESS)
        {
            WCHAR s[80];
            MultiByteToWideChar(CP_UTF8, 0, mesh_name, strlen(mesh_name) + 1, s, sizeof(s) / sizeof(WCHAR));
            Log(L"Network %s created\n", s);
            DisplayCurrentGroup();
        }
        else
        {
            Log(L"Failed to create network:%d\n", res);
        }
    }
}

void CLightControl::OnBnClickedNetworkDelete()
{
    char mesh_name[80], provisioner_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NETWORK, mesh_name, sizeof(mesh_name));
    GetDlgItemTextA(m_hWnd, IDC_PROVISIONER, provisioner_name, sizeof(provisioner_name));
    if (strlen(mesh_name) == 0)
        MessageBoxA(m_hWnd, "Provide mesh name and provisioner name", "Error", MB_ICONERROR);
    else if (!mesh_client_network_exists(mesh_name))
        MessageBoxA(m_hWnd, mesh_name, "Network Already Exists", MB_ICONERROR);
    else
    {
        int res = mesh_client_network_delete(provisioner_name, provisioner_uuid, mesh_name);
        if (res == MESH_CLIENT_SUCCESS)
        {
            WCHAR s[80];
            MultiByteToWideChar(CP_UTF8, 0, mesh_name, strlen(mesh_name) + 1, s, sizeof(s) / sizeof(WCHAR));
            Log(L"Network %s deleted\n", s);
        }
        else
        {
            Log(L"Failed to delete network:%d\n", res);
        }
    }
}

void CLightControl::OnBnClickedNetworkOpen()
{
    char mesh_name[80], provisioner_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NETWORK, mesh_name, sizeof(mesh_name));
    GetDlgItemTextA(m_hWnd, IDC_PROVISIONER, provisioner_name, sizeof(provisioner_name));
    if (mesh_client_network_open(provisioner_name, provisioner_uuid, mesh_name, network_opened) != MESH_CLIENT_SUCCESS)
    {
        MessageBoxA(m_hWnd, mesh_name, "Network Does Not Exists", MB_ICONERROR);
        return;
    }
    WCHAR szGroup[80];
    MultiByteToWideChar(CP_UTF8, 0, mesh_name, strlen(mesh_name) + 1, szGroup, sizeof(szGroup) / sizeof(WCHAR));
    wcscpy(m_szCurrentGroup, szGroup);

    DisplayCurrentGroup();
}

void CLightControl::OnSelchangeCurrentGroup()
{
    CComboBox *p_current_group = (CComboBox *)GetDlgItem(IDC_CURRENT_GROUP);
    int sel = p_current_group->GetCurSel();
    if (sel < 0)
        return;

    p_current_group->GetLBText(sel, m_szCurrentGroup);
    DisplayCurrentGroup();
}

void CLightControl::OnBnClickedNetworkClose()
{
    mesh_client_network_close();
}

void CLightControl::DisplayCurrentGroup()
{
    CComboBox *p_network = (CComboBox *)GetDlgItem(IDC_NETWORK);
    CComboBox *p_current_group = (CComboBox *)GetDlgItem(IDC_CURRENT_GROUP);
    CComboBox *p_rename_devs = (CComboBox *)GetDlgItem(IDC_CONFIGURE_RENAME);
    CComboBox *p_move_devs = (CComboBox *)GetDlgItem(IDC_CONFIGURE_MOVE_DEVICE);
    CComboBox *p_move_groups = (CComboBox *)GetDlgItem(IDC_CONFIGURE_MOVE_TO_GROUP);
    CComboBox *p_configure_control_devs = (CComboBox *)GetDlgItem(IDC_CONFIGURE_CONTROL_DEVICE);
    CComboBox *p_configure_publish_to = (CComboBox *)GetDlgItem(IDC_CONFIGURE_PUBLISH_TO);
    CComboBox *p_target_devs_groups = (CComboBox *)GetDlgItem(IDC_CONTROL_DEVICE);

    int cur_group = p_current_group->GetCurSel();

    p_current_group->ResetContent();
    p_rename_devs->ResetContent();
    p_move_devs->ResetContent();
    p_move_groups->ResetContent();
    p_configure_control_devs->ResetContent();
    p_configure_publish_to->ResetContent();
    p_target_devs_groups->ResetContent();

    WCHAR szName[80] = { 0 };
    p_network->GetLBText(p_network->GetCurSel(), szName);
    p_current_group->AddString(szName);
    p_target_devs_groups->AddString(szName);
    p_move_groups->AddString(szName);

    p_configure_publish_to->AddString(L"none");
    p_configure_publish_to->AddString(L"all-nodes");
    p_configure_publish_to->AddString(L"all-proxies");
    p_configure_publish_to->AddString(L"all_friends");
    p_configure_publish_to->AddString(L"all-relays");
    p_configure_publish_to->AddString(L"this-device");

    char *p;
    char *p_groups = mesh_client_get_all_groups(NULL);
    for (p = p_groups; p != NULL && *p != 0; p += (strlen(p) + 1))
    {
        MultiByteToWideChar(CP_UTF8, 0, p, -1, szName, sizeof(szName) / sizeof(WCHAR));
        p_current_group->AddString(szName);
        p_configure_publish_to->AddString(szName);
        p_target_devs_groups->AddString(szName);
        p_move_groups->AddString(szName);
        p_rename_devs->AddString(szName);
    }
    free(p_groups);

    if ((cur_group >= 0) && (p_current_group->GetCount() >= cur_group))
        p_current_group->SetCurSel(cur_group);
    else if (p_current_group->GetCount() > 0)
        p_current_group->SetCurSel(0);

    // get groups and components for the current group
    char group_name[80];

    WideCharToMultiByte(CP_UTF8, 0, m_szCurrentGroup, -1, group_name, 80, NULL, FALSE);
    Log(L"Current Group: %s\n", m_szCurrentGroup);

    Log(L"Groups:\n");
    p_groups = mesh_client_get_all_groups(group_name);
    for (p = p_groups; p != NULL && *p != 0; p += (strlen(p) + 1))
    {
        MultiByteToWideChar(CP_UTF8, 0, p, -1, szName, sizeof(szName) / sizeof(WCHAR));
        Log(L"%s\n", szName);
    }
    free(p_groups);

    Log(L"Components:\n");
    char *p_components = mesh_client_get_group_components(group_name);
    for (p = p_components; p != NULL && *p != 0; p += (strlen(p) + 1))
    {
        MultiByteToWideChar(CP_UTF8, 0, p, -1, szName, sizeof(szName) / sizeof(WCHAR));
        Log(L"%s\n", szName);

        p_rename_devs->AddString(szName);

        uint8_t component_type = mesh_client_get_component_type(p);
        switch (component_type)
        {
        case DEVICE_TYPE_GENERIC_ON_OFF_SERVER:
        case DEVICE_TYPE_GENERIC_LEVEL_SERVER:
        case DEVICE_TYPE_LIGHT_DIMMABLE:
        case DEVICE_TYPE_POWER_OUTLET:
        case DEVICE_TYPE_LIGHT_HSL:
        case DEVICE_TYPE_LIGHT_CTL:
        case DEVICE_TYPE_LIGHT_XYL:
        case DEVICE_TYPE_SENSOR_SERVER:
            p_move_devs->AddString(szName);
            p_configure_control_devs->AddString(szName);
            p_target_devs_groups->AddString(szName);
            p_configure_publish_to->AddString(szName);
            break;

        case DEVICE_TYPE_GENERIC_ON_OFF_CLIENT:
        case DEVICE_TYPE_GENERIC_LEVEL_CLIENT:
            p_target_devs_groups->AddString(szName);
            p_configure_control_devs->AddString(szName);
            break;

        case DEVICE_TYPE_UNKNOWN:
        case DEVICE_TYPE_VENDOR_SPECIFIC:
            p_move_devs->AddString(szName);
            p_target_devs_groups->AddString(szName);
            p_configure_control_devs->AddString(szName);
            p_configure_publish_to->AddString(szName);
            break;
        }
    }
    free(p_components);
}

void CLightControl::OnBnClickedGroupCreate()
{
    int res;
    char mesh_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NETWORK, mesh_name, sizeof(mesh_name));
    char group_name[80], parent_group_name[80];
    GetDlgItemTextA(m_hWnd, IDC_GROUP_NAME, group_name, sizeof(group_name));
    GetDlgItemTextA(m_hWnd, IDC_CURRENT_GROUP, parent_group_name, sizeof(parent_group_name));

    SetDlgItemText(IDC_GROUP_NAME, L"");

    res = mesh_client_group_create(group_name, parent_group_name);
    if (res == MESH_CLIENT_SUCCESS)
    {
        WCHAR szGroup[80], szParent[80];
        MultiByteToWideChar(CP_UTF8, 0, group_name, strlen(group_name) + 1, szGroup, sizeof(szGroup) / sizeof(WCHAR));
        MultiByteToWideChar(CP_UTF8, 0, parent_group_name, strlen(parent_group_name) + 1, szParent, sizeof(szParent) / sizeof(WCHAR));
        Log(L"Group %s created in group %s\n", szGroup, szParent);
        DisplayCurrentGroup();
    }
    else
    {
        Log(L"Failed to create group:%d\n", res);
    }
}

void CLightControl::OnBnClickedGroupDelete()
{
    char group_name[80];
    GetDlgItemTextA(m_hWnd, IDC_CURRENT_GROUP, group_name, sizeof(group_name));
    mesh_client_group_delete(group_name);
    DisplayCurrentGroup();
}

void CLightControl::ProvisionCompleted()
{
    DisplayCurrentGroup();
    if (provision_test)
    {
        CComboBox *p_target_devs_groups = (CComboBox *)GetDlgItem(IDC_CONTROL_DEVICE);
        p_target_devs_groups->SetCurSel(p_target_devs_groups->GetCount() - 1);
        PostMessage(WM_COMMAND, IDC_NODE_RESET, 0);
        return;
    }
}

void network_opened(uint8_t status)
{
    Log(L"Network opened");
}

void unprovisioned_device(uint8_t *p_uuid, uint16_t oob, uint8_t *name, uint8_t name_len)
{
    CClientDialog *pSheet = (CClientDialog *)theApp.m_pMainWnd;
#ifndef NO_LIGHT_CONTROL
    if (pSheet->m_active_page == 1)
    {
        CLightControl *pDlg = &pSheet->pageLight;
        pDlg->ProcessUnprovisionedDevice(p_uuid, oob, name, name_len);
    }
#endif
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)

    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script

    {
        tMESH_CLIENT_SCRIPT_UNPROVISIONED_DEVICE    unprovisioned_device = { 0 };
        memcpy(&unprovisioned_device.uuid, p_uuid, sizeof(UUID));
        if (name_len > 0 && name)
        {
            memcpy(&unprovisioned_device.name, name, name_len);
            unprovisioned_device.name_len = name_len;
        }
        unprovisioned_device.oob = oob;

        mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_UNPROVISIONED_DEVICE, &unprovisioned_device, sizeof(unprovisioned_device));
    }
#endif
}

/*
 * in general the application knows better when connection to the proxy is established or lost.
 * The only case when this function is called, when search for a node or a network times out.
 */
extern void link_status(uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt)
{
    CClientDialog *pSheet = (CClientDialog *)theApp.m_pMainWnd;
#ifndef NO_LIGHT_CONTROL
    if (pSheet->m_active_page == 1)
    {
        CLightControl *pDlg = &pSheet->pageLight;
        pDlg->LinkStatus(is_connected, conn_id, addr, is_over_gatt);
    }
#endif
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_CONNECT_STATUS connect_status;
    connect_status.is_connected = is_connected;
    connect_status.conn_id = conn_id;
    connect_status.addr = addr;
    connect_status.is_over_gatt = is_over_gatt;
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_CONNECT_STATUS, &connect_status, sizeof(connect_status));

#endif
}

/*
 * Result of the componeent connect operation
 */
extern void node_connect_status(uint8_t status, char *p_device_name)
{
    WCHAR buf[512];
    WCHAR szDevName[80];
    CClientDialog *pSheet = (CClientDialog *)theApp.m_pMainWnd;
#ifndef NO_LIGHT_CONTROL
    if (pSheet->m_active_page == 1)
    {
        CLightControl *pDlg = &pSheet->pageLight;

        MultiByteToWideChar(CP_UTF8, 0, p_device_name, -1, szDevName, sizeof(szDevName)/sizeof(WCHAR));

        switch (status)
        {
        case MESH_CLIENT_NODE_CONNECTED:
            wsprintf(buf, L"Node %s connected continue OTA upgrade\n", szDevName);
            Log(buf);
            pDlg->OnOtaUpgradeContinue();
            break;
    
        case MESH_CLIENT_NODE_WARNING_UNREACHABLE:
            wsprintf(buf, L"Node %s failed to connect\n", szDevName);
            Log(buf);
            break;

        case MESH_CLIENT_NODE_ERROR_UNREACHABLE:
            wsprintf(buf, L"!!! Action Required Node %s unreachable\n", szDevName);
            Log(buf);
            break;
        }
    }
#endif
}

void provision_status(uint8_t status, uint8_t *p_uuid)
{
    WCHAR buf[512];
    char *p_devices = mesh_client_get_device_components(p_uuid);

    wsprintf(buf, L"Provision status:%d Device UUID: ", status);

    for (int i = 0; i < 16; i++)
        wsprintf(&buf[wcslen(buf)], L"%02x ", p_uuid[i]);
    wcscat(buf, L"\n");

    Log(buf);

    if (status == MESH_CLIENT_PROVISION_STATUS_CONNECTING)
    {
        provision_test_connect_unprovisioned_time = GetTickCount();
    }
    if (status == MESH_CLIENT_PROVISION_STATUS_PROVISIONING)
    {
        provision_test_provision_start_time = GetTickCount();
    }
    if (status == MESH_CLIENT_PROVISION_STATUS_END)
    {
        provision_test_connect_provisioned_time = GetTickCount();
    }
    if (status == MESH_CLIENT_PROVISION_STATUS_CONFIGURING)
    {
        provision_test_config_start_time = GetTickCount();
    }
    if (status == MESH_CLIENT_PROVISION_STATUS_SUCCESS)
    {
        provision_test_reset_time = GetTickCount();
    }

    if (status != MESH_CLIENT_PROVISION_STATUS_SUCCESS && status != MESH_CLIENT_PROVISION_STATUS_FAILED)
        return;
    if (status == MESH_CLIENT_PROVISION_STATUS_SUCCESS)
    {
        for (char *p_component_name = p_devices; p_component_name != NULL && *p_component_name != 0; p_component_name += (strlen(p_component_name) + 1))
        {
            WCHAR szDevName[80];
            char *target_methods = mesh_client_get_target_methods(p_component_name);
            char *control_methods = mesh_client_get_control_methods(p_component_name);

            MultiByteToWideChar(CP_UTF8, 0, p_component_name, -1, szDevName, sizeof(szDevName) / sizeof(WCHAR));
            wsprintf(buf, L"Name:%s", szDevName);
            Log(buf);
            if ((target_methods != NULL) && (target_methods[0] != 0))
            {
                wcscpy(buf, L"Can be controlled using: ");
                for (char *p = target_methods; *p != 0; p = p + strlen(p) + 1)
                {
                    MultiByteToWideChar(CP_UTF8, 0, p, -1, &buf[wcslen(buf)], sizeof(buf) / sizeof(WCHAR) - wcslen(buf));
                    wcscat(buf, L", ");
                }
                wcscat(buf, L"\n");
                Log(buf);
            }
            if ((control_methods != NULL) && (control_methods[0] != 0))
            {
                wcscpy(buf, L"Can control: ");
                for (char *p = control_methods; *p != 0; p = p + strlen(p) + 1)
                {
                    MultiByteToWideChar(CP_UTF8, 0, p, -1, &buf[wcslen(buf)], sizeof(buf) / sizeof(WCHAR) - wcslen(buf));
                    wcscat(buf, L", ");
                }
                wcscat(buf, L"\n");
                Log(buf);
            }
        }
    }
    CClientDialog *pSheet = (CClientDialog *)theApp.m_pMainWnd;
#ifndef NO_LIGHT_CONTROL
    if (pSheet->m_active_page == 1)
    {
        CLightControl *pDlg = &pSheet->pageLight;
        pDlg->ProvisionCompleted();
    }
#endif
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_PROVISION_STATUS provision_status = { 0 };
    provision_status.status = status;
    if (p_uuid)
    {
        memcpy(&provision_status.uuid[0], p_uuid, sizeof(provision_status.uuid));
    }
    if (p_devices)
    {
        memcpy(&provision_status.name, p_devices, strlen(p_devices));
    }
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_PROVISION_STATUS, &provision_status, sizeof(provision_status));
#endif
    free(p_devices);
}

void database_changed(char *mesh_name)
{
    Log(L"database changed\n");
}

void onoff_status(const char *device_name, uint8_t present, uint8_t target, uint32_t remaining_time)
{
    WCHAR szDevName[80];
    size_t name_len = device_name ? strlen(device_name) + 1 : 0;
    MultiByteToWideChar(CP_UTF8, 0, device_name, -1, szDevName, sizeof(szDevName)/sizeof(WCHAR));
    Log(L"%s OnOff state:%d\n", szDevName, present);
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_ON_OFF_STATUS onoff_status = { 0 };
    if (device_name && name_len)
    {
        memcpy(&onoff_status.device_name, device_name, name_len);
    }
    onoff_status.present = present;
    onoff_status.target = target;
    onoff_status.remaining_time = remaining_time;
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_ON_OFF_STATUS, &onoff_status, sizeof(onoff_status));
#endif
}

void level_status(const char *device_name, int16_t present, int16_t target, uint32_t remaining_time)
{
    WCHAR szDevName[80];
    size_t name_len = device_name ? strlen(device_name) + 1 : 0;
    MultiByteToWideChar(CP_UTF8, 0, device_name, -1, szDevName, sizeof(szDevName) / sizeof(WCHAR));
    Log(L"%s Level state:%d\n", szDevName, present);

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_LEVEL_STATUS level_status = { 0 };
    if (device_name && name_len)
    {
        memcpy(&level_status.device_name, device_name, name_len);
    }
    level_status.present = present;
    level_status.target = target;
    level_status.remaining_time = remaining_time;
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_LEVEL_STATUS, &level_status, sizeof(level_status));
#endif
}

void lightness_status(const char *device_name, uint16_t present, uint16_t target, uint32_t remaining_time)
{
    WCHAR szDevName[80];
    size_t name_len = device_name ? strlen(device_name) + 1 : 0;
    MultiByteToWideChar(CP_UTF8, 0, device_name, -1, szDevName, sizeof(szDevName) / sizeof(WCHAR));
    Log(L"%s Light:%d\n", szDevName, present);

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_LIGHTNESS_STATUS lightness_status = { 0 };
    if (device_name && name_len)
    {
        memcpy(&lightness_status.device_name, device_name, name_len);
    }
    lightness_status.present = present;
    lightness_status.target = target;
    lightness_status.remaining_time = remaining_time;
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_LIGHTNESS_STATUS, &lightness_status, sizeof(lightness_status));
#endif
}

void hsl_status(const char *device_name, uint16_t lightness, uint16_t hue, uint16_t saturation, uint32_t remaining_time)
{
    WCHAR szDevName[80];
    size_t name_len = device_name ? strlen(device_name) + 1 : 0;
    MultiByteToWideChar(CP_UTF8, 0, device_name, -1, szDevName, sizeof(szDevName) / sizeof(WCHAR));
    Log(L"%s Light:%d Hue:%d Sat:%d\n", szDevName, lightness, hue, saturation);

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_HSL_STATUS hsl_status = { 0 };
    if (device_name && name_len)
    {
        memcpy(&hsl_status.device_name, device_name, name_len);
    }
    hsl_status.lightness = lightness;
    hsl_status.hue = hue;
    hsl_status.saturation = saturation;
    hsl_status.remaining_time = remaining_time;
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_HSL_STATUS, &hsl_status, sizeof(hsl_status));
#endif
}

void ctl_status(const char *device_name, uint16_t present_lightness, uint16_t present_temperature, uint16_t target_lightness, uint16_t target_temperature, uint32_t remaining_time)
{
    WCHAR szDevName[80];
    size_t name_len = device_name ? strlen(device_name) + 1 : 0;
    MultiByteToWideChar(CP_UTF8, 0, device_name, -1, szDevName, sizeof(szDevName) / sizeof(WCHAR));
    Log(L"%s present Light/Temp:%d/%d target Light/Temp:%d/%d\n", szDevName, present_lightness, present_temperature, target_lightness, target_temperature);

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_CTL_STATUS ctl_status = { 0 };
    if (device_name && name_len)
    {
        memcpy(&ctl_status.device_name, device_name, name_len);
    }
    ctl_status.present_lightness = present_lightness;
    ctl_status.present_temperature = present_temperature;
    ctl_status.target_lightness = target_lightness;
    ctl_status.target_temperature = target_temperature;
    ctl_status.remaining_time = remaining_time;
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_CTL_STATUS, &ctl_status, sizeof(ctl_status));
#endif
}

void sensor_status(const char *device_name, int property_id, uint8_t value_len, uint8_t *value)
{
    WCHAR szDevName[80];
    size_t name_len = device_name ? strlen(device_name) + 1 : 0;
    MultiByteToWideChar(CP_UTF8, 0, device_name, -1, szDevName, sizeof(szDevName) / sizeof(WCHAR));

    WCHAR   msg[1002];
    if (property_id == WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE)
        swprintf_s(msg, sizeof(msg) / 2, L"Sensor data from:%s Ambient Temperature %d degrees Celsius", szDevName, value[0] / 2);
    else if (property_id == WICED_BT_MESH_PROPERTY_PRESENCE_DETECTED)
        swprintf_s(msg, sizeof(msg) / 2, L"Sensor data from:%s Presense detected %s", szDevName, value[0] != 0 ? L"true" : L"false");
    else if (property_id == WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL)
        swprintf_s(msg, sizeof(msg) / 2, L"Sensor data from:%s ambient light level %d", szDevName, value[0] + (value[1] << 8) + (value[2] << 16));
    else
    {
        swprintf_s(msg, sizeof(msg) / 2, L"Sensor data from %s Property ID:0x%x 0x: ", szDevName, property_id);
        for (int i = 0; i < value_len; i++)
        {
            int len = swprintf_s(&msg[wcslen(msg)], sizeof(msg) / 2 - wcslen(msg), L"%02x", value[i]);
        }
        wcscat(msg, L"\n");
    }
    Log(msg);
}

void CLightControl::OnBnClickedNodeReset()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_reset_device(name);

    DisplayCurrentGroup();
}

void CLightControl::OnSelchangeNetwork()
{
    OnBnClickedNetworkClose();
}


void CLightControl::OnBnClickedConfigureNewName()
{
    CComboBox *p_rename_devs = (CComboBox *)GetDlgItem(IDC_CONFIGURE_RENAME);
    int sel = p_rename_devs->GetCurSel();
    if (sel < 0)
        return;

    char old_name[80];
    WCHAR szDevName[80];
    p_rename_devs->GetLBText(sel, szDevName);
    WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, old_name, 80, NULL, FALSE);

    char new_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NEW_NAME, new_name, sizeof(new_name) - 1);
    if (strlen(new_name) > 0)
    {
        SetDlgItemText(IDC_NEW_NAME, L"");

        mesh_client_rename(old_name, new_name);
        strcpy(old_name, new_name);
    }
    char group_name[80];
    GetDlgItemTextA(m_hWnd, IDC_CURRENT_GROUP, group_name, sizeof(group_name));

    DeviceConfig.is_gatt_proxy = (BYTE)((CButton *)GetDlgItem(IDC_GATT_PROXY))->GetCheck();
    DeviceConfig.is_friend = (BYTE)((CButton *)GetDlgItem(IDC_FRIEND))->GetCheck();
    DeviceConfig.is_relay = (BYTE)((CButton *)GetDlgItem(IDC_RELAY))->GetCheck();
    DeviceConfig.send_net_beacon = (BYTE)((CButton *)GetDlgItem(IDC_NET_BEACON))->GetCheck();
    DeviceConfig.relay_xmit_count = GetDlgItemInt(IDC_RELAY_TRANSMIT_COUNT);
    DeviceConfig.relay_xmit_interval = GetDlgItemInt(IDC_RELAY_TRANSMIT_INTERVAL);
    DeviceConfig.default_ttl = GetDlgItemInt(IDC_DEFAULT_TTL);
    DeviceConfig.net_xmit_count = (BYTE)GetDlgItemInt(IDC_NETWORK_TRANSMIT_COUNT);
    DeviceConfig.net_xmit_interval = (USHORT)GetDlgItemInt(IDC_NETWORK_TRANSMIT_INTERVAL);

    FILE *fp = fopen("NetParameters.bin", "wb");
    if (fp)
    {
        fwrite(&DeviceConfig, 1, sizeof(DeviceConfig), fp);
        fclose(fp);
    }

    mesh_client_set_device_config(old_name, DeviceConfig.is_gatt_proxy, DeviceConfig.is_friend, DeviceConfig.is_relay, DeviceConfig.send_net_beacon, DeviceConfig.relay_xmit_count, DeviceConfig.relay_xmit_interval, DeviceConfig.default_ttl, DeviceConfig.net_xmit_count, DeviceConfig.net_xmit_interval);
    mesh_client_set_publication_config(DeviceConfig.publish_credential_flag, DeviceConfig.publish_retransmit_count, DeviceConfig.publish_retransmit_interval, DeviceConfig.publish_ttl);

    DisplayCurrentGroup();
}

void CLightControl::OnBnClickedMoveToGroup()
{
    char device_name[80];
    char group_to_name[80] = { 0 };
    char group_from_name[80] = { 0 };
    WCHAR szDevName[80];
    WCHAR szGroupName[80];

    CComboBox *p_device = (CComboBox *)GetDlgItem(IDC_CONFIGURE_MOVE_DEVICE);
    int sel = p_device->GetCurSel();
    if (sel < 0)
        return;

    p_device->GetLBText(sel, szDevName);
    WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, device_name, 80, NULL, FALSE);

    CComboBox *pGroup = (CComboBox *)GetDlgItem(IDC_CONFIGURE_MOVE_TO_GROUP);
    if ((sel = pGroup->GetCurSel()) >= 0)
    {
        pGroup->GetLBText(sel, szGroupName);
        WideCharToMultiByte(CP_UTF8, 0, szGroupName, -1, group_to_name, 80, NULL, FALSE);
    }
    pGroup = (CComboBox *)GetDlgItem(IDC_CONFIGURE_MOVE_FROM_GROUP);
    if ((sel = pGroup->GetCurSel()) >= 0)
    {
        pGroup->GetLBText(sel, szGroupName);
        WideCharToMultiByte(CP_UTF8, 0, szGroupName, -1, group_from_name, 80, NULL, FALSE);
    }
    if ((group_from_name[0] == 0) && (group_to_name[0] != 0))
        mesh_client_add_component_to_group(device_name, group_to_name);
    else if ((group_from_name[0] != 0) && (group_to_name[0] == 0))
        mesh_client_remove_component_from_group(device_name, group_from_name);
    else if ((group_from_name[0] != 0) && (group_to_name[0] != 0))
        mesh_client_move_component_to_group(device_name, group_from_name, group_to_name);
}

void CLightControl::OnCbnSelchangeConfigureControlDevice()
{
    CComboBox *p_device = (CComboBox *)GetDlgItem(IDC_CONFIGURE_CONTROL_DEVICE);
    int sel = p_device->GetCurSel();
    if (sel < 0)
        return;

    WCHAR szDevName[80];
    p_device->GetLBText(sel, szDevName);

    char device_name[80];
    WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, device_name, 80, NULL, FALSE);

    CComboBox *p_method = (CComboBox *)GetDlgItem(IDC_CONFIGURE_PUBLISH_METHOD);
    p_method->ResetContent();

    char *target_methods = mesh_client_get_target_methods(device_name);
    for (char *p = target_methods; p != NULL && *p != 0; p += (strlen(p) + 1))
    {
        WCHAR szPublishMethod[80];
        wcscpy(szPublishMethod, L"send \"");
        MultiByteToWideChar(CP_UTF8, 0, p, strlen(p) + 1, &szPublishMethod[wcslen(szPublishMethod)], sizeof(szPublishMethod) / sizeof(WCHAR) - 28);
        if (strncmp(p, MESH_CONTROL_METHOD_VENDOR, strlen(MESH_CONTROL_METHOD_VENDOR)) == 0)
            wcscat(szPublishMethod, L"\" data to");
        else
            wcscat(szPublishMethod, L"\" status to");
        p_method->AddString(szPublishMethod);
    }
    free(target_methods);

    char *control_methods = mesh_client_get_control_methods(device_name);
    for (char *p = control_methods; p != NULL && *p != 0; p += (strlen(p) + 1))
    {
        WCHAR szPublishMethod[80];
        wcscpy(szPublishMethod, L"control \"");
        MultiByteToWideChar(CP_UTF8, 0, p, strlen(p) + 1, &szPublishMethod[wcslen(szPublishMethod)], sizeof(szPublishMethod) / sizeof(WCHAR) - 24);
        wcscat(szPublishMethod, L"\" of");
        p_method->AddString(szPublishMethod);
    }
    free(control_methods);
    if (p_method->GetCount() != 0)
        p_method->SetCurSel(0);
}


void CLightControl::OnBnClickedConfigurePub()
{
    CComboBox *p_device = (CComboBox *)GetDlgItem(IDC_CONFIGURE_CONTROL_DEVICE);
    int sel = p_device->GetCurSel();
    if (sel < 0)
        return;

    WCHAR szDevName[80];
    p_device->GetLBText(sel, szDevName);

    CComboBox *p_method = (CComboBox *)GetDlgItem(IDC_CONFIGURE_PUBLISH_METHOD);
    if ((sel = p_method->GetCurSel()) < 0)
        return;

    WCHAR szPublishString[80], szPublishMethod[80];
    WCHAR *p;
    p_method->GetLBText(sel, szPublishString);
    uint8_t client_control;
    if (wcsncmp(szPublishString, L"control \"", wcslen(L"control \"")) == 0)
    {
        client_control = 1;
        p = &szPublishString[wcslen(L"control \"")];
    }
    else
    {
        client_control = 0;
        p = &szPublishString[wcslen(L"send \"")];
    }
    int i;
    for (i = 0; *p != '\"'; i++)
        szPublishMethod[i] = *p++;
    szPublishMethod[i] = 0;
    char publish_method[80];
    WideCharToMultiByte(CP_UTF8, 0, szPublishMethod, -1, publish_method, 80, NULL, FALSE);

    CComboBox *p_configure_publish_to = (CComboBox *)GetDlgItem(IDC_CONFIGURE_PUBLISH_TO);
    if ((sel = p_configure_publish_to->GetCurSel()) < 0)
        return;

    WCHAR szPublishToName[80];
    p_configure_publish_to->GetLBText(sel, szPublishToName);

    char device_name[80];
    char publish_to_name[80];
    WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, device_name, 80, NULL, FALSE);
    WideCharToMultiByte(CP_UTF8, 0, szPublishToName, -1, publish_to_name, 80, NULL, FALSE);

    // Get publication parameters from the dialog and tell Client that new parameters should be used
    int publish_credential_flag = (BYTE)((CComboBox *)GetDlgItem(IDC_MODEL_PUB_CREDENTIAL_FLAG))->GetCurSel();
    int publish_period = GetDlgItemInt(IDC_MODEL_PUB_PERIOD);
    int publish_ttl = GetDlgItemInt(IDC_PUBLISH_TTL);
    int publish_retransmit_count = GetDlgItemInt(IDC_MODEL_PUB_RETRANSMIT_COUNT);
    int publish_retransmit_interval = GetDlgItemInt(IDC_MODEL_PUB_RETRANSMIT_INTERVAL);

    mesh_client_set_publication_config(publish_credential_flag, publish_retransmit_count, publish_retransmit_interval, publish_ttl);
    mesh_client_configure_publication(device_name, client_control, publish_method, publish_to_name, publish_period);
}

void CLightControl::OnBnClickedOnOffGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_on_off_get(name);
}

void CLightControl::OnBnClickedOnOffSet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    int on_off = ((CComboBox *)GetDlgItem(IDC_ON_OFF_TARGET))->GetCurSel();
    if (on_off >= 0)
        mesh_client_on_off_set(name, on_off, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
}

void CLightControl::OnBnClickedLevelGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_level_get(name);
}

void CLightControl::OnBnClickedLevelSet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    SHORT target_level = (SHORT)GetDlgItemInt(IDC_LEVEL_TARGET);
    mesh_client_level_set(name, target_level, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
}

void CLightControl::OnBnClickedLightnessGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_lightness_get(name);
}


void CLightControl::OnBnClickedLightnessSet()
{
    SHORT target_lightness = (SHORT)GetDlgItemInt(IDC_LIGHT_LIGHTNESS_TARGET);
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_lightness_set(name, target_lightness, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
}

void CLightControl::OnBnClickedLightHslGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_hsl_get(name);
}

void CLightControl::OnBnClickedLightHslSet()
{
    SHORT target_lightness = (SHORT)GetDlgItemInt(IDC_LIGHT_LIGHTNESS_TARGET);
    SHORT target_hue = (SHORT)GetDlgItemInt(IDC_LIGHT_HSL_HUE_VALUE);
    SHORT target_saturation = (SHORT)GetDlgItemInt(IDC_LIGHT_HSL_SATURATION_VALUE);
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_hsl_set(name, target_lightness, target_hue, target_saturation, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
}

void CLightControl::OnBnClickedLightCtlGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_ctl_get(name);
}

void CLightControl::OnBnClickedLightCtlSet()
{
    SHORT target_lightness = (SHORT)GetDlgItemInt(IDC_LIGHT_LIGHTNESS_TARGET);
    SHORT target_temperature = (SHORT)GetDlgItemInt(IDC_LIGHT_CTL_TEMPERATURE_TARGET);
    SHORT target_delta_uv = (SHORT)GetDlgItemInt(IDC_LIGHT_CTL_DELTA_UV_TARGET);
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_ctl_set(name, target_lightness, target_temperature, target_delta_uv, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
}

void CLightControl::OnBnClickedVsData()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));

    BYTE buffer[400];
    DWORD len = GetHexValue(IDC_TC_NET_LEVEL_TRX_PDU, buffer, sizeof(buffer));

    mesh_client_vendor_data_set(name, buffer, (uint16_t)len);
}

void CLightControl::OnCbnSelchangeControlDevice()
{
    CComboBox *pSensors = (CComboBox *)GetDlgItem(IDC_SENSOR_TARGET);
    char name[80];
    ((CComboBox *)GetDlgItem(IDC_SENSOR_TARGET))->ResetContent();
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));

    int *property_ids = mesh_client_sensor_property_list_get(name);
    if (property_ids == NULL)
        return;
    int *p;

    for (p = property_ids; *p != 0; p++)
    {
        WCHAR szPropertyId[80];
        wsprintf(szPropertyId, L"%04X", *p);
        pSensors->SetItemData(pSensors->AddString(szPropertyId), *p);
    }
    free(property_ids);
    if (pSensors->GetCount() != 0)
        pSensors->SetCurSel(0);
}

void CLightControl::OnBnClickedSensorGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));

    int sel;
    CComboBox *pSensors = (CComboBox *)GetDlgItem(IDC_SENSOR_TARGET);
    if ((sel = pSensors->GetCurSel()) < 0)
        return;

    int property_id = (int)pSensors->GetItemData(sel);

    mesh_client_sensor_get(name, property_id);
}


void CLightControl::ProcessVendorSpecificData(LPBYTE p_data, DWORD len)
{
    DWORD i;
    USHORT src = p_data[0] + (p_data[1] << 8);
    USHORT app_key_idx = (USHORT)p_data[2] + ((USHORT)p_data[3] << 8);
    BYTE   element_idx = p_data[4];
    p_data += 5;
    len -= 5;

    WCHAR buf[100];
    wsprintf(buf, L"VS Data from addr:%x element:%x app_key_idx:%x %d bytes:", src, app_key_idx, element_idx, len);
    m_trace->SetCurSel(m_trace->AddString(buf));

    while (len != 0)
    {
        buf[0] = 0;
        for (i = 0; i < len && i < 32; i++)
            wsprintf(&buf[wcslen(buf)], L"%02x ", p_data[i]);

        len -= i;
        if (len != 0)
            m_trace->SetCurSel(m_trace->AddString(buf));
    }
    m_trace->SetCurSel(m_trace->AddString(buf));
}

void CLightControl::OnBnClickedIdentify()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_identify(name, 10);
}

void component_info_status_callback(uint8_t status, char *component_name, char *component_info)
{
    WCHAR szComponentName[80];
    WCHAR szComponentInfo[80];
    MultiByteToWideChar(CP_UTF8, 0, component_name, -1, szComponentName, sizeof(szComponentName) / sizeof(WCHAR));
    MultiByteToWideChar(CP_UTF8, 0, component_info, -1, szComponentInfo, sizeof(szComponentInfo) / sizeof(WCHAR));
    Log(L"Component Info status:%d from %s Info:%s\n", status, szComponentName, szComponentInfo);
}

void CLightControl::OnBnClickedGetComponentInfo()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    mesh_client_get_component_info(name, component_info_status_callback);
}

void CLightControl::OnBnClickedOtaUpgradeStart()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    int dfu_method = ((CComboBox *)GetDlgItem(IDC_DFU_METHOD))->GetCurSel();

    // Regardless of what are we trying to do, need to connect to a specific component
    // then OtaUpgradeContinue will be executed
    mesh_client_connect_component(name, 1, 10);
}

void CLightControl::OnBnClickedOtaUpgradeStop()
{
    mesh_client_dfu_stop();
}

void fw_distribution_status(uint8_t status, int current_block_num, int total_blocks)
{
}


void CLightControl::OnBnClickedOtaUpgradeStatus()
{
    mesh_client_dfu_get_status(NULL, 0, NULL, 0, &fw_distribution_status);
}

void CLightControl::OnOtaUpgradeContinue()
{
#if 0
    int dfu_method = ((CComboBox *)GetDlgItem(IDC_DFU_METHOD))->GetCurSel();
    if (dfu_method != DFU_METHOD_APP_TO_DEVICE)
    {
        char name[80];
        GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
        EnterCriticalSection(&cs);
        mesh_client_dfu_start(dfu_method, name);
        LeaveCriticalSection(&cs);
        return;
    }
    // We are doing proprietary OTA Upgrad (app to device)
    // If Downloader object is created already
    if (m_pDownloader != NULL)
    {
        delete m_pDownloader;
        m_pDownloader = NULL;
    }
    CString sFilePath;
    GetDlgItemText(IDC_FILENAME, sFilePath);

    if (sFilePath.IsEmpty())
    {
        OnBnClickedBrowse();
        GetDlgItemText(IDC_FILENAME, sFilePath);
        if (sFilePath.IsEmpty())
            return;
    }
    FILE *fPatch;
    if (_wfopen_s(&fPatch, sFilePath, L"rb"))
    {
        MessageBox(L"Failed to open the patch file", L"Error", MB_OK);
        return;
    }

    // Load OTA FW file into memory
    fseek(fPatch, 0, SEEK_END);
    m_dwPatchSize = ftell(fPatch);
    rewind(fPatch);
    if (m_pPatch)
        delete m_pPatch;
    m_pPatch = (LPBYTE)new BYTE[m_dwPatchSize];

    m_dwPatchSize = (DWORD)fread(m_pPatch, 1, m_dwPatchSize, fPatch);
    fclose(fPatch);

    EnterCriticalSection(&cs);
    CBtWin10Interface *pWin10BtInterface = NULL;
    pWin10BtInterface = dynamic_cast<CBtWin10Interface *>(m_btInterface);

    if ((pWin10BtInterface == NULL) || !((CBtWin10Interface*)m_btInterface)->CheckForOTAServices())
    {
        LeaveCriticalSection(&cs);
        MessageBox(L"This device may not support OTA FW Upgrade. Select another device.", L"Error", MB_OK);

        if (m_pPatch)
            delete m_pPatch;
        return;
    }
    LeaveCriticalSection(&cs);
    SetDlgItemText(IDC_OTA_UPGRADE_START, L"OTA Abort");

    // Create new downloader object
    m_pDownloader = new WSDownloader(m_btInterface, m_pPatch, m_dwPatchSize, m_hWnd);
    pWin10BtInterface->m_bConnected = TRUE;

    BTW_GATT_VALUE gatt_value;
    gatt_value.len = 2;
    gatt_value.value[0] = 3;
    gatt_value.value[1] = 0;

    EnterCriticalSection(&cs);
    if (m_btInterface != NULL)
    {
        guidSvcWSUpgrade = m_btInterface->m_bSecure ? GUID_OTA_SEC_FW_UPGRADE_SERVICE : GUID_OTA_FW_UPGRADE_SERVICE;
        pWin10BtInterface->SetDescriptorValue(&guidSvcWSUpgrade, &guidCharWSUpgradeControlPoint, BTW_GATT_UUID_DESCRIPTOR_CLIENT_CONFIG, &gatt_value);
        pWin10BtInterface->RegisterNotification(&guidSvcWSUpgrade, &guidCharWSUpgradeControlPoint);
    }
    LeaveCriticalSection(&cs);

    m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_CONNECTED);
#endif
}

LRESULT CLightControl::OnWsUpgradeCtrlPoint(WPARAM Instance, LPARAM lparam)
{
#if 0
    BTW_GATT_VALUE *pValue = (BTW_GATT_VALUE *)lparam;

    ods("OnWsUpgradeCtrlPoint: len:%d\n", pValue->len);
    if (pValue->len == 1)
    {
        switch (pValue->value[0])
        {
        case WICED_OTA_UPGRADE_STATUS_OK:
            m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_RESPONSE_OK);
            break;
        case WICED_OTA_UPGRADE_STATUS_CONTINUE:
            m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_CONTINUE);
            break;
        default:
            m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_RESPONSE_FAILED);
            break;
        }
    }
    free(pValue);
#endif
    return S_OK;
}

LRESULT CLightControl::OnProgress(WPARAM state, LPARAM param)
{
#if 0
    static UINT total;
    if (state == WSDownloader::WS_UPGRADE_STATE_WAIT_FOR_READY_FOR_DOWNLOAD)
    {
        total = (UINT)param;
        m_Progress.SetRange32(0, (int)param);
        SetDlgItemText(IDC_OTA_UPGRADE_START, L"Abort");
    }
    else if (state == WSDownloader::WS_UPGRADE_STATE_DATA_TRANSFER)
    {
        m_Progress.SetPos((int)param);
        if (param == total)
        {
            m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_START_VERIFICATION);
        }
    }
    else if (state == WSDownloader::WS_UPGRADE_STATE_VERIFIED)
    {
        SetDlgItemText(IDC_OTA_UPGRADE_START, "OTA Start");
    }
    else if (state == WSDownloader::WS_UPGRADE_STATE_ABORTED)
    {
        m_Progress.SetPos(total);
        SetDlgItemText(IDC_OTA_UPGRADE_START, "OTA Start");
    }
#endif
    return S_OK;
}


void CLightControl::OnBnClickedBrowseOta()
{
    static TCHAR BASED_CODE szFilter[] = _T("OTA Files (*.ota.bin)|*.OTA.BIN|");

    CFileDialog dlgFile(TRUE, NULL, NULL, OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, szFilter);
    if (dlgFile.DoModal() == IDOK)
    {
        SetDlgItemText(IDC_FILENAME, dlgFile.GetPathName());
    }
}

void CLightControl::OnBnClickedNetworkImport()
{
    static TCHAR BASED_CODE szFilter[] = _T("JSON Files (*.json)|*.JSON|");

    CFileDialog dlgFile(TRUE, NULL, NULL, OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, szFilter);
    if (dlgFile.DoModal() != IDOK)
        return;
    FILE *fJsonFile;
    if (_wfopen_s(&fJsonFile, dlgFile.GetPathName(), L"rb"))
    {
        MessageBox(L"Failed to open the json file", L"Error", MB_OK);
        return;
    }

    // Load OTA FW file into memory
    fseek(fJsonFile, 0, SEEK_END);
    size_t json_string_size = ftell(fJsonFile);
    rewind(fJsonFile);

    char *json_string = (char *)new BYTE[json_string_size];
    if (json_string == NULL)
        return;
    fread(json_string, 1, json_string_size, fJsonFile);
    char provisioner_name[80];
    char *mesh_name;
    GetDlgItemTextA(m_hWnd, IDC_PROVISIONER, provisioner_name, sizeof(provisioner_name));
    if ((mesh_name = mesh_client_network_import(provisioner_name, provisioner_uuid, json_string, network_opened)) == NULL)
    {
        MessageBox(L"Failed to import json file", L"Error", MB_OK);
    }
    else
    {
        WCHAR s[80];
        MultiByteToWideChar(CP_UTF8, 0, mesh_name, strlen(mesh_name) + 1, s, sizeof(s) / sizeof(WCHAR));
        Log(L"Network %s imported\n", s);

        ((CComboBox *)GetDlgItem(IDC_NETWORK))->ResetContent();

        char *p_networks = mesh_client_get_all_networks();
        char *p = p_networks;
        WCHAR szNetwork[80];
        int i = 0, sel = -1;
        while (p != NULL && *p != NULL)
        {
            MultiByteToWideChar(CP_UTF8, 0, p, strlen(p) + 1, szNetwork, sizeof(szNetwork) / sizeof(WCHAR));
            ((CComboBox *)GetDlgItem(IDC_NETWORK))->AddString(szNetwork);
            p += strlen(p) + 1;
            i++;
            if (strcmp(p, mesh_name) == 0)
                sel = i;
        }
        if (sel >= 0)
        {
            ((CComboBox *)GetDlgItem(IDC_NETWORK))->SetCurSel(sel);
        }
        WCHAR szGroup[80];
        MultiByteToWideChar(CP_UTF8, 0, mesh_name, strlen(mesh_name) + 1, szGroup, sizeof(szGroup) / sizeof(WCHAR));
        wcscpy(m_szCurrentGroup, szGroup);

        DisplayCurrentGroup();
    }
    delete[] json_string;
}

void CLightControl::OnBnClickedNetworkExport()
{
    char mesh_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NETWORK, mesh_name, sizeof(mesh_name));

    CFileDialog dlgFile(TRUE, NULL, NULL, OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, NULL);
    if (dlgFile.DoModal() != IDOK)
        return;

    char *json_string = mesh_client_network_export(mesh_name);
    if (json_string != NULL)
    {
        FILE *fJsonFile;
        if (_wfopen_s(&fJsonFile, dlgFile.GetPathName(), L"w"))
        {
            MessageBox(L"Failed to open the json file", L"Error", MB_OK);
            return;
        }
        fwrite(json_string, 1, strlen(json_string), fJsonFile);
        fclose(fJsonFile);

        free(json_string);
    }
}

void CLightControl::OnBnClickedSensorConfigure()
{
    CSensorConfig dlg;
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, dlg.component_name, sizeof(dlg.component_name));

    CComboBox *pSensors = (CComboBox *)GetDlgItem(IDC_SENSOR_TARGET);
    if (pSensors->GetCurSel() < 0)
        return;

    dlg.property_id = (USHORT)pSensors->GetItemData(pSensors->GetCurSel());

    INT_PTR nResponse = dlg.DoModal();
}

uint32_t CLightControl::GetDfuImageSize()
{
    uint32_t file_size;
    CString sFilePath;

    GetDlgItemText(IDC_FILENAME, sFilePath);

    FILE *fPatch;
    if (_wfopen_s(&fPatch, sFilePath, L"rb"))
        return 0;

    // Load OTA FW file into memory
    fseek(fPatch, 0, SEEK_END);
    file_size = (int)ftell(fPatch);
    fclose(fPatch);
    return file_size;
}

void CLightControl::GetDfuImageChunk(uint8_t *p_data, uint32_t offset, uint16_t data_len)
{
    CString sFilePath;

    GetDlgItemText(IDC_FILENAME, sFilePath);

    FILE *fPatch;
    if (_wfopen_s(&fPatch, sFilePath, L"rb"))
        return;

    // Load OTA FW file into memory
    fseek(fPatch, offset, SEEK_SET);
    fread(p_data, 1, data_len, fPatch);
    fclose(fPatch);
}

extern "C" uint32_t wiced_bt_get_fw_image_size(uint8_t partition)
{
    CLightControl *pDlg = (CLightControl *)theApp.m_pMainWnd;
    return (pDlg != NULL) ? pDlg->GetDfuImageSize() : 0;
}

extern "C" void wiced_bt_get_fw_image_chunk(uint8_t partition, uint32_t offset, uint8_t *p_data, uint16_t data_len)
{
    CLightControl *pDlg = (CLightControl *)theApp.m_pMainWnd;
    if (pDlg != NULL) 
        pDlg->GetDfuImageChunk(p_data, offset, data_len);
}

extern "C" int mesh_client_set_device_config_UI_Ex(const char *device_name, int is_gatt_proxy, int is_friend, int is_relay, int beacon,
    int relay_xmit_count, int relay_xmit_interval, int default_ttl, int net_xmit_count, int net_xmit_interval)
{
    DeviceConfig.is_gatt_proxy = is_gatt_proxy;
    DeviceConfig.is_friend = is_friend;
    DeviceConfig.is_relay = is_relay;
    DeviceConfig.send_net_beacon = beacon;
    DeviceConfig.relay_xmit_count = relay_xmit_count;
    DeviceConfig.relay_xmit_interval = relay_xmit_interval;
    DeviceConfig.default_ttl = default_ttl;
    DeviceConfig.net_xmit_count = net_xmit_count;
    DeviceConfig.net_xmit_interval = net_xmit_interval;

    return 0;
}

extern "C" int mesh_client_set_publication_config_UI_Ex(const char *device_name, int device_type, int publish_credential_flag,
    int publish_retransmit_count, int publish_retransmit_interval, int publish_ttl)
{
    DeviceConfig.publish_credential_flag = publish_credential_flag;
    DeviceConfig.publish_ttl = publish_ttl;
    DeviceConfig.publish_retransmit_count = publish_retransmit_count;
    DeviceConfig.publish_retransmit_interval = publish_retransmit_interval;

    FILE *fp = fopen("NetParameters.bin", "wb");
    if (fp)
    {
        fwrite(&DeviceConfig, 1, sizeof(DeviceConfig), fp);
        fclose(fp);
    }

    return 0;
}

extern "C" int mesh_client_network_create_UI_Ex(const char *provisioner_name, const char *p_provisioner_uuid, char *mesh_name)
{
    return 0;
}

extern "C" int mesh_client_scan_unprovisioned_UI_Ex(int start)
{
    if (!provision_test_bScanning)
    {
        provision_test_scan_unprovisioned_time = GetTickCount();
        provision_test_bScanning = TRUE;
    }
    else
    {
        provision_test_bScanning = FALSE;
    }
    return 0;
}

BOOL CLightControl::OnInitDialog()
{
    CPropertyPage::OnInitDialog();

    m_trace = (CListBox *)GetDlgItem(IDC_TRACE);
    m_trace->AddString(L"NOTE:");
    m_trace->AddString(L"Use Baud rate of 3000000 for CYW920819EVB-02 board and 115200 for CYBT-213043-MESH board.");

    return TRUE;  // return TRUE unless you set the focus to a control
                  // EXCEPTION: OCX Property Pages should return FALSE
}
