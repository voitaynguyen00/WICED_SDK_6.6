/*
* Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
*/

/** @file
* MeshClientDlg.cpp : implementation file
*
*/

#include "stdafx.h"
#include "afxdialogex.h"
#include <setupapi.h>
#include "MeshClient.h"
#include "MeshClientDlg.h"
#include "SensorConfig.h"
#include "add_defines.h"
#include "wiced_mesh_client.h"
#include "hci_control_api.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_bt_mesh_model_defs.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_bt_mesh_db.h"
#include "wiced_bt_dev.h"
#include "MeshScanner.h"

// #define MESH_AUTOMATION_ENABLED TRUE

#if defined(MESH_AUTOMATION_ENABLED) && (MESH_AUTOMATION_ENABLED == TRUE)
#include "mesh_automation.h"
#include "mesh_client_script.h"
#endif


//extern "C" void ods(char * fmt_str, ...);

#define PROVISION_TEST_SCAN_A   1
#define PROVISION_TEST_SCAN_B   2
#define PROVISION_TEST_SCAN_C   3
#define PROVISION_TEST_PROV_A   4
#define PROVISION_TEST_PROV_B   5
#define PROVISION_TEST_PROV_C   6
#define PROVISION_TEST_DEL_A    7
#define PROVISION_TEST_DEL_B    8
#define PROVISION_TEST_DEL_C    9

int provision_test = 0; // PROVISION_TEST_SCAN_A;

uint8_t uuid_a[16] = { 0xaa, 0x22, 0x33, 0x33, 0x22, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa };
uint8_t uuid_b[16] = { 0xbb, 0x22, 0x33, 0x33, 0x22, 0xbb, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa };
uint8_t uuid_c[16] = { 0x99, 0x22, 0x33, 0x33, 0x22, 0x99, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa };

DWORD provision_test_scan_unprovisioned_time;
DWORD provision_test_connect_unprovisioned_time;
DWORD provision_test_provision_start_time;
DWORD provision_test_connect_provisioned_time;
DWORD provision_test_config_start_time;
DWORD provision_test_reset_time;

int control_test = 0;
int control_test_itterration = 0;
DWORD control_test_start_time;
DWORD *p_control_test_results;
void control_test_status(const char *name, DWORD duration);

char *log_filename = "trace.txt";  // if you add full path make sure that directory exists, otherwise it will crash

#ifdef _DEBUG
#define new DEBUG_NEW
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

#if !defined(MESH_AUTOMATION_ENABLED)
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
#endif

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

#define IDT_CONTROL_TEST                1
#define IDT_DISTRIBUTION_STATUS         2
#define IDT_PROVISION_TEST              3

#define DISTRIBUTION_STATUS_TIMEOUT     10000

#define DISTRIBUTION_ACTION_START       1
#define DISTRIBUTION_ACTION_STOP        2
#define DISTRIBUTION_ACTION_APPLY       3
#define DISTRIBUTION_ACTION_GET_STATUS  4

#define DFU_STATE_IDLE                  0
#define DFU_STATE_INITIATING            1       // Initiator -> Distributor
#define DFU_STATE_UPDATING              2       // Distributor -> Updating Node
#define DFU_STATE_COMPLETED             3

extern wiced_bool_t mesh_adv_scanner_open();
extern void mesh_adv_scanner_close(void);
extern "C" void mesh_client_advert_report(uint8_t *bd_addr, uint8_t addr_type, int8_t rssi, uint8_t *adv_data);

extern wiced_bool_t mesh_adv_publisher_open();
extern void mesh_adv_publisher_close(void);

char provisioner_uuid[50];


extern BOOL IsOSWin10();

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
extern DWORD __stdcall socketRecv(HWND handle);
#endif

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

extern void Log(WCHAR *fmt, ...)
{
    WCHAR   *msg;
    va_list cur_arg;
    SYSTEMTIME st;
    GetLocalTime(&st);

    if ((msg = (WCHAR *)malloc(2004)) == NULL)
        return;
    memset(msg, 0, sizeof(2004));

    int len = swprintf_s(msg, 1002, L"%02d:%02d:%02d.%03d ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

    va_start(cur_arg, fmt);
    vswprintf(&msg[wcslen(msg)], (sizeof(msg) / sizeof(WCHAR)) - wcslen(msg), fmt, cur_arg);
    va_end(cur_arg);

    if (log_filename != NULL)
    {
        FILE *fp;
        fopen_s(&fp, log_filename, "a");
        if (fp)
        {
            fputws(msg, fp);
            fclose(fp);
        }
    }
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (pDlg)
        pDlg->PostMessage(WM_USER_LOG, 0, (LPARAM)msg);
}

extern "C" void Log(char *fmt, ...)
{
    char   msg[1002];
    WCHAR  *wmsg;
    va_list cur_arg;
    SYSTEMTIME st;
    GetLocalTime(&st);

    if ((wmsg = (WCHAR *)malloc(2004)) == NULL)
        return;

    memset(msg, 0, sizeof(msg));

    int len = sprintf_s(msg, sizeof(msg), "%02d:%02d:%02d.%03d ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

    va_start(cur_arg, fmt);
    vsprintf(&msg[len], fmt, cur_arg);
    va_end(cur_arg);

    MultiByteToWideChar(CP_UTF8, 0, msg, -1, wmsg, 1002);

    if (log_filename != NULL)
    {
        FILE *fp;
        fopen_s(&fp, log_filename, "a");
        wcscat(wmsg, L"\n");
        if (fp)
        {
            fputws(wmsg, fp);
            fclose(fp);
        }
    }

    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (pDlg)
        pDlg->PostMessage(WM_USER_LOG, 0, (LPARAM)wmsg);
}

LRESULT CMeshClientDlg::OnUserLog(WPARAM wparam, LPARAM lparam)
{
    WCHAR *wmsg = (WCHAR *)lparam;
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (pDlg)
        pDlg->m_trace->SetCurSel(pDlg->m_trace->AddString(wmsg));
    free(wmsg);
    return S_OK;
}

extern "C" CRITICAL_SECTION cs;
CRITICAL_SECTION cs;

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
    CAboutDlg();

    // Dialog Data
    enum { IDD = IDD_ABOUTBOX };

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

                                                        // Implementation
protected:
    DECLARE_MESSAGE_MAP()
};

const WCHAR* data2str(const UINT8* p_data, UINT32 len)
{
    UINT32 i;
    static WCHAR buf[32 * 3 + 4]; // 32 hex bytes with space delimiter and with terminating 0 and with possible "..."
    buf[0] = 0;

    for (i = 0; i < len && i < sizeof(buf) / sizeof(buf[0]) - 4; i++)
    {
        wsprintf(&buf[i * 3], L"%02x ", p_data[i]);
    }
    if (i < len)
        wcscpy_s(&buf[i * 3], sizeof(buf) / sizeof(buf[0]) - (i * 3), L"...");
    return buf;
}

const WCHAR* keyidx2str(const UINT8* p_data, UINT32 len)
{
    static WCHAR buf[8 * 4 + 4]; // 8 key indexes(3 characters each) with space delimiter and with terminating 0 and with possible "..."
    WCHAR *p_out = buf;
    buf[0] = 0;

    while (len > 1)
    {
        if ((p_out - buf) >= (sizeof(buf) / sizeof(buf[0]) - 4))
            break;
        wsprintf(p_out, L"%03x ", LE2TOUINT12(p_data));
        p_out += 4;
        p_data++;
        len--;
        if (len < 2)
            break;
        if ((p_out - buf) >= (sizeof(buf) / sizeof(buf[0]) - 4))
            break;
        wsprintf(p_out, L"%03x ", LE2TOUINT12_2(p_data));
        p_out += 4;
        p_data += 2;
        len -= 2;
    }
    if (len > 1)
        wcscpy_s(p_out, sizeof(buf) / sizeof(buf[0]) - (p_out - buf), L"...");
    return buf;
}

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// CMeshClientDlg dialog
CMeshClientDlg::CMeshClientDlg(CWnd* pParent /*=NULL*/)
    : CDialogEx(CMeshClientDlg::IDD, pParent)
{

    InitializeCriticalSection(&cs);
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
    m_szCurrentGroup[0] = 0;
    m_pDownloader = NULL;
    m_btInterface = NULL;
    m_hCfgEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    FILE *fp = fopen("NetParameters.bin", "rb");
    if (fp)
    {
        fread(&DeviceConfig, 1, sizeof(DeviceConfig), fp);
        fclose(fp);
    }
    m_bConnected = FALSE;
    m_pPatch = NULL;
    m_dwPatchSize = 0;
    m_bConnecting = FALSE;
    m_bScanning = FALSE;
    m_dfuState = DFU_STATE_IDLE;
}

CMeshClientDlg::~CMeshClientDlg()
{
    DeleteCriticalSection(&cs);
    delete m_btInterface;
}

void CMeshClientDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_PROGRESS, m_Progress);
}

BEGIN_MESSAGE_MAP(CMeshClientDlg, CDialogEx)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_WM_TIMER()

    ON_MESSAGE(WM_USER_PROXY_DATA, OnProxyDataIn)
    ON_MESSAGE(WM_USER_PROVISIONING_DATA, OnProvisioningDataIn)

    ON_MESSAGE(WM_USER_LOG, OnUserLog)
    ON_MESSAGE(WM_TIMER_CALLBACK, OnTimerCallback)
    ON_MESSAGE(WM_USER_WS_UPGRADE_CTRL_POINT, &CMeshClientDlg::OnWsUpgradeCtrlPoint)
    ON_MESSAGE(WM_MESH_DEVICE_DISCONNECTED, &CMeshClientDlg::OnMeshDeviceDisconnected)
    ON_MESSAGE(WM_MESH_DEVICE_CONNECT, &CMeshClientDlg::OnMeshDeviceConnect)
    ON_MESSAGE(WM_MESH_DEVICE_DISCONNECT, &CMeshClientDlg::OnMeshDeviceDisconnect)
    ON_MESSAGE(WM_MESH_DEVICE_ADV_REPORT, &CMeshClientDlg::OnMeshDeviceAdvReport)

#if defined(MESH_AUTOMATION_ENABLED) && (MESH_AUTOMATION_ENABLED == TRUE)
    ON_MESSAGE(WM_SOCKET, &CMeshClientDlg::OnSocketMessage)
#endif

    ON_MESSAGE(WM_PROGRESS, OnProgress)

    ON_WM_CLOSE()
    ON_BN_CLICKED(IDC_CLEAR_TRACE, &CMeshClientDlg::OnBnClickedClearTrace)
    ON_BN_CLICKED(IDC_SCAN_UNPROVISIONED, &CMeshClientDlg::OnBnClickedScanUnprovisioned)
    ON_BN_CLICKED(IDC_PROVISION, &CMeshClientDlg::OnBnClickedProvision)
    ON_BN_CLICKED(IDC_NETWORK_CREATE, &CMeshClientDlg::OnBnClickedNetworkCreate)
    ON_BN_CLICKED(IDC_NETWORK_DELETE, &CMeshClientDlg::OnBnClickedNetworkDelete)
    ON_BN_CLICKED(IDC_NETWORK_OPEN, &CMeshClientDlg::OnBnClickedNetworkOpen)
    ON_BN_CLICKED(IDC_NETWORK_CLOSE, &CMeshClientDlg::OnBnClickedNetworkClose)
    ON_BN_CLICKED(IDC_GROUP_CREATE, &CMeshClientDlg::OnBnClickedGroupCreate)
    ON_BN_CLICKED(IDC_GROUP_DELETE, &CMeshClientDlg::OnBnClickedGroupDelete)
    ON_BN_CLICKED(IDC_NODE_RESET, &CMeshClientDlg::OnBnClickedNodeReset)
    ON_CBN_SELCHANGE(IDC_NETWORK, &CMeshClientDlg::OnSelchangeNetwork)
    ON_CBN_SELCHANGE(IDC_CURRENT_GROUP, &CMeshClientDlg::OnSelchangeCurrentGroup)
    ON_BN_CLICKED(IDC_CONFIGURE_MOVE, &CMeshClientDlg::OnBnClickedMoveToGroup)
    ON_BN_CLICKED(IDC_CONFIGURE_PUB, &CMeshClientDlg::OnBnClickedConfigurePub)
    ON_BN_CLICKED(IDC_ON_OFF_GET, &CMeshClientDlg::OnBnClickedOnOffGet)
    ON_BN_CLICKED(IDC_ON_OFF_SET, &CMeshClientDlg::OnBnClickedOnOffSet)
    ON_BN_CLICKED(IDC_LEVEL_GET, &CMeshClientDlg::OnBnClickedLevelGet)
    ON_BN_CLICKED(IDC_LEVEL_SET, &CMeshClientDlg::OnBnClickedLevelSet)
    ON_BN_CLICKED(IDC_LIGHT_HSL_GET, &CMeshClientDlg::OnBnClickedLightHslGet)
    ON_BN_CLICKED(IDC_LIGHT_HSL_SET, &CMeshClientDlg::OnBnClickedLightHslSet)
    ON_BN_CLICKED(IDC_VS_DATA, &CMeshClientDlg::OnBnClickedVsData)
    ON_BN_CLICKED(IDC_LIGHT_CTL_GET, &CMeshClientDlg::OnBnClickedLightCtlGet)
    ON_BN_CLICKED(IDC_LIGHT_CTL_SET, &CMeshClientDlg::OnBnClickedLightCtlSet)
    ON_BN_CLICKED(IDC_LIGHTNESS_GET, &CMeshClientDlg::OnBnClickedLightnessGet)
    ON_BN_CLICKED(IDC_LIGHTNESS_SET, &CMeshClientDlg::OnBnClickedLightnessSet)
    ON_BN_CLICKED(IDC_CONNECTDISCONNECT, &CMeshClientDlg::OnBnClickedConnectdisconnect)
    ON_BN_CLICKED(IDC_OTA_UPGRADE_START, &CMeshClientDlg::OnBnClickedOtaUpgradeStart)
    ON_BN_CLICKED(IDC_IDENTIFY, &CMeshClientDlg::OnBnClickedIdentify)
    ON_BN_CLICKED(IDC_RECONFIGURE, &CMeshClientDlg::OnBnClickedReconfigure)
    ON_BN_CLICKED(IDC_BROWSE, &CMeshClientDlg::OnBnClickedBrowse)
    ON_BN_CLICKED(IDC_INFOBROWSE, &CMeshClientDlg::OnBnClickedInfoBrowse)
    ON_CBN_SELCHANGE(IDC_CONFIGURE_CONTROL_DEVICE, &CMeshClientDlg::OnCbnSelchangeConfigureControlDevice)
    ON_BN_CLICKED(IDC_NETWORK_IMPORT, &CMeshClientDlg::OnBnClickedNetworkImport)
    ON_BN_CLICKED(IDC_NETWORK_EXPORT, &CMeshClientDlg::OnBnClickedNetworkExport)
    ON_BN_CLICKED(IDC_GET_COMPONENT_INFO, &CMeshClientDlg::OnBnClickedGetComponentInfo)
    ON_BN_CLICKED(IDC_OTA_UPGRADE_STATUS, &CMeshClientDlg::OnBnClickedOtaUpgradeStatus)
    ON_BN_CLICKED(IDC_OTA_UPGRADE_STOP, &CMeshClientDlg::OnBnClickedOtaUpgradeStop)
    ON_BN_CLICKED(IDC_OTA_UPGRADE_APPLY, &CMeshClientDlg::OnBnClickedOtaUpgradeApply)
    ON_BN_CLICKED(IDC_SENSOR_GET, &CMeshClientDlg::OnBnClickedSensorGet)
    ON_CBN_SELCHANGE(IDC_CONTROL_DEVICE, &CMeshClientDlg::OnCbnSelchangeControlDevice)
    ON_BN_CLICKED(IDC_SENSOR_CONFIGURE, &CMeshClientDlg::OnBnClickedSensorConfigure)
END_MESSAGE_MAP()

void CMeshClientDlg::OnClose()
{
    mesh_client_network_close();
    Sleep(1000);

    mesh_adv_scanner_close();
    mesh_adv_publisher_close();

    // Save main window position
    WINDOWPLACEMENT wp;
    GetWindowPlacement(&wp);
    AfxGetApp()->WriteProfileBinary((LPCTSTR)"MainFrame", (LPCTSTR)"WP", (LPBYTE)&wp, sizeof(wp));

    CDialogEx::OnClose();
}

void CMeshClientDlg::OnCancel()
{
    mesh_client_network_close();
    Sleep(1000);

    CDialogEx::OnCancel();
}

// CMeshClientDlg message handlers

// Callback function to send provisioning packet.
extern "C" void mesh_provision_gatt_send(uint16_t conn_id, const uint8_t *packet, uint32_t packet_len)
{
    BOOL res = TRUE;
    BTW_GATT_VALUE gatt_value;
    gatt_value.len = packet_len;
    memcpy(gatt_value.value, packet, packet_len);
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    EnterCriticalSection(&cs);
    if ((pDlg != NULL) && (pDlg->m_btInterface != NULL))
        res = pDlg->m_btInterface->WriteCharacteristic(&guidSvcMeshProvisioning, &guidCharProvisioningDataIn, TRUE, &gatt_value);
    LeaveCriticalSection(&cs);
    if (!res)
        ods("provision_gatt_send_cb: WriteCharacteristic failed.");
}

// Callback function to send a packet over GATT connection using GATT Write Command for gatt_char_handle parameter.
extern "C" void proxy_gatt_send_cb(uint32_t conn_id, uint32_t ref_data, const uint8_t* data, uint32_t data_len)
{
    BTW_GATT_VALUE gatt_value;
    gatt_value.len = data_len;
    memcpy(gatt_value.value, data, data_len);
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    EnterCriticalSection(&cs);
    if ((pDlg != NULL) && (pDlg->m_btInterface != NULL))
    {
        if (!pDlg->m_btInterface->WriteCharacteristic(&guidSvcMeshProxy, &guidCharProxyDataIn, TRUE, &gatt_value))
        {
            LeaveCriticalSection(&cs);
            ods("proxy_gatt_send_cb: WriteCharacteristic failed.");
        }
        else
            LeaveCriticalSection(&cs);
    }
    else
    {
        LeaveCriticalSection(&cs);
        ods("proxy_gatt_send_cb: WriteCharacteristic no connection failed.");
    }
}

// Callback function to send a packet over GATT connection using GATT Write Command for gatt_char_handle parameter.
extern "C" void onProxyGattPktReceivedCallback(const uint8_t* data, uint32_t data_len)
{
    BOOL res = TRUE;
    BTW_GATT_VALUE gatt_value;
    gatt_value.len = data_len;
    memcpy(gatt_value.value, data, data_len);
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    EnterCriticalSection(&cs);
    if ((pDlg != NULL) && (pDlg->m_btInterface != NULL))
        res = pDlg->m_btInterface->WriteCharacteristic(&guidSvcMeshProxy, &guidCharProxyDataIn, TRUE, &gatt_value);
    LeaveCriticalSection(&cs);
    if (!res)
        ods("proxy_gatt_send_cb: WriteCharacteristic failed.");
}

// Callback function to send a packet over GATT connection using GATT Write Command for gatt_char_handle parameter.
extern "C" void onProvGattPktReceivedCallback(const uint8_t* data, uint32_t data_len)
{
    BOOL res = TRUE;
    BTW_GATT_VALUE gatt_value;
    gatt_value.len = data_len;
    memcpy(gatt_value.value, data, data_len);
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    EnterCriticalSection(&cs);
    if ((pDlg != NULL) && (pDlg->m_btInterface != NULL))
        res = pDlg->m_btInterface->WriteCharacteristic(&guidSvcMeshProvisioning, &guidCharProxyDataIn, TRUE, &gatt_value);
    LeaveCriticalSection(&cs);
    if (!res)
        ods("proxy_gatt_send_cb: WriteCharacteristic failed.");
}

void CMeshClientDlg::SetDlgItemHex(DWORD id, DWORD val)
{
    WCHAR buf[10];
    wsprintf(buf, L"%x", val);
    SetDlgItemText(id, buf);
}

BOOL CMeshClientDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    srand((unsigned)time(NULL));

    // Add "About..." menu item to system menu.

    // IDM_ABOUTBOX must be in the system command range.
    ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
    ASSERT(IDM_ABOUTBOX < 0xF000);

    CMenu* pSysMenu = GetSystemMenu(FALSE);
    if (pSysMenu != NULL)
    {
        BOOL bNameValid;
        CString strAboutMenu;
        bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
        ASSERT(bNameValid);
        if (!strAboutMenu.IsEmpty())
        {
            pSysMenu->AppendMenu(MF_SEPARATOR);
            pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
        }
    }

    // Set the icon for this dialog.  The framework does this automatically
    //  when the application's main window is not a dialog
    SetIcon(m_hIcon, TRUE);         // Set big icon
    SetIcon(m_hIcon, FALSE);        // Set small icon

    m_trace = (CListBox *)GetDlgItem(IDC_TRACE);

    m_btInterface = NULL;

    SetDlgItemHex(IDC_IDENTITY_DURATION, 1);

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

    SetDlgItemInt(IDC_NUM_ITTERRATIONS, 1);

    // SetDlgItemText(IDC_FILENAME, L"C:\\Users\\vicz\\Documents\\WICED-Studio-6.2\\20719-B1_Bluetooth\\build\\mesh_light_hsl_ctl_server-CYW920719Q40EVB_01-rom-ram-Wiced-release\\mesh_light_hsl_ctl_server-CYW920719Q40EVB_01-rom-ram-Wiced-release.ota.bin");
    CComboBox *pCb = (CComboBox *)GetDlgItem(IDC_DFU_METHOD);
    for (int i = 0; i < ((sizeof(dfuMethods)) / sizeof(dfuMethods[0])); i++)
        pCb->AddString(dfuMethods[i]);
    pCb->SetCurSel(0);

    mesh_adv_scanner_open();

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // MESH_AUTOMATION

    socketRecv(theApp.m_pMainWnd->m_hWnd);

    Sleep(50);
#endif
    return TRUE;  // return TRUE  unless you set the focus to a control
}

void CMeshClientDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
    if ((nID & 0xFFF0) == IDM_ABOUTBOX)
    {
        CAboutDlg dlgAbout;
        dlgAbout.DoModal();
    }
    else
    {
        CDialogEx::OnSysCommand(nID, lParam);
    }
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CMeshClientDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this); // device context for painting

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // Center icon in client rectangle
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        // Draw the icon
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
#if 0
        WINDOWPLACEMENT *lwp;
        UINT nl;

        if (AfxGetApp()->GetProfileBinary((LPCTSTR)"MainFrame", (LPCTSTR)"WP", (LPBYTE*)&lwp, &nl))
        {
            // make sure the window is not completely out of sight
            int max_x = GetSystemMetrics(SM_CXSCREEN) -
                GetSystemMetrics(SM_CXICON);
            int max_y = GetSystemMetrics(SM_CYSCREEN) -
                GetSystemMetrics(SM_CYICON);

            int width = lwp->rcNormalPosition.right - lwp->rcNormalPosition.left;
            int height = lwp->rcNormalPosition.bottom - lwp->rcNormalPosition.top;

            if (lwp->rcNormalPosition.left > max_x || lwp->rcNormalPosition.top > max_y ||
                lwp->rcNormalPosition.left < 0 || lwp->rcNormalPosition.top < 0)
            {
                lwp->rcNormalPosition.left = (max_x / 4);
                lwp->rcNormalPosition.top = (max_y / 4);
                lwp->rcNormalPosition.right = lwp->rcNormalPosition.left + width;
                lwp->rcNormalPosition.bottom = lwp->rcNormalPosition.top + height;
            }

            SetWindowPlacement(lwp);
            delete[] lwp;
        }
#endif
        CDialogEx::OnPaint();
    }
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CMeshClientDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}

void CMeshClientDlg::PostNcDestroy()
{
    CDialogEx::PostNcDestroy();
}

LRESULT CMeshClientDlg::OnProxyDataIn(WPARAM Instance, LPARAM lparam)
{
    BTW_GATT_VALUE *pValue = (BTW_GATT_VALUE *)lparam;
    EnterCriticalSection(&cs);
    mesh_client_proxy_data(pValue->value, pValue->len);
    LeaveCriticalSection(&cs);
    free(pValue);
    return S_OK;
}

BYTE ProcNibble(char n)
{
    if ((n >= '0') && (n <= '9'))
    {
        n -= '0';
    }
    else if ((n >= 'A') && (n <= 'F'))
    {
        n = ((n - 'A') + 10);
    }
    else if ((n >= 'a') && (n <= 'f'))
    {
        n = ((n - 'a') + 10);
    }
    else
    {
        n = (char)0xff;
    }
    return (n);
}

void CMeshClientDlg::SetHexValueInt(DWORD id, DWORD val, DWORD val_len)
{
    BYTE val_hex[4];
    if (val_len > 0)
    {
        if (val_len > 4)
            val_len = 4;
        for (int i = val_len - 1; i >= 0; i--)
        {
            val_hex[i] = (BYTE)(val);
            val = val >> 8;
        }
    }
    SetHexValue(id, val_hex, val_len);
}

void CMeshClientDlg::SetHexValue(DWORD id, LPBYTE val, DWORD val_len)
{
    WCHAR str[1024];
    for (DWORD i = 0; i < val_len; i++)
        wsprintf(&str[3 * i], L"%02X ", val[i]);
    SetDlgItemText(id, str);
}

DWORD CMeshClientDlg::GetHexValue(char *szbuf, LPBYTE buf, DWORD buf_size)
{
    BYTE *pbuf = buf;
    DWORD res = 0;

    // remove leading white space
    while (*szbuf != 0 && (BYTE)*szbuf <= 0x20) szbuf++;
    DWORD len = (DWORD)strlen(szbuf);
    // remove terminating white space
    while (len > 0 && (BYTE)szbuf[len - 1] <= 0x20) len--;

    memset(buf, 0, buf_size);

    if (len == 1)
    {
        szbuf[2] = 0;
        szbuf[1] = szbuf[0];
        szbuf[0] = '0';
    }
    else if (len == 3)
    {
        szbuf[4] = 0;
        szbuf[3] = szbuf[2];
        szbuf[2] = szbuf[1];
        szbuf[1] = szbuf[0];
        szbuf[0] = '0';
    }
    for (DWORD i = 0; (i < len) && (res < buf_size); i++)
    {
        if (isxdigit(szbuf[i]) && isxdigit(szbuf[i + 1]))
        {
            *pbuf++ = (ProcNibble((char)szbuf[i]) << 4) + ProcNibble((char)szbuf[i + 1]);
            res++;
            i++;
        }
    }
    return res;
}

DWORD CMeshClientDlg::GetHexValueInt(DWORD id)
{
    DWORD ret = 0;
    BYTE buf[32];
    DWORD len = GetHexValue(id, buf, sizeof(buf));
    for (DWORD i = 0; i < len; i++)
    {
        ret = (ret << 8) + buf[i];
    }
    return ret;
}

DWORD CMeshClientDlg::GetHexValue(DWORD id, LPBYTE buf, DWORD buf_size)
{
    WCHAR wszbuf[100];
    char szbuf[100];

    GetDlgItemText(id, wszbuf, sizeof(wszbuf) / sizeof(wszbuf[0]));
    WideCharToMultiByte(CP_ACP, 0, wszbuf, -1, szbuf, sizeof(szbuf), NULL, NULL);
    return GetHexValue(szbuf, buf, buf_size);
}

void CMeshClientDlg::GetDlgItemTextUTF8(int id, char* buf, DWORD buf_len)
{
    CString str;
    GetDlgItemText(id, str);
    WideCharToMultiByte(CP_UTF8, 0, str, -1, buf, buf_len, NULL, NULL);
}

// handles packet received from provisioning characteristic
LRESULT CMeshClientDlg::OnProvisioningDataIn(WPARAM Instance, LPARAM lparam)
{
    BTW_GATT_VALUE *pValue = (BTW_GATT_VALUE *)lparam;
    // we started provisioning with hardcoded conn_id=1 in call to wiced_bt_mesh_provision_start()
    EnterCriticalSection(&cs);
    mesh_client_provisioning_data(1, pValue->value, pValue->len);
    LeaveCriticalSection(&cs);
    free(pValue);
    return S_OK;
}

void CMeshClientDlg::OnTimer(UINT_PTR nIDEvent)
{
    //wiced_timer_handle(nIDEvent);
    char name[80];
    CDialogEx::OnTimer(nIDEvent);

    if ((provision_test == PROVISION_TEST_SCAN_A) || (provision_test == PROVISION_TEST_SCAN_B) || (provision_test == PROVISION_TEST_SCAN_C))
    {
        KillTimer(IDT_PROVISION_TEST);
        PostMessage(WM_COMMAND, IDC_PROVISION, 0);
    }

#if 1
    switch (nIDEvent)
    {
    case IDT_CONTROL_TEST:
    if (control_test != 0)
        GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
        control_test_status(name, 0);
        break;

    case IDT_DISTRIBUTION_STATUS:
        EnterCriticalSection(&cs);
        mesh_client_dfu_get_status(NULL, 0, NULL, 0, &fw_distribution_status);
        LeaveCriticalSection(&cs);
        break;
    }
#endif
#if 0
    else
    {
        static short target_lightness = 0;
        static SHORT target_hue = 0;
        static SHORT target_saturation = 0;
        if (target_lightness == 0)
        {
            target_lightness = (SHORT)GetDlgItemInt(IDC_LIGHT_LIGHTNESS_TARGET);
            target_hue = (SHORT)GetDlgItemInt(IDC_LIGHT_HSL_HUE_VALUE);
            target_saturation = (SHORT)GetDlgItemInt(IDC_LIGHT_HSL_SATURATION_VALUE);
        }
        else
        {
            target_lightness += 10;
            target_hue += 10;
            target_saturation += 10;
        }
        char name[80];
        GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
        EnterCriticalSection(&cs);
        mesh_client_hsl_set(name, target_lightness, target_hue, target_saturation, WICED_FALSE, 250 /*DEFAULT_TRANSITION_TIME*/, 0);
        LeaveCriticalSection(&cs);
    }
#endif
}

void CMeshClientDlg::Disconnect()
{
    if (m_btInterface)
    {
        CBtWin10Interface *pWin10BtInterface = dynamic_cast<CBtWin10Interface *>(m_btInterface);
        pWin10BtInterface->ResetInterface();
        delete m_btInterface;
        m_btInterface = NULL;
    }
    EnterCriticalSection(&cs);
    mesh_client_connection_state_changed(0, 0);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::trace(char * fmt_str, ...)
{
    char buf[1000] = { 0 };
    va_list marker = NULL;
    va_start(marker, fmt_str);
    vsnprintf_s(buf, sizeof(buf), _TRUNCATE, fmt_str, marker);
    va_end(marker);
    strcat_s(buf, sizeof(buf), "\n");
    m_trace->SetCurSel(m_trace->AddString(CA2W(buf)));
}

void CMeshClientDlg::OnBnClickedClearTrace()
{
    m_trace->ResetContent();
}

void CMeshClientDlg::ProcessUnprovisionedDevice(uint8_t *p_uuid, uint16_t oob, uint8_t *name, uint8_t name_len)
{
    WCHAR buf[180];
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

    if (m_bScanning && provision_test)
    {
        if (((provision_test == PROVISION_TEST_SCAN_A) && (memcmp(p_uuid, uuid_a, 16) == 0))
          || ((provision_test == PROVISION_TEST_SCAN_B) && (memcmp(p_uuid, uuid_b, 16) == 0))
          || ((provision_test == PROVISION_TEST_SCAN_C) && (memcmp(p_uuid, uuid_c, 16) == 0)))
        {
            PostMessage(WM_COMMAND, IDC_SCAN_UNPROVISIONED, 0);
            SetTimer(IDT_PROVISION_TEST, 1000, NULL);
        }
    }
}

void CMeshClientDlg::LinkStatus(uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt)
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

void CMeshClientDlg::OnBnClickedScanUnprovisioned()
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

    EnterCriticalSection(&cs);
    mesh_client_scan_unprovisioned(m_bScanning);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedProvision()
{
    uint8_t identify_duration = (BYTE)GetHexValueInt(IDC_IDENTITY_DURATION);

    uint8_t uuid[16];
    if (provision_test == PROVISION_TEST_SCAN_A)
        memcpy(uuid, uuid_a, 16);
    else if (provision_test == PROVISION_TEST_SCAN_B)
            memcpy(uuid, uuid_b, 16);
    else if (provision_test == PROVISION_TEST_SCAN_C)
        memcpy(uuid, uuid_c, 16);
    else
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

    EnterCriticalSection(&cs);
    mesh_client_set_device_config(NULL, DeviceConfig.is_gatt_proxy, DeviceConfig.is_friend, DeviceConfig.is_relay, DeviceConfig.send_net_beacon, DeviceConfig.relay_xmit_count, DeviceConfig.relay_xmit_interval, DeviceConfig.default_ttl, DeviceConfig.net_xmit_count, DeviceConfig.net_xmit_interval);
    mesh_client_set_publication_config(DeviceConfig.publish_credential_flag, DeviceConfig.publish_retransmit_count, DeviceConfig.publish_retransmit_interval, DeviceConfig.publish_ttl);

    mesh_client_provision(node_name, group_name, uuid, identify_duration);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedNetworkCreate()
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

void CMeshClientDlg::OnBnClickedNetworkDelete()
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

void CMeshClientDlg::OnBnClickedNetworkOpen()
{
    char mesh_name[80], provisioner_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NETWORK, mesh_name, sizeof(mesh_name));
    GetDlgItemTextA(m_hWnd, IDC_PROVISIONER, provisioner_name, sizeof(provisioner_name));
    EnterCriticalSection(&cs);
    if (mesh_client_network_open(provisioner_name, provisioner_uuid, mesh_name, network_opened) != MESH_CLIENT_SUCCESS)
    {
        LeaveCriticalSection(&cs);
        MessageBoxA(m_hWnd, mesh_name, "Network Does Not Exists", MB_ICONERROR);
        return;
    }
    LeaveCriticalSection(&cs);

    WCHAR szGroup[80];
    MultiByteToWideChar(CP_UTF8, 0, mesh_name, strlen(mesh_name) + 1, szGroup, sizeof(szGroup) / sizeof(WCHAR));
    wcscpy(m_szCurrentGroup, szGroup);

    DisplayCurrentGroup();
}

void CMeshClientDlg::OnSelchangeCurrentGroup()
{
    CComboBox *p_current_group = (CComboBox *)GetDlgItem(IDC_CURRENT_GROUP);
    int sel = p_current_group->GetCurSel();
    if (sel < 0)
        return;

    p_current_group->GetLBText(sel, m_szCurrentGroup);
    DisplayCurrentGroup();
}

void CMeshClientDlg::OnBnClickedNetworkClose()
{
    EnterCriticalSection(&cs);
    mesh_client_network_close();
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::DisplayCurrentGroup()
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

void CMeshClientDlg::OnBnClickedGroupCreate()
{
    int res;
    char mesh_name[80];
    GetDlgItemTextA(m_hWnd, IDC_NETWORK, mesh_name, sizeof(mesh_name));
    char group_name[80], parent_group_name[80];
    GetDlgItemTextA(m_hWnd, IDC_GROUP_NAME, group_name, sizeof(group_name));
    GetDlgItemTextA(m_hWnd, IDC_CURRENT_GROUP, parent_group_name, sizeof(parent_group_name));

    SetDlgItemText(IDC_GROUP_NAME, L"");

    EnterCriticalSection(&cs);
    res = mesh_client_group_create(group_name, parent_group_name);
    LeaveCriticalSection(&cs);

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

void CMeshClientDlg::OnBnClickedGroupDelete()
{
    char group_name[80];
    GetDlgItemTextA(m_hWnd, IDC_CURRENT_GROUP, group_name, sizeof(group_name));
    EnterCriticalSection(&cs);
    mesh_client_group_delete(group_name);
    LeaveCriticalSection(&cs);
    DisplayCurrentGroup();
}

void CMeshClientDlg::ProvisionCompleted()
{
    DisplayCurrentGroup();
    if (provision_test == 0)
        return;
    if (provision_test == PROVISION_TEST_SCAN_A)
    {
        provision_test = PROVISION_TEST_SCAN_B;
        PostMessage(WM_COMMAND, IDC_SCAN_UNPROVISIONED, 0);
        return;
    }
    else if (provision_test == PROVISION_TEST_SCAN_B)
    {
#if 0
        // provision completed of the device B.  Send a backdoor command to factory reset.
        EnterCriticalSection(&cs);
        const GUID guidSrvc = { 0x7FD3, 0, 0x1000, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB };
        const GUID guidChar = { 0x7FCB, 0, 0x1000, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB };
        BTW_GATT_VALUE gatt_value;
        gatt_value.len = 1;
        gatt_value.value[0] = 4;
        m_btInterface->WriteCharacteristic(&guidSrvc, &guidChar, TRUE, &gatt_value);
        LeaveCriticalSection(&cs);
#endif
        provision_test = PROVISION_TEST_SCAN_C;
        PostMessage(WM_COMMAND, IDC_SCAN_UNPROVISIONED, 0);
        return;
    }
    else if (provision_test == PROVISION_TEST_SCAN_C)
    {
        provision_test = PROVISION_TEST_DEL_A;
        PostMessage(WM_COMMAND, IDC_NODE_RESET, 0);
    }
}

void network_opened(uint8_t status)
{
    Log(L"Network opened\n");
}

typedef void (TIMER_CBACK)(void *p_tle);
#define TIMER_PARAM_TYPE    void *
extern "C" void execute_timer_callback(TIMER_CBACK *p_callback, TIMER_PARAM_TYPE arg);

void execute_timer_callback(TIMER_CBACK *p_callback, TIMER_PARAM_TYPE arg)
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    pDlg->PostMessage(WM_TIMER_CALLBACK, (WPARAM)p_callback, (LPARAM)arg);
}

LRESULT CMeshClientDlg::OnTimerCallback(WPARAM op, LPARAM lparam)
{
    TIMER_CBACK *p_cback = (TIMER_CBACK *)op;
    p_cback((TIMER_PARAM_TYPE)lparam);
    return S_OK;
}

void unprovisioned_device(uint8_t *p_uuid, uint16_t oob, uint8_t *name, uint8_t name_len)
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    pDlg->ProcessUnprovisionedDevice(p_uuid, oob, name, name_len);

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
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    pDlg->LinkStatus(is_connected, conn_id, addr, is_over_gatt);

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
    size_t name_len = p_device_name ? strlen(p_device_name) + 1 : 0;
    MultiByteToWideChar(CP_UTF8, 0, p_device_name, -1, szDevName, sizeof(szDevName) / sizeof(WCHAR));
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_NODE_CONNECT_STATUS node_connect_status = { 0 };
    node_connect_status.status = status;
    if (p_device_name && name_len)
    {
        memcpy(&node_connect_status.name, p_device_name, name_len);
    }
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_NODE_CONNECT_STATUS, &node_connect_status, sizeof(node_connect_status));

#endif
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (pDlg == NULL)
        return;

    switch (status)
    {
    case MESH_CLIENT_NODE_CONNECTED:
        wsprintf(buf, L"Node %s connected continue OTA upgrade\n", szDevName);
        Log(buf);
        pDlg->OnNodeConnected();
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

    if (status != MESH_CLIENT_PROVISION_STATUS_SUCCESS)
        return;

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
    free(p_devices);

    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (pDlg != NULL)
        pDlg->ProvisionCompleted();

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
    // Hook location where callback received from the mesh core is queued and forwarded to the Mesh Automation Script
    tMESH_CLIENT_SCRIPT_PROVISION_STATUS provision_status = { 0 };
    provision_status.status = status;
    if (p_uuid)
    {
        memcpy(&provision_status.uuid[0], p_uuid, sizeof(provision_status.uuid));
    }
    mesh_client_enqueue_and_check_event(MESH_CLIENT_SCRIPT_EVT_PROVISION_STATUS, &provision_status, sizeof(provision_status));
#endif


}


void database_changed(char *mesh_name)
{
    Log(L"database changed\n");
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    CComboBox *p_target_devs_groups = (CComboBox *)pDlg->GetDlgItem(IDC_CONTROL_DEVICE);

    if ((provision_test != PROVISION_TEST_DEL_B) && (provision_test != PROVISION_TEST_DEL_C))
        return;

    int count = p_target_devs_groups->GetCount();
    for (int i = 0; i < count; i++)
    {
        char name[80];
        WCHAR szDevName[80];
        p_target_devs_groups->GetLBText(i, szDevName);
        WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, name, 80, NULL, FALSE);
        if ((wcsstr(szDevName, L"0003") != NULL) || (wcsstr(szDevName, L"0004") != NULL))
        {
            pDlg->PostMessage(WM_COMMAND, IDC_NODE_RESET, 0);
            return;
        }
    }
    provision_test = PROVISION_TEST_SCAN_A;
    pDlg->PostMessage(WM_COMMAND, IDC_SCAN_UNPROVISIONED, 0);
}

void control_test_status(const char *name, DWORD duration)
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;

    if (control_test != 0)
    {
        KillTimer(pDlg->m_hWnd, 1);
        if (p_control_test_results != NULL)
        {
            p_control_test_results[control_test_itterration++] = duration;
        }
        if (--control_test == 0)
        {
            WCHAR buf[1000];
            buf[0] = 0;
            for (int i = 0; i < control_test_itterration; i++)
                wsprintf(&buf[wcslen(buf)], L"%d ", p_control_test_results[i]);

            Log(L"Control Test result %s\n", buf);
        }
        else
        {
            SetTimer(pDlg->m_hWnd, IDT_CONTROL_TEST, 5000, NULL);
            pDlg->OnOffGet();
        }
    }
}

void onoff_status(const char *device_name, uint8_t present, uint8_t target, uint32_t remaining_time)
{
    WCHAR szDevName[80];
    size_t name_len = device_name ? strlen(device_name) + 1 : 0;
    MultiByteToWideChar(CP_UTF8, 0, device_name, -1, szDevName, sizeof(szDevName) / sizeof(WCHAR));
    Log(L"%s OnOff state:%d\n", szDevName, present);
    DWORD duration = GetTickCount() - control_test_start_time;

    control_test_status(device_name, duration);

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

void CMeshClientDlg::OnBnClickedNodeReset()
{
    CComboBox *p_target_devs_groups = (CComboBox *)GetDlgItem(IDC_CONTROL_DEVICE);
    int count = p_target_devs_groups->GetCount();
    char name[80];
    WCHAR szDevName[80];

    if (provision_test == PROVISION_TEST_DEL_A)
    {
        for (int i = 0; i < count; i++)
        {
            p_target_devs_groups->GetLBText(i, szDevName);
            WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, name, 80, NULL, FALSE);
            if (wcsstr(szDevName, L"0002") != NULL)
            {
                p_target_devs_groups->SetCurSel(i);
                provision_test = PROVISION_TEST_DEL_B;
                break;
            }
        }
    }
    else if (provision_test == PROVISION_TEST_DEL_B)
    {
        for (int i = 0; i < count; i++)
        {
            p_target_devs_groups->GetLBText(i, szDevName);
            WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, name, 80, NULL, FALSE);
            if (wcsstr(szDevName, L"0003") != NULL)
            {
                p_target_devs_groups->SetCurSel(i);
                provision_test = PROVISION_TEST_DEL_C;
                break;
            }
        }
    }
    else if (provision_test == PROVISION_TEST_DEL_C)
    {
        for (int i = 0; i < count; i++)
        {
            p_target_devs_groups->GetLBText(i, szDevName);
            WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, name, 80, NULL, FALSE);
            if (wcsstr(szDevName, L"0004") != NULL)
            {
                p_target_devs_groups->SetCurSel(i);
                break;
            }
        }
    }
    int sel = p_target_devs_groups->GetCurSel();
    if (sel >= 0)
    {
        p_target_devs_groups->GetLBText(sel, szDevName);
        WideCharToMultiByte(CP_UTF8, 0, szDevName, -1, name, 80, NULL, FALSE);
        EnterCriticalSection(&cs);
        mesh_client_reset_device(name);
        LeaveCriticalSection(&cs);
    }
    DisplayCurrentGroup();
}

void CMeshClientDlg::OnSelchangeNetwork()
{
}

void CMeshClientDlg::OnBnClickedReconfigure()
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

    EnterCriticalSection(&cs);
    mesh_client_set_device_config(old_name, DeviceConfig.is_gatt_proxy, DeviceConfig.is_friend, DeviceConfig.is_relay, DeviceConfig.send_net_beacon, DeviceConfig.relay_xmit_count, DeviceConfig.relay_xmit_interval, DeviceConfig.default_ttl, DeviceConfig.net_xmit_count, DeviceConfig.net_xmit_interval);
    mesh_client_set_publication_config(DeviceConfig.publish_credential_flag, DeviceConfig.publish_retransmit_count, DeviceConfig.publish_retransmit_interval, DeviceConfig.publish_ttl);
    LeaveCriticalSection(&cs);

    DisplayCurrentGroup();
}

void CMeshClientDlg::OnBnClickedMoveToGroup()
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
    EnterCriticalSection(&cs);
    if ((group_from_name[0] == 0) && (group_to_name[0] != 0))
        mesh_client_add_component_to_group(device_name, group_to_name);
    else if ((group_from_name[0] != 0) && (group_to_name[0] == 0))
        mesh_client_remove_component_from_group(device_name, group_from_name);
    else if ((group_from_name[0] != 0) && (group_to_name[0] != 0))
        mesh_client_move_component_to_group(device_name, group_from_name, group_to_name);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnCbnSelchangeConfigureControlDevice()
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

void CMeshClientDlg::OnBnClickedConfigurePub()
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

    EnterCriticalSection(&cs);
    mesh_client_set_publication_config(publish_credential_flag, publish_retransmit_count, publish_retransmit_interval, publish_ttl);
    mesh_client_configure_publication(device_name, client_control, publish_method, publish_to_name, publish_period);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedOnOffGet()
{
    control_test = GetDlgItemInt(IDC_NUM_ITTERRATIONS);

    if (control_test != 0)
    {
        p_control_test_results = (DWORD *)malloc(control_test * sizeof(DWORD));
        if (p_control_test_results != 0)
        {
            memset(p_control_test_results, 0, sizeof(control_test * sizeof(DWORD)));
        }
        control_test_itterration = 0;
        SetTimer(IDT_CONTROL_TEST, 5000, NULL);
    }
    OnOffGet();
}

void CMeshClientDlg::OnOffGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    control_test_start_time = GetTickCount();
    mesh_client_on_off_get(name);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedOnOffSet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    int on_off = ((CComboBox *)GetDlgItem(IDC_ON_OFF_TARGET))->GetCurSel();
    if (on_off >= 0)
    {
        EnterCriticalSection(&cs);
        mesh_client_on_off_set(name, on_off, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
        LeaveCriticalSection(&cs);
    }
}

void CMeshClientDlg::OnBnClickedLevelGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_level_get(name);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedLevelSet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    SHORT target_level = (SHORT)GetDlgItemInt(IDC_LEVEL_TARGET);
    EnterCriticalSection(&cs);
    mesh_client_level_set(name, target_level, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedLightnessGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_lightness_get(name);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedLightnessSet()
{
    SHORT target_lightness = (SHORT)GetDlgItemInt(IDC_LIGHT_LIGHTNESS_TARGET);
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_lightness_set(name, target_lightness, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedLightHslGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_hsl_get(name);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedLightHslSet()
{
    SHORT target_lightness = (SHORT)GetDlgItemInt(IDC_LIGHT_LIGHTNESS_TARGET);
    SHORT target_hue = (SHORT)GetDlgItemInt(IDC_LIGHT_HSL_HUE_VALUE);
    SHORT target_saturation = (SHORT)GetDlgItemInt(IDC_LIGHT_HSL_SATURATION_VALUE);
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_hsl_set(name, target_lightness, target_hue, target_saturation, WICED_TRUE, 250 /*DEFAULT_TRANSITION_TIME*/, 0);
    LeaveCriticalSection(&cs);
    SetTimer(IDT_CONTROL_TEST, 100, NULL);
}

void CMeshClientDlg::OnBnClickedLightCtlGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_ctl_get(name);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedLightCtlSet()
{
    SHORT target_lightness = (SHORT)GetDlgItemInt(IDC_LIGHT_LIGHTNESS_TARGET);
    SHORT target_temperature = (SHORT)GetDlgItemInt(IDC_LIGHT_CTL_TEMPERATURE_TARGET);
    SHORT target_delta_uv = (SHORT)GetDlgItemInt(IDC_LIGHT_CTL_DELTA_UV_TARGET);
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_ctl_set(name, target_lightness, target_temperature, target_delta_uv, WICED_TRUE, DEFAULT_TRANSITION_TIME, 0);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedVsData()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));

    BYTE buffer[400];
    DWORD len = GetHexValue(IDC_TC_NET_LEVEL_TRX_PDU, buffer, sizeof(buffer));

    EnterCriticalSection(&cs);
    mesh_client_vendor_data_set(name, buffer, (uint16_t)len);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnCbnSelchangeControlDevice()
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

void CMeshClientDlg::OnBnClickedSensorGet()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));

    int sel;
    CComboBox *pSensors = (CComboBox *)GetDlgItem(IDC_SENSOR_TARGET);
    if ((sel = pSensors->GetCurSel()) < 0)
        return;

    int property_id = (int)pSensors->GetItemData(sel);

    EnterCriticalSection(&cs);
    mesh_client_sensor_get(name, property_id);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::ProcessVendorSpecificData(LPBYTE p_data, DWORD len)
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

void CMeshClientDlg::OnBnClickedIdentify()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_identify(name, 10);
    LeaveCriticalSection(&cs);
}

void component_info_status_callback(uint8_t status, char *component_name, char *component_info)
{
    WCHAR szComponentName[80];
    WCHAR szComponentInfo[80];
    MultiByteToWideChar(CP_UTF8, 0, component_name, -1, szComponentName, sizeof(szComponentName) / sizeof(WCHAR));
    MultiByteToWideChar(CP_UTF8, 0, component_info, -1, szComponentInfo, sizeof(szComponentInfo) / sizeof(WCHAR));
    Log(L"Component Info status:%d from %s Info:%s\n", status, szComponentName, szComponentInfo);
}

void CMeshClientDlg::OnBnClickedGetComponentInfo()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
    EnterCriticalSection(&cs);
    mesh_client_get_component_info(name, component_info_status_callback);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::ConnectDfuDistributor()
{
    char name[80];
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));

    EnterCriticalSection(&cs);
    mesh_client_connect_component(name, 1, 10);
    LeaveCriticalSection(&cs);
}

void CMeshClientDlg::OnBnClickedOtaUpgradeStart()
{
    if (m_dfuState != DFU_STATE_IDLE)
    {
        m_trace->SetCurSel(m_trace->AddString(L"DFU already started"));
        return;
    }

    m_dfuAction = DISTRIBUTION_ACTION_START;
    ConnectDfuDistributor();
}

void CMeshClientDlg::OnBnClickedOtaUpgradeStop()
{
    // If OTA download is in progress, stop it
    if (m_pDownloader && m_pDownloader->m_state == WSDownloader::WS_UPGRADE_STATE_DATA_TRANSFER)
    {
        m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_ABORT);
        return;
    }

    EnterCriticalSection(&cs);
    mesh_client_dfu_stop();
    LeaveCriticalSection(&cs);

    m_dfuState = DFU_STATE_IDLE;
    KillTimer(IDT_DISTRIBUTION_STATUS);
}

void CMeshClientDlg::OnBnClickedOtaUpgradeApply()
{
    if (m_dfuState != DFU_STATE_COMPLETED)
    {
        m_trace->SetCurSel(m_trace->AddString(L"Firmware distribution not done"));
        return;
    }

    m_dfuAction = DISTRIBUTION_ACTION_APPLY;
    ConnectDfuDistributor();
}

void fw_distribution_status(uint8_t status, int current_block_num, int total_blocks)
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (!pDlg)
        return;

    pDlg->OnDfuStatusCallback(status, current_block_num, total_blocks);
}

void CMeshClientDlg::OnBnClickedOtaUpgradeStatus()
{
    m_dfuAction = DISTRIBUTION_ACTION_GET_STATUS;
    ConnectDfuDistributor();
}

void CMeshClientDlg::OnDfuStatusCallback(uint8_t status, int current_block_num, int total_blocks)
{
    WCHAR buf[256];
    wsprintf(buf, L"status %d block %d total_block %d", status, current_block_num, total_blocks);
    m_trace->SetCurSel(m_trace->AddString(buf));

    if (status == WICED_BT_MESH_FW_DISTRIBUTION_STATUS_ACTIVE)
    {
        m_dfuState = DFU_STATE_UPDATING;
        if (total_blocks != 0)
        {
            m_Progress.SetPos(current_block_num * 100 / total_blocks);
        }
    }
    else if (status == WICED_BT_MESH_FW_DISTRIBUTION_STATUS_FAILED)
    {
        KillTimer(IDT_DISTRIBUTION_STATUS);
        m_dfuState = DFU_STATE_IDLE;
    }
    else if (status == WICED_BT_MESH_FW_DISTRIBUTION_STATUS_COMPLETED)
    {
        m_Progress.SetPos(100);
        KillTimer(IDT_DISTRIBUTION_STATUS);
        m_trace->SetCurSel(m_trace->AddString(L"DFU distribution finish"));
        m_dfuState = DFU_STATE_COMPLETED;
    }
}

BOOL get_firmware_id_from_file(CString sFilePath, mesh_dfu_fw_id_t *fwID)
{
    FILE *pFile;
    if (_wfopen_s(&pFile, sFilePath, L"r"))
        return FALSE;

    char line[100];
    DWORD cid = 0;
    ULONGLONG fwid = 0;

    while (fgets(line, 100, pFile))
    {
        if (strstr(line, "CID=") == line)
            sscanf(&line[4], "0x%x", &cid);
        else if (strstr(line, "FWID=") == line)
            sscanf(&line[5], "0x%llx", &fwid);
    }

    fwID->company_id = (uint16_t)cid;
    fwID->fw_id_len = 8;
    fwID->fw_id[0] = (uint8_t)((fwid >> 56) & 0xFF);
    fwID->fw_id[1] = (uint8_t)((fwid >> 48) & 0xFF);
    fwID->fw_id[2] = (uint8_t)((fwid >> 40) & 0xFF);
    fwID->fw_id[3] = (uint8_t)((fwid >> 32) & 0xFF);
    fwID->fw_id[4] = (uint8_t)((fwid >> 24) & 0xFF);
    fwID->fw_id[5] = (uint8_t)((fwid >> 16) & 0xFF);
    fwID->fw_id[6] = (uint8_t)((fwid >> 8) & 0xFF);
    fwID->fw_id[7] = (uint8_t)(fwid & 0xFF);

    fclose(pFile);
    return TRUE;
}

void CMeshClientDlg::OnNodeConnected()
{
    if (m_dfuAction == DISTRIBUTION_ACTION_START)
    {
        OnOtaUpgradeContinue();
    }
    else if (m_dfuAction == DISTRIBUTION_ACTION_STOP)
    {
        EnterCriticalSection(&cs);
        mesh_client_dfu_stop();
        LeaveCriticalSection(&cs);
    }
    else if (m_dfuAction == DISTRIBUTION_ACTION_APPLY)
    {
        OnOtaUpgradeApply();
    }
    else if (m_dfuAction == DISTRIBUTION_ACTION_GET_STATUS)
    {
        CString sInfoFilePath;
        char name[80];
        mesh_dfu_fw_id_t fw_id;
        int result;
        GetDlgItemText(IDC_INFOFILENAME, sInfoFilePath);
        GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
        if (!get_firmware_id_from_file(sInfoFilePath, &fw_id))
            return;
        EnterCriticalSection(&cs);
        result = mesh_client_dfu_get_status(name, fw_id.company_id, fw_id.fw_id, fw_id.fw_id_len, &fw_distribution_status);
        LeaveCriticalSection(&cs);
        if (result == MESH_CLIENT_SUCCESS)
            SetTimer(IDT_DISTRIBUTION_STATUS, DISTRIBUTION_STATUS_TIMEOUT, NULL);
    }
}

void CMeshClientDlg::OnOtaUpgradeContinue()
{
    CString sFilePath;
    CString sInfoFilePath;
    GetDlgItemText(IDC_FILENAME, sFilePath);
    GetDlgItemText(IDC_INFOFILENAME, sInfoFilePath);

    if (sFilePath.IsEmpty())
    {
        OnBnClickedBrowse();
        GetDlgItemText(IDC_FILENAME, sFilePath);
        if (sFilePath.IsEmpty())
            return;
    }

    mesh_dfu_fw_id_t fw_id = { 0 };
#ifdef WICED_OTA_VERSION_2
    if (!get_firmware_id_from_file(sInfoFilePath, &fw_id))
    {
        Log(L"Failed to find FW ID from DFU Info %s\n", sInfoFilePath);
        return;
    }
#endif
    int dfu_method = ((CComboBox *)GetDlgItem(IDC_DFU_METHOD))->GetCurSel();
    if (dfu_method == DFU_METHOD_APP_TO_ALL)
    {
        char name[80];
        GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
        EnterCriticalSection(&cs);
        mesh_client_dfu_start(dfu_method, name, fw_id.company_id, fw_id.fw_id, fw_id.fw_id_len);
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

    m_trace->SetCurSel(m_trace->AddString(L"Firmware OTA start"));
    m_dfuState = DFU_STATE_INITIATING;

    // Create new downloader object
    m_pDownloader = new WSDownloader(m_btInterface, m_pPatch, m_dwPatchSize, fw_id.company_id, fw_id.fw_id, m_hWnd);
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
}

void CMeshClientDlg::OnOtaUpgradeApply()
{
    // If Downloader object is created already
    if (m_pDownloader != NULL)
    {
        delete m_pDownloader;
        m_pDownloader = NULL;
    }

    EnterCriticalSection(&cs);
    CBtWin10Interface *pWin10BtInterface = NULL;
    pWin10BtInterface = dynamic_cast<CBtWin10Interface *>(m_btInterface);

    if ((pWin10BtInterface == NULL) || !((CBtWin10Interface*)m_btInterface)->CheckForOTAServices())
    {
        LeaveCriticalSection(&cs);
        MessageBox(L"This device may not support OTA FW Upgrade. Select another device.", L"Error", MB_OK);
        return;
    }
    LeaveCriticalSection(&cs);

    // Create new downloader object
    m_pDownloader = new WSDownloader(m_btInterface, NULL, 0, 0, 0, m_hWnd);
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

    m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_APPLY_FIRMWARE);
}

LRESULT CMeshClientDlg::OnWsUpgradeCtrlPoint(WPARAM Instance, LPARAM lparam)
{
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

    return S_OK;
}

LRESULT CMeshClientDlg::OnProgress(WPARAM state, LPARAM param)
{
    static UINT total;
    if (state == WSDownloader::WS_UPGRADE_STATE_WAIT_FOR_READY_FOR_DOWNLOAD)
    {
        total = (UINT)param;
        m_Progress.SetRange32(0, (int)param);
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
        m_trace->SetCurSel(m_trace->AddString(L"Firmware OTA finish"));

        int dfu_method = ((CComboBox *)GetDlgItem(IDC_DFU_METHOD))->GetCurSel();
        if (dfu_method == DFU_METHOD_APP_TO_DEVICE)
        {
#ifdef WICED_OTA_VERSION_2
            m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_APPLY_FIRMWARE);
#else
            m_dfuState = DFU_STATE_IDLE;
#endif
        }
        else
        {
            m_Progress.SetRange32(0, 100);
            m_Progress.SetPos(0);

            CString sInfoFilePath;
            char name[80];
            mesh_dfu_fw_id_t fw_id;
            int result;
            GetDlgItemText(IDC_INFOFILENAME, sInfoFilePath);
            GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, name, sizeof(name));
            if (!get_firmware_id_from_file(sInfoFilePath, &fw_id))
                return S_OK;
            EnterCriticalSection(&cs);
            result = mesh_client_dfu_start(dfu_method, name, fw_id.company_id, fw_id.fw_id, fw_id.fw_id_len);
            LeaveCriticalSection(&cs);
            if (result == MESH_CLIENT_SUCCESS)
            {
                SetTimer(IDT_DISTRIBUTION_STATUS, DISTRIBUTION_STATUS_TIMEOUT, NULL);
            }
            m_trace->SetCurSel(m_trace->AddString(L"DFU distribution start"));
        }
    }
    else if (state == WSDownloader::WS_UPGRADE_STATE_ABORTED)
    {
        m_trace->SetCurSel(m_trace->AddString(L"Firmware OTA failed"));
        m_Progress.SetPos(total);
        m_dfuState = DFU_STATE_IDLE;
    }
    return S_OK;
}

extern "C" wiced_bool_t mesh_bt_gatt_le_connect(wiced_bt_device_address_t bda,
    wiced_bt_ble_address_type_t bd_addr_type,
    wiced_bt_ble_conn_mode_t conn_mode,
    wiced_bool_t is_direct)
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;

    if ((pDlg != NULL) && (pDlg->m_bConnecting == TRUE))
        return WICED_FALSE;

    if ((pDlg == NULL) || ((pDlg != NULL) && pDlg->m_btInterface == NULL))
    {
        BD_ADDR* p_bd_addr = (BD_ADDR*)malloc(BD_ADDR_LEN);
        memcpy(p_bd_addr, bda, BD_ADDR_LEN);
        pDlg->PostMessage(WM_MESH_DEVICE_CONNECT, (WPARAM)0, (LPARAM)p_bd_addr);
        return WICED_TRUE;
    }
    return WICED_FALSE;
}

LRESULT CMeshClientDlg::OnMeshDeviceConnect(WPARAM state, LPARAM param)
{
    BYTE* pbda = (BYTE*)param;
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if ((pDlg == NULL) || (pDlg->m_btInterface != NULL))
    {
        free(pbda);
        EnterCriticalSection(&cs);
        mesh_client_connection_state_changed(0, 0);
        LeaveCriticalSection(&cs);
        return WICED_FALSE;
    }

    BLUETOOTH_ADDRESS bth;
    bth.ullLong = 0;
    ods("LE Connect bda:%02x%02x%02x%02x%02x%02x", pbda[0], pbda[1], pbda[2], pbda[3], pbda[4], pbda[5]);
    for (int i = 0; i < 6; i++)
        bth.rgBytes[5 - i] = pbda[i];

    free(pbda);

    BOOL ConnectProvisioning = mesh_client_is_connecting_provisioning();

    EnterCriticalSection(&cs);
    pDlg->m_bConnecting = TRUE;
    pDlg->m_btInterface = new CBtWin10Interface(&bth, pDlg);
    if (!pDlg->m_btInterface->Init(ConnectProvisioning))
    {
        pDlg->m_bConnecting = FALSE;
        mesh_client_connection_state_changed(0, 0);
        LeaveCriticalSection(&cs);
        pDlg->Disconnect();
        pDlg->m_trace->SetCurSel(pDlg->m_trace->AddString(L"Failed to connect"));
        return WICED_FALSE;
    }

    CBtWin10Interface *pWin10BtInterface = dynamic_cast<CBtWin10Interface *>(pDlg->m_btInterface);
    if (!pWin10BtInterface->CheckForProvProxyServices())
    {
        pDlg->m_bConnecting = FALSE;
        mesh_client_connection_state_changed(0, 0);
        LeaveCriticalSection(&cs);
        pDlg->Disconnect();
        pDlg->m_trace->SetCurSel(pDlg->m_trace->AddString(L"Failed to connect"));
        return WICED_FALSE;
    }

    // To get MTU size
    UINT16 mtu = pWin10BtInterface->GetMTUSize();
    wiced_bt_mesh_core_set_gatt_mtu(mtu);

    BTW_GATT_VALUE gatt_value;

    // Register for Proxy notifications
    gatt_value.len = 2;
    gatt_value.value[0] = 1;
    gatt_value.value[1] = 0;

    // Provisioning / proxy service
    if (ConnectProvisioning)
    {
        pWin10BtInterface->SetDescriptorValue(&guidSvcMeshProvisioning, &guidCharProvisioningDataOut, BTW_GATT_UUID_DESCRIPTOR_CLIENT_CONFIG, &gatt_value);
        pWin10BtInterface->RegisterNotification(&guidSvcMeshProvisioning, &guidCharProvisioningDataOut);
    }
    else
    {
        pWin10BtInterface->SetDescriptorValue(&guidSvcMeshProxy, &guidCharProxyDataOut, BTW_GATT_UUID_DESCRIPTOR_CLIENT_CONFIG, &gatt_value);
        pWin10BtInterface->RegisterNotification(&guidSvcMeshProxy, &guidCharProxyDataOut);
    }
    LeaveCriticalSection(&cs);

    Sleep(500);

    ods("connected\n");
    pDlg->m_trace->SetCurSel(pDlg->m_trace->AddString(L"Connected"));

    EnterCriticalSection(&cs);
    mesh_client_connection_state_changed(1, mtu);
    LeaveCriticalSection(&cs);

    pDlg->m_bConnecting = FALSE;
    return WICED_TRUE;
}

extern "C" wiced_bool_t mesh_bt_gatt_le_disconnect(uint32_t conn_id)
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if ((pDlg != NULL) && (pDlg->m_btInterface != NULL))
    {
        pDlg->PostMessage(WM_MESH_DEVICE_DISCONNECT, (WPARAM)0, (LPARAM)0);
    }
    return WICED_TRUE;
}


LRESULT CMeshClientDlg::OnMeshDeviceDisconnect(WPARAM state, LPARAM param)
{
    Disconnect();
    return WICED_TRUE;
}

void CMeshClientDlg::OnBnClickedConnectdisconnect()
{
    WCHAR buf[128];

    if (!m_bConnected)
    {
        wsprintf(buf, L"Connecting Network");
        m_trace->SetCurSel(m_trace->AddString(buf));
        EnterCriticalSection(&cs);
        mesh_client_connect_network(1, 7);
    }
    else
    {
        wsprintf(buf, L"Disconnecting Network");
        m_trace->SetCurSel(m_trace->AddString(buf));
        EnterCriticalSection(&cs);
        mesh_client_disconnect_network();
    }
    LeaveCriticalSection(&cs);
}

LRESULT CMeshClientDlg::OnMeshDeviceConnected(WPARAM Instance, LPARAM lparam)
{
    ods("OnMeshDeviceConnected:\n");
    EnterCriticalSection(&cs);
    mesh_client_connection_state_changed(1, 75);
    LeaveCriticalSection(&cs);
    return S_OK;
}

LRESULT CMeshClientDlg::OnMeshDeviceDisconnected(WPARAM Instance, LPARAM lparam)
{
    ods("OnMeshDeviceDisconnected:\n");

    if (m_pDownloader)
        m_pDownloader->ProcessEvent(WSDownloader::WS_UPGRADE_DISCONNECTED);

    EnterCriticalSection(&cs);
    CBtWin10Interface *pWin10BtInterface = dynamic_cast<CBtWin10Interface *>(m_btInterface);
    if (pWin10BtInterface)
    {
        pWin10BtInterface->ResetInterface();
        delete m_btInterface;
        m_btInterface = NULL;
    }
    mesh_client_connection_state_changed(0, 0);
    LeaveCriticalSection(&cs);

    return S_OK;
}

LRESULT CMeshClientDlg::OnMeshDeviceAdvReport(WPARAM Instance, LPARAM lparam)
{
    ods("OnMeshDeviceAdvReport:\n");

    mesh_client_app_adv_report_t *p_adv_report = (mesh_client_app_adv_report_t *)lparam;
    if (p_adv_report)
    {
        EnterCriticalSection(&cs);
        mesh_client_advert_report(p_adv_report->bda, p_adv_report->addr_type, p_adv_report->rssi, p_adv_report->adv_data);
        LeaveCriticalSection(&cs);
        free(p_adv_report);
    }

    return S_OK;
}

void CMeshClientDlg::OnBnClickedBrowse()
{
    static TCHAR BASED_CODE szFilter[] = _T("OTA Files (*.ota.bin)|*.OTA.BIN|");

    CFileDialog dlgFile(TRUE, NULL, NULL, OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, szFilter);
    if (dlgFile.DoModal() == IDOK)
    {
        SetDlgItemText(IDC_FILENAME, dlgFile.GetPathName());
    }
}

void CMeshClientDlg::OnBnClickedInfoBrowse()
{
    CFileDialog dlgFile(TRUE, NULL, NULL, OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR);
    if (dlgFile.DoModal() == IDOK)
    {
        SetDlgItemText(IDC_INFOFILENAME, dlgFile.GetPathName());
    }
}

void CMeshClientDlg::UpdateScanState(BOOL bScanning)
{
    if (bScanning)
    {
        SetDlgItemText(IDC_SCAN_UNPROVISIONED, L"Stop Scanning");
        m_bScanning = TRUE;
    }
    else
    {
        SetDlgItemText(IDC_SCAN_UNPROVISIONED, L"Scan Unprovisioned");
        m_bScanning = FALSE;
    }
}

void CMeshClientDlg::OnBnClickedNetworkImport()
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

void CMeshClientDlg::OnBnClickedNetworkExport()
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

void CMeshClientDlg::OnBnClickedSensorConfigure()
{
    CSensorConfig dlg;
    GetDlgItemTextA(m_hWnd, IDC_CONTROL_DEVICE, dlg.component_name, sizeof(dlg.component_name));

    CComboBox *pSensors = (CComboBox *)GetDlgItem(IDC_SENSOR_TARGET);
    if (pSensors->GetCurSel() < 0)
        return;

    dlg.property_id = (USHORT)pSensors->GetItemData(pSensors->GetCurSel());

    mesh_client_set_publication_config(DeviceConfig.publish_credential_flag, DeviceConfig.publish_retransmit_count, DeviceConfig.publish_retransmit_interval, DeviceConfig.publish_ttl);
    INT_PTR nResponse = dlg.DoModal();
}

uint32_t CMeshClientDlg::GetDfuImageSize()
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

void CMeshClientDlg::GetDfuImageChunk(uint8_t *p_data, uint32_t offset, uint16_t data_len)
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
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    return (pDlg != NULL) ? pDlg->GetDfuImageSize() : 0;
}

extern "C" void wiced_bt_get_fw_image_chunk(uint8_t partition, uint32_t offset, uint8_t *p_data, uint16_t data_len)
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (pDlg != NULL) 
        pDlg->GetDfuImageChunk(p_data, offset, data_len);
}

extern "C" wiced_bool_t wiced_ota_fw_upgrade_get_new_fw_info(uint16_t *company_id, uint8_t *fw_id_len, uint8_t *fw_id)
{
    return false;
}

#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)

extern "C" void mesh_client_network_open_UI_Ex()
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (pDlg)
    {
        //Do Something; Example of UI hook to be called from automation rpc;
    }
}

extern "C" int mesh_client_network_create_UI_Ex(const char *provisioner_name, const char *p_provisioner_uuid, char *mesh_name)
{
    CMeshClientDlg *pDlg = (CMeshClientDlg *)theApp.m_pMainWnd;
    if (!pDlg)
        return 0;

    memset(provisioner_uuid, 0, sizeof(provisioner_uuid));
    memcpy(provisioner_uuid, p_provisioner_uuid, 32);

    WCHAR wcStr[128] = { 0 };
    MultiByteToWideChar(CP_UTF8, 0, (char *)mesh_name, strlen(mesh_name) + 1, wcStr, sizeof(wcStr) / sizeof(WCHAR));
    pDlg->SetDlgItemText(IDC_NETWORK, wcStr);

    memset(wcStr, 0, sizeof(wcStr));
    MultiByteToWideChar(CP_UTF8, 0, (char *)provisioner_name, strlen(provisioner_name) + 1, wcStr, sizeof(wcStr) / sizeof(WCHAR));
    pDlg->SetDlgItemText(IDC_PROVISIONER, wcStr);

    return 0;
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

extern "C" int mesh_client_scan_unprovisioned_UI_Ex(int start)
{
	return 0;
}
#endif

#if defined(MESH_AUTOMATION_ENABLED) && (MESH_AUTOMATION_ENABLED == TRUE)

extern void OnFDRead(WPARAM wParam, LPARAM lParam);
extern void OnFDAccept(HWND hwnd, WPARAM wParam, LPARAM lParam);

LRESULT CMeshClientDlg::OnSocketMessage(WPARAM wParam, LPARAM lParam)
{
    ods("CMeshClientDlg: OnSocketMessage() called\n");

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
        OnFDAccept(theApp.m_pMainWnd->m_hWnd, wParam, lParam);
        break;        // Accept an incoming connection
    case FD_READ:
    {
        OnFDRead(wParam, lParam);
    }
    break;        // Receive data from the socket in wParam
    case FD_WRITE:
        //OnWrite(sock);
        ods(">>> FD_WRITE >>> Received");
        break;        // The socket in wParam is ready for sending data
    case FD_CLOSE:
        closesocket(sock);
        break;        // The connection is now closed
    }
    return S_OK;

}

#endif


