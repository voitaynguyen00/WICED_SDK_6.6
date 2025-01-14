#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include "win_data_types.h"

#ifndef WICED_MESH_REPO
#include "bsa_mesh_api.h"
//#include "serial_port.h"
#endif

#include <QMutex>
#include <QWaitCondition>

extern bool g_bUseBsa;

extern "C" uint16_t wiced_hci_send(uint16_t opcode, uint8_t *p_buffer, uint16_t length);
class CommHelper
{
public:
    uint16_t SendWicedCommand(uint16_t opcode, uint8_t *p_buffer, uint16_t length);
};

extern CommHelper * m_ComHelper;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void setActive();
    void OnClose();
    QSettings m_settings;
    void ProcessData(DWORD opcode, LPBYTE p_data, DWORD len);
    void closeEvent(QCloseEvent *event);
    bool OpenCommPort();
    bool SetupCommPortUI();
    bool connect_bsa();
    void disconnect_bsa();
    //WicedSerialPort *m_CommPort;

    QMutex m_write;
    QWaitCondition serial_read_wait;
    int PortWrite(unsigned char * data, DWORD Length);
    bool m_bPortOpen;
    bool mBsaConnected;
    void setDmUI(bool connected);
    int FindBaudRateIndex(int baud);
    void CloseCommPort();
    void ClearPort();
    //void serialPortError(QSerialPort::SerialPortError error);

signals:
   void HandleWicedEvent(unsigned int opcode, unsigned int len, unsigned char *p_data);
   void HandleTrace(QString *pTrace);
   void ScrollToTop();
   void NodeReset();
   void DfuProgress(int pos, int param);

public slots:
    void onHandleWicedEvent(unsigned int, unsigned int, unsigned char *);    
    void on_btnConnectComm();
    void onDeleteNetwork();
    void onCreateNetwork();
    void on_btnOpen();
    void on_btnClose();
    void on_btnConnect();
    void on_btnImport();
    void on_btnExport();
    void on_btnCreateGrp();
    void on_btnGrpDel();
    void on_btnScan();
    void on_btnProv();
    void on_btnReConfig();
    void on_btnConfigSub();
    void on_btnConfigPub();
    void on_btnGetStatus();
    void on_btnGetColor();
    void on_btnGetHue();
    void on_btnGetInfo();
    void on_btnGetLevel();
    void on_btnGetLight();
    void on_btnGetOnOff();
    void on_btnSetColor();
    void on_btnSetHue();
    void on_btnSetLevel();
    void on_btnSetLight();
    void on_btnSetOnOff();
    void on_btnSetVen();
    void on_btnDfuStart();
    void on_btnDfuStop();
    void on_btnClearTrace();
    void onGrpIndexChanged(int);    
    void on_btnNodeReset();
    void on_btnIdentify();
    void on_btnFindDfuFile();
    void on_off_tmr_timeout();
    //void on_dfu_timer_timeout();

    // utility methods
    void processTrace(QString * trace);
    void processScrollToTop();
    void processNodeReset();

public:
    Ui::MainWindow *ui;

public:
    BOOL        m_scan_started;
    BOOL        m_bConnecting;
    BOOL        m_bScanning;
    uint8_t     m_dfuMethod;
    BOOL m_bConnected;
    QStringList m_strComPortsIDs;

    void SetDlgItemHex(DWORD id, DWORD val);
    DWORD GetHexValue(DWORD id, LPBYTE buf, DWORD buf_size);
    DWORD GetHexValue(char *szBuf, LPBYTE buf, DWORD buf_size);

    void DisplayCurrentGroup();
    void ProvisionCompleted();

    char   m_szCurrentGroup[80];

    BOOL    m_fw_download_active;
    BYTE    m_received_evt[261];
    DWORD   m_received_evt_len;
    HANDLE m_event;
#define STATE_IDLE                          0
#define STATE_LOCAL_GET_COMPOSITION_DATA    1
#define STATE_LOCAL_ADD_APPLICATION_KEY     2
#define STATE_LOCAL_BIND_MODELS             3
#define STATE_REMOTE_GET_COMPOSITION_DATA   4
#define STATE_REMOTE_ADD_APPLICATION_KEY    5
#define STATE_REMOTE_BIND_MODELS            6
    int m_state;

//    CProgressCtrl m_Progress;
    LRESULT OnProgress(WPARAM completed, LPARAM total);
//    WSDownloader *m_pDownloader;
    LPBYTE m_pPatch;
    DWORD m_dwPatchSize;

    void ProcessUnprovisionedDevice(uint8_t *p_uuid, uint16_t oob, uint8_t *name, uint8_t name_len);
    void LinkStatus(uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt);
    void ProcessUnprovisionedDevice(LPBYTE p_data, DWORD len);
    void ProcessVendorSpecificData(LPBYTE p_data, DWORD len);
};


class Worker : public QObject
{
    Q_OBJECT

public slots:
    void process();
};

#endif // MAINWINDOW_H
