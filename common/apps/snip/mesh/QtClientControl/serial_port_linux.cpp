#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDateTime>
#include <QFileDialog>
#include <QMutex>
#include <QTimer>
#include <QThread>
#include <wchar.h>
#include <stdarg.h>
#include <QMessageBox>
#include <QtSerialPort/QSerialPortInfo>
#include "add_defines.h"
#include "win_data_types.h"
#include "wiced_mesh_client.h"
#include <unistd.h>
#include <string.h>
#include "hci_control_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include "bt_target.h"
#include <ctype.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/termios.h>
#include <sys/ioctl.h>


void TraceHciPkt(BYTE type, BYTE *buffer, USHORT length, USHORT serial_port=1);
extern "C"
{
extern void Log(char *fmt, ...);
}

extern bool m_bClosing;

extern MainWindow * gMainWindow;
int sock;

bool OpenSerialPort(UINT32 baud_rate, char * device_name);
typedef unsigned char uchar;
#define APPL_TRACE_DEBUG0   printf

// valid baud rates
int as32BaudRate[] =
{
    115200,
    3000000,
#ifndef __MACH__
    4000000
#endif
};

bool MainWindow::SetupCommPortUI()
{
    // read settings for baudrate, serial port and flow-ctrl
    int baud_rate = m_settings.value("baud_rate",3000000).toInt();
    QString comm_port = m_settings.value("comm_port","").toString();

    // get list of all available serial ports
    int port_inx = -1;
    m_strComPortsIDs.clear();
    m_strComPortsIDs.append("<select port>"); // dummy string to match combo box
    QList<QSerialPortInfo> port_list = QSerialPortInfo::availablePorts();
    for (int i =0; i < port_list.size(); i++)
    {
        QString strName = port_list.at(i).portName();
        QString strDesc =  port_list.at(i).description();
        strName += " (" + strDesc + ")";
        QString strPortID = port_list.at(i).systemLocation();

        // m_strComPortsIDs contains serial port ID used to open the port
        m_strComPortsIDs.append(strPortID);

        // cbCommport contains friendly names
        ui->cbCommPort->addItem(strName, strPortID);
    }

    if (g_bUseBsa)
    {
        // m_strComPortsIDs contains serial port ID used to open the port
        m_strComPortsIDs.append("0");

        // cbCommport contains friendly names
        ui->cbCommPort->addItem("BSA SERVER", "0");
    }

    if ( -1 != (port_inx = ui->cbCommPort->findText(comm_port)))
    {
        ui->cbCommPort->setCurrentIndex(port_inx);
    }

    // populate dropdown list of baud rates
    QString strBaud;
    int baud_inx = (sizeof(as32BaudRate) / sizeof(as32BaudRate[0])) - 1; // select default baud rate as highest allowed
    for (int i = 0; i < (int) (sizeof(as32BaudRate) / sizeof(as32BaudRate[0])); i++)
    {
        strBaud.sprintf( "%d", as32BaudRate[i]);
        ui->cbBaudRate->addItem(strBaud,as32BaudRate[i]);
        if (as32BaudRate[i] == baud_rate)
            baud_inx = i;
    }
    ui->cbBaudRate->setCurrentIndex(baud_inx);
}

// Send WICED HCI commmand to embedded device
int MainWindow::PortWrite(unsigned char * data, DWORD len)
{
    int ret, total = 0;
    errno = 0;

    while(len != 0)
    {
        ret = write(sock, data + total, len);
        if (ret < 0)
            Log("PortWrite len = %d, ret = %d, error = %s(%d)", len, ret, strerror(errno), errno);
        total += ret;
        len -= ret;
    }

    return ((UINT16)total);
}

void MainWindow::setDmUI(bool connected)
{
    ui->btnConnectComm->setText(connected ? "Disconnect" : "Connect");
}

// User clicked button to open or close serial port
void MainWindow::on_btnConnectComm()
{
    if (ui->cbCommPort->currentText() == "BSA SERVER")
    {
        if (mBsaConnected)
            disconnect_bsa();
        else
            connect_bsa();
        setDmUI(mBsaConnected);
        return;
    }
    // If port is not open, open it
    if(!m_bPortOpen)
    {
        ui->btnConnectComm->setEnabled(false);
        bool bopen = OpenCommPort();
        ui->btnConnectComm->setEnabled(true);

        if(!bopen)
        {
            QMessageBox(QMessageBox::Information, "Serial Port", "Error opening serial port", QMessageBox::Ok).exec();
        }
        else
        {
            ui->cbCommPort->setEnabled(false);
            ui->btnConnectComm->setText("Disconnect");
            ui->cbBaudRate->setEnabled(false);
        }
    }
    // Close port if open
    else
    {
        ui->btnConnectComm->setText("Connect");
        ClearPort();
    }
    setDmUI(m_bPortOpen);
}

// Open and setup serial port
bool MainWindow::OpenCommPort()
{
    int baud_rate = ui->cbBaudRate->currentText().toInt();

    QString comm_port = ui->cbCommPort->currentData().toString();
    m_settings.setValue("port",comm_port);

    if (comm_port == "BSA SERVER")
    {

    }

    unsigned int serialPortBaudRate = ui->cbBaudRate->currentText().toInt();

    if (m_bPortOpen = OpenSerialPort(serialPortBaudRate,(char *)comm_port.toStdString().c_str()))
    {
        Log("Opened %s at speed: %u", comm_port.toStdString().c_str(), serialPortBaudRate);
    }
    else
    {
        Log("Error opening serial port %s: Error number %d", comm_port.toStdString().c_str(),errno);
    }

    return m_bPortOpen;
}

void MainWindow::CloseCommPort()
{
    m_bClosing = true;
    serial_read_wait.wakeAll();
    m_bPortOpen = false;
}

// Clear port and UI
void MainWindow::ClearPort()
{
    CloseCommPort();
    QThread::sleep(1);

    Log("Serial port closed.");

    ui->cbCommPort->setEnabled(true);
    ui->cbBaudRate->setEnabled(true);

    ui->btnConnectComm->setText("Open Port");
}

static int ReadNewHciPacket(BYTE * pu8Buffer, int bufLen, int * pOffset)
{
    int dwLen, len = 0, offset = 0;

    dwLen = read(sock, pu8Buffer, 1);

    if ((int)dwLen <= 0 || m_bClosing)
        return (-1);

    offset++;

    switch (pu8Buffer[0])
    {
    case HCI_EVENT_PKT:
    {
        dwLen = read(sock, &pu8Buffer[offset], 2);
        if(dwLen == 2)
        {
            len = pu8Buffer[2];
            offset += 2;
            Log("HCI_EVENT_PKT len %d", len);
        }
        else
            Log("error HCI_EVENT_PKT, needed 2 got %d", dwLen);
    }
        break;

    case HCI_ACL_DATA_PKT:
    {
        dwLen = read(sock, &pu8Buffer[offset], 4);
        if(dwLen == 4)
        {
            len = pu8Buffer[3] | (pu8Buffer[4] << 8);
            offset += 4;
            Log("HCI_ACL_DATA_PKT, len %d", len);
        }
        else
            Log("error HCI_ACL_DATA_PKT needed 4 got %d", dwLen);
    }
        break;

    case HCI_WICED_PKT:
    {
        dwLen = read(sock, &pu8Buffer[offset], 4);
        if(dwLen == 4)
        {
            len = pu8Buffer[3] | (pu8Buffer[4] << 8);
            offset += 4;
        }
        else
            Log("error HCI_WICED_PKT,  needed 4 got %d", dwLen);
    }
        break;
    }

    if(len > 1024)
    {        
        Log("bad packet length %d", len);
        return -1; // bad packet
    }

    if (len)
    {
        DWORD lenRd = (len < (DWORD)(bufLen-offset)) ? len : (DWORD)(bufLen-offset);
        dwLen = read(sock, &pu8Buffer[offset], lenRd);
        if(dwLen != lenRd)
            Log("read error to read %d, read %d", lenRd, dwLen);
    }

    *pOffset = offset;

    return len;
}

static void *userial_read_thread(void *arg)
{
    unsigned char au8Hdr[1024 + 6];
    memset(au8Hdr, 0, 1030);
    int           offset = 0, pktLen;
    int           packetType;

    // While the port is not closed, keep reading
    while (!m_bClosing)
    {
        memset(au8Hdr, 0, 1030);
        offset = 0;
        // Read HCI packet
        pktLen = ReadNewHciPacket(au8Hdr, sizeof(au8Hdr), &offset);
        if (m_bClosing || pktLen < 0) // skip this
            break;

        if (pktLen + offset == 0)
            continue;

        packetType = au8Hdr[0];
        if (HCI_WICED_PKT == packetType)
        {
            DWORD channel_id = au8Hdr[1] | (au8Hdr[2] << 8);
            DWORD len = au8Hdr[3] | (au8Hdr[4] << 8);

            if (len > 1024)
            {
                Log("Skip bad packet %d", len);
                continue;
            }

            BYTE * pBuf = NULL;

            if(len)
            {
                // malloc and create a copy of the data.
                //  MainWindow::onHandleWicedEvent deletes the data
                pBuf = (BYTE*)malloc(len);
                memcpy(pBuf, &au8Hdr[5], len);
            }
        // send it to main thread
        // Log("read_serial_thread: send to main, opcode=%d, len = %d", channel_id, len);
        emit gMainWindow->HandleWicedEvent(channel_id, len, pBuf);
        }
    }
}

bool OpenSerialPort(UINT32 baud_rate, char * device_name)
{
    unsigned long baud=B115200;
    UINT8 data_bits;
    UINT16 parity;
    UINT8 stop_bits;
    struct termios termios;

    sleep(1);
    if (baud_rate == 115200)
        baud = B115200;
    else if (baud_rate == 1000000)
        baud = B1000000;
    else if (baud_rate == 2000000)
        baud = B2000000;
    else if (baud_rate == 921600)
        baud = B921600;
    else if (baud_rate == 3000000)
        baud = B3000000;
    Log("USERIAL_Open bad baud:%d", baud);

    data_bits = CS8;
    parity = 0;
    stop_bits = 0;

    int flags = O_RDWR | O_NOCTTY;

    Log("Bluetooth port used:%s", device_name);

    if (-1 == (sock = open(device_name, flags)))
    {
        Log("open errno %d:%s", errno, strerror(errno));
        return false;
    }

    tcflush(sock, TCIOFLUSH);

    tcgetattr(sock, &termios);

    cfmakeraw(&termios);

    termios.c_cflag |= (CRTSCTS | stop_bits);
    tcsetattr(sock, TCSANOW, &termios);

    tcflush(sock, TCIOFLUSH);
    tcsetattr(sock, TCSANOW, &termios);

    tcflush(sock, TCIOFLUSH);

    cfsetospeed(&termios, baud);
    cfsetispeed(&termios, baud);
    tcsetattr(sock, TCSANOW, &termios);

    // setup read thread
    QThread * thr = new QThread;
    Worker  * worker = new Worker();
    worker->moveToThread(thr);
    gMainWindow->connect(thr, SIGNAL(started()), worker, SLOT(process()));
    thr->start();
    return true;
}

void Worker::process()
{
    userial_read_thread(0);
}

void CloseCommPort()
{
    close(sock);
    sock = -1;
}
