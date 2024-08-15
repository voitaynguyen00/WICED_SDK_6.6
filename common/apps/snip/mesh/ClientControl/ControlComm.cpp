#include "stdafx.h"
#include "ControlComm.h"
#include "..\..\..\..\common\include\hci_control_api.h"

extern void Log(WCHAR* _Format, ...);
extern void HandleWicedEvent(DWORD identifier, DWORD len, BYTE *p_data);
extern void HandleHciEvent(BYTE *p_data, DWORD len);

//extern CHFTestDlg  *pCHFTestDlg;

static char _parityChar[] = "NOEMS";
static char* _stopBits[] = { "1", "1.5", "2" };

//
//Class ComHelper Implementation
//
ComHelper::ComHelper(HWND hWnd) :
    m_handle(INVALID_HANDLE_VALUE),
    m_hWnd(hWnd)
{
    memset(&m_OverlapRead, 0, sizeof(m_OverlapRead));
    memset(&m_OverlapWrite, 0, sizeof(m_OverlapWrite));
}

ComHelper::~ComHelper()
{
    ClosePort();
}

DWORD WINAPI ReadThread(LPVOID lpdwThreadParam)
{
    ComHelper *pComHelper = (ComHelper *)lpdwThreadParam;
    return pComHelper->ReadWorker();
}

//
//Open Serial Bus driver
//
BOOL ComHelper::OpenPort(int port, int baudRate)
{
    char lpStr[20];
    sprintf_s(lpStr, 20, "\\\\.\\COM%d", port);

    // open once only
    if (m_handle != NULL && m_handle != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_handle);
    }
    m_handle = CreateFileA(lpStr,
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        NULL);

    if (m_handle != NULL&& m_handle != INVALID_HANDLE_VALUE)
    {
        // setup serial bus device
        BOOL bResult;
        DWORD dwError = 0;
        COMMTIMEOUTS commTimeout;
        COMMPROP commProp;
        COMSTAT comStat;
        DCB serial_config;

        PurgeComm(m_handle, PURGE_RXABORT | PURGE_RXCLEAR |PURGE_TXABORT | PURGE_TXCLEAR);

        // create events for Overlapped IO
        m_OverlapRead.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

        m_OverlapWrite.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

        // set comm timeout
        memset(&commTimeout, 0, sizeof(COMMTIMEOUTS));
        commTimeout.ReadIntervalTimeout = 1;
//		commTimeout.ReadTotalTimeoutConstant = 1000;
//		commTimeout.ReadTotalTimeoutMultiplier = 10;
        commTimeout.WriteTotalTimeoutConstant = 1000;
//		commTimeout.WriteTotalTimeoutMultiplier = 1;
        bResult = SetCommTimeouts(m_handle, &commTimeout);

        // set comm configuration
        memset(&serial_config, 0, sizeof(serial_config));
        serial_config.DCBlength = sizeof (DCB);
        bResult = GetCommState(m_handle, &serial_config);

        serial_config.BaudRate = baudRate;
        serial_config.ByteSize = 8;
        serial_config.Parity = NOPARITY;
        serial_config.StopBits = ONESTOPBIT;
        serial_config.fBinary = TRUE;
        serial_config.fOutxCtsFlow = TRUE; // TRUE;
        serial_config.fRtsControl = RTS_CONTROL_HANDSHAKE;
        serial_config.fOutxDsrFlow = FALSE; // TRUE;
        serial_config.fDtrControl = FALSE;

        serial_config.fOutX = FALSE;
        serial_config.fInX = FALSE;
        serial_config.fErrorChar = FALSE;
        serial_config.fNull = FALSE;
        serial_config.fParity = FALSE;
        serial_config.XonChar = 0;
        serial_config.XoffChar = 0;
        serial_config.ErrorChar = 0;
        serial_config.EofChar = 0;
        serial_config.EvtChar = 0;
        bResult = SetCommState(m_handle, &serial_config);

        if (!bResult)
            Log (L"OpenPort SetCommState failed %d\n", GetLastError());
        else
        {
            // verify CommState
            memset(&serial_config, 0, sizeof(serial_config));
            serial_config.DCBlength = sizeof (DCB);
            bResult = GetCommState(m_handle, &serial_config);
        }

        // set IO buffer size
        memset(&commProp, 0, sizeof(commProp));
        bResult = GetCommProperties(m_handle, &commProp);

        if (!bResult)
            Log (L"OpenPort GetCommProperties failed %d\n", GetLastError());
        else
        {
            // use 4096 byte as preferred buffer size, adjust to fit within allowed Max
            commProp.dwCurrentTxQueue = 4096;
            commProp.dwCurrentRxQueue = 4096;
            if (commProp.dwCurrentTxQueue > commProp.dwMaxTxQueue)
                commProp.dwCurrentTxQueue = commProp.dwMaxTxQueue;
            if (commProp.dwCurrentRxQueue > commProp.dwMaxRxQueue)
                commProp.dwCurrentRxQueue = commProp.dwMaxRxQueue;
            bResult = SetupComm(m_handle, commProp.dwCurrentRxQueue, commProp.dwCurrentTxQueue);

            if (!bResult)
                Log (L"OpenPort SetupComm failed %d\n", GetLastError());
            else
            {
                memset(&commProp, 0, sizeof(commProp));
                bResult = GetCommProperties(m_handle, &commProp);

                if (!bResult)
                    Log (L"OpenPort GetCommProperties failed %d\n", GetLastError());
            }
        }
        memset(&comStat, 0, sizeof(comStat));
        ClearCommError(m_handle, &dwError, &comStat);
    }
    Log (L"Opened COM%d at speed: %u", port, baudRate);
    m_bClosing = FALSE;
    m_hShutdown = CreateEvent(NULL, FALSE, FALSE, NULL);

    // create thread to read the uart data.
    DWORD dwThreadId;
    m_hThreadRead = CreateThread(NULL, 0, ReadThread, this, 0, &dwThreadId);
    if (!m_hThreadRead)
    {
        Log (L"Could not create read thread \n");
        ClosePort();
    }
    return m_handle != NULL && m_handle != INVALID_HANDLE_VALUE;
}

void ComHelper::ClosePort()
{
    SetEvent(m_hShutdown);
    WaitForSingleObject(m_hThreadRead, INFINITE);

    if (m_OverlapRead.hEvent != NULL)
    {
        CloseHandle(m_OverlapRead.hEvent);
        m_OverlapRead.hEvent = NULL;
    }

    if (m_OverlapWrite.hEvent != NULL)
    {
        CloseHandle(m_OverlapWrite.hEvent);
        m_OverlapWrite.hEvent = NULL;
    }
    if (m_handle != NULL && m_handle != INVALID_HANDLE_VALUE)
    {
        // drop DTR
        EscapeCommFunction(m_handle, CLRDTR);
        // purge any outstanding reads/writes and close device handle
        PurgeComm(m_handle, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
        CloseHandle(m_handle);
        m_handle = INVALID_HANDLE_VALUE;
    }
}

BOOL ComHelper::IsOpened()
{
    return (m_handle != NULL && m_handle != INVALID_HANDLE_VALUE);
}

// read a number of bytes from Serial Bus Device
// Parameters:
//	lpBytes - Pointer to the buffer
//	dwLen   - number of bytes to read
// Return:	Number of byte read from the device.  
//
DWORD ComHelper::Read(LPBYTE lpBytes, DWORD dwLen)
{
    LPBYTE p = lpBytes;
    DWORD Length = dwLen;
    DWORD dwRead = 0;
    DWORD dwTotalRead = 0;

    // Loop here until request is fulfilled
    while (Length)
    {
        DWORD dwRet = WAIT_TIMEOUT;
        dwRead = 0;
        ResetEvent(m_OverlapRead.hEvent);
//        m_OverlapRead.Internal = ERROR_SUCCESS;
//        m_OverlapRead.InternalHigh = 0;
        if (!ReadFile(m_handle, (LPVOID)p, Length, &dwRead, &m_OverlapRead))
        {
            // Overlapped IO returns FALSE with ERROR_IO_PENDING
            if (GetLastError() != ERROR_IO_PENDING)
            {
                Log (L"ComHelper::ReadFile failed with %ld\n", GetLastError());
                m_bClosing = TRUE;
                dwTotalRead = 0;
                PostMessage(m_hWnd, WM_CLOSE, 0, 0);
                break;
            }

//            //Clear the LastError and wait for the IO to Complete
//            SetLastError(ERROR_SUCCESS);
//			dwRet = WaitForSingleObject(m_OverlapRead.hEvent, 10000);
            HANDLE handles[2];
            handles[0] = m_OverlapRead.hEvent;
            handles[1] = m_hShutdown;

            dwRet = WaitForMultipleObjects(2, handles, FALSE, INFINITE);
            // dwRet = WaitForSingleObject(m_OverlapRead.hEvent, INFINITE);
            if (dwRet == WAIT_OBJECT_0 + 1)
            {
                m_bClosing = TRUE;
                break;
            }
            else if (dwRet != WAIT_OBJECT_0)
            {
                Log (L"ComHelper::WaitForSingleObject returned with %ld err=%d\n", dwRet, GetLastError());
                dwTotalRead = 0;
                break;
            }

            // IO completed, retrieve Overlapped result
            GetOverlappedResult(m_handle, &m_OverlapRead, &dwRead, TRUE);

//			// if dwRead is not updated, retrieve it from OVERLAPPED structure
//            if (dwRead == 0)
//                dwRead = (DWORD)m_OverlapRead.InternalHigh;
        }
        if (dwRead > Length)
            break;
        p += dwRead;
        Length -= dwRead;
        dwTotalRead += dwRead;
    }

    return dwTotalRead;
}

DWORD ComHelper::SendWicedCommand(UINT16 command, LPBYTE payload, DWORD len)
{
    BYTE    data[1040];
    char    descr[30];
    int     header = 0;

    data[header++] = HCI_WICED_PKT;
    data[header++] = command & 0xff;
    data[header++] = (command >> 8) & 0xff;
    data[header++] = len & 0xff;
    data[header++] = (len >> 8) & 0xff;

    memcpy(&data[header], payload, len);

    sprintf_s(descr, sizeof(descr), "Xmit %3u bytes: ", len + header);
    DumpData(descr, data, len + header, 1);

    DWORD written = Write(data, len + header);

    return written;
}

// Write a number of bytes to Serial Bus Device
// Parameters:
//	lpBytes - Pointer to the buffer
//	dwLen   - number of bytes to write
// Return:	Number of byte Written to the device.  
//
DWORD ComHelper::Write(LPBYTE lpBytes, DWORD dwLen)
{
    LPBYTE p = lpBytes;
    DWORD Length = dwLen;
    DWORD dwWritten = 0;
    DWORD dwTotalWritten = 0;

    if (m_handle == INVALID_HANDLE_VALUE)
    {
        Log (L"ERROR - COM Port not opened");
        return (0);
    }

    while (Length)
    {
        dwWritten = 0;
        SetLastError(ERROR_SUCCESS);
        ResetEvent(m_OverlapWrite.hEvent);
        if (!WriteFile(m_handle, p, Length, &dwWritten, &m_OverlapWrite))
        {
            if (GetLastError() != ERROR_IO_PENDING)
            {
                Log (L"ComHelper::WriteFile failed with %ld", GetLastError());
                break;
            }
            DWORD dwRet = WaitForSingleObject(m_OverlapWrite.hEvent, INFINITE);
            if (dwRet != WAIT_OBJECT_0)
            {
                Log (L"ComHelper::Write WaitForSingleObject failed with %ld\n", GetLastError());
                break;
            }
            GetOverlappedResult(m_handle, &m_OverlapWrite, &dwWritten, FALSE);
        }
        if (dwWritten > Length)
            break;
        p += dwWritten;
        Length -= dwWritten;
        dwTotalWritten += dwWritten;
    }
    return dwTotalWritten;
}



DWORD ComHelper::ReadNewHciPacket(BYTE * pu8Buffer, int bufLen, int * pOffset)
{
    DWORD dwLen, len = 0, offset = 0;

    dwLen = Read(&pu8Buffer[offset], 1);

    if (dwLen == 0)
        return (0);

    offset++;

    switch (pu8Buffer[0])
    {
    case HCI_EVENT_PKT:
        Read(&pu8Buffer[offset], 2);
        len = pu8Buffer[2];
        offset += 2;
        break;

    case HCI_ACL_DATA_PKT:
        Read(&pu8Buffer[offset], 4);
        len = pu8Buffer[3] | (pu8Buffer[4] << 8);
        offset += 4;
        break;

    case HCI_WICED_PKT:
        Read(&pu8Buffer[offset], 4);
        len = pu8Buffer[3] | (pu8Buffer[4] << 8);
        offset += 4;
        break;
    }

    if (len)
        Read(&pu8Buffer[offset], min(len, (DWORD)bufLen));

    *pOffset = offset;

    return len;
}


DWORD ComHelper::ReadWorker()
{
    unsigned char au8Hdr[1024 + 6];
    int           offset = 0, pktLen;
    char          descr[30];
    int           packetType;
    int bytesToWrite = 0;

    while (1)
    {
        pktLen = ReadNewHciPacket(au8Hdr, sizeof(au8Hdr), &offset);
        if (m_bClosing)
            break;

        if (pktLen + offset == 0)
            continue;

        sprintf_s(descr, sizeof(descr), "Rcvd %3u bytes: ", pktLen + offset);
        DumpData(descr, au8Hdr, pktLen + offset, 1);

        packetType = au8Hdr[0];
        switch (packetType)
        {
        case HCI_EVENT_PKT:
            HandleHciEvent(au8Hdr, pktLen + offset);
            break;

        case HCI_ACL_DATA_PKT:
            break;

        case HCI_WICED_PKT:
            {
                DWORD channel_id = au8Hdr[1] | (au8Hdr[2] << 8);
                DWORD len = au8Hdr[3] | (au8Hdr[4] << 8);

                // au8Hdr[5] is the Reserved byte, ignore it.
                HandleWicedEvent(channel_id, len, &au8Hdr[5]);
            }
            break;
        }
    }
    return 0;
}

// prints data in ascii format to the std out
void DumpData(char *description, void* p, UINT32 length, UINT32 max_lines)
{
    char    buff[100];
    UINT    i, j;
    char    full_buff[3000];

    if (p != NULL)
    {
        for (j = 0; j < max_lines && (32 * j) < length; j++)
        {
            for (i = 0; (i < 32) && ((i + (32 * j)) < length); i++)
            {
                sprintf_s(&buff[3 * i], sizeof(buff) - 3 * i, "%02x \n", ((UINT8*)p)[i + (j * 32)]);
            }
            if (j == 0)
            {
                strcpy_s(full_buff, sizeof(full_buff), description);
                strcat_s(full_buff, sizeof(full_buff), buff);
//                pCHFTestDlg->Log (L"%s", full_buff);
                OutputDebugStringA(full_buff);
            }
            else
            {
//                pCHFTestDlg->Log (L"%s", buff);
                OutputDebugStringA(buff);
            }
        }
    }
}



