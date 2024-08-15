#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <map>
#include "DSDLCppWrapper.h"
#include "oscommon.h"
#include "trancommon.h"
#include "logmanager.h"
#include "miscfunc.h"
#include <stdarg.h>

#ifdef WIN32
#include "cysocketserialconnector.h"
#endif

using namespace std;

typedef     map<string,string> MAPSTR2STR;
string      g_sLogName;     // log file path name. Empty means no log.
LogManager  g_oLog;

// by default send trace received in EventDeviceWicedTrace and EventDeviceWicedTrace to the SpyLite
bool        g_SpyLiteTraceEnabled = true;   

//global buffer to receive bytes from device and keep not handled bytes for next read
UINT8_t                 g_arRxBytes[1024];
UINT32_t                g_nNumRxBytes = 0;

CDSDLInstanceCollection *g_collection = NULL;
int                     g_collectionCount = 0;
int                     g_collectionIdx = 0;

const int   THREAD_SLEEP_INTERVAL_MS = 2;

#ifdef WIN32
#include <WinSock2.h>
SOCKET log_sock = INVALID_SOCKET;

// mapping between wiced trace types and spy trace types (evt, cmd, rx data, tx data)
static int wiced_trace_to_spy_trace[] = { 0, 4, 3, 6, 7 };

void TraceHciPkt(BYTE type, const BYTE *buffer, USHORT length)
{
    SOCKADDR_IN socket_addr;
    static int initialized = FALSE;
    BYTE buf[1100];
    USHORT offset = 0;
    USHORT *p = (USHORT*)buf;

    if (!initialized)
    {
        initialized = TRUE;

        WSADATA wsaData;
        int err = WSAStartup(MAKEWORD(2, 0), &wsaData);
        if (err != 0)
            return;
        log_sock = socket(AF_INET, SOCK_DGRAM, 0);

        if (log_sock == INVALID_SOCKET)
            return;

        memset(&socket_addr, 0, sizeof(socket_addr));
        socket_addr.sin_family = AF_INET;
        socket_addr.sin_addr.s_addr = ADDR_ANY;
        socket_addr.sin_port = 0;

        err = bind(log_sock, (SOCKADDR *)&socket_addr, sizeof(socket_addr));
        if (err != 0)
        {
            closesocket(log_sock);
            log_sock = INVALID_SOCKET;
            return;
        }
    }
    if (log_sock == INVALID_SOCKET)
        return;

    if (length > 1024)
        length = 1024;

    *p++ = wiced_trace_to_spy_trace[type];
    *p++ = length;
    *p++ = 0;
    *p++ = 1;
    memcpy(p, buffer, length);

    memset(&socket_addr, 0, sizeof(socket_addr));
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_addr.s_addr = ntohl(0x7f000001);
    socket_addr.sin_port = 9876;

    length = sendto(log_sock, (const char *)buf, length + 8, 0, (SOCKADDR *)&socket_addr, sizeof(SOCKADDR_IN));
}
#endif

string
format(const char *fmt_str, ...)
{
    char buf[1024];
    va_list marker;

    va_start (marker, fmt_str );
#ifdef LINUX
	vsnprintf (buf, sizeof(buf), fmt_str, marker );
#else
	vsnprintf_s (buf, sizeof(buf), _TRUNCATE, fmt_str, marker );
#endif
    va_end (marker);

    return string(buf);
}

string
ProcessArgsAndPrepare (int argc, char* argv[], string * sPort, string * sBaudrate, string * sMsgDefinitionsCmd, string * sMsgDefinitionsEvt, string* sBaudrateDownload, string* sRTS, string* sCTS, bool* hostModeEnabled)
{
    string      sErr = "";
    int         nArgIndex = 1;
    string      sArg;
    string      sVal;
    
    *sPort = "";
    *sBaudrate = "";
    *sBaudrateDownload = "";

    *sMsgDefinitionsCmd = "";
    *sMsgDefinitionsEvt = "";

    *sRTS = "";
    *sCTS = "";

	*hostModeEnabled = false;

    while ((nArgIndex < argc) && (sErr == ""))
    {   
        if ((argv [nArgIndex][0] == '-') || (argv [nArgIndex][0] == '/'))
        {
            sArg = argv [nArgIndex] + 1;
            STRING_TO_UPPER((char*)sArg.c_str());

            nArgIndex++;
            if (nArgIndex < argc)
                sVal = argv [nArgIndex];
            else
                sVal = "";

            nArgIndex++;

			if (sArg == "HOSTMODE")
			{
				*hostModeEnabled = true;
			}
			else if (sVal != "")
            {
                if ((sArg == "PORT") && (*sPort == ""))
                {
                    *sPort = sVal;
                }
                else if ((sArg == "BAUDRATE") && (*sBaudrate == ""))
                {
                    *sBaudrate = sVal;
                }
                else if ((sArg == "BAUDRATEDWNLD") && (*sBaudrateDownload == ""))
                {
                    *sBaudrateDownload = sVal;
                }
                else if (sArg == "LOGTO")
                {
                    g_sLogName = sVal;
                }
                else if (sArg == "LOGTOSPY")
                {
                    STRING_TO_UPPER((char*)sVal.c_str());
                    g_SpyLiteTraceEnabled = (sVal == "Y" || sVal == "YES" || sVal == "1");
                }
                else if ((sArg == "MSGCMDDEFS") && (*sMsgDefinitionsCmd == ""))
                {
                    *sMsgDefinitionsCmd = sVal;
                }
                else if ((sArg == "MSGEVTDEFS") && (*sMsgDefinitionsEvt == ""))
                {
                    *sMsgDefinitionsEvt = sVal;
                }
                else if ((sArg == "RTS") && (*sRTS == ""))
                {
                    *sRTS = sVal;
                }
                else if ((sArg == "CTS") && (*sCTS == ""))
                {
                    *sCTS = sVal;
                }				
            }
            else
            {
                sErr = "Missing value for argument " + sArg;
            }
        }
        else
            sErr = "Unexpected argument " + string(argv [nArgIndex]);
    }

    if (sErr == "")
    {
        if (!hostModeEnabled && (*sPort == ""))
            sErr = "Communication port must be specified.";
        else if (*sMsgDefinitionsCmd == "")
            sErr = "Message definitions file must be specified.";
    }

    if (sErr != "")
    {
        fprintf (stderr, "ERROR! %s\n", sErr.c_str ());
        fprintf (stderr, "\n");
        fprintf (stderr, "\n");
    }
    if ((sErr != "") || (argc == 1))
    {
        fprintf (stderr, "Calling format: \n");
        fprintf (stderr, "    %s -PORT portid -BAUDRATE baud -MSGCMDDEFS messagedefinitionfile [-MSGEVTDEFS eventDefinitionFile] [-LOGTO logfile] [-LOGTOSPY (yes or no)] [-HOSTMODE]\n", argv [0]);
        fprintf (stderr, "  where \n");
        fprintf (stderr, "\n");
        fprintf (stderr, "    MSGCMDDEFS is the file containing the definitions of various message structures for commands and optionaly for events\n");
        fprintf (stderr, "\n");
        fprintf (stderr, "    MSGEVTDEFS is the file containing the definitions of various message structures for events\n");
        fprintf (stderr, "\n");
        fprintf (stderr, "    PORT is a standard platform-specific transport specifier, e.g. COM3 on Windows or tty-serxxx on another system\n");
        fprintf (stderr, "\n");
        fprintf (stderr, "    BAUDRATE, just like PORT, is a platform-specific speed specification, e.g. 115200\n");
        fprintf (stderr, "\n");
        fprintf (stderr, "    BAUDRATEDWNLD, baud rate to switch to at FW download, e.g. 3000000 (default value is 3000000)\n");
        fprintf (stderr, "\n");
        fprintf (stderr, "    LOGTO is an optional parameter to specify the name of the log file\n");
        fprintf (stderr, "\n");
        fprintf (stderr, "    LOGTOSPY is an optional parameter to enable/disable trace in the SpyLite, e.g. yes or no. Default is yes\n");
        fprintf (stderr, "\n");
		fprintf	(stderr, "    HOSTMODE is an optional parameter to enable host mode of operation. When this option is enabled, HWTalk ignores the port configuration and establishes a socket connection to WICED-X daemon running on the host PC.\n");
		fprintf	(stderr, "\n");
        fprintf (stderr, "\n");
        fprintf (stderr, "\n");
                 
    }

    return sErr;
}

string
HexByteString (UINT8_t * pBuf, UINT32_t nNumBytes)
{
    string sHex;
    if (pBuf && (nNumBytes > 0))
    {
        BytesToHexString (pBuf, nNumBytes, sHex); 
    }
    else
        sHex = "INVALID";

    return sHex;
}

string
FetchMoreReceivedBytes (CSerialConnector * pSerialConnector, UINT8_t * pDataBuf, UINT32_t nBufSize, UINT32_t * nTotalRxByteCount)
{
    string      sErr;
    UINT32_t    nNumRxBytes;

    if (*nTotalRxByteCount >= nBufSize)
        sErr = "FetchMoreReceivedBytes: Rx buffer full!";
    else
    {
        sErr = pSerialConnector->ReadReceivedBytes (pDataBuf + *nTotalRxByteCount, nBufSize - *nTotalRxByteCount, &nNumRxBytes);
        if (sErr == "")
        {
            if (nNumRxBytes > 0)
                g_oLog.WriteRcvData (HexByteString (pDataBuf + *nTotalRxByteCount, nNumRxBytes));
            //else
                //g_oLog.WriteLog ("FetchMoreReceivedBytes: 0 bytes received");

            *nTotalRxByteCount += nNumRxBytes;
        }
        else
            sErr = string("FetchMoreReceivedBytes: ReadReceivedBytes failed: ") + sErr;
    }

    return sErr;
}

bool
SendCommandAndReceiveResponse (CSerialConnector * pSerialConnector, 
                                 UINT8_t * pTxData, 
                                 UINT32_t nTxNumBytes, 
                                 UINT8_t * pRxBuf, 
                                 UINT32_t nRxBufSize, 
                                 UINT32_t * nNumRxBytes, 
                                 UINT32_t nMaxWaitMS, 
                                 string * sErr)
{
    UINT32_t    nNumTries = 0;
    UINT32_t    nMaxLoopTries = nMaxWaitMS / THREAD_SLEEP_INTERVAL_MS;

    *nNumRxBytes = 0;

    if (nMaxLoopTries < 1)
        nMaxLoopTries = 1;

    *sErr = "";

    if (pTxData)
    {
        pSerialConnector->TransmitBytes (pTxData, nTxNumBytes);
        g_oLog.WriteSendData (HexByteString (pTxData, nTxNumBytes));
    }

    UINT32_t    tStart = GetCurrentOSTimeMS ();

    while ((nNumTries < nMaxLoopTries) && (*sErr == "") && (GetCurrentOSTimeMS () - tStart <= nMaxWaitMS) && (*nNumRxBytes < nRxBufSize))
    {
        nNumTries++;
        PutThreadToSleep (THREAD_SLEEP_INTERVAL_MS);
        *sErr = FetchMoreReceivedBytes (pSerialConnector, pRxBuf, nRxBufSize, nNumRxBytes);
    }

    if(*nNumRxBytes)
        g_oLog.WriteRcvData (HexByteString (pRxBuf, *nNumRxBytes));

    return (*sErr == "");
}


string
SendReceiveCompare(CSerialConnector * pConnector,
    UINT8_t     *arHciCommandTx, UINT32_t nHciCommandTxLen,
    UINT8_t     *arBytesExpectedRx, UINT32_t nBytesExpectedRxLen,
    UINT32_t    nMaxWaitMS)
{
    string      sErr;
    UINT8_t     arRxBytes[128];
    UINT32_t    nNumRxBytes = 0;

    SendCommandAndReceiveResponse(pConnector, arHciCommandTx, nHciCommandTxLen, arRxBytes, nBytesExpectedRxLen, &nNumRxBytes, nMaxWaitMS, &sErr);

    if(sErr != "")
        sErr = string("SendReceiveCompare: SendCommandAndReceiveResponse failed:") + sErr;
    else if (nNumRxBytes < nBytesExpectedRxLen)
        sErr = "SendReceiveCompare: invalid length of received data";
    else if (memcmp(arRxBytes, arBytesExpectedRx, nBytesExpectedRxLen) != 0)
        sErr = "SendReceiveCompare: invalid received data";
    return sErr;
}

string
SendHciReset(CSerialConnector * pConnector)
{
    string      sErr;
    UINT8_t     arHciCommandTx[] = { 0x01, 0x03, 0x0C, 0x00 };
    UINT8_t     arBytesExpectedRx[] = { 0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00 };

    return SendReceiveCompare(pConnector, arHciCommandTx, sizeof(arHciCommandTx), arBytesExpectedRx, sizeof(arBytesExpectedRx), 500);
}

string
SendUpdateBaudRate(CSerialConnector * pConnector, int newBaudRate)
{
    UINT8_t     arHciCommandTx[] = { 0x01, 0x18, 0xFC, 0x06, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA };
    UINT8_t     arBytesExpectedRx[] = { 0x04, 0x0E, 0x04, 0x01, 0x18, 0xFC, 0x00 };

    arHciCommandTx[6] = (UINT8_t)newBaudRate;
    arHciCommandTx[7] = (UINT8_t)(newBaudRate >> 8);
    arHciCommandTx[8] = (UINT8_t)(newBaudRate >> 16);
    arHciCommandTx[9] = (UINT8_t)(newBaudRate >> 24);

    return SendReceiveCompare(pConnector, arHciCommandTx, sizeof(arHciCommandTx), arBytesExpectedRx, sizeof(arBytesExpectedRx), 500);
}

string
SendDownloadMinidriver(CSerialConnector * pConnector)
{
    UINT8_t     arHciCommandTx[] = { 0x01, 0x2E, 0xFC, 0x00 };
    UINT8_t     arBytesExpectedRx[] = { 0x04, 0x0E, 0x04, 0x01, 0x2E, 0xFC, 0x00 };

    return SendReceiveCompare(pConnector, arHciCommandTx, sizeof(arHciCommandTx), arBytesExpectedRx, sizeof(arBytesExpectedRx), 500);
}

string
SendHcdRecord(CSerialConnector * pConnector, UINT32_t nAddr, UINT32_t nHCDRecSize, UINT8_t * arHCDDataBuffer)
{
    UINT8_t     arHciCommandTx[261] = { 0x01, 0x4C, 0xFC, 0x00 };
    UINT8_t     arBytesExpectedRx[] = { 0x04, 0x0E, 0x04, 0x01, 0x4C, 0xFC, 0x00 };

    arHciCommandTx[3] = (UINT8_t)(4 + nHCDRecSize);
    arHciCommandTx[4] = (UINT8_t)nAddr;
    arHciCommandTx[5] = (UINT8_t)(nAddr >> 8);
    arHciCommandTx[6] = (UINT8_t)(nAddr >> 16);
    arHciCommandTx[7] = (UINT8_t)(nAddr >> 24);
    memcpy(&arHciCommandTx[8], arHCDDataBuffer, nHCDRecSize);

    return SendReceiveCompare(pConnector, arHciCommandTx, sizeof(arHciCommandTx), arBytesExpectedRx, sizeof(arBytesExpectedRx), 500);
}

string
SendLaunchRam(CSerialConnector * pConnector)
{
    string      sErr;
    UINT8_t     arHciCommandTx[] = { 0x01, 0x4E, 0xFC, 0x04, 0xFF, 0xFF, 0xFF, 0xFF };
    UINT8_t     arBytesExpectedRx[] = { 0x04, 0x0E, 0x04, 0x01, 0x4E, 0xFC, 0x00 };

    return SendReceiveCompare(pConnector, arHciCommandTx, sizeof(arHciCommandTx), arBytesExpectedRx, sizeof(arBytesExpectedRx), 500);
}


string
ReadNextHCDRecord(FILE * fHCD, UINT32_t* nAddr, UINT32_t* nHCDRecSize, UINT8_t* arHCDDataBuffer, bool* bIsLaunch)
{
    string      sErr;
    const       int HCD_LAUNCH_COMMAND = 0x4E;
    const       int HCD_WRITE_COMMAND = 0x4C;
    const       int HCD_COMMAND_BYTE2 = 0xFC;

    UINT8_t     arRecHeader[3];
    UINT8_t     byRecLen;
    UINT8_t     arAddress[4];

    *bIsLaunch = false;

    do
    {
        if (fread(arRecHeader, 1, 3, fHCD) != 3)               // Unexpected EOF
        {
            sErr = "ReadNextHCDRecord: fread failed";
            break;
        }

        byRecLen = arRecHeader[2];

        if ((byRecLen < 4) || (arRecHeader[1] != HCD_COMMAND_BYTE2) ||
            ((arRecHeader[0] != HCD_WRITE_COMMAND) && (arRecHeader[0] != HCD_LAUNCH_COMMAND)))
        {
            sErr = "ReadNextHCDRecord: Wrong HCD file format trying to read the command information";
            break;
        }

        if (fread(arAddress, sizeof(arAddress), 1, fHCD) != 1)      // Unexpected EOF
        {
            sErr = "ReadNextHCDRecord:Wrong HCD file format trying to read 32-bit address";
            break;
        }

        *nAddr = arAddress[0] + (arAddress[1] << 8) + (arAddress[2] << 16) + (arAddress[3] << 24);

        *bIsLaunch = (arRecHeader[0] == HCD_LAUNCH_COMMAND);

        *nHCDRecSize = byRecLen - 4;

        if (*nHCDRecSize > 0)
        {
            if (fread(arHCDDataBuffer, 1, *nHCDRecSize, fHCD) != *nHCDRecSize)   // Unexpected EOF
            {
                sErr = "ReadNextHCDRecord: Not enough HCD data bytes in record";
                break;
            }
        }
    }while(false);

    return sErr;
}

// disconnects and connects again with new baud rate
string
SetBaudRate(CSerialConnector **ppConnector, const char* port, int nBaudRate)
{
    string sErr;
    string sConnectionParams;

    // disconnect and delete object
    (*ppConnector)->Disconnect();
    delete (*ppConnector);

    // create string with connection parameters and connect to device
    sConnectionParams = format("PORT=%s\n BAUDRATE=%d\n", port, nBaudRate);
    *ppConnector = CreateSerialConnector(sConnectionParams.c_str(), &sErr, NULL, NULL);
    if (!*ppConnector)
        sErr = "SetBaudRate: Failed to open the transport connection with " + sConnectionParams + " res:" + sErr;
    else
        sErr = "";
    return sErr;
}

void
resetCollection() {
    if(g_collection) {
        delete g_collection;
        if(g_collectionCount)
            g_oLog.WriteLog(format("resetCollection: dropped %d messages", g_collectionCount));
    }
    g_collection = NULL;
    g_collectionCount = 0;
    g_collectionIdx = 0;
}

string
handleReceivedData(CDSDLFactory * pMsgFactoryEvt) {
    string      sErr;
    int         used = 0;
    do {
        
        resetCollection();
        
        // exit with success if no bytes
        if (g_nNumRxBytes == 0) {
            g_oLog.WriteLog("handleReceivedData: received 0 bytes");
            break;
        }

        // decode received packet
        g_collection = pMsgFactoryEvt->CreateInstancesFromBytes((const char*)g_arRxBytes, g_nNumRxBytes, &used);
        // exit with success if can't parse - probably data is comming
        if (!g_collection) {
            g_oLog.WriteLog(string("handleReceivedData: Failed to decode received message (probably it is not fully arrived yet): ") +  HexByteString(g_arRxBytes, g_nNumRxBytes));
            break;
        }
        g_collectionCount = g_collection->CountInstances();
        // exit with success if no message is decoded - probably data is comming
        if(g_collectionCount == 0) {
            g_oLog.WriteLog(string("handleReceivedData: got empty collection ") +  HexByteString(g_arRxBytes, g_nNumRxBytes));
            break;
        }
        //posit before first instance in the collection
        g_collectionIdx = -1;

        // leave remaining (not decoded data) in the buffer
        g_nNumRxBytes -= used;
        if(g_nNumRxBytes > 0) {
            memcpy(g_arRxBytes, &g_arRxBytes[used], g_nNumRxBytes);
            g_oLog.WriteLog(format("handleReceivedData: remains %d bytes", g_nNumRxBytes));
        }
        // if trace is configured then print all received messages to the log file or SpyLite
        if(g_sLogName.length() > 0 || g_SpyLiteTraceEnabled) {
            CDSDLInstance *inst;
            string fieldName;
            for (int i = 0; i < g_collectionCount; ++i)
            {
                inst = g_collection->GetInstanceByIndex(i);
                fieldName = inst->GetTypeName();
                if (!inst)
                    continue;
                if(g_sLogName.length() > 0)
                    g_oLog.WriteLog("Received Name:" + fieldName + "\n" + inst->GetAllFields());
#ifdef WIN32
                if(g_SpyLiteTraceEnabled) {
                    UINT16_t len;
                    const UINT8_t* p;

                    len = (UINT16_t)inst->GetTotalByteLength();
                    p = inst->GetByteBuffer()->GetByteBuffer();
                    if(fieldName == "EventDeviceHciTrace")
                        TraceHciPkt(p[5] + 1, &p[6], len - 6);
                    else if(fieldName == "EventDeviceWicedTrace")
                        TraceHciPkt(0, &p[5], len - 5);
                }
#endif
            }
        }
    }while(false);
    return sErr;
}

// receives and deletes all data till there is no partial message received but not longer then nMaxWaitMS
string
flashPort(CSerialConnector *pConnector, CDSDLFactory *pMsgFactoryEvt, UINT32_t nMaxWaitMS)
{
    string      sErr;
    UINT32_t    nNumRxBytesStart;
    UINT32_t    nNumTries = 0;
    UINT32_t    nMaxLoopTries = nMaxWaitMS / THREAD_SLEEP_INTERVAL_MS;
    UINT32_t    tStart = GetCurrentOSTimeMS ();
    if(nMaxLoopTries < 1)
        nMaxLoopTries = 1;

    while ((nNumTries < nMaxLoopTries) && (sErr == "") && (GetCurrentOSTimeMS () - tStart <= nMaxWaitMS) && (g_nNumRxBytes < sizeof(g_arRxBytes)))
    {
        nNumTries++;
        PutThreadToSleep (THREAD_SLEEP_INTERVAL_MS);
        nNumRxBytesStart = g_nNumRxBytes;
        sErr = FetchMoreReceivedBytes (pConnector, g_arRxBytes, sizeof(g_arRxBytes), &g_nNumRxBytes);
        // if we received new bytes then try to handle them
        if(nNumRxBytesStart != g_nNumRxBytes && sErr == "") {
            sErr = handleReceivedData(pMsgFactoryEvt);
        }
        //exit loop if there is no unhandled data
        if(g_nNumRxBytes == 0)
            break;
    }
    resetCollection();
    if(g_nNumRxBytes != 0) {
        g_oLog.WriteLog(format("flashPort: dropped %d bytes", g_nNumRxBytes));
        g_nNumRxBytes = 0;
    }
    return sErr;
}

/*
    <send> := send <structureName> <fields>
    <structureName> - any non-abstract structure name from sdl file
    <fields> := <field> | <field>;<fields>
    <field> := <fieldname>=<fieldValue>
    <fieldName> - name of the any member of structure <structureName>
    <fieldValue> - value of the field <fieldName>

    returns "success"
    on error returns empty string
*/
string
sendRequest(CSerialConnector * pConnector, CDSDLFactory * pMsgFactoryCmd, CDSDLFactory * pMsgFactoryEvt, const char* structureName, const char* fields)
{
    string          sErr;

    g_oLog.WriteLog(string("sendRequest: structureName:") + structureName + " fields:" + fields);

    do {
        // flash port and forget received bytes and collection
        sErr = flashPort(pConnector, pMsgFactoryEvt, 1000);
        if (sErr != "")
            break;

        // create instance of structure
        CDSDLInstance   *inst = pMsgFactoryCmd->CreateInstanceOfType(structureName, fields);
        if (!inst) {
            sErr = "sendRequest: Failed to create structure";
            break;
        }

        // print it to the log
        g_oLog.WriteLog("Sending Name:" + inst->GetTypeName() + "\n" + inst->GetAllFields());

        //std::string sMessage = inst->GetAllFields("\n\t");
        //std::string sHexBuffer = inst->GetByteBufferAsHexStr();

        // send structure to the device and receive packets during 500ms
        CDSDLBuffer* bByteBuffer = inst->GetByteBuffer();
        SendCommandAndReceiveResponse(pConnector, (UINT8_t*)(bByteBuffer->GetByteBuffer()), bByteBuffer->GetByteLength(),
            g_arRxBytes, sizeof(g_arRxBytes), &g_nNumRxBytes, 0, &sErr);
        // exit on error
        if(sErr != "") {
            sErr = string("sendRequest: SendCommandAndReceiveResponse failed:") + sErr;
            break;
        }

        sErr = handleReceivedData(pMsgFactoryEvt);

    }while(false);
    if(sErr != "") {
        g_oLog.WriteLog(sErr);
        // empty returned string is sign of error
        sErr = "";
        // forget unused received bytes and collection
        resetCollection();
        g_nNumRxBytes = 0;
    }
    else
        sErr = "success";   // sign of success
    return sErr;
}

/*
    if collection is not empty then gets next message from it.
    Otherwise reads till at least one full first message is received but not longer then nMaxWaitMS

    <next> := next <timeout>
    returns string with <structureName> of the next received packet
    on error returns empty string
*/
string
getNextPacket(CSerialConnector * pConnector, CDSDLFactory * pMsgFactoryEvt, UINT32_t nMaxWaitMS)
{
    string      sErr;
    UINT32_t    nNumRxBytesStart = g_nNumRxBytes;

    g_oLog.WriteLog(format("getNextPacket: nMaxWaitMS:%d", nMaxWaitMS));

    // posit to the next instance in the collection
    g_collectionIdx++;
    // if we out of instances in collection then read till at least one full first message is received but not longer then nMaxWaitMS
    if(g_collectionIdx >= g_collectionCount)
    {
        resetCollection();

        UINT32_t    nNumTries = 0;
        UINT32_t    nMaxLoopTries = nMaxWaitMS / THREAD_SLEEP_INTERVAL_MS;
        UINT32_t    tStart = GetCurrentOSTimeMS ();
        if(nMaxLoopTries < 1)
            nMaxLoopTries = 1;

        // read and handle data till at least one full first message is received but not longer then nMaxWaitMS
        while ((nNumTries < nMaxLoopTries) && (sErr == "") && (GetCurrentOSTimeMS () - tStart <= nMaxWaitMS) && (g_nNumRxBytes < sizeof(g_arRxBytes)))
        {
            nNumTries++;
            PutThreadToSleep (THREAD_SLEEP_INTERVAL_MS);
            sErr = FetchMoreReceivedBytes (pConnector, g_arRxBytes, sizeof(g_arRxBytes), &g_nNumRxBytes);
            // if we received new bytes then try to handle them
            if(nNumRxBytesStart != g_nNumRxBytes && sErr == "") {
                nNumRxBytesStart = g_nNumRxBytes;
                sErr = handleReceivedData(pMsgFactoryEvt);
                //exit loop if at least one message is received
                if(g_collectionCount > 0)
                    break;
            }
        }
        if(sErr != "") {
            g_oLog.WriteLog(sErr);
            // empty returned string is sign of error
            sErr = "";
            // forget unused received collection
            resetCollection();
            g_nNumRxBytes = 0;
        }
        // on success posit at the first instance in the collection
        else
            g_collectionIdx = 0;
    }

    if(g_collectionIdx < g_collectionCount)
    {
        CDSDLInstance *inst = g_collection->GetInstanceByIndex(g_collectionIdx);
        sErr = inst->GetTypeName();
        g_oLog.WriteLog("Next Name" + sErr + "\n" + inst->GetAllFields());
    }
    return sErr;
}

/*
    <get> := get <fieldname>
    <fieldName> - name of the any member of the current <structureName>
    <fieldValue> - value of the field <fieldName>

    returns string with <fieldValue> of the current <structureName>
    on error returns empty string
*/
string
getField(const char* fieldName)
{
    string sErr;

    g_oLog.WriteLog(string("getField:") + fieldName);

    if(g_collectionIdx >= g_collectionCount)
        g_oLog.WriteLog("getField: empty collection");
    else {
        CDSDLInstance *inst = g_collection->GetInstanceByIndex(g_collectionIdx);
        CDSDLField *field = inst->FindFieldFromName(fieldName);
        if(!field)
            g_oLog.WriteLog("getField: no such field");
        else
            sErr = field->GetValue()->AsText();
    }
    return sErr;
}

/*
    <download> := download <filePathName>       - Downloads file <filePathName> into device and returns string success or error
    <filePathName> - file path name

    returns ""success"
    on error returns empty string
*/
string
download(CSerialConnector **ppConnector, const char* filePathName, const char* port, int baudRate)
{
    string      sErr;
    FILE *      fHCD = NULL;

    g_oLog.WriteLog(format("download: baudRate:%d port: ", baudRate) + port + " file: " + filePathName);
    do
    {
        fHCD = fopen(filePathName, "rb");
        if (!fHCD) {
            g_oLog.WriteLog(string("download: fopen failed ") + filePathName);
            sErr = "error";
            break;
        }

        g_oLog.WriteLog("download: send HciReset...");
        sErr = SendHciReset(*ppConnector);
        if (sErr != "")
        {
            g_oLog.WriteLog("download: SendHciReset failed: " + sErr);
            g_oLog.WriteLog("download: set BaudRate ...");
            sErr = SetBaudRate(ppConnector, port, baudRate);
            if (sErr != "") {
                g_oLog.WriteLog("download: SetBaudRate failed " + sErr);
                break;
            }
            g_oLog.WriteLog("download: send HciReset...");
            sErr = SendHciReset(*ppConnector);
            if (sErr != "") {
                g_oLog.WriteLog("download: SendHciReset failed: " + sErr);
                break;
            }
        }

        g_oLog.WriteLog("download: send UpdateBaudRate ...");
        sErr = SendUpdateBaudRate(*ppConnector, baudRate);
        if (sErr != "") {
            g_oLog.WriteLog("download: SendUpdateBaudRate failed: " + sErr);
            break;
        }
        g_oLog.WriteLog("download: set BaudRate ...");
        sErr = SetBaudRate(ppConnector, port, baudRate);
        if (sErr != "") {
            g_oLog.WriteLog("download: SetBaudRate 2 failed " + sErr);
            break;
        }
        
        g_oLog.WriteLog("download: send DownloadMinidriver...");
        sErr = SendDownloadMinidriver(*ppConnector);
        if (sErr != "") {
            g_oLog.WriteLog("download: SendDownloadMinidriver failed " + sErr);
            break;
        }

        UINT32_t    nAddr, nHCDRecSize;
        UINT8_t     arHCDDataBuffer[256];
        bool        bIsLaunch = false;
        g_oLog.WriteLog("download: download HCD...");
        while (true)
        {
            sErr = ReadNextHCDRecord(fHCD, &nAddr, &nHCDRecSize, arHCDDataBuffer, &bIsLaunch);
            if(sErr != "") {
                g_oLog.WriteLog(format("download: ReadNextHCDRecord failed nAddr:0x%x nHCDRecSize:0x%x - ", nAddr, nHCDRecSize) + sErr);
                break;
            }
            sErr = SendHcdRecord(*ppConnector, nAddr, nHCDRecSize, arHCDDataBuffer);
            if(sErr != "") {
                g_oLog.WriteLog(format("download: SendHcdRecord failed nAddr:0x%x nHCDRecSize:0x%x - ", nAddr, nHCDRecSize) + sErr);
                break;
            }
            if (bIsLaunch)
                break;
        }
        if(sErr != "")
            break;

        g_oLog.WriteLog("download: send LaunchRam...");
        sErr = SendLaunchRam(*ppConnector);
        if (sErr != "") {
            g_oLog.WriteLog("download: SendLaunchRam failed " + sErr);
            break;
        }
        g_oLog.WriteLog("download: success");
    }while(false);
    if(fHCD)
        fclose(fHCD);
    if(sErr == "")
        sErr = "success";
    else
        sErr = "";

    return sErr;
}

#define SERVICE_ADDRESS inet_addr("127.0.0.1")
#define PORT_NUM 12012
#define SOCKET_RECEIVE_TIMEOUT 1

CSerialConnector* CreateHostModeConnector(string* err)
{
#ifdef WIN32
	CSerialConnector* connector = new CySocketSerialConnector(SERVICE_ADDRESS, PORT_NUM, SOCKET_RECEIVE_TIMEOUT);
	
	*err = connector->Connect(CCommunicationParameters(), nullptr, nullptr);
	if (*err != "")
	{
		delete connector;
		connector = nullptr;
	}

	return connector;
#else
	return nullptr;
#endif
}

#define RW_BUF_SIZE 16384
/*
App starts, opens and configures communication port and log file, parses specific sdl file.
On error exits.
Then app in the loop receives one line from stdin, handles it as request and prints one line of response to stdout.
App exits when receives command "exit"

command line:
    -PORT portid -BAUDRATE baud -MSGCMDDEFS commandsdefinitionfile -MSGEVTDEFS eventsdefinitionfile [-LOGTO logfile] [-RTS 0or1] [-CTS 0or1]
where
    MSGCMDDEFS is the file containing the definitions of various message structures of commands
    MSGEVTDEFS is the file containing the definitions of various message structures for events
    PORT is a standard platform-specific transport specifier, e.g. COM3 on Windows or tty-serxxx on another system
    BAUDRATE, just like PORT, is a platform-specific speed specification, e.g. 115200 (default value is 115200)
    BAUDRATEDWNLD, baud rate to switch to at FW download, e.g. 3000000 (default value is 3000000)
    LOGTO is an optional parameter to specify the name of the log file
    RTS is an optional RTS value passed to connection string
    CTS is an optional CTS value passed to connection string

requests:
    <request> := <send> | <next> | <get> | <download> | <exit>
    <send> := send <structureName> <fields>     - Creates packet and sends it to deviece. Returns string success or error
    <next> := next <timeout>                    - Returns string with <structureName> of the next received packet waiting for <timeout> if needed
    <get> := get <fieldname>                    - Returns string with <fieldValue> of the latest <structureName>
    <exit> := exit                              - Exits app
    <download> := download <filePathName>       - Downloads file <filePathName> into device and returns string success or error
    <structureName> - any non-abstract structure name from sdl file
    <fields> := <field> | <field>;<fields>
    <field> := <fieldname>=<fieldValue>
    <fieldName> - name of the any member of structure <structureName>
    <fieldValue> - value of the field <fieldName>
    <filePathName> - file path name
*/
int main(int argc, char* argv[])
{
    int                 res = -1;
    char                *buf;
    string              sErr;
    string              sPort, sBaudrate, sConnectionParams, sMsgDefinitionsCmd, sMsgDefinitionsEvt, sBaudrateDownload, sRTS, sCTS;
	bool				hostModeEnabled;
	CSerialConnector    *pConnector = NULL;
    CDSDLFactory        oMsgFactoryCmd;
    CDSDLFactory        oMsgFactoryEvt;
    CDSDLFactory        *pMsgFactoryEvt = NULL;
    int                 baudRateDownload = 0;

    do
    {
        if(NULL == (buf = (char*)malloc(RW_BUF_SIZE)))
        {
            fprintf (stderr, "malloc failed\n");
            break;
        }
        // parse command line
        sErr = ProcessArgsAndPrepare (argc, argv, &sPort, &sBaudrate, &sMsgDefinitionsCmd, &sMsgDefinitionsEvt, &sBaudrateDownload, &sRTS, &sCTS, &hostModeEnabled);
        if (sErr != "")
        {
            fprintf (stderr, "Error processing command-line arguments: %s\n", sErr.c_str());
            break;
        }
        //default value for sBaudrate
        if(sBaudrate == "")
            sBaudrate = "115200";

        //default value for sBaudrateDownload
        if(sBaudrateDownload == "")
            sBaudrateDownload = "3000000";
        baudRateDownload = strtol(sBaudrateDownload.c_str(), NULL, 10);

        // prepare log file
        if (g_sLogName.length())
            g_oLog.OpenLogFile (g_sLogName);
        //fprintf(stdout, "Logging goes to file: %s\n\n", g_sLogName.c_str());

        // parse messages description sdl file for commands
        sErr = oMsgFactoryCmd.Open(sMsgDefinitionsCmd.c_str(), "");
        if (sErr != "")
        {
            g_oLog.WriteLog(string("Failed to read message definitions from file ") + sMsgDefinitionsCmd + " res:" + sErr);
            break;
        }

        // if events idl file is not specified then us command idl
        if(sMsgDefinitionsEvt == "")
            pMsgFactoryEvt = &oMsgFactoryCmd;
        else {
            pMsgFactoryEvt = &oMsgFactoryEvt;
            // parse messages description sdl file for events
            sErr = oMsgFactoryEvt.Open(sMsgDefinitionsEvt.c_str(), "");
            if (sErr != "")
            {
                g_oLog.WriteLog(string("Failed to read message definitions from file ") + sMsgDefinitionsEvt + " res:" + sErr);
                break;
            }
        }

        // create string with connection parameters and connect to device
        sConnectionParams += "PORT=" + sPort + "\n BAUDRATE=" + sBaudrate + "\n";
        if(sRTS != "")
            sConnectionParams += " RTS=" + sRTS + "\n";
        if(sCTS != "")
            sConnectionParams += " CTS=" + sCTS + "\n";
		
		if (hostModeEnabled)
			pConnector = CreateHostModeConnector(&sErr);
		else
			pConnector = CreateSerialConnector(sConnectionParams.c_str(), &sErr);

        if (!pConnector)
        {
            g_oLog.WriteLog(string("Failed to open the transport connection with ") + sConnectionParams + " res:" + sErr);
            break;
        }

        // receive command from the stdin in the loop, handle it and send response to the stdout
        while (true)
        {
            // receve command from the stdin
#ifdef LINUX
            if (fgets(buf,RW_BUF_SIZE,stdin) != NULL)
                buf[strlen(buf) - 1] = '\0';
#else
			gets_s(buf, RW_BUF_SIZE);
#endif
            g_oLog.WriteLog(format("main: received command:'%s' count:%d  idx:%d bytes:%d", buf, g_collectionCount, g_collectionIdx, g_nNumRxBytes));

            // exit loop on exit command
            if (!strcmp(buf, "exit")) {
                break;
            }
            sErr = "";
            // parse and execute received command
            if(!strncmp(buf, "send ", 5)) {
                char *structureName = &buf[5];
                // skip white space
                while(*structureName != 0 && (unsigned char)*structureName <= 0x20) structureName++;
                // find end of the structure name(start of fields)
                char* fields = structureName;
                // skip till white space
                while(*fields != 0 && (unsigned char)*fields > 0x20) fields++;
                // if fields isn't empty
                if(*fields != 0) {
                    // make structure name 0-terminated and skip white space
                    *fields++ = 0;
                    while(*fields != 0 && (unsigned char)*fields <= 0x20) fields++;
                }
                sErr = sendRequest(pConnector, &oMsgFactoryCmd, pMsgFactoryEvt, structureName, fields);
            }
            else if(!strncmp(buf, "next", 4) || !strncmp(buf, "n", 1))
            {
                // get timeout from the command string
                UINT32_t nMaxWaitMS = 0;
                char *p = &buf[4];
                if(strncmp(buf, "next", 4))
                    p = &buf[1];
                // skip white space
                while(*p != 0 && (unsigned char)*p <= 0x20) p++;
                // if params is not empty then it should be timeout
                if(*p)
                {
                    char * pEnd;
                    UINT32_t n = strtoul(p, &pEnd, 10);
                    if(pEnd > p && (unsigned char)*pEnd == 0)
                        nMaxWaitMS = n;
                    p = pEnd;
                }
                // on success get next packet
                if(*p == 0)
                    sErr = getNextPacket(pConnector, pMsgFactoryEvt, nMaxWaitMS);
                else
                    g_oLog.WriteLog("main: invalid command");
            }
            else if(!strncmp(buf, "get ", 4)) {
                char *p = &buf[4];
                // skip white space
                while(*p != 0 && (unsigned char)*p <= 0x20) p++;
                if(*p)
                    sErr = getField(p);
            }
            else if(!strncmp(buf, "download ", 9)) {
                char *p = &buf[9];
                // skip white space
                while(*p != 0 && (unsigned char)*p <= 0x20) p++;
                if(*p)
                    sErr = download(&pConnector, p, sPort.c_str(), baudRateDownload);
            }
            // empty strings means error
            if(sErr == "")
                sErr = "error";
            //send response and flush buffer to make sure script receives it
            g_oLog.WriteLog(string("main: answering: ") + sErr);
            printf("%s\n", sErr.c_str());
#ifdef LINUX
	    fflush(NULL);
#else
            _flushall();
#endif
        }
        res = 0;
    }while(false);
    if(buf)
        free(buf);
    if(g_collection)
        delete g_collection;
    if(pConnector) {
        pConnector->Disconnect();
        delete pConnector;
    }
    g_oLog.WriteLog("main: exits");
    return res;
}

