#include "cysocketserialconnector.h"

CySocketSerialConnector::CySocketSerialConnector(ULONG addr, USHORT port, int recvTimeoutInMSec)
{
	m_recvTimeoutInMSec = recvTimeoutInMSec;

	m_service.sin_family = AF_INET;
	m_service.sin_addr.s_addr = addr;
	m_service.sin_port = htons(port);

	m_socket = INVALID_SOCKET;
}

CySocketSerialConnector::~CySocketSerialConnector()
{
}

string CySocketSerialConnector::Connect(
	CCommunicationParameters& refParams, 
	FUNC_PROCESS_RECEIVED_BYTES pFunc, 
	void* pSomeDataForFunc)
{
	UNUSED(refParams);
	m_readCb = pFunc;
	m_tagData = pSomeDataForFunc;

	return openSocket();
}

string CySocketSerialConnector::Disconnect()
{
	string err;
	if (isOpen())
	{
		int result =::closesocket(m_socket);
		if (result == SOCKET_ERROR)
			err = string("Failed to close socket with error code: ") + toString((int)WSAGetLastError());

		m_socket = INVALID_SOCKET;
	}

	return err;
}

string CySocketSerialConnector::TransmitBytes(UINT8_t* pByteBuffer, UINT32_t nNumBytes)
{
	string err;
	if (!isOpen())
	{
		err = openSocket();
		if (err != "")
			return err;
	}

	int result = send(m_socket, (char*)pByteBuffer, nNumBytes, 0);
	if (SOCKET_ERROR == result)
	{
		err = string("Failed to transmit data with error code: ") + toString((int)WSAGetLastError());
		return err;
	}

	return err;
}

string CySocketSerialConnector::ReadReceivedBytes(
	UINT8_t * pDestBuf,
	UINT32_t nBufSize,
	UINT32_t * nNumBytesReturned, bool bPeekOnly)
{
	string err;
	if (!isOpen())
	{
		err = openSocket();
		if (err != "")
			return err;
	}

	int result = recv(m_socket, (char*)pDestBuf, nBufSize, 0);
	if (SOCKET_ERROR == result)
	{
		err = string("Failed to read from socket with error code: ") + toString(WSAGetLastError());
		return err;
	}

	if (nNumBytesReturned != nullptr)
		*nNumBytesReturned = result;

	return err;
}

string CySocketSerialConnector::openSocket()
{
	string err = initWinsock();
	if (err != "")
		return err;

	if (INVALID_SOCKET == (m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)))
	{
		err = string("Failed to create socket with error code: ") + toString(WSAGetLastError());
		return err;
	}

	if (SOCKET_ERROR == connect(m_socket, (const sockaddr*)&m_service, sizeof(m_service)))
	{
		err = string("Failed to open socket with error code: ") + toString(WSAGetLastError());
		Disconnect();
		return err;
	}

	if (m_recvTimeoutInMSec > 0)
	{
		if (SOCKET_ERROR == 
			setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (char*)&m_recvTimeoutInMSec, sizeof(m_recvTimeoutInMSec)))
		{
			err = string("Failed to set receive timeout with error code: ") + toString(WSAGetLastError());
			Disconnect();
			return err;
		}
	}

	return err;
}

string CySocketSerialConnector::initWinsock()
{
	string err;

#ifdef WIN32
	WSADATA wsaData;
	int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (result != 0)
		err = string("Failed to initialize socket. Error code: ") + toString(result);
#endif

	return err;
}

bool CySocketSerialConnector::isOpen() const
{
	return m_socket != INVALID_SOCKET;
}
