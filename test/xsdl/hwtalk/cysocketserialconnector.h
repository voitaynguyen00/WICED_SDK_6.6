#ifndef CY_SOCKET_SERIAL_CONNECTOR
#define CY_SOCKET_SERIAL_CONNECTOR

#include "trancommon.h"

#include <WinSock2.h>
#include <sstream>
#include <string>

#define UNUSED(x) (void)x

template <typename T>
inline string toString(T value)
{
	//create an output string stream
	ostringstream os;

	//throw the value into the string stream
	os << value;

	//convert the string stream into a string and return
	return os.str();
}

class CySocketSerialConnector : public CSerialConnector
{

public:
	CySocketSerialConnector(ULONG addr, USHORT port, int recvTimeoutInMSec = 0);
	virtual ~CySocketSerialConnector();

public:
	virtual string Connect(CCommunicationParameters& refParams, FUNC_PROCESS_RECEIVED_BYTES pFunc, void * pSomeDataForFunc) override;
	virtual string Disconnect() override;
	virtual string TransmitBytes(UINT8_t * pByteBuffer, UINT32_t nNumBytes) override;
	virtual string ReadReceivedBytes(UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesReturned, bool bPeekOnly = false) override;

private:
	string openSocket();
	string initWinsock();
	bool isOpen() const;

private:
	SOCKET m_socket;
	SOCKADDR_IN m_service;
	FUNC_PROCESS_RECEIVED_BYTES m_readCb;
	void* m_tagData;
	int m_recvTimeoutInMSec;
};

#endif