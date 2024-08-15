#ifndef CONTROL_COMM_H
#define CONTROL_COMM_H

//**************************************************************************************************
//*** Definitions for BTW Serial Bus
//**************************************************************************************************

// Helper class to print debug messages to Debug Console
class DebugHelper
{
public:
	void DebugOut( ) { OutputDebugStringA( m_Buffer ); }
	void DebugOut( LPCSTR v ) { OutputDebugStringA( v ); }
	void DebugOut( LPCSTR fmt, LPCSTR v );
	void DebugOut( LPCSTR fmt, DWORD v1 );
	void DebugOut( LPCSTR fmt, DWORD v1, DWORD v2 );
	void PrintCommProp( COMMPROP& commProp );
	void PrintCommState( DCB& serial_config );

	static char m_Buffer[1024];
};

//
// Serial Bus class, use this class to read/write from/to the serial bus device
//
class ComHelper
{
public:
    ComHelper(HWND hWnd);
    virtual ~ComHelper( );

	// oopen serialbus driver to access device
    BOOL OpenPort( int port, int baudRate );
    void ClosePort( );

	// read data from device
    DWORD Read( LPBYTE b, DWORD dwLen );
    DWORD ReadNewHciPacket( BYTE * pu8Buffer, int bufLen, int * pOffset );
    DWORD ReadWorker( );

	// write data to device
    DWORD SendWicedCommand( UINT16 command, LPBYTE b, DWORD dwLen );
    DWORD Write( LPBYTE b, DWORD dwLen );

    BOOL IsOpened( );

private:
    HWND m_hWnd;
	// overlap IO for Read and Write
    OVERLAPPED m_OverlapRead;
    OVERLAPPED m_OverlapWrite;
    HANDLE m_handle;
    HANDLE m_hThreadRead;
    HANDLE m_hShutdown;
    BOOL m_bClosing;
    BOOL m_CleanHciState;
};



extern void DumpData( char *description, void* p, UINT32 length, UINT32 max_lines );

#endif
