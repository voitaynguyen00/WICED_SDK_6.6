#ifndef UARTTRANDATA_H_DEFINED
#define UARTTRANDATA_H_DEFINED

#include <termios.h>
#include "trancommon.h"
#include "osobjects.h"

#define FALSE   0
#define TRUE    1





class CUARTTransportData : public CTransportData
{
public:
	CUARTTransportData();
	virtual ~CUARTTransportData();

  
	TRANSPORT_STATUS    WriteBytesForTransmission (UINT8_t * pSrcBuf, UINT32_t nNumBytes);


  
	void	OutputDebugString (const char* sText);

	INT32_t            m_nFileDescriptor;
	bool			   m_bStopThread;
	struct termios     m_oOldTIO;      //place for old and new port settings for serial port



	CSynchronizationMutex*  m_pThreadStoppedMutex;
        CSynchronizationMutex*  m_pDataMutex;

protected:

};



#endif /* UARTTRANDATA_H_DEFINED */
