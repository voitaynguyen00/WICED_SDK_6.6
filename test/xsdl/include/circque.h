#ifndef CIRCQUE_H_DEFINED

#define     CIRCQUE_H_DEFINED


#include "basetype.h"

#define		DEFAULT_MAX_DATA_QUEUE_SIZE	    0x100000

#define     DEFAULT_LINEAR_BUFFER_SIZE      (DEFAULT_MAX_DATA_QUEUE_SIZE/4)

class DATA_QUEUE
{

public:

    DATA_QUEUE (UINT32_t nQueueByteSize = DEFAULT_MAX_DATA_QUEUE_SIZE, UINT32_t nLinearBufSize = DEFAULT_LINEAR_BUFFER_SIZE);

    ~DATA_QUEUE ();

    UINT32_t            m_nQueueByteSize;
    UINT32_t            m_nLinearBufferByteSize;


	UINT8_t *           m_pqueData;
	UINT32_t	        m_nTotalBytesStored;	/* from the time it started filling up */
	UINT32_t	        m_nTotalBytesRead;	    /* <= m_nTotalBytesStored */

	UINT8_t *           m_pNextByteToStore;		/* points within m_queData */
	UINT8_t *           m_pNextByteToRead;	    /* points within m_queData */

	UINT8_t *           m_pFirstArrayByte;		/* points to m_queData[0] */
	UINT8_t *           m_pLastArrayByte;		/* points to m_queData[MAX_DATA_QUEUE_SIZE-1] */

    /* In rare cases when data is wrapped, utilize this linear array to return the bytes */
    UINT8_t *           m_pLinearBuffer;



};




UINT32_t	DataQueueCountTotalBytesStored (DATA_QUEUE* pQueue);
UINT32_t	DataQueueCountTotalBytesRead (DATA_QUEUE* pQueue);
UINT32_t	DataQueueCountBytesRemainingToRead (DATA_QUEUE* pQueue);
UINT32_t	DataQueueReadNextBytes(DATA_QUEUE* pQueue, UINT32_t nNumToReadOrZeroForAll, UINT8_t ** pOutBytePointer);
UINT32_t	DataQueuePeekNextBytes(DATA_QUEUE* pQueue, UINT32_t nNumToReadOrZeroForAll, UINT8_t ** pOutBytePointer); /* Leaves queue pointers unchanged */
UINT32_t	DataQueueAddBytes(DATA_QUEUE* pQueue, UINT32_t nNumToAdd, UINT8_t * pSrc);
UINT32_t    DataQueueSkipNextBytes(DATA_QUEUE* pQueue, UINT32_t nNumToSkipOrZeroForAll);




class CppQueueWrapper
{
public:

    CppQueueWrapper (DATA_QUEUE& refQueue)
    {
        m_pQueue = &refQueue;
    }

    CppQueueWrapper (const CppQueueWrapper & cp)
    {
        m_pQueue = cp.m_pQueue;
    }


    CppQueueWrapper& operator= (const CppQueueWrapper & cp)
    {
        m_pQueue = cp.m_pQueue;
        return (*this);
    }

    ~CppQueueWrapper ()
    {
    }


    UINT32_t	CountTotalBytesStored ()
    {
        return DataQueueCountTotalBytesStored (m_pQueue);
    }

    UINT32_t	CountTotalBytesRead ()
    {
        return DataQueueCountTotalBytesRead (m_pQueue);
    }

    UINT32_t	CountBytesRemainingToRead ()
    {
        return DataQueueCountBytesRemainingToRead (m_pQueue);
    }


    UINT32_t	ReadNextBytes (UINT32_t nNumToReadOrZeroForAll, UINT8_t ** pOutBytePointer)
    {
        return DataQueueReadNextBytes (m_pQueue, nNumToReadOrZeroForAll, pOutBytePointer);
    }

    UINT32_t	PeekNextBytes (UINT32_t nNumToReadOrZeroForAll, UINT8_t ** pOutBytePointer)
    {
        return DataQueuePeekNextBytes (m_pQueue, nNumToReadOrZeroForAll, pOutBytePointer);
    }

    UINT32_t	AddBytes (UINT32_t nNumToAdd, UINT8_t * pSrc)
    {
        return DataQueueAddBytes (m_pQueue, nNumToAdd, pSrc);
    }


    UINT32_t	SkipNextBytes (UINT32_t nNumToSkipOrZeroForAll)
    {
        return DataQueueSkipNextBytes (m_pQueue, nNumToSkipOrZeroForAll);
    }


protected:

    DATA_QUEUE*     m_pQueue;
};




#endif

