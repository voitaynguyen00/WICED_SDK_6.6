#ifndef BASETYPE_H_DEFINED


#define BASETYPE_H_DEFINED



#include "platform.h"



#if !defined(PROCESSOR_IS_LITTLE_ENDIAN) && !defined(PROCESSOR_IS_BIG_ENDIAN)
#error Processor endianness must be defined by defining one of the compile flags: PROCESSOR_IS_LITTLE_ENDIAN or PROCESSOR_IS_BIG_ENDIAN
#endif

#if defined(PROCESSOR_IS_LITTLE_ENDIAN) && defined(PROCESSOR_IS_BIG_ENDIAN)
#error Flags PROCESSOR_IS_LITTLE_ENDIAN and PROCESSOR_IS_BIG_ENDIAN are mutually exclusive -- you cannot define both!
#endif


#include <stdio.h>
#include <string.h>









#define     BDADDR_LENGTH       6

typedef struct
{
    UINT8_t     m_arAddressBytes [BDADDR_LENGTH];
} BDADDR;



// Convenient type to keep track of boolean options that may have a third state (UNSPECIFIED)
enum BooleanOptions
{
    BooleanOptionNotSpecified = -1,
    BooleanOptionFalse = 0,
    BooleanOptionTrue = 1
};






#define BUFFER_MEMMGT_MODE_UNDEFINED                    0
#define BUFFER_MEMMGT_MODE_COPY                         1
#define BUFFER_MEMMGT_MODE_USE_EXISTING                 2
#define BUFFER_MEMMGT_MODE_USE_EXISTING_AND_FREE        3

class CByteBuffer
{
public:
    CByteBuffer (UINT8_t * pData = NULL, UINT32_t nNumBytes = 0, UINT8_t byMemMgt = BUFFER_MEMMGT_MODE_COPY);

    CByteBuffer (UINT32_t nNumBytes, UINT8_t byFiller = 0);

    ~CByteBuffer ();

    CByteBuffer (const CByteBuffer& cp);

    CByteBuffer& operator= (const CByteBuffer& cp) ;

    void    AssignData (UINT8_t * pData, UINT32_t nNumBytes, UINT8_t byMemMgt = BUFFER_MEMMGT_MODE_COPY);
    void    AppendData (UINT8_t * pData, UINT32_t nNumBytes);

    void SetBaseAddress (UINT32_t addr)
    {
        m_nBaseAddress = addr;
    }

    UINT32_t GetBaseAddress () const
    {
        return m_nBaseAddress;
    }

    UINT32_t GetBaseAddress ()
    {
        return m_nBaseAddress;
    }

    UINT8_t*    GetDataBuffer ()
    {
        return m_pBytes;
    }
    

    UINT32_t    GetDataByteSize ()
    {
        return m_nNumBytes;
    }

    UINT8_t     GetMemoryManagementMode ()
    {
        return m_byMemMgtMode;
    }

    UINT8_t     GetFlag ()
    {
        return m_byFlag;
    }

    void        SetFlag (UINT8_t byFlag)
    {
        m_byFlag = byFlag;
    }

	void SetName(const char* name);

	const char* GetName() const
	{
		return m_sName;
	}


    void        Clear ();


protected:

    UINT8_t*    m_pBytes;
    UINT32_t    m_nNumBytes;
    UINT8_t     m_byMemMgtMode;


    UINT32_t    m_nBaseAddress; // Can be utilized to associate some starting address with this sequence of bytes.

    UINT8_t     m_byFlag;       // Can be used for indicating some information.
	char*		m_sName;
};





class CBitSequence
{

public:

    // nBitIndex 0 is bit 7 (msb) of the first byte
    // nBitIndex 1 is bit 6 of the first byte
    //      .
    //      .
    // nBitIndex 7 is bit 0 (lsb) of the first byte
    // nBitIndex 8 is bit 7 (msb) of the second byte
    // nBitIndex 9 is bit 6 of the second byte
    //      .
    //      .
    // nBitIndex 15 is bit 0 (lsb) of the second byte
    // nBitIndex 16 is bit 7 (msb) of the third byte
    // nBitIndex 17 is bit 6 of the third byte
    //      .
    //      .
    //      .
    //      .

    CBitSequence (UINT32_t nNumBits = 0) : m_oBytes ((nNumBits + 7) >> 3, 0)
    {
        m_nNumBits = nNumBits;
    }

    CBitSequence (const CBitSequence& cp) : m_oBytes (cp.m_oBytes)
    {
        m_nNumBits = cp.m_nNumBits;
    }

    CBitSequence& operator= (const CBitSequence& cp) 
    {
        m_oBytes = cp.m_oBytes;
        m_nNumBits = cp.m_nNumBits;
        return (*this);
    }

    ~CBitSequence ()
    {
    }


    UINT32_t    CountBits ()
    {
        return m_nNumBits;
    }

    INT16_t     GetBitValue (UINT32_t nBitIndex)
    {
        if ((nBitIndex < 0) || (nBitIndex >= m_nNumBits))
            return -1;


        UINT8_t*    pByte = m_oBytes.GetDataBuffer () + (nBitIndex >> 3);

        return *pByte & (0x01 << (7 - (nBitIndex & 7)));
    }

    bool        SetBitValue (UINT32_t nBitIndex, UINT8_t byValue)
    {
        if ((nBitIndex < 0) || (nBitIndex >= m_nNumBits) || (byValue > 1))
            return false;


        UINT8_t*    pByte = m_oBytes.GetDataBuffer () + (nBitIndex >> 3);

        if (byValue)
            *pByte |= (0x01 << (7 - (nBitIndex & 7)));
        else
            *pByte &= ~(0x01 << (7 - (nBitIndex & 7)));

        return true;
    }


    char *      ExtractBitsAsString (UINT32_t nLeftBitIndex, UINT32_t nNumBitsToExtract, char * sLongEnoughDestBuf)
    {
        if (!sLongEnoughDestBuf || (nLeftBitIndex < 0) || (nNumBitsToExtract == 0) || (nLeftBitIndex + nNumBitsToExtract- 1 >= m_nNumBits))
            return NULL;

        *sLongEnoughDestBuf = '\0';

        for (UINT32_t i = 0; i < nNumBitsToExtract; i++)
        {
            INT16_t nBitVal = GetBitValue (nLeftBitIndex + i);

            if (nBitVal < 0)
                return NULL;
            if (nBitVal == 0)
                strcat (sLongEnoughDestBuf, "0");
            else
                strcat (sLongEnoughDestBuf, "1");
        }
        return sLongEnoughDestBuf;
    }

    char *      GetAllBitsAsString (char * sLongEnoughDestBuf)
    {
        return ExtractBitsAsString (0, m_nNumBits, sLongEnoughDestBuf);
    }


    bool SetBitsFromString (UINT32_t nLeftBitIndex,  const char * sBits)
    {
        UINT16_t    nNumBits = (UINT16_t)strlen(sBits);

        if ((nLeftBitIndex < 0) || (nNumBits == 0) || (nLeftBitIndex + nNumBits - 1 >= m_nNumBits))
            return false;


        for (UINT16_t i = 0; i < nNumBits; i++)
        {
            char    cBit = sBits [i];

            if ((cBit != '0') && (cBit != '1'))
                return false;
            if (!SetBitValue (nLeftBitIndex + i, cBit - '0'))
                return false;
        }
        return true;
    }

    UINT32_t ExtractBitsAsUINT32 (UINT32_t nLeftBitIndex, UINT8_t nNumBitsToExtract)
    {

        // Interpret the value of the bits as an unsigned integer when read from left to right.
        // For example, bits 1000000111 will return as value 519, regardless of the platform.
        // On the big-endian platform the return value bytes will have bits in the same order,
        // i.e. 00000000 00000000 00000010 00000111, but on the little-endian platform, the
        // return value bytes will be reversed -- 00000111 00000010 00000000 00000000

        if (nNumBitsToExtract > 32)
            return 0;


        UINT32_t nSum = 0;


        for (UINT32_t i = 0; i < nNumBitsToExtract; i++)
        {
            nSum = (nSum << 1);

            INT16_t nBitVal = GetBitValue (nLeftBitIndex + i);

            if (nBitVal < 0)
                return 0;
            if (nBitVal == 1)
                nSum++;
        }
        return nSum;
    }





    bool SetBitsFromUINT32 (UINT32_t nValueToSet, UINT32_t nLeftBitIndex, UINT8_t nNumBits)
    {
        if ((nLeftBitIndex < 0) || (nNumBits > 32) || (nNumBits == 0) || (nLeftBitIndex + nNumBits - 1 >= m_nNumBits))
            return false;


        // Start from lsb of nValueToSet -- it will land at the rightmost end of the
        // bit sequence buffer (at position nLeftBitIndex+nNumBits-1)

        for (UINT32_t i = 0; i < nNumBits; i++)
        {
            UINT8_t nBitVal = (UINT8_t)(nValueToSet & 0x01);


            if (!SetBitValue (nLeftBitIndex + nNumBits - 1 - i, nBitVal))
                return false;

            nValueToSet = nValueToSet >> 1;
        }
        return true;
    }



protected:

    CByteBuffer m_oBytes;
    UINT32_t    m_nNumBits;
};






/* 
    For the sake of portable buffer storage, all multi-byte (2,4,8) integers will be stored in
    the big-endian arrangement -- that's the same as the network format defined by TCP.
    So, we will utilize these specific functions for writing/reading integers to/from byte buffers.
    These functions will be implemented differently for the little-endian and the big-endian platforms.
 */


/* This set of functions is for single integer copying. */
/* For CopyXXXXXToByteBuffer, the return value is the number of bytes copied. */
/* For CopyByteBufferToXXXXX, the return value is the value itself. */
UINT8_t     CopyInt16ToByteBuffer (INT16_t nVal, UINT8_t * pDest);
INT16_t     CopyByteBufferToInt16 (UINT8_t * pSrc);
UINT8_t     CopyUint16ToByteBuffer (UINT16_t nVal, UINT8_t * pDest);
UINT16_t    CopyByteBufferToUint16 (UINT8_t * pSrc);

UINT8_t     CopyInt32ToByteBuffer (INT32_t nVal, UINT8_t * pDest);
INT32_t     CopyByteBufferToInt32 (UINT8_t * pSrc);
UINT8_t     CopyUint32ToByteBuffer (UINT32_t nVal, UINT8_t * pDest);
UINT32_t    CopyByteBufferToUint32 (UINT8_t * pSrc);

UINT8_t     CopyInt64ToByteBuffer (INT64_t nVal, UINT8_t * pDest);
INT64_t     CopyByteBufferToInt64 (UINT8_t * pSrc);
UINT8_t     CopyUint64ToByteBuffer (UINT64_t nVal, UINT8_t * pDest);
UINT64_t    CopyByteBufferToUint64 (UINT8_t * pSrc);



/* This set of functions is for integer array copying. */
/* For all functions, the return value is the total number of bytes copied. */
UINT32_t    CopyInt16ArrayToByteBuffer (INT16_t * pSrc, UINT32_t nNumElems, UINT8_t * pDest);
UINT32_t    CopyByteBufferToInt16Array (UINT8_t * pSrc, INT16_t * pDest, UINT32_t nNumElems);
UINT32_t    CopyUInt16ArrayToByteBuffer (UINT16_t * pSrc, UINT32_t nNumElems, UINT8_t * pDest);
UINT32_t    CopyByteBufferToUInt16Array (UINT8_t * pSrc, UINT16_t * pDest, UINT32_t nNumElems);

UINT32_t    CopyInt32ArrayToByteBuffer (INT32_t * pSrc, UINT32_t nNumElems, UINT8_t * pDest);
UINT32_t    CopyByteBufferToInt32Array (UINT8_t * pSrc, INT32_t * pDest, UINT32_t nNumElems);
UINT32_t    CopyUInt32ArrayToByteBuffer (UINT32_t * pSrc, UINT32_t nNumElems, UINT8_t * pDest);
UINT32_t    CopyByteBufferToUInt32Array (UINT8_t * pSrc, UINT32_t * pDest, UINT32_t nNumElems);

UINT32_t    CopyInt64ArrayToByteBuffer (INT64_t * pSrc, UINT32_t nNumElems, UINT8_t * pDest);
UINT32_t    CopyByteBufferToInt64Array (UINT8_t * pSrc, INT64_t * pDest, UINT32_t nNumElems);
UINT32_t    CopyUInt64ArrayToByteBuffer (UINT64_t * pSrc, UINT32_t nNumElems, UINT8_t * pDest);
UINT32_t    CopyByteBufferToUInt64Array (UINT8_t * pSrc, UINT64_t * pDest, UINT32_t nNumElems);







UINT8_t     CopyUint32ToLittleEndianBuffer (UINT32_t nVal, UINT8_t * pDest);
UINT8_t     CopyUint16ToLittleEndianBuffer (UINT16_t nVal, UINT8_t * pDest);


UINT32_t    CopyLittleEndianBufferToUint32(UINT8_t * pSrc);
UINT16_t    CopyLittleEndianBufferToUint16(UINT8_t * pSrc);




// Variable length unsigned integer (LEB128) format:
//   the value is stored 7 bits at a time, starting with the lowest order bits.
//   The msb in each byte is a continuation flag: 1 means more bytes, 0 means end,
// e.g. all values 0..127 are stored as one byte (same value).
// Value 128 is stored in 2 bytes as 0x80 0x01
// Value 129 is stored in 2 bytes as 0x81 0x01
// Value 0x7777 is stored in 3 bytes as 0xF7 0xEE 0x01
//  etc.


UINT32_t    CopyByteBufferLEB128ToUint32 (UINT8_t * pSrc, UINT8_t * nNumBytesUsed = NULL);
// Function return value is the value of the variable length integer that is scanned.
// If nNumBytesUsed parameter is provided, it will return the number of bytes taken by this integer.

UINT8_t     CopyUint32ToByteBufferLEB128 (UINT32_t nVal, UINT8_t * pDest);
// The return value here is the numbetr of bytes the integer value nVal has taken in the buffer.



typedef enum VarTypes
{
	VARTYPE_UNDEFINED = 0,
	VARTYPE_U8,
	VARTYPE_I8,
	VARTYPE_U16,
	VARTYPE_I16,
	VARTYPE_U32,
	VARTYPE_I32,
	VARTYPE_U64,
	VARTYPE_I64,
	VARTYPE_F32,
	VARTYPE_F64,
	VARTYPE_BOOL,
	VARTYPE_STRING,
    VARTYPE_DATETIME,
    END_OF_VALID_TYPES
} VARDATATYPE;



typedef struct dth
{
    UINT16_t    m_YYYY;
    UINT8_t     m_MM;
    UINT8_t     m_DD;
    UINT8_t     m_hh;
    UINT8_t     m_mm;
    UINT8_t     m_ss;
    UINT16_t    m_ms;
} DATETIMEHOLDER;


class CDateTimeHolder
{
public:
    
    DATETIMEHOLDER  m_dt;


	CDateTimeHolder ()
	{
		m_dt.m_YYYY = 0;
		m_dt.m_MM = 0;
		m_dt.m_DD = 0;
		m_dt.m_hh = 0;
		m_dt.m_mm = 0;
		m_dt.m_ss = 0;
        m_dt.m_ms = 0;
	}

	CDateTimeHolder (DATETIMEHOLDER dt)
	{
		m_dt = dt;
	}

	CDateTimeHolder (UINT16_t nYear, UINT16_t nMonth, UINT16_t nDate, UINT16_t nHours, UINT16_t nMin, UINT16_t nSec, UINT16_t nMilliSec)
	{
		m_dt.m_YYYY = nYear;
		m_dt.m_MM = (UINT8_t) nMonth;
		m_dt.m_DD = (UINT8_t) nDate;
		m_dt.m_hh = (UINT8_t) nHours;
		m_dt.m_mm = (UINT8_t) nMin;
		m_dt.m_ss = (UINT8_t) nSec;
        m_dt.m_ms = nMilliSec;
	}

	CDateTimeHolder (UINT16_t nHours, UINT16_t nMin, UINT16_t nSec, UINT16_t nMilliSec)
	{
		m_dt.m_YYYY = 0;
		m_dt.m_MM = 0;
		m_dt.m_DD = 0;
		m_dt.m_hh = (UINT8_t) nHours;
		m_dt.m_mm = (UINT8_t) nMin;
		m_dt.m_ss = (UINT8_t) nSec;
        m_dt.m_ms = nMilliSec;
	}


	CDateTimeHolder (const CDateTimeHolder& cp)
    {
		m_dt.m_YYYY = cp.m_dt.m_YYYY;
		m_dt.m_MM = cp.m_dt.m_MM;
		m_dt.m_DD = cp.m_dt.m_DD;
		m_dt.m_hh = cp.m_dt.m_hh;
		m_dt.m_mm = cp.m_dt.m_mm;
		m_dt.m_ss = cp.m_dt.m_ss;
        m_dt.m_ms = cp.m_dt.m_ms;
    }

	CDateTimeHolder (const char * sDateTimeAsText);


	CDateTimeHolder& operator= (const CDateTimeHolder& cp)
    {
		m_dt.m_YYYY = cp.m_dt.m_YYYY;
		m_dt.m_MM = cp.m_dt.m_MM;
		m_dt.m_DD = cp.m_dt.m_DD;
		m_dt.m_hh = cp.m_dt.m_hh;
		m_dt.m_mm = cp.m_dt.m_mm;
		m_dt.m_ss = cp.m_dt.m_ss;
        m_dt.m_ms = cp.m_dt.m_ms;
        return (*this);
    }


    char *  AsText (char * sDestBuf);

    void    Now (void (*fnGetTime) (DATETIMEHOLDER*))
    {
        DATETIMEHOLDER  dt;

        (*fnGetTime) (&dt);
		m_dt.m_YYYY = dt.m_YYYY;
		m_dt.m_MM = dt.m_MM;
		m_dt.m_DD = dt.m_DD;
		m_dt.m_hh = dt.m_hh;
		m_dt.m_mm = dt.m_mm;
		m_dt.m_ss = dt.m_ss;
        m_dt.m_ms = dt.m_ms;
    }


    void Clear ()
    {
		m_dt.m_YYYY = 0;
		m_dt.m_MM = 0;
		m_dt.m_DD = 0;
		m_dt.m_hh = 0;
		m_dt.m_mm = 0;
		m_dt.m_ss = 0;
        m_dt.m_ms = 0;
    }

    bool IsEmpty ()
    {
        return (!m_dt.m_YYYY && !m_dt.m_MM && !m_dt.m_DD && !m_dt.m_hh
                && !m_dt.m_mm && !m_dt.m_ss && !m_dt.m_ms);
    }

};




class CVariableValue
{
public:


	CVariableValue ();

    bool AssignName (const char * sName);

    void ClearValue ();


	~CVariableValue ();



	CVariableValue (const CVariableValue& cp);


	CVariableValue& operator= (const CVariableValue& cp);


	CVariableValue (UINT8_t nVal);

	CVariableValue (INT8_t nVal);

	CVariableValue (UINT16_t nVal);

	CVariableValue (INT16_t nVal);



	CVariableValue (UINT32_t nVal);

	CVariableValue (INT32_t nVal);



	CVariableValue (UINT64_t nVal);

	CVariableValue (INT64_t nVal);

	CVariableValue (bool bVal);


	CVariableValue (const char * sVal);


	CVariableValue (CDateTimeHolder dt);



	CVariableValue& operator= (UINT8_t nVal);

	CVariableValue& operator= (INT8_t nVal);

	CVariableValue& operator= (UINT16_t nVal);

	CVariableValue& operator= (INT16_t nVal);

	CVariableValue& operator= (UINT32_t nVal);


	CVariableValue& operator= (INT32_t nVal);

	CVariableValue& operator= (UINT64_t nVal);

	CVariableValue& operator= (INT64_t nVal);


	CVariableValue& operator= (float rVal);





	CVariableValue& operator= (double rVal);

	CVariableValue& operator= (bool bVal);

	CVariableValue& operator= (const char * sText);


	CVariableValue& operator= (CDateTimeHolder dt);




	const char * GetName ()
	{
		return m_sName;
	}

	UINT8_t GetType ()
	{
		return (UINT8_t)m_nType;
	}

	bool IsInteger ()
	{
		return (m_nType == VARTYPE_U8) || (m_nType == VARTYPE_I8) || 
               (m_nType == VARTYPE_U16) || (m_nType == VARTYPE_I16) || 
               (m_nType == VARTYPE_U32) || (m_nType == VARTYPE_I32) || 
               (m_nType == VARTYPE_U64) || (m_nType == VARTYPE_I64);
	}


	bool IsRealNumber ()
	{
		return (m_nType == VARTYPE_F32) || (m_nType == VARTYPE_F64);
	}

	bool IsSigned ()
	{
		return (m_nType == VARTYPE_I8) || 
               (m_nType == VARTYPE_I16) || 
               (m_nType == VARTYPE_I32) || 
               (m_nType == VARTYPE_I64) || IsRealNumber ();
	}

    bool IsDateTime ()
    {
		return (m_nType == VARTYPE_DATETIME);
    }


	bool IsNumeric ()
	{
		return (IsRealNumber () || IsInteger ());
	}

	bool IsString ()
	{
		return (m_nType == VARTYPE_STRING);
	}

	bool IsBoolean ()
	{
		return (m_nType == VARTYPE_BOOL);
	}

    DATETIMEHOLDER * GetDateTime ()
    {
        if (IsDateTime ())
            return &m_Value.m_dt;
        else
            return NULL;
    }

	bool GetBooleanValue ()
	{
		if (m_nType == VARTYPE_BOOL)
			return (m_Value.m_bValue);
		else
			return false;
	}

	double GetRealNumberValue ()
	{
		if (m_nType == VARTYPE_F32)
			return (double) (m_Value.m_fValueF32);
		else if (m_nType == VARTYPE_F64)
			return (m_Value.m_fValueF64);
		else
			return 0.0;
	}


	INT32_t GetInt32Value ();

	UINT32_t GetUInt32Value ();





	INT64_t GetInt64Value ();

	UINT64_t GetUInt64Value ();




	const char * GetTextValue ()
	{
		if (m_nType == VARTYPE_STRING)
			return (m_Value.m_sTextDataValue);
		else
			return NULL;
	}


	bool operator== (const CVariableValue& op2);



    char * GetValueAsString (char * sDestBuf, UINT16_t nMaxChars);

    bool ScanValueFromString (const char * sVal);


    char * PackIntoString (char * pLargeEnoughTextBuffer);


    bool UnpackFromString (char * pPacked);

    char * Description (const char * sTitle, char * sBuf);

protected:


	char *      m_sName;
	
	VARDATATYPE m_nType;
		

	union uu
	{
		UINT8_t         m_byValueU8;
		INT8_t          m_byValueI8;
		UINT16_t        m_nValueU16;
		INT16_t         m_nValueI16;
		UINT32_t        m_nValueU32;
		INT32_t         m_nValueI32;
		UINT64_t        m_nValueU64;
		INT64_t         m_nValueI64;
		float           m_fValueF32;
		double		    m_fValueF64;
		bool		    m_bValue;
    	char *          m_sTextDataValue;
        DATETIMEHOLDER  m_dt;
	} m_Value;






    bool IsValidName (const char * sName);

};




#endif

