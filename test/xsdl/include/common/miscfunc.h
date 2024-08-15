#ifndef MISCFUNC_H_DEFINED


#define MISCFUNC_H_DEFINED



#include <basetype.h>
#include <string>



char    HexDigit (UINT8_t byVal);
bool    ParseHexBytesFromString (const char * sHexByteSequence, UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesParsed);
bool    ParseHexBytesFromStringWithDontCares (const char * sHexByteSequence, INT16_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesParsed, char cDontCareCharacter = '*');
bool    ParseHexBytesWithoutDelimiters (const char * sHexByteSequence, UINT8_t * pDestBuf, UINT32_t nBufSize, UINT32_t * nNumBytesParsed, char cWildCardDigit = 0);
bool    DoesFileExist (const char * sFileName);
char *	ReadNextCleanLine (char * sLine, FILE * f, long * nLineNum, char COMMENT_START_MARKER);
bool    IsDoubleQuoted (const char * s);
bool    IsSingleQuoted (const char * s);
bool    IsDecInt (const char * s);
bool    IsHexInt (const char * s);
bool    TryToGetIntValue (const char * s, INT32_t * nOutVal);
bool    TryToGetUIntValue (const char * s, UINT32_t * nOutVal);
INT32_t GetIntValue (const char * s);
UINT32_t GetUIntValue (const char * s);
bool    TryToGetRealValue (const char * s, double * fOutVal);
double  GetRealValue (const char * s);
bool    IsRealNumber (const char * s);
INT32_t BytesToHexString (const UINT8_t * pBuffer, UINT32_t nBufSize, std::string & refDest, char cSeparator = ' ', bool bUse0x = false, bool bReverseBytes = false);
char *  UIntToHex (UINT32_t nVal, char * sDestBuf, UINT8_t nNumDigits = 8, bool bUse0x = false);


char *  ConcatHexBytesToString (char * sText, UINT8_t * pData, INT32_t nNumBytes, INT32_t nMaxBytesToPrint = 256);









// WAV file data structures.
//

typedef struct 
{
    char            riffID [4];
    UINT32_t        nNumBytes;
    char            waveID [4];
} TYPERIFF;




typedef struct 
{
    char            chunkID [4];
    UINT32_t        chunkSize;

    UINT16_t        wFormatTag;
    UINT16_t        wChannels;
    UINT32_t        dwSamplesPerSec;
    UINT32_t        dwAvgBytesPerSec;
    UINT16_t        wBlockAlign;
    UINT16_t        wBitsPerSample;
} TYPEFORMATCHUNK;

typedef struct 
{
    char            chunkID [4];
    UINT32_t        chunkSize;

    //unsigned char   waveformData[];
} TYPEDATACHUNK;

typedef struct 
{
    TYPERIFF        riff;
    TYPEFORMATCHUNK formatchunk;
    TYPEDATACHUNK   datachunk;
} WAVHEADER;


//
// END OF WAV file data structures.





// The reason we want to have these functions is because the WAV structures keep integers
// in little-endian form. So if we want to execute our code on a big-endian platform, we
// cannot just fread/fwrite, but need to use conversions.
// Will return an integer number of bytes read or written if successful, or -1 otherwise.

INT32_t     ReadWAVHeaderFromBuffer (UINT8_t * pSrcDataBuffer, WAVHEADER * pHeader);
INT32_t     WriteWAVHeaderToBuffer (WAVHEADER * pHeader, UINT8_t * pDestDataBuffer);


INT32_t     ReadAudioSamples (FILE * fWAVFile, WAVHEADER * pHeader, UINT16_t nChannelIndexZeroBased, UINT32_t nStartingSampleIndexZeroBased, void * pSamplesBuffer, UINT32_t nNumSamplesToRead);











#endif
