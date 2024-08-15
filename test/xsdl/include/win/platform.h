#ifndef PLATFORM_H_DEFINED

#ifdef _MSC_VER
	#pragma warning(disable: 4786)	// 4786:
#endif


#define PLATFORM_H_DEFINED

#define PROCESSOR_IS_LITTLE_ENDIAN


typedef unsigned    char            UINT8_t;
typedef             char            INT8_t;
typedef unsigned    short           UINT16_t;
typedef             short           INT16_t;
typedef unsigned    long            UINT32_t;
typedef             long            INT32_t;
typedef unsigned    __int64         UINT64_t;
typedef             __int64         INT64_t;

#define         PRINTF_INT64_MODIFIER   "I64"




#define       STRING_TO_UPPER         _strupr    
#define       STRING_COMPARE_NOCASE   strcmpi

#endif