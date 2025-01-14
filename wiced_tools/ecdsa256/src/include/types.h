/*
********************************************************************
* THIS INFORMATION IS PROPRIETARY TO
* BROADCOM CORP.
*-------------------------------------------------------------------
*
*           Copyright (c) 2003 Broadcom Corp.
*                      ALL RIGHTS RESERVED
*
********************************************************************

********************************************************************
*    File Name: Types.h
*
*    Abstract: Basic type definitions for the BCM204X Baseband Controller
*              Firmware
*
*    $History:$
*
********************************************************************
*/

#ifndef __TYPES_H__
#define    __TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

//
// Basic Types
//

#ifndef VOID
#define VOID void
#endif

#if defined(WIN32)
typedef char                CHAR;
typedef short               SHORT;
typedef long                LONG;
typedef unsigned char       UINT8;
typedef signed   char       INT8;
typedef unsigned short      UINT16;
typedef signed   short      INT16;
typedef unsigned int        UINT32;
typedef signed   int        INT32;
typedef signed   __int64    INT64;
typedef unsigned __int64    UINT64;

typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;

typedef unsigned char       uint8;
typedef signed   char       int8;
typedef unsigned short      uint16;
typedef signed   short      int16;
typedef unsigned long       uint32;
typedef signed   long       int32;

#else

typedef unsigned char       UINT8;
typedef signed   char       INT8;
typedef unsigned short      UINT16;
typedef signed   short      INT16;
typedef unsigned int       UINT32;
typedef signed   int       INT32;
typedef signed long long    INT64;
typedef unsigned long long  UINT64;

typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned int        DWORD;

typedef unsigned char       uint8;
typedef signed   char       int8;
typedef unsigned short      uint16;
typedef signed   short      int16;
typedef unsigned int       uint32;
typedef signed   int       int32;

#endif // win32

// Machine dependent data types, for processor efficiency
typedef unsigned int MBOOL;
typedef unsigned int MU8;
typedef unsigned int MU16;
typedef unsigned int MU32;
typedef int MI8;
typedef int MI16;
typedef int MI32;


// This definition is for backward compatibility
#ifndef _WIN32
typedef UINT8            BOOL;
#endif
typedef UINT8            BOOLEAN;


typedef UINT32          BOOL32;
typedef UINT8            BOOL8;

typedef unsigned char  byte;

#ifndef NULL
#define NULL 0
#endif


//
// Basic Definitions
//

#ifndef TRUE
#define    TRUE    1
#endif

#ifndef FALSE
#define    FALSE    0
#endif


//
// pack macro
//
#if defined(COMPILER_ARM)
#define PACKED    __packed
#define ALIGN4  __align(4)
#define ATTRIBUTE(x) __attribute__(x)
#define PLACE_IN_DROM1 (section("const_drom_var"))
//Satya -- need this for seamless debugging..no run time penalty
#define PLACE_IN_DROM ATTRIBUTE(PLACE_IN_DROM1)
#else // some makefiles may not define COMPILER_GHS
#define PACKED
#define ALIGN4
#define ATTRIBUTE(x)
#define PLACE_IN_DROM
#endif

//
// inline macro
//

#ifndef INLINE
#ifdef __cplusplus
#if defined(COMPILER_ARM)
#define INLINE __inline
#else
#define INLINE inline
#endif
#else
#define INLINE __inline
#endif
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
//
// #define FOO FREESODA
// printf("%s    %s\n", STRINGIFY(FOO), TOSTRING(FOO)); 
//
// The code above will generate the following output:
//        FOO    FREESODA
  
//
// Assume we are compiled under Little Endian system.
#define BE_SWAP_16(var)  (( UINT16)( ((var) << 8) | (((var) >> 8) & 0xff) ) )
#define LE_SWAP_16(var)  (var)
#define BE_SWAP_32(var)  (     ((var & 0x000000FF)<<24)  \
                           |   ((var & 0x0000FF00)<<8)   \
                           |   ((var & 0x00FF0000)>>8)   \
                           |  ((var & 0xFF000000)>>24)   \
                         )
#define LE_SWAP_32(var)    (var)

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))


void nv_log_assert (char *file, int line, UINT32 status);
BOOL32 nv_log_is_asserts_present(void);

#ifdef DUMP_STACK_AT_CRASH
void nv_log_assert_stack(UINT32 *sp, UINT32 offset);
void nv_log_assert_ext (char *file, int line, UINT32 status, int status_ext);
#endif


#ifndef ASSERT
#ifdef ENABLE_ASSERT

#ifdef FPGA_BD_2045
void assert_fail( char*   file,   int  line , UINT32 status, char* fatal);       
#define   ASSERT( X , status)        		do{	if((X) == FALSE)	assert_fail(__FILE__, __LINE__, (UINT32)status, NULL);} while(0)
//void assert_fail( char*   file,   int  line , int status, char* fatal);
#define	ASSERT_FATAL(X)	  				 assert_fail( __FILE__, __LINE__, 0, X)
#else

void assert_fail( char*   file,   int  line , UINT32 status);
#define   ASSERT( X , status)   			do {if((X) == FALSE)	assert_fail(__FILE__, __LINE__, (UINT32)status);} while(0)
#define	ASSERT_FATAL(X)	  				 assert_fail( __FILE__, __LINE__, 0);

#endif

#else // compile assertion out
  #define   ASSERT( X , status)
  #define   ASSERT_FATAL(X)
#endif
#endif

#ifdef __cplusplus
}
#endif

/*
*******************************************************************************
*
* $HISTORY:$
*
*******************************************************************************
*/
#endif // __TYPES_H__
