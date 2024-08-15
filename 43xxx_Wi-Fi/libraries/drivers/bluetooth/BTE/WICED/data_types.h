/*
* Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
*/

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#ifndef NULL
#define NULL     0
#endif

#ifndef FALSE
#define FALSE  0
#endif

#if defined( __IAR_SYSTEMS_ICC__ )
typedef __UINT8_T_TYPE__  UINT8;
typedef __UINT16_T_TYPE__ UINT16;
typedef __UINT32_T_TYPE__ UINT32;
typedef __UINT64_T_TYPE__ UINT64;
typedef __INT32_T_TYPE__  INT32;
typedef __INT8_T_TYPE__   INT8;
typedef __INT16_T_TYPE__  INT16;
#else
typedef unsigned char   UINT8;
typedef unsigned short  UINT16;
typedef unsigned long   UINT32;
typedef unsigned long long int UINT64;
typedef signed   long   INT32;
typedef signed   char   INT8;
typedef signed   short  INT16;
#endif /* defined( __IAR_SYSTEMS_ICC__ ) */

typedef unsigned char   BOOLEAN;
typedef UINT32          UINTPTR;
typedef UINT32          TIME_STAMP;


typedef UINT8           BOOL8;
typedef UINT32          BOOL32;

typedef unsigned long long __int64;

#ifndef TRUE
#define TRUE   (!FALSE)
#endif

typedef unsigned char   UBYTE;

#ifdef __arm
#define PACKED  __packed
#define INLINE  __inline
#else
#define PACKED
#ifndef INLINE
#define INLINE
#endif
#endif

#ifndef BIG_ENDIAN
#define BIG_ENDIAN FALSE
#endif

#define UINT16_LOW_BYTE(x)      ((x) & 0xff)
#define UINT16_HI_BYTE(x)       ((x) >> 8)

/* MACRO definitions for safe string functions */
/* Replace standard string functions with safe functions if available */
#define BCM_STRCAT_S(x1,x2,x3)      strcat((x1),(x3))
#define BCM_STRNCAT_S(x1,x2,x3,x4)  strncat((x1),(x3),(x4))
#define BCM_STRCPY_S(x1,x2,x3)      strcpy((x1),(x3))
#define BCM_STRNCPY_S(x1,x2,x3,x4)  strncpy((x1),(x3),(x4))
#define BCM_SPRINTF_S(x1,x2,x3,x4)  sprintf((x1),(x3),(x4))
#define BCM_VSPRINTF_S(x1,x2,x3,x4) vsprintf((x1),(x3),(x4))

#define BT_MEMCPY(a,b,c)            memcpy( (a), (b), (c) )
#define BT_MEMSET(a,b,c)            memset( (a), (b), (c) )
#define BT_STRLEN(a)                strlen( (a) )
#define BT_MEMCMP(a,b,c)            memcmp( (a), (b), (c) )
#define BT_MEMCMP_OK                0                       // memcmp success

/* Timer list entry callback type
*/
typedef void (TIMER_CBACK)(void *p_tle);
#ifndef TIMER_PARAM_TYPE
#define TIMER_PARAM_TYPE    UINT32
#endif

#endif

