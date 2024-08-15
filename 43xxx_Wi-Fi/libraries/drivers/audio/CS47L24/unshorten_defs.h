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

/*
 * Copyright (C) 2013 Wolfson Microelectronics plc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/* #define INCLUDE_PASSTHRU_DATA */

#ifdef _DEBUG
#define IF_DEBUG(x)               x
#else
#define IF_DEBUG(x)
#endif

#define MACHINE_WORD_SIZE         24
#define MAX_SIGNED_WORD           0x7fffff

#define BITS_PER_SAMPLE_V         16
#define MAX_SAMPLE_VALUE          32767
#define MIN_SAMPLE_VALUE          (-32768)
#define BITS_PER_ADSP_WORD        24
#define SAMPLE_MASK_V             ((1<<BITS_PER_SAMPLE_V)-1)
#define BITS_TO_ADSP_WORDS(n)     (((n)+BITS_PER_ADSP_WORD-1)/BITS_PER_ADSP_WORD)
#define BLOCK_SIZE_SAMPLES        240
#define BLOCK_SIZE_WORDS_V        BITS_TO_ADSP_WORDS(BLOCK_SIZE_SAMPLES*BITS_PER_SAMPLE_V)
#define NUM_ORDERS                4
#define SAMPLES_CARRIED           (NUM_ORDERS - 1)

#define TYPE_FIELD_BITS           4
#define TYPE_FIELD_MASK           ((1<<TYPE_FIELD_BITS)-1)
#ifdef INCLUDE_PASSTHRU_DATA
#define LENGTH_FIELD_BITS         9
#else
#define LENGTH_FIELD_BITS         8
#endif
#define LENGTH_FIELD_MASK         ((1<<LENGTH_FIELD_BITS)-1)
#define HN_FIELD_BITS             4
#define HN_FIELD_MASK             ((1<<HN_FIELD_BITS)-1)

#define MAX_BLOCK_SIZE_V          (BLOCK_SIZE_WORDS_V + 1)
#ifdef INCLUDE_PASSTHRU_DATA
#define MAX_BLOCK_SIZE            LENGTH_FIELD_MASK
#else
#define MAX_BLOCK_SIZE            MAX_BLOCK_SIZE_V
#endif

/* extract record type from first word */
#define RECORD_TYPE(fw)           ((fw) & TYPE_FIELD_MASK)

/* extract record size from first word */
#define RECORD_SIZE(fw)           (((fw) >> TYPE_FIELD_BITS) & LENGTH_FIELD_MASK)

/* extract previous record size from first word */
#define RECORD_PREV_SIZE(fw)      (((fw) >> (TYPE_FIELD_BITS + LENGTH_FIELD_BITS)) & LENGTH_FIELD_MASK)

/******************************************************
 *                     Constants
 ******************************************************/

#define COMPR_RECORD_ORDER0       0
#define COMPR_RECORD_ORDER1       1
#define COMPR_RECORD_ORDER2       2
#define COMPR_RECORD_ORDER3       3
#define COMPR_RECORD_VERBATIM     4

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

#ifndef __ADSP2__
typedef signed char               s8;
typedef signed short              s16;
typedef signed long               s32;
typedef unsigned char             u8;
typedef unsigned short            u16;
typedef unsigned long             u32;
typedef s16                       Sample;

#ifdef __VC32__
typedef signed __int64            s64;
typedef unsigned __int64          u64;
#else
typedef signed long long          s64;
typedef unsigned long long        u64;
#endif
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
