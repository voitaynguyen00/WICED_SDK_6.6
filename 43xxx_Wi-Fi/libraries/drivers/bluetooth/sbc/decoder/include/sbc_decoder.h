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


#ifndef SBC_DECODER_H
#define SBC_DECODER_H

#define DECODER_VERSION "0019"

#include "bt_target.h"

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif

/*DEFINES*/
#define SBC_CRC_FAILURE -1
#define SBC_SYNC_FAILURE -2
#define SBC_EMPTY_FAILURE -3
#define SBC_MEM_FAILURE -4
#define SBC_FAILURE 0
#define SBC_SUCCESS 1

#define SBC_NULL 0

#define SBC_MAX_NUM_OF_SUBBANDS	8
#define	SBC_MAX_NUM_OF_CHANNELS	2
#define SBC_MAX_NUM_OF_BLOCKS	16

#define SBC_LOUDNESS	0
#define SBC_SNR	1

#define SBC_SUB_BANDS_8	8
#define SBC_SUB_BANDS_4	4

#define SBC_sf16000	0
#define SBC_sf32000	1
#define SBC_sf44100	2
#define SBC_sf48000	3

#define SBC_MONO	0
#define SBC_DUAL	1
#define SBC_STEREO	2
#define SBC_JOINT_STEREO	3

#define SBC_BLOCK_0	4
#define SBC_BLOCK_1	8
#define SBC_BLOCK_2	12
#define SBC_BLOCK_3	16

#define SBC_HEADER_SIZE 4 /* sync byte, ch mode/num subb/num blk byte, bit pool byte, and crc */

#ifndef SBC_MAX_PACKET_LENGTH
#define SBC_MAX_PACKET_LENGTH 1000
#endif
/*constants used for index calculation*/
#define CHL	SBC_MAX_NUM_OF_SUBBANDS
#define BLK	(SBC_MAX_NUM_OF_CHANNELS*CHL)

#define DEC_VX_MINIMUM_BUFFER_SIZE (SBC_MAX_NUM_OF_SUBBANDS*SBC_MAX_NUM_OF_CHANNELS*20)

#ifndef DEC_VX_BUFFER_SIZE
#define DEC_VX_BUFFER_SIZE (DEC_VX_MINIMUM_BUFFER_SIZE+(20*SBC_SUB_BANDS_8))
/*#define DEC_VX_BUFFER_SIZE DEC_VX_MINIMUM_BUFFER_SIZE*/
#endif

/* this  SBC_ARM_OPT flag should not be change, it woudl degrate the quality bellow the sig accptance criteria */
#ifndef SBC_ARM_OPT
#define SBC_ARM_OPT FALSE
#endif 

/* this  SBC_ARM_INLINE_ASM it to used only if SBC_ARM_OPT == TRUE. please refer to the above comment */
#ifndef SBC_ARM_INLINE_ASM
#define SBC_ARM_INLINE_ASM FALSE
#endif 

#ifndef SBC_C5402_OPT
#define SBC_C5402_OPT FALSE
#endif

#ifndef SBC_OPTIMIZATION
#define SBC_OPTIMIZATION TRUE
#endif 

/* 32 and 64 bit mult will be performed using SINT64 ( usualy __int64 ) cast that usualy give optimal performance if supported */
#ifndef SBC_IPAQ_OPT
#define SBC_IPAQ_OPT TRUE
#endif 

/* TRUE to perform the dequantification 32x32, 16x32 operation in 64 bit, false to perform those in 32 bit */
#ifndef SBC_IS_64_BITS_IN_DEQUANTIZER
#define SBC_IS_64_BITS_IN_DEQUANTIZER TRUE
#endif

/* TRUE to perform the windowing MAC 32x32, 16x32 operation in 64 bit, false to perform those in 32 bit */
#ifndef SBC_IS_64_BITS_IN_WINDOWING
#define SBC_IS_64_BITS_IN_WINDOWING FALSE
#endif

/* TRUE to hold the VX vector in 16 bit, FALSE for 32 bit */
#ifndef SBC_VECTOR_VX_ON_16BITS
#define SBC_VECTOR_VX_ON_16BITS FALSE
#endif

/* TRUE to perform the DCT 32x32, 16x32 operation in 64 bit, false to perform those in 32 bit */
#ifndef SBC_IS_64_MULT_IN_DCT
#define SBC_IS_64_MULT_IN_DCT FALSE
#endif

/* 32 and 64 bit mult will be performed using SINT64 ( long long ) cast that usualy give optimal performance if supported */
#ifndef SBC_FOR_EMBEDDED_LINUX
#define SBC_FOR_EMBEDDED_LINUX FALSE
#endif

#ifndef SBC_GKI_BUFFERBASED
#define SBC_GKI_BUFFERBASED FALSE
#endif /* SBC_GKI_BUFFERBASED */

#ifndef SBC_MINIMUM_BUFFER_COPIES
#define SBC_MINIMUM_BUFFER_COPIES FALSE
#endif

/* this SBC_DEC_SANITY_CHECK verify that we have enough byte to decode before starting the decoding */
/* this is sometime necessary when the applicaiton is not able to provide the data require when they call sbc decoder */
#ifndef SBC_DEC_SANITY_CHECK
#define SBC_DEC_SANITY_CHECK FALSE
#endif

/* exclude/include 4 subband code */
#ifndef SBC_DEC_4_SUBD_INCLUDED
#define SBC_DEC_4_SUBD_INCLUDED TRUE
#endif

/* exclude/include 8 subband code */
#ifndef SBC_DEC_8_SUBD_INCLUDED
#define SBC_DEC_8_SUBD_INCLUDED TRUE
#endif

/* exclude/include stereo code */
#ifndef SBC_DEC_STEREO_INCLUDED
#define SBC_DEC_STEREO_INCLUDED TRUE
#endif

#include "sbc_types.h"

typedef struct SBC_DEC_PARAMS_TAG
{
    UINT16 (*AllocBufferFP)(SINT16 **, UINT16); 
    UINT16 (*FillBufferFP)(UINT8 *, UINT16); 
    UINT16 (*EmptyBufferFP)(SINT16 *, UINT16);
    SINT32 *s32ScartchMem;
    SINT32 *s32StaticMem;
    SINT16 *scartchMemForBitAlloc;
    UINT8  *packet;
    SINT16 *scaleFactor;
    SINT16 *pcmBuffer;
    SINT16 *bits;
#if (SBC_ARM_OPT==TRUE)
    SINT16 *sbBuffer;
#else
    SINT32 *sbBuffer;
#endif
#if (SBC_ARM_OPT==FALSE && SBC_IPAQ_OPT==FALSE && SBC_OPTIMIZATION==FALSE) 
    SINT32 *scartchMemForFilter;
#endif
#if (SBC_VECTOR_VX_ON_16BITS==TRUE)
    SINT16 *VX;
#else
    SINT32 *VX;
#endif
    SINT32 ShiftCounter[2];
    SINT16 samplingFreq;        /*16k, 32k, 44.1k or 48k*/
    SINT16 channelMode;         /*mono, dual, streo or joint streo*/
    SINT16 numOfSubBands;       /*4 or 8*/
    SINT16 numOfChannels;
    SINT16 numOfBlocks;         /*4, 8, 12 or 16*/
    SINT16 allocationMethod;    /*loudness or SNR*/
    SINT16 bitPool;             /* 16*numOfSb for mono & dual; 32*numOfSb for stereo & joint stereo */
    UINT8 join[SBC_MAX_NUM_OF_SUBBANDS];       /*0 if not joint stereo*/

    UINT16 u16PrevPacLen;
    UINT16 u16ReaminingBytes;

}SBC_DEC_PARAMS;

/*FUNCTION DECLARATIONS*/
extern const SINT16 DCTcoeff4[];
extern const SINT16 DCTcoeff8[];
void SbcSynthesisFilter(SBC_DEC_PARAMS *);

#ifdef __cplusplus
extern "C"
{
#endif
SBC_API extern SINT16 SBC_Decoder(SBC_DEC_PARAMS *strDecParams);
SBC_API extern SINT16 SBC_Decoder_Init(SBC_DEC_PARAMS *strDecParams);
#ifdef __cplusplus
}
#endif


SINT16 DecUnPacking(SBC_DEC_PARAMS *strDecParams);

#if (SBC_C5402_OPT==TRUE)
    void SBC_Multiply_32_16_Simplified(SINT16 s32In2Temp,SINT32  s32In1Temp ,SINT32 *s32OutLow);
    void SBC_Multiply_64(SINT32 s32In1Temp,SINT32 s32In2Temp,SINT32 *s32OutLow,SINT32 *s32OutHi);
    void SBC_Multiply_32_32(SINT32 s32In2Temp,SINT32 s32In1Temp,SINT32 *s32OutLow);
    void SBC_Multiply_32_32_Simplified(SINT32 s32In2Temp,SINT32 s32In1Temp,SINT32 *s32OutLow);
#endif
#endif
