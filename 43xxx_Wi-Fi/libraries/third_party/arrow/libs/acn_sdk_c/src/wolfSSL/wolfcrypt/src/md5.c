/* md5.c
 *
 * Copyright (C) 2006-2016 wolfSSL Inc.
 *
 * This file is part of wolfSSL.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */



#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

#include <wolfssl/wolfcrypt/settings.h>
typedef int __pedantic_dummy;

#if !defined(NO_MD5)

#if defined(WOLFSSL_TI_HASH)
    /* #include <wolfcrypt/src/port/ti/ti-hash.c> included by wc_port.c */
#else

#ifdef WOLFSSL_PIC32MZ_HASH
#define wc_InitMd5   wc_InitMd5_sw
#define wc_Md5Update wc_Md5Update_sw
#define wc_Md5Final  wc_Md5Final_sw
#endif

#include <wolfssl/wolfcrypt/md5.h>
#include <wolfssl/wolfcrypt/error-crypt.h>

#ifdef NO_INLINE
    #include <wolfssl/wolfcrypt/misc.h>
#else
    #include <wolfcrypt/src/misc.c>
#endif

#ifdef FREESCALE_MMCAU
    #include "cau_api.h"
    #define XTRANSFORM(S,B)  Transform((S), (B))
#else
    #define XTRANSFORM(S,B)  Transform((S))
#endif


#ifdef STM32F2_HASH
    /*
     * STM32F2 hardware MD5 support through the STM32F2 standard peripheral
     * library. Documentation located in STM32F2xx Standard Peripheral Library
     * document (See note in README).
     */
    #include "stm32f2xx.h"

    void wc_InitMd5(Md5* md5)
    {
        /* STM32F2 struct notes:
         * md5->buffer  = first 4 bytes used to hold partial block if needed 
         * md5->buffLen = num bytes currently stored in md5->buffer
         * md5->loLen   = num bytes that have been written to STM32 FIFO
         */
        XMEMSET(md5->buffer, 0, MD5_REG_SIZE);
			
        md5->buffLen = 0;
        md5->loLen = 0;

        /* initialize HASH peripheral */
        HASH_DeInit();

        /* configure algo used, algo mode, datatype */
        HASH->CR &= ~ (HASH_CR_ALGO | HASH_CR_DATATYPE | HASH_CR_MODE);
        HASH->CR |= (HASH_AlgoSelection_MD5 | HASH_AlgoMode_HASH 
                 | HASH_DataType_8b);

        /* reset HASH processor */
        HASH->CR |= HASH_CR_INIT;
    }

    void wc_Md5Update(Md5* md5, const byte* data, word32 len)
    {
        word32 i = 0;
        word32 fill = 0;
        word32 diff = 0;

        /* if saved partial block is available */
        if (md5->buffLen > 0) {
            fill = 4 - md5->buffLen;

            /* if enough data to fill, fill and push to FIFO */
            if (fill <= len) {
                XMEMCPY((byte*)md5->buffer + md5->buffLen, data, fill);
                HASH_DataIn(*(uint32_t*)md5->buffer);

                data += fill;
                len -= fill;
                md5->loLen += 4;
                md5->buffLen = 0;
            } else {
                /* append partial to existing stored block */
                XMEMCPY((byte*)md5->buffer + md5->buffLen, data, len);
                md5->buffLen += len;
                return;
            }
        }

        /* write input block in the IN FIFO */
        for (i = 0; i < len; i += 4)
        {
            diff = len - i;
            if (diff < 4) {
                /* store incomplete last block, not yet in FIFO */
                XMEMSET(md5->buffer, 0, MD5_REG_SIZE);
                XMEMCPY((byte*)md5->buffer, data, diff);
                md5->buffLen = diff;
            } else {
                HASH_DataIn(*(uint32_t*)data);
                data+=4;
            }
        }

        /* keep track of total data length thus far */
        md5->loLen += (len - md5->buffLen);
    }

    void wc_Md5Final(Md5* md5, byte* hash)
    {
        __IO uint16_t nbvalidbitsdata = 0;

        /* finish reading any trailing bytes into FIFO */
        if (md5->buffLen > 0) {
            HASH_DataIn(*(uint32_t*)md5->buffer);
            md5->loLen += md5->buffLen;
        }

        /* calculate number of valid bits in last word of input data */
        nbvalidbitsdata = 8 * (md5->loLen % MD5_REG_SIZE);

        /* configure number of valid bits in last word of the data */
        HASH_SetLastWordValidBitsNbr(nbvalidbitsdata);

        /* start HASH processor */
        HASH_StartDigest();

        /* wait until Busy flag == RESET */
        while (HASH_GetFlagStatus(HASH_FLAG_BUSY) != RESET) {}
        
        /* read message digest */
        md5->digest[0] = HASH->HR[0];
        md5->digest[1] = HASH->HR[1];
        md5->digest[2] = HASH->HR[2];
        md5->digest[3] = HASH->HR[3];

        ByteReverseWords(md5->digest, md5->digest, MD5_DIGEST_SIZE);

        XMEMCPY(hash, md5->digest, MD5_DIGEST_SIZE);

        wc_InitMd5(md5);  /* reset state */
    }

#else /* CTaoCrypt software implementation */

#ifndef WOLFSSL_HAVE_MIN
#define WOLFSSL_HAVE_MIN

    static INLINE word32 min(word32 a, word32 b)
    {
        return a > b ? b : a;
    }

#endif /* WOLFSSL_HAVE_MIN */

void wc_InitMd5(Md5* md5)
{
    md5->digest[0] = 0x67452301L;
    md5->digest[1] = 0xefcdab89L;
    md5->digest[2] = 0x98badcfeL;
    md5->digest[3] = 0x10325476L;

    md5->buffLen = 0;
    md5->loLen   = 0;
    md5->hiLen   = 0;
}

#ifdef FREESCALE_MMCAU
static int Transform(Md5* md5, byte* data)
{
    int ret = wolfSSL_CryptHwMutexLock();
    if(ret == 0) {
        cau_md5_hash_n(data, 1, (unsigned char*)md5->digest);
        wolfSSL_CryptHwMutexUnLock();
    }
    return ret;
}
#endif /* FREESCALE_MMCAU */

#ifndef FREESCALE_MMCAU

static void Transform(Md5* md5)
{
#define F1(x, y, z) (z ^ (x & (y ^ z)))
#define F2(x, y, z) F1(z, x, y)
#define F3(x, y, z) (x ^ y ^ z)
#define F4(x, y, z) (y ^ (x | ~z))

#define MD5STEP(f, w, x, y, z, data, s) \
    w = rotlFixed(w + f(x, y, z) + data, s) + x

    /* Copy context->state[] to working vars  */
    word32 a = md5->digest[0];
    word32 b = md5->digest[1];
    word32 c = md5->digest[2];
    word32 d = md5->digest[3];

    MD5STEP(F1, a, b, c, d, md5->buffer[0]  + 0xd76aa478,  7);
    MD5STEP(F1, d, a, b, c, md5->buffer[1]  + 0xe8c7b756, 12);
    MD5STEP(F1, c, d, a, b, md5->buffer[2]  + 0x242070db, 17);
    MD5STEP(F1, b, c, d, a, md5->buffer[3]  + 0xc1bdceee, 22);
    MD5STEP(F1, a, b, c, d, md5->buffer[4]  + 0xf57c0faf,  7);
    MD5STEP(F1, d, a, b, c, md5->buffer[5]  + 0x4787c62a, 12);
    MD5STEP(F1, c, d, a, b, md5->buffer[6]  + 0xa8304613, 17);
    MD5STEP(F1, b, c, d, a, md5->buffer[7]  + 0xfd469501, 22);
    MD5STEP(F1, a, b, c, d, md5->buffer[8]  + 0x698098d8,  7);
    MD5STEP(F1, d, a, b, c, md5->buffer[9]  + 0x8b44f7af, 12);
    MD5STEP(F1, c, d, a, b, md5->buffer[10] + 0xffff5bb1, 17);
    MD5STEP(F1, b, c, d, a, md5->buffer[11] + 0x895cd7be, 22);
    MD5STEP(F1, a, b, c, d, md5->buffer[12] + 0x6b901122,  7);
    MD5STEP(F1, d, a, b, c, md5->buffer[13] + 0xfd987193, 12);
    MD5STEP(F1, c, d, a, b, md5->buffer[14] + 0xa679438e, 17);
    MD5STEP(F1, b, c, d, a, md5->buffer[15] + 0x49b40821, 22);

    MD5STEP(F2, a, b, c, d, md5->buffer[1]  + 0xf61e2562,  5);
    MD5STEP(F2, d, a, b, c, md5->buffer[6]  + 0xc040b340,  9);
    MD5STEP(F2, c, d, a, b, md5->buffer[11] + 0x265e5a51, 14);
    MD5STEP(F2, b, c, d, a, md5->buffer[0]  + 0xe9b6c7aa, 20);
    MD5STEP(F2, a, b, c, d, md5->buffer[5]  + 0xd62f105d,  5);
    MD5STEP(F2, d, a, b, c, md5->buffer[10] + 0x02441453,  9);
    MD5STEP(F2, c, d, a, b, md5->buffer[15] + 0xd8a1e681, 14);
    MD5STEP(F2, b, c, d, a, md5->buffer[4]  + 0xe7d3fbc8, 20);
    MD5STEP(F2, a, b, c, d, md5->buffer[9]  + 0x21e1cde6,  5);
    MD5STEP(F2, d, a, b, c, md5->buffer[14] + 0xc33707d6,  9);
    MD5STEP(F2, c, d, a, b, md5->buffer[3]  + 0xf4d50d87, 14);
    MD5STEP(F2, b, c, d, a, md5->buffer[8]  + 0x455a14ed, 20);
    MD5STEP(F2, a, b, c, d, md5->buffer[13] + 0xa9e3e905,  5);
    MD5STEP(F2, d, a, b, c, md5->buffer[2]  + 0xfcefa3f8,  9);
    MD5STEP(F2, c, d, a, b, md5->buffer[7]  + 0x676f02d9, 14);
    MD5STEP(F2, b, c, d, a, md5->buffer[12] + 0x8d2a4c8a, 20);

    MD5STEP(F3, a, b, c, d, md5->buffer[5]  + 0xfffa3942,  4);
    MD5STEP(F3, d, a, b, c, md5->buffer[8]  + 0x8771f681, 11);
    MD5STEP(F3, c, d, a, b, md5->buffer[11] + 0x6d9d6122, 16);
    MD5STEP(F3, b, c, d, a, md5->buffer[14] + 0xfde5380c, 23);
    MD5STEP(F3, a, b, c, d, md5->buffer[1]  + 0xa4beea44,  4);
    MD5STEP(F3, d, a, b, c, md5->buffer[4]  + 0x4bdecfa9, 11);
    MD5STEP(F3, c, d, a, b, md5->buffer[7]  + 0xf6bb4b60, 16);
    MD5STEP(F3, b, c, d, a, md5->buffer[10] + 0xbebfbc70, 23);
    MD5STEP(F3, a, b, c, d, md5->buffer[13] + 0x289b7ec6,  4);
    MD5STEP(F3, d, a, b, c, md5->buffer[0]  + 0xeaa127fa, 11);
    MD5STEP(F3, c, d, a, b, md5->buffer[3]  + 0xd4ef3085, 16);
    MD5STEP(F3, b, c, d, a, md5->buffer[6]  + 0x04881d05, 23);
    MD5STEP(F3, a, b, c, d, md5->buffer[9]  + 0xd9d4d039,  4);
    MD5STEP(F3, d, a, b, c, md5->buffer[12] + 0xe6db99e5, 11);
    MD5STEP(F3, c, d, a, b, md5->buffer[15] + 0x1fa27cf8, 16);
    MD5STEP(F3, b, c, d, a, md5->buffer[2]  + 0xc4ac5665, 23);

    MD5STEP(F4, a, b, c, d, md5->buffer[0]  + 0xf4292244,  6);
    MD5STEP(F4, d, a, b, c, md5->buffer[7]  + 0x432aff97, 10);
    MD5STEP(F4, c, d, a, b, md5->buffer[14] + 0xab9423a7, 15);
    MD5STEP(F4, b, c, d, a, md5->buffer[5]  + 0xfc93a039, 21);
    MD5STEP(F4, a, b, c, d, md5->buffer[12] + 0x655b59c3,  6);
    MD5STEP(F4, d, a, b, c, md5->buffer[3]  + 0x8f0ccc92, 10);
    MD5STEP(F4, c, d, a, b, md5->buffer[10] + 0xffeff47d, 15);
    MD5STEP(F4, b, c, d, a, md5->buffer[1]  + 0x85845dd1, 21);
    MD5STEP(F4, a, b, c, d, md5->buffer[8]  + 0x6fa87e4f,  6);
    MD5STEP(F4, d, a, b, c, md5->buffer[15] + 0xfe2ce6e0, 10);
    MD5STEP(F4, c, d, a, b, md5->buffer[6]  + 0xa3014314, 15);
    MD5STEP(F4, b, c, d, a, md5->buffer[13] + 0x4e0811a1, 21);
    MD5STEP(F4, a, b, c, d, md5->buffer[4]  + 0xf7537e82,  6);
    MD5STEP(F4, d, a, b, c, md5->buffer[11] + 0xbd3af235, 10);
    MD5STEP(F4, c, d, a, b, md5->buffer[2]  + 0x2ad7d2bb, 15);
    MD5STEP(F4, b, c, d, a, md5->buffer[9]  + 0xeb86d391, 21);
    
    /* Add the working vars back into digest state[]  */
    md5->digest[0] += a;
    md5->digest[1] += b;
    md5->digest[2] += c;
    md5->digest[3] += d;
}

#endif /* FREESCALE_MMCAU */


static INLINE void AddLength(Md5* md5, word32 len)
{
    word32 tmp = md5->loLen;
    if ( (md5->loLen += len) < tmp)
        md5->hiLen++;                       /* carry low to high */
}


void wc_Md5Update(Md5* md5, const byte* data, word32 len)
{
    /* do block size increments */
    byte* local = (byte*)md5->buffer;

    while (len) {
        word32 add = min(len, MD5_BLOCK_SIZE - md5->buffLen);
        XMEMCPY(&local[md5->buffLen], data, add);

        md5->buffLen += add;
        data         += add;
        len          -= add;

        if (md5->buffLen == MD5_BLOCK_SIZE) {
            #if defined(BIG_ENDIAN_ORDER) && !defined(FREESCALE_MMCAU)
                ByteReverseWords(md5->buffer, md5->buffer, MD5_BLOCK_SIZE);
            #endif
            XTRANSFORM(md5, local);
            AddLength(md5, MD5_BLOCK_SIZE);
            md5->buffLen = 0;
        }
    }
}


void wc_Md5Final(Md5* md5, byte* hash)
{
    byte* local = (byte*)md5->buffer;

    AddLength(md5, md5->buffLen);  /* before adding pads */

    local[md5->buffLen++] = 0x80;  /* add 1 */

    /* pad with zeros */
    if (md5->buffLen > MD5_PAD_SIZE) {
        XMEMSET(&local[md5->buffLen], 0, MD5_BLOCK_SIZE - md5->buffLen);
        md5->buffLen += MD5_BLOCK_SIZE - md5->buffLen;

        #if defined(BIG_ENDIAN_ORDER) && !defined(FREESCALE_MMCAU)
            ByteReverseWords(md5->buffer, md5->buffer, MD5_BLOCK_SIZE);
        #endif
        XTRANSFORM(md5, local);
        md5->buffLen = 0;
    }
    XMEMSET(&local[md5->buffLen], 0, MD5_PAD_SIZE - md5->buffLen);
   
    /* put lengths in bits */
    md5->hiLen = (md5->loLen >> (8*sizeof(md5->loLen) - 3)) + 
                 (md5->hiLen << 3);
    md5->loLen = md5->loLen << 3;

    /* store lengths */
    #if defined(BIG_ENDIAN_ORDER) && !defined(FREESCALE_MMCAU)
        ByteReverseWords(md5->buffer, md5->buffer, MD5_BLOCK_SIZE);
    #endif
    /* ! length ordering dependent on digest endian type ! */
    XMEMCPY(&local[MD5_PAD_SIZE], &md5->loLen, sizeof(word32));
    XMEMCPY(&local[MD5_PAD_SIZE + sizeof(word32)], &md5->hiLen, sizeof(word32));

    XTRANSFORM(md5, local);
    #ifdef BIG_ENDIAN_ORDER
        ByteReverseWords(md5->digest, md5->digest, MD5_DIGEST_SIZE);
    #endif
    XMEMCPY(hash, md5->digest, MD5_DIGEST_SIZE);

    wc_InitMd5(md5);  /* reset state */
}

#endif /* STM32F2_HASH */


int wc_Md5Hash(const byte* data, word32 len, byte* hash)
{
#ifdef WOLFSSL_SMALL_STACK
    Md5* md5;
#else
    Md5 md5[1];
#endif

#ifdef WOLFSSL_SMALL_STACK
    md5 = (Md5*)XMALLOC(sizeof(Md5), NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (md5 == NULL)
        return MEMORY_E;
#endif

    wc_InitMd5(md5);
    wc_Md5Update(md5, data, len);
    wc_Md5Final(md5, hash);

#ifdef WOLFSSL_SMALL_STACK
    XFREE(md5, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return 0;
}

#endif /* WOLFSSL_TI_HASH */

#endif /* NO_MD5 */

