/*
 *  FIPS-180-2 compliant SHA-256 implementation
 *
 *  Copyright (C) 2006-2010, Brainspark B.V.
 *
 *  This file is part of PolarSSL (http://www.polarssl.org)
 *  Lead Maintainer: Paul Bakker <polarssl_maintainer at polarssl.org>
 *
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
/*
 *  The SHA-256 Secure Hash Standard was published by NIST in 2002.
 *
 *  http://csrc.nist.gov/publications/fips/fips180-2/fips180-2.pdf
 */

#include "sha2.h"
//unsigned long sha2_algo = 0;

/*
 * 32-bit integer manipulation macros (big endian)
 */
unsigned long GET_ULONG_BE(const unsigned char* b, unsigned long i)
{
	return (b[i] << 24 )|(b[i+1] << 16 )|(b[i+2] <<  8 )| (b[i+3]);
}
void PUT_ULONG_BE(unsigned long n, unsigned char* b, unsigned long i)
{
    (b)[(i)    ] = (unsigned char) ( (n) >> 24 );
    (b)[(i) + 1] = (unsigned char) ( (n) >> 16 );
    (b)[(i) + 2] = (unsigned char) ( (n) >>  8 );
    (b)[(i) + 3] = (unsigned char) ( (n)       );	
}
/*
 * SHA-256 context setup
 */

const unsigned long X[8] = {
0x6A09E667,
0xBB67AE85,
0x3C6EF372,
0xA54FF53A,
0x510E527F,
0x9B05688C,
0x1F83D9AB,
0x5BE0CD19
};
const unsigned long Z[64] = {0x428A2F98, 0x71374491, 0xB5C0FBCF, 0xE9B5DBA5,
						   0x3956C25B, 0x59F111F1, 0x923F82A4, 0xAB1C5ED5,
						   0xD807AA98, 0x12835B01, 0x243185BE, 0x550C7DC3,
						   0x72BE5D74, 0x80DEB1FE, 0x9BDC06A7, 0xC19BF174,
						   0xE49B69C1, 0xEFBE4786, 0x0FC19DC6, 0x240CA1CC,
						   0x2DE92C6F, 0x4A7484AA, 0x5CB0A9DC, 0x76F988DA,
						   0x983E5152, 0xA831C66D, 0xB00327C8, 0xBF597FC7,
						   0xC6E00BF3, 0xD5A79147, 0x06CA6351, 0x14292967,
						   0x27B70A85, 0x2E1B2138, 0x4D2C6DFC, 0x53380D13,
						   0x650A7354, 0x766A0ABB, 0x81C2C92E, 0x92722C85,
						   0xA2BFE8A1, 0xA81A664B, 0xC24B8B70, 0xC76C51A3,
						   0xD192E819, 0xD6990624, 0xF40E3585, 0x106AA070,
						   0x19A4C116, 0x1E376C08, 0x2748774C, 0x34B0BCB5,
						   0x391C0CB3, 0x4ED8AA4A, 0x5B9CCA4F, 0x682E6FF3,
						   0x748F82EE, 0x78A5636F, 0x84C87814, 0x8CC70208,
						   0x90BEFFFA, 0xA4506CEB, 0xBEF9A3F7, 0xC67178F2
	};


void sha2_starts( sha2_context *ctx, int is224 )
{
    ctx->total[0] = 0;
    ctx->total[1] = 0;
	memcpy(ctx->state, X, 8*sizeof(unsigned long));

}

#define  SHR(x,n) ((x & 0xFFFFFFFF) >> n)
#define ROTR(x,n) (SHR(x,n) | (x << (32 - n)))

#define S0(x) (ROTR(x, 7) ^ ROTR(x,18) ^  SHR(x, 3))
#define S1(x) (ROTR(x,17) ^ ROTR(x,19) ^  SHR(x,10))

#define S2(x) (ROTR(x, 2) ^ ROTR(x,13) ^ ROTR(x,22))
#define S3(x) (ROTR(x, 6) ^ ROTR(x,11) ^ ROTR(x,25))

#define F0(x,y,z) ((x & y) | (z & (x | y)))
#define F1(x,y,z) (z ^ (x & (y ^ z)))

static void sha2_process( sha2_context *ctx, const unsigned char data[64] )
{
    unsigned long temp1, temp2, W[64], i;
	unsigned long Y[8];
			
	memcpy(Y, ctx->state, 8*sizeof(unsigned long));

	for(i=0;i<64;i++)
	{
		unsigned long j = (64-i)%8;
		if(i<16)
			W[i] = GET_ULONG_BE(data,  i*4 );
		else
			W[i] = S1(W[i -  2]) + W[i -  7] + S0(W[i - 15]) + W[i - 16];
		
		temp1 = Y[(j+7)%8] + S3(Y[(j+4)%8]) + F1(Y[(j+4)%8],Y[(j+5)%8],Y[(j+6)%8]) + Z[i] + W[i];
		temp2 = S2(Y[j%8]) + F0(Y[j%8],Y[(j+1)%8],Y[(j+2)%8]);
		Y[(j+3)%8] += temp1;
		Y[(j+7)%8] = temp1 + temp2;
	}
	for(i=0;i<8;i++)
		ctx->state[i] += Y[i];
}

/*
 * SHA-256 process buffer
 */
void sha2_update( sha2_context *ctx, unsigned char *input, size_t ilen )
{
    size_t fill;
    unsigned long left;

    if( ilen <= 0 )
        return;

    left = ctx->total[0] & 0x3F;
    fill = 64 - left;

    ctx->total[0] += (unsigned long) ilen;
    ctx->total[0] &= 0xFFFFFFFF;

    if( ctx->total[0] < (unsigned long) ilen )
        ctx->total[1]++;

    if( left && ilen >= fill )
    {
        memcpy( (void *) (ctx->buffer + left),
                (void *) input, fill );
        sha2_process( ctx, ctx->buffer );
        input += fill;
        ilen  -= fill;
        left = 0;
    }

    while( ilen >= 64 )
    {
        sha2_process( ctx, input );
        input += 64;
        ilen  -= 64;
    }

    if( ilen > 0 )
    {
        memcpy( (void *) (ctx->buffer + left),
                (void *) input, ilen );
    }
}

/*
 * SHA-256 final digest
 */
void sha2_finish( sha2_context *ctx, unsigned char output[32] )
{
    unsigned long last, padn, i;
    unsigned long high, low;
    unsigned char msglen[8];
	unsigned char sha2_padding[64];

	memset(sha2_padding, 0, 64);
	sha2_padding[0] = 0x80;
	
    high = ( ctx->total[0] >> 29 )
         | ( ctx->total[1] <<  3 );
    low  = ( ctx->total[0] <<  3 );

    PUT_ULONG_BE( high, msglen, 0 );
    PUT_ULONG_BE( low,  msglen, 4 );

    last = ctx->total[0] & 0x3F;
    padn = ( last < 56 ) ? ( 56 - last ) : ( 120 - last );

    sha2_update( ctx, (unsigned char *) sha2_padding, padn );
    sha2_update( ctx, msglen, 8 );

	for(i=0;i<8;i++)
		PUT_ULONG_BE( ctx->state[i], output,  i*4 );
}

#if defined(SHA2_SELF_TEST)

#include <stdio.h>

/*
* FIPS-180-2 test vectors
*/
static const unsigned char sha2_test_buf[3][57] =
{
    { "abc" },
    { "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq" },
    { "" }
};

static const int sha2_test_buflen[3] =
{
    3, 56, 1000
};

static const unsigned char sha2_test_sum[6][32] =
{
    /*
    * SHA-224 test vectors
    */
    { 0x23, 0x09, 0x7D, 0x22, 0x34, 0x05, 0xD8, 0x22,
    0x86, 0x42, 0xA4, 0x77, 0xBD, 0xA2, 0x55, 0xB3,
    0x2A, 0xAD, 0xBC, 0xE4, 0xBD, 0xA0, 0xB3, 0xF7,
    0xE3, 0x6C, 0x9D, 0xA7 },
    { 0x75, 0x38, 0x8B, 0x16, 0x51, 0x27, 0x76, 0xCC,
    0x5D, 0xBA, 0x5D, 0xA1, 0xFD, 0x89, 0x01, 0x50,
    0xB0, 0xC6, 0x45, 0x5C, 0xB4, 0xF5, 0x8B, 0x19,
    0x52, 0x52, 0x25, 0x25 },
    { 0x20, 0x79, 0x46, 0x55, 0x98, 0x0C, 0x91, 0xD8,
    0xBB, 0xB4, 0xC1, 0xEA, 0x97, 0x61, 0x8A, 0x4B,
    0xF0, 0x3F, 0x42, 0x58, 0x19, 0x48, 0xB2, 0xEE,
    0x4E, 0xE7, 0xAD, 0x67 },

    /*
    * SHA-256 test vectors
    */
    { 0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA,
    0x41, 0x41, 0x40, 0xDE, 0x5D, 0xAE, 0x22, 0x23,
    0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17, 0x7A, 0x9C,
    0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD },
    { 0x24, 0x8D, 0x6A, 0x61, 0xD2, 0x06, 0x38, 0xB8,
    0xE5, 0xC0, 0x26, 0x93, 0x0C, 0x3E, 0x60, 0x39,
    0xA3, 0x3C, 0xE4, 0x59, 0x64, 0xFF, 0x21, 0x67,
    0xF6, 0xEC, 0xED, 0xD4, 0x19, 0xDB, 0x06, 0xC1 },
    { 0xCD, 0xC7, 0x6E, 0x5C, 0x99, 0x14, 0xFB, 0x92,
    0x81, 0xA1, 0xC7, 0xE2, 0x84, 0xD7, 0x3E, 0x67,
    0xF1, 0x80, 0x9A, 0x48, 0xA4, 0x97, 0x20, 0x0E,
    0x04, 0x6D, 0x39, 0xCC, 0xC7, 0x11, 0x2C, 0xD0 }
};

/*
* Checkup routine
*/
int sha2_self_test(int verbose)
{
    int i, j, k, buflen;
    unsigned char buf[1024];
    unsigned char sha2sum[32];
    sha2_context ctx;

    for (i = 3; i < 6; i++)
    {
        j = i % 3;
        k = i < 3;

        if (verbose != 0)
            printf("  SHA-%ld test #%ld: ", (long)(256 - k * 32), (long)(j + 1));

        sha2_starts(&ctx, k);

        if (j == 2)
        {
            memset(buf, 'a', buflen = 1000);

            for (j = 0; j < 1000; j++)
                sha2_update(&ctx, buf, buflen);
        }
        else
            sha2_update(&ctx, sha2_test_buf[j],
                sha2_test_buflen[j]);

        sha2_finish(&ctx, sha2sum);

        if (memcmp(sha2sum, sha2_test_sum[i], 32 - k * 4) != 0)
        {
            if (verbose != 0)
                printf("failed\n");

            return(1);
        }

        if (verbose != 0)
            printf("passed\n");
    }

}

#endif
