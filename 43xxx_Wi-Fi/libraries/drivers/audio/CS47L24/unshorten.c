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
 * Copyright (c) 2018 Cirrus Logic International (UK) Ltd.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wiced_result.h"
#include "unshorten.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                     Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                   Structures
 ******************************************************/

/******************************************************
 *              Variable declarations
 ******************************************************/

/******************************************************
 *             Function declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t init_unshorten(struct unshorten *u)
{
    IF_DEBUG(printf("init_unshorten: u=%p\n", u));
    u->input_data = 0;
    u->input_data_size = 0;
    u->input_index = 0;
    u->curr_block_size = 0;
    u->block_buf_index = 0;
    u->output_buf = (u8*) malloc((BLOCK_SIZE_SAMPLES + SAMPLES_CARRIED) * sizeof(Sample));
    if (!u->output_buf)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    u->output_buf += SAMPLES_CARRIED * sizeof(Sample);
    u->block_buf = (u8*)malloc(MAX_BLOCK_SIZE * BITS_PER_ADSP_WORD / 8);
    if (!u->block_buf)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
#ifdef INCLUDE_PASSTHRU_DATA
    u->passthru_buf = malloc(BLOCK_SIZE_SAMPLES * sizeof(Sample));
    if (!u->passthru_buf)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
#endif
    return WICED_SUCCESS;
}

void free_unshorten(struct unshorten *u)
{
    IF_DEBUG(printf("free_unshorten: u=%p\n", u));
#ifdef INCLUDE_PASSTHRU_DATA
    free(u->passthru_buf);
#endif
    free(u->block_buf);
    if (u->output_buf)
    {
        free(u->output_buf - SAMPLES_CARRIED * sizeof(Sample));
    }
}

void reset_unshorten(struct unshorten *u)
{
    u->input_data = 0;
    u->input_data_size = 0;
    u->input_index = 0;
    u->curr_block_size = 0;
    u->block_buf_index = 0;
}

wiced_result_t unshorten_supply_input(struct unshorten *u, const void *data, u32 size)
{
    IF_DEBUG(printf("unshorten_supply_input: u=%p data=%p size=%lu\n", u, data, size));
    if (u->input_data)
    {
        return WICED_PENDING;
    }
    if (size > 0)
    {
        u->input_data = (const u8 *)data;
        u->input_data_size = size;
        u->input_index = 0;
    }
    return WICED_SUCCESS;
}

const Sample *unshorten_extract_output(struct unshorten *u)
{
    IF_DEBUG(printf("unshorten_extract_output: u=%p\n", u));
    return (const Sample *)u->output_buf;
}

#ifdef INCLUDE_PASSTHRU_DATA
const Sample *unshorten_extract_passthru(struct unshorten *u)
{
    IF_DEBUG(printf("unshorten_extract_passthru: u=%p\n", u));
    return u->passthru_buf;
}
#endif

/* get the next bit */
static u32 get_next_bit(struct unshorten *u)
{
    u32 bbi = u->block_buf_index++;
    return ((u->block_buf[bbi >> 3] >> (bbi & 7)) & 1);
}

/* get a field of 1-16 bits */
static u32 get_next_field(struct unshorten *u, u32 n)
{
    u32 bbi = u->block_buf_index;
    u32 bbi_byte = bbi >> 3;
    const u8 *s = u->block_buf + bbi_byte;
    u32 bbiE = bbi + n;
    u32 x = s[0];
    u->block_buf_index = bbiE;
    bbi &= 7;
    bbiE -= (bbi_byte << 3);
    if (bbiE > 8)
    {
        x |= (s[1] << 8);
        if (bbiE > 16)
        {
            x |= (s[2] << 16);
        }
    }
    x >>= bbi;
    return (x & ((1u << n) - 1));
}

/* return number of zeros before next 1 */
static u32 get_next_1(struct unshorten *u)
{
    u32 bbi = u->block_buf_index;
    u32 bs = u->curr_block_size;
    const u8 *s = u->block_buf + (bbi >> 3);
    const u8 *sE = u->block_buf + bs;
    u32 z = 0;
    u32 x;
    u32 b = bbi & 7;

    for (; (s < sE) && !(*s >> b); z += (8 - b), b = 0, ++s)
    {
        ;
    }
    if (s < sE)
    {
        x = *s >> b;
        while (!(x & 1))
        {
            x >>= 1, ++z;
        }
        ++bbi; /* skip over terminating 1 */
    }
    u->block_buf_index = bbi + z;
    return z;
}


static int decode_block(struct unshorten *u)
{
    u32 type, hN, order;
    Sample* d = (Sample *)u->output_buf;
    u32 i;
    u32 bits_per_sample = BITS_PER_SAMPLE_V;
    u32 b = 0;
    u->block_buf_index = 0;
    type = get_next_field(u, TYPE_FIELD_BITS);  /* get block type */
    b = type / 5;
    type %= 5;
    bits_per_sample -= b;
    get_next_field(u, LENGTH_FIELD_BITS);       /* block size */
    get_next_field(u, LENGTH_FIELD_BITS);       /* prev block size */
    IF_DEBUG(printf("type=%lu b=%lu\n", type, b));

    if (type == COMPR_RECORD_VERBATIM)
    {
        /* samples are stored uncompressed */
        for (i = 0; i < BLOCK_SIZE_SAMPLES; ++i)
        {
            d[i] = (Sample)(get_next_field(u, bits_per_sample) << b);
        }
        u->curr_block_size = 0;
        u->block_buf_index = 0;
        return UNSHORTEN_OUTPUT_AVAILABLE | BLOCK_SIZE_SAMPLES;
    }

    /* samples are stored compressed, type = order of predictor */
    /* sign bit, low order bits, a run of zeros followed by a 1 */
    order = type & 3;
    hN = get_next_field(u, HN_FIELD_BITS);
    IF_DEBUG(printf("order=%lu hN=%lu\n", order, hN));
#ifdef INCLUDE_PASSTHRU_DATA
    /* get pass-through data */
    for (i = 0; i < BLOCK_SIZE_SAMPLES; ++i)
    {
        u->passthru_buf[i] = (Sample)(get_next_field(u, bits_per_sample) << b);
    }
#endif
    for (i = order; i >= 1; --i)
    {
        int ix = i;
        d[-ix] = (Sample)(get_next_field(u, bits_per_sample) << b);
        IF_DEBUG(printf("carried d[%d]=%d\n", -ix, d[-ix]));
    }
    for (i = 0; i < BLOCK_SIZE_SAMPLES; ++i)
    {
        u32 sign = get_next_bit(u);
        u32 low = get_next_field(u, hN);
        u32 high = get_next_1(u);
        s32 sample = (s32)low;
        sample |= ((s32)high << hN);
        if (sign)
        {
            sample = -sample;
        }
        d[i] = (Sample)(sample << b);
        IF_DEBUG(printf("resid d[%lu]=%d\n", i, d[i]));
    }
    if (order > 0)
    {
        for (i = 0; i < BLOCK_SIZE_SAMPLES; ++i)
        {
            s32 pred = d[i-1];
            if (order == 2)
            {
                s32 delta = d[i-1] - d[i-2];
                pred += delta;
            }
            else if (order == 3)
            {
                s32 delta = d[i-1] - d[i-2];
                delta *= 3;
                delta += d[i-3];
                pred = delta;
            }
            d[i] = (Sample)(pred + d[i]);
            IF_DEBUG(printf("decoded d[%lu]=%d\n", i, d[i]));
        }
    }
    u->curr_block_size = 0;
    u->block_buf_index = 0;
    return UNSHORTEN_OUTPUT_AVAILABLE | i;
}

int unshorten_process(struct unshorten *u)
{
    const u8 *id = u->input_data;
    u8 *bb = u->block_buf;
    int ii = u->input_index;
    int bbi = u->block_buf_index;
    int req, take, ret;
    IF_DEBUG(printf("unshorten_process: u=%p ii=%d bbi=%d\n", u, ii, bbi));

    if (!u->input_data)
    {
        IF_DEBUG(printf("no input data\n"));
        return UNSHORTEN_INPUT_REQUIRED;
    }

    if (!u->curr_block_size)
    {
        /* size not yet determined - must have <3 bytes in block_buf */
        u32 fw;
        int sz;
        for (; bbi < 3 && ii < u->input_data_size; ++bbi, ++ii)
        {
            IF_DEBUG(printf("bb[%d] = id[%d]\n", bbi, ii));
            bb[bbi] = id[ii];
        }
        if (bbi < 3)
        {
            u->input_data = 0;
            u->input_index = ii;
            u->block_buf_index = bbi;
            return UNSHORTEN_INPUT_REQUIRED;
        }
        fw = bb[0] | (bb[1] << 8) | (bb[2] << 16);
        sz = RECORD_SIZE(fw);

        IF_DEBUG(printf("sz = %d (fw=0x%06lx)\n", sz, fw));

        if (sz > MAX_BLOCK_SIZE)
        {
            IF_DEBUG(printf("sz > MAX_BLOCK_SIZE\n"));
            return UNSHORTEN_CORRUPT;
        }

        /* sz is in words - convert to bytes */
        sz *= (BITS_PER_ADSP_WORD / 8);
        u->curr_block_size = sz;
    }
    req = u->curr_block_size - bbi;
    if (req + ii > u->input_data_size)
    {
        /* don't have enough data to decode a block */
        take = u->input_data_size - ii;
    }
    else
    {
        take = req;
    }

    IF_DEBUG(printf("take = %d\n", take));

    if (take < 0)
    {
        IF_DEBUG(printf("take < 0\n"));
        return UNSHORTEN_CORRUPT;
    }

    IF_DEBUG(printf("bb[%d..%d] = id[%d..%d]\n", bbi, bbi + take - 1, ii, ii + take - 1));

    memcpy(bb + bbi, id + ii, take);
    bbi += take;
    ii += take;
    if (ii == u->input_data_size)
    {
        /* all input data now consumed */
        u->input_data = 0;
        u->input_data_size = 0;
        u->input_index = 0;
        IF_DEBUG(printf("input consumed\n"));
    }
    else
    {
        u->input_index = ii;
    }

    if (take == req)
    {
        ret = decode_block(u);
    }
    else
    {
        u->block_buf_index = bbi;
        IF_DEBUG(printf("need more data to decode block\n"));
        ret = UNSHORTEN_INPUT_REQUIRED;
    }

    return ret;
}

///////////////////////////////// END OF FILE //////////////////////////////////
