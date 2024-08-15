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

////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2014-2015 Cirrus Logic International (UK) Ltd.  All rights reserved.
//
// This software as well as any related documentation is furnished under
// license and may only be used or copied in accordance with the terms of the
// license. The information in this file is furnished for informational use
// only, is subject to change without notice, and should not be construed as
// a commitment by Cirrus Logic International (UK) Ltd.  Cirrus Logic International
// (UK) Ltd assumes no responsibility or liability for any errors or inaccuracies
// that may appear in this document or any software that may be provided in
// association with this document.
//
// Except as permitted by such license, no part of this document may be
// reproduced, stored in a retrieval system, or transmitted in any form or by
// any means without the express written consent of Cirrus Logic International
// (UK) Ltd or affiliated companies.
//
////////////////////////////////////////////////////////////////////////////////

/**
 * @file Parses a WMFW file/blob for loading to Cirrus CS47L24 DSPs
 */

#ifdef CS47L24_ENABLE_DSP_DEBUG_TRACE
#include <unistd.h>
#include <stdio.h>
#endif /* CS47L24_ENABLE_DSP_DEBUG_TRACE */
#include <stdlib.h>
#include <string.h>

#include "wiced_log.h"
#include "wiced_rtos.h"

#include "cs47l24.h"
#include "cs47l24_private.h"
#include "cs47l24_register_map.h"
#include "resources.h"

/******************************************************
 *                      Macros
 ******************************************************/
/******************************************************
 *                    Constants
 ******************************************************/

#define CS47L24_REG_ADDR_LENGTH              (sizeof(uint32_t))
#define CS47L24_PADDING_LENGTH               (sizeof(uint16_t))
#define CS47L24_REG_ADDR_PLUS_PADDING_LENGTH (CS47L24_REG_ADDR_LENGTH + CS47L24_PADDING_LENGTH)

/******************************************************
 *                 Type Definitions
 ******************************************************/
/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WM_MAGIC_ID_UNKNOWN,
    WM_MAGIC_ID_WMFW,
    WM_MAGIC_ID_WMDR,
} wm_magic_id_t;

typedef enum
{
    WMFW_PM                   = 0x02,                ///< ADSPx program memory
    WMFW_DM                   = 0x03,                ///< ADSP1 data memory
    WMFW_XM                   = 0x05,                ///< ADSP2 X coefficient memory
    WMFW_YM                   = 0x06,                ///< ADSP2 Y coefficient memory
    WMFW_ZM                   = 0x04,                ///< ADSP1 & ADSP2 Z coefficient memory
    WMFW_ABS_ADDRESS          = 0xF0,                ///< Absolute addressing for the device
    WMFW_REL_ADDRESS          = 0xF1,                ///< Relative addressing from the start of the first coefficients block
    WMFW_ALGO_INFO_BLOCK      = 0xF2,                ///< Algorithm information data block
    WMFW_COEFF_INFO_SUB_BLOCK = 0xF3,                ///< Coefficient information data sub-block
    WMFW_USER_TEXT            = 0xFE,                ///< User-defined name text
    WMFW_INFO_STRING          = 0xFF,                ///< Informational ASCII string
} wmfw_region_t;

/******************************************************
 *                    Structures
 ******************************************************/
/*
 * Structure defining the header of a WMFW block
 *
 * Format for a block:
 *
 *      31       24 23      16 15       8 7        0
 *  0   +----------+----------+----------+---------+
 *      | type[7:0]|          offset[23:0]         |
 *  4   +----------+-------------------------------+
 *      |                dataLength                |
 *  8   +------------------------------------------+
 *      |                   data                   |
 *      :                   ....                   :
 *      :                                          :
 *
 * The offset/region and data length are little endian.
 * The data is formatted big-endian to facilitate writing straight to the core.
 */
typedef struct
{
    uint32_t offset : 24;
    uint32_t region : 8;
    uint32_t data_length;
} wmfw_block_header_t;


/*
 * Structure defining the header of a WMDR block (version 0x1)
 *
 * Format for a block:
 *
 *      31       24 23      16 15       8 7        0
 *  0   +----------+----------+----------+---------+
 *      |       type[15:0]    |     offset[15:0]   |
 *  4   +----------+-------------------------------+
 *      |        Algorithm Identification          |
 *  8   +------------------------------------------+
 *      |           Algorithm Version              |
 * 12   +------------------------------------------+
 *      |               Sample Rate                |
 * 16   +------------------------------------------+
 *      |                dataLength                |
 * 20   +------------------------------------------+
 *      |                   data                   |
 *      :                   ....                   :
 *      :                                          :
 *
 * The offset/region and data length are little endian.
 * The data is formatted big-endian to facilitate writing straight to the core.
 */
typedef struct
{
    uint32_t offset : 16;
    uint32_t region : 16;
    uint32_t algo_id;
    uint32_t algo_version;
    uint32_t sample_rate;
    uint32_t data_length;
} wmdr_block_header_t;


typedef struct
{
    uint32_t reg;
    uint16_t value;
} cs47l24_dsp_register_value_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

static const char      g_wmfw_id_string[] = "WMFW";
static const char      g_wmdr_id_string[] = "WMDR";
static const uint32_t  g_wmfw_id_length   = 4;

const cs47l24_dsp_core_details_t g_cs47l24_dsp[CS47L24_FIRMWARE_DSP_MAX] =
{
    {
        CS47L24_DSP2_PM_START,       // PM start
        CS47L24_DSP2_XM_START,       // XM start
        CS47L24_DSP2_YM_START,       // YM start
        CS47L24_DSP2_ZM_START,       // ZM start
        CS47L24_DSP2_CONTROL_1,      // DSP2 control 1
        CS47L24_DSP2_CLOCKING_1,     // DSP2 clocking 1
        CS47L24_DSP2_STATUS_1,       // DSP2 status 1
        CS47L24_DSP2_WDMA_CONFIG_1,  // DSP2 WDMA Config 1 (DMA buffer length)
        CS47L24_DSP2_WDMA_CONFIG_2,  // DSP2 WDMA Config 2
        CS47L24_DSP2_RDMA_CONFIG_1,  // DSP2 RDMA Config 1
        CS47L24_SYSCLOCK_73M728_67M7376,
    },
    {
        CS47L24_DSP3_PM_START,       // PM start
        CS47L24_DSP3_XM_START,       // XM start
        CS47L24_DSP3_YM_START,       // YM start
        CS47L24_DSP3_ZM_START,       // ZM start
        CS47L24_DSP3_CONTROL_1,      // DSP3 control 1
        CS47L24_DSP3_CLOCKING_1,     // DSP3 clocking 1
        CS47L24_DSP3_STATUS_1,       // DSP3 status 1
        CS47L24_DSP3_WDMA_CONFIG_1,  // DSP3 WDMA Config 1 (DMA buffer length)
        CS47L24_DSP3_WDMA_CONFIG_2,  // DSP3 WDMA Config 2
        CS47L24_DSP3_RDMA_CONFIG_1,  // DSP3 RDMA Config 1
        CS47L24_SYSCLOCK_147M456_135M4752,
    },
};

/******************************************************
 *            Static Function Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t cs47l24_dsp_core_reset(cs47l24_device_cmn_data_t* cs47l24, cs47l24_dsp_firmware_type_t dsp_type)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Disable DSP core first */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_CORE_ENA_MASK, 0), result, _exit);
    /* Re-enable core */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_CORE_ENA_MASK, CS47L24_DSP_CORE_ENA), result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_core_reset: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_prepare_core_for_upload(cs47l24_device_cmn_data_t* cs47l24, cs47l24_dsp_firmware_type_t dsp_type)
{
    wiced_result_t result     = WICED_SUCCESS;
    uint32_t       loop_count;
    uint16_t       ram_ready;

    /*
     * SYSCLK must be up and running at this point !
     */

    /* Enable DSP core clocking */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_SYS_ENA_MASK, CS47L24_DSP_SYS_ENA), result, _exit);
    /* Make sure DSP memory is disabled */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_MEM_ENA_MASK, 0), result, _exit);
    /* Disable/Re-enable DSP core clocking */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_SYS_ENA_MASK, 0), result, _exit);
    wiced_rtos_delay_milliseconds(1);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_SYS_ENA_MASK, CS47L24_DSP_SYS_ENA), result, _exit);
    /* Disable DSP core first */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dsp_control1, CS47L24_DSP_CORE_ENA_MASK, 0), result, _exit);
    /* Clear out the DMA length. It will be re-initialized by the new firmware */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].dma_config1, CS47L24_DSP_WDMA_BUFFER_LENGTH_MASK, 0), result, _exit);
    /* Disable the DMA channels */
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].wdma_config2, CS47L24_DSP_WDMA_CHANNEL_ENA_MASK, 0), result, _exit);
    WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_type].rdma_config1, CS47L24_DSP_RDMA_CHANNEL_ENA_MASK, 0), result, _exit);
    /*
     * Check for RAM readiness
     */
    WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, g_cs47l24_dsp[dsp_type].dsp_status1, &ram_ready), result, _exit );
    loop_count = 0;
    while (((ram_ready & CS47L24_DSP_RAM_RDY_MASK) == 0) && (loop_count < CS47L24_READ_RETRY_COUNT))
    {
        wiced_rtos_delay_milliseconds(1);
        WICED_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_RAW_STATUS_4, &ram_ready), result, _exit );
        loop_count++;
    }
    CS47L24_CHECK_RETRY_COUNT(loop_count, result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_prepare_core_for_upload: failed !\n");
    }
    return result;
}


static wiced_result_t cs47l24_process_wmfw_header( uint8_t* wmfw_buffer, uint32_t wmfw_buffer_length, uint32_t* wmfw_buffer_index, wm_magic_id_t* magic_id )
{
    wiced_result_t result        = WICED_SUCCESS;
    uint32_t       length;       /* Header length */

    *magic_id = WM_MAGIC_ID_UNKNOWN;

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < g_wmfw_id_length) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: buffer too short to read magic marker !\n");
        result = WICED_ERROR;
        goto _done;
    }

    /* Check for {'W','M','F','W'} or {'W','M','D','R'} magic markers */
    if (memcmp(g_wmfw_id_string, &wmfw_buffer[*wmfw_buffer_index], g_wmfw_id_length) == 0)
    {
        *magic_id = WM_MAGIC_ID_WMFW;
    }
    else if (memcmp(g_wmdr_id_string, &wmfw_buffer[*wmfw_buffer_index], g_wmfw_id_length) == 0)
    {
        *magic_id = WM_MAGIC_ID_WMDR;
    }
    else
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: 'WMFW'/'WMDR' not found at the start of the file [%c%c%c%c]!\n",
                      wmfw_buffer[*wmfw_buffer_index], wmfw_buffer[*wmfw_buffer_index + 1], wmfw_buffer[*wmfw_buffer_index + 2], wmfw_buffer[*wmfw_buffer_index + 3]);
        result = WICED_ERROR;
        goto _done;
    }

    /* Skip magic marker */
    *wmfw_buffer_index += g_wmfw_id_length;

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < sizeof(length)) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: buffer too short to read header length !\n");
        result = WICED_ERROR;
        goto _done;
    }

    /*
     * Read header length
     */

    length = WICED_READ_32(&wmfw_buffer[*wmfw_buffer_index]);
    *wmfw_buffer_index += sizeof(length);

    /*
     * Now skip the rest of the header.
     * This is in bytes and includes the magic and length, so we need to subtract 8.
     */

    length -= (g_wmfw_id_length + sizeof(length));

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < length) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: buffer too short to skip header !\n");
        result = WICED_ERROR;
        goto _done;
    }

    *wmfw_buffer_index += length;

    /* And we're done. */

 _done:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_header: failed !\n");
    }
    return result;
}


/*
 * This function assumes the buffer is pre-formatted for an auto-incrementing
 * write to the registers across the control bus (e.g. I2C or SPI).
 */
static wiced_result_t cs47l24_write_data_to_dsp( cs47l24_device_cmn_data_t* cs47l24, uint8_t* buffer, uint32_t start_address, uint32_t length_in_bytes )
{
    wiced_result_t result      = WICED_SUCCESS;
    uint32_t       i           = 0;
    uint32_t       reg_address = start_address;

    /*
     * Check we have an even number of bytes - i.e. a full number of registers
     */
    if ( length_in_bytes & 1 )
    {
        /*
         * We have an odd number of bytes, so drop the half register.
         */
        wiced_log_msg(WLF_DRIVER, WICED_LOG_WARNING, "cs47l24_write_data_to_dsp: dropping odd byte\n");
        length_in_bytes &= ~1;
    }

    /*
     * We're "making room" to setup an uninterrupted transfer over SPI.
     * The gist of it is that the device will start auto-incrementing the register address
     * after the following sequence: 4-byte of register start address + 2-byte padding + 2-byte of actual data
     * After this initial 8-byte sequence, the device will auto-increment register address with every 2-bytes of data.
     * See the "control interface" section of the datasheet for details.
     * Problem is our contiguous buffer only contains data; no register address and no padding.
     * So, we have to "free up" the first 6 bytes in order to write the register address followed by 2-bytes of padding
     */
    for (i = 0; i < MIN(CS47L24_REG_ADDR_PLUS_PADDING_LENGTH, length_in_bytes); i += 2)
    {
        uint16_t reg_data = 0;

        reg_data = (buffer[i] << 8) | buffer[i+1];
        WICED_VERIFY_GOTO( cs47l24_reg_write(cs47l24, reg_address, reg_data), result, _exit );

        reg_address++;
    }

    if (length_in_bytes <= CS47L24_REG_ADDR_PLUS_PADDING_LENGTH)
    {
        /*
         * That's it; we're done here
         */
        goto _exit;
    }

    /*
     * 1. Write 4-byte register address in network / BE byte order
     * 2. Add 2-byte padding
     * 3. Start transfer
     */
    reg_address = htonl(reg_address);
    memcpy(buffer, &reg_address, sizeof(reg_address));
    memset(&buffer[sizeof(reg_address)], 0, sizeof(uint16_t));

    WICED_VERIFY_GOTO( cs47l24_block_write(cs47l24, buffer, length_in_bytes), result, _exit );

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_write_data_to_dsp: failed !\n");
    }
    return result;
}


/*
 * Processes a block of the WMFW file and writes it to the DSP if appropriate
 */
static wiced_result_t cs47l24_process_next_wmfw_block( cs47l24_device_cmn_data_t* cs47l24, cs47l24_dsp_firmware_type_t dsp_type,
                                                       wm_magic_id_t magic_id, uint8_t* wmfw_buffer, uint32_t wmfw_buffer_length, uint32_t* wmfw_buffer_index )
{
    wiced_result_t      result                = WICED_SUCCESS;
    wmfw_block_header_t block_header_wmfw     = { 0 };
    wmdr_block_header_t block_header_wmdr     = { 0 };
    uint32_t            region_start;
    uint32_t            registers_per_address;
    uint32_t            offset_in_registers;
    uint32_t            start_address;
    uint32_t            region;
    uint32_t            data_length;

    if (magic_id == WM_MAGIC_ID_UNKNOWN)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: buffer content is unknown !\n");
        result = WICED_ERROR;
        goto _done;
    }

    if (*wmfw_buffer_index >= wmfw_buffer_length)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: past buffer boundary !\n");
        result = WICED_ERROR;
        goto _done;
    }

    if ((magic_id == WM_MAGIC_ID_WMFW) && ((wmfw_buffer_length - *wmfw_buffer_index) >= sizeof(block_header_wmfw)))
    {
        memcpy(&block_header_wmfw, &wmfw_buffer[*wmfw_buffer_index], sizeof(block_header_wmfw));
        *wmfw_buffer_index += sizeof(block_header_wmfw);
        region              = block_header_wmfw.region;
        data_length         = block_header_wmfw.data_length;
        offset_in_registers = block_header_wmfw.offset;
    }
    else if ((magic_id == WM_MAGIC_ID_WMDR) && ((wmfw_buffer_length - *wmfw_buffer_index) >= sizeof(block_header_wmdr)))
    {
        memcpy(&block_header_wmdr, &wmfw_buffer[*wmfw_buffer_index], sizeof(block_header_wmdr));
        *wmfw_buffer_index += sizeof(block_header_wmdr);
        region              = block_header_wmdr.region;
        data_length         = block_header_wmdr.data_length;
        offset_in_registers = block_header_wmdr.offset;
    }
    else
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: buffer too short to read WMFW/WMDR block header !\n");
        result = WICED_ERROR;
        goto _done;
    }

    /*
     * Check it's a block we understand
     */
    switch (region)
    {
        case WMFW_PM:
            region_start = g_cs47l24_dsp[dsp_type].pm_start;
            registers_per_address = 3;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: PM region data (%lu bytes @offset %lu)\n",
                          data_length, (offset_in_registers * registers_per_address));
            break;

        case WMFW_XM:
            region_start = g_cs47l24_dsp[dsp_type].xm_start;
            registers_per_address = 2;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: XM region data (%lu bytes @offset %lu)\n",
                          data_length, (offset_in_registers * registers_per_address));
            break;

        case WMFW_YM:
            region_start = g_cs47l24_dsp[dsp_type].ym_start;
            registers_per_address = 2;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: YM region data (%lu bytes @offset %lu)\n",
                          data_length, (offset_in_registers * registers_per_address));
            break;

        case WMFW_ZM:
            region_start = g_cs47l24_dsp[dsp_type].zm_start;
            registers_per_address = 2;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: ZM region data (%lu bytes @offset %lu)\n",
                          data_length, (offset_in_registers * registers_per_address));
            break;

        case WMFW_ABS_ADDRESS:
            if (region == WMFW_ABS_ADDRESS)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Absolute addressing\n");
            }
            /* And fall through */
        case WMFW_REL_ADDRESS:
            if (region == WMFW_REL_ADDRESS)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Relative addressing from first ZM block\n");
            }
            /* And fall through */
        case WMFW_ALGO_INFO_BLOCK:
            if (region == WMFW_ALGO_INFO_BLOCK)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Algorithm information data block (%lu bytes)\n", data_length);
            }
            /* And fall through */
        case WMFW_COEFF_INFO_SUB_BLOCK:
            if (region == WMFW_COEFF_INFO_SUB_BLOCK)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Coefficient information data sub-block\n");
            }
            /* And fall through */
        case WMFW_USER_TEXT:
            if (region == WMFW_USER_TEXT)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: User-defined name text\n");
            }
            /* And fall through */
        case WMFW_INFO_STRING:
            if (region == WMFW_INFO_STRING)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: It's an info string (%lu bytes)\n", data_length);
#ifdef CS47L24_ENABLE_DSP_DEBUG_TRACE
                write(STDOUT_FILENO, &wmfw_buffer[(*wmfw_buffer_index)], block_header.data_length);
                printf("\n");
#endif /* CS47L24_ENABLE_DSP_DEBUG_TRACE */
            }
            /* And fall through */
        default: /* Unknown block ! */
            /*
             * Skip over the rest of the block
             */
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_process_next_wmfw_block: Skipping over %lu bytes to end of 0x%X block\n", data_length, region);
            if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < data_length) )
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: buffer too short to skip WMFW/WMDR block header data !\n");
                result = WICED_ERROR;
            }
            else
            {
                *wmfw_buffer_index += data_length;
            }
            /*
             * And move on
             */
            goto _done;
    }

    if ( (*wmfw_buffer_index >= wmfw_buffer_length) || ((wmfw_buffer_length - *wmfw_buffer_index) < data_length) )
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: buffer too short to consume WMFW/WMDR block header data !\n");
        result = WICED_ERROR;
        goto _done;
    }

    /*
     * Work out where to write data to
     */
    if (magic_id == WM_MAGIC_ID_WMFW)
    {
        /*
         * IMPORTANT: for some reason, the offset field in WMDR files ASSUMES that
         * DSP words will be accessed with multiple (2 or 3) 32-bit registers
         * So, offset field already carries the proper value and doesn't need
         * to be adjusted in regard to DSP word size.
         */
        offset_in_registers *= registers_per_address;
    }
    start_address        = region_start + offset_in_registers;

    /*
     * And write the data.
     */
    WICED_VERIFY_GOTO( cs47l24_write_data_to_dsp(cs47l24, &wmfw_buffer[*wmfw_buffer_index], start_address, data_length), result, _done );

    /*
     * We've finished this block.  Go round for the next one
     */

    *wmfw_buffer_index += data_length;

 _done:
    if (*wmfw_buffer_index >= wmfw_buffer_length)
    {
        result = WICED_PENDING;
    }
    if ((result != WICED_SUCCESS) && (result != WICED_PENDING))
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_next_wmfw_block: failed !\n");
    }
    return result;
}


/*
 * This function reads in a WMFW blob, processes it, and sends it block by block to the DSP
 */
wiced_result_t cs47l24_process_wmfw_blob( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t         result       = WICED_SUCCESS;
    uint8_t*               buffer       = NULL;
    uint32_t               buffer_size  = 0;
    uint32_t               buffer_index = 0;
    uint32_t               dsp_index    = 0;
    uint32_t               fw_index     = 0;
    wm_magic_id_t          magic_id     = WM_MAGIC_ID_UNKNOWN;
    const resource_hnd_t** fw_res       = NULL;

    WICED_VERIFY_GOTO( cs47l24_dsp_scc_init(cs47l24), result, _direct_exit );

    for (dsp_index = 0; dsp_index < CS47L24_FIRMWARE_DSP_MAX; dsp_index++)
    {
        fw_res = cs47l24->dsp->dsp_res_table[dsp_index];

        if (fw_res == NULL)
        {
            continue;
        }

        /* Prepare the core for upload */
        WICED_VERIFY_GOTO( cs47l24_prepare_core_for_upload(cs47l24, dsp_index), result, _exit );

        for (fw_index = 0; fw_res[fw_index] != NULL; fw_index++)
        {
            buffer       = NULL;
            buffer_size  = 0;
            buffer_index = 0;
            magic_id     = WM_MAGIC_ID_UNKNOWN;

            if (resource_get_readonly_buffer(fw_res[fw_index], 0, fw_res[fw_index]->size,
                                             &buffer_size, (const void**)&buffer) != RESOURCE_SUCCESS)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_blob: resource_get_readonly_buffer() failed\n");
                result = WICED_ERROR;
                goto _exit;
            }

            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG0, "cs47l24_process_wmfw_blob: start on %s, blob #%lu...\n",
                          (dsp_index == CS47L24_FIRMWARE_DSP2) ? "DSP2" : "DSP3", (fw_index + 1));

            /* Read and process the header */
            WICED_VERIFY_GOTO( cs47l24_process_wmfw_header(buffer, buffer_size, &buffer_index, &magic_id), result, _exit );

            /* Now process all the blocks in the buffer */
            do
            {
                result = cs47l24_process_next_wmfw_block(cs47l24, dsp_index, magic_id, buffer, buffer_size, &buffer_index);
            } while (result == WICED_SUCCESS);

            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG0, "cs47l24_process_wmfw_blob: %s, blob #%lu done.\n",
                          (dsp_index == CS47L24_FIRMWARE_DSP2) ? "DSP2" : "DSP3", (fw_index + 1));

            resource_free_readonly_buffer(fw_res[fw_index], buffer);
            buffer       = NULL;
            buffer_size  = 0;
            buffer_index = 0;

            if ((result != WICED_SUCCESS) && (result != WICED_PENDING))
            {
                goto _exit;
            }
        }

        /*
         * Now the firmware(s) image(s) is/are downloaded;
         * send any coefficients and parameters ready for starting
         */

        if (dsp_index == CS47L24_FIRMWARE_DSP2)
        {
            WICED_VERIFY_GOTO( cs47l24_dsp_scc_setup_pre(cs47l24), result, _exit );
        }

        /*
         * Set the core going again.
         * This will send a DSP_START to the core and get everything up and running.
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_index].dsp_control1, CS47L24_DSP_CORE_ENA_MASK, CS47L24_DSP_CORE_ENA), result, _exit);
        /*
         * Make sure DSP memory persists across software/hardware resets
         */
        WICED_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_cs47l24_dsp[dsp_index].dsp_control1, CS47L24_DSP_MEM_ENA_MASK, CS47L24_DSP_MEM_ENA), result, _exit);
    }

 _exit:
    if ((fw_res != NULL) && (fw_res[fw_index] != NULL) && (buffer != NULL))
    {
        resource_free_readonly_buffer(fw_res[fw_index], buffer);
        buffer       = NULL;
        buffer_size  = 0;
        buffer_index = 0;
    }

    if (result == WICED_SUCCESS)
    {
        WICED_VERIFY_GOTO( cs47l24_dsp_scc_setup_post(cs47l24), result, _direct_exit );
    }

 _direct_exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_process_wmfw_blob: failed\n");
    }
    return result;
}
