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

#pragma once

#include "unshorten_defs.h"
#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                     Constants
 ******************************************************/

#define UNSHORTEN_INPUT_REQUIRED    (0x80000000)
#define UNSHORTEN_OUTPUT_AVAILABLE  (0x40000000)
#define UNSHORTEN_CORRUPT           (0x20000000)
#define UNSHORTEN_OUTPUT_SIZE(x)    ((x)&0xffff)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                   Structures
 ******************************************************/

struct unshorten {
    u8 *output_buf;
    const u8 *input_data;
    int input_data_size;
    int input_index;
    u8 *block_buf;
    int curr_block_size;
    int block_buf_index;
#ifdef INCLUDE_PASSTHRU_DATA
    Sample *passthru_buf;
#endif
};

/******************************************************
 *              Variable declarations
 ******************************************************/

/******************************************************
 *             Function declarations
 ******************************************************/

wiced_result_t init_unshorten(struct unshorten *u);
void free_unshorten(struct unshorten *u);
int unshorten_process(struct unshorten *u);
wiced_result_t unshorten_supply_input(struct unshorten *u, const void *data, u32 size);
const Sample *unshorten_extract_output(struct unshorten *u);

#ifdef INCLUDE_PASSTHRU_DATA
extern const Sample *unshorten_extract_passthru(struct unshorten *u);
#endif

void reset_unshorten(struct unshorten *u);

#ifdef __cplusplus
} /* extern "C" */
#endif
