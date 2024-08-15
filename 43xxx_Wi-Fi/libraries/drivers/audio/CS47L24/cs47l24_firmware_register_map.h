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

/** @file CS47L24 firmware register map
 *
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/*
 * SoundClear DSP3 firmware registers
 * IMPORTANT: addresses for these registers WILL likely change with new firmware updates !
 */

#define SC_FF_DUET_REG_LATCH_REQUEST                       0x7FFF
#define SC_FF_DUET_REG_LATCH_COMPLETE                      0x8000

#define SC_FF_DUET_WRITEREGID_1                            0x3A8004
#define SC_FF_DUET_WRITEREGID_0                            0x3A8005
#define SC_FF_DUET_READREGID_1                             0x3A8006
#define SC_FF_DUET_READREGID_0                             0x3A8007

/*
 * Mic Gain
 */
#define SC_FF_DUET_SCPGA_MICINGAINS4_1                     0x3A8008
#define SC_FF_DUET_SCPGA_MICINGAINS4_0                     0x3A8009
#define SC_FF_DUET_SCPGA_MICINGAINS4_FRAC_WIDTH                  16

/*
 * LineOut Gain / Playback gain (Music/Alexa voice playback level)
 */
#define SC_FF_DUET_SCPGA_LINEOUTGAINS4_1                   0x3A800A
#define SC_FF_DUET_SCPGA_LINEOUTGAINS4_0                   0x3A800B
#define SC_FF_DUET_SCPGA_LINEOUTGAINS4_FRAC_WIDTH                16

/*
 * LineIn Gain / Recording gain (Virtual mic recording level)
 */
#define SC_FF_DUET_SCPGA_LINEINGAINS4_1                    0x3A800C
#define SC_FF_DUET_SCPGA_LINEINGAINS4_0                    0x3A800D
#define SC_FF_DUET_SCPGA_LINEINGAINS4_FRAC_WIDTH                 16

/*
 * AEC Reference Bulk Delay (4 ms default)
 */
#define SC_FF_DUET_SCEC1CONFIG_BULKDELAYMSS11_1            0x3A8020
#define SC_FF_DUET_SCEC1CONFIG_BULKDELAYMSS11_0            0x3A8021
#define SC_FF_DUET_SCEC1CONFIG_BULKDELAYMSS11_FRAC_WIDTH          9

/*
 * AEC Mic Bulk Delay (4 ms default)
 */
#define SC_FF_DUET_SCEC1CONFIG_TXBULKDELAYMSS11_1          0x3A8024
#define SC_FF_DUET_SCEC1CONFIG_TXBULKDELAYMSS11_0          0x3A8025
#define SC_FF_DUET_SCEC1CONFIG_TXBULKDELAYMSS11_FRAC_WIDTH        9

/*
 * DTD Threshold
 */
#define SC_FF_DUET_SCVAD_DTDTHRESHS0_1                     0x3A8032
#define SC_FF_DUET_SCVAD_DTDTHRESHS0_0                     0x3A8033
#define SC_FF_DUET_SCVAD_DTDTHRESHS0_FRAC_WIDTH                  20

/*
 * Mic Spacing
 */
#define SC_FF_DUET_SCMMCONFIG_MICSPACINGMMS9_1             0x3A8082
#define SC_FF_DUET_SCMMCONFIG_MICSPACINGMMS9_0             0x3A8083
#define SC_FF_DUET_SCMMCONFIG_MICSPACINGMMS9_FRAC_WIDTH          11

/*
 * SoundClear Enabled  (1=SC, 2=Passthrough)
 */
#define SC_FF_DUET_APPPROCMODE_MODE_1                      0x3A8126
#define SC_FF_DUET_APPPROCMODE_MODE_0                      0x3A8127

/*
 * SoundClear Passthough Gain
 */
#define SC_FF_DUET_TRPGA_TXGAINS4_1                        0x3A8128
#define SC_FF_DUET_TRPGA_TXGAINS4_0                        0x3A8129
#define SC_FF_DUET_TRPGA_TXGAINS4_FRAC_WIDTH                     16


/*
 * Applies to SC_FF_DUET_SCPGA_MICINGAINS4, SC_FF_DUET_SCPGA_LINEOUTGAINS4, SC_FF_DUET_SCPGA_LINEINGAINS4, SC_FF_DUET_TRPGA_TXGAINS4_MIN
 * Min gain is -INF
 */
#define SC_FF_DUET_SCPGA_GAINS4_MIN                        (-64.0)
#define SC_FF_DUET_SCPGA_GAINS4_MAX                        (24.0824)

/*
 * SC_FF_DUET_SCEC1CONFIG_BULKDELAYMSS11, AEC Reference bulk delay
 */
#define SC_FF_DUET_SCEC1CONFIG_BULKDELAYMSS11_MIN          (4.0)
#define SC_FF_DUET_SCEC1CONFIG_BULKDELAYMSS11_MAX          (12.0)
#define SC_FF_DUET_SCEC1CONFIG_BULKDELAYMSS11_DEFAULT      (4.0)

/*
 * SC_FF_DUET_SCEC1CONFIG_TXBULKDELAYMSS11, AEC Mic bulk delay / Mic input delay
 */
#define SC_FF_DUET_SCEC1CONFIG_TXBULKDELAYMSS11_MIN        (0.0)
#define SC_FF_DUET_SCEC1CONFIG_TXBULKDELAYMSS11_MAX        (12.0)
#define SC_FF_DUET_SCEC1CONFIG_TXBULKDELAYMSS11_DEFAULT    (4.0)

/*
 * SC_FF_DUET_SCVAD_DTDTHRESHS, Double Talk Detection Threshold
 */
#define SC_FF_DUET_SCVAD_DTDTHRESHS_MIN                    (0.0)
#define SC_FF_DUET_SCVAD_DTDTHRESHS_MAX                    (1.0)
#define SC_FF_DUET_SCVAD_DTDTHRESHS_DEFAULT                (0.4)

/*
 * SC_FF_DUET_SCMMCONFIG_MICSPACINGMMS9, Mic Spacing
 */
#define SC_FF_DUET_SCMMCONFIG_MICSPACINGMMS9_MIN           (0.0)
#define SC_FF_DUET_SCMMCONFIG_MICSPACINGMMS9_MAX           (512.0)
#define SC_FF_DUET_SCMMCONFIG_MICSPACINGMMS9_DEFAULT       (27.0)

/*
 * SC_FF_DUET_APPPROCMODE_MODE
 */
#define SC_FF_DUET_APPPROCMODE_MODE_ENABLED                (1)
#define SC_FF_DUET_APPPROCMODE_MODE_PASSTHROUGH            (2)
#define SC_FF_DUET_APPPROCMODE_MODE_DEFAULT                SC_FF_DUET_APPPROCMODE_MODE_ENABLED

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
