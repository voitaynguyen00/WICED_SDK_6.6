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
 * SNTP internal structures and definitions.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define NTP_PORT                        (123)
#define NTP_EPOCH                       (86400U * (365U * 70U + 17U))

/* NTP protocol header values */
/* see: RFC 5905  */

/* LI: leap indicator */
#define NTP_LEAP_INDICATOR_NO_LEAP_SECOND_ADJ       (0)
#define NTP_LEAP_INDICATOR_LAST_MINUTE_61           (1)
#define NTP_LEAP_INDICATOR_LAST_MINUTE_59           (2)
#define NTP_LEAP_INDICATOR_CLOCK_UNSYNCRONIZED      (3)

/* VN: version number */
#define NTP_VERSION_NUM_V3                          (3)
#define NTP_VERSION_NUM_V4                          (4)

/* MODE */
#define NTP_MODE_SYMMETRIC_ACTIVE                   (1)
#define NTP_MODE_SYMMETRIC_PASSIVE                  (2)
#define NTP_MODE_CLIENT                             (3)
#define NTP_MODE_SERVER                             (4)
#define NTP_MODE_BROADCAST                          (5)
#define NTP_MODE_CONTROL                            (6)
#define NTP_MODE_RESERVED                           (7)

/* STRATUM */
#define NTP_STRATUM_UNSPECIFIED_OR_INVALID          (0)
#define NTP_STRATUM_PRIMARY                         (1)
#define NTP_STRATUM_SECONDARY_MIN                   (2)
#define NTP_STRATUM_SECONDARY_MAX                  (15)
#define NTP_STRATUM_UNSYNCRONIZED                  (16)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/*
 * Taken from RFC 1305
 * http://www.ietf.org/rfc/rfc1305.txt
 */
typedef struct
{
    unsigned int mode : 3;
    unsigned int vn   : 3;
    unsigned int li   : 2;
    uint8_t      stratum;
    int8_t       poll;
    uint8_t      precision;
    uint32_t     root_delay;
    uint32_t     root_dispersion;
    uint32_t     reference_identifier;
    uint32_t     reference_timestamp_seconds;
    uint32_t     reference_timestamp_fraction;
    uint32_t     originate_timestamp_seconds;
    uint32_t     originate_timestamp_fraction;
    uint32_t     receive_timestamp_seconds;
    uint32_t     receive_timestamp_fraction;
    uint32_t     transmit_timestamp_seconds;
    uint32_t     transmit_timestamp_fraction;
} ntp_packet_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


#ifdef __cplusplus
} /*extern "C" */
#endif
