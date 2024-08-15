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

/** @file
 *
 * Scan for all channels including prohibited channel and update
 * the country code.
 *
 * Features demonstrated
 *  - WICED scan API
 *  - Get and set current country dynamically (without power off/on of WLAN module)
 *
 * This application snippet periodically scans for AP's containing country IE's and
 * picks the strongest, greater than some minimum RSSI, and changes the current
 * country (as long as it is different than the current country).  Each periodic scan will
 * print the strongest AP with a country IE.
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 */


#include <stdlib.h>
#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define DELAY_BETWEEN_AUTOCOUNTRY_UPDATES       ( 5000 )
#define MIN_RSSI                                ( -85  )
#define COUNTRY_CODE_SIZE                       (  2   )

static const uint16_t valid_channel_list_2g5g[] =
{
        /* valid 2g channels */
        1, 2, 3, 4, 5, 6, 7, 8,
        9, 10, 11, 12, 13,

        /* valid 5g_channels */
        36, 40, 44, 48, 52, 56, 60, 64,
        100, 104, 108, 112, 116, 120, 124, 128, 132,
        136, 140, 144, 149, 153, 157, 161, 165
};

/******************************************************
 *                    Structures
 ******************************************************/


/******************************************************
 *               Static Function Declarations
 ******************************************************/
/* Run autocountry update to scan and update ccode if necessary */
static wiced_result_t wifi_autocountry_update( void );

/* scan result callback to process a scan result */
static wiced_result_t wifi_location_scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );

/* Process location scan record */
static wwd_result_t wifi_process_location_scan_record( wiced_scan_result_t* record, wwd_country_t * candidate_ccode);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static int16_t candidate_rssi;
wwd_country_t  candidate_ccode;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_init( );

    while(1)
    {
        wiced_time_t    scan_start_time;
        wiced_time_t    scan_end_time;
        wiced_result_t  result;

        /* Start the scan */
        wiced_time_get_time(&scan_start_time);
        result = wifi_autocountry_update();
        if ( result == WICED_SUCCESS)
        {
            wiced_time_get_time(&scan_end_time);
            WPRINT_APP_INFO( ("Autocountry update complete in %lu milliseconds\n\n", (unsigned long )(scan_end_time - scan_start_time)) );
        }
        else
        {
            /* This Autocountry can fail if the Scan is aborted or if get country code
             * fails
             */
            WPRINT_APP_INFO( ("Autocountry failed! %d\n\n", result) );
        }
        /* Issuing next scan after some delay (optional) */
        wiced_rtos_delay_milliseconds(DELAY_BETWEEN_AUTOCOUNTRY_UPDATES);
    }
}

/*
 * Callback function to handle scan results
 */
wiced_result_t wifi_location_scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{

    if ( malloced_scan_result != NULL )
    {
        wiced_semaphore_t   *scan_sema = ( wiced_semaphore_t * )malloced_scan_result->user_data;
        malloc_transfer_to_curr_thread( malloced_scan_result );

        if ( malloced_scan_result->status == WICED_SCAN_INCOMPLETE )
        {
            wifi_process_location_scan_record(&malloced_scan_result->ap_details, &candidate_ccode);
        }
        else
        {
            wiced_rtos_set_semaphore(scan_sema);
        }

        free( malloced_scan_result );
        malloced_scan_result = NULL;
    }

    return WICED_SUCCESS;
}

static wiced_result_t wifi_autocountry_update( void )
{
    wwd_country_t                  current_ccode;
    wiced_result_t                 retval;
    wiced_semaphore_t              scan_sema; /* Semaphore used for signaling scan complete */
    wiced_scan_type_t              scan_type = WICED_SCAN_TYPE_ACTIVE | WICED_SCAN_TYPE_PROHIBITED_CHANNELS;
    char *ccode                    = (char *)&candidate_ccode;
    int i;

    /* Initialize the semaphore that will tell us when the scan is complete */
    wiced_rtos_init_semaphore(&scan_sema);

    candidate_rssi = MIN_RSSI;
    memset(&candidate_ccode, 0, sizeof(candidate_ccode ));

    /* scan for wifi networks and find the best country */
    retval = wiced_wifi_scan_networks_ex ( wifi_location_scan_result_handler, &scan_sema, \
            scan_type, WICED_BSS_TYPE_ANY, NULL, NULL, valid_channel_list_2g5g, NULL, WICED_STA_INTERFACE  );
    if ( retval == WICED_SUCCESS )
    {
        wiced_rtos_get_semaphore(&scan_sema, WICED_WAIT_FOREVER);
    }
    else
    {
        WPRINT_WICED_INFO( ( "Error starting scan! Error=%d\n", retval ) );
        goto finalize;
    }

    /* No country was found with RSSI greater than MIN_RSSI with country IE.*/
    if (( ccode[0] == '\0') && ( ccode[1] == '\0' ) )
    {
        goto finalize;
    }

    /* check if the country code is between 'A' and 'Z' */
    for (i = 0; i < COUNTRY_CODE_SIZE; i++ )
    {
        if ( (ccode[i] < 'A') || (ccode[i] > 'Z') )
        {
            WPRINT_WICED_INFO( ( "Error Country code is not between 'A' or 'Z' !\n") );
            goto finalize;
        }
    }


    retval = wwd_wifi_get_ccode ( &current_ccode );
    if ( retval != WICED_SUCCESS )
    {
        WPRINT_WICED_INFO( ( "Error getting ccode candidate!\n") );
        goto finalize;
    }

    /* If we found a "best" country then use it if new */
    if  ( memcmp( current_ccode, candidate_ccode, sizeof(candidate_ccode)) != 0 )

    {
        retval = wwd_wifi_set_ccode(&candidate_ccode);
        WPRINT_WICED_INFO( ( "Ccode update to %s\n", candidate_ccode ) );
    }

finalize:
    wiced_rtos_deinit_semaphore(&scan_sema);
    return retval;
}


static wwd_result_t wifi_process_location_scan_record ( wiced_scan_result_t* record, wwd_country_t *ccode )
{
    char country[2] = {'\0', '\0' };

    if ( record->bss_type != WICED_BSS_TYPE_INFRASTRUCTURE )
        return WWD_SUCCESS;

    /* ignore if no ccode present */
    if ( memcmp(record->ccode, country, sizeof(country) ) == 0 )
    {
        return WWD_SUCCESS;
    }

    WPRINT_APP_INFO(("\n===========  rssi: %d ccode: %c%c band: %s ============\n",
                 candidate_rssi,
                 record->ccode[0], record->ccode[1],
                 (record->band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz"));

    /* check the signal strength and if better than previous update it
     * and copy the country code
     */
    if ( record->signal_strength > candidate_rssi )
    {
        candidate_rssi = record->signal_strength;
        memcpy(ccode, record->ccode, sizeof(record->ccode) );
    }

     return WWD_SUCCESS;
}


