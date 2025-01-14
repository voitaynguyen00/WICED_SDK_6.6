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
 * Bluetooth Internet Gateway (BIG) HTTP proxy server (HPS) Demo Application
 *
 * HPS server llibrary is an application library that enables a BLE and WiFi/ethernet enabled device to function as a HPS server.
 *
 * To run the app, work through the following steps.
 * Modify Wi-Fi configuration settings CLIENT_AP_SSID and CLIENT_AP_PASSPHRASE in wifi_config_dct.h to match your router settings.
 *
 * To run and test the HPS Server, open up a HPS client (such as LightBlue HPS)
 *
 * Use the HPS client to send HTTP data to the WICED HPS server as BLE GATT packet
 *
 * HPS server shall extract the HTTP data and send it to the destination HTTP server
 *
 *  Build and run this application.
 */

#include "bt_internet_gateway.h"
#include "bt_http_proxy_server.h"
#include "wiced_rtos.h"
#include "wiced_network.h"
#include "wiced_tcpip.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define WICED_HPS_INTERFACE (WICED_STA_INTERFACE)
#define MAX_HPS_CONNECTIONS (3)

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t local_keys_callback        ( big_security_event_t event, wiced_bt_local_identity_keys_t* keys );
static wiced_result_t paired_device_keys_callback( big_security_event_t event, wiced_bt_device_link_keys_t* keys );

/******************************************************
 *               Variable Definitions
 ******************************************************/

hps_connection_t hps_connection_array[ MAX_HPS_CONNECTIONS ];

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    wiced_init();

    bt_internet_gateway_init( local_keys_callback );

    hps_server_start( &hps_connection_array[ 0 ], MAX_HPS_CONNECTIONS, WICED_HPS_INTERFACE, paired_device_keys_callback );
}

static wiced_result_t local_keys_callback( big_security_event_t event, wiced_bt_local_identity_keys_t* keys )
{
    wiced_result_t result = WICED_BT_SUCCESS;

    switch ( event )
    {
        case BIG_UPDATE_SECURITY_KEYS_EVENT:
        {
            WPRINT_APP_INFO( ( "Store local keys in non-volatile memory\n" ) );

            /* Return WICED_BT_SUCCESS to indicate to the stack that keys are successfully stored */
            result = WICED_BT_SUCCESS;
            break;
        }
        case BIG_REQUEST_SECURITY_KEYS_EVENT:
        {
            WPRINT_APP_INFO( ( "Stack requests for local security keys\n" ) );

            /* Return WICED_BT_ERROR to indicate to the stack that keys aren't available. The stack needs to generate them internally */
            result = WICED_BT_ERROR;
            break;
        }
    }

    return result;
}

static wiced_result_t paired_device_keys_callback( big_security_event_t event, wiced_bt_device_link_keys_t* keys )
{
    wiced_result_t result = WICED_BT_SUCCESS;

    switch ( event )
    {
        case BIG_UPDATE_SECURITY_KEYS_EVENT:
        {
            WPRINT_APP_INFO( ( "Store paired device keys in non-volatile memory\n" ) );

            /* Return WICED_BT_SUCCESS to indicate to the stack that keys are successfully stored */
            result = WICED_BT_SUCCESS;
            break;
        }
        case BIG_REQUEST_SECURITY_KEYS_EVENT:
        {
            WPRINT_APP_INFO( ( "Stack requests for paired device security keys\n" ) );

            /* Return WICED_BT_ERROR to indicate to the stack that keys aren't available. The stack needs to generate them internally */
            result = WICED_BT_ERROR;
            break;
        }
    }

    return result;
}
