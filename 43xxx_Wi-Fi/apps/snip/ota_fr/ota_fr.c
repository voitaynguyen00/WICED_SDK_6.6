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
 * Over-The-Air (OTA) Upgrade & Factory Reset Application
 *
 * ------------------------------------------------------
 * PLEASE read the following documentation before trying
 * this application!
 * ------------------------------------------------------
 *
 * This application demonstrates how to use the WICED build system with a WICED development
 * board to demonstrate the OTA upgrade and factory reset capability.
 *
 * Features demonstrated
 *  - WICED OTA Upgrade
 *  - Factory Reset process
 *
 * WICED Multi-Application Support:
 * =================================
 * As of WICED-SDK 3.1.1, WICED Application Framework (WAF) supports loading and storing of multiple
 * application binaries in the external serial flash. Up to 8 binaries are supported. The first
 * five binaries are reserved for internal usage and the last three binaries are free for users to use.
 * The binaries are organised as follows:
 *  - Factory reset application (FR_APP)
 *  - DCT backup image (DCT_IMAGE)
 *  - OTA upgrade application (OTA_APP)
 *  - Resources file system (FILESYSTEM_IMAGE)
 *  - WIFI firmware (WIFI_FIRMWARE)
 *  - Application 0 (APP0)
 *  - Application 1 (APP1)
 *  - Application 2 (APP2)
 *
 * OTA Snippet Application:
 * =========================
 * This snippet application demonstrates how to use WICED multi-application support to perform
 * factory reset and OTA upgrade. The following steps assume you have a BCM943362WCD4 WICED evaluation board
 * (a BCM943362WCD4 WICED module on a WICED evaluation board). If your board is different, substitute
 * BCM943362WCD4 for your platform name.
 *
 * Prepare the WICED evaluation board for OTA upgrade
 *     1. Build the snip.ota_fr application to function as your factory reset and OTA
 *        application
 *     2. Notice that the factory reset application (FR_APP) is set in <WICED-SDK>/apps/snip/ota_fr/ota_fr.mk
 *        to point to the snip.ota_fr application elf file.
 *     3. Run the following make target to download your production and factory reset applications
 *        to the board:
 *            make snip.ota_fr-BCM943362WCD4 download download_apps run
 *     4. Build an application that will upgrade the current application.
 *        For this example we will build the snip.scan application:
 *            make snip.scan-BCM943362WCD4
 *
 * Upgrade the application running on the WICED evaluation board
 *   After carefully completing the above steps, the WICED evaluation board is ready to for an OTA upgrade.
 *   'Loading OTA upgrade app' log message is displayed in the terminal when the OTA upgrade application is ready.
 *   Work through the following steps:
 *   - Using the Wi-Fi connection manager on your computer, search for, and connect to,
 *     the Wi-Fi AP called : Wiced_Device
 *   - Open a web browser and enter the IP address of the eval board: 192.168.10.1 (default)
 *   - After a short period, the WICED Webserver OTA Upgrade webpage appears
 *   - Click 'Choose File' and navigate to the file
 *     <WICED-SDK>/build/snip_scan-BCM943362WCD4/Binary/snip_scan-BCM943362WCD4.stripped.elf
 *     (this is the snip.scan application binary file that was created in step 2 above)
 *   - Click 'Open' (the dialogue box disappears)
 *   - Click 'Start upgrade' to begin the upgrade process
 *      - The progress bar within the webpage indicates the upgrade progress
 *      - The webpage displays 'Transfer completed, WICED device is rebooting now' when the
 *        process completes
 *   - With the upgrade complete, the snip.scan application runs and Wi-Fi scan results are regularly
 *     printed to the terminal
 *
 * Perform factory reset on the WICED evaluation board
 *   To perform factory reset on the WICED evaluation board, work through the following steps:
 *   - Push and hold the SW1 button THEN momentarily press and release the Reset button.
 *     The D1 LED flashes quickly to indicate factory reset will occur *IF* SW1 is held
 *     for a further 5 seconds. Continue to hold SW1.
 *   - After the copy process is complete, the WICED evaluation board reboots and runs the factory
 *     reset (OTA_FR) application. Observe the log messages at the terminal to confirm the factory reset
 *     is completed successfully.
 *
 */

#include "wiced.h"
#include "wwd_debug.h"
#include "wiced_framework.h"
#include "wiced_ota_server.h"

/******************************************************
 *                      Macros
 ******************************************************/

#ifndef GPIO_LED_NOT_SUPPORTED
#ifndef WICED_LED2
#define WICED_LED2  WICED_LED1
#endif
#endif

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
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_config_soft_ap_t* soft_ap;
    wiced_result_t          result;
#ifndef GPIO_LED_NOT_SUPPORTED
    uint32_t i;
#endif
    wiced_init( );

    WPRINT_APP_INFO( ( "Hi, I'm the Production App (ota_fr).\r\n" ) );

#ifndef GPIO_LED_NOT_SUPPORTED
    WPRINT_APP_INFO( ( "Watch while I toggle some LEDs ...\r\n" ) );

    for ( i = 0; i < 15; i++ )
    {
        wiced_gpio_output_high( WICED_LED1 );
        wiced_gpio_output_low( WICED_LED2 );
        wiced_rtos_delay_milliseconds( 300 );
        wiced_gpio_output_low( WICED_LED1 );
        wiced_gpio_output_high( WICED_LED2 );
        wiced_rtos_delay_milliseconds( 300 );
    }
#endif

    WPRINT_APP_INFO( ( "Time for an upgrade. OTA upgrade starting ...\r\n" ) );

    /* Get the SoftAP name and output to console */
    result = wiced_dct_read_lock( (void**) &soft_ap, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t, soft_ap_settings), sizeof(wiced_config_soft_ap_t) );
    if ( result == WICED_SUCCESS )
    {
        char ssid_name[65] = { 0 };
        memcpy(ssid_name, soft_ap->SSID.value, soft_ap->SSID.length);
        WPRINT_APP_INFO( ( "\r\nSoftAP start, AP name: %s\r\n", ssid_name));
        memset(ssid_name, 0x00, sizeof(ssid_name));
//        memcpy(ssid_name, soft_ap->security_key, soft_ap->security_key_length);
//        WPRINT_APP_INFO( ( "           passphrase: %s\r\n\r\n", ssid_name));
        wiced_dct_read_unlock( soft_ap, WICED_FALSE );
    }

    /* Bringup the network interface */
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );
    wiced_ota_server_start( WICED_AP_INTERFACE );
    while ( 1 )
    {
        wiced_rtos_delay_milliseconds( 100 );
    }
}



