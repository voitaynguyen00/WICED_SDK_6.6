/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * Battery Service Sample Application (GATT Server database)
 *
 */
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "gatt_server_db.h"
#include "wiced_bt_uuid.h"

/* GATT database */
const uint8_t gatt_db[] =
{
    // Generic Attribute service
    PRIMARY_SERVICE_UUID16 (HANDLE_BS_GATT_SERVICE, UUID_SERVICE_GATT),

    // Generic Access service
    PRIMARY_SERVICE_UUID16 (HANDLE_BS_GAP_SERVICE, UUID_SERVICE_GAP),

    CHARACTERISTIC_UUID16 (HANDLE_BS_GAP_SERVICE_CHAR_DEV_NAME,
                           HANDLE_BS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                           UUID_CHARACTERISTIC_DEVICE_NAME,
                               LEGATTDB_CHAR_PROP_READ,
                               LEGATTDB_PERM_READABLE),

    CHARACTERISTIC_UUID16 (HANDLE_BS_GAP_SERVICE_CHAR_DEV_APPEARANCE,
                           HANDLE_BS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                           UUID_CHARACTERISTIC_APPEARANCE,
                           LEGATTDB_CHAR_PROP_READ,
                           LEGATTDB_PERM_READABLE),

   /* Declare Battery service */
    PRIMARY_SERVICE_UUID16( HANDLE_BATTERY_SERVICE, UUID_SERVICE_BATTERY ),

    CHARACTERISTIC_UUID16(HANDLE_BATTERY_SERVICE_CHAR_LEVEL,
                          HANDLE_BATTERY_SERVICE_CHAR_LEVEL_VAL,
                          UUID_CHARACTERISTIC_BATTERY_LEVEL,
                          LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
                          LEGATTDB_PERM_READABLE),

  /* Declare client characteristic configuration descriptor
   * Value of the descriptor can be modified by the client
   * Value modified shall be retained during connection and across connection
   * for bonded devices.  Setting value to 1 tells this application to send notification
   * when value of the characteristic changes.  Value 2 is to allow indications. */

    CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_BATERY_SERVICE_BATTERY_LEVEL_CHAR_CFG_DESC,
                          UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                           LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE),

};

const uint16_t gatt_db_size = sizeof(gatt_db);
