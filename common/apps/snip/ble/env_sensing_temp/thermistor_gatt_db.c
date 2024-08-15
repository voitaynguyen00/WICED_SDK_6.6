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

/*
 * @file thermistor_gatt_db.c
 *
 * @brief
 *  This file implements the functions populating the GATT Database.
 */

#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "thermistor_gatt_db.h"

/*************************************************************************************
** GATT server definitions
*************************************************************************************/

const uint8_t gatt_database[] = // Define GATT database
{
    /* Primary Service 'Generic Attribute' */
    PRIMARY_SERVICE_UUID16 (HDLS_GENERIC_ATTRIBUTE, UUID_SERVICE_GATT),

    /* Primary Service 'Generic Access' */
    PRIMARY_SERVICE_UUID16 (HDLS_GENERIC_ACCESS, UUID_SERVICE_GAP),

        /* Characteristic 'Device Name' */
        CHARACTERISTIC_UUID16 (HDLC_GENERIC_ACCESS_DEVICE_NAME, HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,
            UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

        /* Characteristic 'Appearance' */
        CHARACTERISTIC_UUID16 (HDLC_GENERIC_ACCESS_APPEARANCE, HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,
            UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

    /* Primary Service 'Environmental Sensing' */
    PRIMARY_SERVICE_UUID16 (HDLS_ENVIRONMENTAL_SENSING, UUID_SERVICE_ENVIRONMENTAL_SENSING),

        /* Characteristic 'Temperature' */
        CHARACTERISTIC_UUID16 (HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE, HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE_VALUE,
            UUID_CHARACTERISTIC_TEMPERATURE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,
            LEGATTDB_PERM_READABLE),

            /* Client Characteristic Configuration Descriptor */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_ENVIRONMENTAL_SENSING_TEMPERATURE_CLIENT_CONFIGURATION,
                UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ ),

            /* Environment Sensing Trigger Settings Descriptor */
            CHAR_DESCRIPTOR_UUID16 (HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING,
                UUID_DESCRIPTOR_ENVIRONMENT_SENSING_TRIGGER_SETTING, LEGATTDB_PERM_READABLE),
};
// Length of the GATT database
const uint16_t gatt_database_len = sizeof(gatt_database);

uint8_t thermistor_device_name[]          = { 't', 'h', 'e', 'r', 'm', 'i','s', 't', 'o', 'r' };
uint8_t thermistor_appearance[]           = { BIT16_TO_8(APPEARANCE_GENERIC_THERMOMETER) };
uint8_t thermistor_last_reading[]         = { 0x00, 0x00 };
uint8_t thermistor_client_configuration[] = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};

// Use a fixed time interval between transmissions once per 1 second
#define ENVIROMENTAL_SENSING_TRIGGER_SETTING_USE_FIXED_TIME_INTERVAL    1
uint8_t thermistor_trigger_setting[] = { ENVIROMENTAL_SENSING_TRIGGER_SETTING_USE_FIXED_TIME_INTERVAL, 0x01, 0x00, 0x00 };

/* GATT attribute lookup table                                */
/* (attributes externally referenced by GATT server database) */
wiced_bt_gatt_data_t thermistor_gatt_db_ext_attr_tbl[] =
        {
            /* { attribute handle,                                      length, offset, attribute data } */
            { HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,                        11, 11, thermistor_device_name },
            { HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,                         2, 2, thermistor_appearance },
            { HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE_VALUE,                 2, 2, thermistor_last_reading },
            { HDLD_ENVIRONMENTAL_SENSING_TEMPERATURE_CLIENT_CONFIGURATION,  2, 2, thermistor_client_configuration },
            { HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING,                   4, 4, thermistor_trigger_setting },
        };

const uint16_t thermistor_gatt_db_ext_attr_tbl_size = (sizeof(thermistor_gatt_db_ext_attr_tbl) / sizeof (wiced_bt_gatt_data_t));
