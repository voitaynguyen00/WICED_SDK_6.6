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
 * Definitions for constants used in the device's GATT database and function
 * prototypes.
 */

#ifndef __GATT_DATABASE_H__
#define __GATT_DATABASE_H__


// ***** Primary Service 'Generic Attribute'
#define HDLS_GENERIC_ATTRIBUTE                                                 0x0001

// ***** Primary Service 'Generic Access'
#define HDLS_GENERIC_ACCESS                                                    0x0014
// ----- Characteristic 'Device Name'
#define HDLC_GENERIC_ACCESS_DEVICE_NAME                                        0x0015
#define HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE                                  0x0016
// ----- Characteristic 'Appearance'
#define HDLC_GENERIC_ACCESS_APPEARANCE                                         0x0017
#define HDLC_GENERIC_ACCESS_APPEARANCE_VALUE                                   0x0018

// ***** Primary Service 'Device Information'
#define HDLS_DEVICE_INFORMATION                                                0x0080
// ----- Characteristic 'PnP ID'
#define HDLC_DEVICE_INFORMATION_PNP_ID                                         0x0081
#define HDLC_DEVICE_INFORMATION_PNP_ID_VALUE                                   0x0082
// ----- Characteristic 'Manufacturer Name String'
#define HDLC_DEVICE_INFORMATION_MANUFACTURER_NAME_STRING                       0x0083
#define HDLC_DEVICE_INFORMATION_MANUFACTURER_NAME_STRING_VALUE                 0x0084
// ----- Characteristic 'Model Number String'
#define HDLC_DEVICE_INFORMATION_MODEL_NUMBER_STRING                            0x0085
#define HDLC_DEVICE_INFORMATION_MODEL_NUMBER_STRING_VALUE                      0x0086
// ----- Characteristic 'Firmware Revision String'
#define HDLC_DEVICE_INFORMATION_FIRMWARE_REVISION_STRING                       0x0087
#define HDLC_DEVICE_INFORMATION_FIRMWARE_REVISION_STRING_VALUE                 0x0088
// ----- Characteristic 'Software Revision String'
#define HDLC_DEVICE_INFORMATION_SOFTWARE_REVISION_STRING                       0x0089
#define HDLC_DEVICE_INFORMATION_SOFTWARE_REVISION_STRING_VALUE                 0x008A

// External Lookup Table Entry
typedef struct
{
    uint16_t handle;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t  *p_data;
} gatt_db_lookup_table_t;

// External definitions

extern const uint8_t            gatt_server_db[];
extern const uint16_t           gatt_server_db_len;

extern const uint8_t            gatt_server_db_appearance[];
extern const uint16_t           gatt_server_db_appearance_len;

extern const uint8_t            gatt_server_db_device_name[];
extern const uint16_t           gatt_server_db_device_name_len;

extern const uint16_t           ams_gatt_db_ext_attr_tbl_size;
extern gatt_db_lookup_table_t   ams_gatt_db_ext_attr_tbl[];

#endif /* __GATT_DATABASE_H__ */
