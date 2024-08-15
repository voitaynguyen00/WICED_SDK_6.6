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
 * @file thermistor_gatt_db.h
 *
 * @brief
 *
 *   This file defines the attributes of the GATT Database.
 *
 */

#ifndef __GATT_DATABASE_H__
#define __GATT_DATABASE_H__

#include <stdint.h>
#include "wiced_bt_gatt.h"

// ***** Primary Service 'Generic Attribute'
#define HDLS_GENERIC_ATTRIBUTE                                      0x0001

// ***** Primary Service 'Generic Access'
#define HDLS_GENERIC_ACCESS                                         0x0014
// ----- Characteristic 'Device Name'
#define HDLC_GENERIC_ACCESS_DEVICE_NAME                             0x0015
#define HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE                       0x0016
// ----- Characteristic 'Appearance'
#define HDLC_GENERIC_ACCESS_APPEARANCE                              0x0017
#define HDLC_GENERIC_ACCESS_APPEARANCE_VALUE                        0x0018

// ***** Primary Service 'Environmental Sensing'
#define HDLS_ENVIRONMENTAL_SENSING                                  0x0028
// ----- Characteristic 'Temperature'
#define HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE                      0x0029
#define HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE_VALUE                0x002A
// Client Configuration Descriptors
#define HDLD_ENVIRONMENTAL_SENSING_TEMPERATURE_CLIENT_CONFIGURATION 0x002B
#define HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING                  0x002C

// External definitions
extern const uint8_t  gatt_database[];
extern const uint16_t gatt_database_len;
extern wiced_bt_gatt_data_t thermistor_gatt_db_ext_attr_tbl[];
extern const uint16_t thermistor_gatt_db_ext_attr_tbl_size;
extern uint8_t thermistor_device_name[];
extern uint8_t thermistor_appearance[];
extern uint8_t thermistor_last_reading[2];
extern uint8_t thermistor_client_configuration[];
extern uint8_t BT_LOCAL_NAME[];
extern const uint16_t BT_LOCAL_NAME_CAPACITY;

#endif /* __GATT_DATABASE_H__ */
