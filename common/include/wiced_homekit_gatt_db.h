/***************************************************************************//**
* \file <wiced_homekit_db.h>
*
* \brief
* 	The Bluetooth Low Energy HomeKit accessory GATT database.
*
*//*****************************************************************************
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
*******************************************************************************/

#ifndef WICED_HOMEKIT_GATT_DB_H
#define WICED_HOMEKIT_GATT_DB_H

#include "wiced_bt_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
*                    Macros
******************************************************************************/

/* Macro definitions of some general HomeKit GATT database entries */

/* Service Instance ID characteristic */


#define HOMEKIT_GATT_DB_SVC_INSTANCE_ID(char_hdl, val_hdl)                          \
    CHARACTERISTIC_UUID128(char_hdl, val_hdl, UUID_HAP_CHAR_SERVICE_INSTANCE_ID,    \
            LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE)

/* Characteristic Instance ID characteristic descriptor */

#define HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(hdl)   \
    CHAR_DESCRIPTOR_UUID128(hdl, UUID_HAP_DESCR_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE)

/* Client Characteristic Configuration characteristic descriptor */

#define HOMEKIT_GATT_DB_CLIENT_CONFIG(hdl)                                                      \
    CHAR_DESCRIPTOR_UUID16_WRITABLE(hdl, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ)


/*
 * Services
 */

/* Accessory Information */

#define HOMEKIT_GATT_DB_SVC_ACCESSORY_INFO(svc_hdl, inst_id_hdl, inst_id_val_hdl)   \
    PRIMARY_SERVICE_UUID128(svc_hdl, UUID_HAP_SERVICE_ACCESSORY_INFORMATION),       \
        HOMEKIT_GATT_DB_SVC_INSTANCE_ID(inst_id_hdl, inst_id_val_hdl)

/* Protocol Information */

#define HOMEKIT_GATT_DB_SVC_PROTOCOL_INFO(svc_hdl, inst_id_hdl, inst_id_val_hdl)    \
    PRIMARY_SERVICE_UUID128(svc_hdl, UUID_HAP_SERVICE_PROTOCOL_INFORMATION),        \
        HOMEKIT_GATT_DB_SVC_INSTANCE_ID(inst_id_hdl, inst_id_val_hdl)

/* Lightbulb */

#define HOMEKIT_GATT_DB_SVC_LIGHTBULB(svc_hdl, inst_id_hdl, inst_id_val_hdl)        \
    PRIMARY_SERVICE_UUID128(svc_hdl, UUID_HAP_SERVICE_LIGHTBULB),                   \
        HOMEKIT_GATT_DB_SVC_INSTANCE_ID(inst_id_hdl, inst_id_val_hdl)

/* Lock Management */

#define HOMEKIT_GATT_DB_SVC_LOCK_MANAGEMENT(svc_hdl, inst_id_hdl, inst_id_val_hdl)  \
    PRIMARY_SERVICE_UUID128(svc_hdl, UUID_HAP_SERVICE_LOCK_MANAGEMENT),             \
        HOMEKIT_GATT_DB_SVC_INSTANCE_ID(inst_id_hdl, inst_id_val_hdl)

/* Lock Mechanism */

#define HOMEKIT_GATT_DB_SVC_LOCK_MECHANISM(svc_hdl, inst_id_hdl, inst_id_val_hdl)   \
    PRIMARY_SERVICE_UUID128(svc_hdl, UUID_HAP_SERVICE_LOCK_MECHANISM),              \
        HOMEKIT_GATT_DB_SVC_INSTANCE_ID(inst_id_hdl, inst_id_val_hdl)

/* Pairing */

#define HOMEKIT_GATT_DB_SVC_PAIRING(svc_hdl, inst_id_hdl, inst_id_val_hdl)          \
    PRIMARY_SERVICE_UUID128(svc_hdl, UUID_HAP_SERVICE_PAIRING),                     \
        HOMEKIT_GATT_DB_SVC_INSTANCE_ID(inst_id_hdl, inst_id_val_hdl)


/*
 * Characteristics
 */

/* Identify */
#define HOMEKIT_GATT_DB_CHAR_IDENTIFY(char_hdl, val_hdl, inst_id_hdl)                           \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_IDENTIFY,                  \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Manufacturer */
#define HOMEKIT_GATT_DB_CHAR_MANUFACTURER(char_hdl, val_hdl, inst_id_hdl)                       \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_MANUFACTURER,              \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Model */
#define HOMEKIT_GATT_DB_CHAR_MODEL(char_hdl, val_hdl, inst_id_hdl)                              \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_MODEL,                     \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Name */
#define HOMEKIT_GATT_DB_CHAR_NAME(char_hdl, val_hdl, inst_id_hdl)                               \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_NAME,                      \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Serial Number */
#define HOMEKIT_GATT_DB_CHAR_SERIAL_NUMBER(char_hdl, val_hdl, inst_id_hdl)                      \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_SERIAL_NUMBER,             \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Firmware Revision */
#define HOMEKIT_GATT_DB_CHAR_FIRMWARE_REVISION(char_hdl, val_hdl, inst_id_hdl)                  \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_FIRMWARE_REVISION,         \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Version */
#define HOMEKIT_GATT_DB_CHAR_VERSION(char_hdl, val_hdl, inst_id_hdl)                            \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_VERSION,                   \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Service Signature */

#define HOMEKIT_GATT_DB_CHAR_SERVICE_SIGNATURE(char_hdl, val_hdl, inst_id_hdl)                  \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_SERVICE_SIGNATURE,         \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Brightness */

#define HOMEKIT_GATT_DB_CHAR_BRIGHTNESS(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)            \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_BRIGHTNESS,                \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* On */
#define HOMEKIT_GATT_DB_CHAR_ON(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)                    \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_ON,                        \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Hue */
#define HOMEKIT_GATT_DB_CHAR_HUE(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)                   \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_HUE,                       \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Saturation */
#define HOMEKIT_GATT_DB_CHAR_SATURATION(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)            \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_SATURATION,                \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Lock Control Point */
#define HOMEKIT_GATT_DB_CHAR_LOCK_CONTROL_POINT(char_hdl, val_hdl, inst_id_hdl)                 \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_LOCK_MANAGEMENT_CONTROL_POINT, \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH), \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Logs */
#define HOMEKIT_GATT_DB_CHAR_LOGS(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)                  \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_LOGS,                      \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Audio Feedback */
#define HOMEKIT_GATT_DB_CHAR_AUDIO_FEEDBACK(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)        \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_AUDIO_FEEDBACK,            \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Lock Management Auto Security Timeout */
#define HOMEKIT_GATT_DB_CHAR_LOCK_AUTO_SECURE_TIMEOUT(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl) \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_LOCK_MANAGEMENT_AUTO_SECURE_TIMEOUT, \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Administrator Only Access */
#define HOMEKIT_GATT_DB_CHAR_ADMIN_ONLY_ACCESS(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)     \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_ADMINISTRATOR_ONLY_ACCESS, \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Lock Last Known Action */
#define HOMEKIT_GATT_DB_CHAR_LOCK_LAST_KNOWN_ACTION(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl) \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_LOCK_MECHANISM_LAST_KNOWN_ACTION, \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Current Door State */
#define HOMEKIT_GATT_DB_CHAR_CURRENT_DOOR_STATE(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)    \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_DOOR_STATE_CURRENT,        \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Motion Detected */
#define HOMEKIT_GATT_DB_CHAR_MOTION_DETECTED(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)       \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_MOTION_DETECTED,           \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Lock Current State */
#define HOMEKIT_GATT_DB_CHAR_LOCK_CURRENT_STATE(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)    \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_LOCK_MECHANISM_CURRENT_STATE, \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Lock Target State */
#define HOMEKIT_GATT_DB_CHAR_LOCK_TARGET_STATE(char_hdl, val_hdl, inst_id_hdl, cli_cfg_hdl)     \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_LOCK_MECHANISM_TARGET_STATE, \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,   \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl),                                          \
        HOMEKIT_GATT_DB_CLIENT_CONFIG(cli_cfg_hdl)

/* Pair setup */
#define HOMEKIT_GATT_DB_CHAR_PAIR_SETUP(char_hdl, val_hdl, inst_id_hdl)                         \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_PAIRING_PAIR_SETUP,        \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH), \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Pair verify */
#define HOMEKIT_GATT_DB_CHAR_PAIR_VERIFY(char_hdl, val_hdl, inst_id_hdl)                        \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_PAIRING_PAIR_VERIFY,       \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH), \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Pairing features */
#define HOMEKIT_GATT_DB_CHAR_PAIR_FEATURES(char_hdl, val_hdl, inst_id_hdl)                      \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_PAIRING_FEATURES,          \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),   \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

/* Pairing pairings */
#define HOMEKIT_GATT_DB_CHAR_PAIR_PAIRINGS(char_hdl, val_hdl, inst_id_hdl)                      \
    CHARACTERISTIC_UUID128_WRITABLE(char_hdl, val_hdl, UUID_HAP_CHAR_PAIRING_PAIRINGS,          \
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,                                 \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH), \
        HOMEKIT_GATT_DB_CHAR_INSTANCE_ID(inst_id_hdl)

#ifdef __cplusplus
}
#endif

#endif /* _WICED_HOMEKIT_GATT_DB_H_ */
