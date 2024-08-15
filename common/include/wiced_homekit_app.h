/***************************************************************************//**
* \file <wiced_homekit_app.h>
*
* \brief
* 	Bluetooth Low Energy HomeKit accessory application library
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

#ifndef WICED_HOMEKIT_APP_H
#define WICED_HOMEKIT_APP_H

#include "wiced_bt_uuid.h"
#include "wiced_homekit.h"
#include "wiced_homekit_gatt_db.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
*                    Constants
******************************************************************************/

/* HomeKit GATT database handles */
enum
{
    HDLS_GATT                                       = 0x01,

    HDLS_GAP                                        = 0x14,
    HDLC_GAP_DEVICE_NAME,
    HDLC_GAP_DEVICE_NAME_VALUE,
    HDLC_GAP_APPEARANCE,
    HDLC_GAP_APPEARANCE_NAME_VALUE,

#ifndef MESH_HOMEKIT_COMBO_APP
    HDLS_ACCESSORY_INFO                             = 0x28,
#else
    HDLS_ACCESSORY_INFO                             = 0x80,
#endif
    HDLC_ACCESSORY_INFO_INSTANCE_ID,
    HDLC_ACCESSORY_INFO_INSTANCE_ID_VALUE,
    HDLC_ACCESSORY_INFO_IDENTIFY,
    HDLC_ACCESSORY_INFO_IDENTIFY_VALUE,
    HDLD_ACCESSORY_INFO_IDENTIFY_INSTANCE_ID,
    HDLC_ACCESSORY_INFO_MANUFACTURER,
    HDLC_ACCESSORY_INFO_MANUFACTURER_VALUE,
    HDLD_ACCESSORY_INFO_MANUFACTURER_INSTANCE_ID,
    HDLC_ACCESSORY_INFO_MODEL,
    HDLC_ACCESSORY_INFO_MODEL_VALUE,
    HDLD_ACCESSORY_INFO_MODEL_INSTANCE_ID,
    HDLC_ACCESSORY_INFO_NAME,
    HDLC_ACCESSORY_INFO_NAME_VALUE,
    HDLD_ACCESSORY_INFO_NAME_INSTANCE_ID,
    HDLC_ACCESSORY_INFO_SERIAL_NUMBER,
    HDLC_ACCESSORY_INFO_SERIAL_NUMBER_VALUE,
    HDLD_ACCESSORY_INFO_SERIAL_NUMBER_INSTANCE_ID,
    HDLC_ACCESSORY_INFO_FIRMWARE_REVISION,
    HDLC_ACCESSORY_INFO_FIRMWARE_REVISION_VALUE,
    HDLD_ACCESSORY_INFO_FIRMWARE_REVISION_INSTANCE_ID,

#ifndef MESH_HOMEKIT_COMBO_APP
    HDLS_PROTOCOL_INFO                              = 0x40,
#else
    HDLS_PROTOCOL_INFO                              = 0xA0,
#endif
    HDLC_PROTOCOL_INFO_INSTANCE_ID,
    HDLC_PROTOCOL_INFO_INSTANCE_ID_VALUE,
    HDLC_PROTOCOL_INFO_SERVICE_SIGNATURE,
    HDLC_PROTOCOL_INFO_SERVICE_SIGNATURE_VALUE,
    HDLD_PROTOCOL_INFO_SERVICE_SIGNATURE_INSTANCE_ID,
    HDLC_PROTOCOL_INFO_VERSION,
    HDLC_PROTOCOL_INFO_VERSION_VALUE,
    HDLD_PROTOCOL_INFO_VERSION_INSTANCE_ID,

/* Application specific characteristics use handles 0x50 - 0xEF */
};


/*  Pre-definitions of GATT/GAP services */

#define HOMEKIT_GATT_DB_GATT_SERVICE    \
    /* Primary service GATT */          \
    PRIMARY_SERVICE_UUID16(HDLS_GATT, UUID_SERVICE_GATT)

#define HOMEKIT_GATT_DB_GAP_SERVICE     \
    /* Primary service GAP */           \
    PRIMARY_SERVICE_UUID16(HDLS_GAP, UUID_SERVICE_GAP),     \
        /* Characteristic 'Device Name' */          \
        CHARACTERISTIC_UUID16(HDLC_GAP_DEVICE_NAME, HDLC_GAP_DEVICE_NAME_VALUE, GATT_UUID_GAP_DEVICE_NAME,  \
                LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE),                \
        /* Characteristic 'Appearance' */           \
        CHARACTERISTIC_UUID16(HDLC_GAP_APPEARANCE, HDLC_GAP_APPEARANCE_NAME_VALUE, GATT_UUID_GAP_ICON,      \
                LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE)

/*
 *  Pre-definition of HomeKit Accessory Information service
 */

#define HOMEKIT_GATT_DB_ACCESSORY_INFO_SERVICE      \
    /* Primary service 'Accessory Information' */   \
    HOMEKIT_GATT_DB_SVC_ACCESSORY_INFO(HDLS_ACCESSORY_INFO, HDLC_ACCESSORY_INFO_INSTANCE_ID, HDLC_ACCESSORY_INFO_INSTANCE_ID_VALUE), \
        /* characteristic 'Identify' */             \
        HOMEKIT_GATT_DB_CHAR_IDENTIFY(HDLC_ACCESSORY_INFO_IDENTIFY, HDLC_ACCESSORY_INFO_IDENTIFY_VALUE, HDLD_ACCESSORY_INFO_IDENTIFY_INSTANCE_ID), \
        /* characteristic 'Manufacturer */          \
        HOMEKIT_GATT_DB_CHAR_MANUFACTURER(HDLC_ACCESSORY_INFO_MANUFACTURER, HDLC_ACCESSORY_INFO_MANUFACTURER_VALUE, HDLD_ACCESSORY_INFO_MANUFACTURER_INSTANCE_ID), \
        /* characteristic 'Model' */                \
        HOMEKIT_GATT_DB_CHAR_MODEL(HDLC_ACCESSORY_INFO_MODEL, HDLC_ACCESSORY_INFO_MODEL_VALUE, HDLD_ACCESSORY_INFO_MODEL_INSTANCE_ID), \
        /* characteristic 'Name' */                 \
        HOMEKIT_GATT_DB_CHAR_NAME(HDLC_ACCESSORY_INFO_NAME, HDLC_ACCESSORY_INFO_NAME_VALUE, HDLD_ACCESSORY_INFO_NAME_INSTANCE_ID), \
        /* characteristic 'Serial Number' */        \
        HOMEKIT_GATT_DB_CHAR_SERIAL_NUMBER(HDLC_ACCESSORY_INFO_SERIAL_NUMBER, HDLC_ACCESSORY_INFO_SERIAL_NUMBER_VALUE, HDLD_ACCESSORY_INFO_SERIAL_NUMBER_INSTANCE_ID), \
        /* characteristic 'Firmware Revision' */    \
        HOMEKIT_GATT_DB_CHAR_FIRMWARE_REVISION(HDLC_ACCESSORY_INFO_FIRMWARE_REVISION, HDLC_ACCESSORY_INFO_FIRMWARE_REVISION_VALUE, HDLD_ACCESSORY_INFO_FIRMWARE_REVISION_INSTANCE_ID)

/*
 *  Pre-definition of HomeKit Protocol Information service
 */

#define HOMEKIT_GATT_DB_PROTOCOL_INFO_SERVICE       \
    /* Primary service 'Protocol Information' */    \
    HOMEKIT_GATT_DB_SVC_PROTOCOL_INFO(HDLS_PROTOCOL_INFO, HDLC_PROTOCOL_INFO_INSTANCE_ID, HDLC_PROTOCOL_INFO_INSTANCE_ID_VALUE), \
        /* characteristic 'service signature' */    \
        HOMEKIT_GATT_DB_CHAR_SERVICE_SIGNATURE(HDLC_PROTOCOL_INFO_SERVICE_SIGNATURE, HDLC_PROTOCOL_INFO_SERVICE_SIGNATURE_VALUE, HDLD_PROTOCOL_INFO_SERVICE_SIGNATURE_INSTANCE_ID), \
        /* characteristic 'Version' */              \
        HOMEKIT_GATT_DB_CHAR_VERSION(HDLC_PROTOCOL_INFO_VERSION, HDLC_PROTOCOL_INFO_VERSION_VALUE, HDLD_PROTOCOL_INFO_VERSION_INSTANCE_ID)

/*
 *  Pre-definition of HomeKit Pairing service
 */

#define HOMEKIT_GATT_DB_PAIRING_SERVICE             \
    /* Primary service 'Pairing' */                 \
    HOMEKIT_GATT_DB_SVC_PAIRING(HDLS_PAIRING, HDLC_PAIRING_SERVICE_INSTANCE, HDLC_PAIRING_SERVICE_INSTANCE_VALUE), \
        /* characteristic 'pair_setup' */           \
        HOMEKIT_GATT_DB_CHAR_PAIR_SETUP(HDLC_PAIRING_PAIR_SETUP, HDLC_PAIRING_PAIR_SETUP_VALUE, HDLD_PAIRING_PAIR_SETUP_INSTANCE_ID), \
        /* characteristic 'pair_verify' */          \
        HOMEKIT_GATT_DB_CHAR_PAIR_VERIFY(HDLC_PAIRING_PAIR_VERIFY, HDLC_PAIRING_PAIR_VERIFY_VALUE, HDLD_PAIRING_PAIR_VERIFY_INSTANCE_ID), \
        /* characteristic 'features' */             \
        HOMEKIT_GATT_DB_CHAR_PAIR_FEATURES(HDLC_PAIRING_FEATURES, HDLC_PAIRING_FEATURES_VALUE, HDLD_PAIRING_FEATURES_INSTANCE_ID), \
        /* characteristic 'pairings' */             \
        HOMEKIT_GATT_DB_CHAR_PAIR_PAIRINGS(HDLC_PAIRING_MANAGE, HDLC_PAIRING_MANAGE_VALUE, HDLD_PAIRING_MANAGE_INSTANCE_ID)

/*
 *  Pre-definition of WICED OTA service (note: this is not a HAP service)
 */

#define HOMEKIT_GATT_DB_WICED_OTA_SERVICE           \
    /* Cypress proprietary OTA Firmware Upgrade Service */      \
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE),    \
        /* characteristic WS Control Point */       \
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,    \
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,   \
                LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH),    \
        /* client characteristic configuration descriptor */    \
        CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,              \
            UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ), \
        /* characteristic WS Data */                \
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, HANDLE_OTA_FW_UPGRADE_DATA,      \
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_WRITE,                              \
            LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH),        \
        /* characteristic Application Info */       \
        CHARACTERISTIC_UUID128(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_APP_INFO, HANDLE_OTA_FW_UPGRADE_APP_INFO,       \
            UUID_OTA_FW_UPGRADE_SERVICE_CHARACTERISTIC_APP_INFO, LEGATTDB_CHAR_PROP_READ,                   \
            LEGATTDB_PERM_READABLE)

/*
 *  Pre-definition of HAP OTA service
 */

#define HOMEKIT_GATT_DB_HAP_OTA_SERVICE             \
    /* HAP OTA Firmware Upgrade Service */          \
    PRIMARY_SERVICE_UUID128(HDLS_OTA_FW_UPGRADE_SERVICE, UUID_HAP_FW_UPDATE_SERVICE),   \
        /* Characteristic 'Service Instance ID' */  \
        CHARACTERISTIC_UUID128(HDLC_OTA_FW_UPGRADE_SERVICE_INSTANCE, HDLC_OTA_FW_UPGRADE_SERVICE_INSTANCE_VALUE,    \
            UUID_HAP_CHAR_SERVICE_INSTANCE_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),       \
        /* characteristic service signature */      \
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE, HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE_VALUE, \
            UUID_HAP_CHAR_SERVICE_SIGNATURE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,       \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),               \
            /* Instance ID */                       \
            CHAR_DESCRIPTOR_UUID128(HDLD_OTA_FW_UPGRADE_SERVICE_SIGNATURE_INSTANCE_ID,  \
                UUID_HAP_DESCR_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),           \
        /* characteristic WS Control Point */       \
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_OTA_FW_UPGRADE_CONTROL_POINT, HDLC_OTA_FW_UPGRADE_CONTROL_POINT_VALUE, \
            UUID_HAP_FW_UPDATE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,  \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH),   \
            /* Instance ID */                       \
            CHAR_DESCRIPTOR_UUID128(HDLD_OTA_FW_UPGRADE_CONTROL_POINT_INSTANCE_ID,      \
                UUID_HAP_DESCR_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),           \
            /* client characteristic configuration descriptor */    \
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_OTA_FW_UPGRADE_CONTROL_POINT_CLNT_CHAR_CFG,                \
                UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ), \
        /* characteristic WS Data */                \
        /* This characteristic is used to send next portion of the FW Similar to the control point */       \
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_OTA_FW_UPGRADE_DATA, HDLC_OTA_FW_UPGRADE_DATA_VALUE,           \
            UUID_HAP_FW_UPDATE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,     \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH),   \
            /* Instance ID */                       \
            CHAR_DESCRIPTOR_UUID128(HDLD_OTA_FW_UPGRADE_DATA_INSTANCE_ID,               \
                UUID_HAP_DESCR_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),           \
        /* characteristic Application Info */       \
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_OTA_FW_UPGRADE_APP_INFO, HDLC_OTA_FW_UPGRADE_APP_INFO_VALUE,   \
            UUID_HAP_FW_UPDATE_SERVICE_CHARACTERISTIC_APP_INFO, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, \
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),               \
            /* Instance ID */                       \
            CHAR_DESCRIPTOR_UUID128(HDLD_OTA_FW_UPGRADE_APP_INFO_INSTANCE_ID,           \
                UUID_HAP_DESCR_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE)


/*
 *  Pre-definitions of HAP characteristic parameters entries
 */

/* Accessory information service characteristics */
#define HOMEKIT_ACCESSORY_INFO_SERVICE_CHARACTERISTICS      \
    { HDLC_ACCESSORY_INFO_IDENTIFY_VALUE, HDLD_ACCESSORY_INFO_IDENTIFY_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_WRITE,                          \
      HAP_CHARACTERISTIC_FORMAT_BOOL, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },   \
    { HDLC_ACCESSORY_INFO_MANUFACTURER_VALUE, HDLD_ACCESSORY_INFO_MANUFACTURER_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,            \
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 }, \
    { HDLC_ACCESSORY_INFO_MODEL_VALUE, HDLD_ACCESSORY_INFO_MODEL_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,                          \
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 }, \
    { HDLC_ACCESSORY_INFO_NAME_VALUE, HDLD_ACCESSORY_INFO_NAME_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,                            \
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 }, \
    { HDLC_ACCESSORY_INFO_SERIAL_NUMBER_VALUE, HDLD_ACCESSORY_INFO_SERIAL_NUMBER_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,          \
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 }, \
    { HDLC_ACCESSORY_INFO_FIRMWARE_REVISION_VALUE, HDLD_ACCESSORY_INFO_FIRMWARE_REVISION_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,  \
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 }

/* Protocol information service characteristics */
#define HOMEKIT_PROTOCOL_INFO_SERVICE_CHARACTERISTICS       \
    { HDLC_PROTOCOL_INFO_SERVICE_SIGNATURE_VALUE, HDLD_PROTOCOL_INFO_SERVICE_SIGNATURE_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ,           \
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },   \
    { HDLC_PROTOCOL_INFO_VERSION_VALUE, HDLD_PROTOCOL_INFO_VERSION_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,                        \
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 }

/* Pairing service characteristics */
#define HOMEKIT_PAIRING_SERVICE_CHARACTERISTICS             \
    { HDLC_PAIRING_PAIR_SETUP_VALUE, HDLD_PAIRING_PAIR_SETUP_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ | HAP_CHARACTERISTIC_PROPERTY_WRITE,     \
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },   \
    { HDLC_PAIRING_PAIR_VERIFY_VALUE, HDLD_PAIRING_PAIR_VERIFY_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ | HAP_CHARACTERISTIC_PROPERTY_WRITE,   \
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },   \
    { HDLC_PAIRING_FEATURES_VALUE, HDLD_PAIRING_FEATURES_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ,                                             \
      HAP_CHARACTERISTIC_FORMAT_UINT8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },  \
    { HDLC_PAIRING_MANAGE_VALUE, HDLD_PAIRING_MANAGE_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE,   \
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 }

/* HAP OTA service characteristics */
#define HOMEKIT_HAP_OTA_SERVICE_CHARACTERISTICS             \
    { HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE_VALUE, HDLD_OTA_FW_UPGRADE_SERVICE_SIGNATURE_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ,         \
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },   \
    { HDLC_OTA_FW_UPGRADE_CONTROL_POINT_VALUE, HDLD_OTA_FW_UPGRADE_CONTROL_POINT_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE | HAP_CHARACTERISTIC_PROPERTY_NOTIFY_ALWAYS,   \
      HAP_CHARACTERISTIC_FORMAT_DATA, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },   \
    { HDLC_OTA_FW_UPGRADE_DATA_VALUE, HDLD_OTA_FW_UPGRADE_DATA_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE, \
      HAP_CHARACTERISTIC_FORMAT_DATA, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },   \
    { HDLC_OTA_FW_UPGRADE_APP_INFO_VALUE, HDLD_OTA_FW_UPGRADE_APP_INFO_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,                    \
      HAP_CHARACTERISTIC_FORMAT_DATA, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 }

/* HomeKit event from application library */
enum
{
    HOMEKIT_EVENT_BT_STACK_ENABLED,
    HOMEKIT_EVENT_GATT_CONNECTION_STATUS,
    HOMEKIT_EVENT_SET_CHARACTERISTIC_VALUE,
    HOMEKIT_EVENT_GET_CHARACTERISTIC_VALUE,
    HOMEKIT_EVENT_FACTORY_RESET,
};

/* Set/get characteristic value event source */
#define HOMEKIT_VALUE_EVENT_FROM_LOCAL      0       /* Local set/get request */
#define HOMEKIT_VALUE_EVENT_FROM_GATT       1       /* Read/write request from GATT */
#define HOMEKIT_VALUE_EVENT_FROM_HAP        2       /* Read/write request from HAP (HomeKit characteristic) */

/* NVRAM IDs */
#define BTLE_HOMEKIT_NVRAM_VSID_START                   (WICED_NVRAM_VSID_START + 5)
#define BTLE_HOMEKIT_NVRAM_VSID_CHAR_VALUES             (BTLE_HOMEKIT_NVRAM_VSID_START + 0)
#define BTLE_HOMEKIT_NVRAM_VSID_LOCAL_KEYS              (BTLE_HOMEKIT_NVRAM_VSID_START + 1)
#define BTLE_HOMEKIT_NVRAM_VSID_PEER_KEYS               (BTLE_HOMEKIT_NVRAM_VSID_START + 2)
#define BTLE_HOMEKIT_NVRAM_BDA_STATIC_RANDOM            (BTLE_HOMEKIT_NVRAM_VSID_START + 3)
#define BTLE_HOMEKIT_NVRAM_VSID_LAST                    (APPLE_HOMEKIT_NVRAM_VSID_START - 1)


/******************************************************************************
*                    Structures
******************************************************************************/

typedef struct
{
    wiced_bt_device_address_t   bd_addr;
    uint16_t                    conn_id;
    wiced_bool_t                connected;
    uint16_t                    reason;             /* Disconnect reason */
} wiced_homekit_gatt_conn_t;

typedef struct
{
    uint16_t    from;
    uint16_t    handle;
    uint8_t*    value;
    uint16_t    value_length;
} wiced_homekit_value_t;

typedef union
{
    wiced_homekit_gatt_conn_t   gatt_conn;          /* HOMEKIT_EVENT_GATT_CONNECTION_STATUS */
    wiced_bool_t                paired;             /* HOMEKIT_EVENT_PAIRING_STATUS */
    wiced_bool_t                hap_connected;      /* HOMEKIT_EVENT_SECURE_CONNECTION_STATUS */
    wiced_homekit_value_t       char_value;         /* HOMEKIT_EVENT_SET_CHARACTERISTIC_VALUE */
                                                    /* HOMEKIT_EVENT_GET_CHARACTERISTIC_VALUE */
} wiced_homekit_event_message_t;


/******************************************************************************
*               Function Declarations
******************************************************************************/

typedef void homekit_event_cback_t(uint16_t event, wiced_homekit_event_message_t* message);

/******************************************************************************
*
* \name homekit_app_accessory_setup
*
* \brief Sets up accessory parameters in the application library.
*
******************************************************************************/
void homekit_app_accessory_setup(wiced_btle_hap_info_t* hap_info, homekit_event_cback_t* event_cback);

/******************************************************************************
*
* \name homekit_app_set_value_complete
*
* \details This function is called in HOMEKIT_EVENT_SET_CHARACTERISTIC_VALUE callback.  
*          However, the application can return from event callback first and call
*          this function later when set value operation is completed.
*          Application can also call this function when a value is changed locally.
*
******************************************************************************/
void homekit_app_set_value_complete(wiced_bt_gatt_status_t status, wiced_homekit_value_t* p_value, wiced_bool_t value_changed);

/******************************************************************************
*
* \name homekit_app_get_value_complete
*
* \details This function is called in HOMEKIT_EVENT_GET_CHARACTERISTIC_VALUE callback.
*          However, application can return from event callback first and call
*          this function later when get value operation is completed.
*
******************************************************************************/
void homekit_app_get_value_complete(wiced_bt_gatt_status_t status, wiced_homekit_value_t* p_value);


#ifdef __cplusplus
}
#endif

#endif /* _WICED_HOMEKIT_APP_H_ */

