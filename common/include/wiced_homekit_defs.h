/***************************************************************************//**
* \file <wiced_homekit_db.h>
*
* \brief
* 	The Bluetooth Low Energy HomeKit accessory definitions.
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
#ifndef WICED_HOMEKIT_DEFS_H
#define WICED_HOMEKIT_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
*                    Constants
******************************************************************************/
/* Apple HomeKit Accessory Protocol (HAP) UUIDs */
#define APPLE_HOMEKIT_UUID(a,b)                         0x91, 0x52, 0x76, 0xBB, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, b, a, 0x00, 0x00

/* Apple defined Characteristics */

#define UUID_HAP_CHAR_ADMINISTRATOR_ONLY_ACCESS         APPLE_HOMEKIT_UUID(0x00,0x01)
#define UUID_HAP_CHAR_AUDIO_FEEDBACK                    APPLE_HOMEKIT_UUID(0x00,0x05)
#define UUID_HAP_CHAR_BRIGHTNESS                        APPLE_HOMEKIT_UUID(0x00,0x08)
#define UUID_HAP_CHAR_TEMPERATURE_COOLING_THRESHOLD     APPLE_HOMEKIT_UUID(0x00,0x0D)
#define UUID_HAP_CHAR_DOOR_STATE_CURRENT                APPLE_HOMEKIT_UUID(0x00,0x0E)
#define UUID_HAP_CHAR_HEATING_COOLING_CURRENT           APPLE_HOMEKIT_UUID(0x00,0x0F)
#define UUID_HAP_CHAR_RELATIVE_HUMIDITY_CURRENT         APPLE_HOMEKIT_UUID(0x00,0x10)
#define UUID_HAP_CHAR_TEMPERATURE_CURRENT               APPLE_HOMEKIT_UUID(0x00,0x11)
#define UUID_HAP_CHAR_FIRMWARE_REVISION                 APPLE_HOMEKIT_UUID(0x00,0x52)
#define UUID_HAP_CHAR_HARDWARE_REVISION                 APPLE_HOMEKIT_UUID(0x00,0x53)
#define UUID_HAP_CHAR_TEMPERATURE_HEATING_THRESHOLD     APPLE_HOMEKIT_UUID(0x00,0x12)
#define UUID_HAP_CHAR_HUE                               APPLE_HOMEKIT_UUID(0x00,0x13)
#define UUID_HAP_CHAR_IDENTIFY                          APPLE_HOMEKIT_UUID(0x00,0x14)
#define UUID_HAP_CHAR_LOCK_MANAGEMENT_CONTROL_POINT     APPLE_HOMEKIT_UUID(0x00,0x19)
#define UUID_HAP_CHAR_LOCK_MANAGEMENT_AUTO_SECURE_TIMEOUT APPLE_HOMEKIT_UUID(0x00,0x1A)
#define UUID_HAP_CHAR_LOCK_MECHANISM_LAST_KNOWN_ACTION  APPLE_HOMEKIT_UUID(0x00,0x1C)
#define UUID_HAP_CHAR_LOCK_MECHANISM_CURRENT_STATE      APPLE_HOMEKIT_UUID(0x00,0x1D)
#define UUID_HAP_CHAR_LOCK_MECHANISM_TARGET_STATE       APPLE_HOMEKIT_UUID(0x00,0x1E)
#define UUID_HAP_CHAR_LOGS                              APPLE_HOMEKIT_UUID(0x00,0x1F)
#define UUID_HAP_CHAR_MANUFACTURER                      APPLE_HOMEKIT_UUID(0x00,0x20)
#define UUID_HAP_CHAR_MODEL                             APPLE_HOMEKIT_UUID(0x00,0x21)
#define UUID_HAP_CHAR_MOTION_DETECTED                   APPLE_HOMEKIT_UUID(0x00,0x22)
#define UUID_HAP_CHAR_NAME                              APPLE_HOMEKIT_UUID(0x00,0x23)
#define UUID_HAP_CHAR_OBSTRUCTION_DETECTED              APPLE_HOMEKIT_UUID(0x00,0x24)
#define UUID_HAP_CHAR_ON                                APPLE_HOMEKIT_UUID(0x00,0x25)
#define UUID_HAP_CHAR_OUTLET_IN_USE                     APPLE_HOMEKIT_UUID(0x00,0x26)
#define UUID_HAP_CHAR_ROTATION_DIRECTION                APPLE_HOMEKIT_UUID(0x00,0x28)
#define UUID_HAP_CHAR_ROTATION_SPEED                    APPLE_HOMEKIT_UUID(0x00,0x29)
#define UUID_HAP_CHAR_SATURATION                        APPLE_HOMEKIT_UUID(0x00,0x2F)
#define UUID_HAP_CHAR_SERIAL_NUMBER                     APPLE_HOMEKIT_UUID(0x00,0x30)
#define UUID_HAP_CHAR_DOOR_STATE_TARGET                 APPLE_HOMEKIT_UUID(0x00,0x32)
#define UUID_HAP_CHAR_HEATING_COOLING_TARGET            APPLE_HOMEKIT_UUID(0x00,0x33)
#define UUID_HAP_CHAR_RELATIVE_HUMIDITY_TARGET          APPLE_HOMEKIT_UUID(0x00,0x34)
#define UUID_HAP_CHAR_TEMPERATURE_TARGET                APPLE_HOMEKIT_UUID(0x00,0x35)
#define UUID_HAP_CHAR_TEMPERATURE_UNIT                  APPLE_HOMEKIT_UUID(0x00,0x36)
#define UUID_HAP_CHAR_VERSION                           APPLE_HOMEKIT_UUID(0x00,0x37)
#define UUID_HAP_CHAR_AIR_PARTICULATE_DENSITY           APPLE_HOMEKIT_UUID(0x00,0x64)
#define UUID_HAP_CHAR_AIR_PARTICULATE_SIZE              APPLE_HOMEKIT_UUID(0x00,0x65)
#define UUID_HAP_CHAR_SECURITY_SYSTEM_STATE_CURRENT     APPLE_HOMEKIT_UUID(0x00,0x66)
#define UUID_HAP_CHAR_SECURITY_SYSTEM_STATE_TARGET      APPLE_HOMEKIT_UUID(0x00,0x67)
#define UUID_HAP_CHAR_BATTERY_LEVEL                     APPLE_HOMEKIT_UUID(0x00,0x68)
#define UUID_HAP_CHAR_CARBON_MONOXIDE_DETECTED          APPLE_HOMEKIT_UUID(0x00,0x69)
#define UUID_HAP_CHAR_CONTACT_STATE                     APPLE_HOMEKIT_UUID(0x00,0x6A)
#define UUID_HAP_CHAR_LIGHT_LEVEL_CURRENT               APPLE_HOMEKIT_UUID(0x00,0x6B)
#define UUID_HAP_CHAR_HORIZONTAL_TILT_CURRENT           APPLE_HOMEKIT_UUID(0x00,0x6C)
#define UUID_HAP_CHAR_POSITION_CURRENT                  APPLE_HOMEKIT_UUID(0x00,0x6D)
#define UUID_HAP_CHAR_VERTICAL_TILT_CURRENT             APPLE_HOMEKIT_UUID(0x00,0x6E)
#define UUID_HAP_CHAR_POSITION_HOLD                     APPLE_HOMEKIT_UUID(0x00,0x6F)
#define UUID_HAP_CHAR_LEAK_DETECTED                     APPLE_HOMEKIT_UUID(0x00,0x70)
#define UUID_HAP_CHAR_OCCUPANCY_DETECTED                APPLE_HOMEKIT_UUID(0x00,0x71)
#define UUID_HAP_CHAR_POSITION_STATE                    APPLE_HOMEKIT_UUID(0x00,0x72)
#define UUID_HAP_CHAR_INPUT_EVENT                       APPLE_HOMEKIT_UUID(0x00,0x73)
#define UUID_HAP_CHAR_STATUS_ACTIVE                     APPLE_HOMEKIT_UUID(0x00,0x75)
#define UUID_HAP_CHAR_SMOKE_DETECTED                    APPLE_HOMEKIT_UUID(0x00,0x76)
#define UUID_HAP_CHAR_STATUS_FAULT                      APPLE_HOMEKIT_UUID(0x00,0x77)
#define UUID_HAP_CHAR_STATUS_JAMMED                     APPLE_HOMEKIT_UUID(0x00,0x78)
#define UUID_HAP_CHAR_STATUS_LO_BATT                    APPLE_HOMEKIT_UUID(0x00,0x79)
#define UUID_HAP_CHAR_STATUS_TAMPERED                   APPLE_HOMEKIT_UUID(0x00,0x7A)
#define UUID_HAP_CHAR_HORIZONTAL_TILT_TARGET            APPLE_HOMEKIT_UUID(0x00,0x7B)
#define UUID_HAP_CHAR_POSITION_TARGET                   APPLE_HOMEKIT_UUID(0x00,0x7C)
#define UUID_HAP_CHAR_VERTICAL_TILT_TARGET              APPLE_HOMEKIT_UUID(0x00,0x7D)
#define UUID_HAP_CHAR_SECURITY_SYSTEM_ALARM_TYPE        APPLE_HOMEKIT_UUID(0x00,0x8E)
#define UUID_HAP_CHAR_CHARGING_STATE                    APPLE_HOMEKIT_UUID(0x00,0x8F)
#define UUID_HAP_CHAR_CARBON_MONOXIDE_LEVEL             APPLE_HOMEKIT_UUID(0x00,0x90)
#define UUID_HAP_CHAR_CARBON_MONOXIDE_PEAK_LEVEL        APPLE_HOMEKIT_UUID(0x00,0x91)
#define UUID_HAP_CHAR_CARBON_DIOXIDE_DETECTED           APPLE_HOMEKIT_UUID(0x00,0x92)
#define UUID_HAP_CHAR_CARBON_DIOXIDE_LEVEL              APPLE_HOMEKIT_UUID(0x00,0x93)
#define UUID_HAP_CHAR_CARBON_DIOXIDE_PEAK_LEVEL         APPLE_HOMEKIT_UUID(0x00,0x94)
#define UUID_HAP_CHAR_AIR_QUALITY                       APPLE_HOMEKIT_UUID(0x00,0x95)
#define UUID_HAP_CHAR_STREAMING_STATUS                  APPLE_HOMEKIT_UUID(0x01,0x20)
#define UUID_HAP_CHAR_SUPPORTED_VIDEO_CONFIGURATION     APPLE_HOMEKIT_UUID(0x01,0x14)
#define UUID_HAP_CHAR_SUPPORTED_AUDIO_CONFIGURATION     APPLE_HOMEKIT_UUID(0x01,0x15)
#define UUID_HAP_CHAR_SUPPORTED_RTP_CONFIGURATION       APPLE_HOMEKIT_UUID(0x01,0x16)
#define UUID_HAP_CHAR_SELECTED_RTP_CONFIGURATION        APPLE_HOMEKIT_UUID(0x01,0x17)
#define UUID_HAP_CHAR_SETUP_ENPOINTS                    APPLE_HOMEKIT_UUID(0x01,0x18)
#define UUID_HAP_CHAR_VOLUME                            APPLE_HOMEKIT_UUID(0x01,0x19)
#define UUID_HAP_CHAR_MUTE                              APPLE_HOMEKIT_UUID(0x01,0x1A)
#define UUID_HAP_CHAR_NIGHT_VISION                      APPLE_HOMEKIT_UUID(0x01,0x1B)
#define UUID_HAP_CHAR_OPTICAL_ZOOM                      APPLE_HOMEKIT_UUID(0x01,0x1C)
#define UUID_HAP_CHAR_DIGITAL_ZOOM                      APPLE_HOMEKIT_UUID(0x01,0x1D)
#define UUID_HAP_CHAR_IMAGE_ROTATION                    APPLE_HOMEKIT_UUID(0x01,0x1E)
#define UUID_HAP_CHAR_IMAGE_MIRROR                      APPLE_HOMEKIT_UUID(0x01,0x1F)
#define UUID_HAP_CHAR_ACCESSORY_PROPERTIES              APPLE_HOMEKIT_UUID(0x00,0xA6)
#define UUID_HAP_CHAR_LOCK_PHYSICAL_CONTROLS            APPLE_HOMEKIT_UUID(0x00,0xA7)
#define UUID_HAP_CHAR_AIR_PURIFIER_STATE_TARGET         APPLE_HOMEKIT_UUID(0x00,0xA8)
#define UUID_HAP_CHAR_AIR_PURIFIER_STATE_CURRENT        APPLE_HOMEKIT_UUID(0x00,0xA9)
#define UUID_HAP_CHAR_SLAT_STATE_CURRENT                APPLE_HOMEKIT_UUID(0x00,0xAA)
#define UUID_HAP_CHAR_TYPE_SLAT                         APPLE_HOMEKIT_UUID(0x00,0xC0)
#define UUID_HAP_CHAR_FILTER_LIFE_LEVEL                 APPLE_HOMEKIT_UUID(0x00,0xAB)
#define UUID_HAP_CHAR_FILTER_CHANGE_INDICATION          APPLE_HOMEKIT_UUID(0x00,0xAC)
#define UUID_HAP_CHAR_FILTER_RESET_INDICATION           APPLE_HOMEKIT_UUID(0x00,0xAD)
#define UUID_HAP_CHAR_FAN_STATE_TARGET                  APPLE_HOMEKIT_UUID(0x00,0xBF)
#define UUID_HAP_CHAR_FAN_STATE_CURRENT                 APPLE_HOMEKIT_UUID(0x00,0xAF)
#define UUID_HAP_CHAR_ACTIVE                            APPLE_HOMEKIT_UUID(0x00,0xB0)
#define UUID_HAP_CHAR_SWING_MODE                        APPLE_HOMEKIT_UUID(0x00,0xB6)
#define UUID_HAP_CHAR_TILT_CURRENT                      APPLE_HOMEKIT_UUID(0x00,0xC1)
#define UUID_HAP_CHAR_TILT_TARGET                       APPLE_HOMEKIT_UUID(0x00,0xC2)
#define UUID_HAP_CHAR_DENSITY_OZONE                     APPLE_HOMEKIT_UUID(0x00,0xC3)
#define UUID_HAP_CHAR_DENSITY_NO2                       APPLE_HOMEKIT_UUID(0x00,0xC4)
#define UUID_HAP_CHAR_DENSITY_SO2                       APPLE_HOMEKIT_UUID(0x00,0xC5)
#define UUID_HAP_CHAR_DENSITY_PM25                      APPLE_HOMEKIT_UUID(0x00,0xC6)
#define UUID_HAP_CHAR_DENSITY_PM10                      APPLE_HOMEKIT_UUID(0x00,0xC7)
#define UUID_HAP_CHAR_DENSITY_VOC                       APPLE_HOMEKIT_UUID(0x00,0xC8)
#define UUID_HAP_CHAR_SERVICE_LABEL_INDEX               APPLE_HOMEKIT_UUID(0x00,0xCB)
#define UUID_HAP_CHAR_SERVICE_LABEL_NAMESPACE           APPLE_HOMEKIT_UUID(0x00,0xCD)
#define UUID_HAP_CHAR_COLOR_TEMPERATURE                 APPLE_HOMEKIT_UUID(0x00,0xCE)
#define UUID_HAP_CHAR_HEATER_COOLER_STATE_CURRENT       APPLE_HOMEKIT_UUID(0x00,0xB1)
#define UUID_HAP_CHAR_HEATER_COOLER_STATE_TARGET        APPLE_HOMEKIT_UUID(0x00,0xB2)
#define UUID_HAP_CHAR_HUMIDIFIER_DEHUMIDIFIER_STATE_CURRENT APPLE_HOMEKIT_UUID(0x00,0xB3)
#define UUID_HAP_CHAR_HUMIDIFIER_DEHUMIDIFIER_STATE_TARGET  APPLE_HOMEKIT_UUID(0x00,0xB4)
#define UUID_HAP_CHAR_WATER_LEVEL                       APPLE_HOMEKIT_UUID(0x00,0xB5)
#define UUID_HAP_CHAR_RELATIVE_HUMIDIFIER_DEHUMIDIFIER_THRESHOLD APPLE_HOMEKIT_UUID(0x00,0xC9)
#define UUID_HAP_CHAR_RELATIVE_HUMIDITY_HUMIDIFIER_THRESHOLD     APPLE_HOMEKIT_UUID(0x00,0xCA)

/* Apple defined Services */

#define UUID_HAP_SERVICE_ACCESSORY_INFORMATION          APPLE_HOMEKIT_UUID(0x00,0x3E)
#define UUID_HAP_SERVICE_PROTOCOL_INFORMATION           APPLE_HOMEKIT_UUID(0x00,0xA2)
#define UUID_HAP_SERVICE_FAN                            APPLE_HOMEKIT_UUID(0x00,0x40)
#define UUID_HAP_SERVICE_GARAGE_DOOR_OPENER             APPLE_HOMEKIT_UUID(0x00,0x41)
#define UUID_HAP_SERVICE_LIGHTBULB                      APPLE_HOMEKIT_UUID(0x00,0x43)
#define UUID_HAP_SERVICE_LOCK_MANAGEMENT                APPLE_HOMEKIT_UUID(0x00,0x44)
#define UUID_HAP_SERVICE_LOCK_MECHANISM                 APPLE_HOMEKIT_UUID(0x00,0x45)
#define UUID_HAP_SERVICE_OUTLET                         APPLE_HOMEKIT_UUID(0x00,0x47)
#define UUID_HAP_SERVICE_SWITCH                         APPLE_HOMEKIT_UUID(0x00,0x49)
#define UUID_HAP_SERVICE_THERMOSTAT                     APPLE_HOMEKIT_UUID(0x00,0x4A)
#define UUID_HAP_SERVICE_SENSOR_AIR_QUALITY             APPLE_HOMEKIT_UUID(0x00,0x8D)
#define UUID_HAP_SERVICE_SECURITY_SYSTEM                APPLE_HOMEKIT_UUID(0x00,0x7E)
#define UUID_HAP_SERVICE_SENSOR_CARBON_MONOXIDE         APPLE_HOMEKIT_UUID(0x00,0x7F)
#define UUID_HAP_SERVICE_SENSOR_CONTACT                 APPLE_HOMEKIT_UUID(0x00,0x80)
#define UUID_HAP_SERVICE_DOOR                           APPLE_HOMEKIT_UUID(0x00,0x81)
#define UUID_HAP_SERVICE_SENSOR_HUMIDITY                APPLE_HOMEKIT_UUID(0x00,0x82)
#define UUID_HAP_SERVICE_SENSOR_LEAK                    APPLE_HOMEKIT_UUID(0x00,0x83)
#define UUID_HAP_SERVICE_SENSOR_LIGHT                   APPLE_HOMEKIT_UUID(0x00,0x84)
#define UUID_HAP_SERVICE_SENSOR_MOTION                  APPLE_HOMEKIT_UUID(0x00,0x85)
#define UUID_HAP_SERVICE_SENSOR_OCCUPANCY               APPLE_HOMEKIT_UUID(0x00,0x86)
#define UUID_HAP_SERVICE_SENSOR_SMOKE                   APPLE_HOMEKIT_UUID(0x00,0x87)
#define UUID_HAP_SERVICE_STATELESS_PROGRAMMABLE_SWITCH  APPLE_HOMEKIT_UUID(0x00,0x89)
#define UUID_HAP_SERVICE_SENSOR_TEMPERATURE             APPLE_HOMEKIT_UUID(0x00,0x8A)
#define UUID_HAP_SERVICE_WINDOW                         APPLE_HOMEKIT_UUID(0x00,0x8B)
#define UUID_HAP_SERVICE_WINDOW_COVERING                APPLE_HOMEKIT_UUID(0x00,0x8C)
#define UUID_HAP_SERVICE_BATTERY                        APPLE_HOMEKIT_UUID(0x00,0x96)
#define UUID_HAP_SERVICE_SENSOR_CARBON_DIOXIDE          APPLE_HOMEKIT_UUID(0x00,0x97)
#define UUID_HAP_SERVICE_CAMERA_RTP_STREAM_MANAGEMENT   APPLE_HOMEKIT_UUID(0x01,0x10)
#define UUID_HAP_SERVICE_MICROPHONE                     APPLE_HOMEKIT_UUID(0x01,0x12)
#define UUID_HAP_SERVICE_SPEAKER                        APPLE_HOMEKIT_UUID(0x01,0x13)
#define UUID_HAP_SERVICE_DOORBELL                       APPLE_HOMEKIT_UUID(0x01,0x21)
#define UUID_HAP_SERVICE_FANV2                          APPLE_HOMEKIT_UUID(0x00,0xB7)
#define UUID_HAP_SERVICE_VERTICAL_SLAT                  APPLE_HOMEKIT_UUID(0x00,0xB9)
#define UUID_HAP_SERVICE_FILTER_MAINTENANCE             APPLE_HOMEKIT_UUID(0x00,0xBA)
#define UUID_HAP_SERVICE_AIR_PURIFIER                   APPLE_HOMEKIT_UUID(0x00,0xBB)
#define UUID_HAP_SERVICE_HEATER_COOLER                  APPLE_HOMEKIT_UUID(0x00,0xBC)
#define UUID_HAP_SERVICE_HUMIDIFIER_DEHUMIDIFIER        APPLE_HOMEKIT_UUID(0x00,0xBD)
#define UUID_HAP_SERVICE_SERVICE_LABEL                  APPLE_HOMEKIT_UUID(0x00,0xCC)

/* Other UUIDs */

#define UUID_HAP_CHAR_SERVICE_SIGNATURE                 APPLE_HOMEKIT_UUID(0x00,0xA5)
#define UUID_HAP_CHAR_SERVICE_INSTANCE_ID               0xd1, 0xa0, 0x83, 0x50, 0x00, 0xaa, 0xd3, 0x87, 0x17, 0x48, 0x59, 0xa7, 0x5d, 0xe9, 0x04, 0xe6
#define UUID_HAP_DESCR_CHAR_INSTANCE_ID                 0x9a, 0x93, 0x96, 0xd7, 0xbd, 0x6a, 0xd9, 0xb5, 0x16, 0x46, 0xd2, 0x81, 0xfe, 0xf0, 0x46, 0xdc

/* HomeKit Pairing service and characteristics */

#define UUID_HAP_SERVICE_PAIRING                        APPLE_HOMEKIT_UUID(0x00,0x55)
#define UUID_HAP_CHAR_PAIRING_PAIR_SETUP                APPLE_HOMEKIT_UUID(0x00,0x4C)
#define UUID_HAP_CHAR_PAIRING_PAIR_VERIFY               APPLE_HOMEKIT_UUID(0x00,0x4E)
#define UUID_HAP_CHAR_PAIRING_FEATURES                  APPLE_HOMEKIT_UUID(0x00,0x4F)
#define UUID_HAP_CHAR_PAIRING_PAIRINGS                  APPLE_HOMEKIT_UUID(0x00,0x50)

/* Pairing feature flags */
#define PAIRING_FEATURE_HARDWARE_CERTIFICATE_AUTHENTICATION     0x01
#define PAIRING_FEATURE_SOFTWARE_TOKEN_AUTHENTICATION           0x02
#define PAIRING_FEATURE_SOFTWARE_CERTIFICATE_AUTHENTICATION     0x04

/* An accessory with support of multiple categories should advertise the primary category.
   An accessory for which a primary category cannot be determined or the primary category
   is not among the well-defined categories (2-9) falls in the "Other" category. */
#define HOMEKIT_ACCESSORY_CATEGORY_OTHER                1
#define HOMEKIT_ACCESSORY_CATEGORY_BRIDGE               2
#define HOMEKIT_ACCESSORY_CATEGORY_FAN                  3
#define HOMEKIT_ACCESSORY_CATEGORY_GARAGE_DOOR_OPENER   4
#define HOMEKIT_ACCESSORY_CATEGORY_LIGHTBULB            5
#define HOMEKIT_ACCESSORY_CATEGORY_DOOR_LOCK            6
#define HOMEKIT_ACCESSORY_CATEGORY_OUTLET               7
#define HOMEKIT_ACCESSORY_CATEGORY_SWITCH               8
#define HOMEKIT_ACCESSORY_CATEGORY_THERMOSTAT           9
#define HOMEKIT_ACCESSORY_CATEGORY_SENSOR               10
#define HOMEKIT_ACCESSORY_CATEGORY_SECURITY_SYSTEM      11
#define HOMEKIT_ACCESSORY_CATEGORY_DOOR                 12
#define HOMEKIT_ACCESSORY_CATEGORY_WINDOW               13
#define HOMEKIT_ACCESSORY_CATEGORY_WINDOW_COVERING      14
#define HOMEKIT_ACCESSORY_CATEGORY_PROGRAMMABLE_SWITCH  15
#define HOMEKIT_ACCESSORY_CATEGORY_RANGE_EXTENDER       16
#define HOMEKIT_ACCESSORY_CATEGORY_IP_CAMERA            17
#define HOMEKIT_ACCESSORY_CATEGORY_VIDEO_DOOR_BELL      18
#define HOMEKIT_ACCESSORY_CATEGORY_AIR_PURIFIER         19

/* HAP-BLE PDU Control field */
#define HAP_CONTROL_FIELD_REQUEST                           0x00
#define HAP_CONTROL_FIELD_RESPONSE                          0x02
#define HAP_CONTROL_FIELD_FRAGMENT                          0x80

/* HAP Opcode */
#define HAP_CHARACTERISTIC_SIGNATURE_READ                   0x01
#define HAP_CHARACTERISTIC_WRITE                            0x02
#define HAP_CHARACTERISTIC_READ                             0x03
#define HAP_CHARACTERISTIC_TIMED_WRITE                      0x04
#define HAP_CHARACTERISTIC_EXECUTE_WRITE                    0x05
#define HAP_SERVICE_SIGNATURE_READ                          0x06
#define HAP_CHARACTERISTIC_CONFIGURATION                    0x07
#define HAP_PROTOCOL_CONFIGURATION                          0x08

/* HAP Software Token Authentication Opcodes */
#define HAP_TOKEN_READ                                      0x10
#define HAP_TOKEN_UPDATE                                    0x11
#define HAP_INFO_READ                                       0x12

/* Software Token Authentication TLV types */
#define HAP_PARAM_UUID                                      0x01
#define HAP_PARAM_SOFTWARE_TOKEN                            0x02

/* HAP Info Parameter Types */
#define HAP_INFO_PARAM_CURRENT_STATE_NUMBER                 0x01
#define HAP_INFO_PARAM_CURRENT_CONFIG_NUMBER                0x02
#define HAP_INFO_PARAM_DEVICE_IDENTIFIER                    0x03
#define HAP_INFO_PARAM_FEATURE_FLAGS                        0x04
#define HAP_INFO_PARAM_MODEL_NAME                           0x05
#define HAP_INFO_PARAM_PROTOCOL_VERSION                     0x06
#define HAP_INFO_PARAM_STATUS_FLAG                          0x07
#define HAP_INFO_PARAM_CATEGORY_IDENTIFIER                  0x08
#define HAP_INFO_PARAM_SETUP_HASH                           0x09

/* HAP-BLE PDU additional parameter types */
#define HAP_PARAM_VALUE                                     0x01
#define HAP_PARAM_ADDITIONAL_AUTHORIZATION_DATA             0x02
#define HAP_PARAM_ORIGIN                                    0x03
#define HAP_PARAM_CHARACTERISTIC_TYPE                       0x04
#define HAP_PARAM_CHARACTERISTIC_INSTANCE_ID                0x05
#define HAP_PARAM_SERVICE_TYPE                              0x06
#define HAP_PARAM_SERVICE_INSTANCE_ID                       0x07
#define HAP_PARAM_TTL                                       0x08
#define HAP_PARAM_RETURN_RESPONSE                           0x09
#define HAP_PARAM_HAP_CHARACTERISTIC_PROPERTIES_DESCRIPTOR  0x0a
#define HAP_PARAM_GATT_USER_DESCRIPTION_DESCRIPTOR          0x0b
#define HAP_PARAM_GATT_PRESENTATION_FORMAT_DESCRIPTOR       0x0c
#define HAP_PARAM_GATT_VALID_RANGE                          0x0d
#define HAP_PARAM_HAP_STEP_VALUE_DESCRIPTOR                 0x0e
#define HAP_PARAM_HAP_SERVICE_PROPERTIES                    0x0f
#define HAP_PARAM_HAP_LINKED_SERVICES                       0x10
#define HAP_PARAM_HAP_VALID_VALUES_DESCRIPTOR               0x11
#define HAP_PARAM_HAP_VALIE_VALUES_RANGE_DESCRIPTOR         0x12

/* HAP characteristic configuration parameter types */
#define HAP_CHAR_CFG_PARAM_PROPERTIES                       0x01
#define HAP_CHAR_CFG_PARAM_BROADCAST_INTERVAL               0x02

/* HAP characteristic configuration properties */
#define HAP_CHAR_CFG_PROPERTIES_BROADCAST_NOTIF_BITMASK     0x0001

/* HAP characteristic configuration broadcast interval */
#define HAP_CHAR_CFG_BROADCAST_INTERVAL_20MS                0x01
#define HAP_CHAR_CFG_BROADCAST_INTERVAL_1280MS              0x02
#define HAP_CHAR_CFG_BROADCAST_INTERVAL_2560MS              0x03

/* HAP protocol configuration request parameter types */
#define HAP_PROTO_CFG_PARAM_GENERATE_BROADCAST_KEY          0x01
#define HAP_PROTO_CFG_PARAM_GET_ALL_PARAMS                  0x02
#define HAP_PROTO_CFG_PARAM_SET_ACCESSORY_ADV_ID            0x03

/* HAP protocol configuration response parameter types */
#define HAP_PROTO_CFG_PARAM_CURRENT_STATE_NUMBER            0x01
#define HAP_PROTO_CFG_PARAM_CURRENT_CONFIG_NUMBER           0x02
#define HAP_PROTO_CFG_PARAM_ACCESSORY_ADV_ID                0x03
#define HAP_PROTO_CFG_PARAM_BROADCAST_KEY                   0x04

/* HAP status code */
#define HAP_STATUS_SUCCESS                                  0x00
#define HAP_STATUS_UNSUPPORTED_PDU                          0x01
#define HAP_STATUS_MAX_PROCEDURES                           0x02
#define HAP_STATUS_INSUFFICIENT_AUTHORIZATION               0x03
#define HAP_STATUS_INVALID_INSTANCE_ID                      0x04
#define HAP_STATUS_INSUFFICIENT_AUTHENTICATION              0x05
#define HAP_STATUS_INVALID_REQUEST                          0x06

/* HAP Service properties */
#define HAP_SERVICE_PROPERTY_PRIMARY_SERVICE                0x0001
#define HAP_SERVICE_PROPERTY_HIDDEN_SERVICE                 0x0002
#define HAP_SERVICE_PROPERTY_SUPPORTS_CONFIGURATION         0x0004

/* HAP Characteristic properties */
#define HAP_CHARACTERISTIC_PROPERTY_READ                    0x0001
#define HAP_CHARACTERISTIC_PROPERTY_WRITE                   0x0002
#define HAP_CHARACTERISTIC_PROPERTY_ADDITIONAL_AUTH_DATA    0x0004
#define HAP_CHARACTERISTIC_PROPERTY_TIMED_WRITE_PROCEDURE   0x0008
#define HAP_CHARACTERISTIC_PROPERTY_SECURE_READ             0x0010
#define HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE            0x0020
#define HAP_CHARACTERISTIC_PROPERTY_INVISIBLE_TO_USER       0x0040
#define HAP_CHARACTERISTIC_PROPERTY_NOTIFY_IN_CONNECTED     0x0080
#define HAP_CHARACTERISTIC_PROPERTY_NOTIFY_IN_DISCONNECTED  0x0100
#define HAP_CHARACTERISTIC_PROPERTY_BROADCAST_NOTIFY        0x0200

#define HAP_CHARACTERISTIC_PROPERTY_NOTIFY_ALWAYS           (HAP_CHARACTERISTIC_PROPERTY_NOTIFY_IN_CONNECTED | HAP_CHARACTERISTIC_PROPERTY_NOTIFY_IN_DISCONNECTED | HAP_CHARACTERISTIC_PROPERTY_BROADCAST_NOTIFY)

/* HAP Characteristic presentation format types */
#define HAP_CHARACTERISTIC_FORMAT_BOOL                      0x01
#define HAP_CHARACTERISTIC_FORMAT_UINT8                     0x04
#define HAP_CHARACTERISTIC_FORMAT_UINT16                    0x06
#define HAP_CHARACTERISTIC_FORMAT_UINT32                    0x08
#define HAP_CHARACTERISTIC_FORMAT_UINT64                    0x0a
#define HAP_CHARACTERISTIC_FORMAT_INT32                     0x10
#define HAP_CHARACTERISTIC_FORMAT_FLOAT                     0x14
#define HAP_CHARACTERISTIC_FORMAT_STRING                    0x19
#define HAP_CHARACTERISTIC_FORMAT_TLV8                      0x1b
#define HAP_CHARACTERISTIC_FORMAT_DATA                      0x1b

/* HAP Characteristic presentation format unit types */
#define HAP_CHARACTERISTIC_FORMAT_UNIT_CELSIUS              0x272f
#define HAP_CHARACTERISTIC_FORMAT_UNIT_ARCDEGREES           0x2763
#define HAP_CHARACTERISTIC_FORMAT_UNIT_PERCENTAGE           0x27ad
#define HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS             0x2700
#define HAP_CHARACTERISTIC_FORMAT_UNIT_LUX                  0x2731
#define HAP_CHARACTERISTIC_FORMAT_UNIT_SECONDS              0x2703

/* HAP PDU sizes */
#define HAP_PDU_REQUEST_HEADER_SIZE                         5
#define HAP_PDU_RESPONSE_HEADER_SIZE                        3
#define HAP_PDU_FRAGMENT_HEADER_SIZE                        2
#define HAP_PDU_BODY_LEN_SIZE                               2
#define HAP_PDU_REQUEST_BODY_VALUE_OFFSET                   (HAP_PDU_REQUEST_HEADER_SIZE + HAP_PDU_BODY_LEN_SIZE)
#define HAP_PDU_RESPONSE_BODY_VALUE_OFFSET                  (HAP_PDU_RESPONSE_HEADER_SIZE + HAP_PDU_BODY_LEN_SIZE)

/******************************************************************************
*                   Enumerations
******************************************************************************/

/******************************************************************************
*                    Structures
******************************************************************************/

#pragma pack(1)
typedef struct
{
    uint8_t         control;        /* Control Field (1 Byte) */
    uint8_t         opcode;         /* HAP Opcode (1 Byte) */
    uint8_t         tid;            /* Transaction Identifier (1 Byte) */
    uint8_t         cid[2];         /* Characteristic Instance Identifier (2 Bytes) */
    uint8_t         body_len[2];    /* PDU Body Length (2 Bytes) */
    uint8_t         body_value[1];  /* Additional Params and Values (1-n Bytes) */
} wiced_btle_hap_request_t;

typedef struct
{
    uint8_t         control;        /* Control Field (1 Byte) */
    uint8_t         tid;            /* Transaction Identifier (1 Byte) */
    uint8_t         status;         /* Status (1Byte) */
    uint8_t         body_len[2];    /* PDU Body Length (2 Bytes) */
    uint8_t         body_value[1];  /* Additional Params and Values (1-n Bytes) */
} wiced_btle_hap_response_t;

typedef struct
{
    uint8_t         type;
    uint8_t         length;
    uint8_t         value[1];
} hap_pdu_tlv8_t;
#pragma pack()

#ifdef __cplusplus
}
#endif

#endif /* WICED_HOMEKIT_DEFS_H_ */
