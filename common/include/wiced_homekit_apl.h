/***************************************************************************//**
* \file <wiced_homekit_apl.h>
*
* \brief
* 	The Bluetooth Low Energy HomeKit accessory application.
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

#ifndef WICED_HOMEKIT_APL_H
#define WICED_HOMEKIT_APL_H

#include "HAP.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
*                    Constants
******************************************************************************/

/* NVRAM IDs */
#define HOMEKIT_APP_NVRAM_VSID_START                    (WICED_NVRAM_VSID_START)
#define HOMEKIT_APP_NVRAM_VSID_LOCAL_KEYS               (HOMEKIT_APP_NVRAM_VSID_START + 0)
#define HOMEKIT_APP_NVRAM_VSID_PEER_KEYS                (HOMEKIT_APP_NVRAM_VSID_START + 1)

#define HOMEKIT_LIB_NVRAM_VSID_START                    (WICED_NVRAM_VSID_END - 337)
#define HOMEKIT_LIB_NVRAM_VSID_END                      (WICED_NVRAM_VSID_END - 200)

/* GATT database HAP handles */
#define HAP_GATT_DB_HANDLE_START        0x0200
#define HAP_GATT_DB_HANDLE_END          0x0400

/*
 *  Application connection mode
 */
#define HOMEKIT_APP_MODE_HAP            0
#define HOMEKIT_APP_MODE_PRIVATE        1

/******************************************************
 *                    Functions
 ******************************************************/
/**
 * Set application connection mode
 *
 * If the application is in a non-HAP connection, call this function to set it to PRIVATE
 * mode so the HAP library won't disconnect it due to time out.
 */
void HAPSetAppConnectionMode(uint32_t mode);

/******************************************************************************
*         Functions to Implement by Application
******************************************************************************/

extern HAPBLEAccessoryServerStorage bleAccessoryServerStorage;

/******************************************************************************
*
* 
* \name AppCreate
*
* \brief Initializes the application.
*
******************************************************************************/
void AppCreate(
    HAPAccessoryServerRef *server,
    HAPPlatformKeyValueStoreRef keyValueStore);

/******************************************************************************
*
* \name AppRelease
*
* \brief Deinitializes the application.
*
******************************************************************************/
void AppRelease(void);

/******************************************************************************
*
* \name AppAccessoryServerStart
*
* \brief Starts the accessory server for the app.
*
******************************************************************************/
void AppAccessoryServerStart(void);

/******************************************************************************
*
* \name AccessoryServerHandleUpdatedState
*
* \brief Handles the updated state of the Accessory Server.
*
******************************************************************************/
void AccessoryServerHandleUpdatedState(
    HAPAccessoryServerRef *server,
    void *_Nullable context);

/******************************************************************************
* 
* \name AppRestoreFactorySettings
*
* \brief Restores the app specific factory settings.
*
******************************************************************************/
void AppRestoreFactorySettings(void);


typedef struct {
    uint16_t iid;
    char * description;
} HAPAccessoryAttrInfo;

/******************************************************************************
* 
* \name AccessoryGetAttributes
*
* \brief Gets a list of accessory attributes (for client control UI).
*
******************************************************************************/
void AccessoryGetAttributes(HAPAccessoryAttrInfo **attrInfo, size_t *numAttr);

/******************************************************************************
*
* \name AccessoryReadAttributeValue
*
* \brief Reads the value from an accessory attribute (for client control UI).
*
******************************************************************************/
void AccessoryReadAttributeValue(uint16_t iid, uint8_t **data, size_t *dataLength);

/******************************************************************************
*
* \name AccessoryWriteAttributeValue
*
* \brief Sets the value of an accessory attribute (for client control UI).
*
******************************************************************************/
void AccessoryWriteAttributeValue(uint16_t iid, uint8_t *data, size_t dataLength);


#ifdef __cplusplus
}
#endif

#endif /* _WICED_HOMEKIT_APL_H_ */

