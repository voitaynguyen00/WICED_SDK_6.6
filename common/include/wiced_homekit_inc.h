/***************************************************************************//**
* \file <wiced_homekit_db.h>
*
* \brief
* 	The Bluetooth Low Energy HomeKit accessory include definitions.
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
#ifndef WICED_HOMEKIT_INC_H
#define WICED_HOMEKIT_INC_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
*                    Constants
******************************************************************************/

/* Static data starting addresses */
#define STATIC_DATA_START_ADDR              0x0800
#define STATIC_DATA_SRP_VERIFIER_OFFSET     0x100
#define STATIC_DATA_TOKEN_UUID_OFFSET       0x300
#define STATIC_DATA_TOKEN_SIZE_OFFSET       (STATIC_DATA_TOKEN_UUID_OFFSET + 16)
#define STATIC_DATA_TOKEN_OFFSET            (STATIC_DATA_TOKEN_SIZE_OFFSET + 4)


/* HCI Control API */

/* Apple HomeKit group code */
#define HCI_CONTROL_GROUP_HK                                  0xFE

/* Apple HomeKit commands */
#define HCI_CONTROL_HK_COMMAND_READ                         ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x01 )    /* Read characteristic */
#define HCI_CONTROL_HK_COMMAND_WRITE                        ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x02 )    /* Write characteristic */
#define HCI_CONTROL_HK_COMMAND_LIST                         ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x03 )    /* List all characteristics */
#define HCI_CONTROL_HK_COMMAND_FACTORY_RESET                ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x04 )    /* Factory reset */
#define HCI_CONTROL_HK_COMMAND_GET_TOKEN                    ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x05 )    /* Get software authentication token */

/* Apple HomeKit events */
#define HCI_CONTROL_HK_EVENT_READ_RESPONSE                  ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x01 )    /* Response to read characteristic command */
#define HCI_CONTROL_HK_EVENT_UPDATE                         ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x02 )    /* Characteristic value update */
#define HCI_CONTROL_HK_EVENT_LIST_ITEM                      ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x03 )    /* Characteristic list item */
#define HCI_CONTROL_HK_EVENT_TOKEN_DATA                     ( ( HCI_CONTROL_GROUP_HK << 8 ) | 0x04 )    /* Software token data */

#define HCI_TOKEN_DATA_FLAG_START           0x01
#define HCI_TOKEN_DATA_FLAG_END             0x02
#define HCI_TOKEN_DATA_FLAG_UUID            0x04


#ifdef __cplusplus
}
#endif /* _WICED_HOMEKIT_INC_H_ */

#endif

