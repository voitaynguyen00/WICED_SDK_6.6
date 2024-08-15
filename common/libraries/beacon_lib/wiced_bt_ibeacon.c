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
* Apple iBeacon API implementation
*
* This file implements Apple iBeacon protocol. Application can call the APIs to populate
* the advertisement data. 
*
*/

#include "wiced_bt_stack.h"
#include "wiced_bt_beacon.h"
#include "wiced_bt_trace.h"
#include "string.h"
/******************************************************************************
 *                              Variables Definitions
 ******************************************************************************/

 /* local data used by methods */
static wiced_bt_beacon_ble_advert_elem_t ibeacon_adv_elem[IBEACON_ELEM_NUM];
const uint8_t ibeacon_type[ LEN_UUID_16 ] = { IBEACON_PROXIMITY };
const uint8_t ibeacon_company_id[ LEN_UUID_16 ] = { IBEACON_COMPANY_ID_APPLE };

/******************************************************************************
*                              Function Definitions
******************************************************************************/
void wiced_bt_beacon_set_adv_data(wiced_bt_beacon_ble_advert_elem_t *beacon_adv_elem, uint8_t num_elem,
    uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len);

/*
 * This function creates Apple iBeacon advertising data format. Calling applications provides
 * the uuid, major and minor number, tx power. The function return the advertiment 
 * data (adv_data) and length (adv_len)
 */
void wiced_bt_ibeacon_set_adv_data(uint8_t ibeacon_uuid[LEN_UUID_128], 
                                    uint16_t ibeacon_major_number, 
                                    uint16_t ibeacon_minor_number,
                                    uint8_t tx_power_lcl,
                                    uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len)
{   
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG|BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t ibeacon_data[IBEACON_DATA_LENGTH];

    /* first adv element */
    ibeacon_adv_elem[0].len          = sizeof(uint8_t)+1;
    ibeacon_adv_elem[0].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    ibeacon_adv_elem[0].data[0]       = flag;

    /* Second adv element */
    ibeacon_adv_elem[1].len          = IBEACON_DATA_LENGTH+1;
    ibeacon_adv_elem[1].advert_type  = BTM_BLE_ADVERT_TYPE_MANUFACTURER;

    /* Setting Company Identifier */
    ibeacon_data[0] = ibeacon_company_id[0];
    ibeacon_data[1] = ibeacon_company_id[1];

    /* Setting beacon type */
    ibeacon_data[2] = ibeacon_type[0];
    ibeacon_data[3] = ibeacon_type[1];

    /* Setting the ibeacon UUID in the manufacturer data */
    memcpy( &ibeacon_data[4], ibeacon_uuid, LEN_UUID_128 );    

    /* Setting the Major field */
    ibeacon_data[20] = ibeacon_major_number & 0xff;
    ibeacon_data[21] = (ibeacon_major_number >> 8) & 0xff; 

    /* Setting the Minor field */
    ibeacon_data[22] = ibeacon_minor_number & 0xff; 
    ibeacon_data[23] = (ibeacon_minor_number >> 8) & 0xff;

    /* Measured power */
    ibeacon_data[24] = tx_power_lcl;

    memcpy(ibeacon_adv_elem[1].data, ibeacon_data, IBEACON_DATA_LENGTH);
     
    // Copy the adv data to output buffer
    wiced_bt_beacon_set_adv_data(ibeacon_adv_elem, IBEACON_ELEM_NUM, adv_data, adv_len);
}

