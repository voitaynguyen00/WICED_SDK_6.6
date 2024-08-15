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
*  Creates Google Eddystone format advertising data
*
*/
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_timer.h"
#include "wiced_bt_beacon.h"

/******************************************************************************
*                               Functions
******************************************************************************/
static void wiced_bt_eddystone_set_data_common(wiced_bt_beacon_ble_advert_elem_t *eddystone_adv_elem, uint8_t frame_type, uint8_t frame_len);
void wiced_bt_beacon_set_adv_data(wiced_bt_beacon_ble_advert_elem_t *beacon_adv_elem, uint8_t num_elem,uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len);
/******************************************************************************
 *                               Constants
 ******************************************************************************/
/* Google Eddystone */
static wiced_bt_beacon_ble_advert_elem_t eddystone_adv_elem[EDDYSTONE_ELEM_NUM];

/*
 * This function creates Google Eddystone UID format advertising data
 */
void wiced_bt_eddystone_set_data_for_uid(uint8_t eddystone_ranging_data, 
                                        uint8_t eddystone_namespace[EDDYSTONE_UID_NAMESPACE_LEN],  
                                        uint8_t eddystone_instance[EDDYSTONE_UID_INSTANCE_ID_LEN], 
                                        uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len)
{       
    uint8_t frame_data[EDDYSTONE_UID_FRAME_LEN];
    
    WICED_BT_TRACE("wiced_bt_eddystone_set_data_for_uid\n");

    // Set common portion of the adv data
    wiced_bt_eddystone_set_data_common(eddystone_adv_elem, EDDYSTONE_FRAME_TYPE_UID, EDDYSTONE_UID_FRAME_LEN);

    // Set frame data
    frame_data[0] = EDDYSTONE_FRAME_TYPE_UID;
    frame_data[1] = eddystone_ranging_data;
    memcpy( &frame_data[2], eddystone_namespace, EDDYSTONE_UID_NAMESPACE_LEN );
    memcpy( &frame_data[12], eddystone_instance, EDDYSTONE_UID_INSTANCE_ID_LEN );

    memcpy(&(eddystone_adv_elem[2].data[2]), frame_data, EDDYSTONE_UID_FRAME_LEN);

    // Copy the adv data to output buffer
    wiced_bt_beacon_set_adv_data(eddystone_adv_elem, EDDYSTONE_ELEM_NUM, adv_data, adv_len);
   
}

/*
 * This function creates Google Eddystone URL format advertising data
 */
void wiced_bt_eddystone_set_data_for_url(uint8_t tx_power, 
                                        uint8_t urlscheme, 
                                        uint8_t encoded_url[EDDYSTONE_URL_VALUE_MAX_LEN],
                                        uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len)
{
    uint8_t frame_data[EDDYSTONE_URL_FRAME_LEN];
    uint8_t len = strlen((char *)encoded_url);

    WICED_BT_TRACE("eddystone_set_data_for_url_adv\n");

    // Set common portion of the adv data
    wiced_bt_eddystone_set_data_common(eddystone_adv_elem, EDDYSTONE_FRAME_TYPE_URL, len + 3);

    // Set frame data
    frame_data[0] = EDDYSTONE_FRAME_TYPE_URL;
    frame_data[1] = tx_power;
    frame_data[2] = urlscheme;
    memcpy(&frame_data[3], encoded_url, len);
    
    memcpy(&(eddystone_adv_elem[2].data[2]), frame_data, len+3);

    // Copy the adv data to output buffer
    wiced_bt_beacon_set_adv_data(eddystone_adv_elem, EDDYSTONE_ELEM_NUM, adv_data, adv_len);
}

/*
 * This function creates Google Eddystone EID format advertising data
 */
void wiced_bt_eddystone_set_data_for_eid(uint8_t eddystone_ranging_data, 
                                                uint8_t eid[EDDYSTONE_EID_LEN], 
                                                uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len)
{
    uint8_t frame_data[EDDYSTONE_EID_FRAME_LEN];

    WICED_BT_TRACE("eddystone_set_data_for_uid_adv\n");

    // Set common portion of the adv data
    wiced_bt_eddystone_set_data_common(eddystone_adv_elem, EDDYSTONE_FRAME_TYPE_EID, EDDYSTONE_EID_FRAME_LEN);

    // Set frame data
    frame_data[0] = EDDYSTONE_FRAME_TYPE_EID;
    frame_data[1] = eddystone_ranging_data;
    memcpy( &frame_data[2], eid, EDDYSTONE_EID_LEN );

    memcpy(&(eddystone_adv_elem[2].data[2]), frame_data, EDDYSTONE_EID_FRAME_LEN);

    // Copy the adv data to output buffer
    wiced_bt_beacon_set_adv_data(eddystone_adv_elem, EDDYSTONE_ELEM_NUM, adv_data, adv_len);
}

/*
* This function creates Google Eddystone TLM unencrypted format advertising data
*/
void wiced_bt_eddystone_set_data_for_tlm_unencrypted(uint16_t vbatt, 
                                                            uint16_t temp, 
                                                            uint32_t adv_cnt, 
                                                            uint32_t sec_cnt, 
                                                            uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len)
{
    uint8_t frame_data[EDDYSTONE_TLM_UNENCRYPTED_FRAME_LEN];

    WICED_BT_TRACE("wiced_bt_eddystone_set_data_for_tlm_unencrypted\n");

    // Set common portion of the adv data
    wiced_bt_eddystone_set_data_common(eddystone_adv_elem, EDDYSTONE_FRAME_TYPE_TLM, EDDYSTONE_TLM_UNENCRYPTED_FRAME_LEN);

    // Set frame data
    frame_data[0] = EDDYSTONE_FRAME_TYPE_TLM;
    frame_data[1] = EDDYSTONE_TLM_UNENCRYPTED_VERSION;
    frame_data[2] = vbatt & 0xff;
    frame_data[3] = (vbatt >> 8) & 0xff;
    frame_data[4] = temp & 0xff;
    frame_data[5] = (temp >> 8) & 0xff;
    frame_data[6] = adv_cnt & 0xff;
    frame_data[7] = (adv_cnt >> 8) & 0xff;
    frame_data[8] = (adv_cnt >> 16) & 0xff;
    frame_data[9] = (adv_cnt >> 24) & 0xff;
    frame_data[10] = sec_cnt & 0xff;
    frame_data[11] = (sec_cnt >> 8) & 0xff;
    frame_data[12] = (sec_cnt >> 16) & 0xff;
    frame_data[13] = (sec_cnt >> 24) & 0xff;

    memcpy(&(eddystone_adv_elem[2].data[2]), frame_data, EDDYSTONE_TLM_UNENCRYPTED_FRAME_LEN);

    // Copy the adv data to output buffer
    wiced_bt_beacon_set_adv_data(eddystone_adv_elem, EDDYSTONE_ELEM_NUM, adv_data, adv_len);
}

/*
* This function creates Google Eddystone TLM encrypted format advertising data
*/
void wiced_bt_eddystone_set_data_for_tlm_encrypted(uint8_t etlm[EDDYSTONE_ETLM_LEN], 
                                                          uint16_t salt, uint16_t mic, 
                                                          uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len)
{
    uint8_t frame_data[EDDYSTONE_TLM_ENCRYPTED_FRAME_LEN];

    WICED_BT_TRACE("wiced_bt_eddystone_set_data_for_tlm_encrypted\n");

    // Set common portion of the adv data
    wiced_bt_eddystone_set_data_common(eddystone_adv_elem, EDDYSTONE_FRAME_TYPE_TLM, EDDYSTONE_TLM_ENCRYPTED_FRAME_LEN);

    // Set frame data
    frame_data[0] = EDDYSTONE_FRAME_TYPE_TLM;
    frame_data[1] = EDDYSTONE_TLM_ENCRYPTED_VERSION;
    memcpy(&frame_data[2], etlm, EDDYSTONE_ETLM_LEN);
    frame_data[14] = salt & 0xff;
    frame_data[15] = (salt >> 8) & 0xff;
    frame_data[16] = mic & 0xff;
    frame_data[17] = (mic >> 8) & 0xff;
        
    memcpy(&(eddystone_adv_elem[2].data[2]), frame_data, EDDYSTONE_TLM_ENCRYPTED_FRAME_LEN);

    // Copy the adv data to output buffer
    wiced_bt_beacon_set_adv_data(eddystone_adv_elem, EDDYSTONE_ELEM_NUM, adv_data, adv_len);
}

/* Sets up data common for all Eddystone frames */
static void wiced_bt_eddystone_set_data_common(wiced_bt_beacon_ble_advert_elem_t *eddystone_adv_elem, uint8_t frame_type, uint8_t frame_len)
{
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t eddyston_uuid[LEN_UUID_16] = { BIT16_TO_8(EDDYSTONE_UUID16) };

    // First adv element
    eddystone_adv_elem[0].len = 2;
    eddystone_adv_elem[0].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    eddystone_adv_elem[0].data[0] = flag;

    // Second adv element
    eddystone_adv_elem[1].len = 3;
    eddystone_adv_elem[1].advert_type = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    memcpy(eddystone_adv_elem[1].data, eddyston_uuid, 2);

    // Third adv element (partial), rest is frame specific
    eddystone_adv_elem[2].len = frame_len + 3;  // frame_len + advert_type (1) + uuid (2)
    eddystone_adv_elem[2].advert_type = BTM_BLE_ADVERT_TYPE_SERVICE_DATA;
    eddystone_adv_elem[2].data[0] = eddyston_uuid[0];
    eddystone_adv_elem[2].data[1] = eddyston_uuid[1];
}
