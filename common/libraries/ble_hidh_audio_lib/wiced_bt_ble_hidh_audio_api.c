/*
 *  Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
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

#include "wiced_bt_ble_hidh_audio_int.h"

/*
 * wiced_bt_ble_hidh_audio_init
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_audio_init(
        wiced_bt_ble_hidh_audio_cback_t *p_callback)
{
    wiced_bt_ble_hidh_status_t status;
    wiced_bt_ble_hidh_audio_sink_cb_t *p_sink_cb = &wiced_bt_ble_hidh_audio_cb.sink;

    WICED_BT_BLE_HIDH_AUDIO_TRACE("cback:0x%x\n", p_callback);

    if (p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    memset(&wiced_bt_ble_hidh_audio_cb, 0, sizeof(wiced_bt_ble_hidh_audio_cb));
    wiced_bt_ble_hidh_audio_cb.sink.handle = WICED_BT_BLE_HIDH_AUDIO_INVALID_HANDLE;

    /* Initialize mSBC Decoder */
    status = wiced_bt_ble_hidh_audio_sbc_dec_init();
    if (status != WICED_BT_BLE_HIDH_STATUS_SUCCESS)
    {
        return status;
    }

    /* Register the BLE HIDH Audio event filter */
    status = wiced_bt_ble_hidh_filter_register(wiced_bt_ble_hidh_audio_cback);
    if (status != WICED_BT_BLE_HIDH_STATUS_SUCCESS)
    {
        return status;
    }

    wiced_bt_ble_hidh_audio_cb.p_callback = p_callback;

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

