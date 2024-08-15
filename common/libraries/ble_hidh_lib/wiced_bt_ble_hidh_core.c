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

#include "wiced_bt_ble_hidh_core.h"

/******************************************************
 *                 Global Variables
 ******************************************************/
wiced_bt_ble_hidh_cb_t wiced_bt_ble_hidh_cb;

/*
 * wiced_bt_ble_hidh_core_dev_alloc
 * Allocate a structure to save information about a BLE HID device
 */
wiced_bt_ble_hidh_dev_t *wiced_bt_ble_hidh_core_dev_alloc(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    int i;

    p_dev = &wiced_bt_ble_hidh_cb.devices[0];
    for (i = 0 ; i < WICED_BT_BLE_HIDH_DEV_MAX ; i++, p_dev++)
    {
        if ((p_dev->dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ALLOCATED) == 0)
        {
            WICED_BT_BLE_HIDH_TRACE("i:%d\n", i);
            memset(p_dev, 0, sizeof(*p_dev));
            memcpy(p_dev->bdaddr, bdaddr, BD_ADDR_LEN);
            p_dev->dev_state_msk |= WICED_BT_BLE_HIDH_DEV_STATE_ALLOCATED;
            return p_dev;
        }
    }
    WICED_BT_BLE_HIDH_TRACE_ERR("mem full\n", i);
    return NULL;
}

/*
 * wiced_bt_ble_hidh_core_dev_free
 * Free a structure to save information about a BLE HID device
 */
void wiced_bt_ble_hidh_core_dev_free(wiced_bt_ble_hidh_dev_t *p_dev)
{
    WICED_BT_BLE_HIDH_TRACE("i:%d\n", p_dev - &wiced_bt_ble_hidh_cb.devices[0]);

    memset(p_dev, 0, sizeof(*p_dev));
}

/*
 * wiced_bt_ble_hidh_core_dev_from_bdaddr
 * Retrieve a structure containing information about a BLE HID device from it's BdAddr
 */
wiced_bt_ble_hidh_dev_t *wiced_bt_ble_hidh_core_dev_from_bdaddr(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    int i;

    p_dev = &wiced_bt_ble_hidh_cb.devices[0];
    for (i = 0 ; i < WICED_BT_BLE_HIDH_DEV_MAX ; i++, p_dev++)
    {
        if ((p_dev->dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ALLOCATED) &&
            (memcmp(p_dev->bdaddr, bdaddr, BD_ADDR_LEN) == 0))
        {
            WICED_BT_BLE_HIDH_TRACE("bdaddr:%B i:%d\n", bdaddr, i);
            return p_dev;
        }
    }
    WICED_BT_BLE_HIDH_TRACE("unknown dev:%B\n", bdaddr);
    return NULL;
}

/*
 * wiced_bt_ble_hidh_core_dev_from_conn_id
 * Retrieve a structure containing information about a BLE HID device from it's Connection Handle
 */
wiced_bt_ble_hidh_dev_t *wiced_bt_ble_hidh_core_dev_from_conn_id(uint16_t conn_id)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    int i;

    p_dev = &wiced_bt_ble_hidh_cb.devices[0];
    for (i = 0; i < WICED_BT_BLE_HIDH_DEV_MAX; i++, p_dev++)
    {
        if ((p_dev->dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ALLOCATED)
                && (p_dev->conn_id == conn_id))
        {
            WICED_BT_BLE_HIDH_TRACE("conn_id:%d i:%d\n", conn_id, i);
            return p_dev;
        }
    }
    WICED_BT_BLE_HIDH_TRACE_ERR("unknown conn_id:%d\n", conn_id);
    return NULL;
}

/*
 * wiced_bt_ble_hidh_core_gatt_complete
 * This function is called when the BLE HID GATT operation is complete
 */
void wiced_bt_ble_hidh_core_gatt_complete(wiced_bt_ble_hidh_dev_t *p_dev,
        wiced_bt_ble_hidh_status_t status)
{
    wiced_bt_ble_hidh_event_data_t hidh_evt;

    WICED_BT_BLE_HIDH_TRACE("conn_id:%d status:%d\n",
            p_dev->conn_id, status);

    if (status == WICED_BT_BLE_HIDH_STATUS_SUCCESS)
    {
        WICED_BT_BLE_HIDH_TRACE("connected dev:%B\n", p_dev->bdaddr);
        /* Send a BLE HID Open Event */
        memcpy(&hidh_evt.connected.bdaddr, p_dev->bdaddr, BD_ADDR_LEN);
        hidh_evt.connected.handle = p_dev->conn_id + WICED_BT_BLE_HIDH_HANDLE_OFFSET;
        hidh_evt.connected.status = WICED_BT_BLE_HIDH_STATUS_SUCCESS;
        wiced_bt_ble_hidh_core_callback(WICED_BT_BLE_HIDH_OPEN_EVT, &hidh_evt);

        /* Send a BLE HID GATT Cache Event */
        memset(&hidh_evt.gatt_cache, 0, sizeof(&hidh_evt.gatt_cache));
        memcpy(&hidh_evt.gatt_cache.bdaddr, p_dev->bdaddr, BD_ADDR_LEN);
        hidh_evt.gatt_cache.characteristics_nb = p_dev->database.characteristics_nb;
        memcpy(&hidh_evt.gatt_cache.characteristics, p_dev->database.characteristics,
                p_dev->database.characteristics_nb * sizeof(wiced_bt_ble_hidh_gatt_char_t));
        hidh_evt.gatt_cache.report_descs_nb = p_dev->database.report_descs_nb;
        memcpy(&hidh_evt.gatt_cache.report_descs, p_dev->database.report_descs,
                p_dev->database.report_descs_nb * sizeof(wiced_bt_ble_hidh_gatt_report_t));
        wiced_bt_ble_hidh_core_callback(WICED_BT_BLE_HIDH_GATT_CACHE_EVT, &hidh_evt);
    }
    else
    {
        status = wiced_bt_gatt_disconnect(p_dev->conn_id);
        if (status != WICED_BT_BLE_HIDH_STATUS_SUCCESS)
            WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_gatt_disconnect failed:%s", status);
    }
}

/*
 * wiced_bt_ble_hidh_core_callback
 * This function will check (one by one) all the event filter registered.
 * If none 'catch' it, the event will be sent to the application
 */
void wiced_bt_ble_hidh_core_callback(wiced_bt_ble_hidh_event_t event,
        wiced_bt_ble_hidh_event_data_t *p_event_data)
{
    int filter_idx;
    wiced_bool_t event_filtered;

    WICED_BT_BLE_HIDH_TRACE("event:%d\n", event);

    /* Call every event filter installed */
    for (filter_idx = 0 ; filter_idx < WICED_BT_BLE_HIDH_EVENT_FILTER_NB_MAX; filter_idx++)
    {
        if (wiced_bt_ble_hidh_cb.p_filter_callbacks[filter_idx] != NULL)
        {
            /* If this event filter is registered */
            event_filtered = wiced_bt_ble_hidh_cb.p_filter_callbacks[filter_idx](event, p_event_data);
            /* Return as soon as an event filter 'catches' it (return TRUE) */
            if (event_filtered != WICED_FALSE)
                return;
        }
    }

    /* If no Event filter 'caught' the event, send it to the app */
    if (wiced_bt_ble_hidh_cb.p_callback != NULL)
    {
        wiced_bt_ble_hidh_cb.p_callback(event, p_event_data);
    }
}
