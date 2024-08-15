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

#include "wiced_bt_ble_hidh_int.h"
#include "wiced_bt_ble_hidh_core.h"
#include "wiced_bt_ble_hidh_gattc.h"
#include "wiced_bt_ble_hidh_wakeup.h"
#include "wiced_hal_gpio.h"

/*
 * wiced_bt_ble_hidh_init
 * Application must call this function to initialize/start BLE HID library
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_init(wiced_bt_ble_hidh_cback_t *p_callback)
{
    WICED_BT_BLE_HIDH_TRACE("cback:0x%x\n", p_callback);

    if (p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    memset(&wiced_bt_ble_hidh_cb, 0, sizeof(wiced_bt_ble_hidh_cb));
    wiced_bt_ble_hidh_cb.p_callback = p_callback;

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}


/*
 * wiced_bt_ble_hidh_connect
 * Application must call this function to connect a BLE HID Device
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_connect(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_address_type_t addr_type)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bool_t status;

    WICED_BT_BLE_HIDH_TRACE("dev:%B type:%d\n", bdaddr, addr_type);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        p_dev = wiced_bt_ble_hidh_core_dev_alloc(bdaddr);
        if (p_dev == NULL)
        {
            WICED_BT_BLE_HIDH_TRACE_ERR("Mem full dev:%B Mem Full\n", bdaddr);
            return WICED_BT_BLE_HIDH_STATUS_MEM_FULL;
        }
    }
    else
    {
        if (p_dev->conn_id != 0)
        {
            WICED_BT_BLE_HIDH_TRACE_ERR("dev:%B already connected\n", bdaddr);
            return WICED_BT_BLE_HIDH_STATUS_ERROR;
        }
    }

    WICED_BT_BLE_HIDH_TRACE("connect LE dev:%B AddrType:%d\n", bdaddr, addr_type);

    status = wiced_bt_gatt_le_connect(bdaddr, addr_type, BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE);
    if (status != WICED_TRUE)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("dev:%B wiced_bt_gatt_le_connect failed\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

/*
 * wiced_bt_ble_hidh_disconnect
 * Application must call this function to disconnect a BLE HID Device
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_disconnect(uint16_t handle)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_gatt_status_t status;

    WICED_BT_BLE_HIDH_TRACE("handle:%d\n", handle);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_conn_id(handle - WICED_BT_BLE_HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown conn_id:%B\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if (p_dev->conn_id == 0)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("not connected conn_id:%d\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    status = wiced_bt_gatt_disconnect(handle - WICED_BT_BLE_HIDH_HANDLE_OFFSET);
    if (status != WICED_BT_GATT_SUCCESS)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_gatt_le_disconnect failed conn_id:%d\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

/*
 * wiced_bt_ble_hidh_add
 * Application must call this function to add a BLE HID Device (to allow it to reconnect)
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_add(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_address_type_t addr_type, wiced_bt_ble_hidh_gatt_cache_t *p_gatt_cache)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bool_t status;

    WICED_BT_BLE_HIDH_TRACE("address:%B type:%d\n", bdaddr, addr_type);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    /* check if this device is already allocated */
    p_dev = wiced_bt_ble_hidh_core_dev_from_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        p_dev = wiced_bt_ble_hidh_core_dev_alloc(bdaddr);
        if (p_dev == NULL)
        {
            WICED_BT_BLE_HIDH_TRACE_ERR("Mem full dev:%B\n", bdaddr);
            return WICED_BT_BLE_HIDH_STATUS_MEM_FULL;
        }
    }

    /* Save the Address Type */
    p_dev->addr_type = addr_type;

    /* Restore the GATT Cache information (if provided) */
    if(p_gatt_cache)
    {
        if ((p_gatt_cache->characteristics_nb <= WICED_BT_BLE_HIDH_DEV_CHAR_MAX) &&
            (p_gatt_cache->report_descs_nb <= WICED_BT_BLE_HIDH_REPORT_DESC_MAX))
        {
            p_dev->database.characteristics_nb = p_gatt_cache->characteristics_nb;
            memcpy(p_dev->database.characteristics, p_gatt_cache->characteristics,
                    sizeof(p_dev->database.characteristics));
            p_dev->database.report_descs_nb = p_gatt_cache->report_descs_nb;
            memcpy(p_dev->database.report_descs, p_gatt_cache->report_descs,
                    sizeof(p_dev->database.report_descs));
            p_dev->database.db_state = WICED_BT_BLE_HIDH_GATTC_STATE_DONE;

            wiced_bt_ble_hidh_gattc_dump(p_dev);
        }
        else
        {
            WICED_BT_BLE_HIDH_TRACE_ERR("Wrong GATT Cache nb_char:%d nb_report:%d\n",
                    p_gatt_cache->characteristics_nb, p_gatt_cache->report_descs_nb);
        }
    }

    if (p_dev->dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ADDED)
    {
        /* Device already Added. nothing to do */
        WICED_BT_BLE_HIDH_TRACE("already added dev:%B\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
    }

    status = wiced_bt_ble_set_background_connection_type(BTM_BLE_CONN_AUTO, NULL);
    if (status == WICED_FALSE)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_ble_set_background_connection_type failed\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    /* Allow Background connection for this device */
    status = wiced_bt_gatt_le_connect(bdaddr, addr_type, BLE_CONN_MODE_LOW_DUTY, WICED_FALSE);
    if (status == WICED_FALSE)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_gatt_le_connect failed\n");
        return WICED_BT_BLE_HIDH_STATUS_GATT_ERROR;
    }

    p_dev->dev_state_msk |= WICED_BT_BLE_HIDH_DEV_STATE_ADDED;

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

/*
 * wiced_bt_ble_hidh_remove
 * Application must call this function to remove a BLE HID Device (to do not allow it to reconnect)
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_remove(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bool_t status;

    WICED_BT_BLE_HIDH_TRACE("address:%B\n", bdaddr);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("dev:%B unknown\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if (p_dev->conn_id != 0)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("dev:%B connected\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if ((p_dev->dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ADDED) == 0)
    {
        /* Device was not Added */
        WICED_BT_BLE_HIDH_TRACE_ERR("was not added dev:%B\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    /* Do not allow this device to Reconnect (using Background connection) */
    status = wiced_bt_gatt_cancel_connect(bdaddr, WICED_FALSE);
    if (status == WICED_FALSE)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_gatt_cancel_connect failed\n");
        /* Continue (to free the device) even if it fails */
    }

    wiced_bt_ble_hidh_core_dev_free(p_dev);

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

/*
 * wiced_bt_ble_hidh_get_descriptor
 * Application must call this function to retrieve the HID descriptor of a connected BLE HID Device
 */
wiced_bt_gatt_status_t wiced_bt_ble_hidh_get_descriptor(uint16_t handle)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_gatt_status_t status;

    WICED_BT_BLE_HIDH_TRACE("handle:%d\n", handle);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_conn_id(handle - WICED_BT_BLE_HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown conn_id:%B\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if (p_dev->conn_id == 0)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("not connected conn_id:%d\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    /* Read the Descriptor */
    return wiced_bt_ble_hidh_gattc_read_descriptor(p_dev);
}

/*
 * wiced_bt_ble_hidh_gatt_operation_complete
 * Application must call this function when GATT_OPERATION_CPLT_EVT event is received
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_set_report(uint16_t handle,
        wiced_bt_ble_hidh_report_type_t report_type, uint8_t report_id, uint8_t *p_data,
        uint16_t length)
{
    wiced_bt_ble_hidh_dev_t *p_dev;

    WICED_BT_BLE_HIDH_TRACE("handle:%d type:%d id:0x%x len:%d%\n", handle, report_type, report_id,
            length);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_conn_id(handle - WICED_BT_BLE_HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown conn_id:%B\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if (p_dev->conn_id == 0)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("not connected conn_id:%d\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    /* Set the Report */
    return wiced_bt_ble_hidh_gattc_set_report(p_dev, report_type, report_id, p_data, length);
}

/*
 * wiced_bt_ble_hidh_get_report
 * Application calls this function to Get and HID Report
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_get_report(uint16_t handle,
        wiced_bt_ble_hidh_report_type_t report_type, uint8_t report_id, uint16_t length)
{
    wiced_bt_ble_hidh_dev_t *p_dev;

    WICED_BT_BLE_HIDH_TRACE("handle:%d type:%d id:0x%x len:%d%\n", handle, report_type, report_id,
            length);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_conn_id(handle - WICED_BT_BLE_HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown conn_id:%B\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if (p_dev->conn_id == 0)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("not connected conn_id:%d\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    /* Get the Report */
    return wiced_bt_ble_hidh_gattc_get_report(p_dev, report_type, report_id, length);
}

/*
 * wiced_bt_ble_hidh_set_protocol
 * Application calls this function to Set the HID Protocol
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_set_protocol(uint16_t handle,
        wiced_bt_ble_hidh_protocol_t protocol)
{
    wiced_bt_ble_hidh_dev_t *p_dev;

    WICED_BT_BLE_HIDH_TRACE("handle:%d protocol:%d%\n", handle, protocol);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_conn_id(handle - WICED_BT_BLE_HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown conn_id:%B\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if (p_dev->conn_id == 0)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("not connected conn_id:%d\n", handle);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    /* Set the Protocol */
    return wiced_bt_ble_hidh_gattc_set_protocol(p_dev, protocol);
}

/*
 * wiced_bt_ble_hidh_up
 * Application must call this function to when a BLE connection is established
 */
void wiced_bt_ble_hidh_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_ble_hidh_event_data_t hidh_evt;
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_ble_hidh_status_t status;
    wiced_result_t result;
    wiced_bt_ble_sec_action_type_t encryption_type = BTM_BLE_SEC_ENCRYPT;

    WICED_BT_BLE_HIDH_TRACE("address:%B conn_id:%d", p_conn_status->bd_addr,
            p_conn_status->conn_id);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return;
    }

    if (p_conn_status->transport != BT_TRANSPORT_LE)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("not LE dev:%B\n", p_conn_status->bd_addr);
        return;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_bdaddr(p_conn_status->bd_addr);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("dev:%B unknown\n", p_conn_status->bd_addr);
        return;
    }

    /* Connection success, Save Connection Id */
    p_dev->conn_id = p_conn_status->conn_id;
    p_dev->dev_state_msk |= WICED_BT_BLE_HIDH_DEV_STATE_CONNECTED;

    /* If the link is not yet Encrypted */
    if ((p_dev->dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ENCRYPTED) == 0)
    {

        if ((p_dev->dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ADDED) == 0)
        {
            WICED_BT_BLE_HIDH_TRACE("wiced_bt_dev_sec_bond dev:%B\n", p_conn_status->bd_addr);
            result = wiced_bt_dev_sec_bond (p_dev->bdaddr, p_dev->addr_type, BT_TRANSPORT_LE, 0, NULL);
            if (result == WICED_BT_PENDING)
            {
                /* Pairing Started. The Stack will automatically start Encryption */
                WICED_BT_BLE_HIDH_TRACE("wiced_bt_dev_sec_bond pending\n");
                return;
            }
            else if (result == WICED_BT_SUCCESS)
            {
                /* Already Paired???. */
                WICED_BT_BLE_HIDH_TRACE("already Paired???\n");
            }
            else
            {
                /* Pairing failed to start. Disconnect link */
                WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_dev_sec_bond failed\n");
                wiced_bt_gatt_disconnect(p_dev->conn_id);
                return;
            }
        }

        /* Already Paired. Let's Start Encryption */
        WICED_BT_BLE_HIDH_TRACE("wiced_bt_dev_set_encryption\n");
        result = wiced_bt_dev_set_encryption (p_dev->bdaddr, BT_TRANSPORT_LE, &encryption_type);
        if (result == WICED_BT_PENDING)
        {
            /* Encryption Started */
            WICED_BT_BLE_HIDH_TRACE("wiced_bt_dev_set_encryption pending\n");
            return;
        }
        else if (result == WICED_BT_SUCCESS)
        {
            /* Already Encrypted ???. Let's continue */
            p_dev->dev_state_msk |= WICED_BT_BLE_HIDH_DEV_STATE_ENCRYPTED;
        }
        else
        {
            /* Encryption failed to start. Disconnect link */
            WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_dev_set_encryption failed\n");
            wiced_bt_gatt_disconnect(p_dev->conn_id);
            return;
        }
    }

    /*
     * The link is already Encrypted
     */

    /* If the database is empty, discover it */
    if (p_dev->database.db_state == WICED_BT_BLE_HIDH_GATTC_STATE_EMPTY)
    {
        status =  wiced_bt_ble_hidh_gattc_search(p_dev);
        if (status != WICED_BT_BLE_HIDH_STATUS_SUCCESS)
        {
            WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_ble_hidh_gatt_search failed\n");
            /* Disconnect the link */
            wiced_bt_gatt_disconnect(p_dev->conn_id);
        }
        return;
    }

    WICED_BT_BLE_HIDH_TRACE("connected dev:%B\n", p_conn_status->bd_addr);
    memcpy(&hidh_evt.connected.bdaddr, p_conn_status->bd_addr, BD_ADDR_LEN);
    hidh_evt.connected.handle = p_conn_status->conn_id + WICED_BT_BLE_HIDH_HANDLE_OFFSET;
    hidh_evt.connected.status = WICED_BT_BLE_HIDH_STATUS_SUCCESS;
    wiced_bt_ble_hidh_core_callback(WICED_BT_BLE_HIDH_OPEN_EVT, &hidh_evt);

    return;
}


/*
 * wiced_bt_ble_hidh_down
 * Application must call this function to when a BLE connection is released
 */
void wiced_bt_ble_hidh_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_ble_hidh_event_data_t hidh_evt;
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_ble_hidh_status_t status;
    wiced_bt_ble_hidh_dev_state_t dev_state_msk;

    WICED_BT_BLE_HIDH_TRACE("BdAddr:%B reason:%d\n",
            p_conn_status->bd_addr, p_conn_status->reason);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return;
    }

    if (p_conn_status->transport != BT_TRANSPORT_LE)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("not LE dev:%B\n", p_conn_status->bd_addr);
        return;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_bdaddr(p_conn_status->bd_addr);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("unknown BdAddr:%B\n", p_conn_status->bd_addr);
        return;
    }

    /* Save the device's state */
    dev_state_msk = p_dev->dev_state_msk;

    /* If this device has not been 'added', free it */
    if ((dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ADDED) == 0)
    {
        wiced_bt_ble_hidh_core_dev_free(p_dev);
    }
    else
    {
        /* Reset any state information excepted the Allocated and Added */
        p_dev->dev_state_msk = WICED_BT_BLE_HIDH_DEV_STATE_ALLOCATED |
                               WICED_BT_BLE_HIDH_DEV_STATE_ADDED;
        p_dev->conn_id = 0;
    }

    /* If the device was connected, it's a disconnection */
    if (dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_CONNECTED)
    {
        hidh_evt.disconnected.handle = p_conn_status->conn_id + WICED_BT_BLE_HIDH_HANDLE_OFFSET;
        hidh_evt.disconnected.reason = p_conn_status->reason;
        wiced_bt_ble_hidh_core_callback(WICED_BT_BLE_HIDH_CLOSE_EVT, &hidh_evt);
    }
    /* Else, the device was not yet connected, it's an outgoing connection failure */
    else
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("connection failed dev:%B\n", p_conn_status->bd_addr);
        memcpy(&hidh_evt.connected.bdaddr, p_conn_status->bd_addr, BD_ADDR_LEN);
        hidh_evt.connected.handle = 0;
        hidh_evt.connected.status = WICED_BT_BLE_HIDH_STATUS_CONNECTION_FAILED;
        wiced_bt_ble_hidh_core_callback(WICED_BT_BLE_HIDH_OPEN_EVT, &hidh_evt);
    }
}

/*
 * wiced_bt_ble_hidh_encryption_changed
 * Application must call this function to when a Encryption changes
 */
void wiced_bt_ble_hidh_encryption_changed(wiced_bt_dev_encryption_status_t *p_encryption_changed)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_gatt_status_t status;
    wiced_bt_ble_hidh_event_data_t hidh_evt;

    /* Ignore BR/EDR Encryption Changed Event */
    if (p_encryption_changed->transport != BT_TRANSPORT_LE)
    {
        return;
    }

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_bdaddr(p_encryption_changed->bd_addr);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("BdAddr:%B\n",
                p_encryption_changed->bd_addr);
        return;
    }

    if (p_encryption_changed->result != WICED_BT_SUCCESS)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Failed result:%d\n", p_encryption_changed->result);
        /* Disconnect the link */
        wiced_bt_gatt_disconnect(p_dev->conn_id);

        // remove it from HIDH database
        wiced_bt_ble_hidh_remove(p_encryption_changed->bd_addr);
        return;
    }

    p_dev->dev_state_msk |= WICED_BT_BLE_HIDH_DEV_STATE_ENCRYPTED;

    WICED_BT_BLE_HIDH_TRACE("Success\n");

    /* If the database is empty, discover it */
    if (p_dev->database.db_state == WICED_BT_BLE_HIDH_GATTC_STATE_EMPTY)
    {
        status =  wiced_bt_ble_hidh_gattc_search(p_dev);
        if (status != WICED_BT_BLE_HIDH_STATUS_SUCCESS)
        {
            WICED_BT_BLE_HIDH_TRACE_ERR("wiced_bt_ble_hidh_gatt_search failed\n");
            /* Disconnect the link */
            wiced_bt_gatt_disconnect(p_dev->conn_id);
        }
        return;
    }

    WICED_BT_BLE_HIDH_TRACE("connected dev:%B\n", p_dev->bdaddr);
    memcpy(&hidh_evt.connected.bdaddr, p_dev->bdaddr, BD_ADDR_LEN);
    hidh_evt.connected.handle = p_dev->conn_id + WICED_BT_BLE_HIDH_HANDLE_OFFSET;
    hidh_evt.connected.status = WICED_BT_BLE_HIDH_STATUS_SUCCESS;
    wiced_bt_ble_hidh_core_callback(WICED_BT_BLE_HIDH_OPEN_EVT, &hidh_evt);
}

/*
 * wiced_bt_ble_hidh_gatt_discovery_result
 * Application must call this function when GATT_DISCOVERY_RESULT_EVT event is received
 */
wiced_bt_gatt_status_t wiced_bt_ble_hidh_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_discovery_result)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_gatt_status_t status;

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_GATT_WRONG_STATE;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_conn_id(p_discovery_result->conn_id);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown conn_id:%d\n",
                p_discovery_result->conn_id);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    return wiced_bt_ble_hidh_gattc_discovery_result(p_dev, p_discovery_result);
}


/*
 * wiced_bt_ble_hidh_gatt_discovery_complete
 * Application must call this function when GATT_DISCOVERY_CPLT_EVT event is received
 */
wiced_bt_gatt_status_t wiced_bt_ble_hidh_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_discovery_complete)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_gatt_status_t status;

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_GATT_WRONG_STATE;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_conn_id(p_discovery_complete->conn_id);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown conn_id:%d\n",
                p_discovery_complete->conn_id);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    return wiced_bt_ble_hidh_gattc_discovery_complete(p_dev, p_discovery_complete);
}


/*
 * wiced_bt_ble_hidh_gatt_operation_complete
 * Application must call this function when GATT_OPERATION_CPLT_EVT event is received
 */
wiced_bt_gatt_status_t wiced_bt_ble_hidh_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_operation_complete)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_gatt_status_t status;

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_GATT_WRONG_STATE;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_conn_id(p_operation_complete->conn_id);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown conn_id:%d\n", p_operation_complete->conn_id);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    return wiced_bt_ble_hidh_gattc_operation_complete(p_dev, p_operation_complete);
}

/*
 * wiced_bt_ble_hidh_wakeup_pattern_set
 * Application call this function to set the WakeUp pattern
 * This function must be called after the wiced_bt_ble_hidh_add function is called
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_wakeup_pattern_set(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_hidh_wakeup_pattern_cmd_t command, uint16_t report_id,
        uint8_t *p_pattern, uint16_t pattern_len)
{
    wiced_bt_ble_hidh_dev_t *p_dev;
    wiced_bt_gatt_status_t status;

    WICED_BT_BLE_HIDH_TRACE("dev:%B cmd:%d report_id:0x%x\n", bdaddr, command, report_id);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_ble_hidh_core_dev_from_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown device:%B\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if ((p_dev->dev_state_msk & WICED_BT_BLE_HIDH_DEV_STATE_ADDED) == 0)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("device:%B not added\n", bdaddr);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    if (command == WICED_BT_BLE_HIDH_WAKEUP_PATTERN_CMD_ADD)
    {
        status = wiced_bt_ble_hidh_wakeup_pattern_add(p_dev, report_id, p_pattern, pattern_len);
    }
    else if (command == WICED_BT_BLE_HIDH_WAKEUP_PATTERN_CMD_DEL)
    {
        status =WICED_BT_BLE_HIDH_STATUS_NOT_YET_IMPLEMENTED;
    }
    else if (command == WICED_BT_BLE_HIDH_WAKEUP_PATTERN_CMD_LIST)
    {
        status =WICED_BT_BLE_HIDH_STATUS_NOT_YET_IMPLEMENTED;
    }
    else
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Unknown command:%d\n", command);
        status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    return status;
}

/*
 * wiced_bt_ble_hidh_wakeup_pattern_control
 * Application call this function to enable/disable the WakeUp
 * This function is typically called after wiced_bt_ble_hidh_wakeup_pattern_set
 * The p_data and data_len, could be used (later) to add additional GPIO Control (duration,
 * pattern, etc.).
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_wakeup_pattern_control(wiced_bool_t enable,
        uint32_t gpio_num, uint8_t polarity, uint8_t *p_data, uint8_t data_len)
{
    WICED_BT_BLE_HIDH_TRACE("enable:%d gpio:%d polarity:%x\n", enable, gpio_num, polarity);

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    wiced_bt_ble_hidh_cb.wakeup_control.enable = enable;

    if (enable)
    {
        /* Save GPIO information */
        wiced_bt_ble_hidh_cb.wakeup_control.gpio_num = gpio_num;
        wiced_bt_ble_hidh_cb.wakeup_control.polarity = polarity;
    }

    /* Configure the GPIO as Output and desassert it */
    wiced_hal_gpio_configure_pin( gpio_num, GPIO_OUTPUT_ENABLE, polarity?0:1);

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

/*
 * wiced_bt_ble_hidh_filter_register
 * BLE HID Host libraries (e.g. BLE HID Host Audio) use this function to register an event filter
 * function (to filter/catch BLE HID Host events)
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_filter_register(
        wiced_bt_ble_hidh_filter_cback_t *p_filter_callback)
{
    int filter_idx;

    WICED_BT_BLE_HIDH_TRACE("\n");

    if (wiced_bt_ble_hidh_cb.p_callback == NULL)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("no callback\n");
        return WICED_BT_BLE_HIDH_STATUS_ERROR;
    }

    /* Search for a free Event Filter entry */
    for (filter_idx = 0 ; filter_idx < WICED_BT_BLE_HIDH_EVENT_FILTER_NB_MAX; filter_idx++)
    {
        if (wiced_bt_ble_hidh_cb.p_filter_callbacks[filter_idx] == NULL)
        {
            /* Register the Filter callback */
            wiced_bt_ble_hidh_cb.p_filter_callbacks[filter_idx] = p_filter_callback;
            return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
        }
    }
    WICED_BT_BLE_HIDH_TRACE_ERR("Mem Full (max:%d)\n", WICED_BT_BLE_HIDH_EVENT_FILTER_NB_MAX);
    return WICED_BT_BLE_HIDH_STATUS_MEM_FULL;
}

