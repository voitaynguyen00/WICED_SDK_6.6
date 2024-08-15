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
 * WICED HID Host interface
 *
 */

#include <string.h>
#include "wiced_bt_hidh.h"
#include "wiced_bt_hidh_int.h"
#include "wiced_bt_hidh_con.h"
#include "wiced_bt_hidh_sdp.h"
#include "wiced_bt_hidh_core.h"
#include "wiced_bt_hidh_utils.h"

/*
 * Definitions
 */

/*
 * Types
 */
/*
 * Globals
 */
wiced_bt_hidh_cb_t wiced_bt_hidh_cb;

/*
 * Local functions
 */

/*
 *  wiced_bt_hidh_init
 *  HID Host Control Block initialization
 */
void wiced_bt_hidh_init(void)
{
    memset(&wiced_bt_hidh_cb, 0, sizeof(wiced_bt_hidh_cb));
    wiced_bt_hidh_con_init();
}

/*
 *  wiced_bt_hidh_enable
 *  Start HID Host (register application callback and configure L2CAP channels)
 */
wiced_bt_hidh_status_t wiced_bt_hidh_enable(wiced_bt_hidh_cback_t *p_cback)
{
    wiced_bt_hidh_status_t status;

    HIDH_TRACE_DBG("wiced_bt_hidh_enable\n");

    if (wiced_bt_hidh_cb.p_cback)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_enable failed (not initialized or already enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    status = wiced_bt_hidh_con_register(wiced_bt_hidh_con_cback);
    if (status != WICED_BT_HIDH_STATUS_OK)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_enable wiced_bt_hidh_con_register failed\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    /* Save the application callback */
    wiced_bt_hidh_cb.p_cback = p_cback;


    return WICED_BT_HIDH_STATUS_OK;
}

/*
 *  wiced_bt_hidh_open
 *  Open an HID Host connection to a peer device.
 *  This function will start with a SDP Request to verify that it supports HID Device profile.
 *  It will then open the two l2CAP channels (Control and Interrupt)
 */
wiced_bt_hidh_status_t wiced_bt_hidh_open(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_hidh_dev_t *p_dev;

    HIDH_TRACE_DBG("wiced_bt_hidh_open [%B]\n", bdaddr);

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_open HIDH Not enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    /* Check if this device is present in the device list */
    p_dev = wiced_bt_hidh_dev_get_by_bdaddr(bdaddr);
    if (p_dev)
    {
        if ((p_dev->state == WICED_BT_HIDH_STATE_IN_CONNECTING) ||
            (p_dev->state == WICED_BT_HIDH_STATE_OUT_CONNECTING) ||
            (p_dev->state == WICED_BT_HIDH_STATE_CONNECTED))
        {
            HIDH_TRACE_DBG("Err: wiced_bt_hidh_open bad state:%d\n", p_dev->state);
            return WICED_BT_HIDH_STATUS_ERROR;
        }
    }
    else
    {
        p_dev = wiced_bt_hidh_dev_alloc(bdaddr);
        if (p_dev == NULL)
        {
            HIDH_TRACE_DBG("Err: wiced_bt_hidh_open Mem full\n");
            return WICED_BT_HIDH_STATUS_MEM_FULL;
        }
    }

    wiced_bt_hidh_dev_set_state(p_dev, WICED_BT_HIDH_STATE_OUT_CONNECTING);


    /* Start SDP Request to check if peer device contains an HID Device SDP Record */
    if (wiced_bt_hidh_sdp_discover(p_dev) != WICED_BT_HIDH_STATUS_OK)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_open SDP failed\n");
        if (p_dev->added == WICED_FALSE)
        {
            wiced_bt_hidh_dev_free(p_dev);
        }
        else
        {
            wiced_bt_hidh_dev_set_state(p_dev, WICED_BT_HIDH_STATE_REGISTERED);
        }
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_close
 * This function will disconnect the two L2CAP channels of an HID Connection.
 * Once the two L2CAP channels are disconnected the device will be erased if it has not been
 * previously 'added' by the application (via wiced_bt_hidh_add).
 */
wiced_bt_hidh_status_t wiced_bt_hidh_close(uint16_t handle)
{
    wiced_bt_hidh_dev_t *p_dev;

    HIDH_TRACE_DBG("wiced_bt_hidh_close handle:%d\n", handle);

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_close HIDH Not enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    if (handle >= WICED_BT_HIDH_NUM_DEV_MAX)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_close bad handle:0x%x\n", handle);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    p_dev = &wiced_bt_hidh_cb.devices[handle];
    if (wiced_bt_hidh_dev_get_state(p_dev) != WICED_BT_HIDH_STATE_CONNECTED)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_close device handle:0x%x not connected\n", handle);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    wiced_bt_hidh_dev_set_state(p_dev, WICED_BT_HIDH_STATE_DISCONNECTING);

    return wiced_bt_hidh_core_close(p_dev);
}

/*
 * wiced_bt_hidh_add
 * This function allocates an HID Device in the HID database to allow it to reconnect.
 */
wiced_bt_hidh_status_t wiced_bt_hidh_add(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_hidh_dev_t *p_dev;

    HIDH_TRACE_DBG("wiced_bt_hidh_add [%B]\n", bdaddr);

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_add HIDH Not enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    /* Check if this device is known (either connected or registered) */
    p_dev = wiced_bt_hidh_dev_get_by_bdaddr(bdaddr);
    if (p_dev)
    {
        p_dev->added = WICED_TRUE;
        return WICED_BT_HIDH_STATUS_OK;
    }

    p_dev = wiced_bt_hidh_dev_alloc(bdaddr);
    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_add Mem full\n");
        return WICED_BT_HIDH_STATUS_MEM_FULL;
    }

    wiced_bt_hidh_dev_set_state(p_dev, WICED_BT_HIDH_STATE_REGISTERED);
    p_dev->added = WICED_TRUE;

    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_remove
 * This function free an HID Device from the HID database to do not allow it to reconnect.
 */
wiced_bt_hidh_status_t wiced_bt_hidh_remove(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_hidh_dev_t *p_dev;

    HIDH_TRACE_DBG("wiced_bt_hidh_remove [%B]\n", bdaddr);

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_remove HIDH Not enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    p_dev = wiced_bt_hidh_dev_get_by_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_remove unknown device\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    p_dev->added = WICED_FALSE;

    if (wiced_bt_hidh_dev_get_state(p_dev) == WICED_BT_HIDH_STATE_CONNECTED)
    {
        wiced_bt_hidh_close(wiced_bt_hidh_dev_get_handle(p_dev));
    }
    else
    {
        wiced_bt_hidh_dev_free(p_dev);
    }

    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_get_descriptor
 * This function will perform an SDP Request (to a connected device) to retrieve it's HID
 * Descriptor (describing how to parse its HID Reports)
 */
wiced_bt_hidh_status_t wiced_bt_hidh_get_descriptor(uint16_t handle)
{
    wiced_bt_hidh_dev_t *p_dev;

    HIDH_TRACE_DBG("wiced_bt_hidh_get_descriptor handle:0x%x\n", handle);

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_get_descriptor HIDH Not enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    if (handle >= WICED_BT_HIDH_NUM_DEV_MAX)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_get_descriptor bad handle:0x%x\n", handle);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    p_dev = &wiced_bt_hidh_cb.devices[handle];
    if (wiced_bt_hidh_dev_get_state(p_dev) != WICED_BT_HIDH_STATE_CONNECTED)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_get_descriptor device handle:0x%x not connected\n",
                handle);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    return wiced_bt_hidh_sdp_get_descriptor(p_dev);
}

/*
 * wiced_bt_hidh_set_report
 * This function will Set (write) an HID Report in a connected HID Device.
 * This function can be used, for example, to control a Led (e.g. NumLock) or for other feedback.
 */
wiced_bt_hidh_status_t wiced_bt_hidh_set_report(uint16_t handle, wiced_bt_hidh_channel_t channel,
        wiced_bt_hidh_report_type_t type, uint8_t report_id, uint8_t *p_data, uint16_t length)
{
    wiced_bt_hidh_dev_t *p_dev;

    HIDH_TRACE_DBG("wiced_bt_hidh_set_report handle:0x%x Channel:%d Type:%d Id:0x%x length:%d\n",
            handle, channel, type, report_id, length);

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_set_report HIDH Not enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    if (handle >= WICED_BT_HIDH_NUM_DEV_MAX)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_set_report bad handle:0x%x\n", handle);
        return WICED_BT_HIDH_STATUS_INVALID_PARAM;
    }

    p_dev = &wiced_bt_hidh_cb.devices[handle];
    if (wiced_bt_hidh_dev_get_state(p_dev) != WICED_BT_HIDH_STATE_CONNECTED)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_set_report device handle:0x%x not connected\n", handle);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    return wiced_bt_hidh_core_set_report(p_dev, channel, type, report_id, p_data, length);
}

/*
 * wiced_bt_hidh_get_report
 * This function will Get (read) an HID Report from a connected HID Device.
 */
wiced_bt_hidh_status_t wiced_bt_hidh_get_report(uint16_t handle,
        wiced_bt_hidh_report_type_t type, uint8_t report_id, uint16_t length)
{
    wiced_bt_hidh_dev_t *p_dev;

    HIDH_TRACE_DBG("wiced_bt_hidh_get_report handle:0x%x type:%d ReportId:0x%x length:%d\n",
            handle, type, report_id, length);

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_get_report HIDH Not enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    if (handle >= WICED_BT_HIDH_NUM_DEV_MAX)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_get_report bad handle:0x%x\n", handle);
        return WICED_BT_HIDH_STATUS_INVALID_PARAM;
    }

    p_dev = &wiced_bt_hidh_cb.devices[handle];
    if (wiced_bt_hidh_dev_get_state(p_dev) != WICED_BT_HIDH_STATE_CONNECTED)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_get_report device handle:0x%x not connected\n", handle);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    return wiced_bt_hidh_core_get_report(p_dev, type, report_id, length);
}

/*
 * wiced_bt_hidh_set_protocol
 * This function will send an HID command to the peer device to change its Protocol Mode.
 */
wiced_bt_hidh_status_t wiced_bt_hidh_set_protocol(uint16_t handle,
        wiced_bt_hidh_protocol_t protocol)
{

    wiced_bt_hidh_dev_t *p_dev;

    HIDH_TRACE_DBG("wiced_bt_hidh_set_protocol handle:0x%x protocol:%d\n", handle, protocol);

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_set_protocol HIDH Not enabled\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    if (handle >= WICED_BT_HIDH_NUM_DEV_MAX)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_set_protocol bad handle:0x%x\n", handle);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    p_dev = &wiced_bt_hidh_cb.devices[handle];
    if (wiced_bt_hidh_dev_get_state(p_dev) != WICED_BT_HIDH_STATE_CONNECTED)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_set_protocol device handle:0x%x not connected\n", handle);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    return wiced_bt_hidh_core_set_protocol(p_dev, protocol);
}

