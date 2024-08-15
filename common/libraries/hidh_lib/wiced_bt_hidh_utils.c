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
 * WICED HID Host Utility functions
 *
 */


#include <string.h>
#include "wiced_bt_hidh_int.h"
#include "wiced_bt_hidh_utils.h"
#include "wiced_bt_hid_defs.h"

/*
 * Local functions
 */
char *wiced_bt_hidh_dev_get_state_desc(wiced_bt_hidh_dev_state_t state);

/*
 * wiced_bt_hidh_dev_alloc
 */
wiced_bt_hidh_dev_t *wiced_bt_hidh_dev_alloc(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_hidh_dev_t *p_dev = wiced_bt_hidh_cb.devices;
    int idx;

    for (idx = 0 ; idx < WICED_BT_HIDH_NUM_DEV_MAX ; idx++, p_dev++)
    {
        if (p_dev->state == WICED_BT_HIDH_STATE_FREE)
        {
            HIDH_TRACE_DBG("wiced_bt_hidh_dev_alloc dev:%d\n", wiced_bt_hidh_dev_get_handle(p_dev));
            memset(p_dev, 0, sizeof(*p_dev));
            memcpy(p_dev->bdaddr, bdaddr, BD_ADDR_LEN);
            return p_dev;
        }
    }
    return NULL;
}

/*
 * wiced_bt_hidh_dev_free
 */
void wiced_bt_hidh_dev_free(wiced_bt_hidh_dev_t *p_dev)
{
    wiced_bt_hidh_dev_t *p_dev_tmp = wiced_bt_hidh_cb.devices;
    int idx;

    for (idx = 0 ; idx < WICED_BT_HIDH_NUM_DEV_MAX ; idx++, p_dev_tmp++)
    {
        if (p_dev_tmp == p_dev)
        {
            HIDH_TRACE_DBG("wiced_bt_hidh_dev_free dev:%d\n", wiced_bt_hidh_dev_get_handle(p_dev));
            memset(p_dev, 0, sizeof(*p_dev));
            return;
        }
    }
    HIDH_TRACE_DBG("wiced_bt_hidh_dev_free device not found\n");
}

/*
 * wiced_bt_hidh_dev_set_state
 */
void wiced_bt_hidh_dev_set_state(wiced_bt_hidh_dev_t *p_dev, wiced_bt_hidh_dev_state_t state)
{
    HIDH_TRACE_DBG("wiced_bt_hidh_dev_set_state dev:%d state:%s (%d)\n",
            wiced_bt_hidh_dev_get_handle(p_dev), wiced_bt_hidh_dev_get_state_desc(state), state);
    p_dev->state = state;
}

/*
 * wiced_bt_hidh_dev_get_state
 */
wiced_bt_hidh_dev_state_t wiced_bt_hidh_dev_get_state(wiced_bt_hidh_dev_t *p_dev)
{
    HIDH_TRACE_DBG("wiced_bt_hidh_dev_get_state dev:%d state:%s (%d)\n",
            wiced_bt_hidh_dev_get_handle(p_dev), wiced_bt_hidh_dev_get_state_desc(p_dev->state),
            p_dev->state);
    return p_dev->state;
}

/*
 * wiced_bt_hidh_dev_get_by_bdaddr
 */
wiced_bt_hidh_dev_t *wiced_bt_hidh_dev_get_by_bdaddr(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_hidh_dev_t *p_dev = wiced_bt_hidh_cb.devices;
    int idx;

    for (idx = 0 ; idx < WICED_BT_HIDH_NUM_DEV_MAX ; idx++, p_dev++)
    {
        if ((p_dev->state != WICED_BT_HIDH_STATE_FREE) &&
            (memcmp(p_dev->bdaddr, bdaddr, BD_ADDR_LEN) == 0))
        {
            return p_dev;
        }
    }
    HIDH_TRACE_DBG("wiced_bt_hidh_dev_get_by_bdaddr [%B] not found\n", bdaddr);
    return NULL;
}

/*
 * wiced_bt_hidh_dev_get_by_con_cid
 */
wiced_bt_hidh_dev_t *wiced_bt_hidh_dev_get_by_con_cid(uint16_t cid)
{
    wiced_bt_hidh_dev_t *p_dev = wiced_bt_hidh_cb.devices;
    int idx;

    for (idx = 0 ; idx < WICED_BT_HIDH_NUM_DEV_MAX ; idx++, p_dev++)
    {
        if ((p_dev->state != WICED_BT_HIDH_STATE_FREE) &&
            ((p_dev->connection.ctrl_cid == cid) || (p_dev->connection.intr_cid == cid)))
        {
            return p_dev;
        }
    }
    HIDH_TRACE_DBG("wiced_bt_hidh_dev_get_by_con_cid cid:%d not found\n", cid);
    return NULL;
}

/*
 * wiced_bt_hidh_dev_get_handle
 */
uint16_t wiced_bt_hidh_dev_get_handle(wiced_bt_hidh_dev_t *p_dev)
{
    return (p_dev - wiced_bt_hidh_cb.devices);
}

/*
 * wiced_bt_hidh_convert_error
 */
wiced_bt_hidh_status_t wiced_bt_hidh_convert_error(uint8_t error)
{
    switch (error)
    {
    case HID_PAR_HANDSHAKE_RSP_SUCCESS: return WICED_BT_HIDH_STATUS_OK;
    case HID_PAR_HANDSHAKE_RSP_NOT_READY:return WICED_BT_HIDH_STATUS_BUSY;
    case HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID: return WICED_BT_HIDH_STATUS_INVALID_REP_ID;
    case HID_PAR_HANDSHAKE_RSP_ERR_UNSUPPORTED_REQ: return WICED_BT_HIDH_STATUS_UNSUPPORTED_REQ;
    case HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM: return WICED_BT_HIDH_STATUS_INVALID_PARAM;
    case HID_PAR_HANDSHAKE_RSP_ERR_UNKNOWN: return WICED_BT_HIDH_STATUS_UNKNOWN;
    case HID_PAR_HANDSHAKE_RSP_ERR_FATAL: return WICED_BT_HIDH_STATUS_FATAL;
    }
    return WICED_BT_HIDH_STATUS_ERROR;

}

/*
 * wiced_bt_hidh_dev_get_state_desc
 */
char *wiced_bt_hidh_dev_get_state_desc(wiced_bt_hidh_dev_state_t state)
{
    switch(state)
    {
    case WICED_BT_HIDH_STATE_FREE: return "FREE";
    case WICED_BT_HIDH_STATE_REGISTERED: return "REGISTERED";
    case WICED_BT_HIDH_STATE_IN_CONNECTING: return "IN_CONNECTING";
    case WICED_BT_HIDH_STATE_OUT_CONNECTING: return "OUT_CONNECTING";
    case WICED_BT_HIDH_STATE_CONNECTED: return "CONNECTED";
    case WICED_BT_HIDH_STATE_DISCONNECTING: return "DISCONNECTING";
    default: return "Unknown";
    }
}

/*
 * wiced_bt_hidh_dev_print
 */
void wiced_bt_hidh_dev_print(void)
{
    wiced_bt_hidh_dev_t *p_dev = wiced_bt_hidh_cb.devices;
    int idx;

    for (idx = 0 ; idx < WICED_BT_HIDH_NUM_DEV_MAX ; idx++, p_dev++)
    {
        HIDH_TRACE_DBG("dev:%d state:%d Addr [%B]\n", idx, p_dev->state, p_dev->bdaddr);
    }
}

