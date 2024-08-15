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
 * WICED HID Host Connection (L2CAP)
 *
 */

#include <string.h>
#include "wiced_bt_hidh_int.h"
#include "wiced_bt_hidh_con.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_hid_defs.h"
#include "wiced_bt_hidh_utils.h"

/*
 * Definitions
 */
#define HID_DEV_FLUSH_TO    0xffff

typedef struct
{
    wiced_bt_hidh_con_cback_t *p_cback;
    wiced_bt_l2cap_appl_information_t l2cap_ctrl_appl_info;
    wiced_bt_l2cap_appl_information_t l2cap_intr_appl_info;
} wiced_bt_hidh_con_cb_t;

static wiced_bt_hidh_con_cb_t wiced_bt_hidh_con_cb;

/*
 * External definitions
 */
uint16_t btm_get_acl_disc_reason_code (void);

/*
 * Local functions
 */
static void wiced_bt_hidh_con_ctrl_connected(void *context, wiced_bt_device_address_t bdaddr,
        uint16_t l2cap_cid, uint16_t mtu);
static void wiced_bt_hidh_con_intr_connected(void *context, wiced_bt_device_address_t bdaddr,
        uint16_t l2cap_cid, uint16_t mtu);
static void wiced_bt_hidh_con_disconnected_ind(void *context, uint16_t l2cap_cid,
        wiced_bool_t ack_needed);
static void wiced_bt_hidh_con_disconnected_cfm(void *context, uint16_t l2cap_cid, uint16_t result);
static void wiced_bt_hidh_con_rx_data_ind(void *context, uint16_t l2cap_cid, uint8_t *p_data,
        uint16_t buf_len);
static void wiced_bt_hidh_con_cong_ind(void *context, uint16_t l2cap_cid, wiced_bool_t congested);

/*
 * wiced_bt_hidh_con_init
 */
void wiced_bt_hidh_con_init(void)
{
    wiced_bt_l2cap_appl_information_t *p_l2c_info;

    memset(&wiced_bt_hidh_con_cb, 0, sizeof(wiced_bt_hidh_con_cb));

    p_l2c_info = &wiced_bt_hidh_con_cb.l2cap_ctrl_appl_info;
    p_l2c_info->connected_cback = wiced_bt_hidh_con_ctrl_connected;
    p_l2c_info->disconnect_indication_cback = wiced_bt_hidh_con_disconnected_ind;
    p_l2c_info->disconnect_confirm_cback = wiced_bt_hidh_con_disconnected_cfm;
    p_l2c_info->data_indication_cback = wiced_bt_hidh_con_rx_data_ind;
    p_l2c_info->congestion_status_cback = wiced_bt_hidh_con_cong_ind;
    p_l2c_info->mtu = HID_DEV_MTU_SIZE;
    p_l2c_info->flush_timeout_present = TRUE;
    p_l2c_info->flush_timeout = HID_DEV_FLUSH_TO;

    p_l2c_info = &wiced_bt_hidh_con_cb.l2cap_intr_appl_info;
    p_l2c_info->connected_cback = wiced_bt_hidh_con_intr_connected;
    p_l2c_info->disconnect_indication_cback = wiced_bt_hidh_con_disconnected_ind;
    p_l2c_info->disconnect_confirm_cback = wiced_bt_hidh_con_disconnected_cfm;
    p_l2c_info->data_indication_cback = wiced_bt_hidh_con_rx_data_ind;
    p_l2c_info->congestion_status_cback = wiced_bt_hidh_con_cong_ind;
    p_l2c_info->mtu = HID_DEV_MTU_SIZE;
    p_l2c_info->flush_timeout_present = TRUE;
    p_l2c_info->flush_timeout = HID_DEV_FLUSH_TO;
}

/*
 * wiced_bt_hidh_con_register
 */
wiced_bt_hidh_status_t wiced_bt_hidh_con_register(wiced_bt_hidh_con_cback_t *p_cback)
{
    HIDH_TRACE_DBG("wiced_bt_hidh_con_register\n");

    if (p_cback == NULL)
    {
        HIDH_TRACE_DBG("wiced_bt_hidh_con_register null callback\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    /* Now, register with L2CAP for control and interrupt PSMs*/
    if (wiced_bt_l2cap_register(HID_PSM_CONTROL, &wiced_bt_hidh_con_cb.l2cap_ctrl_appl_info, NULL) == 0)
    {
        HIDH_TRACE_DBG("Err: HIDD Control Registration failed\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    if (wiced_bt_l2cap_register(HID_PSM_INTERRUPT, &wiced_bt_hidh_con_cb.l2cap_intr_appl_info, NULL) == 0)
    {
        HIDH_TRACE_DBG("Err: HIDD Interrupt Registration failed\n");
        wiced_bt_l2cap_deregister( HID_PSM_CONTROL ) ;
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    wiced_bt_hidh_con_cb.p_cback = p_cback;

    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_con_deregister
 */
wiced_bt_hidh_status_t wiced_bt_hidh_con_deregister(void)
{
    wiced_bt_l2cap_deregister (HID_PSM_CONTROL);
    wiced_bt_l2cap_deregister (HID_PSM_INTERRUPT);
    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_con_connect
 */
wiced_bt_hidh_status_t wiced_bt_hidh_con_connect(wiced_bt_hidh_dev_t *p_dev)
{
    uint16_t ctrl_cid;

    /* Open the HID control channel */
    ctrl_cid = wiced_bt_l2cap_connect_req (HID_PSM_CONTROL, p_dev->bdaddr, NULL);
    if (ctrl_cid == 0)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_con_connect wiced_bt_l2cap_connect_req failed\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }
    p_dev->connection.ctrl_cid =  ctrl_cid;
    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_con_disconnect
 */
wiced_bt_hidh_status_t wiced_bt_hidh_con_disconnect(wiced_bt_hidh_dev_t *p_dev)
{
    wiced_bool_t result;
    wiced_bt_hidh_status_t status = WICED_BT_HIDH_STATUS_OK;

    /* Close the HID Interrupt channel */
    result = wiced_bt_l2cap_disconnect_req (p_dev->connection.intr_cid);
    if (result == WICED_FALSE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_con_disconnect (intr) failed\n");
        status |= WICED_BT_HIDH_STATUS_ERROR;
    }

    /* Close the HID control channel */
    result = wiced_bt_l2cap_disconnect_req (p_dev->connection.ctrl_cid);
    if (result == WICED_FALSE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_con_disconnect (ctrl) failed\n");
        status |= WICED_BT_HIDH_STATUS_ERROR;
    }
    return status;
}

/*
 * wiced_bt_hidh_con_tx_data
 */
wiced_bt_hidh_status_t wiced_bt_hidh_con_tx_data(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_channel_t channel, uint8_t *p_data, uint16_t length)
{
    uint16_t cid;
    uint8_t result;

    if (channel == WICED_BT_HIDH_CHANNEL_CONTROL)
        cid = p_dev->connection.ctrl_cid;
    else
        cid = p_dev->connection.intr_cid;

    result = wiced_bt_l2cap_data_write(cid, p_data, length, L2CAP_NON_FLUSHABLE_PACKET);
    if (result == L2CAP_DATAWRITE_SUCCESS)
        return WICED_BT_HIDH_STATUS_OK;
    else
        return WICED_BT_HIDH_STATUS_ERROR;
}

/*
 * wiced_bt_hidh_con_ctrl_connected
 */
static void wiced_bt_hidh_con_ctrl_connected(void *context, wiced_bt_device_address_t bdaddr,
        uint16_t l2cap_cid, uint16_t mtu)
{
    wiced_bt_hidh_dev_t *p_dev;
    uint16_t intr_cid;

    HIDH_TRACE_DBG("wiced_bt_hidh_con_ctrl_connected %B cid:0x%x mtu:%d\n", bdaddr, l2cap_cid, mtu);

    /* Check if this device is present in the device list */
    p_dev = wiced_bt_hidh_dev_get_by_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("wiced_bt_hidh_con_ctrl_connected unknown device. Disconnecting\n");
        wiced_bt_l2cap_disconnect_req (l2cap_cid);
        return;
    }

    p_dev->connection.ctrl_cid = l2cap_cid;
    p_dev->connection.ctrl_mtu = mtu;

    /* If we are initiator, then connect interrupt channel */
    if (wiced_bt_hidh_dev_get_state(p_dev) == WICED_BT_HIDH_STATE_OUT_CONNECTING)
    {
        intr_cid = wiced_bt_l2cap_connect_req (HID_PSM_INTERRUPT, p_dev->bdaddr, NULL);
        if (intr_cid == 0)
        {
            HIDH_TRACE_DBG("wiced_bt_hidh_con_ctrl_connected Intr con failed\n");
            wiced_bt_l2cap_disconnect_req (l2cap_cid);
            if (wiced_bt_hidh_con_cb.p_cback)
            {
                wiced_bt_hidh_con_data_t event_data;
                event_data.connected.p_dev = p_dev;
                event_data.connected.status = WICED_BT_HIDH_STATUS_ERROR;
                wiced_bt_hidh_con_cb.p_cback(WICED_BT_HIDH_CON_CONNECTED, &event_data);
            }
            return;
        }
        p_dev->connection.intr_cid = intr_cid;
    }
}

/*
 * wiced_bt_hidh_con_intr_connected
 */
static void wiced_bt_hidh_con_intr_connected(void *context, wiced_bt_device_address_t bdaddr,
        uint16_t l2cap_cid, uint16_t mtu)
{
    wiced_bt_hidh_dev_t *p_dev;
    uint16_t intr_cid;
    wiced_bt_hidh_con_data_t event_data;

    HIDH_TRACE_DBG("wiced_bt_hidh_con_intr_connected %B cid:0x%x mtu:%d\n", bdaddr, l2cap_cid, mtu);

    /* Check if this device is present in the device list */
    p_dev = wiced_bt_hidh_dev_get_by_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("wiced_bt_hidh_con_intr_connected unknown device. Disconnecting\n");
        wiced_bt_l2cap_disconnect_req (l2cap_cid);
        return;
    }

    /* Interrupt Channel must be opened after Control Channel */
    if (p_dev->connection.ctrl_cid == 0)
    {
        wiced_bt_l2cap_disconnect_req (l2cap_cid);
        return;
    }

    p_dev->connection.intr_cid = l2cap_cid;
    p_dev->connection.intr_mtu = mtu;

    if (wiced_bt_hidh_con_cb.p_cback)
    {
        event_data.connected.p_dev = p_dev;
        event_data.connected.status = WICED_BT_HIDH_STATUS_OK;
        wiced_bt_hidh_con_cb.p_cback(WICED_BT_HIDH_CON_CONNECTED, &event_data);
    }
}
/*
 * wiced_bt_hidh_con_disconnected_ind
 */
static void wiced_bt_hidh_con_disconnected_ind(void *context, uint16_t l2cap_cid,
        wiced_bool_t ack_needed)
{
    wiced_bt_hidh_dev_t *p_dev;
    uint16_t disc_res = HCI_SUCCESS;
    wiced_bt_hidh_con_data_t event_data;

    HIDH_TRACE_DBG("wiced_bt_hidh_con_disconnected_ind cid:0x%x\n", l2cap_cid);

    if (ack_needed)
    {
        wiced_bt_l2cap_disconnect_rsp(l2cap_cid);
    }

    /* Check if this device is present in the device list */
    p_dev = wiced_bt_hidh_dev_get_by_con_cid(l2cap_cid);
    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("wiced_bt_hidh_con_disconnected_ind unknown device. Ignoring\n");
        return;
    }

    if (p_dev->connection.ctrl_cid == l2cap_cid)
        p_dev->connection.ctrl_cid = 0;
    else
        p_dev->connection.intr_cid = 0;

    if ((p_dev->connection.ctrl_cid == 0) &&
        (p_dev->connection.intr_cid == 0) &&
        (wiced_bt_hidh_con_cb.p_cback))
    {
        if (!ack_needed)
            disc_res = btm_get_acl_disc_reason_code();

        if( disc_res == HCI_ERR_CONNECTION_TOUT || disc_res == HCI_ERR_UNSPECIFIED )
            event_data.disconnected.reason = WICED_BT_HIDH_STATUS_CON_LOST;
        else
            event_data.disconnected.reason = WICED_BT_HIDH_STATUS_OK;
        event_data.disconnected.p_dev = p_dev;
        wiced_bt_hidh_con_cb.p_cback(WICED_BT_HIDH_CON_DISCONNECTED, &event_data);
    }
}

/*
 * wiced_bt_hidh_con_disconnected_cfm
 */
static void wiced_bt_hidh_con_disconnected_cfm (void *context, uint16_t l2cap_cid, uint16_t result)
{
    wiced_bt_hidh_dev_t *p_dev;
    uint16_t disc_res;
    wiced_bt_hidh_con_data_t event_data;

    HIDH_TRACE_DBG("wiced_bt_hidh_con_disconnected_cfm cid:0x%x\n", l2cap_cid);

    /* Check if this device is present in the device list */
    p_dev = wiced_bt_hidh_dev_get_by_con_cid(l2cap_cid);
    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("wiced_bt_hidh_con_disconnected_cfm unknown device. Ignoring\n");
        return;
    }

    if (p_dev->connection.ctrl_cid == l2cap_cid)
        p_dev->connection.ctrl_cid = 0;
    else
        p_dev->connection.intr_cid = 0;


    if ((p_dev->connection.ctrl_cid == 0) &&
        (p_dev->connection.intr_cid == 0) &&
        (wiced_bt_hidh_con_cb.p_cback))
    {
        event_data.disconnected.reason = WICED_BT_HIDH_STATUS_OK;
        event_data.disconnected.p_dev = p_dev;
        wiced_bt_hidh_con_cb.p_cback(WICED_BT_HIDH_CON_DISCONNECTED, &event_data);
    }
}

/*
 * wiced_bt_hidh_con_rx_data_ind
 * Note: L2CAP frees the buffer after calling this callback.
 *       So, wiced_bt_free_buffer doe not have to be called
 */
static void wiced_bt_hidh_con_rx_data_ind (void *context, uint16_t l2cap_cid, uint8_t *p_data,
        uint16_t buf_len)
{
    wiced_bt_hidh_dev_t *p_dev;
    uint16_t disc_res;
    wiced_bt_hidh_con_data_t event_data;

    HIDH_TRACE_DBG("wiced_bt_hidh_con_rx_data_ind cid:0x%x buf_len:%d\n", l2cap_cid, buf_len);

    /* Check if this device is present in the device list */
    p_dev = wiced_bt_hidh_dev_get_by_con_cid(l2cap_cid);
    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_con_rx_data_ind unknown device. Ignoring\n");
        return;
    }

    event_data.rx_data.length = buf_len;
    event_data.rx_data.p_data = p_data;
    event_data.rx_data.p_dev = p_dev;
    if (l2cap_cid == p_dev->connection.ctrl_cid)
    {
        event_data.rx_data.channel = WICED_BT_HIDH_CHANNEL_CONTROL;
    }
    else
    {
        event_data.rx_data.channel = WICED_BT_HIDH_CHANNEL_INTERRUPT;
    }
    if (wiced_bt_hidh_con_cb.p_cback)
    {
        wiced_bt_hidh_con_cb.p_cback(WICED_BT_HIDH_CON_RX_DATA, &event_data) ;
    }
}

/*
 * wiced_bt_hidh_con_cong_ind
 */
static void wiced_bt_hidh_con_cong_ind (void *context, uint16_t l2cap_cid, wiced_bool_t congested)
{
    HIDH_TRACE_DBG("wiced_bt_hidh_con_cong_ind congested:%d not yet implemented\n", congested);
}

