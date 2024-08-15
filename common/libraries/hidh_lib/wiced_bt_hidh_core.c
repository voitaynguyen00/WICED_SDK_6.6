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
 * WICED HID Host State Machine
 *
 */

#include <string.h>
#include "wiced_memory.h"
#include "wiced_bt_hidh_core.h"
#include "wiced_bt_hidh_con.h"
#include "wiced_bt_hidh_utils.h"
#include "wiced_bt_hid_defs.h"

/*
 * Local functions
 */
static void wiced_bt_hidh_rx_data_handler(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_channel_t channel, uint8_t *p_data, uint16_t length);

/*
 * wiced_bt_hidh_con_cback
 */
void wiced_bt_hidh_con_cback (wiced_bt_hidh_con_event_t event, wiced_bt_hidh_con_data_t *p_data)
{
    wiced_bt_hidh_dev_t *p_dev;
    wiced_bt_hidh_data_t event_data;

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_con_cback cback is NULL\n");
        return;
    }

    switch (event)
    {
    case WICED_BT_HIDH_CON_CONNECTED:
        p_dev = p_data->connected.p_dev;
        if (p_dev == NULL)
        {
            HIDH_TRACE_DBG("Err: wiced_bt_hidh_con_cback p_dev is NULL (Con)\n");
            return;
        }
        HIDH_TRACE_DBG("wiced_bt_hidh_con_cback CONNECTED status:%d address:%B handle:%d\n",
                p_data->connected.status, p_dev->bdaddr, wiced_bt_hidh_dev_get_handle(p_dev));

        event_data.open.handle = wiced_bt_hidh_dev_get_handle(p_dev);
        event_data.open.status = p_data->connected.status;
        memcpy(event_data.open.bdaddr, p_dev->bdaddr, BD_ADDR_LEN);

        if (p_data->connected.status == WICED_BT_HIDH_STATUS_OK)
        {
            wiced_bt_hidh_dev_set_state(p_dev, WICED_BT_HIDH_STATE_CONNECTED);
        }
        else
        {
            /* If this device is not 'added' (not allowed to reconnect) */
            if (p_dev->added == WICED_FALSE)
            {
                wiced_bt_hidh_dev_free(p_dev);  /* Free it */
            }
            else
            {
                wiced_bt_hidh_dev_set_state(p_dev, WICED_BT_HIDH_STATE_REGISTERED);
            }
        }

        /* Call the HIDH Callback */
        wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_OPEN_EVT, &event_data);
        break;

    case WICED_BT_HIDH_CON_DISCONNECTED:
        p_dev = p_data->disconnected.p_dev;
        if (p_dev == NULL)
        {
            HIDH_TRACE_DBG("Err: wiced_bt_hidh_con_cback p_dev is NULL (Disc)\n");
            return;
        }
        event_data.close.handle = wiced_bt_hidh_dev_get_handle(p_dev);
        event_data.close.reason = p_data->disconnected.reason;

        HIDH_TRACE_DBG("wiced_bt_hidh_con_cback CON_DISCONNECTED handle:%d\n",
                event_data.close.handle);

        /* If this device is not 'added' (not allowed to reconnect) */
        if (p_dev->added == WICED_FALSE)
        {
            wiced_bt_hidh_dev_free(p_dev);
        }
        else
        {
            p_dev->request = WICED_BT_HIDH_REQUEST_IDLE;
            wiced_bt_hidh_dev_set_state(p_dev, WICED_BT_HIDH_STATE_REGISTERED);
        }

        /* Call the HIDH Callback */
        wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_CLOSE_EVT, &event_data);
        break;

    case WICED_BT_HIDH_CON_RX_DATA:
        p_dev = p_data->rx_data.p_dev;
        if (p_dev == NULL)
        {
            HIDH_TRACE_DBG("Err: wiced_bt_hidh_con_cback p_dev is NULL (Rx)\n");
            return;
        }
        wiced_bt_hidh_rx_data_handler(p_dev, p_data->rx_data.channel, p_data->rx_data.p_data,
                p_data->rx_data.length);
        break;

    default:
        HIDH_TRACE_DBG("wiced_bt_hidh_con_cback unknown event:%d\n", event);
        break;
    }
}
/*
 * wiced_bt_hidh_core_discovery_done
 */
void wiced_bt_hidh_core_discovery_done (wiced_bt_hidh_dev_t *p_dev, wiced_bt_hidh_status_t status)
{
    wiced_bt_hidh_data_t event_data;
    wiced_bt_hidh_status_t con_status;

    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_discovery_done p_dev is NULL\n");
        return;
    }

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_discovery_done cback is NULL\n");
        return;
    }

    /* If SDP Success */
    if (status == WICED_BT_HIDH_STATUS_OK)
    {
        /* Try to open HIDH connection */
        status = wiced_bt_hidh_con_connect(p_dev);
    }

    if (status != WICED_BT_HIDH_STATUS_OK)
    {
        event_data.open.handle = 0;
        event_data.open.status = status;
        memcpy(event_data.open.bdaddr, p_dev->bdaddr, BD_ADDR_LEN);

        if (p_dev->added == WICED_FALSE)
        {
            wiced_bt_hidh_dev_free(p_dev);
        }
        else
        {
            wiced_bt_hidh_dev_set_state(p_dev, WICED_BT_HIDH_STATE_REGISTERED);
        }

        /* Call the HIDH Callback */
        wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_OPEN_EVT, &(event_data));
    }
}

/*
 * wiced_bt_hidh_core_descriptor_done
 */
void wiced_bt_hidh_core_descriptor_done(wiced_bt_hidh_dev_t *p_dev, wiced_bt_hidh_status_t status,
        uint8_t *p_descriptor, uint16_t length)
{
    wiced_bt_hidh_data_t event_data;

    HIDH_TRACE_DBG("wiced_bt_hidh_core_descriptor_done status:%d\n", status);

    if (p_dev == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_descriptor_done p_dev is NULL\n");
        return;
    }

    if (wiced_bt_hidh_cb.p_cback == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_descriptor_done cback is NULL\n");
        return;
    }

    memset(&event_data.descriptor, 0, sizeof(event_data.descriptor));
    event_data.descriptor.status = status;
    event_data.descriptor.handle = wiced_bt_hidh_dev_get_handle(p_dev);
    if (status == WICED_BT_HIDH_STATUS_OK)
    {
        event_data.descriptor.length = length;
        event_data.descriptor.p_descriptor = p_descriptor;
    }

    /* Call the HIDH Callback */
    wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_DESCRIPTOR_EVT, &(event_data));
}

/*
 * wiced_bt_hidh_rx_data_handler
 */
static void wiced_bt_hidh_rx_data_handler(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_channel_t channel, uint8_t *p_data, uint16_t length)
{
    uint8_t msg_type, param, rep_type, idle_rate;
    wiced_bt_hidh_data_t event_data;

    msg_type = HID_GET_TRANS_FROM_HDR(*p_data);
    param    = HID_GET_PARAM_FROM_HDR(*p_data);
    rep_type = param & HID_PAR_REP_TYPE_MASK;
    p_data++;
    length--;

    HIDH_TRACE_DBG("HIDH Data Type:0x%x Param:0x%x\n", msg_type, param);

    switch (msg_type)
    {
    case HID_TRANS_HANDSHAKE:
        switch(p_dev->request)
        {
        case WICED_BT_HIDH_REQUEST_SET_REPORT:
            HIDH_TRACE_DBG("SetReport handshake status:%d\n", param);
            p_dev->request = WICED_BT_HIDH_REQUEST_IDLE;
            event_data.set_report.handle = wiced_bt_hidh_dev_get_handle(p_dev);
            event_data.set_report.status = wiced_bt_hidh_convert_error(param);
            if (wiced_bt_hidh_cb.p_cback)
                wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_SET_REPORT_EVT, &event_data) ;
            break;

        case WICED_BT_HIDH_REQUEST_GET_REPORT:
            /* This HandShake message is supposed to be received if the GetReport fails only */
            HIDH_TRACE_DBG("GetReport handshake status:%d\n", param);
            p_dev->request = WICED_BT_HIDH_REQUEST_IDLE;
            event_data.get_report.handle = wiced_bt_hidh_dev_get_handle(p_dev);
            event_data.get_report.status = wiced_bt_hidh_convert_error(param);
            event_data.get_report.length = 0;
            event_data.get_report.p_data = NULL;
            if (wiced_bt_hidh_cb.p_cback)
                wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_GET_REPORT_EVT, &event_data) ;
            break;

        case WICED_BT_HIDH_REQUEST_SET_PROTOCOL:
            HIDH_TRACE_DBG("SetProtocol handshake status:%d\n", param);
            p_dev->request = WICED_BT_HIDH_REQUEST_IDLE;
            event_data.set_protocol.handle = wiced_bt_hidh_dev_get_handle(p_dev);
            event_data.set_report.status = wiced_bt_hidh_convert_error(param);
            if (wiced_bt_hidh_cb.p_cback)
                wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_SET_PROTOCOL_EVT, &event_data) ;
            break;

        default:
            HIDH_TRACE_DBG("Err: Unexpected Handshake\n");
            break;
        }
        break;

    case HID_TRANS_DATA:
        /* Data packet received on the HID Control channel are Get Report response */
        if (channel == WICED_BT_HIDH_CHANNEL_CONTROL)
        {
            HIDH_TRACE_DBG("HID Data received on control channel rep_type:%d\n", rep_type);
            if (p_dev->request != WICED_BT_HIDH_REQUEST_GET_REPORT)
            {
                HIDH_TRACE_DBG("Err: GetReport response not expected!!!\n");
                break;
            }
            HIDH_TRACE_DBG("GetReport response received\n");
            p_dev->request = WICED_BT_HIDH_REQUEST_IDLE;
            event_data.get_report.handle = wiced_bt_hidh_dev_get_handle(p_dev);
            event_data.get_report.status = WICED_BT_HIDH_STATUS_OK;
            event_data.get_report.length = length;
            event_data.get_report.p_data = p_data;
            if (wiced_bt_hidh_cb.p_cback)
                wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_GET_REPORT_EVT, &event_data) ;
        }
        /* Data packet received on the HID Interrupt channel are regular HID Reports */
        else
        {
            if (length > 0)
            {
                event_data.report.handle = wiced_bt_hidh_dev_get_handle(p_dev);
                event_data.report.report_id = *p_data++;
                length--;
                event_data.report.length = length;
                event_data.report.p_data = p_data;
                if (wiced_bt_hidh_cb.p_cback)
                    wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_REPORT_EVT, &event_data) ;
            }
        }
        break;

    case HID_TRANS_DATAC:
        HIDH_TRACE_DBG("HID_TRANS_DATAC not supported (deprecated)\n");
        break;

    case HID_TRANS_CONTROL:
        switch(param)
        {
        case HID_PAR_CONTROL_VIRTUAL_CABLE_UNPLUG:
            /* Pairing information must be erased and the HID link must be disconnected */
            HIDH_TRACE_DBG("Virtual_Cable_Unplug received\n", param);

            /*
             * Device not anymore allowed to reconnect.
             * The device will be freed after disconnection
             */
            p_dev->added = WICED_FALSE;

            event_data.virtual_unplug.handle = wiced_bt_hidh_dev_get_handle(p_dev);
            if (wiced_bt_hidh_cb.p_cback)
                wiced_bt_hidh_cb.p_cback(WICED_BT_HIDH_VIRTUAL_UNPLUG_EVT, &event_data) ;

            /* Disconnect the Device */
            wiced_bt_hidh_core_close(p_dev);
            break;

        case HID_PAR_CONTROL_NOP:
        case HID_PAR_CONTROL_HARD_RESET:
        case HID_PAR_CONTROL_SOFT_RESET:
        case HID_PAR_CONTROL_SUSPEND:
        case HID_PAR_CONTROL_EXIT_SUSPEND:
        default:
            HIDH_TRACE_DBG("Err: Wrong HID_CONTROL Operation (0x%x) received\n", param);
        }
        break;

    case HID_TRANS_GET_REPORT:
    case HID_TRANS_SET_REPORT:
    case HID_TRANS_GET_PROTOCOL:
    case HID_TRANS_SET_PROTOCOL:
        HIDH_TRACE_DBG("HIDH wrong type:0x%x received\n", msg_type);
        break;

    default:
        HIDH_TRACE_DBG("HIDH unknown type:0x%x received\n", msg_type);
        return;
    }
}

/*
 * wiced_bt_hidh_core_set_report
 */
wiced_bt_hidh_status_t wiced_bt_hidh_core_set_report(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_channel_t channel, wiced_bt_hidh_report_type_t type, uint8_t report_id,
        uint8_t *p_data, uint16_t length)
{
    uint8_t *p_buf;
    uint8_t *p;
    wiced_bt_hidh_status_t status;

    HIDH_TRACE_DBG("wiced_bt_hidh_core_set_report Channel:%d Type:%d Id:0x%x length:%d\n",
            channel, type, report_id, length);

    if (channel > WICED_BT_HIDH_CHANNEL_INTERRUPT)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_report wrong channel:%d\n", channel);
        return WICED_BT_HIDH_STATUS_INVALID_PARAM;
    }

    if (type > WICED_BT_HIDH_REPORT_TYPE_FEATURE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_report wrong type:%d\n", type);
        return WICED_BT_HIDH_STATUS_MEM_FULL;
    }

    /* If the channel is the Control channel */
    if (channel == WICED_BT_HIDH_CHANNEL_CONTROL)
    {
        /* Only one Tx pending packet allowed on Control Channel */
        if (p_dev->request != WICED_BT_HIDH_REQUEST_IDLE)
        {
            HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_report device busy (pending:%d)\n",
                    p_dev->request);
            return WICED_BT_HIDH_STATUS_BUSY;
        }
        if ((HID_HDR_LEN + sizeof(report_id) + length) > p_dev->connection.ctrl_mtu)
        {
            HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_report packet length:%d bigger than mtu:%d\n",
                    HID_HDR_LEN + sizeof(report_id) + length, p_dev->connection.ctrl_mtu);
            return WICED_BT_HIDH_STATUS_INVALID_PARAM;
        }
    }
    /* Else, the channel is the Interrupt channel */
    else
    {
        if ((HID_HDR_LEN + sizeof(report_id) + length) > p_dev->connection.intr_cid)
        {
            HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_report packet length:%d bigger than mtu:%d\n",
                    HID_HDR_LEN + sizeof(report_id) + length, p_dev->connection.ctrl_mtu);
            return WICED_BT_HIDH_STATUS_INVALID_PARAM;
        }
    }

    /* Allocate a Tx buffer */
    p_buf = (uint8_t *)wiced_bt_get_buffer(HID_HDR_LEN + sizeof(report_id) + length);
    if (p_buf == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_report Mem full\n");
        return WICED_BT_HIDH_STATUS_MEM_FULL;
    }
    p = p_buf;

    /* Build the Set_Report message */
    UINT8_TO_STREAM(p, HID_BUILD_HDR(HID_TRANS_SET_REPORT, type));
    UINT8_TO_STREAM(p, report_id);
    if (length > 0)
    {
        memcpy(p, p_data, length);
        p += length;
    }

    if (channel == WICED_BT_HIDH_CHANNEL_CONTROL)
    {
        p_dev->request = WICED_BT_HIDH_REQUEST_SET_REPORT;
    }

    /* Send the built HID Request */
    status =  wiced_bt_hidh_con_tx_data(p_dev, channel, p_buf,
            length + HID_HDR_LEN + sizeof(report_id));
    if (status != WICED_BT_HIDH_STATUS_OK)
    {
        if (channel == WICED_BT_HIDH_CHANNEL_CONTROL)
        {
            p_dev->request = WICED_BT_HIDH_REQUEST_IDLE;
        }
    }
    wiced_bt_free_buffer(p_buf);

    return status;
}

/*
 * wiced_bt_hidh_core_get_report
 */
wiced_bt_hidh_status_t wiced_bt_hidh_core_get_report(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_report_type_t type, uint8_t report_id, uint16_t length)
{
    uint8_t hid_cmd_buf[1 + 1 + 2];
    uint8_t *p = hid_cmd_buf;
    wiced_bt_hidh_status_t status;
    uint8_t header;

    if (type > WICED_BT_HIDH_REPORT_TYPE_FEATURE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_get_report wrong type:%d\n", type);
        return WICED_BT_HIDH_STATUS_MEM_FULL;
    }

    /* Only one Tx pending packet allowed on Control Channel */
    if (p_dev->request != WICED_BT_HIDH_REQUEST_IDLE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_get_report device busy (pending:%d)\n",
                    p_dev->request);
        return WICED_BT_HIDH_STATUS_BUSY;
    }

    /* Build the Set_Report message */
    header = HID_BUILD_HDR(HID_TRANS_GET_REPORT, type);
    /* If the application specifies an BufferSize length, include it in the Request */
    if (length != 0)
    {
        header |= HID_PAR_GET_REP_BUFSIZE_FOLLOWS;
    }
    UINT8_TO_STREAM(p, header);

    /* The ReportId parameter is optional */
    if (report_id != 0)
    {
        UINT8_TO_STREAM(p, report_id);
    }

    /* The BufferSize length parameter is optional */
    if (length != 0)
    {
        UINT16_TO_STREAM(p, length);
    }

    /* Remember that a GetReport is pending */
    p_dev->request = WICED_BT_HIDH_REQUEST_GET_REPORT;

    /* Send the built HID Request */
    status =  wiced_bt_hidh_con_tx_data(p_dev, WICED_BT_HIDH_CHANNEL_CONTROL, hid_cmd_buf,
            p - hid_cmd_buf);
    if (status != WICED_BT_HIDH_STATUS_OK)
    {
            p_dev->request = WICED_BT_HIDH_REQUEST_IDLE;
    }

    return status;
}

/*
 * wiced_bt_hidh_core_close
 */
wiced_bt_hidh_status_t wiced_bt_hidh_core_close(wiced_bt_hidh_dev_t *p_dev)
{
    return wiced_bt_hidh_con_disconnect(p_dev);
}

/*
 * wiced_bt_hidh_core_set_protocol
 */
wiced_bt_hidh_status_t wiced_bt_hidh_core_set_protocol(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_protocol_t protocol)
{
    uint8_t *p_buf, *p;
    uint8_t protocol_param;

    switch(protocol)
    {
    case WICED_BT_HIDH_PROTOCOL_REPORT:
        protocol_param = HID_PAR_PROTOCOL_REPORT;
        break;

    case WICED_BT_HIDH_PROTOCOL_BOOT:
        protocol_param = HID_PAR_PROTOCOL_BOOT_MODE;
        break;

    default:
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_protocol unknown protocol:%d\n", protocol);
        return WICED_BT_HIDH_STATUS_INVALID_PARAM;
    }

    /* Only one Tx pending packet allowed on Control Channel */
    if (p_dev->request != WICED_BT_HIDH_REQUEST_IDLE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_protocol device busy (pending:%d)\n",
                p_dev->request);
        return WICED_BT_HIDH_STATUS_BUSY;
    }

    /* Allocate a Tx buffer */
    p_buf = (uint8_t *)wiced_bt_get_buffer(HID_HDR_LEN);
    if (p_buf == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_core_set_report Mem full\n");
        return WICED_BT_HIDH_STATUS_MEM_FULL;
    }
    p = p_buf;

    p_dev->request = WICED_BT_HIDH_REQUEST_SET_PROTOCOL;

    /* Build the Set_Report message */
    UINT8_TO_STREAM(p, HID_BUILD_HDR(HID_TRANS_SET_PROTOCOL, protocol_param));

    /* Send the built HID Request */
    return wiced_bt_hidh_con_tx_data(p_dev, WICED_BT_HIDH_CHANNEL_CONTROL, p_buf, HID_HDR_LEN);
}
