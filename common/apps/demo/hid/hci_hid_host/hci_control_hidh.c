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
 * WICED HID Host Application
 *
 */

#include "wiced_bt_trace.h"
#include "wiced_bt_dev.h"
#include "wiced_transport.h"
#include "wiced_memory.h"
#include "hci_control_hidh.h"
#include "hci_control_api.h"
#include "wiced_bt_hidh.h"
#include "wiced_bt_types.h"

/*
 * Local functions
 */
void hci_control_hidh_cback( wiced_bt_hidh_event_t event, wiced_bt_hidh_data_t *p_data );

/*
 * hci_control_hidh_init
 */
void hci_control_hidh_init( void )
{
    wiced_bt_hidh_init();
}

/*
 * hci_control_hidh_start
 */
void hci_control_hidh_start( void )
{
    wiced_bt_hidh_enable(hci_control_hidh_cback);
}

/*
 * hci_control_hidh_handle_command
 * Handle HID Host commands from MCU
 */
void hci_control_hidh_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    wiced_bt_device_address_t bdaddr;
    uint8_t protocol;
    uint8_t channel;
    uint16_t handle;
    uint8_t report_type, report_id;
    wiced_bt_hidh_status_t status;
    uint16_t buffer_size;
    uint8_t event_data[3];

    event_data[1] = cmd_opcode & 0xff;
    event_data[2] = cmd_opcode >> 8;

    switch(cmd_opcode)
    {
    /* MCU Request Connection to an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_CONNECT:
        if (data_len == sizeof(bdaddr))
        {
            STREAM_TO_BDADDR(bdaddr, p_data);
            status = wiced_bt_hidh_open(bdaddr);
        }
        else
            status = WICED_BT_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Request disconnection of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_DISCONNECT:
        if (data_len == sizeof(handle))
        {
            STREAM_TO_UINT16(handle, p_data);
            status = wiced_bt_hidh_close(handle);
        }
        else
            status = WICED_BT_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Request Adds an HID Device (to allow it to reconnect) */
    case HCI_CONTROL_HIDH_COMMAND_ADD:
        if (data_len == sizeof(bdaddr))
        {
            STREAM_TO_BDADDR(bdaddr, p_data);
            status = wiced_bt_hidh_add(bdaddr);
        }
        else
            status = WICED_BT_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Request Removes an HID Device (to do not allow it to reconnect) */
    case HCI_CONTROL_HIDH_COMMAND_REMOVE:
        if (data_len == sizeof(bdaddr))
        {
            STREAM_TO_BDADDR(bdaddr, p_data);
            status = wiced_bt_hidh_remove(bdaddr);
        }
        else
            status = WICED_BT_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Requests to read the HID Descriptor of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_GET_DESCRIPTOR:
        if (data_len == sizeof(handle))
        {
            STREAM_TO_UINT16(handle, p_data);
            status = wiced_bt_hidh_get_descriptor(handle);
        }
        else
            status = WICED_BT_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Requests to Write/Set the HID Report of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_SET_REPORT:
        if (data_len >= (sizeof(handle)) + sizeof(channel) + sizeof(report_type) +
                         sizeof(report_id))
        {
            STREAM_TO_UINT16(handle, p_data);
            STREAM_TO_UINT8(channel, p_data);
            STREAM_TO_UINT8(report_type, p_data);
            STREAM_TO_UINT8(report_id, p_data);
            status = wiced_bt_hidh_set_report(handle, channel, report_type, report_id, p_data,
                    data_len - 5);
        }
        else
            status = WICED_BT_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Requests to Read/Get the HID Report of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_GET_REPORT:
        if (data_len >= (sizeof(handle)) + sizeof(report_type) + sizeof(report_id) +
                         sizeof(buffer_size))
        {
            STREAM_TO_UINT16(handle, p_data);
            STREAM_TO_UINT8(report_type, p_data);
            STREAM_TO_UINT8(report_id, p_data);
            STREAM_TO_UINT16(buffer_size, p_data);
            status = wiced_bt_hidh_get_report(handle, report_type, report_id, buffer_size);
        }
        else
            status = WICED_BT_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Requests to Set the HID Protocol of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_SET_PROTOCOL:
        if (data_len == (sizeof(handle)) + sizeof(protocol))
        {
            STREAM_TO_UINT16(handle, p_data);
            STREAM_TO_UINT8(protocol, p_data);
            status = wiced_bt_hidh_set_protocol(handle, protocol);
        }
        else
            status = WICED_BT_HIDH_STATUS_INVALID_PARAM;
        break;

    default:
        WICED_BT_TRACE("Unknown HIDH opcode:0x04X\n", cmd_opcode);
        status = WICED_BT_HIDH_STATUS_ERROR;
        break;
    }

    event_data[0] = status;

    wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_STATUS, event_data, 3);
}

/*
 * hci_control_hidh_cback
 * Handle HID Host events from HID Host library
 */
void hci_control_hidh_cback( wiced_bt_hidh_event_t event, wiced_bt_hidh_data_t *p_data )
{
    uint8_t event_data[sizeof(uint16_t) + sizeof(uint8_t) + HID_DEV_MTU_SIZE];
    uint8_t *p = event_data;
    uint8_t *p_buf;

    WICED_BT_TRACE("hci_control_hidh_cback event:%d\n", event);

    switch(event)
    {
    case WICED_BT_HIDH_OPEN_EVT:
        UINT8_TO_STREAM(p, p_data->open.status);
        BDADDR_TO_STREAM(p, p_data->open.bdaddr);
        UINT16_TO_STREAM(p, p_data->open.handle);
        wiced_transport_send_data( HCI_CONTROL_HIDH_EVENT_CONNECTED, event_data,
                (uint16_t)(p - event_data));
        break;

    case WICED_BT_HIDH_CLOSE_EVT:
        UINT16_TO_STREAM(p, p_data->close.handle);
        UINT8_TO_STREAM(p, p_data->close.reason);
        wiced_transport_send_data( HCI_CONTROL_HIDH_EVENT_DISCONNECTED, event_data,
                (uint16_t)(p - event_data));
        break;

    case WICED_BT_HIDH_DESCRIPTOR_EVT:
        /* Allocate a buffer to send the HID Descriptor which can be big (1 Kbyte) */
        p_buf = (uint8_t *)wiced_bt_get_buffer(
                sizeof(uint16_t) + sizeof(uint8_t) + p_data->descriptor.length);
        if (p_buf == NULL)
        {
            UINT16_TO_STREAM(p, p_data->descriptor.handle);
            UINT8_TO_STREAM(p, WICED_BT_HIDH_STATUS_MEM_FULL);
            wiced_transport_send_data( HCI_CONTROL_HIDH_EVENT_DESCRIPTOR, event_data,
                    (uint16_t)(p - event_data));
        }
        else
        {
            p = p_buf;
            UINT16_TO_STREAM(p, p_data->descriptor.handle);
            UINT8_TO_STREAM(p, p_data->descriptor.status);
            memcpy(p, p_data->descriptor.p_descriptor, p_data->descriptor.length);
            p += p_data->descriptor.length;
            wiced_transport_send_data( HCI_CONTROL_HIDH_EVENT_DESCRIPTOR, p_buf,
                    (uint16_t)(p - p_buf));
            wiced_bt_free_buffer(p_buf);
        }
        break;

    case WICED_BT_HIDH_REPORT_EVT:
        UINT16_TO_STREAM(p, p_data->report.handle);
        UINT8_TO_STREAM(p, p_data->report.report_id);
        memcpy(p, p_data->report.p_data, p_data->report.length);
        p += p_data->report.length;
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_REPORT, event_data,
                (uint16_t)(p - event_data));
        break;

    case WICED_BT_HIDH_SET_REPORT_EVT:
        UINT16_TO_STREAM(p, p_data->set_report.handle);
        UINT8_TO_STREAM(p, p_data->set_report.status);
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_SET_REPORT, event_data,
                (uint16_t)(p - event_data));
        break;

    case WICED_BT_HIDH_GET_REPORT_EVT:
        UINT16_TO_STREAM(p, p_data->get_report.handle);
        UINT8_TO_STREAM(p, p_data->get_report.status);
        memcpy(p, p_data->get_report.p_data, p_data->get_report.length);
        p += p_data->get_report.length;
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_GET_REPORT, event_data,
                (uint16_t)(p - event_data));
        break;

    case WICED_BT_HIDH_VIRTUAL_UNPLUG_EVT:
        UINT16_TO_STREAM(p, p_data->virtual_unplug.handle);
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_VIRTUAL_UNPLUG, event_data,
                (uint16_t)(p - event_data));
        break;

    case WICED_BT_HIDH_SET_PROTOCOL_EVT:
        UINT16_TO_STREAM(p, p_data->set_protocol.handle);
        UINT8_TO_STREAM(p, p_data->set_protocol.status);
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_SET_PROTOCOL, event_data,
                (uint16_t)(p - event_data));
        break;

    default:
        WICED_BT_TRACE("hci_control_hidh_cback Unknown HIDH event:%d\n", event);
        break;
    }
}

