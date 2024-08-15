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

#include "sparcommon.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble_hidh.h"
#include "wiced_bt_ble_hidh_audio.h"
#include "hci_control_ble_hidh.h"
#include "hci_control_api.h"
#include "hci_control.h"
#include "string.h"

/*
 * Local functions
 */
static void hci_control_ble_hidh_cback( wiced_bt_ble_hidh_event_t event,
        wiced_bt_ble_hidh_event_data_t *p_data );
static void hci_control_ble_hidh_audio_cback(wiced_bt_ble_hidh_audio_event_t event,
        wiced_bt_ble_hidh_audio_event_data_t *p_event_data);

/*
 * hci_control_ble_hidh_init
 */
void hci_control_ble_hidh_init( void )
{
    wiced_bt_ble_hidh_init(hci_control_ble_hidh_cback);
    wiced_bt_ble_hidh_audio_init(hci_control_ble_hidh_audio_cback);
}

/*
 * hci_control_ble_hidh_handle_command
 * Handle HID Host commands from MCU
 */
void hci_control_ble_hidh_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    wiced_bt_device_address_t bdaddr;
    wiced_bt_ble_address_type_t addr_type;
    uint8_t protocol;
    uint8_t channel;
    uint16_t handle;
    uint8_t report_type, report_id;
    uint8_t wakeup_cmd, wakeup_enable, wakeup_gpio, wakeup_polarity;
    wiced_bt_ble_hidh_status_t status;
    wiced_bt_ble_hidh_gatt_cache_t gatt_cache;
    int gatt_cache_size;
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
            /* Extract the BdAddr from the Wiced HCI command */
            STREAM_TO_BDADDR(bdaddr, p_data);

            /* To connect a BLE Device, we need to know the Address Type (Random or Public)
             * Let's try to retrieve it from either the last Scan or from the Security database.
             */
            addr_type = hci_control_get_address_type(bdaddr);
            if (addr_type > BLE_ADDR_RANDOM_ID)
            {
                status = WICED_BT_BLE_HIDH_STATUS_ERROR;
            }
            else
            {
                /* Open the BLE HID Connection */
                status = wiced_bt_ble_hidh_connect(bdaddr, addr_type);
            }
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Request disconnection of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_DISCONNECT:
        if (data_len == sizeof(handle))
        {
            /* Extract the BLE HID Connection Handle from the Wiced HCI command */
            STREAM_TO_UINT16(handle, p_data);
            /* Disconnect the BLE HID connection */
            status = wiced_bt_ble_hidh_disconnect(handle);
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Request Adds an HID Device (to allow it to reconnect) */
    case HCI_CONTROL_HIDH_COMMAND_ADD:
        if (data_len == sizeof(bdaddr))
        {
            /* Extract the BdAddr from the Wiced HCI command */
            STREAM_TO_BDADDR(bdaddr, p_data);

            /* To Add a BLE Device, we need to know the Address Type (Random or Public)
             * Let's try to retrieve it from either the last Scan or from the Security database.
             */
            addr_type = hci_control_get_address_type(bdaddr);
            if (addr_type > BLE_ADDR_RANDOM_ID)
            {
                status = WICED_BT_BLE_HIDH_STATUS_ERROR;
            }
            else
            {
                /* Retrieve the BLE HID GATT Cache for this device */
                gatt_cache_size = hci_control_nvram_read_ble_hidh_gatt_cache(bdaddr, &gatt_cache);
                if (gatt_cache_size != sizeof(gatt_cache))
                {
                    WICED_BT_TRACE("Err: hci_control_ble_hidh_handle_command ADD without GATT Cache\n");
                    /* No GATT Cache. Add it anyway. The BLE HID Host library will have to browse &
                     * configure the GATT attributes from the device (not very efficient) */
                    status = wiced_bt_ble_hidh_add(bdaddr, addr_type, NULL);
                }
                else
                {
                    /* Add the BLE HID Device. We will be able to receive the HID Reports immediately. */
                    status = wiced_bt_ble_hidh_add(bdaddr, addr_type, &gatt_cache);
                }
            }
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Request Removes an HID Device (to do not allow it to reconnect) */
    case HCI_CONTROL_HIDH_COMMAND_REMOVE:
        if (data_len == sizeof(bdaddr))
        {
            /* Extract the BdAddr from the Wiced HCI command */
            STREAM_TO_BDADDR(bdaddr, p_data);
            /* Remove this BLE HID Device. It will not be allowed to reconnect. */
            status = wiced_bt_ble_hidh_remove(bdaddr);
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Requests to read the HID Descriptor of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_GET_DESCRIPTOR:
        if (data_len == sizeof(handle))
        {
            /* Extract the BLE HID Connection Handle from the Wiced HCI command */
            STREAM_TO_UINT16(handle, p_data);
            /* Read the HID Descriptor */
            status = wiced_bt_ble_hidh_get_descriptor(handle);
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Requests to Write/Set the HID Report of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_SET_REPORT:
        if (data_len >= (sizeof(handle)) + sizeof(channel) + sizeof(report_type) +
                         sizeof(report_id))
        {
            /* Extract the Set Report parameters (Connection Handle, etc) from the Wiced HCI command */
            STREAM_TO_UINT16(handle, p_data);
            STREAM_TO_UINT8(channel, p_data);   /* Ignored for BLE HID */
            STREAM_TO_UINT8(report_type, p_data);
            STREAM_TO_UINT8(report_id, p_data);
            /* Set the BLE HID Report */
            status = wiced_bt_ble_hidh_set_report(handle, report_type, report_id, p_data,
                    data_len - 5);
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Requests to Read/Get the HID Report of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_GET_REPORT:
        if (data_len >= (sizeof(handle)) + sizeof(report_type) + sizeof(report_id) +
                sizeof(buffer_size))
        {
            /* Extract the Get Report parameters (Connection Handle, etc) from the Wiced HCI command */
            STREAM_TO_UINT16(handle, p_data);
            STREAM_TO_UINT8(report_type, p_data);
            STREAM_TO_UINT8(report_id, p_data);
            STREAM_TO_UINT16(buffer_size, p_data);
            /* Get the BLE HID Report */
            status = wiced_bt_ble_hidh_get_report(handle, report_type, report_id, buffer_size);
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    /* MCU Requests to Set the HID Protocol of an HID Device */
    case HCI_CONTROL_HIDH_COMMAND_SET_PROTOCOL:
        if (data_len == (sizeof(handle)) + sizeof(protocol))
        {
            /* Extract the Set Protocol parameters (Connection Handle, etc) from the Wiced HCI command */
            STREAM_TO_UINT16(handle, p_data);
            STREAM_TO_UINT8(protocol, p_data);
            /* Set the BLE HID Protocol */
            status = wiced_bt_ble_hidh_set_protocol(handle, protocol);
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    case HCI_CONTROL_HIDH_COMMAND_WAKEUP_PATTERN_SET:
        if (data_len >= (sizeof(bdaddr) + sizeof(wakeup_cmd) + sizeof(report_id)))
        {
            /* Extract the BdAddr from the Wiced HCI command */
            STREAM_TO_BDADDR(bdaddr, p_data);
            STREAM_TO_UINT8(wakeup_cmd, p_data);
            STREAM_TO_UINT8(report_id, p_data);
            /* Set the BLE HID WakeUp Pattern */
            status = wiced_bt_ble_hidh_wakeup_pattern_set(bdaddr,
                    wakeup_cmd, report_id, p_data, data_len - 8);
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    case HCI_CONTROL_HIDH_COMMAND_WAKEUP_CONTROL:
        if (data_len >= (sizeof(wakeup_enable) + sizeof(wakeup_gpio) + sizeof(wakeup_polarity)))
        {
            /* Extract the BdAddr from the Wiced HCI command */
            STREAM_TO_UINT8(wakeup_enable, p_data);
            STREAM_TO_UINT8(wakeup_gpio, p_data);
            STREAM_TO_UINT8(wakeup_polarity, p_data);
            /* Set the BLE HID WakeUp Pattern */
            status = wiced_bt_ble_hidh_wakeup_pattern_control(wakeup_enable, wakeup_gpio,
                    wakeup_polarity, p_data, data_len - 3);
        }
        else
            status = WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
        break;

    default:
        WICED_BT_TRACE("Unknown HIDH opcode:0x%04X\n", cmd_opcode);
        status = WICED_BT_BLE_HIDH_STATUS_ERROR;
        break;
    }

    event_data[0] = status;

    /* Send back the Command status to the Host */
    wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_STATUS, event_data, 3);
}

/*
 * hci_control_ble_hidh_cback
 * Handle HID Host events from HID Host library
 */
static void hci_control_ble_hidh_cback( wiced_bt_ble_hidh_event_t event,
        wiced_bt_ble_hidh_event_data_t *p_data )
{
    WICED_BT_TRACE("hci_control_ble_hidh_cback event:%d\n", event);
    uint8_t event_data[sizeof(uint16_t) + sizeof(uint8_t) + HID_DEV_MTU_SIZE];
    uint8_t *p = event_data;
    uint8_t *p_buf;
    wiced_result_t result;

    switch(event)
    {
    /* BLE HID Device connected */
    case WICED_BT_BLE_HIDH_OPEN_EVT:
        UINT8_TO_STREAM(p, p_data->connected.status);
        BDADDR_TO_STREAM(p, p_data->connected.bdaddr);
        UINT16_TO_STREAM(p, p_data->connected.handle);
        wiced_transport_send_data( HCI_CONTROL_HIDH_EVENT_CONNECTED, event_data,
                (uint16_t)(p - event_data));
        break;

    /* BLE HID Device disconnected */
    case WICED_BT_BLE_HIDH_CLOSE_EVT:
        UINT16_TO_STREAM(p, p_data->disconnected.handle);
        UINT8_TO_STREAM(p, p_data->disconnected.reason);
        wiced_transport_send_data( HCI_CONTROL_HIDH_EVENT_DISCONNECTED, event_data,
                (uint16_t)(p - event_data));
        break;

    /* BLE HID Device Descriptor */
    case WICED_BT_BLE_HIDH_DESCRIPTOR_EVT:
        /* Allocate a buffer to send the HID Descriptor which can be big (up to 512 bytes) */
        p_buf = wiced_transport_allocate_buffer(p_transport_pool);
        if (p_buf == NULL)
        {
            WICED_BT_TRACE("hci_control_ble_hidh_cback MemFull\n");
            UINT16_TO_STREAM(p, p_data->descriptor.handle);
            UINT8_TO_STREAM(p, WICED_BT_BLE_HIDH_STATUS_MEM_FULL);
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
            result = wiced_transport_send_buffer( HCI_CONTROL_HIDH_EVENT_DESCRIPTOR, p_buf,
                    (uint16_t)(p - p_buf));
            if (result != WICED_BT_SUCCESS)
                WICED_BT_TRACE("Err: wiced_transport_send_buffer failed:%d\n", result);
        }
        break;

    /* BLE HID Device GATT Cache information (to be saved in NVRAM) */
    case WICED_BT_BLE_HIDH_GATT_CACHE_EVT:
        hci_control_nvram_write_ble_hidh_gatt_cache(p_data->gatt_cache.bdaddr,
                &p_data->gatt_cache, WICED_TRUE);
        break;

    /* BLE HID Device Report Received (keypressed, mouse movement, etc) */
    case WICED_BT_BLE_HIDH_REPORT_EVT:
        UINT16_TO_STREAM(p, p_data->report.handle);
        UINT8_TO_STREAM(p, p_data->report.report_id);
        memcpy(p, p_data->report.p_data, p_data->report.length);
        p += p_data->report.length;
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_REPORT, event_data,
                (uint16_t)(p - event_data));
        break;

    /* BLE HID Device Set Report confirmation */
    case WICED_BT_BLE_HIDH_SET_REPORT_EVT:
        UINT16_TO_STREAM(p, p_data->set_report.handle);
        UINT8_TO_STREAM(p, p_data->set_report.status);
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_SET_REPORT, event_data,
                (uint16_t)(p - event_data));
        break;

    /* BLE HID Device Get Report event */
    case WICED_BT_BLE_HIDH_GET_REPORT_EVT:
        UINT16_TO_STREAM(p, p_data->get_report.handle);
        UINT8_TO_STREAM(p, p_data->get_report.status);
        memcpy(p, p_data->get_report.p_data, p_data->get_report.length);
        p += p_data->get_report.length;
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_GET_REPORT, event_data,
                (uint16_t)(p - event_data));
        break;

    /* BLE HID Device Set Protocol confirmation */
    case WICED_BT_BLE_HIDH_SET_PROTOCOL_EVT:
        UINT16_TO_STREAM(p, p_data->set_protocol.handle);
        UINT8_TO_STREAM(p, p_data->set_protocol.status);
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_SET_PROTOCOL, event_data,
                (uint16_t)(p - event_data));
        break;

    /* BLE HID Device Unplugged event */
    case WICED_BT_BLE_HIDH_VIRTUAL_UNPLUG_EVT:
        UINT16_TO_STREAM(p, p_data->virtual_unplug.handle);
        wiced_transport_send_data(HCI_CONTROL_HIDH_EVENT_VIRTUAL_UNPLUG, event_data,
                (uint16_t)(p - event_data));
        break;

    default:
        WICED_BT_TRACE("hci_control_ble_hidh_cback Unknown HIDH event:%d\n", event);
        break;
    }
}

/*
 * hci_control_ble_hidh_audio_cback
 */
static void hci_control_ble_hidh_audio_cback(wiced_bt_ble_hidh_audio_event_t event,
        wiced_bt_ble_hidh_audio_event_data_t *p_event_data)
{
    WICED_BT_TRACE("hci_control_ble_hidh_cback event:%d\n", event);
    uint8_t event_data[sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t)];
    uint8_t *p = event_data;
    uint8_t *p_buf;
    wiced_result_t result;

    switch(event)
    {
    case WICED_BT_BLE_HIDH_AUDIO_START_EVT:
        WICED_BT_TRACE("hci_control_ble_hidh_audio_cback Start hdl:%d format:%d NbChannel:%d freq:%d\n",
                p_event_data->start.handle, p_event_data->start.format, p_event_data->start.channel_nb,
                p_event_data->start.frequency);
        UINT16_TO_STREAM(p, p_event_data->start.handle);
        UINT8_TO_STREAM(p, p_event_data->start.format);
        UINT8_TO_STREAM(p, p_event_data->start.channel_nb);
        UINT16_TO_STREAM(p, p_event_data->start.frequency);
        wiced_transport_send_data( HCI_CONTROL_HIDH_EVENT_AUDIO_START, event_data,
                (uint16_t)(p - event_data));
        break;

    case WICED_BT_BLE_HIDH_AUDIO_STOP_EVT:
        WICED_BT_TRACE("hci_control_ble_hidh_audio_cback Stop hdl:%d\n",
                p_event_data->stop.handle);
        UINT16_TO_STREAM(p, p_event_data->stop.handle);
        wiced_transport_send_data( HCI_CONTROL_HIDH_EVENT_AUDIO_STOP, event_data,
                (uint16_t)(p - event_data));
        break;

    case WICED_BT_BLE_HIDH_AUDIO_RX_DATA_EVT:
        WICED_BT_TRACE("hci_control_ble_hidh_audio_cback RxData hdl:%d length:%d\n",
                p_event_data->rx_data.handle, p_event_data->rx_data.length);
        /* Allocate a buffer to send the Audio Data (typically 240 bytes) */
        p_buf = wiced_transport_allocate_buffer(p_transport_pool);
        if (p_buf == NULL)
        {
            WICED_BT_TRACE("hci_control_ble_hidh_audio_cback MemFull\n");
        }
        else
        {
            p = p_buf;
            UINT16_TO_STREAM(p, p_event_data->rx_data.handle);
            memcpy(p, p_event_data->rx_data.p_data, p_event_data->rx_data.length);
            p += p_event_data->rx_data.length;
            result = wiced_transport_send_buffer( HCI_CONTROL_HIDH_EVENT_AUDIO_DATA, p_buf,
                    (uint16_t)(p - p_buf));
            if (result != WICED_BT_SUCCESS)
                WICED_BT_TRACE("Err: wiced_transport_send_buffer failed:%d\n", result);
        }
        break;

    default:
        WICED_BT_TRACE("hci_control_ble_hidh_audio_cback event:%d\n", event);
        break;
    }
}

