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
 * This file implement BLE application controlled over UART. The GATT database is set by MCU,
 * and all the communications are also handled by MCU.
 *
 */
#include "wiced.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_uuid.h"
#include "wiced_memory.h"
#include "wiced_timer.h"

#include "hci_control.h"
#include "wiced_app_cfg.h"
#include "wiced_bt_gatt_db.h"
#include "string.h"
#include "wiced_transport.h"


/******************************************************
 *                     Constants
 ******************************************************/
#define LE_CONTROL_MAX_CONNECTIONS                  20
#define LE_CONTROL_MAX_ADV_ELEM_NUM                 5
#define LE_CONTROL_CONNECT_TIMEOUT                  10

#define LE_CONTROL_STATE_IDLE                       0
#define LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES  1
#define LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS   2
#define LE_CONTROL_STATE_DISCOVER_DESCRIPTORS       3
#define LE_CONTROL_STATE_READ_VALUE                 4
#define LE_CONTROL_STATE_WRITE_VALUE                5
#define LE_CONTROL_STATE_WRITE_NO_RESPONSE_VALUE    6
#define LE_CONTROL_STATE_NOTIFY_VALUE               7
#define LE_CONTROL_STATE_INDICATE_VALUE             8
#define LE_CONTROL_STATE_WRITE_DESCRIPTOR_VALUE     9
#define LE_CONTROL_STATE_DISCONNECTING              10

/******************************************************
 *                     Structures
 ******************************************************/
typedef struct
{
    uint8_t       state;           // Application discovery state
    wiced_bool_t  indication_sent; // TRUE if indication sent and not acked
    BD_ADDR       bd_addr;
    uint16_t      conn_id;         // Connection ID used for exchange with the stack
    uint16_t      peer_mtu;        // MTU received in the MTU request (or 23 if peer did not send MTU request)
    uint8_t       role;            // HCI_ROLE_MASTER or HCI_ROLE_SLAVE
} le_control_conn_state_t;

typedef struct
{
    le_control_conn_state_t conn[LE_CONTROL_MAX_CONNECTIONS + 1];
} le_control_cb_t;;

typedef struct
{
    wiced_bool_t  tx_buf_saved;
    uint16_t      tx_buf_conn_id;
    uint16_t      tx_buf_type;
    uint16_t      tx_buf_len;
    uint16_t      tx_buf_handle;
    uint8_t       tx_buf_data[HCI_CONTROL_GATT_COMMAND_MAX_TX_BUFFER];
} le_control_pending_tx_buffer_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

static le_control_cb_t                le_control_cb;
static wiced_timer_t                  hci_control_le_connect_timer;
static BD_ADDR                        hci_control_le_remote_bdaddr;
static le_control_pending_tx_buffer_t hci_control_le_pending_tx_buffer;

/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_bt_gatt_status_t hci_control_le_gatt_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static wiced_result_t         hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t         hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static void                   hci_control_le_connect_timeout(uint32_t count);
static wiced_result_t         hci_control_le_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data);
static void                   hci_control_le_notification_handler(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);
static void                   hci_control_le_indication_handler(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);
static void                   hci_control_le_process_data(uint16_t type, uint16_t conn_id, uint16_t handle, uint8_t *data, int len);
static void                   hci_control_le_send_scan_state_event(uint8_t state);
static void                   hci_control_le_send_advertisement_state_event(uint8_t state);
static void                   hci_control_le_send_advertisement_report(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);
static void                   hci_control_le_send_connect_event(uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role);
static void                   hci_control_le_send_disconnect_evt(uint8_t reason, uint16_t con_handle);
static void                   hci_control_le_send_discover_complete(uint16_t con_handle);
static void                   hci_control_le_send_discovered_service16(uint16_t con_handle, uint16_t uuid, uint16_t s_handle, uint16_t e_handle);
static void                   hci_control_le_send_discovered_service128(uint16_t con_handle, uint8_t *uuid, uint16_t s_handle, uint16_t e_handle);
static void                   hci_control_le_send_discovered_characteristic16(uint16_t con_handle, uint16_t char_handle, uint8_t properties, uint16_t handle, uint16_t uuid);
static void                   hci_control_le_send_discovered_characteristic128(uint16_t con_handle, uint16_t char_handle, uint8_t properties, uint16_t handle, uint8_t *uuid);
static void                   hci_control_le_send_discovered_descriptor16(uint16_t con_handle, uint16_t handle, uint16_t uuid);
static void                   hci_control_le_send_discovered_descriptor128(uint16_t con_handle, uint16_t handle, uint8_t *uuid);
static void                   hci_control_le_send_read_rsp(uint16_t con_handle, uint8_t *data, int len);
static void                   hci_control_le_send_read_req(uint16_t con_handle, uint16_t handle);
static void                   hci_control_le_send_write_completed(uint16_t con_handle, uint8_t result);
static void                   hci_control_le_num_complete_callback(void);
static void                   hci_control_le_handle_scan_cmd(wiced_bool_t enable, wiced_bool_t filter_duplicates);
static void                   hci_control_le_handle_advertise_cmd(wiced_bool_t enable);
static void                   hci_control_le_handle_connect_cmd(uint8_t addr_type, BD_ADDR addr);
static void                   hci_control_le_handle_cancel_connect_cmd(uint8_t addr_type, BD_ADDR addr);
static void                   hci_control_le_handle_disconnect_cmd(uint16_t conn_id);
static void                   hci_control_le_handle_service_discovery(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle);
static void                   hci_control_le_handle_characteristic_discovery(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle);
static void                   hci_control_le_handle_descriptor_discovery(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle);
static void                   hci_control_le_handle_delete_nvram_info_and_initiate_pair (BD_ADDR addr, uint16_t nvram_id);
static void                   hci_control_le_handle_get_identity_address (BD_ADDR bd_addr);
static void                   hci_control_le_handle_set_channel_classification (uint8_t ble_channel_map[ HCI_BLE_CHNL_MAP_SIZE ]);
static void                   hci_control_le_handle_set_advertisement_raw_data (uint8_t* p, uint32_t data_len);
static void                   hci_control_le_handle_db_init (void);
static void                   hci_control_le_handle_db_primary_service_add (uint8_t* p, uint32_t data_len);
static void                   hci_control_le_handle_db_secondary_service_add (uint8_t* p, uint32_t data_len);
static void                   hci_control_le_handle_db_included_service_add (uint8_t* p, uint32_t data_len);
static void                   hci_control_le_handle_db_characteristic_add (uint8_t* p, uint32_t data_len);
static void                   hci_control_le_handle_db_descriptor_add(uint8_t* p, uint32_t data_len);
static void                   hci_control_le_handle_read_req(uint16_t con_handle, uint16_t handle);
static void                   hci_control_le_handle_read_rsp(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);
static wiced_bool_t           hci_control_le_handle_write_cmd(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);
static void                   hci_control_le_handle_write_req(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);
static void                   hci_control_le_handle_write_rsp(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);
static wiced_bool_t           hci_control_le_handle_notify(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);
static void                   hci_control_le_handle_indicate(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);

static void                   hci_control_le_set_advertisement_data(uint8_t *data, int len);
static void                   hci_control_le_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);


/*
 * Enable LE Control
 */
void hci_control_le_enable(void)
{
    wiced_bt_gatt_status_t  gatt_status;

    WICED_BT_TRACE("hci_control_le_enable\n");

    memset(&le_control_cb,                0, sizeof(le_control_cb));
    memset(&hci_control_le_pending_tx_buffer, 0, sizeof(hci_control_le_pending_tx_buffer));

    /* GATT registration */
    gatt_status = wiced_bt_gatt_register(hci_control_le_gatt_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register status %d\n", gatt_status);

    /* Initialize connection timer */
    wiced_init_timer(&hci_control_le_connect_timer, &hci_control_le_connect_timeout, 0, WICED_SECONDS_TIMER);
}


/*
 * Convert a raw advertisement data buffer to adv_elem list and send the adv data
 * to the stack. This function assumes the adv data content conforms to the SIG adv spec.
 */
void hci_control_le_set_advertisement_data(uint8_t *data, int len)
{
    wiced_bt_ble_advert_elem_t adv_elem[LE_CONTROL_MAX_ADV_ELEM_NUM];
    uint8_t num_elem = 0;
    
    /* We assume standard advertisement raw data follows the SIG standard */
    while (len >= 3)
    {
        adv_elem[num_elem].advert_type = *data++;
        adv_elem[num_elem].len         = *data++;
        adv_elem[num_elem].p_data      = data;
        data += adv_elem[num_elem].len;
        len -= (2 + adv_elem[num_elem].len);
        num_elem++;
    }

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*
 * Process advertisement packet received from LE scan
 */
void hci_control_le_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    if (p_scan_result)
    {
        WICED_BT_TRACE("Device: %B\n", p_scan_result->remote_bd_addr);
        hci_control_le_send_advertisement_report(p_scan_result, p_adv_data);
    }
    else
    {
        WICED_BT_TRACE("Scan completed\n");
    }
}

/*
 * Process connection up event
 */
wiced_result_t hci_control_le_connection_up(wiced_bt_gatt_connection_status_t *p_status)
{
    uint32_t  conn_id = p_status->conn_id;
    uint8_t   role;

    wiced_bt_dev_get_role(p_status->bd_addr, &role, p_status->transport);
    le_control_cb.conn[conn_id].role = role;

    WICED_BT_TRACE("hci_control_le_connection_up id:%d bd (%B) role:%d\n:", p_status->conn_id, p_status->bd_addr, role);

    /* Stop LE connection timer*/
    wiced_stop_timer(&hci_control_le_connect_timer);

    memcpy(le_control_cb.conn[conn_id].bd_addr, p_status->bd_addr, BD_ADDR_LEN);
    le_control_cb.conn[conn_id].state    = LE_CONTROL_STATE_IDLE;
    le_control_cb.conn[conn_id].conn_id  = p_status->conn_id;
    le_control_cb.conn[conn_id].peer_mtu = GATT_DEF_BLE_MTU_SIZE;

    // if we connected as a master, configure slave to enable notifications
    if (hci_control_cb.pairing_allowed && (role == HCI_ROLE_MASTER))
    {
        wiced_result_t res = wiced_bt_dev_sec_bond(p_status->bd_addr, p_status->addr_type,
                p_status->transport, 0, NULL);
        WICED_BT_TRACE("master role: after bond res:%d\n", res);
    }

    hci_control_le_send_connect_event(p_status->addr_type, p_status->bd_addr, conn_id, role);
    return (WICED_SUCCESS);
}

/*
 * Process connection down event
 */
wiced_result_t hci_control_le_connection_down(wiced_bt_gatt_connection_status_t *p_status)
{
    uint16_t  conn_id = p_status->conn_id;

    WICED_BT_TRACE( "le_connection_down conn_id:%x Disc_Reason: %02x\n", conn_id, p_status->reason );

    /* Stop LE connection timer*/
    wiced_stop_timer(&hci_control_le_connect_timer);

    le_control_cb.conn[conn_id].state   = LE_CONTROL_STATE_IDLE;
    le_control_cb.conn[conn_id].conn_id = 0;

    if ((conn_id == hci_control_le_pending_tx_buffer.tx_buf_conn_id) &&
         hci_control_le_pending_tx_buffer.tx_buf_saved)
    {
        hci_control_le_pending_tx_buffer.tx_buf_saved = WICED_FALSE;
    }

    hci_control_le_send_disconnect_evt(p_status->reason, conn_id);

    return (WICED_SUCCESS);
}


/*
 * Process connection status callback
 */
wiced_result_t hci_control_le_conn_status_callback(wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        return hci_control_le_connection_up(p_status);
    }
    else
    {
        return hci_control_le_connection_down(p_status);
    }
}

/*
 * Operation complete received from the GATT server
 */
wiced_result_t hci_control_le_gatt_operation_comp_cb(wiced_bt_gatt_operation_complete_t *p_complete)
{
    uint16_t conn_id = p_complete->conn_id;

    switch (p_complete->op)
    {
    case GATTC_OPTYPE_DISCOVERY:
        WICED_BT_TRACE("!!! Disc compl conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state);
        break;

    case GATTC_OPTYPE_READ:
        // read response received, pass it up and set state to idle
        WICED_BT_TRACE("Read response conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state);
        if (le_control_cb.conn[conn_id].state == LE_CONTROL_STATE_READ_VALUE)
        {
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_IDLE;
            if (p_complete->status != WICED_SUCCESS)
                hci_control_le_send_read_rsp(conn_id, NULL, 0);
            else
                hci_control_le_send_read_rsp(conn_id, p_complete->response_data.att_value.p_data,
                        p_complete->response_data.att_value.len);
        }
        break;

    case GATTC_OPTYPE_WRITE:
    case GATTC_OPTYPE_EXE_WRITE:
        // write response received, pass it up and set state to idle
        WICED_BT_TRACE("Write response conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state);
        if (le_control_cb.conn[conn_id].state == LE_CONTROL_STATE_WRITE_VALUE)
        {
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_IDLE;
            hci_control_le_send_write_completed(conn_id, p_complete->status);
        }
        break;

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE("Config conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state);
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        WICED_BT_TRACE("Notification conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state);
        hci_control_le_notification_handler(conn_id,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len);
        break;

    case GATTC_OPTYPE_INDICATION:
        WICED_BT_TRACE("Indication conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state);
        hci_control_le_indication_handler(conn_id,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len);
        break;
    }
    return (WICED_SUCCESS);
}

/*
 * Discovery result received from the GATT server
 */
wiced_result_t hci_control_le_gatt_disc_result_cb(wiced_bt_gatt_discovery_result_t *p_result)
{
    uint16_t conn_id = p_result->conn_id;

    WICED_BT_TRACE("Discovery result conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state);

    switch (le_control_cb.conn[conn_id].state)
    {
    case LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES:
        if ((p_result->discovery_type == GATT_DISCOVER_SERVICES_ALL) ||
            (p_result->discovery_type == GATT_DISCOVER_SERVICES_BY_UUID))
        {
            WICED_BT_TRACE("Service s:%04x e:%04x uuid:%04x\n", p_result->discovery_data.group_value.s_handle,
                    p_result->discovery_data.group_value.e_handle, p_result->discovery_data.group_value.service_type.uu.uuid16);

            // services on the server can be based on 2 or 16 bytes UUIDs
            if (p_result->discovery_data.group_value.service_type.len == 2)
            {
                hci_control_le_send_discovered_service16(conn_id, p_result->discovery_data.group_value.service_type.uu.uuid16,
                        p_result->discovery_data.group_value.s_handle, p_result->discovery_data.group_value.e_handle);
            }
            else
            {
                hci_control_le_send_discovered_service128(conn_id, p_result->discovery_data.group_value.service_type.uu.uuid128,
                        p_result->discovery_data.group_value.s_handle, p_result->discovery_data.group_value.e_handle);
            }
        }
        break;

    case LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS:
        if (p_result->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
        {
            WICED_BT_TRACE("Found cha - uuid:%04x, hdl:%04x\n", p_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid16, p_result->discovery_data.characteristic_declaration.handle);

            // characteristic can be based on 2 or 16 bytes UUIDs
            if (p_result->discovery_data.characteristic_declaration.char_uuid.len == 2)
            {
                hci_control_le_send_discovered_characteristic16(conn_id, p_result->discovery_data.characteristic_declaration.handle,
                        p_result->discovery_data.characteristic_declaration.characteristic_properties,
                        p_result->discovery_data.characteristic_declaration.val_handle,
                        p_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid16);
            }
            else
            {
                hci_control_le_send_discovered_characteristic128(conn_id, p_result->discovery_data.characteristic_declaration.handle,
                        p_result->discovery_data.characteristic_declaration.characteristic_properties,
                        p_result->discovery_data.characteristic_declaration.val_handle,
                        p_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid128);
            }
        }
        break;

    case LE_CONTROL_STATE_DISCOVER_DESCRIPTORS:
        if (p_result->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
        {
            WICED_BT_TRACE("Found descr - uuid:%04x handle:%04x\n", p_result->discovery_data.char_descr_info.type.uu.uuid16, p_result->discovery_data.char_descr_info.handle);

            // descriptor can be based on 2 or 16 bytes UUIDs
            if (p_result->discovery_data.char_descr_info.type.len == 2)
            {
                hci_control_le_send_discovered_descriptor16(conn_id, p_result->discovery_data.char_descr_info.handle, p_result->discovery_data.char_descr_info.type.uu.uuid16);
            }
            else
            {
                hci_control_le_send_discovered_descriptor128(conn_id, p_result->discovery_data.char_descr_info.handle, p_result->discovery_data.char_descr_info.type.uu.uuid128);
            }
        }
        break;

    default:
        WICED_BT_TRACE("ignored\n");
        break;
    }
    return (WICED_SUCCESS);
}

/*
 * process discovery complete notification from the stack
 */
wiced_result_t hci_control_le_gatt_disc_comp_cb(wiced_bt_gatt_discovery_complete_t *p_data)
{
    /* if we got here peer returned no more services, or we read up to the handle
     * asked by client, report complete.
     */
    le_control_cb.conn[p_data->conn_id].state = LE_CONTROL_STATE_IDLE;
    hci_control_le_send_discover_complete(p_data->conn_id);

    return (WICED_SUCCESS);
}


/*
 * The function invoked on timeout of hci_control_le_connect_timer
 */
void hci_control_le_connect_timeout(uint32_t count)
{
    WICED_BT_TRACE("%s\n", __func__);

    /* Stop le connection timer*/
    wiced_stop_timer(&hci_control_le_connect_timer);

    /* Cancel connection request */
    wiced_bt_gatt_cancel_connect(hci_control_le_remote_bdaddr, WICED_TRUE);
}


/*
 * This function is called when peer issues a Read Request for a characteristic value. Send
 * the request to MCU and return PENDING - the Read Response will be sent when data is ready.
 */
wiced_bt_gatt_status_t hci_control_le_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t *p_req)
{
    uint8_t   buf[8];
    uint8_t   *p = buf;

    WICED_BT_TRACE("%s: conn_id = %d, handle = %d, offset = %d, len = %d\n", __func__, conn_id, p_req->handle, p_req->offset, *p_req->p_val_len);

    UINT16_TO_STREAM(p, conn_id);
    UINT16_TO_STREAM(p, p_req->handle);
    UINT16_TO_STREAM(p, p_req->offset);
    UINT16_TO_STREAM(p, *p_req->p_val_len);

    wiced_transport_send_data(HCI_CONTROL_GATT_EVENT_READ_REQUEST, buf, (int)(p - buf));
    return WICED_BT_GATT_PENDING;
}

/*
 * This function is called when peer issues a Write request to set a characteristic value. Send
 * the request to MCU and return PENDING - the Write Response will be sent by MCU.
 */
wiced_result_t hci_control_le_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_req)
{
    uint8_t   buf[300];
    uint8_t   *p = buf;

    WICED_BT_TRACE("%s: conn_id:%d handle:%d offset:%d len:%d\n", __func__, conn_id, p_req->handle, p_req->offset, p_req->val_len);

    UINT16_TO_STREAM(p, conn_id);
    UINT16_TO_STREAM(p, p_req->handle);
    UINT16_TO_STREAM(p, p_req->offset);
    UINT16_TO_STREAM(p, p_req->val_len);
    ARRAY_TO_STREAM(p, p_req->p_val, p_req->val_len);

    wiced_transport_send_data(HCI_CONTROL_GATT_EVENT_WRITE_REQUEST, buf, (int)(p - buf));
    return WICED_BT_GATT_PENDING;
}


/*
 * This function is called when peer issues a GATTS_REQ_TYPE_WRITE_EXEC request.
 * Not supported, hence do nothing.
 */
wiced_result_t hci_control_le_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t flag)
{
    return (WICED_SUCCESS);
}


/*
 * This function handles the MTU setting command
 */
wiced_result_t hci_control_le_mtu_handler(uint16_t conn_id, uint16_t mtu)
{
    le_control_cb.conn[conn_id].peer_mtu = mtu;

    return (WICED_SUCCESS);
}

/*
 * Process the GATTS_REQ_TYPE_CONF message
 */
wiced_result_t hci_control_le_conf_handler(uint16_t conn_id, uint16_t handle)
{
    WICED_BT_TRACE("hci_control_le_conf_handler conn_id:%d state:%d handle:%x\n", conn_id, le_control_cb.conn[conn_id].state, handle);

    if (le_control_cb.conn[conn_id].state == LE_CONTROL_STATE_INDICATE_VALUE)
    {
        le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_IDLE;
        hci_control_le_send_write_completed(conn_id, 0);
    }
    return (WICED_SUCCESS);
}

/*
 * This is a GATT request callback
 */
wiced_bt_gatt_status_t hci_control_le_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_req)
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_SUCCESS;
    uint16_t               conn_id = p_req->conn_id;

    WICED_BT_TRACE("GATT request conn_id:%d type:%d\n", conn_id, p_req->request_type);

    switch (p_req->request_type)
    {
        case GATTS_REQ_TYPE_READ:
            result = hci_control_le_read_handler(p_req->conn_id, &p_req->data.read_req);
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = hci_control_le_write_handler(p_req->conn_id, &p_req->data.write_req);
             break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
            result = hci_control_le_write_exec_handler(p_req->conn_id, p_req->data.exec_write);
            break;

        case GATTS_REQ_TYPE_MTU:
            result = hci_control_le_mtu_handler(p_req->conn_id, p_req->data.mtu);
            break;

        case GATTS_REQ_TYPE_CONF:
            result = hci_control_le_conf_handler(p_req->conn_id, p_req->data.handle);
            break;

       default:
            break;
    }

    return result;
}


/*
 * This is a GATT client request callback
 */
wiced_bt_gatt_status_t hci_control_le_gatt_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hci_control_le_conn_status_callback(&p_data->connection_status);
        break;

    case GATT_OPERATION_CPLT_EVT:
        result = hci_control_le_gatt_operation_comp_cb(&p_data->operation_complete);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
        result = hci_control_le_gatt_disc_result_cb(&p_data->discovery_result);
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        result = hci_control_le_gatt_disc_comp_cb(&p_data->discovery_complete);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hci_control_le_gatt_req_cb(&p_data->attribute_request);
        break;

    default:
        break;
    }

    return result;
}


/*
 * This function sends write to the peer GATT server
 */
wiced_bt_gatt_status_t hci_control_le_send_write(uint8_t conn_id, uint16_t attr_handle, uint8_t *p_data, uint16_t len, wiced_bt_gatt_write_type_t type)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;

    // Allocating a buffer to send the write request
    wiced_bt_gatt_value_t *p_write = (wiced_bt_gatt_value_t*)wiced_bt_get_buffer(GATT_RESPONSE_SIZE(2));

    if (p_write)
    {
        p_write->handle   = attr_handle;
        p_write->offset   = 0;
        p_write->len      = len;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        memcpy(p_write->value, p_data, len);

        // Register with the server to receive notification
        status = wiced_bt_gatt_send_write (conn_id, type, p_write);

        WICED_BT_TRACE("wiced_bt_gatt_send_write ", status);

        wiced_bt_free_buffer(p_write);
    }
    return (status);
}


/*
 * Stack runs the advertisement state machine switching between high duty, low
 * duty, no advertisements, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_advert_state_changed(wiced_bt_ble_advert_mode_t mode)
{
    uint8_t hci_control_le_event = 0;

    WICED_BT_TRACE("Advertisement State Change:%d\n", mode);

    switch (mode)
    {
    case BTM_BLE_ADVERT_OFF:
        hci_control_le_event = LE_ADV_STATE_NO_DISCOVERABLE;
        break;
    case BTM_BLE_ADVERT_DIRECTED_HIGH:
    case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
    case BTM_BLE_ADVERT_NONCONN_HIGH:
    case BTM_BLE_ADVERT_DISCOVERABLE_HIGH:
        hci_control_le_event = LE_ADV_STATE_HIGH_DISCOVERABLE;
        break;
    case BTM_BLE_ADVERT_DIRECTED_LOW:
    case BTM_BLE_ADVERT_UNDIRECTED_LOW:
    case BTM_BLE_ADVERT_NONCONN_LOW:
    case BTM_BLE_ADVERT_DISCOVERABLE_LOW:
        hci_control_le_event = LE_ADV_STATE_LOW_DISCOVERABLE;
        break;
    }
    hci_control_le_send_advertisement_state_event(hci_control_le_event);
}

/*
 * Stack runs the scan state machine switching between high duty, low
 * duty, no scan, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_scan_state_changed(wiced_bt_ble_scan_type_t state)
{
    uint8_t hci_control_le_event = 0;

    WICED_BT_TRACE("Scan State Changed:%d\n", state);

    switch (state)
    {
    case BTM_BLE_SCAN_TYPE_NONE:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_NO_SCAN;
        break;
    case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_HIGH_SCAN;
        break;
    case BTM_BLE_SCAN_TYPE_LOW_DUTY:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_LOW_SCAN;
        break;
    }
    hci_control_le_send_scan_state_event(hci_control_le_event);
}

/*
 * This function is called when notification is received from the connected GATT Server
 */
void hci_control_le_notification_handler(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{
    WICED_BT_TRACE("Notification conn_id:%d handle:%04x len:%d\n", conn_id, handle, len);

    hci_control_le_process_data(HCI_CONTROL_GATT_EVENT_NOTIFICATION, conn_id, handle, p_data, len);
}

/*
 * This function is called when indication is received from the connected GATT Server
 */
void hci_control_le_indication_handler(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{
    WICED_BT_TRACE("Indication conn_id:%d handle:%04x len:%d\n", conn_id, handle, len);

    hci_control_le_process_data(HCI_CONTROL_GATT_EVENT_INDICATION, conn_id, handle, p_data, len);

    wiced_bt_gatt_send_indication_confirm(conn_id, handle);
}

/*
 * Handle various LE GAP and GATT commands received from the Control app.
 */
void hci_control_le_handle_command(uint16_t cmd_opcode, uint8_t* p, uint32_t data_len)
{
    WICED_BT_TRACE("%s: opcode = 0x%04x, len = %d\n", __func__, cmd_opcode, data_len);
    
    switch(cmd_opcode)
    {
    case HCI_CONTROL_LE_COMMAND_SCAN:
        hci_control_le_handle_scan_cmd((wiced_bool_t)p[0], (wiced_bool_t)p[1]);
        break;

    case HCI_CONTROL_LE_COMMAND_ADVERTISE:
        hci_control_le_handle_advertise_cmd((wiced_bool_t)p[0]);
        break;

    case HCI_CONTROL_LE_COMMAND_CONNECT:
        hci_control_le_handle_connect_cmd(*p, &p[1]);
        break;

    case HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT:
        hci_control_le_handle_cancel_connect_cmd(*p, &p[1]);
        break;

    case HCI_CONTROL_LE_COMMAND_DISCONNECT:
        hci_control_le_handle_disconnect_cmd(p[0] + (p[1] << 8));
        break;

    case HCI_CONTROL_LE_RE_PAIR:
        hci_control_le_handle_delete_nvram_info_and_initiate_pair (&p[0], p[6] + (p[7] << 8));
        break;

    case HCI_CONTROL_LE_COMMAND_GET_IDENTITY_ADDRESS:
        hci_control_le_handle_get_identity_address (&p[0]);
        break;

    case HCI_CONTROL_LE_COMMAND_SET_CHANNEL_CLASSIFICATION:
        hci_control_le_handle_set_channel_classification(&p[0]);
        break;

    case HCI_CONTROL_LE_COMMAND_SET_RAW_ADVERTISE_DATA:
        hci_control_le_handle_set_advertisement_raw_data(p, data_len);
        break;

    case HCI_CONTROL_GATT_COMMAND_DISCOVER_SERVICES:
        hci_control_le_handle_service_discovery(p[0] + (p[1] << 8), p[2] + (p[3] << 8), p[4] + (p[5] << 8));
        break;

    case HCI_CONTROL_GATT_COMMAND_DISCOVER_CHARACTERISTICS:
        hci_control_le_handle_characteristic_discovery(p[0] + (p[1] << 8), p[2] + (p[3] << 8), p[4] + (p[5] << 8));
        break;

    case HCI_CONTROL_GATT_COMMAND_DISCOVER_DESCRIPTORS:
        hci_control_le_handle_descriptor_discovery(p[0] + (p[1] << 8), p[2] + (p[3] << 8), p[4] + (p[5] << 8));
        break;

    case HCI_CONTROL_GATT_COMMAND_READ_REQUEST:
        hci_control_le_handle_read_req(p[0] + (p[1] << 8), p[2] + (p[3] << 8));
        break;

    case HCI_CONTROL_GATT_COMMAND_READ_RESPONSE:
        hci_control_le_handle_read_rsp(p[0] + (p[1] << 8), p[2] + (p[3] << 8), &p[4], data_len - 4);
        break;

    case HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND:
        hci_control_le_handle_write_cmd(p[0] + (p[1] << 8), p[2] + (p[3] << 8), &p[4], data_len - 4);
        break;

    case HCI_CONTROL_GATT_COMMAND_WRITE_REQUEST:
        hci_control_le_handle_write_req(p[0] + (p[1] << 8), p[2] + (p[3] << 8), &p[4], data_len - 4);
        break;

    case HCI_CONTROL_GATT_COMMAND_WRITE_RESPONSE:
        hci_control_le_handle_write_rsp(p[0] + (p[1] << 8), p[2] + (p[3] << 8), &p[4], data_len - 4);
        break;

    case HCI_CONTROL_GATT_COMMAND_NOTIFY:
        hci_control_le_handle_notify(p[0] + (p[1] << 8), p[2] + (p[3] << 8), &p[4], data_len - 4);
        break;

    case HCI_CONTROL_GATT_COMMAND_INDICATE:
        hci_control_le_handle_indicate(p[0] + (p[1] << 8), p[2] + (p[3] << 8), &p[4], data_len - 4);
        break;

    case HCI_CONTROL_GATT_COMMAND_DB_INIT:
        hci_control_le_handle_db_init();
        break;

    case HCI_CONTROL_GATT_COMMAND_DB_PRIMARY_SERVICE_ADD:
        hci_control_le_handle_db_primary_service_add(p, data_len);
        break;
        
    case HCI_CONTROL_GATT_COMMAND_DB_SECONDARY_SERVICE_ADD:
        hci_control_le_handle_db_secondary_service_add(p, data_len);
        break;
        
    case HCI_CONTROL_GATT_COMMAND_DB_INCLUDED_SERVICE_ADD:
        hci_control_le_handle_db_included_service_add(p, data_len);
        break;
        
    case HCI_CONTROL_GATT_COMMAND_DB_CHARACTERISTIC_ADD:
        hci_control_le_handle_db_characteristic_add(p, data_len);
        break;
        
    case HCI_CONTROL_GATT_COMMAND_DB_DESCRIPTOR_ADD:
        hci_control_le_handle_db_descriptor_add(p, data_len);
        break;
        
    default:
        WICED_BT_TRACE("Default case : %d, cmd_opcode:%x \n",__LINE__, cmd_opcode);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_UNKNOWN_COMMAND);
        break;
    }
}

/*
 * handle scan command from UART
 */
void hci_control_le_handle_scan_cmd(wiced_bool_t enable, wiced_bool_t filter_duplicates)
{
    wiced_result_t status;
    if (enable)
    {
        status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, filter_duplicates, hci_control_le_scan_result_cback);
    }
    else
    {
        status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, filter_duplicates, hci_control_le_scan_result_cback);
    }
    WICED_BT_TRACE("hci_control_le_handle_scan_cmd:%d status:%x\n", enable, status);

    if((status == WICED_BT_SUCCESS) || (status == WICED_BT_PENDING))
    {
        status = HCI_CONTROL_STATUS_SUCCESS;
    }
    else
    {
        status = HCI_CONTROL_STATUS_FAILED;
    }

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status);
}

/*
 * handle advertise command from UART
 */
void hci_control_le_handle_advertise_cmd(wiced_bool_t enable)
{
    if (enable)
    {
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
    }
    else
    {
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
    }
    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}


/*
 * handle connect command from UART
 */
void hci_control_le_handle_connect_cmd(uint8_t addr_type, BD_ADDR addr)
{
    int                    i;
    wiced_bool_t status;

    /* Initiate the connection */
    STREAM_TO_BDADDR(hci_control_le_remote_bdaddr,addr);

    status = wiced_bt_gatt_le_connect(hci_control_le_remote_bdaddr, addr_type,
            BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE);

    /* Start le connection timer */
    if (status == WICED_TRUE)
        wiced_start_timer(&hci_control_le_connect_timer, LE_CONTROL_CONNECT_TIMEOUT);

    WICED_BT_TRACE("wiced_bt_gatt_connect status %d, BDA %B, Addr Type %x\n",
                                          status, hci_control_le_remote_bdaddr, addr_type);

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, 
        status == WICED_TRUE ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED);
}

/*
 * handle cancel connect command from uart
 */
void hci_control_le_handle_cancel_connect_cmd(uint8_t addr_type, BD_ADDR addr)
{
    int                    i;
    BD_ADDR                bda;
    STREAM_TO_BDADDR(bda,addr);
    wiced_bt_gatt_cancel_connect (bda, WICED_TRUE);

    /* Stop le connection timer*/
    wiced_stop_timer(&hci_control_le_connect_timer);

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}

/*
 *  handle disconnect command from UART
 */
void hci_control_le_handle_disconnect_cmd(uint16_t conn_id)
{
    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if (le_control_cb.conn[conn_id].conn_id != conn_id)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_NOT_CONNECTED);
    }
    else
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);

        le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_DISCONNECTING;

        wiced_bt_gatt_disconnect(conn_id);
    }
}

/*
 * handle start services discovery command from UART
 */
void hci_control_le_handle_service_discovery(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t params;
    wiced_bt_gatt_status_t          rc;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if (le_control_cb.conn[conn_id].state != LE_CONTROL_STATE_IDLE)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE);
        WICED_BT_TRACE("illegal state:%d\n", le_control_cb.conn[conn_id].state);
    }
    else if ((s_handle > e_handle) || (s_handle == 0) || (e_handle == 0))
    {
        WICED_BT_TRACE("illegal handles:%04x-%04x\n", s_handle, e_handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else
    {
        // perform service search
        memset(&params, 0, sizeof(params));
        params.s_handle = s_handle;
        params.e_handle = e_handle;

        WICED_BT_TRACE("discover services conn_id:%d %04x-%04x\n", conn_id, s_handle, e_handle);
        if ((rc = wiced_bt_gatt_send_discover(conn_id, GATT_DISCOVER_SERVICES_ALL, &params)) == WICED_BT_GATT_SUCCESS)
        {
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES;
        }
        else
        {
            WICED_BT_TRACE("discover failed:0x%x\n", rc);
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED);
        }
    }
}

/*
 * handle start characteristics discovery command from UART
 */
void hci_control_le_handle_characteristic_discovery(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t params;
    wiced_bt_gatt_status_t          rc;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if (le_control_cb.conn[conn_id].state != LE_CONTROL_STATE_IDLE)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE);
        WICED_BT_TRACE("illegal state:%d\n", le_control_cb.conn[conn_id].state);
    }
    else if ((s_handle > e_handle) || (s_handle == 0) || (e_handle == 0))
    {
        WICED_BT_TRACE("illegal handles:%04x-%04x\n", s_handle, e_handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else
    {
        // perform characteristics search
        memset(&params, 0, sizeof(params));
        params.s_handle = s_handle;
        params.e_handle = e_handle;

        if ((rc = wiced_bt_gatt_send_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, &params)) == WICED_BT_GATT_SUCCESS)
        {
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS;
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
        }
        else
        {
            WICED_BT_TRACE("discover failed:0x%x\n", rc);
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED);
        }
    }
}

/*
 * handle start descriptors discovery command from UART
 */
void hci_control_le_handle_descriptor_discovery(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t params;
    wiced_bt_gatt_status_t          rc;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if (le_control_cb.conn[conn_id].state != LE_CONTROL_STATE_IDLE)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE);
        WICED_BT_TRACE("illegal state:%d\n", le_control_cb.conn[conn_id].state);
    }
    else if ((s_handle > e_handle) || (s_handle == 0) || (e_handle == 0))
    {
        WICED_BT_TRACE("illegal handles:%04x-%04x\n", s_handle, e_handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else
    {
        memset(&params, 0, sizeof(params));
        params.s_handle = s_handle;
        params.e_handle = e_handle;

        if ((rc = wiced_bt_gatt_send_discover(conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, &params)) == WICED_BT_GATT_SUCCESS)
        {
            // perform find info procedure to read all descriptors
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_DISCOVER_DESCRIPTORS;
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
        }
        else
        {
            WICED_BT_TRACE("discover failed:0x%x\n", rc);
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED);
        }
    }
}

void hci_control_le_handle_read_req(uint16_t conn_id, uint16_t handle)
{
    wiced_bt_gatt_read_param_t read_req;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if (le_control_cb.conn[conn_id].state != LE_CONTROL_STATE_IDLE)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE);
        WICED_BT_TRACE("illegal state:%d\n", le_control_cb.conn[conn_id].state);
    }
    else if (handle == 0)
    {
        WICED_BT_TRACE("illegal handle:%04x\n", handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else
    {
        // execute read procedure
        memset(&read_req, 0, sizeof(wiced_bt_gatt_read_param_t));

        read_req.by_handle.auth_req = GATT_AUTH_REQ_NONE;
        read_req.by_handle.handle = handle;

        if (wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_HANDLE, &read_req) == WICED_BT_GATT_SUCCESS)
        {
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_READ_VALUE;
        }
        else
        {
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED);
        }
    }
}

/*
 * Host replied the write request command processing result.
 */
void hci_control_le_handle_write_rsp(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{
    uint8_t                hci_status = HCI_CONTROL_STATUS_SUCCESS;
    wiced_bt_gatt_status_t status = p_data[0];
    
    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else 
    {
        WICED_BT_TRACE("%s: conn_id = %d, handle = %d, status = %d\n", __func__, conn_id, handle, status);
        if (wiced_bt_gatt_send_response(status, conn_id, handle, 0, 0, NULL) != WICED_BT_GATT_SUCCESS)
        {
            hci_status = HCI_CONTROL_STATUS_FAILED;
        }
    }

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, hci_status);
}


/*
 * Host replied with the data to be passed in the GATT Read Response
 */
void hci_control_le_handle_read_rsp(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{   
    uint8_t                hci_status = HCI_CONTROL_STATUS_SUCCESS;
    wiced_bt_gatt_status_t status;
    uint16_t               offset = 0;

    WICED_BT_TRACE("%s: len = %d\n", __func__, len);

    STREAM_TO_UINT8(status, p_data);
    len--;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("%s: illegal conn_id = %d\n", __func__, conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else
    {
        WICED_BT_TRACE("%s: conn_id:%d handle:%d status:%d len:%d\n", __func__, conn_id, handle, status, len);
        if (wiced_bt_gatt_send_response(status, conn_id, handle, len, offset, p_data) != WICED_BT_GATT_SUCCESS)
        {
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
        }
        else
        {
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, hci_status);
        }
    }
}

/*
 * Process write command received over UART.  Return TRUE if buffer has been
 * queued, FALSE if buffer has been shipped.
 */
wiced_bool_t hci_control_le_handle_write_cmd(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{
    wiced_bool_t           command_queued = WICED_FALSE;
    wiced_bt_gatt_status_t status;
    wiced_bt_gatt_value_t  *p_write_req;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if ((le_control_cb.conn[conn_id].conn_id != conn_id) || (le_control_cb.conn[conn_id].state != LE_CONTROL_STATE_IDLE))
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE);
        WICED_BT_TRACE("illegal state:%d\n", le_control_cb.conn[conn_id].state);
    }
    else if (handle == 0)
    {
        WICED_BT_TRACE("illegal handle:%04x\n", handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else if (hci_control_le_pending_tx_buffer.tx_buf_saved)
    {
        WICED_BT_TRACE("client sent next packet before previous is acked:%04x\n", handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_IN_PROGRESS);
    }
    else
    {
        hci_control_le_pending_tx_buffer.tx_buf_saved   = WICED_TRUE;
        hci_control_le_pending_tx_buffer.tx_buf_conn_id = conn_id;
        hci_control_le_pending_tx_buffer.tx_buf_type    = HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND;
        hci_control_le_pending_tx_buffer.tx_buf_handle  = handle;
        hci_control_le_pending_tx_buffer.tx_buf_len     = len <= sizeof(hci_control_le_pending_tx_buffer.tx_buf_data) ?
                                                      len : sizeof(hci_control_le_pending_tx_buffer.tx_buf_data);
        memcpy(hci_control_le_pending_tx_buffer.tx_buf_data, p_data, hci_control_le_pending_tx_buffer.tx_buf_len);

        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);

        if ((status = hci_control_le_send_write(conn_id, handle, p_data, len, GATT_WRITE_NO_RSP)) != WICED_BT_GATT_SUCCESS)
        {
            WICED_BT_TRACE("Failed to send:%x\n", status);
        }
        else
        {
            // Write Command sent successfully, clear tx_buf_saved flag
            hci_control_le_pending_tx_buffer.tx_buf_saved  = WICED_FALSE;

            hci_control_le_send_write_completed(conn_id, status);
        }
    }
    return (hci_control_le_pending_tx_buffer.tx_buf_saved);
}

void hci_control_le_handle_write_req(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{
    wiced_bt_gatt_status_t status;
    uint16_t               write_req_len;
    wiced_bt_gatt_value_t  *p_write_req;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if (le_control_cb.conn[conn_id].state != LE_CONTROL_STATE_IDLE)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE);
        WICED_BT_TRACE("illegal state:%d\n", le_control_cb.conn[conn_id].state);
    }
    else if (handle == 0)
    {
        WICED_BT_TRACE("illegal handle:%04x\n", handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else
    {
        // perform write procedure
        write_req_len = (sizeof(wiced_bt_gatt_value_t) - 1 + len);
        p_write_req = (wiced_bt_gatt_value_t *)wiced_bt_get_buffer(write_req_len);
        if (p_write_req == NULL)
        {
            WICED_BT_TRACE("%s: out of memory, conn_id = %d\n", __func__, conn_id);
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_OUT_OF_MEMORY);
            return;
        }

        memset(p_write_req, 0, write_req_len);
        p_write_req->handle   = handle;
        p_write_req->offset   = 0;
        p_write_req->len      = len;
        p_write_req->auth_req = GATT_AUTH_REQ_NONE;

        memcpy(p_write_req->value, p_data, len);

        // Change the state to indicate that we are waiting for write response
        le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_WRITE_VALUE;

        status = wiced_bt_gatt_send_write(conn_id, GATT_WRITE, p_write_req);

        WICED_BT_TRACE("send write: conn_id %d, len %d, status %d\n", conn_id, len, status);

        wiced_bt_free_buffer(p_write_req);

        if (status != WICED_BT_SUCCESS)
        {
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_IDLE;
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED);
        }
        else
        {
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
        }
    }
}


/*
 * Process notify command received over UART.  Return TRUE if buffer has been
 * queued, FALSE if buffer has been shipped.
 */
wiced_bool_t hci_control_le_handle_notify(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{
    wiced_bool_t           command_queued = WICED_FALSE;
    wiced_bt_gatt_status_t status;
    wiced_bt_gatt_value_t  *p_write_req;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if (le_control_cb.conn[conn_id].conn_id != conn_id)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE);
        WICED_BT_TRACE("illegal state:%d\n", le_control_cb.conn[conn_id].state);
    }
    else if (handle == 0)
    {
        WICED_BT_TRACE("illegal handle:%04x\n", handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else if (hci_control_le_pending_tx_buffer.tx_buf_saved)
    {
        WICED_BT_TRACE("client sent next packet before previous is acked:%04x\n", handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_IN_PROGRESS);
    }
    else
    {
        hci_control_le_pending_tx_buffer.tx_buf_saved   = WICED_TRUE;
        hci_control_le_pending_tx_buffer.tx_buf_conn_id = conn_id;
        hci_control_le_pending_tx_buffer.tx_buf_type    = HCI_CONTROL_GATT_COMMAND_NOTIFY;
        hci_control_le_pending_tx_buffer.tx_buf_handle  = handle;
        hci_control_le_pending_tx_buffer.tx_buf_len     = len <= sizeof(hci_control_le_pending_tx_buffer.tx_buf_data) ?
                                                      len : sizeof(hci_control_le_pending_tx_buffer.tx_buf_data);
        memcpy(hci_control_le_pending_tx_buffer.tx_buf_data, p_data, hci_control_le_pending_tx_buffer.tx_buf_len);

        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);

        if ((status = wiced_bt_gatt_send_notification(conn_id, handle, len, p_data)) != WICED_BT_GATT_SUCCESS)
        {
            WICED_BT_TRACE("Failed to send:%x\n", status);
        }
        else
        {
            // Write Command sent successfully, clear tx_buf_saved flag
            hci_control_le_pending_tx_buffer.tx_buf_saved  = WICED_FALSE;
            hci_control_le_send_write_completed(conn_id, status);
        }
    }
    return (hci_control_le_pending_tx_buffer.tx_buf_saved);
}

/*
 * Process indicate command received over UART
 */
void hci_control_le_handle_indicate(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{
    wiced_bt_gatt_status_t status;

    if (conn_id > LE_CONTROL_MAX_CONNECTIONS)
    {
        WICED_BT_TRACE("illegal conn_id:%04x\n", conn_id);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE);
    }
    else if (le_control_cb.conn[conn_id].state != LE_CONTROL_STATE_IDLE)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE);
        WICED_BT_TRACE("illegal state:%d\n", le_control_cb.conn[conn_id].state);
    }
    else if (handle == 0)
    {
        WICED_BT_TRACE("illegal handle:%04x\n", handle);
        hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else
    {
        // Change the state to indicate that we are waiting for write response
        le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_INDICATE_VALUE;

        status = wiced_bt_gatt_send_indication(conn_id, handle, len, p_data);

        WICED_BT_TRACE("send indicate: conn_id %d, len %d, status %d\n", conn_id, len, status);

        if (status != WICED_BT_SUCCESS)
        {
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_IDLE;
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED);
        }
        else
        {
            hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
        }
    }
}

/*
 * Command received over UART to remove the NVRAM stored info and initiate pair
 */
void hci_control_le_handle_delete_nvram_info_and_initiate_pair(BD_ADDR addr, uint16_t nvram_id)
{
    int                    i;
    BD_ADDR                bda;
    wiced_result_t         result;

    STREAM_TO_BDADDR(bda,addr);

    WICED_BT_TRACE("hci_control_le_handle_delete_nvram_info_and_initiate_pair %02x:%02x:%02x:%02x:%02x:%02x\n",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    hci_control_delete_nvram(nvram_id, WICED_FALSE);
//    wiced_bt_sec_delete_device_record(bda);

//    result = wiced_bt_dev_sec_bond(bda, 0, NULL);
    result = wiced_bt_dev_sec_bond(bda, 0, BT_TRANSPORT_LE, 0, NULL);
    WICED_BT_TRACE("wiced_bt_dev_sec_bond:%B, result %x\n", bda, result);

    if((result == WICED_BT_SUCCESS) || (result == WICED_BT_PENDING))
    {
        result = HCI_CONTROL_STATUS_SUCCESS;
    }
    else
    {
        result = HCI_CONTROL_STATUS_FAILED;
    }

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, result);
}

/*
 * Command received over UART to get the identity address of the remote device
 */
void hci_control_le_handle_get_identity_address (BD_ADDR bd_addr)
{
    BD_ADDR identity_addr;

    memset(identity_addr, 0, sizeof(BD_ADDR));

    if (0) //wiced_bt_get_identity_address(bd_addr, identity_addr))
    {
        WICED_BT_TRACE("Identity address is %B\n", identity_addr);
    }
    else
    {
        WICED_BT_TRACE("Identity address not found for this device %B\n", bd_addr);
    }

    wiced_transport_send_data(HCI_CONTROL_LE_EVENT_IDENTITY_ADDRESS, identity_addr, sizeof(BD_ADDR));
}

/*
 * Command received over UART to set the channel classification for the available 40 channels
 */
void hci_control_le_handle_set_channel_classification (uint8_t ble_channel_map[ HCI_BLE_CHNL_MAP_SIZE ])
{
    WICED_BT_TRACE("Setting ble channel classification [ %02x %02x %02x %02x %02x]\n",
                    ble_channel_map[0], ble_channel_map[1], ble_channel_map[2], ble_channel_map[3], ble_channel_map[4]);

    wiced_bt_ble_set_channel_classification(ble_channel_map);

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}

/*
 * Command received over UART to set the raw advertisement data
 */
void hci_control_le_handle_set_advertisement_raw_data (uint8_t* p, uint32_t data_len)
{
    WICED_BT_TRACE("%s: len = %d\n", __func__, data_len);

    hci_control_le_set_advertisement_data(p, data_len);

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}

/*
 * Command received over UART to initialize the GATT database
 */
void hci_control_le_handle_db_init (void)
{
    wiced_bt_gatt_status_t gatt_status;

    gatt_status = wiced_bt_gatt_db_init(gatt_server_db, gatt_server_db_len);

    WICED_BT_TRACE("%s: status = %d, db_len = %d\n", __func__, gatt_status, gatt_server_db_len);

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}


/*
 * Command received over UART to set the primary service for the GATT database
 */
void hci_control_le_handle_db_primary_service_add (uint8_t* p, uint32_t data_len)
{
    uint16_t        handle;
    wiced_bt_uuid_t uuid;
    uint8_t         status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t         val;

    WICED_BT_TRACE("%s: len = %d\n", __func__, data_len);

    if (data_len < 5)
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        STREAM_TO_UINT16(handle, p);
        STREAM_TO_UINT8(val, p);
        uuid.len = val;
        data_len -= 3;
        if (data_len < uuid.len)
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (uuid.len == LEN_UUID_16)
        {
            STREAM_TO_UINT16(uuid.uu.uuid16, p);
        }
        else if (uuid.len == LEN_UUID_128)
        {
            STREAM_TO_ARRAY(uuid.uu.uuid128, p, LEN_UUID_128);
        }
        else
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (wiced_bt_gatt_db_primary_service_add(handle, &uuid) == WICED_FALSE)
        {
            status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
        }
    }

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status);
}

/*
 * Command received over UART to set the secondary service for the GATT database
 */
void hci_control_le_handle_db_secondary_service_add (uint8_t* p, uint32_t data_len)
{
    uint16_t        handle;
    wiced_bt_uuid_t uuid;
    uint8_t         status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t         val;

    WICED_BT_TRACE("%s: len = %d\n", __func__, data_len);

    if (data_len < 5)
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        STREAM_TO_UINT16(handle, p);
        STREAM_TO_UINT8(val, p);
        uuid.len = val;
        data_len -= 3;
        if (data_len < uuid.len)
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (uuid.len == LEN_UUID_16)
        {
            STREAM_TO_UINT16(uuid.uu.uuid16, p);
        }
        else if (uuid.len == LEN_UUID_128)
        {
            STREAM_TO_ARRAY(uuid.uu.uuid128, p, LEN_UUID_128);
        }
        else
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (wiced_bt_gatt_db_secondary_service_add(handle, &uuid) == WICED_FALSE)
        {
            status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
        }
    }

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status);
}

/*
 * Command received over UART to set the included service for the GATT database
 */
void hci_control_le_handle_db_included_service_add (uint8_t* p, uint32_t data_len)
{
    uint16_t        handle;
    uint16_t        service_handle;
    uint16_t        end_group_handle;
    wiced_bt_uuid_t uuid;
    uint8_t         status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t         val;

    WICED_BT_TRACE("%s: len = %d\n", __func__, data_len);

    if (data_len < 9)
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        STREAM_TO_UINT16(handle, p);
        STREAM_TO_UINT16(service_handle, p);
        STREAM_TO_UINT16(end_group_handle, p);
        STREAM_TO_UINT8(val, p);
        uuid.len = val;
        data_len -= 7;
        if (data_len < uuid.len)
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (uuid.len == LEN_UUID_16)
        {
            STREAM_TO_UINT16(uuid.uu.uuid16, p);
        }
        else if (uuid.len == LEN_UUID_128)
        {
            STREAM_TO_ARRAY(uuid.uu.uuid128, p, LEN_UUID_128);
        }
        else
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (wiced_bt_gatt_db_included_service_add(handle, service_handle, end_group_handle, &uuid) == WICED_FALSE)
        {
            status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
        }
    }

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status);
}

/*
 * Command received over UART to set the characteristic for the GATT database
 */
void hci_control_le_handle_db_characteristic_add (uint8_t* p, uint32_t data_len)
{
    uint16_t        handle;
    uint16_t        handle_value;
    uint8_t         property;
    uint8_t         permission;
    wiced_bt_uuid_t uuid;
    uint8_t         status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t         val;

    WICED_BT_TRACE("%s: len = %d\n", __func__, data_len);

    if (data_len < 9)
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        STREAM_TO_UINT16(handle, p);
        STREAM_TO_UINT16(handle_value, p);
        STREAM_TO_UINT8(property, p);
        STREAM_TO_UINT8(permission, p);
        STREAM_TO_UINT8(val, p);
        uuid.len = val;
        data_len -= 7;
        if (data_len < uuid.len)
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (uuid.len == LEN_UUID_16)
        {
            STREAM_TO_UINT16(uuid.uu.uuid16, p);
        }
        else if (uuid.len == LEN_UUID_128)
        {
            STREAM_TO_ARRAY(uuid.uu.uuid128, p, LEN_UUID_128);
        }
        else
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (wiced_bt_gatt_db_characteristic_add(handle, handle_value, &uuid, property, permission) == WICED_FALSE)
        {
            status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
        }
    }

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status);
}

/*
 * Command received over UART to set the descriptor service for the GATT database
 */
void hci_control_le_handle_db_descriptor_add (uint8_t* p, uint32_t data_len)
{
    uint16_t        handle;
    uint8_t         permission;
    wiced_bt_uuid_t uuid;
    uint8_t         status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t         val;

    WICED_BT_TRACE("%s: len = %d\n", __func__, data_len);

    if (data_len < 6)
    {
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        STREAM_TO_UINT16(handle, p);
        STREAM_TO_UINT8(permission, p);
        STREAM_TO_UINT8(val, p);
        uuid.len = val;
        data_len -= 4;
        if (data_len < uuid.len)
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (uuid.len == LEN_UUID_16)
        {
            STREAM_TO_UINT16(uuid.uu.uuid16, p);
        }
        else if (uuid.len == LEN_UUID_128)
        {
            STREAM_TO_ARRAY(uuid.uu.uuid128, p, LEN_UUID_128);
        }
        else
        {
            status = HCI_CONTROL_STATUS_INVALID_ARGS;
        }
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        if (wiced_bt_gatt_db_descriptor_add(handle, &uuid, permission) == WICED_FALSE)
        {
            status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
        }
    }

    hci_control_send_command_status_evt(HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status);
}

/*
 * forward received data packet over UART
 */
void hci_control_le_process_data(uint16_t type, uint16_t conn_id, uint16_t handle, uint8_t *data, int len)
{
    int       i;
    uint8_t   tx_buf [300];
    uint8_t   *p = tx_buf;

    *p++ = conn_id & 0xff;
    *p++ = (conn_id >> 8) & 0xff;
    *p++ = handle & 0xff;
    *p++ = (handle >> 8) & 0xff;
    for (i = 0; i < len; i++)
    {
        *p++ = data[i];
    }
    wiced_transport_send_data (type, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer scan event to UART
 */
void hci_control_le_send_scan_state_event(uint8_t status)
{
    wiced_transport_send_data (HCI_CONTROL_LE_EVENT_SCAN_STATUS, &status, 1);
}

/*
 *  transfer advertisement report event to UART
 */
void hci_control_le_send_advertisement_report(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    int       i;
    uint8_t   tx_buf[70];
    uint8_t   *p = tx_buf;
    uint8_t   len;

    *p++ = p_scan_result->ble_evt_type;
    *p++ = p_scan_result->ble_addr_type;
    for (i = 0; i < 6; i++)
        *p++ = p_scan_result->remote_bd_addr[5 - i];
    *p++ = p_scan_result->rssi;

    // currently callback does not pass the data of the adv data, need to go through the data
    // zero len in the LTV means that there is no more data
    while ((len = *p_adv_data) != 0)
    {
        for (i = 0; i < len + 1; i++)
            *p++ = *p_adv_data++;
    }
    wiced_transport_send_data (HCI_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer advertise  event to uart
 */
void hci_control_le_send_advertisement_state_event(uint8_t state)
{
    wiced_transport_send_data (HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE, &state, 1);
}

/*
 *  transfer connection event to uart
 */
void hci_control_le_send_connect_event(uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role)
{
    int i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = addr_type;
    for (i = 0; i < 6; i++)
        *p++ = addr[5 - i];
    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    *p++ = role;

    wiced_transport_send_data (HCI_CONTROL_LE_EVENT_CONNECTED, tx_buf, (int)(p - tx_buf));
}


/*
 *  transfer disconnection event to UART
 */
void hci_control_le_send_disconnect_evt(uint8_t reason, uint16_t con_handle)
{
    uint8_t   tx_buf [3];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    *p++ = reason;

    wiced_transport_send_data (HCI_CONTROL_LE_EVENT_DISCONNECTED, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer discover complete event to UART
 */
void hci_control_le_send_discover_complete(uint16_t con_handle)
{
    uint8_t   tx_buf [2];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_DISCOVERY_COMPLETE, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer discovery response event to UART
 */
void hci_control_le_send_discovered_service16(uint16_t con_handle, uint16_t uuid, uint16_t s_handle, uint16_t e_handle)
{
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    *p++ = uuid & 0xff;
    *p++ = (uuid >> 8) & 0xff;
    *p++ = s_handle & 0xff;
    *p++ = (s_handle >> 8) & 0xff;
    *p++ = e_handle & 0xff;
    *p++ = (e_handle >> 8) & 0xff;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_SERVICE_DISCOVERED, tx_buf, (int)(p - tx_buf));
}

void hci_control_le_send_discovered_service128(uint16_t con_handle, uint8_t *uuid, uint16_t s_handle, uint16_t e_handle)
{
    int       i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    for (i = 0; i < 16; i++)
    {
        *p++ = uuid[15 - i];
    }
    *p++ = s_handle & 0xff;
    *p++ = (s_handle >> 8) & 0xff;
    *p++ = e_handle & 0xff;
    *p++ = (e_handle >> 8) & 0xff;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_SERVICE_DISCOVERED, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer discovery characteristic response event to UART
 */
void hci_control_le_send_discovered_characteristic16(uint16_t con_handle, uint16_t char_handle, uint8_t properties, uint16_t handle, uint16_t uuid)
{
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    *p++ = char_handle & 0xff;
    *p++ = (char_handle >> 8) & 0xff;
    *p++ = uuid & 0xff;
    *p++ = (uuid >> 8) & 0xff;
    *p++ = properties;
    *p++ = handle & 0xff;
    *p++ = (handle >> 8) & 0xff;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer discovery characteristic response event to UART
 */
void hci_control_le_send_discovered_characteristic128(uint16_t con_handle, uint16_t char_handle, uint8_t properties, uint16_t handle, uint8_t *uuid)
{
    int       i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    *p++ = char_handle & 0xff;
    *p++ = (char_handle >> 8) & 0xff;
    for (i = 0; i < 16; i++)
    {
        *p++ = uuid[15 - i];
    }
    *p++ = properties;
    *p++ = handle & 0xff;
    *p++ = (handle >> 8) & 0xff;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer discovery characteristic response event to UART
 */
void hci_control_le_send_discovered_descriptor16(uint16_t con_handle, uint16_t handle, uint16_t uuid)
{
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    *p++ = uuid & 0xff;
    *p++ = (uuid >> 8) & 0xff;
    *p++ = handle & 0xff;
    *p++ = (handle >> 8) & 0xff;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_DESCRIPTOR_DISCOVERED, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer discovery characteristic response event to UART
 */
void hci_control_le_send_discovered_descriptor128(uint16_t con_handle, uint16_t handle, uint8_t *uuid)
{
    int       i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    for (i = 0; i < 16; i++)
    {
        *p++ = uuid[15 - i];
    }
    *p++ = handle & 0xff;
    *p++ = (handle >> 8) & 0xff;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer result of the read procedure to UART
 */
void hci_control_le_send_read_rsp(uint16_t con_handle, uint8_t *data, int len)
{
    int       i;
    uint8_t   tx_buf [300];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    for (i = 0; i < len; i++)
    {
        *p++ = data[i];
    }

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_READ_RESPONSE, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer read request event to UART
 */
void hci_control_le_send_read_req(uint16_t con_handle, uint16_t handle)
{
    uint8_t   tx_buf[10];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    *p++ = handle & 0xff;
    *p++ = (handle >> 8) & 0xff;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_READ_REQUEST, tx_buf, (int)(p - tx_buf));
}

/*
 *  transfer result of the read procedure to UART
 */
void hci_control_le_send_write_completed(uint16_t con_handle, uint8_t result)
{
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = (con_handle >> 8) & 0xff;
    *p++ = result;

    wiced_transport_send_data (HCI_CONTROL_GATT_EVENT_WRITE_RESPONSE, tx_buf, (int)(p - tx_buf));
}