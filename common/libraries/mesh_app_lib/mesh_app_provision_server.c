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
 * This file contains functionality required for the device to be provisioned to be
 * a part of the mesh network.
 *
 */

#include "sparcommon.h"
#include "bt_types.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_trace.h"
#include "mesh_application.h"

#if ( defined(CYW20719B0) || defined(CYW20719B1) || defined(CYW20721B1) || defined (CYW20819A1) )
#include "wiced_memory.h"
#elif ( defined(CYW20735B0) || defined(CYW20735B1) )
#include "wiced_gki.h"
#endif

/******************************************************
 *          Structures
 ******************************************************/
typedef struct
{
    uint32_t conn_id;
    wiced_bt_mesh_provision_server_callback_t *p_parent_callback;
} mesh_app_provision_state_t;

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void            mesh_app_provision_started(uint32_t conn_id);
static void            mesh_app_provision_end(uint32_t conn_id, uint16_t addr, uint16_t net_key_idx, uint8_t result, const uint8_t *p_dev_key);
static wiced_bool_t    mesh_app_provision_get_oob(uint32_t conn_id, uint8_t type, uint8_t size, uint8_t action);
static wiced_bool_t    mesh_app_provision_get_capabilities(uint32_t conn_id);
static void            mesh_app_provision_gatt_send_cb(uint16_t conn_id, const uint8_t *packet, uint32_t packet_len);

/******************************************************
 *          Variables Definitions
 ******************************************************/
wiced_bt_mesh_provision_capabilities_data_t provisioning_config =
{
    .pub_key_type      = 0,      // If 1 Public Key OOB information available
    .static_oob_type   = 0,      // Supported static OOB Types (1 if available)
    .output_oob_size   = 0,      // Maximum size of Output OOB supported (0 - device does not support output OOB, 1-8 max size in octets supported by the device)
    .output_oob_action = 0,      // Output OOB Action field values (see @ref BT_MESH_OUT_OOB_ACT "Output OOB Action field values")
    .input_oob_size    = 0,      // Maximum size in octets of Input OOB supported
    .input_oob_action  = 0,      // Supported Input OOB Actions (see @ref BT_MESH_IN_OOB_ACT "Input OOB Action field values")
};

mesh_app_provision_state_t state =
{
    .conn_id = 0,
    .p_parent_callback = NULL,
};

/*
 * Application can call this function to provide its own private key and to register a callback 
 * to be executed when OOB information is requested by the provisioner.
 */
void wiced_bt_mesh_app_provision_server_init(uint8_t *p_priv_key, wiced_bt_mesh_provision_server_callback_t *p_callback)
{
    state.p_parent_callback = p_callback;

    // Initialize or reinitialize provisioning layer
    wiced_bt_mesh_core_provision_server_init(p_priv_key, mesh_app_provision_started, mesh_app_provision_end,
        mesh_app_provision_get_capabilities, mesh_app_provision_get_oob, mesh_app_provision_gatt_send_cb);
}

/*
 * By default application does not expose provisioning public key and does not use any of the
 * provisioning authentication methods. Hopefully a real application will call this function
 * during startup to setup provisioning capabilities.
 */
void wiced_bt_mesh_app_provision_server_configure(wiced_bt_mesh_provision_capabilities_data_t *p_config)
{
    memcpy(&provisioning_config, p_config, sizeof(wiced_bt_mesh_provision_capabilities_data_t));
}


/*
 * This callback is executed by the core library on successfull start of provisioning.
 */
void mesh_app_provision_started(uint32_t conn_id)
{
    WICED_BT_TRACE("mesh_app_provision_started: conn_id:%x\n", conn_id);
    if (state.p_parent_callback)
        (*state.p_parent_callback)(WICED_BT_MESH_PROVISION_STARTED, NULL);

}

/*
 * This callback is executed by the core library at the end of the provisioning.
 */
void mesh_app_provision_end(uint32_t conn_id, uint16_t addr, uint16_t net_key_idx, uint8_t result, const uint8_t *p_dev_key)
{
    wiced_bt_mesh_provision_status_data_t data;
    WICED_BT_TRACE("mesh_app_provision_end: conn_id:%x result:%d dev_key:\n", conn_id, result);
    WICED_BT_TRACE_ARRAY((uint8_t*)p_dev_key, 16, "");

    if (state.p_parent_callback != NULL)
    {
        data.addr        = addr;
        data.net_key_idx = net_key_idx;
        data.result      = result;
        if (p_dev_key != NULL)
        {
            memcpy(data.dev_key, p_dev_key, WICED_BT_MESH_KEY_LEN);
        }
        (*state.p_parent_callback)(WICED_BT_MESH_PROVISION_END, &data);
    }
}

/*
 * This callback is executed by the core library to get OOB data.
 */
wiced_bool_t mesh_app_provision_get_oob(uint32_t conn_id, uint8_t type, uint8_t size, uint8_t action)
{
    wiced_bt_mesh_provision_device_oob_request_data_t data;
    if (state.p_parent_callback != NULL)
    {
        data.type = type;
        data.size = size;
        data.action = action;

        (*state.p_parent_callback)(WICED_BT_MESH_PROVISION_GET_OOB_DATA, &data);
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/*
* Application can call this function with oob provisioning data.
*/
wiced_bool_t wiced_bt_mesh_provision_set_oob(wiced_bt_mesh_provision_oob_value_data_t *p_oob)
{
    return wiced_bt_mesh_core_provision_set_oob(p_oob->data, p_oob->data_size);
}


// Callback function to send provisioning packet.
void mesh_app_provision_gatt_send_cb(uint16_t conn_id, const uint8_t *packet, uint32_t packet_len)
{
    //WICED_BT_TRACE_ARRAY((char*)packet, packet_len, "");
    wiced_bt_gatt_send_notification(conn_id, HANDLE_CHAR_MESH_PROVISIONING_DATA_OUT_VALUE, packet_len, (uint8_t*)packet);
    WICED_BT_TRACE("mesh_app_provision_gatt_send_cb: conn_id:%x\n", conn_id);
}


// for provisioning app it should set its capabilities.
wiced_bool_t mesh_app_provision_get_capabilities(uint32_t conn_id)
{
    // We use 0 in all members of capabilities (except algorithm) which means no any OOB data
    wiced_bt_mesh_core_provision_capabilities_t capabilities;
    memset(&capabilities, 0, sizeof(capabilities));
    capabilities.elements_num = mesh_config.elements_num;
    UINT16TOBE2(capabilities.algorithms, WICED_BT_MESH_PROVISION_ALG_FIPS_P256_ELLIPTIC_CURVE);
    capabilities.pub_key_type = provisioning_config.pub_key_type;    // WICED_BT_MESH_PROVISION_CAPS_PUB_KEY_TYPE_AVAILABLE
    capabilities.static_oob_type = provisioning_config.static_oob_type;
    UINT16TOBE2(capabilities.output_oob_action, provisioning_config.output_oob_action);
    capabilities.output_oob_size = provisioning_config.output_oob_size;
    UINT16TOBE2(capabilities.input_oob_action, provisioning_config.input_oob_action);
    capabilities.input_oob_size = provisioning_config.input_oob_size;
    WICED_BT_TRACE("mesh_app_provision_get_capabilities: conn_id:%x\n", conn_id);
    // now we can proceed
    wiced_bt_mesh_core_provision_set_capabilities(conn_id, &capabilities);
    return WICED_TRUE;
}

