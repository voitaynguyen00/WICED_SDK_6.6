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
/* Definitions for Mesh Client RPC function codes
*/
#ifndef _MESH_CLIENTSCRIPT_H_
#define _MESH_CLIENTSCRIPT_H_

#include "bt_types.h"

/* Define events we will use while waiting
*/
typedef enum
{
    MESH_CLIENT_SCRIPT_EVT_UNPROVISIONED_DEVICE,
    MESH_CLIENT_SCRIPT_EVT_PROVISION_STATUS,
    MESH_CLIENT_SCRIPT_EVT_CONNECT_STATUS,
    MESH_CLIENT_SCRIPT_EVT_NODE_CONNECT_STATUS,
    MESH_CLIENT_SCRIPT_EVT_NODE_RESET_STATUS,
    MESH_CLIENT_SCRIPT_EVT_ON_OFF_STATUS,
    MESH_CLIENT_SCRIPT_EVT_LEVEL_STATUS,
    MESH_CLIENT_SCRIPT_EVT_LIGHTNESS_STATUS,
    MESH_CLIENT_SCRIPT_EVT_HSL_STATUS,
    MESH_CLIENT_SCRIPT_EVT_CTL_STATUS,
} tMESH_CLIENT_SCRIPT_EVENT;

#define MESH_CLIENT_SCRIPT_MAX_NAME_LENTH      128

/* Parameters returned from waiting for a unprovisioned device
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
                                                    /* TRUE if connection succeeded, FALSE otherwise    */
                                                    /* Only relevant if connection succeeded            */
    UINT8       uuid[16];
    UINT16      oob;
    UINT32      uri_hash;
    UINT32      gatt_supported;
    UINT8       name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
    UINT32      name_len;

} tMESH_CLIENT_SCRIPT_UNPROVISIONED_DEVICE;

/* Parameters returned from waiting for provision status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT32      status;
    UINT8       uuid[16];
	UINT8		name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
} tMESH_CLIENT_SCRIPT_PROVISION_STATUS;

/* Parameters returned from waiting for connect status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT32      is_connected;
    UINT32      conn_id;
    UINT32      addr;
    UINT32      is_over_gatt;

} tMESH_CLIENT_SCRIPT_CONNECT_STATUS;

/* Parameters returned from waiting for node connect status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT32      status;
    UINT8       name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
} tMESH_CLIENT_SCRIPT_NODE_CONNECT_STATUS;

/* Parameters returned from waiting for node reset status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT32      status;
    UINT8       device_name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
} tMESH_CLIENT_SCRIPT_NODE_RESET_STATUS;

/* Parameters returned from waiting for on off status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT8       device_name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
    UINT32      target;
    UINT32      present;
    UINT32      remaining_time;
} tMESH_CLIENT_SCRIPT_ON_OFF_STATUS;

/* Parameters returned from waiting for level status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT8       device_name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
    UINT32      target;
    UINT32      present;
    UINT32      remaining_time;
} tMESH_CLIENT_SCRIPT_LEVEL_STATUS;

/* Parameters returned from waiting for lightness status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT8       device_name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
    UINT32      target;
    UINT32      present;
    UINT32      remaining_time;
} tMESH_CLIENT_SCRIPT_LIGHTNESS_STATUS;

/* Parameters returned from waiting for hsl status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT8       device_name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
    UINT32      lightness;
    UINT32      hue;
    UINT32      saturation;
    UINT32      remaining_time;
} tMESH_CLIENT_SCRIPT_HSL_STATUS;

/* Parameters returned from waiting for ctl status
*/
typedef struct
{
    UINT8       wait_status;                        /* Must be the first parameter of any 'wait' return */
    UINT8       device_name[MESH_CLIENT_SCRIPT_MAX_NAME_LENTH];
    UINT32      present_lightness;
    UINT32      present_temperature;
    UINT32      target_lightness;
    UINT32      target_temperature;
    UINT32      remaining_time;
} tMESH_CLIENT_SCRIPT_CTL_STATUS;


/* Define a union of the wait structures
*/
typedef struct
{
    tMESH_CLIENT_SCRIPT_EVENT                       rcvd_event;
    union
    {
        tMESH_CLIENT_SCRIPT_UNPROVISIONED_DEVICE    unprovisioned_device;
        tMESH_CLIENT_SCRIPT_PROVISION_STATUS        provision_status;
        tMESH_CLIENT_SCRIPT_CONNECT_STATUS          connect_status;
        tMESH_CLIENT_SCRIPT_NODE_CONNECT_STATUS     node_connect_status;
        tMESH_CLIENT_SCRIPT_NODE_RESET_STATUS       node_reset_status;
        tMESH_CLIENT_SCRIPT_ON_OFF_STATUS           on_off_status;
        tMESH_CLIENT_SCRIPT_LEVEL_STATUS            level_status;
        tMESH_CLIENT_SCRIPT_LIGHTNESS_STATUS        lightness_status;
        tMESH_CLIENT_SCRIPT_HSL_STATUS              hsl_status;
        tMESH_CLIENT_SCRIPT_CTL_STATUS              ctl_status;
    } uu;
} tMESH_CLIENT_SCRIPT_EVENT_PARAMS;

/* Function prototypes
*/
#ifdef __cplusplus
extern "C" {
#endif
    extern void mesh_client_enqueue_and_check_event(tMESH_CLIENT_SCRIPT_EVENT event, void *p_params, UINT16 size_of_params);
    extern void mesh_client_network_open_UI_Ex();
    extern int mesh_client_network_create_UI_Ex(const char *provisioner_name, const char *p_provisioner_uuid, char *mesh_name);
    extern int mesh_client_set_device_config_UI_Ex(const char *device_name, int is_gatt_proxy, int is_friend, int is_relay, int beacon,
        int relay_xmit_count, int relay_xmit_interval, int default_ttl, int net_xmit_count, int net_xmit_interval);
    extern int mesh_client_set_publication_config_UI_Ex(const char *device_name, int device_type, int publish_credential_flag,
        int publish_retransmit_count, int publish_retransmit_interval, int publish_ttl);
    extern int mesh_client_scan_unprovisioned_UI_Ex(int start);
#ifdef __cplusplus
};
#endif
#endif
