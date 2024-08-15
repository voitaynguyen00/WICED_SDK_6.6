
#include "script_app.h"
#include "wiced_bt_trace.h"
#include "rpc.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <mesh_client.pb.h>
#include "wiced_mesh_client.h"

#define MESH_AUTOMATION_ENABLED TRUE

static BOOL32 mesh_client_rpc_process_network_exists(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_network_exists request = PROTOBUF_mesh_client_network_exists_init_default;
    request.mesh_name.funcs.decode = &read_bytes;
    request.mesh_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_network_exists_fields, &request))
        response.res = mesh_client_network_exists((char *)request.mesh_name.arg);
    return  SendUint32Response(parm, &response);
}

static BOOL32 mesh_client_rpc_process_network_create(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_network_create request = PROTOBUF_mesh_client_network_create_init_default;
    request.provisioner_name.funcs.decode = &read_bytes;
    request.provisioner_name.arg = get_protobuf_buffer(TRUE);
    request.provisioner_uuid.funcs.decode = &read_bytes;
    request.provisioner_uuid.arg = (char*)request.provisioner_name.arg + 256;
    request.mesh_name.funcs.decode = &read_bytes;
    request.mesh_name.arg = (char*)request.provisioner_uuid.arg + 256;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_network_create_fields, &request))
    {
        #if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
        mesh_client_network_create_UI_Ex((const char *)request.provisioner_name.arg,
            (const char *)request.provisioner_uuid.arg,
            (char *)request.mesh_name.arg);
        #endif

        response.res = mesh_client_network_create((const char *)request.provisioner_name.arg,
            (const char *)request.provisioner_uuid.arg,
            (char *)request.mesh_name.arg);
    }
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_network_delete(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_network_delete request = PROTOBUF_mesh_client_network_delete_init_default;
    request.provisioner_name.funcs.decode = &read_bytes;
    request.provisioner_name.arg = get_protobuf_buffer(TRUE);
    request.provisioner_uuid.funcs.decode = &read_bytes;
    request.provisioner_uuid.arg = (char*)request.provisioner_name.arg + 256;
    request.mesh_name.funcs.decode = &read_bytes;
    request.mesh_name.arg = (char*)request.provisioner_uuid.arg + 256;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_network_delete_fields, &request))
        response.res = mesh_client_network_delete((const char *)request.provisioner_name.arg,
                                                    (const char *)request.provisioner_uuid.arg,
                                                    (char *)request.mesh_name.arg);
    return  SendUint32Response(parm, &response);
}

void network_opened(uint8_t status)
{
	Log(L"Network opened");
}


static BOOL32 mesh_client_rpc_process_network_open(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_network_open request = PROTOBUF_mesh_client_network_open_init_default;
    request.provisioner_name.funcs.decode = &read_bytes;
    request.provisioner_name.arg = get_protobuf_buffer(TRUE);
    request.provisioner_uuid.funcs.decode = &read_bytes;
    request.provisioner_uuid.arg = (char*)request.provisioner_name.arg + 256;
    request.mesh_name.funcs.decode = &read_bytes;
    request.mesh_name.arg = (char*)request.provisioner_uuid.arg + 256;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_network_open_fields, &request))
    {
        response.res = mesh_client_network_open((const char *)request.provisioner_name.arg,
            (const char *)request.provisioner_uuid.arg,
            (char *)request.mesh_name.arg,
			network_opened);
    }

    return  SendUint32Response(parm, &response);
}

static BOOL32 mesh_client_rpc_process_network_import(PROTOBUF_PARAM* parm)
{
    char* res = NULL;
    PROTOBUF_mesh_client_network_import request = PROTOBUF_mesh_client_network_import_init_default;
    request.provisioner_name.funcs.decode = &read_bytes;
    request.provisioner_name.arg = get_protobuf_buffer(TRUE);
    request.provisioner_uuid.funcs.decode = &read_bytes;
    request.provisioner_uuid.arg = (char*)request.provisioner_name.arg + 256;
    request.json_string.funcs.decode = &read_bytes;
    request.json_string.arg = (char*)request.provisioner_uuid.arg + 256;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_network_import_fields, &request))
        res = mesh_client_network_import((const char *)request.provisioner_name.arg,
                                                    (const char *)request.provisioner_uuid.arg,
                                                    (char *)request.json_string.arg,
													network_opened);
    return  SendStringResponse(parm, res);
}
static BOOL32 mesh_client_rpc_process_network_export(PROTOBUF_PARAM* parm)
{
    char* res = NULL;
    PROTOBUF_mesh_client_network_export request = PROTOBUF_mesh_client_network_export_init_default;
    request.mesh_name.funcs.decode = &read_bytes;
    request.mesh_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_network_export_fields, &request))
        res = mesh_client_network_export((char *)request.mesh_name.arg);
    return SendStringResponse(parm, res);
}
void mesh_client_reset_callback_EvtList();

static BOOL32 mesh_client_rpc_process_network_close(PROTOBUF_PARAM* parm)
{
    mesh_client_network_close();
    mesh_client_reset_callback_EvtList();
    return  SendVoidResponse(parm);
}
static BOOL32 mesh_client_rpc_process_group_create(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_group_create request = PROTOBUF_mesh_client_group_create_init_default;
    request.group_name.funcs.decode = &read_bytes;
    request.group_name.arg = get_protobuf_buffer(TRUE);
    request.parent_group_name.funcs.decode = &read_bytes;
    request.parent_group_name.arg = (char*)request.group_name.arg + 256;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_group_create_fields, &request))
        response.res = mesh_client_group_create((char *)request.group_name.arg,
                                                (char *)request.parent_group_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_group_delete(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_group_delete request = PROTOBUF_mesh_client_group_delete_init_default;
    request.group_name.funcs.decode = &read_bytes;
    request.group_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_group_delete_fields, &request))
        response.res = mesh_client_group_delete((char *)request.group_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_get_all_networks(PROTOBUF_PARAM* parm)
{
    char* res = mesh_client_get_all_networks();
    return  SendStringResponse(parm, res);
}
static BOOL32 mesh_client_rpc_process_get_all_groups(PROTOBUF_PARAM* parm)
{
    char* res = NULL;
    PROTOBUF_mesh_client_get_all_groups request = PROTOBUF_mesh_client_get_all_groups_init_default;
    request.in_group.funcs.decode = &read_bytes;
    request.in_group.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_group_delete_fields, &request))
        res = mesh_client_get_all_groups((char *)request.in_group.arg);
    return  SendStringResponse(parm, res);
}
static BOOL32 mesh_client_rpc_process_get_all_provisioners(PROTOBUF_PARAM* parm)
{
    char* res = mesh_client_get_all_provisioners();
    return  SendStringResponse(parm, res);
}
static BOOL32 mesh_client_rpc_process_get_device_components(PROTOBUF_PARAM* parm)
{
    char* res = NULL;
    PROTOBUF_mesh_client_get_device_components request = PROTOBUF_mesh_client_get_device_components_init_default;
    request.uuid.funcs.decode = &read_bytes;
    request.uuid.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_get_device_components_fields, &request))
        res = mesh_client_get_device_components((uint8_t *)request.uuid.arg);
    return  SendStringResponse(parm, res);
}
static BOOL32 mesh_client_rpc_process_get_target_methods(PROTOBUF_PARAM* parm)
{
    char* res = NULL;
    PROTOBUF_mesh_client_get_target_methods request = PROTOBUF_mesh_client_get_target_methods_init_default;
    request.component_name.funcs.decode = &read_bytes;
    request.component_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_get_target_methods_fields, &request))
        res = mesh_client_get_target_methods((uint8_t *)request.component_name.arg);
    return  SendStringResponse(parm, res);
}
static BOOL32 mesh_client_rpc_process_get_control_methods(PROTOBUF_PARAM* parm)
{
    char* res = NULL;
    PROTOBUF_mesh_client_get_control_methods request = PROTOBUF_mesh_client_get_control_methods_init_default;
    request.component_name.funcs.decode = &read_bytes;
    request.component_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_get_control_methods_fields, &request))
        res = mesh_client_get_control_methods((uint8_t *)request.component_name.arg);
    return  SendStringResponse(parm, res);
}
static BOOL32 mesh_client_rpc_process_get_group_components(PROTOBUF_PARAM* parm)
{
    char* res = NULL;
    PROTOBUF_mesh_client_get_group_components request = PROTOBUF_mesh_client_get_group_components_init_default;
    request.p_group_name.funcs.decode = &read_bytes;
    request.p_group_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_get_group_components_fields, &request))
        res = mesh_client_get_group_components((uint8_t *)request.p_group_name.arg);
    return  SendStringResponse(parm, res);
}
static BOOL32 mesh_client_rpc_process_get_component_type(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_get_component_type request = PROTOBUF_mesh_client_get_component_type_init_default;
    request.component_name.funcs.decode = &read_bytes;
    request.component_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_get_component_type_fields, &request))
        response.res = mesh_client_get_component_type((uint8_t *)request.component_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_rename(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_rename request = PROTOBUF_mesh_client_rename_init_default;
    request.old_name.funcs.decode = &read_bytes;
    request.old_name.arg = get_protobuf_buffer(TRUE);
    request.new_name.funcs.decode = &read_bytes;
    request.new_name.arg = (char*)request.old_name.arg + 256;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_rename_fields, &request))
        response.res = mesh_client_rename((char *)request.old_name.arg, (char *)request.new_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_move_component_to_group(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_move_component_to_group request = PROTOBUF_mesh_client_move_component_to_group_init_default;
    request.component_name.funcs.decode = &read_bytes;
    request.component_name.arg = get_protobuf_buffer(TRUE);
    request.from_group_name.funcs.decode = &read_bytes;
    request.from_group_name.arg = (char*)request.component_name.arg + 256;
	request.to_group_name.funcs.decode = &read_bytes;
	request.to_group_name.arg = (char*)request.from_group_name.arg + 256;
	if (pb_decode(parm->stream, PROTOBUF_mesh_client_move_component_to_group_fields, &request))
        response.res = mesh_client_move_component_to_group((char *)request.component_name.arg, (char *)request.from_group_name.arg, (char *)request.to_group_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_configure_publication(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_configure_publication request = PROTOBUF_mesh_client_configure_publication_init_default;
    request.component_name.funcs.decode = &read_bytes;
    request.component_name.arg = get_protobuf_buffer(TRUE);
    request.method.funcs.decode = &read_bytes;
    request.method.arg = (char*)request.component_name.arg + 256;
    request.target_name.funcs.decode = &read_bytes;
    request.target_name.arg = (char*)request.method.arg + 256;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_configure_publication_fields, &request))
    	response.res = mesh_client_configure_publication((const char *)request.component_name.arg,
                                                            (uint8_t)request.is_command,
                                                            (const char *)request.method.arg,
                                                            (const char *)request.target_name.arg,
                                                            request.publish_period);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_set_device_config(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_set_device_config request = PROTOBUF_mesh_client_set_device_config_init_default;
    request.device_name.funcs.decode = &read_bytes;
    const char* p = (const char*)get_protobuf_buffer(TRUE);
    request.device_name.arg = p;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_set_device_config_fields, &request))
    {
        if (*p == '\0')
        {
            p = NULL;
        }
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
        mesh_client_set_device_config_UI_Ex(p,
            (int)request.is_gatt_proxy,
            (int)request.is_friend,
            (int)request.is_relay,
            (int)request.send_net_beacon,
            (int)request.relay_xmit_count,
            (int)request.relay_xmit_interval,
            (int)request.default_ttl,
            (int)request.net_xmit_count,
            (int)request.net_xmit_interval);
#endif

        response.res = mesh_client_set_device_config(p,
            (int)request.is_gatt_proxy,
            (int)request.is_friend,
            (int)request.is_relay,
            (int)request.send_net_beacon,
            (int)request.relay_xmit_count,
            (int)request.relay_xmit_interval,
            (int)request.default_ttl,
            (int)request.net_xmit_count,
            (int)request.net_xmit_interval);
    }
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_set_publication_config(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_set_publication_config request = PROTOBUF_mesh_client_set_publication_config_init_default;
    request.device_name.funcs.decode = &read_bytes;
    const char* p = (const char*)get_protobuf_buffer(TRUE);
    request.device_name.arg = p;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_set_publication_config_fields, &request))
    {
        if (*p == '\0')
        {
            p = NULL;
        }
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
        mesh_client_set_publication_config_UI_Ex(p,
            (int)request.device_type,
            (int)request.publish_credential_flag,
            (int)request.publish_retransmit_count,
            (int)request.publish_retransmit_interval,
            (int)request.publish_ttl);
#endif

        response.res = mesh_client_set_publication_config(p,
            (int)request.device_type,
            (int)request.publish_credential_flag,
            (int)request.publish_retransmit_count,
            (int)request.publish_retransmit_interval,
            (int)request.publish_ttl);
    }
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_scan_unprovisioned(PROTOBUF_PARAM* parm)
{
	RPC_Uint32Response response = RPC_Uint32Response_init_default;
	PROTOBUF_mesh_client_scan_unprovisioned request = PROTOBUF_mesh_client_scan_unprovisioned_init_default;
	if (pb_decode(parm->stream, PROTOBUF_mesh_client_scan_unprovisioned_fields, &request))
	{
		response.res = mesh_client_scan_unprovisioned((int)request.start);
#if defined( MESH_AUTOMATION_ENABLED ) && (MESH_AUTOMATION_ENABLED == TRUE)
		mesh_client_scan_unprovisioned_UI_Ex((int)request.start);
#endif
	}
	return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_provision(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_provision request = PROTOBUF_mesh_client_provision_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    request.group_name.funcs.decode = &read_bytes;
    request.group_name.arg = (char*)request.device_name.arg + 256;
    request.uuid.funcs.decode = &read_bytes;
    request.uuid.arg = (char*)request.group_name.arg + 256;
	if (pb_decode(parm->stream, PROTOBUF_mesh_client_provision_fields, &request))
		response.res = mesh_client_provision((const char *)request.device_name.arg,
		(const char *)request.group_name.arg,
			(uint8_t *)request.uuid.arg,
			(uint8_t)request.identify_duration); 
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_reset_device(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_reset_device request = PROTOBUF_mesh_client_reset_device_init_default;
    request.component_name.funcs.decode = &read_bytes;
    request.component_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_reset_device_fields, &request))
        response.res = mesh_client_reset_device((char *)request.component_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_connect_network(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_connect_network request = PROTOBUF_mesh_client_connect_network_init_default;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_connect_network_fields, &request))
        response.res = mesh_client_connect_network((uint8_t)request. use_gatt_proxy, (uint8_t)request.scan_duration);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_disconnect_network(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_disconnect_network request = PROTOBUF_mesh_client_disconnect_network_init_default;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_disconnect_network_fields, &request))
        response.res = mesh_client_disconnect_network();
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_connect_component(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_connect_component request = PROTOBUF_mesh_client_connect_component_init_default;
    request.component_name.funcs.decode = &read_bytes;
    request.component_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_connect_component_fields, &request))
        response.res = mesh_client_connect_component((char *)request.component_name.arg, (uint8_t)request.use_proxy, (uint8_t)request.scan_duration);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_connection_state_changed(PROTOBUF_PARAM* parm)
{
    PROTOBUF_mesh_client_connection_state_changed request = PROTOBUF_mesh_client_connection_state_changed_init_default;
    if(pb_decode(parm->stream, PROTOBUF_mesh_client_connection_state_changed_fields, &request))
        mesh_client_connection_state_changed((uint16_t)request.conn_id, (uint16_t)request.mtu);
    return  SendVoidResponse(parm);
}
static BOOL32 mesh_client_rpc_process_is_connecting_provisioning(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    response.res = mesh_client_is_connecting_provisioning();
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_identify(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_identify request = PROTOBUF_mesh_client_identify_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_identify_fields, &request))
        response.res = mesh_client_identify((const char *)request.device_name.arg, (uint8_t)request.duration);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_on_off_get(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_on_off_get request = PROTOBUF_mesh_client_on_off_get_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_on_off_get_fields, &request))
        response.res = mesh_client_on_off_get((const char *)request.device_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_on_off_set(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_on_off_set request = PROTOBUF_mesh_client_on_off_set_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_on_off_set_fields, &request))
        response.res = mesh_client_on_off_set((const char *)request.device_name.arg,
                                                (uint8_t)request.on_off,
												(wiced_bool_t)request.reliable,
												(uint32_t)request.transition_time,
                                                (uint16_t)request.delay);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_level_get(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_level_get request = PROTOBUF_mesh_client_level_get_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_level_get_fields, &request))
        response.res = mesh_client_level_get((const char *)request.device_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_level_set(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_level_set request = PROTOBUF_mesh_client_level_set_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_level_set_fields, &request))
        response.res = mesh_client_level_set((const char *)request.device_name.arg,
                                                (int16_t)request.level,
                                                (wiced_bool_t)request.interim,
                                                (uint32_t)request.transition_time,
                                                (uint16_t)request.delay);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_lightness_get(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_lightness_get request = PROTOBUF_mesh_client_lightness_get_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_lightness_get_fields, &request))
        response.res = mesh_client_lightness_get((const char *)request.device_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_lightness_set(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_lightness_set request = PROTOBUF_mesh_client_lightness_set_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_lightness_set_fields, &request))
        response.res = mesh_client_lightness_set((const char *)request.device_name.arg,
                                                    (int16_t)request.lightness,
                                                    (wiced_bool_t)request.interim,
                                                    (uint32_t)request.transition_time,
                                                    (uint16_t)request.delay);

    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_hsl_get(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_hsl_get request = PROTOBUF_mesh_client_hsl_get_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_hsl_get_fields, &request))
        response.res = mesh_client_hsl_get((const char *)request.device_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_hsl_set(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_hsl_set request = PROTOBUF_mesh_client_hsl_set_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_hsl_set_fields, &request))
        response.res = mesh_client_hsl_set((const char *)request.device_name.arg,
                                            (int16_t)request.lightness,
                                            (int16_t)request.hue,
                                            (int16_t)request.saturation,
                                            (wiced_bool_t)request.interim,
                                            (uint32_t)request.transition_time,
                                            (uint16_t)request.delay);

    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_ctl_get(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_ctl_get request = PROTOBUF_mesh_client_ctl_get_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_ctl_get_fields, &request))
        response.res = mesh_client_ctl_get((const char *)request.device_name.arg);
    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_ctl_set(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_ctl_set request = PROTOBUF_mesh_client_ctl_set_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_ctl_set_fields, &request))
        response.res = mesh_client_ctl_set((const char *)request.device_name.arg,
                                            (int16_t)request.lightness,
                                            (int16_t)request.temperature,
                                            (int16_t)request.delta_uv,
                                            (wiced_bool_t)request.interim,
                                            (uint32_t)request.transition_time,
                                            (uint16_t)request.delay);

    return  SendUint32Response(parm, &response);
}
static BOOL32 mesh_client_rpc_process_vendor_data_set(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
    PROTOBUF_mesh_client_vendor_data_set request = PROTOBUF_mesh_client_vendor_data_set_init_default;
    request.device_name.funcs.decode = &read_bytes;
    request.device_name.arg = get_protobuf_buffer(TRUE);
    request.buffer.funcs.decode = &read_bytes;
    request.buffer.arg = (char*)request.device_name.arg + 256;
    if (pb_decode(parm->stream, PROTOBUF_mesh_client_vendor_data_set_fields, &request))
        response.res = mesh_client_vendor_data_set((const char *)request.device_name.arg,
        (uint8_t*)request.buffer.arg, (uint16_t) request.len);
    return  SendUint32Response(parm, &response);
}

extern void _wait_mesh_client_event(UINT32 timeout, tMESH_CLIENT_SCRIPT_EVENT wait_event);

BOOL32 mesh_rpc_process_wait_event(PROTOBUF_PARAM* parm)
{
    BOOL32 status;
    PROTOBUF_mesh_client_wait_event req = PROTOBUF_mesh_client_wait_event_init_default;
    status = pb_decode(parm->stream, PROTOBUF_mesh_client_wait_event_fields, &req);
    if (status)
        _wait_mesh_client_event((UINT32)req.timeout, (tMESH_CLIENT_SCRIPT_EVENT)req.wait_event);
    *(parm->ret_size) = 0;
    return true;
}

static BOOL32 mesh_client_rpc_process_is_proxy_connected(PROTOBUF_PARAM* parm)
{
    RPC_Uint32Response response = RPC_Uint32Response_init_default;
//    response.res = mesh_client_is_proxy_connected();
    return  SendUint32Response(parm, &response);
}

tPROTOBUF_RPC_FUNC_PROXY *rpc_mesh_client_proxies[] =
{
    mesh_client_rpc_process_network_exists,
    mesh_client_rpc_process_network_create,
    mesh_client_rpc_process_network_delete,
    mesh_client_rpc_process_network_open,
    mesh_client_rpc_process_network_import,
    mesh_client_rpc_process_network_export,
    mesh_client_rpc_process_network_close,
    mesh_client_rpc_process_group_create,
    mesh_client_rpc_process_group_delete,
    mesh_client_rpc_process_get_all_networks,
    mesh_client_rpc_process_get_all_groups,
    mesh_client_rpc_process_get_all_provisioners,
    mesh_client_rpc_process_get_device_components,
    mesh_client_rpc_process_get_target_methods,
    mesh_client_rpc_process_get_control_methods,
    mesh_client_rpc_process_get_group_components,
    mesh_client_rpc_process_get_component_type,
    mesh_client_rpc_process_rename,
    mesh_client_rpc_process_move_component_to_group,
    mesh_client_rpc_process_configure_publication,
    mesh_client_rpc_process_set_device_config,
    mesh_client_rpc_process_set_publication_config,
    mesh_client_rpc_process_scan_unprovisioned,
    mesh_client_rpc_process_provision,
    mesh_client_rpc_process_reset_device,
    mesh_client_rpc_process_connect_network,
    mesh_client_rpc_process_disconnect_network,
    mesh_client_rpc_process_connect_component,
    mesh_client_rpc_process_connection_state_changed,
    mesh_client_rpc_process_is_connecting_provisioning,
    mesh_client_rpc_process_identify,
    mesh_client_rpc_process_on_off_get,
    mesh_client_rpc_process_on_off_set,
    mesh_client_rpc_process_level_get,
    mesh_client_rpc_process_level_set,
    mesh_client_rpc_process_lightness_get,
    mesh_client_rpc_process_lightness_set,
    mesh_client_rpc_process_hsl_get,
    mesh_client_rpc_process_hsl_set,
    mesh_client_rpc_process_ctl_get,
    mesh_client_rpc_process_ctl_set,
    mesh_client_rpc_process_vendor_data_set,
    mesh_rpc_process_wait_event,
    mesh_client_rpc_process_is_proxy_connected,
};


BOOLEAN mesh_client_rpc_dispatch(RPC_HEADER *header, pb_istream_t *stream, void* pRet, USHORT* ret_size)
{
    PROTOBUF_PARAM parm;
    parm.header = header;
    parm.stream = stream;
    parm.pRet = pRet;
    parm.ret_size = ret_size;
    bool status = false;
    if (header->function_code.mesh_client_function_code < sizeof(rpc_mesh_client_proxies) / sizeof(rpc_mesh_client_proxies[0]))
        status = (*rpc_mesh_client_proxies[header->function_code.mesh_client_function_code])(&parm);
    return status;
}

/*
* callbacks
*/
static void mesh_client_unprovisioned_device_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, uint8_t *uuid, uint16_t oob, uint32_t uri_hash, uint8_t gatt_supported, uint8_t *name, uint8_t name_len)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_unprovisioned_device_tag;
    param.event_params.unprovisioned_device.uuid.funcs.encode = &write_uuid_128;
    param.event_params.unprovisioned_device.uuid.arg = uuid;
    param.event_params.unprovisioned_device.oob = oob;
    param.event_params.unprovisioned_device.uri_hash = uri_hash;
    param.event_params.unprovisioned_device.gatt_supported = gatt_supported;
    param.event_params.unprovisioned_device.name.funcs.encode = &write_string;
    param.event_params.unprovisioned_device.name.arg = name;

    BOOLEAN status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_provision_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, uint8_t status, uint8_t *p_uuid, uint8_t *p_name)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_provision_status_tag;

    param.event_params.provision_status.status = status;
    param.event_params.provision_status.uuid.funcs.encode = &write_uuid_128;
    param.event_params.provision_status.uuid.arg = p_uuid;

    param.event_params.provision_status.name.funcs.encode = &write_string;
    param.event_params.provision_status.name.arg = p_name;
    BOOLEAN send_status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_connect_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_connect_status_tag;

    param.event_params.connect_status.is_connected = is_connected;
    param.event_params.connect_status.conn_id = conn_id;
    param.event_params.connect_status.addr = addr;
    param.event_params.connect_status.is_over_gatt = is_over_gatt;

    BOOLEAN status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_node_connect_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, uint8_t status, char *p_name)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_node_connect_status_tag;

    param.event_params.node_connect_status.status = status;
    param.event_params.node_connect_status.device_name.funcs.encode = &write_string;
    param.event_params.node_connect_status.device_name.arg = p_name;

    BOOLEAN send_status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_node_reset_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, uint8_t status, char *device_name)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_node_reset_status_tag;

    param.event_params.node_reset_status.status = status;
    param.event_params.node_reset_status.device_name.funcs.encode = &write_string;
    param.event_params.node_reset_status.device_name.arg = device_name;

    BOOLEAN send_status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_on_off_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, const char *device_name, uint8_t target, uint8_t present, uint32_t remaining_time)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_on_off_status_tag;

    param.event_params.on_off_status.device_name.funcs.encode = &write_string;
    param.event_params.on_off_status.device_name.arg = (void*)device_name;
    param.event_params.on_off_status.target = target;
    param.event_params.on_off_status.present = present;
    param.event_params.on_off_status.remaining_time = remaining_time;

    BOOLEAN status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_level_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, const char *device_name, uint16_t target, uint16_t present, uint32_t remaining_time)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_level_status_tag;

    param.event_params.level_status.device_name.funcs.encode = &write_string;
    param.event_params.level_status.device_name.arg = (void*)device_name;
    param.event_params.level_status.target = target;
    param.event_params.level_status.present = present;
    param.event_params.level_status.remaining_time = remaining_time;

    BOOLEAN status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_lightness_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, const char *device_name, uint16_t target, uint16_t present, uint32_t remaining_time)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_lightness_status_tag;


    param.event_params.lightness_status.device_name.funcs.encode = &write_string;
    param.event_params.lightness_status.device_name.arg = (void*)device_name;
    param.event_params.lightness_status.target = target;
    param.event_params.lightness_status.present = present;
    param.event_params.lightness_status.remaining_time = remaining_time;

    BOOLEAN status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_hsl_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, const char *device_name, uint16_t lightness, uint16_t hue, uint16_t saturation, uint32_t remaining_time)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_hsl_status_tag;


    param.event_params.hsl_status.device_name.funcs.encode = &write_string;
    param.event_params.hsl_status.device_name.arg = (void*)device_name;
    param.event_params.hsl_status.lightness = lightness;
    param.event_params.hsl_status.hue = hue;
    param.event_params.hsl_status.saturation = saturation;
    param.event_params.hsl_status.remaining_time = remaining_time;

    BOOLEAN status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

static void mesh_client_ctl_status_rpc_callback(tMESH_CLIENT_SCRIPT_EVENT rcvd_event, const char *device_name, uint16_t present_lightness, uint16_t present_temperature, uint16_t target_lightness, uint16_t target_temperature, uint32_t remaining_time)
{
    PROTOBUF_mesh_client_EVENT_PARAMS param = PROTOBUF_mesh_client_EVENT_PARAMS_init_default;
    param.rcvd_event = rcvd_event;
    param.which_event_params = PROTOBUF_mesh_client_EVENT_PARAMS_ctl_status_tag;


    param.event_params.ctl_status.device_name.funcs.encode = &write_string;
    param.event_params.ctl_status.device_name.arg = (void*)device_name;
    param.event_params.ctl_status.present_lightness = present_lightness;
    param.event_params.ctl_status.present_temperature = present_temperature;
    param.event_params.ctl_status.target_lightness = target_lightness;
    param.event_params.ctl_status.target_temperature = target_temperature;
    param.event_params.ctl_status.remaining_time = remaining_time;

    BOOLEAN status = EncodeAndSendHeaderEventData(NULL, PROTOBUF_mesh_client_EVENT_PARAMS_fields, &param);
}

void mesh_client_rpc_return_event(tMESH_CLIENT_SCRIPT_EVENT_PARAMS *p_evt)
{
    switch (p_evt->rcvd_event)
    {
    case MESH_CLIENT_SCRIPT_EVT_UNPROVISIONED_DEVICE:
        mesh_client_unprovisioned_device_rpc_callback(p_evt->rcvd_event, 
                                                        (uint8_t*)&p_evt->uu.unprovisioned_device.uuid,
                                                        p_evt->uu.unprovisioned_device.oob, 
                                                        p_evt->uu.unprovisioned_device.uri_hash, 
                                                        p_evt->uu.unprovisioned_device.gatt_supported,
                                                        (uint8_t*)&p_evt->uu.unprovisioned_device.name, 
                                                        p_evt->uu.unprovisioned_device.name_len);
    break;

    case MESH_CLIENT_SCRIPT_EVT_PROVISION_STATUS:
            mesh_client_provision_status_rpc_callback(p_evt->rcvd_event, 
                                                        p_evt->uu.provision_status.status, 
                                                        (uint8_t*)&p_evt->uu.provision_status.uuid, 
                                                        (uint8_t*)&p_evt->uu.provision_status.name);
    break;

    case MESH_CLIENT_SCRIPT_EVT_CONNECT_STATUS:
        mesh_client_connect_status_rpc_callback(p_evt->rcvd_event, 
                                                p_evt->uu.connect_status.is_connected,
                                                p_evt->uu.connect_status.conn_id, 
                                                p_evt->uu.connect_status.addr, 
                                                p_evt->uu.connect_status.is_over_gatt);
    break;

    case MESH_CLIENT_SCRIPT_EVT_NODE_CONNECT_STATUS:
        mesh_client_node_connect_status_rpc_callback(p_evt->rcvd_event, 
                                                        p_evt->uu.node_connect_status.status, 
                                                        (char*)&p_evt->uu.node_connect_status.name);
    break;

    case MESH_CLIENT_SCRIPT_EVT_NODE_RESET_STATUS:
        mesh_client_node_reset_status_rpc_callback(p_evt->rcvd_event, 
                                                    p_evt->uu.node_reset_status.status, 
                                                    (char*)&p_evt->uu.node_reset_status.device_name);
    break;

    case MESH_CLIENT_SCRIPT_EVT_ON_OFF_STATUS:
        mesh_client_on_off_status_rpc_callback(p_evt->rcvd_event, 
                                                (char*)&p_evt->uu.on_off_status.device_name, 
                                                p_evt->uu.on_off_status.target, 
                                                p_evt->uu.on_off_status.present, 
                                                p_evt->uu.on_off_status.remaining_time);
    break;

    case MESH_CLIENT_SCRIPT_EVT_LEVEL_STATUS:
        mesh_client_level_status_rpc_callback(p_evt->rcvd_event, 
                                                (char*)&p_evt->uu.level_status.device_name, 
                                                p_evt->uu.level_status.target, 
                                                p_evt->uu.level_status.present, 
                                                p_evt->uu.level_status.remaining_time);
    break;

    case MESH_CLIENT_SCRIPT_EVT_LIGHTNESS_STATUS:
        mesh_client_lightness_status_rpc_callback(p_evt->rcvd_event, 
                                                    (char*)&p_evt->uu.lightness_status.device_name, 
                                                    p_evt->uu.lightness_status.target, 
                                                    p_evt->uu.lightness_status.present, 
                                                    p_evt->uu.lightness_status.remaining_time);
    break;

    case MESH_CLIENT_SCRIPT_EVT_HSL_STATUS:
        mesh_client_hsl_status_rpc_callback(p_evt->rcvd_event, 
                                            (char*)&p_evt->uu.hsl_status.device_name, 
                                            p_evt->uu.hsl_status.lightness, 
                                            p_evt->uu.hsl_status.hue, 
                                            p_evt->uu.hsl_status.saturation, 
                                            p_evt->uu.hsl_status.remaining_time);
    break;

    case MESH_CLIENT_SCRIPT_EVT_CTL_STATUS:
        mesh_client_ctl_status_rpc_callback(p_evt->rcvd_event, 
                                            (char*)&p_evt->uu.ctl_status.device_name, 
                                            p_evt->uu.ctl_status.present_lightness, 
                                            p_evt->uu.ctl_status.present_temperature,
                                            p_evt->uu.ctl_status.target_lightness, 
                                            p_evt->uu.ctl_status.target_temperature, 
                                            p_evt->uu.ctl_status.remaining_time);
    break;

    default:
        return;
        break;
    }

}
