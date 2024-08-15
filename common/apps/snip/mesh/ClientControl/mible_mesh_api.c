#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include <memory.h>
#include "wiced_bt_mesh_model_defs.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_provision.h"
#include "mible_mesh_api.h"
#include "mible_type.h"

/* default ttl set params */
typedef struct 
{
    uint16_t dst_addr;
    uint8_t  ttl;
} mible_mesh_default_ttl_t;

typedef struct
{
    uint32_t iv_index;          /**< IV Index */
    uint8_t  iv_update;         /**< IV-UPDATE flag. Can be 0 or 1 */
} mesh_client_iv_t;

typedef struct
{
    uint8_t  previous_iv_idx;       /**< 0 - it is SEQ for current IV INDEX. 1 - for previous. If addr is 0 then it is ignored. */
    uint8_t  seq[3];                /**< Little Endian Sequence Number (SEQ). */
    uint16_t addr;
} mesh_client_seq_t;

#define MIBLE_KEY_REFRESH_FLAG              0x01
#define MIBLE_KEY_IV_UPDATE_FLAG            0x02
#define FND_FEATURE_BIT_RELAY               0x0001
#define FND_FEATURE_BIT_PROXY               0x0002
#define FND_FEATURE_BIT_FRIEND              0x0004
#define FND_FEATURE_BIT_LOW_POWER           0x0008

#define MESH_CONFIG_RELAY_RETRANSMIT_INTERVAL_STEP      10      ///< Interval is in 10ms steps
#define MESH_CONFIG_PUBLISH_RETRANSMIT_INTERVAL_STEP    50      ///< Interval is in 50ms steps

typedef uint32_t tBSA_MESH_EVT;
typedef uint8_t tBSA_MESH_EVT_MSG;

static void mible_mesh_callback(tBSA_MESH_EVT event, tBSA_MESH_EVT_MSG *p_data);
static int mible_mesh_gateway_set_default_ttl(mible_mesh_default_ttl_t *param);

static void mesh_process_iv_changed(wiced_bt_mesh_core_state_iv_t *p_iv);
static void mesh_process_seq_changed(wiced_bt_mesh_core_state_seq_t *p_seq_changed);
static void mesh_del_seq(uint16_t addr);
static uint8_t mesh_time_ms_to_transtime(uint32_t time_ms);
static uint8_t uint32_to_log_heartbeat(uint32_t period);
static void download_iv(uint32_t *p_iv_idx, uint8_t *p_iv_update);
static void download_rpl_list(void);

extern wiced_bt_mesh_event_t *wiced_bt_mesh_event_from_hci_header(uint8_t **p_buffer, uint16_t *len);
extern void mible_wiced_init(void);
extern void mible_wiced_wait_event(uint32_t timeout);
extern void mible_wiced_set_event(void);

mible_mesh_event_cb_t mesh_event_cb = NULL;

typedef struct
{
#define MIBLE_LOAD_STATE_IDLE                   0
#define MIBLE_LOAD_STATE_ADD_APPKEY             1
#define MIBLE_LOAD_STATE_SIG_MODEL_APP_BIND     2
#define MIBLE_LOAD_STATE_SIG_MODEL_SUB          3
#define MIBLE_LOAD_STATE_VENDOR_MODEL_APP_BIND  4
#define MIBLE_LOAD_STATE_VENDOR_MODEL_SUB       5
#define MIBLE_LOAD_STATE_DEFAULT_TTL            6
    uint8_t state;
    uint8_t model_idx;
    mible_mesh_gateway_info_t info;
} wiced_mible_mesh_state_t;

wiced_mible_mesh_state_t mesh_state = { 0 };

uint16_t received_opcode = 0xffff;
int wait_event_opcode(uint16_t opcode)
{
    while(opcode != received_opcode)
        mible_wiced_wait_event(1000);
    return MI_SUCCESS;
}

/**
 *@brief    start recv unprovision beacon, report result by MIBLE_EVENT.
 *@return   0: success, negetive value: failure
 */
int mible_mesh_start_recv_unprovbeacon(void)
{
    wiced_bt_mesh_provision_scan_start_data_t data;
    memset(&data, 0, sizeof(data));

    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, mesh_state.info.unicast_address, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    data.scanned_items_limit = 10;
    data.timeout = 255;

    return wiced_bt_mesh_provision_scan_start(p_event, &data) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    stop recv unprovision beacon, terminate report result.
 *@return   0: success, negetive value: failure
 */
int mible_mesh_stop_recv_unprovbeacon(void)
{
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, mesh_state.info.unicast_address, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    return wiced_bt_mesh_provision_scan_stop(p_event) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    Config Composition Data Get, mesh profile 4.3.2.4, Report 4.3.2.5 Config Composition Data Status.
 *          report event: MIBLE_MESH_EVENT_CONFIG_MESSAGE_CB, data: mible_mesh_access_message_rx_t.
 *@param    [in] unicast_address: node address
 *@param    [in] netkey_index: key index for node
 *@param    [in] page: page number of the composition data
 *@return   0: success, negetive value: failure
 */
int mible_mesh_node_get_composition_data(uint16_t unicast_address, uint16_t global_netkey_index, uint8_t page)
{
    wiced_bt_mesh_config_composition_data_get_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, unicast_address, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    data.page_number = page;

    return wiced_bt_mesh_config_composition_data_get(p_event, &data) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    appkey information for node, mesh profile 4.3.2.37-39, Report 4.3.2.40 Config AppKey Status.
 *          report event: MIBLE_MESH_EVENT_CONFIG_MESSAGE_CB, data: mible_mesh_access_message_rx_t.
 *@param    [in] opcode : mesh spec opcode, add/update/delete ...
 *@param    [in] param : appkey parameters corresponding to node
 *@return   0: success, negetive value: failure
 */
int mible_mesh_node_set_appkey(uint16_t opcode, mible_mesh_appkey_params_t *param)
{
    wiced_bt_mesh_config_appkey_change_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, param->dst_addr, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    data.operation = (opcode == WICED_BT_MESH_CORE_CMD_APPKEY_ADD) ? OPERATION_ADD : ((opcode == WICED_BT_MESH_CORE_CMD_APPKEY_DELETE) ? OPERATION_DELETE : OPERATION_UPDATE);
    data.net_key_idx = param->netkey_index;
    data.app_key_idx = param->appkey_index;
    memcpy(data.app_key, param->appkey, MIBLE_MESH_KEY_LEN);

    return wiced_bt_mesh_config_appkey_change(p_event, &data) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    bind appkey information for node, mesh profile 4.3.2.46-47, Report 4.3.2.48 Config Model App Status.
 *          report event: MIBLE_MESH_EVENT_CONFIG_MESSAGE_CB, data: mible_mesh_access_message_rx_t.
 *@param    [in] opcode : mesh spec opcode, bind/unbind
 *@param    [in] param : bind parameters corresponding to node
 *@return   0: success, negetive value: failure
 */
int mible_mesh_node_bind_appkey(uint16_t opcode, mible_mesh_model_app_params_t *param)
{
    wiced_bt_mesh_config_model_app_bind_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, param->dst_addr, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;
    
    data.operation = (opcode == WICED_BT_MESH_CORE_CMD_MODEL_APP_BIND) ? OPERATION_BIND : OPERATION_UNBIND;
    data.element_addr = param->element_addr;
    // Check with Xiaomi if following 2 lines are correct
    data.company_id = param->model_id.company_id;
    data.model_id = param->model_id.model_id;
    data.app_key_idx = param->appkey_index;

    return wiced_bt_mesh_config_model_app_bind(p_event, &data) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    set publication information for node, mesh profile 4.3.2.15-17,
 *          Report 4.3.2.18 Config Model Publication Status.
 *          report event: MIBLE_MESH_EVENT_CONFIG_MESSAGE_CB, data: mible_mesh_access_message_rx_t.
 *@param    [in] opcode : mesh spec opcode, add/delete/overwrite ...
 *@param    [in] param : publish parameters corresponding to node
 *@return   0: success, negetive value: failure
 */
int mible_mesh_node_set_publication(uint16_t opcode, mible_mesh_publication_params_t * param)
{
    wiced_bt_mesh_config_model_publication_set_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, param->dst_addr, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    memset(&data, 0, sizeof(data));

    data.element_addr = param->element_addr;
    // Check with Xiaomi if following 2 lines are correct
    data.company_id = param->model_id.company_id;
    data.model_id = param->model_id.model_id;
    data.app_key_idx = param->appkey_index;
    data.credential_flag = param->credential_flag;
    data.publish_ttl = param->pub_ttl;
    data.publish_period = param->pub_period;
    data.publish_retransmit_count = param->pub_retrans_count;
    data.publish_retransmit_interval = param->pub_retrans_intvl_steps * MESH_CONFIG_PUBLISH_RETRANSMIT_INTERVAL_STEP; // we use value in msec
    if (param->publish_addr.type == MIBLE_MESH_ADDRESS_TYPE_VIRTUAL)
        memcpy(data.publish_addr, param->publish_addr.virtual_uuid, MIBLE_MESH_KEY_LEN);
    else
        *(uint16_t *)data.publish_addr = param->publish_addr.value;

    return wiced_bt_mesh_config_model_publication_set(p_event, &data) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    set subscription information for node, mesh profile 4.3.2.19-25.
 *          Report 4.3.2.26 Config Model Subscription Status.
 *          report event: MIBLE_MESH_EVENT_CONFIG_MESSAGE_CB, data: mible_mesh_access_message_rx_t.
 *@param    [in] opcode : mesh spec opcode, add/delete/overwrite ...
 *@param    [in] param : subscription parameters corresponding to node
 *@return   0: success, negetive value: failure
 */
int mible_mesh_node_set_subscription(uint16_t opcode, mible_mesh_subscription_params_t * param)
{
    wiced_bt_mesh_config_model_subscription_change_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, param->dst_addr, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    memset(&data, 0, sizeof(data));

    switch (opcode)
    {
    case WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_ADD:
    case WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_VIRT_ADDR_ADD:
        data.operation = OPERATION_ADD;
        break;
    case WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_DELETE:
    case WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_VIRT_ADDR_DELETE:
        data.operation = OPERATION_DELETE;
        break;
    case WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_OVERWRITE:
    case WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_VIRT_ADDR_OVERWRITE:
        data.operation = OPERATION_OVERWRITE;
        break;
    case WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_DELETE_ALL:
        data.operation = OPERATION_DELETE_ALL;
        break;
    default:
        return MI_ERR_INVALID_PARAM;
    }
    data.element_addr = param->element_addr;                  /* Address of the element */
    data.company_id = param->model_id.company_id;
    data.model_id = param->model_id.model_id;
    if (param->sub_addr.type == MIBLE_MESH_ADDRESS_TYPE_VIRTUAL)
        memcpy(data.addr, param->sub_addr.virtual_uuid, MIBLE_MESH_KEY_LEN);
    else
        *(uint16_t *)data.addr = param->sub_addr.value;

    Log("Model Sub Operation:%d addr:%04x elem:%x comp_id:%04x model:%04x addr:%04x", data.operation, p_event->dst, data.element_addr, data.company_id, data.model_id, data.addr[0] + (data.addr[1] << 8));

    return wiced_bt_mesh_config_model_subscription_change(p_event, &data) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    reset node, 4.3.2.53 Config Node Reset, Report 4.3.2.54 Config Node Reset Status.
 *          report event: MIBLE_MESH_EVENT_CONFIG_MESSAGE_CB, data: mible_mesh_access_message_rx_t.
 *@param    [in] opcode : mesh spec opcode, reset
 *@param    [in] param : reset parameters corresponding to node
 *@return   0: success, negetive value: failure
 */
int mible_mesh_node_reset(uint16_t opcode, mible_mesh_reset_params_t *param)
{
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, param->dst_addr, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    return wiced_bt_mesh_config_node_reset(p_event) ? MI_SUCCESS : MI_ERR_INTERNAL;
}
	
/**
 *@brief    set relay params node, mesh profile 4.3.2.12, Report 4.3.2.14 Config Relay Status.
 *          report event: MIBLE_MESH_EVENT_CONFIG_MESSAGE_CB, data: mible_mesh_access_message_rx_t.
 *@param    [in] opcode : relay
 *@param    [in] param : relay parameters corresponding to node
 *@return   0: success, negetive value: failure
 */
int mible_mesh_node_set_relay_param(uint16_t opcode, mible_mesh_relay_params_t *param)
{
    wiced_bt_mesh_config_relay_set_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, param->dst_addr, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    data.state = param->relay;
    data.retransmit_count = param->retransmit_count;
    data.retransmit_interval = param->retransmit_interval_steps * MESH_CONFIG_RELAY_RETRANSMIT_INTERVAL_STEP; // we use value in msec
		
    return wiced_bt_mesh_config_relay_set(p_event, &data) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    generic message, Mesh model 3.2, 4.2, 5.2, 6.3, or 7 Summary.
 *          report event: MIBLE_MESH_EVENT_GENERIC_OPTION_CB, data: mible_mesh_access_message_rx_t.
 *@param    [in] param : control parameters corresponding to node
 *          according to opcode, generate a mesh message; extral params: ack_opcode, tid, get_or_set.
 *@return   0: success, negetive value: failure
 */
int mible_mesh_node_generic_control(mible_mesh_generic_params_t * param)
{
    wiced_bt_mesh_event_t *p_event;

    // ToDo, not sure how to handle virtual address
    p_event = wiced_bt_mesh_create_event((uint8_t)param->element_index, param->opcode.company_id, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_CLNT, param->dst_addr.value, param->global_appkey_index);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    p_event->opcode = param->opcode.opcode;

    return (wiced_bt_mesh_core_send(p_event, param->data, param->data_len, NULL) == WICED_BT_SUCCESS) ? MI_SUCCESS : MI_ERR_INTERNAL;
}

/**
 *@brief    sync method, register event callback
 *@param    [in] mible_mesh_event_cb : event callback
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_register_event_callback(mible_mesh_event_cb_t mible_mesh_event_cb)
{
    mesh_event_cb = mible_mesh_event_cb;
    return MI_SUCCESS;
}

/**
 *@brief    sync method, unregister event callback
 *@param    [in] mible_mesh_event_cb : event callback
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_unregister_event_callback(mible_mesh_event_cb_t mible_mesh_event_cb)
{
    mesh_event_cb = NULL;
    return MI_SUCCESS;
}

/**
 *@brief    async method, init mesh stack.
 *          report event: MIBLE_MESH_EVENT_STACK_INIT_DONE, data: NULL.
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_init_stack(void)
{
    if (mesh_event_cb == NULL)
        return MI_ERR_NOSYS;

    mible_wiced_init();
#if BSA
    /* Open connection to BSA Server */
    if (app_mgr_open(APP_DEFAULT_UIPC_PATH) == -1)
    {
        APP_ERROR0("Unable to connect to server");
        return MI_ERR_INTERNAL;
    }

    /* Example of function to start MESH application */
    // If needed specify a callback app_mesh_evt_cback
    status = app_mesh_enable(mible_mesh_callback);
    if (status != BSA_SUCCESS)
    {
        APP_ERROR0("main: Unable to start MESH");
        app_mgr_close();
        return status;
    }
#endif
    // ToDo, I believe there should be an event from BSA server that init is done.
    mesh_event_cb(MIBLE_MESH_EVENT_STACK_INIT_DONE, NULL);
    return MI_SUCCESS;
}

/**
 *@brief    async method, init mesh provisioner
 *          load self info, include unicast address, iv, seq_num, init model;
 *          clear local db, related appkey_list, netkey_list, device_key_list,
 *          we will load latest data for cloud;
 *          report event: MIBLE_MESH_EVENT_PROVISIONER_INIT_DONE, data: NULL.
 *@param    [in] info : init parameters corresponding to gateway
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_init_provisioner(mible_mesh_gateway_info_t *info)
{
    Log("mible_mesh_gateway_init_provisioner\n");

    if (mesh_event_cb == NULL)
        return MI_ERR_NOSYS;

    memcpy(&mesh_state.info, info, sizeof(mible_mesh_gateway_info_t));

    mesh_event_cb(MIBLE_MESH_EVENT_PROVISIONER_INIT_DONE, NULL);
    return MI_SUCCESS;
}

/**
 *@brief    sync method, create mesh network for provisioner.
 *@param    [in] netkey_index : key index for netkey
 *@param    [in] netkey : netkey value
 *@param    [in|out] stack_netkey_index : [in] default value: 0xFFFF, [out] stack generates netkey_index
 *          if your stack don't manage netkey_index and stack_netkey_index relationships, update stack_netkey_index;
 *          otherwise, do nothing.
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_create_network(uint16_t netkey_index, uint8_t *netkey, uint16_t *stack_netkey_index)
{
    wiced_bt_mesh_local_device_set_data_t set;

    Log("mible_mesh_gateway_init_provisioner\n");

    if (mesh_event_cb == NULL)
        return MI_ERR_NOSYS;

    set.addr = mesh_state.info.unicast_address;                                     /* Local Node Address */
    // set.dev_key[16];                                                             /* Local Device Key */
    memcpy(set.network_key, netkey, MIBLE_MESH_KEY_LEN);                            /* Mesh Network Key */
    set.net_key_idx = netkey_index;                                                 /* Network Key Index */
    set.iv_idx = mesh_state.info.iv_index;                                          /* Current Network IV Index */
    set.key_refresh = (mesh_state.info.flags & MIBLE_KEY_REFRESH_FLAG) != 0;        /* 1 if Key Refresh Phase 2 is in progress */
    set.iv_update = (mesh_state.info.flags & MIBLE_KEY_IV_UPDATE_FLAG) != 0;        /* 1 if IV Update procedure is in progress */
    set.model_level_access = 1;

    wiced_bt_mesh_provision_local_device_set(&set);

    download_rpl_list();
    return MI_SUCCESS;
}


#if 0
void mesh_load_config_process_opcode(uint32_t opcode)
{
    uint16_t model_id;

    Log("load config state:%d opcode:%04x\n", mesh_state.state, opcode);

    switch (mesh_state.state)
    {
    case MIBLE_LOAD_STATE_ADD_APPKEY:
        if (opcode != WICED_BT_MESH_CORE_CMD_APPKEY_STATUS)
        {
            break;
        }
        mesh_state.state = MIBLE_LOAD_STATE_SIG_MODEL_APP_BIND;
        mesh_state.model_idx = 0;

        // Fall through to the next state
        opcode = WICED_BT_MESH_CORE_CMD_MODEL_APP_STATUS;

    case MIBLE_LOAD_STATE_SIG_MODEL_APP_BIND:
        if (opcode != WICED_BT_MESH_CORE_CMD_MODEL_APP_STATUS)
        {
            break;
        }
        // bind appkey to the model
        for (; mesh_state.model_idx < mesh_state.info.sig_model_db.model_num; mesh_state.model_idx++)
        {
            if ((mesh_state.info.sig_model_db.model_list[mesh_state.model_idx] == WICED_BT_MESH_CORE_MODEL_ID_CONFIG_SRV) ||
                (mesh_state.info.sig_model_db.model_list[mesh_state.model_idx] == WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT))
                continue;

            mible_mesh_gateway_set_model_app(MIBLE_MESH_OP_ADD, MIBLE_MESH_COMPANY_ID_SIG, mesh_state.info.sig_model_db.model_list[mesh_state.model_idx], mesh_state.info.appkey_index);
            mesh_state.model_idx++;
            return;
        }
        mesh_state.state = MIBLE_LOAD_STATE_SIG_MODEL_SUB;
        mesh_state.model_idx = 0;
        opcode = WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_STATUS;

    case MIBLE_LOAD_STATE_SIG_MODEL_SUB:
        if (opcode == WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_STATUS)
        {
            /* Add subscribtion of every model to the primary group address */
            for (; mesh_state.model_idx < mesh_state.info.sig_model_db.model_num; mesh_state.model_idx++)
            {
                if ((mesh_state.info.sig_model_db.model_list[mesh_state.model_idx] == WICED_BT_MESH_CORE_MODEL_ID_CONFIG_SRV) ||
                    (mesh_state.info.sig_model_db.model_list[mesh_state.model_idx] == WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT))
                    continue;

                model_id = mesh_state.info.sig_model_db.model_list[mesh_state.model_idx];

                mible_mesh_subscription_params_t param;
                param.dst_addr = mesh_state.info.unicast_address;
                param.element_addr = mesh_state.info.unicast_address;
                param.model_id = model_id;
                param.sub_addr.type = MIBLE_MESH_ADDRESS_TYPE_GROUP;
                param.sub_addr.value = mesh_state.info.group_address;
                param.global_netkey_index = mesh_state.info.netkey_index;

                mible_mesh_gateway_set_sub_address(MIBLE_MESH_OP_ADD, &param);
                mesh_state.model_idx++;
                return;
            }
        }
        mesh_state.state = MIBLE_LOAD_STATE_VENDOR_MODEL_APP_BIND;
        mesh_state.model_idx = 0;

        // Fall through to the next state
        opcode = WICED_BT_MESH_CORE_CMD_MODEL_APP_STATUS;

    case MIBLE_LOAD_STATE_VENDOR_MODEL_APP_BIND:
        if (opcode != WICED_BT_MESH_CORE_CMD_MODEL_APP_STATUS)
        {
            break;
        }
        // bind appkey to the model
        for (; mesh_state.model_idx < mesh_state.info.vendor_model_db.model_num; mesh_state.model_idx++)
        {
            mible_mesh_gateway_set_model_app(MIBLE_MESH_OP_ADD, mesh_state.info.vendor_model_db.model_list[mesh_state.model_idx].model_id.company_id, mesh_state.info.vendor_model_db.model_list[mesh_state.model_idx].model_id.model_id, mesh_state.info.appkey_index);
            mesh_state.model_idx++;
            return;
        }
        mesh_state.state = MIBLE_LOAD_STATE_VENDOR_MODEL_SUB;
        mesh_state.model_idx = 0;
        opcode = WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_STATUS;

    case MIBLE_LOAD_STATE_VENDOR_MODEL_SUB:
        if (opcode != WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_STATUS)
        {
            break;
        }
        /* Add subscribtion of every model to the primary group address */
        for (; mesh_state.model_idx < mesh_state.info.vendor_model_db.model_num; mesh_state.model_idx++)
        {
            model_id = mesh_state.info.vendor_model_db.model_list[mesh_state.model_idx].model_id;

            mible_mesh_subscription_params_t param;
            param.dst_addr = mesh_state.info.unicast_address;
            param.element_addr = mesh_state.info.unicast_address;
            param.model_id = model_id;
            param.sub_addr.type = MIBLE_MESH_ADDRESS_TYPE_GROUP;
            param.sub_addr.value = mesh_state.info.group_address;
            param.global_netkey_index = mesh_state.info.netkey_index;

            mible_mesh_gateway_set_sub_address(MIBLE_MESH_OP_ADD, &param);
            mesh_state.model_idx++;
            return;
        }
        mesh_state.state = MIBLE_LOAD_STATE_DEFAULT_TTL;
        mible_mesh_default_ttl_t param;
        param.dst_addr = mesh_state.info.unicast_address;
        param.ttl = (uint8_t)mesh_state.info.default_ttl;

        mible_mesh_gateway_set_default_ttl(&param);
        return;

    case MIBLE_LOAD_STATE_DEFAULT_TTL:
        if (opcode != WICED_BT_MESH_CORE_CMD_CONFIG_DEFAULT_TTL_STATUS)
        {
            break;
        }
        mesh_state.state = MIBLE_LOAD_STATE_IDLE;
        if (mesh_event_cb != NULL)
            mesh_event_cb(MIBLE_MESH_EVENT_LOAD_CONFIG_DONE, NULL);
        return;
    }
    Log("Ignored\n");
}
#endif

/**
 *@brief    set local provisioner network transmit params.
 *@param    [in] count : advertise counter for every adv packet, adv transmit times
 *@param    [in] interval_steps : adv interval = interval_steps*0.625ms
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_set_network_transmit_param(uint8_t count, uint8_t interval_steps)
{
    wiced_bt_mesh_config_network_transmit_set_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, mesh_state.info.unicast_address, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    data.count = count;
    data.interval = interval_steps * MESH_CONFIG_RELAY_RETRANSMIT_INTERVAL_STEP; // we use value in msec

    if (!wiced_bt_mesh_config_network_transmit_params_set(p_event, &data))
        return MI_ERR_INTERNAL;

    return wait_event_opcode(WICED_BT_MESH_CORE_CMD_CONFIG_NETWORK_TRANSMIT_STATUS);
}

/**
 *@brief    update iv index, .
 *@param    [in] iv_index : current IV Index
 *@param    [in] flags : contains the Key Refresh Flag and IV Update Flag
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_update_iv_info(uint32_t iv_index, uint8_t flags)
{
    return MI_ERR_NOSYS;
}

/**
 *@brief    add/delete local netkey.
 *@param    [in] op : add or delete
 *@param    [in] netkey_index : key index for netkey
 *@param    [in] netkey : netkey value
 *@param    [in|out] stack_netkey_index : [in] default value: 0xFFFF, [out] stack generates netkey_index
 *          if your stack don't manage netkey_index and stack_netkey_index relationships, update stack_netkey_index.
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_set_netkey(mible_mesh_op_t op, uint16_t netkey_index, uint8_t *netkey,
            uint16_t *stack_netkey_index)
{
    wiced_bt_mesh_config_netkey_change_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, mesh_state.info.unicast_address, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    if (op == MIBLE_MESH_OP_DELETE)
    {
        data.operation = OPERATION_DELETE;
    }
    else
    {
        data.operation = (op == MIBLE_MESH_OP_ADD) ? OPERATION_ADD : OPERATION_UPDATE;
        memcpy(data.net_key, netkey, MIBLE_MESH_KEY_LEN);
    }

    data.net_key_idx = netkey_index;

    *stack_netkey_index = netkey_index;

    Log("NetKey %s addr:%04x net_key_idx:%04x", data.operation == OPERATION_ADD ? "Add" : "Update", p_event->dst, data.net_key_idx);
    if (!wiced_bt_mesh_config_netkey_change(p_event, &data))
        return MI_ERR_INTERNAL;

    return wait_event_opcode(WICED_BT_MESH_CORE_CMD_NETKEY_STATUS);
}

/**
 *@brief    add/delete local appkey.
 *@param    [in] op : add or delete
 *@param    [in] netkey_index : key index for netkey
 *@param    [in] appkey_index : key index for appkey
 *@param    [in] appkey : appkey value
 *@param    [in|out] stack_appkey_index : [in] default value: 0xFFFF, [out] stack generates appkey_index
 *          if your stack don't manage appkey_index and stack_appkey_index relationships, update stack_appkey_index.
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_set_appkey(mible_mesh_op_t op, uint16_t netkey_index, uint16_t appkey_index,
            uint8_t * appkey, uint16_t *stack_appkey_index)
{
    wiced_bt_mesh_config_appkey_change_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, mesh_state.info.unicast_address, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    if (op == MIBLE_MESH_OP_DELETE)
    {
        data.operation = OPERATION_DELETE;
    }
    else
    {
        data.operation = (op == MIBLE_MESH_OP_ADD) ? OPERATION_ADD : OPERATION_UPDATE;
        memcpy(data.app_key, appkey, MIBLE_MESH_KEY_LEN);
    }
    data.net_key_idx = netkey_index;
    data.app_key_idx = appkey_index;
    
    if (stack_appkey_index != NULL)
        *stack_appkey_index = appkey_index;

    Log("AppKey %s addr:%04x net_key_idx:%04x app_key_idx:%04x\n", data.operation == OPERATION_ADD ? "Add" : "Update", p_event->dst, data.net_key_idx, data.app_key_idx);

    if (!wiced_bt_mesh_config_appkey_change(p_event, &data))
        return MI_ERR_INTERNAL;

    return wait_event_opcode(WICED_BT_MESH_CORE_CMD_APPKEY_STATUS);
}

/**
*@brief    bind/unbind model app.
*@param    [in] op : bind is MIBLE_MESH_OP_ADD, unbind is MIBLE_MESH_OP_DELETE
*@param    [in] company_id: company id
*@param    [in] model_id : model_id
*@param    [in] appkey_index : key index for appkey
*@return   0: success, negetive value: failure
*/
int mible_mesh_gateway_set_model_app(mible_mesh_op_t op, uint16_t company_id, uint16_t model_id,
    uint16_t appkey_index)
{
    wiced_bt_mesh_config_model_app_bind_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, mesh_state.info.unicast_address, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    data.operation = op == 0 ? OPERATION_BIND : OPERATION_UNBIND;
    data.element_addr = mesh_state.info.unicast_address;
    data.company_id = company_id;
    data.model_id = model_id;
    data.app_key_idx = appkey_index;

    Log("Model App %s addr:0x%04x element_addr:%04x comp_id:%04x model_id:%04x app_key_idx:%04x\n", data.operation == OPERATION_BIND ? "Bind" : "Unbind", p_event->dst, data.element_addr, data.company_id, data.model_id, data.app_key_idx);
    if (!wiced_bt_mesh_config_model_app_bind(p_event, &data))
        return MI_ERR_INTERNAL;

    return wait_event_opcode(WICED_BT_MESH_CORE_CMD_MODEL_APP_STATUS);
}

/**
 *@brief    add/delete device key.
 *@param    [in] op : add or delete
 *@param    [in] unicast_address: remote device unicast address
 *@param    [in] device_key : device key value
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_set_device_key(mible_mesh_op_t op, mible_mesh_node_info_t *device)
{
    wiced_bt_mesh_set_dev_key_data_t set;

    if (op != MIBLE_MESH_OP_ADD)
    {
        Log("Del device key not supported:%04x\n", device->unicast_address);
        return MI_ERR_NOSYS;
    }
    set.dst = device->unicast_address;
    set.net_key_idx = device->netkey_index;
    memcpy(set.dev_key, device->device_key, MIBLE_MESH_KEY_LEN);
    // uint16_t device->elements_num;
    // uint8_t device->uuid[MIBLE_MESH_DEV_UUID_LEN];

    wiced_bt_mesh_provision_set_dev_key(&set);
    return MI_SUCCESS;
}

/**
 *@brief    add/delete subscription params.
 *@param    [in] op : add or delete
 *@param    [in] param: subscription params
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_set_sub_address(mible_mesh_op_t op, mible_mesh_subscription_params_t *param)
{
    wiced_bt_mesh_config_model_subscription_change_data_t data;
    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, param->dst_addr, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    memset(&data, 0, sizeof(data));

    data.operation = (op == MIBLE_MESH_OP_ADD) ? OPERATION_ADD : OPERATION_DELETE;
    data.element_addr = param->element_addr;                  /* Address of the element */
    data.company_id = param->model_id.company_id;
    data.model_id = param->model_id.model_id;
    if (param->sub_addr.type == MIBLE_MESH_ADDRESS_TYPE_VIRTUAL)
        memcpy(data.addr, param->sub_addr.virtual_uuid, MIBLE_MESH_KEY_LEN);
    else
        *(uint16_t *)data.addr = param->sub_addr.value;

    Log("Model Sub Operation:%d addr:%04x elem:%x comp_id:%04x model:%x addr:%04x", data.operation, p_event->dst, data.element_addr, data.company_id, data.model_id, data.addr[0] + (data.addr[1] << 8));

    if (!wiced_bt_mesh_config_model_subscription_change(p_event, &data))
        return MI_ERR_INTERNAL;

    return wait_event_opcode(WICED_BT_MESH_CORE_CMD_CONFIG_MODEL_SUBSCRIPTION_STATUS);
}

/**
 *@brief    TTL value
 *@return   0: success, negetive value: failure
 */
int mible_mesh_gateway_set_default_ttl(mible_mesh_default_ttl_t *param)
{
    wiced_bt_mesh_config_default_ttl_set_data_t data;

    wiced_bt_mesh_event_t *p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, param->dst_addr, 0xFFFF);
    if (p_event == NULL)
        return MI_ERR_NO_MEM;

    memset(&data, 0, sizeof(data));

    data.ttl = param->ttl;

    Log("Set TTL addr:%04x ttl:%d", p_event->dst, data.ttl);

    if (!wiced_bt_mesh_config_default_ttl_set(p_event, &data))
        return MI_ERR_INTERNAL;

    return MI_SUCCESS;
}

/**
 *@brief    suspend adv send for mesh stack.
 *@param    [in] NULL
 *@return   0: success, negetive value: failure
 */
int mible_mesh_suspend_transmission(void)
{
    return MI_ERR_NOSYS;
}

/**
 *@brief    resume adv send for mesh stack.
 *@param    [in] NULL
 *@return   0: success, negetive value: failure
 */
int mible_mesh_resume_transmission(void)
{
    return MI_ERR_NOSYS;
}

#if BSA
static void mible_mesh_callback(tBSA_MESH_EVT event, tBSA_MESH_EVT_MSG *p_data)
{
    switch (event)
    {
    case BSA_MESH_DISABLE_EVT:
        Log("BSA_MESH_DISABLE_EVT");
        break;

    case BSA_MESH_CORE_EVT:
        Log("BSA_MESH_CORE_EVT Received - status: %d", p_data->core_evt.status);
        Log("BSA_MESH_CORE_EVT Received - opcode: %d, length: %d", (uint16_t)p_data->core_evt.evt_type, (uint16_t)p_data->core_evt.len);

        wiced_hci_process_data((uint16_t)p_data->core_evt.evt_type, p_data->core_evt.data, (uint16_t)p_data->core_evt.len);
        break;

    default:
        Log("mible_mesh_cback unknown event:%d", event);
        break;
    }
}
#endif

void mesh_bsa_provision_process_event(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    mible_mesh_event_params_t params;
    uint8_t *p_comp_data = NULL;

    if (event == WICED_BT_MESH_SEQ_CHANGED)
    {
        mesh_process_seq_changed((wiced_bt_mesh_core_state_seq_t *)p_data);
        return;
    }
    if (event == WICED_BT_MESH_COMMAND_STATUS)
    {
        if (p_event != NULL)
            wiced_bt_mesh_release_event(p_event);
        return;
    }

    if (event == WICED_BT_MESH_IV_CHANGED)
    {
        params.mesh_iv.iv_index = ((wiced_bt_mesh_core_state_iv_t *)p_data)->index;
        params.mesh_iv.flags = ((wiced_bt_mesh_core_state_iv_t *)p_data)->update_flag;

        if (mesh_event_cb != NULL)
            mesh_event_cb(MIBLE_MESH_EVENT_IV_UPDATE, &params);

        mesh_process_iv_changed((wiced_bt_mesh_core_state_iv_t *)p_data);
        if (p_event != NULL)
            wiced_bt_mesh_release_event(p_event);
        return;
    }
    if (event == WICED_BT_MESH_RAW_MODEL_DATA)
    {
        params.generic_msg.opcode.company_id = p_event->company_id;
        params.generic_msg.opcode.opcode = p_event->opcode;

        params.generic_msg.meta_data.src_addr = p_event->src;
        params.generic_msg.meta_data.dst_addr = p_event->dst;
        params.generic_msg.meta_data.appkey_index = p_event->app_key_idx;
        params.generic_msg.meta_data.netkey_index = 0; // this field shall not be present, because it is uniquely defined by the appkey_index.
        params.generic_msg.meta_data.rssi = p_event->rssi;
        params.generic_msg.meta_data.ttl = p_event->ttl;
        params.generic_msg.buf = p_data;
        params.generic_msg.buf_len = p_event->data_len;

        if (mesh_event_cb != NULL)
            mesh_event_cb(MIBLE_MESH_EVENT_GENERIC_MESSAGE_CB, &params);
        if (p_event != NULL)
            wiced_bt_mesh_release_event(p_event);
        return;
    }
    if (event == WICED_BT_MESH_PROVISION_SCAN_REPORT)
    {
        memcpy(params.unprov_beacon.device_uuid, ((wiced_bt_mesh_provision_scan_report_data_t *)p_data)->uuid, MIBLE_MESH_DEV_UUID_LEN);
        params.unprov_beacon.oob_info = ((wiced_bt_mesh_provision_scan_report_data_t *)p_data)->oob;
        memcpy(params.unprov_beacon.uri_hash, &((wiced_bt_mesh_provision_scan_report_data_t *)p_data)->uri_hash, MIBLE_MESH_URI_HASH_LEN);
#ifdef PROVISION_SCAN_REPORT_INCLUDE_BDADDR
        memcpy(params.unprov_beacon.mac, ((wiced_bt_mesh_provision_scan_report_data_t *)p_data)->bdaddr, MIBLE_ADDR_LEN);
#endif
        if (mesh_event_cb != NULL)
            mesh_event_cb(MIBLE_MESH_EVENT_UNPROV_DEVICE, &params);
        if (p_event != NULL)
            wiced_bt_mesh_release_event(p_event);
        return;
    }
    if (event == WICED_BT_MESH_PROVISION_DEVICE_CAPABILITIES)
    {
        memcpy(&params.dev_caps, p_data, sizeof(params.dev_caps));
        if (mesh_event_cb != NULL)
            mesh_event_cb(MIBLE_MESH_EVENT_PROV_CAPS, &params);
        if (p_event != NULL)
            wiced_bt_mesh_release_event(p_event);
        return;
    }
    if (event == WICED_BT_MESH_PROVISION_END)
    {
        memcpy(&params.prov_end, p_data, sizeof(params.prov_end));
        if (mesh_event_cb != NULL)
            mesh_event_cb(MIBLE_MESH_EVENT_PROV_END, &params);
        if (p_event != NULL)
            wiced_bt_mesh_release_event(p_event);
        return;
    }
    if (p_event->src == mesh_state.info.unicast_address)
    {
        received_opcode = p_event->opcode;
        mible_wiced_set_event();
        if (p_event != NULL)
            wiced_bt_mesh_release_event(p_event);
        return;
    }
    params.config_msg.opcode.company_id = p_event->company_id;
    params.config_msg.opcode.opcode = p_event->opcode;

    params.config_msg.meta_data.src_addr = p_event->src;
    params.config_msg.meta_data.dst_addr = p_event->dst;
    params.config_msg.meta_data.appkey_index = p_event->app_key_idx;
    params.config_msg.meta_data.netkey_index = 0; // this field shall not be present, because it is uniquely defined by the appkey_index.
    params.config_msg.meta_data.rssi = p_event->rssi;
    params.config_msg.meta_data.ttl = p_event->ttl;

    switch (event)
    {
    case WICED_BT_MESH_CONFIG_NODE_RESET_STATUS:
        break;

    case WICED_BT_MESH_CONFIG_COMPOSITION_DATA_STATUS:
        p_comp_data = ((wiced_bt_mesh_config_composition_data_status_data_t *)p_data)->data;
        params.config_msg.compo_data_status.page = ((wiced_bt_mesh_config_composition_data_status_data_t *)p_data)->page_number;
        params.config_msg.compo_data_status.company_id = p_comp_data[0] + (p_comp_data[1] << 8);
        params.config_msg.compo_data_status.product_id = p_comp_data[2] + (p_comp_data[3] << 8);
        params.config_msg.compo_data_status.version_id = p_comp_data[4] + (p_comp_data[5] << 8);
        params.config_msg.compo_data_status.crpl = p_comp_data[6] + (p_comp_data[7] << 8);
        params.config_msg.compo_data_status.features = p_comp_data[8] + (p_comp_data[9] << 8);
        params.config_msg.compo_data_status.data_len = ((wiced_bt_mesh_config_composition_data_status_data_t *)p_data)->data_len - 10;
        params.config_msg.compo_data_status.data = &p_comp_data[10];
        break;

    case WICED_BT_MESH_CONFIG_NETKEY_STATUS:
        params.config_msg.netkey_status.status = ((wiced_bt_mesh_config_netkey_status_data_t *)p_data)->status;
        params.config_msg.netkey_status.netkey_index = ((wiced_bt_mesh_config_netkey_status_data_t *)p_data)->net_key_idx;
        break;

    case WICED_BT_MESH_CONFIG_KEY_REFRESH_PHASE_STATUS:
        params.config_msg.keyrefresh_phase_status.status = ((wiced_bt_mesh_config_key_refresh_phase_status_data_t *)p_data)->status;
        params.config_msg.keyrefresh_phase_status.netkey_index = ((wiced_bt_mesh_config_key_refresh_phase_status_data_t *)p_data)->net_key_idx;
        params.config_msg.keyrefresh_phase_status.phase = ((wiced_bt_mesh_config_key_refresh_phase_status_data_t *)p_data)->phase;
        break;

    case WICED_BT_MESH_CONFIG_APPKEY_STATUS:
        params.config_msg.appkey_status.status = ((wiced_bt_mesh_config_appkey_status_data_t *)p_data)->status;
        params.config_msg.appkey_status.appkey_index = ((wiced_bt_mesh_config_appkey_status_data_t *)p_data)->app_key_idx;
        params.config_msg.appkey_status.netkey_index = ((wiced_bt_mesh_config_appkey_status_data_t *)p_data)->net_key_idx;
        break;

    case WICED_BT_MESH_CONFIG_MODEL_APP_BIND_STATUS:
        params.config_msg.model_app_status.status = ((wiced_bt_mesh_config_model_app_bind_status_data_t *)p_data)->status;
        params.config_msg.model_app_status.elem_addr = ((wiced_bt_mesh_config_model_app_bind_status_data_t *)p_data)->element_addr;
        params.config_msg.model_app_status.model_id.company_id = ((wiced_bt_mesh_config_model_app_bind_status_data_t *)p_data)->company_id;
        params.config_msg.model_app_status.model_id.model_id = ((wiced_bt_mesh_config_model_app_bind_status_data_t *)p_data)->model_id;
        params.config_msg.model_app_status.appkey_index = ((wiced_bt_mesh_config_model_app_bind_status_data_t *)p_data)->app_key_idx;
        break;

    case WICED_BT_MESH_CONFIG_MODEL_SUBSCRIPTION_STATUS:
        params.config_msg.model_sub_status.status = ((wiced_bt_mesh_config_model_subscription_status_data_t *)p_data)->status;
        params.config_msg.model_sub_status.elem_addr = ((wiced_bt_mesh_config_model_subscription_status_data_t *)p_data)->element_addr;
        params.config_msg.model_sub_status.model_id.company_id = ((wiced_bt_mesh_config_model_subscription_status_data_t *)p_data)->company_id;
        params.config_msg.model_sub_status.model_id.model_id = ((wiced_bt_mesh_config_model_subscription_status_data_t *)p_data)->model_id;
        params.config_msg.model_sub_status.address = ((wiced_bt_mesh_config_model_subscription_status_data_t *)p_data)->addr;
        break;

    case WICED_BT_MESH_CONFIG_MODEL_PUBLICATION_STATUS:
        params.config_msg.model_pub_status.status = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->status;
        params.config_msg.model_pub_status.elem_addr = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->element_addr;
        params.config_msg.model_pub_status.model_id.company_id = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->company_id;
        params.config_msg.model_pub_status.model_id.model_id = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->model_id;
        params.config_msg.model_pub_status.pub_addr = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->publish_addr;
        params.config_msg.model_pub_status.appkey_index = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->app_key_idx;
        params.config_msg.model_pub_status.cred_flag = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->credential_flag;
        params.config_msg.model_pub_status.pub_ttl = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->publish_ttl;
        params.config_msg.model_pub_status.pub_perid = mesh_time_ms_to_transtime(((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->publish_period);
        params.config_msg.model_pub_status.pub_retrans_cnt = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->publish_retransmit_count;
        params.config_msg.model_pub_status.pub_retrans_intvl_steps = ((wiced_bt_mesh_config_model_publication_status_data_t *)p_data)->publish_retransmit_interval / MESH_CONFIG_PUBLISH_RETRANSMIT_INTERVAL_STEP;
        break;

    case WICED_BT_MESH_CONFIG_NETWORK_TRANSMIT_STATUS:
        params.config_msg.nwk_trans_status.nwk_transcnt = ((wiced_bt_mesh_config_network_transmit_status_data_t *)p_data)->count;
        params.config_msg.nwk_trans_status.nwk_trans_intvl_steps = ((wiced_bt_mesh_config_network_transmit_status_data_t *)p_data)->interval / MESH_CONFIG_RELAY_RETRANSMIT_INTERVAL_STEP;
        break;

    case WICED_BT_MESH_CONFIG_DEFAULT_TTL_STATUS:
        params.config_msg.def_ttl_status.ttl = ((wiced_bt_mesh_config_default_ttl_status_data_t *)p_data)->ttl;
        break;

    case WICED_BT_MESH_CONFIG_RELAY_STATUS:
        params.config_msg.relay_status.relay = ((wiced_bt_mesh_config_relay_status_data_t *)p_data)->state;
        params.config_msg.relay_status.relay_retrans_cnt = ((wiced_bt_mesh_config_relay_status_data_t *)p_data)->retransmit_count;
        params.config_msg.relay_status.relay_retrans_intvlsteps = ((wiced_bt_mesh_config_relay_status_data_t *)p_data)->retransmit_interval / MESH_CONFIG_RELAY_RETRANSMIT_INTERVAL_STEP;
        break;

    case WICED_BT_MESH_CONFIG_FRIEND_STATUS:
        params.config_msg.frnd_status.friend = ((wiced_bt_mesh_config_friend_status_data_t *)p_data)->state;
        break;

    case WICED_BT_MESH_CONFIG_GATT_PROXY_STATUS:
        params.config_msg.gatt_proxy_status.gatt_proxy = ((wiced_bt_mesh_config_gatt_proxy_status_data_t *)p_data)->state;
        break;

    case WICED_BT_MESH_CONFIG_BEACON_STATUS:
        params.config_msg.beacon_status.beacon = ((wiced_bt_mesh_config_beacon_status_data_t *)p_data)->state;
        break;

    case WICED_BT_MESH_CONFIG_NODE_IDENTITY_STATUS:
        params.config_msg.node_ident_status.status = ((wiced_bt_mesh_config_node_identity_status_data_t *)p_data)->status;
        params.config_msg.node_ident_status.netkey_index = ((wiced_bt_mesh_config_node_identity_status_data_t *)p_data)->net_key_idx;
        params.config_msg.node_ident_status.identity = ((wiced_bt_mesh_config_node_identity_status_data_t *)p_data)->identity;
        break;

    case WICED_BT_MESH_CONFIG_MODEL_SUBSCRIPTION_LIST:
        if (((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->company_id == MESH_COMPANY_ID_BT_SIG)
        {
            params.config_msg.sig_model_sub_list.status = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->status;
            params.config_msg.sig_model_sub_list.elem_addr = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->element_addr;
            params.config_msg.sig_model_sub_list.sig_model_id = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->model_id;
            params.config_msg.sig_model_sub_list.num = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->num_addr;
            params.config_msg.sig_model_sub_list.addresses = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->addr;
        }
        else
        {
            params.config_msg.vnd_model_sub_list.status = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->status;
            params.config_msg.vnd_model_sub_list.elem_addr = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->element_addr;
            params.config_msg.vnd_model_sub_list.vnd_model_id.company_id = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->company_id;
            params.config_msg.vnd_model_sub_list.vnd_model_id.model_id = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->model_id;
            params.config_msg.vnd_model_sub_list.num = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->num_addr;
            params.config_msg.vnd_model_sub_list.addresses = ((wiced_bt_mesh_config_model_subscription_list_data_t *)p_data)->addr;
        }
        break;

    case WICED_BT_MESH_CONFIG_NETKEY_LIST:
        params.config_msg.netkey_list.num_of_netkey = ((wiced_bt_mesh_config_netkey_list_data_t *)p_data)->num_keys;
        params.config_msg.netkey_list.pnetkeyindexes = ((wiced_bt_mesh_config_netkey_list_data_t *)p_data)->net_key_idx;
        break;

    case WICED_BT_MESH_CONFIG_APPKEY_LIST:
        params.config_msg.appkey_list.status = ((wiced_bt_mesh_config_appkey_list_data_t *)p_data)->status;
        params.config_msg.appkey_list.netkey_index = ((wiced_bt_mesh_config_appkey_list_data_t *)p_data)->net_key_idx;
        params.config_msg.appkey_list.num_of_appkey = ((wiced_bt_mesh_config_appkey_list_data_t *)p_data)->num_keys;
        params.config_msg.appkey_list.pappkeyindexes = ((wiced_bt_mesh_config_appkey_list_data_t *)p_data)->app_key_idx;
        break;

    case WICED_BT_MESH_CONFIG_MODEL_APP_BIND_LIST:
        if (((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->company_id == MESH_COMPANY_ID_BT_SIG)
        {
            params.config_msg.sig_model_app_list.status = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->status;
            params.config_msg.sig_model_app_list.elem_addr = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->element_addr;
            params.config_msg.sig_model_app_list.model_id = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->model_id;
            params.config_msg.sig_model_app_list.num_of_appkey = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->num_keys;
            params.config_msg.sig_model_app_list.pappkeyindexes = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->app_key_idx;
        }
        else
        {
            params.config_msg.vnd_model_app_list.status = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->status;
            params.config_msg.vnd_model_app_list.elem_addr = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->element_addr;
            params.config_msg.vnd_model_app_list.vnd_model_id.company_id = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->company_id;
            params.config_msg.vnd_model_app_list.vnd_model_id.model_id = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->model_id;
            params.config_msg.vnd_model_app_list.num_of_appkey = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->num_keys;
            params.config_msg.vnd_model_app_list.pappkeyindexes = ((wiced_bt_mesh_config_model_app_list_data_t *)p_data)->app_key_idx;
        }
        break;

    case WICED_BT_MESH_CONFIG_HEARBEAT_SUBSCRIPTION_STATUS:
        params.config_msg.hb_sub_status.status = ((wiced_bt_mesh_config_heartbeat_subscription_status_data_t *)p_data)->status;
        params.config_msg.hb_sub_status.src_addr = ((wiced_bt_mesh_config_heartbeat_subscription_status_data_t *)p_data)->subscription_src;
        params.config_msg.hb_sub_status.dst_addr = ((wiced_bt_mesh_config_heartbeat_subscription_status_data_t *)p_data)->subscription_dst;
        params.config_msg.hb_sub_status.period_log = uint32_to_log_heartbeat(((wiced_bt_mesh_config_heartbeat_subscription_status_data_t *)p_data)->period); 
        params.config_msg.hb_sub_status.count_log = uint32_to_log_heartbeat(((wiced_bt_mesh_config_heartbeat_subscription_status_data_t *)p_data)->count);
        params.config_msg.hb_sub_status.min_hops = ((wiced_bt_mesh_config_heartbeat_subscription_status_data_t *)p_data)->min_hops;
        params.config_msg.hb_sub_status.max_hops = ((wiced_bt_mesh_config_heartbeat_subscription_status_data_t *)p_data)->max_hops;
        break;

    case WICED_BT_MESH_CONFIG_HEARBEAT_PUBLICATION_STATUS:
        params.config_msg.hb_pub_status.status = ((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->status;
        params.config_msg.hb_pub_status.dest_addr = ((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->publication_dst;
        params.config_msg.hb_pub_status.period_log = uint32_to_log_heartbeat(((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->count); 
        params.config_msg.hb_pub_status.count_log = uint32_to_log_heartbeat(((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->period);
        params.config_msg.hb_pub_status.ttl = ((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->ttl;
        params.config_msg.hb_pub_status.netkey_index = ((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->net_key_idx;
        params.config_msg.hb_pub_status.features = 
            ((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->feature_friend ? FND_FEATURE_BIT_FRIEND : 0 |
            ((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->feature_relay ? FND_FEATURE_BIT_RELAY : 0 |
            ((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->feature_proxy ? FND_FEATURE_BIT_PROXY : 0 |
            ((wiced_bt_mesh_config_heartbeat_publication_status_data_t *)p_data)->feature_low_power ? FND_FEATURE_BIT_LOW_POWER : 0;
        break;

    case WICED_BT_MESH_CONFIG_LPN_POLL_TIMEOUT_STATUS:
        params.config_msg.lpn_polltimeout_status.lpn_addr = ((wiced_bt_mesh_lpn_poll_timeout_status_data_t *)p_data)->lpn_addr;
        params.config_msg.lpn_polltimeout_status.polltimeout = ((wiced_bt_mesh_lpn_poll_timeout_status_data_t *)p_data)->poll_timeout;
        break;

    case WICED_BT_MESH_HEALTH_FAULT_STATUS:
        params.config_msg.hlth_fault_status.test_id = ((wiced_bt_mesh_health_fault_status_data_t *)p_data)->test_id;
        params.config_msg.hlth_fault_status.company_id = ((wiced_bt_mesh_health_fault_status_data_t *)p_data)->company_id;
        params.config_msg.hlth_fault_status.fault_num = ((wiced_bt_mesh_health_fault_status_data_t *)p_data)->count;
        params.config_msg.hlth_fault_status.fault_array = ((wiced_bt_mesh_health_fault_status_data_t *)p_data)->fault_array;
        break;

    case WICED_BT_MESH_HEALTH_PERIOD_STATUS:
        params.config_msg.hlth_period_status.fast_peirod_div = ((wiced_bt_mesh_health_period_status_data_t *)p_data)->divisor;
        break;

    case WICED_BT_MESH_HEALTH_ATTENTION_STATUS:
        params.config_msg.hlth_atten_status.attention = ((wiced_bt_mesh_health_attention_status_data_t *)p_data)->timer;
        break;

    case WICED_BT_MESH_PROXY_FILTER_STATUS:
        ((wiced_bt_mesh_proxy_filter_status_data_t *)p_data)->type;
        ((wiced_bt_mesh_proxy_filter_status_data_t *)p_data)->list_size;
        break;

    default:
        break;
    }
    if (mesh_event_cb != NULL)
        mesh_event_cb(MIBLE_MESH_EVENT_CONFIG_MESSAGE_CB, &params);
    if (p_event != NULL)
        wiced_bt_mesh_release_event(p_event);
}

void get_rpl_filename(char *filename)
{
    strcpy(filename, "mesh_rpl.bin");
}


void download_iv(uint32_t *p_iv_idx, uint8_t *p_iv_update)
{
    FILE                *fp;
    mesh_client_iv_t    iv;
    char                filename[50];

    // default values for the case when we don't have RPL file
    *p_iv_idx = 0;
    *p_iv_update = WICED_FALSE;

    get_rpl_filename(filename);

    fp = fopen(filename, "rb");
    if (!fp)
        return;

    // just read IV record - first record in the file
    if (fread(&iv, 1, sizeof(iv), fp) == sizeof(iv))
    {
        *p_iv_idx = iv.iv_index;
        *p_iv_update = iv.iv_update != 0 ? WICED_TRUE : WICED_FALSE;
    }
    fclose(fp);
}

void download_rpl_list(void)
{
    FILE *            fp;
    mesh_client_seq_t entry;
    mesh_client_iv_t  iv;
    char              filename[50];
    uint32_t          seq;

    get_rpl_filename(filename);

    fp = fopen(filename, "rb");
    if (!fp)
        return;

    // skip IV record - first record in the file
    if (fread(&iv, 1, sizeof(iv), fp) == sizeof(iv))
    {
        // Read all SEQ records passing them to the mesh core
        while (fread(&entry, 1, sizeof(entry), fp) == sizeof(entry))
        {
            seq = entry.seq[0] + (((uint32_t)entry.seq[1]) << 8) + (((uint32_t)entry.seq[2]) << 16);
            wiced_bt_mesh_core_set_seq(entry.addr, seq, entry.previous_iv_idx != 0 ? WICED_TRUE : WICED_FALSE);
        }
    }
    fclose(fp);
}

void mesh_process_iv_changed(wiced_bt_mesh_core_state_iv_t *p_iv)
{
    FILE                *fp;
    mesh_client_iv_t    iv;
    char                filename[50];

    get_rpl_filename(filename);

    fp = fopen(filename, "rb+");
    if (!fp)
    {
        fp = fopen(filename, "wb+");
        if (!fp)
            return;
    }
    iv.iv_index = p_iv->index;
    iv.iv_update = p_iv->update_flag ? 1 : 0;
    fwrite(&iv, 1, sizeof(iv), fp);
    fclose(fp);
}

void mesh_process_seq_changed(wiced_bt_mesh_core_state_seq_t *p_seq_changed)
{
    FILE *fp;
    mesh_client_seq_t entry;
    mesh_client_iv_t  iv;
    char filename[50];
    uint32_t            seq;

    get_rpl_filename(filename);

    fp = fopen(filename, "rb+");
    if (!fp)
    {
        fp = fopen(filename, "wb+");
        if (!fp)
            return;
    }
    // skip IV record - first record in the file
    if (fread(&iv, 1, sizeof(iv), fp) != sizeof(iv))
    {
        // no IV record. Create it with 0 values
        iv.iv_index = 0;
        iv.iv_update = 0;
        fwrite(&iv, 1, sizeof(iv), fp);
    }
    else
    {
        while (fread(&entry, 1, sizeof(entry), fp) == sizeof(entry))
        {
            if (p_seq_changed->addr == entry.addr)
            {
                // entry with the same addr found, update the entry if it is changed and return
                seq = entry.seq[0] + (((uint32_t)entry.seq[1]) << 8) + (((uint32_t)entry.seq[2]) << 16);
                if ((seq != p_seq_changed->seq)
                    || ((p_seq_changed->previous_iv_idx == WICED_TRUE) != (entry.previous_iv_idx != 0)))
                {
                    entry.previous_iv_idx = p_seq_changed->previous_iv_idx ? 1 : 0;
                    entry.seq[0] = (uint8_t)p_seq_changed->seq;
                    entry.seq[1] = (uint8_t)(p_seq_changed->seq >> 8);
                    entry.seq[2] = (uint8_t)(p_seq_changed->seq >> 16);
                    fseek(fp, 0 - sizeof(entry), SEEK_CUR);
                    fwrite(&entry, 1, sizeof(entry), fp);
                    fclose(fp);
                }
                return;
            }
        }
    }
    // not found entry with addr, need to create one and stick at the end of the file
    entry.addr = p_seq_changed->addr;
    entry.previous_iv_idx = p_seq_changed->previous_iv_idx ? 1 : 0;
    entry.seq[0] = (uint8_t)p_seq_changed->seq;
    entry.seq[1] = (uint8_t)(p_seq_changed->seq >> 8);
    entry.seq[2] = (uint8_t)(p_seq_changed->seq >> 16);

    fseek(fp, 0, SEEK_END);
    fwrite(&entry, 1, sizeof(entry), fp);
    fclose(fp);
}

void mesh_del_seq(uint16_t addr)
{
    FILE *fp;
    mesh_client_seq_t *p_entry;
    long file_size;
    char filename[50];
    uint8_t *p_buffer;
    get_rpl_filename(filename);

    fp = fopen(filename, "rb");
    if (!fp)
        return;

    fseek(fp, 0, SEEK_END);
    file_size = ftell(fp);
    p_buffer = (uint8_t *)malloc(file_size);
    if (p_buffer == NULL)
        return;
    fseek(fp, 0, SEEK_SET);
    fread(p_buffer, 1, file_size, fp);
    fclose(fp);

    for (p_entry = (mesh_client_seq_t *)(p_buffer + sizeof(mesh_client_iv_t)); (uint8_t *)p_entry < p_buffer + file_size; p_entry++)
    {
        if (p_entry->addr == addr)
            break;
    }
    if ((uint8_t *)p_entry == p_buffer + file_size)
    {
        free(p_buffer);
        return;
    }
    fp = fopen(filename, "wb");
    if (!fp)
    {
        free(p_buffer);
        return;
    }
    memmove(p_entry, p_entry + 1, file_size - ((uint8_t *)p_entry - p_buffer) - sizeof(mesh_client_seq_t));
    file_size -= sizeof(mesh_client_seq_t);
    fwrite(p_buffer, 1, file_size, fp);
    fclose(fp);
    free(p_buffer);
}

// same as uint32_to_log() but for heartbeat publication/subscription count/period - it is based on ext**(log-1)
uint8_t uint32_to_log_heartbeat(uint32_t period)
{
    uint8_t     log = 0;
    uint32_t    exp = 1;

    if (period >= 0xffff)
        return 0xff;
    if (period == 0)
        return 0;
    while (exp < period)
    {
        log++;
        exp = exp << 1;
    }
    return log + 1;
}

// Generic Default Transition Time state
#define GENDEFTRANSTIME_STEPS_MASK                  0x3F
#define GENDEFTRANSTIME_STEPS_IMMEDIATE             0x00
#define GENDEFTRANSTIME_STEPS_MAX                   0x3E
#define GENDEFTRANSTIME_STEPS_UNKNOWN               0x3F

#define GENDEFTRANSTIME_STEP_RESOLUTION_MASK        0xC0
#define GENDEFTRANSTIME_STEP_RESOLUTION_100MS       0x00
#define GENDEFTRANSTIME_STEP_RESOLUTION_1S          0x40
#define GENDEFTRANSTIME_STEP_RESOLUTION_10S         0x80
#define GENDEFTRANSTIME_STEP_RESOLUTION_10M         0xC0

uint8_t mesh_time_ms_to_transtime(uint32_t time_ms)
{
    uint8_t time_onoff;
    if (time_ms == 0xFFFFFFFF)
        time_onoff = GENDEFTRANSTIME_STEPS_UNKNOWN;
    else if (time_ms <= GENDEFTRANSTIME_STEPS_MAX * 100)
        time_onoff = (uint8_t)(time_ms / 100) | (uint8_t)GENDEFTRANSTIME_STEP_RESOLUTION_100MS;
    else if (time_ms <= GENDEFTRANSTIME_STEPS_MAX * 1000)
        time_onoff = (uint8_t)(time_ms / 1000) | (uint8_t)GENDEFTRANSTIME_STEP_RESOLUTION_1S;
    else if (time_ms <= GENDEFTRANSTIME_STEPS_MAX * 10000)
        time_onoff = (uint8_t)(time_ms / 10000) | (uint8_t)GENDEFTRANSTIME_STEP_RESOLUTION_10S;
    else
        time_onoff = (uint8_t)(time_ms / 600000) | (uint8_t)GENDEFTRANSTIME_STEP_RESOLUTION_10M;
    return time_onoff;
}
