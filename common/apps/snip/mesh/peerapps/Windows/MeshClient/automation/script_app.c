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


#include "hci_control_api.h"
#include "wiced_timer.h"
#include <rpc.pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "script_app.h"

#define WICED_BT_TRACE //ods

extern uint8_t pairing_allowed;

extern BOOLEAN bStreamingActive;

#define GATT_SCRIPT_MAX_ATTR_LEN                200         /* Maximum length of an attribute       */

#define KEY_INFO_POOL_BUFFER_SIZE               145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT              10  //Correspond's to the number of peer devices
#define SCRIPT_EIR_BUF_MAX_SIZE                 264

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/

/******************************************************************************
 *                         Variable Definitions
 ******************************************************************************/
tSCRIPT_CB  script_cb;

wiced_timer_t script_wait_timer;

extern void sendToHost(int type, UINT8 *pData, UINT32 len);

static size_t length = 0;
static UINT8 buffer[1024];
void* get_protobuf_buffer(bool bInitialize)
{
    if (bInitialize)
        memset(buffer, 0, 1024);
    return (void*)buffer;
}

size_t get_protobuf_length()
{
    return length;
}

BOOLEAN SendStreamedResponse(PROTOBUF_PARAM* parm, pb_ostream_t *postream)
{
    BOOLEAN status = false;
    if (postream->bytes_written > 0)
    {
        memcpy(parm->pRet, buffer, postream->bytes_written);
        *(parm->ret_size) = (USHORT)postream->bytes_written;
        status = true;
    }
    return status;
}

BOOLEAN SendBooleanResponse(PROTOBUF_PARAM* parm, RPC_BooleanResponse* response)
{
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    BOOLEAN status = pb_encode(&ostream, RPC_HEADER_fields, parm->header);
    if (status)
        pb_encode(&ostream, RPC_BooleanResponse_fields, response);
    return SendStreamedResponse(parm, &ostream);
}

BOOLEAN SendVoidResponse(PROTOBUF_PARAM* parm)
{
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    BOOLEAN status = pb_encode(&ostream, RPC_HEADER_fields, parm->header);
    return SendStreamedResponse(parm, &ostream);
}

extern wiced_result_t send_script_event_helper(uint8_t *p_data, uint16_t data_size);

BOOLEAN EncodeAndSendHeaderEventData(RPC_HEADER* header, const pb_field_t fields[], const void* param)
{
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    BOOL32 status = TRUE;
    if (header != NULL)
        status = pb_encode(&ostream, RPC_HEADER_fields, header);
    if (status)
{
        status = pb_encode(&ostream, fields, param);
        send_script_event_helper(buffer, (uint16_t)ostream.bytes_written);
    }
    return status;
}

BOOLEAN SendUint32Response(PROTOBUF_PARAM* parm, RPC_Uint32Response* response)
{
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    BOOLEAN status = pb_encode(&ostream, RPC_HEADER_fields, parm->header);
    if (status)
        pb_encode(&ostream, RPC_Uint32Response_fields, response);
    return SendStreamedResponse(parm, &ostream);
}

BOOLEAN SendStringResponse(PROTOBUF_PARAM* parm, char* res)
{
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    BOOLEAN status = pb_encode(&ostream, RPC_HEADER_fields, parm->header);
    char buffer[1] = { 0 };
    if (res == NULL)
        res = buffer;
    if (status)
    {
        RPC_BytesResponse response = RPC_BytesResponse_init_default;
        response.res.arg = res;
        response.res.funcs.encode = &write_string;
        pb_encode(&ostream, RPC_BytesResponse_fields, &response);
    }
    return SendStreamedResponse(parm, &ostream);
}

bool read_bd_addr(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    size_t len = 6;
    return pb_read(stream, *arg, len);
}

bool read_uuid_128(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    size_t len = 16;
    return pb_read(stream, *arg, len);
}

bool read_devclass(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    size_t len = 3;
    return pb_read(stream, *arg, len);
}

bool uuid_encode_cb(pb_ostream_t * stream, const pb_field_t *field, void * const*arg)
{
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, *arg, 16);
}

bool write_bd_addr(pb_ostream_t * stream, const pb_field_t *field, void * const*arg)
{
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, *arg, 6);
}

bool write_uuid_128(pb_ostream_t * stream, const pb_field_t *field, void * const*arg)
{
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, *arg, 16);
}

bool write_devclass(pb_ostream_t * stream, const pb_field_t *field, void * const*arg)
{
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, *arg, 3);
}

bool data_write_cb(pb_ostream_t * stream, const pb_field_t *field, void * const*arg)
{
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, *arg, GATT_SCRIPT_MAX_ATTR_LEN);
}

bool noti_indi_cb(pb_ostream_t * stream, const pb_field_t *field, void * const*arg)
{
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, *arg, GATT_SCRIPT_MAX_ATTR_LEN);
}

bool read_write_cmpl_cb(pb_ostream_t * stream, const pb_field_t *field, void * const*arg)
{
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, *arg, GATT_SCRIPT_MAX_ATTR_LEN);
}

bool read_bytes(pb_istream_t * stream, const pb_field_t *field, void **arg)
{
    BOOLEAN status;
    size_t len = stream->bytes_left;
    length = len;
    status =  pb_read(stream, *arg, len);
    *((UINT8*)*arg + len) = '\0';
    return status;
}

bool write_bytes(pb_ostream_t * stream, const pb_field_t *field, void * const* arg)
{
    PROTOBUF_ARGS const *args = (PROTOBUF_ARGS *)*arg;
    size_t len = args->data_len;

    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, args->p_data, len);
}

bool write_string(pb_ostream_t * stream, const pb_field_t *field, void * const* arg)
{
    const char* str = (const char*)*arg;
    size_t len = strlen(str);

    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, *arg, len);
}


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

 /*******************************************************************************
 * Script HCI Control handlers
 *******************************************************************************/

//extern BOOLEAN mesh_client_rpc_dispatch(PROTOBUF_PARAM *parm);
extern BOOLEAN mesh_client_rpc_dispatch(RPC_HEADER *header, pb_istream_t *stream, void* pRet, USHORT* ret_size);

/*
 * Handles the script commands
 */
wiced_result_t send_script_event_helper(uint8_t *p_data, uint16_t data_size)
{
    int retType = HCI_CONTROL_SCRIPT_EVENT_RET_CODE;
    sendToHost(retType, p_data, data_size);

    return WICED_TRUE;
}

extern void hci_control_script_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t len)
{
    pb_istream_t stream_header;
    pb_istream_t stream_body;
    USHORT retLen = 0;
    bool status;
    RPC_HEADER header = RPC_HEADER_init_zero;

    int retType = HCI_CONTROL_SCRIPT_EVENT_RET_CODE;
    // Prepare for the parameter parser and initialize return code header.
    script_cb.pParams = p_data;
    script_cb.paramsLen = len;
    script_cb.pRetP = &script_cb.retParms[0];

    stream_header = pb_istream_from_buffer(p_data, RPC_HEADER_size);
    status = pb_decode(&stream_header, RPC_HEADER_fields, &header);

    {
        stream_body = pb_istream_from_buffer(p_data+ RPC_HEADER_size, len- RPC_HEADER_size);
        switch (header.which_function_code)
        {
            case RPC_HEADER_mesh_client_function_code_tag:
                retLen = len - RPC_HEADER_size;
                status = mesh_client_rpc_dispatch(&header, &stream_body, script_cb.pRetP, &retLen);
                break;
            default:
                break;
        }
    }
    if (!status)
    {
        retType = HCI_CONTROL_SCRIPT_EVENT_UNKNOWN_CMD;
    }
    else if (retLen > 0)
    {
        script_cb.paramsLen = (UINT8)retLen;
        sendToHost(retType, script_cb.retParms, script_cb.paramsLen);
    }
}

void script_start_wait_timer (UINT32 timeout)
{
    wiced_start_timer (&script_wait_timer, timeout);
}

void script_stop_wait_timer (void)
{
    wiced_stop_timer (&script_wait_timer);
}

/* Wait timer callback handler
*/
void script_timeout_handler (uint32_t param)
{
    WICED_BT_TRACE("script_timeout_handler - TIMEOUT - subsys: 0x%02x  func: 0x%02x !!!", script_cb.retParms[0], script_cb.retParms[1]);

    *script_cb.pRetP++ = SCRIPT_WAIT_STATUS_TIMEOUT;       // First parameter of any wait return is always the status 

    sendToHost(HCI_CONTROL_SCRIPT_EVENT_RET_CODE, script_cb.retParms, script_cb.pRetP - script_cb.retParms);
}

void script_post_bt_init()
{
    wiced_init_timer (&script_wait_timer, script_timeout_handler, 0, WICED_MILLI_SECONDS_TIMER);
}

void wiced_trace_error(char *msg)
{
    WICED_BT_TRACE(msg);
}
