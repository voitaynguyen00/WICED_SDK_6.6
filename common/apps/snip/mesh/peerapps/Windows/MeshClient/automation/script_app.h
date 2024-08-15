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
 * A2DP support for HCI AV Source application
 */
#ifndef _SCRIPT_APP_H
#define _SCRIPT_APP_H

#include "bt_types.h"
#include "platform.h"
//#include "script_defs.h"
//#include "l2cscript.h"
#include "mesh_client_script.h"
#include <rpc.pb.h>


#define CASE_RETURN_STR(const) case const: return #const;

typedef unsigned short USHORT;

typedef struct  __protobuf_param {
	RPC_HEADER *header;
	pb_istream_t *stream;
	void* pRet;
	USHORT* ret_size;
} PROTOBUF_PARAM;

typedef struct __write_bytes_args {
	void* p_data;
	UINT16 data_len;
} PROTOBUF_ARGS;

extern void* get_protobuf_buffer();
extern size_t get_protobuf_length();
typedef BOOL32 tPROTOBUF_RPC_FUNC_PROXY(PROTOBUF_PARAM* parm);

extern pb_ostream_t get_protobuf_ostream(size_t len);
extern BOOLEAN EncodeAndSendHeaderEventData(RPC_HEADER* header, const pb_field_t fields[], const void* param);
//extern void SendEventData(pb_ostream_t* ostream);
extern BOOLEAN SendBooleanResponse(PROTOBUF_PARAM* parm, RPC_BooleanResponse* response);
extern BOOLEAN SendUint32Response(PROTOBUF_PARAM* parm, RPC_Uint32Response* response);
extern BOOLEAN SendVoidResponse(PROTOBUF_PARAM* parm);
extern BOOLEAN SendStringResponse(PROTOBUF_PARAM* parm, char* res);

extern bool read_bd_addr(pb_istream_t * stream, const pb_field_t *field, void **arg);
extern bool read_bytes(pb_istream_t * stream, const pb_field_t *field, void **arg);
extern bool read_devclass(pb_istream_t * stream, const pb_field_t *field, void **arg);
extern bool write_devclass(pb_ostream_t * stream, const pb_field_t *field, void **arg);

extern bool write_bd_addr(pb_ostream_t * stream, const pb_field_t *field, void * const*arg);
extern bool write_bytes(pb_ostream_t * stream, const pb_field_t *field, void * const* arg);

extern bool write_string(pb_ostream_t * stream, const pb_field_t *field, void * const* arg);
extern bool write_uuid_128(pb_ostream_t * stream, const pb_field_t *field, void * const*arg);


// The first parameter of any wait return is always the status of the wait operation
//
#define SCRIPT_WAIT_STATUS_SUCCESS             0               /* Wait completed successfully - got the expected event */
#define SCRIPT_WAIT_STATUS_TIMEOUT             1               /* Timed-out waiting for the expected event             */
#define SCRIPT_WAIT_STATUS_WRONG_EVENT         2               /* Got an unexpected event                              */
#define SCRIPT_WAIT_STATUS_PARAM_MISMATCH      3               /* Got a expected event with parameter mismatch         */


/* Script message header, included in each command and event
*/
typedef struct
{
    unsigned char   subsystem;          // Value in a command accepted event is the same as that in the command
    unsigned char   code;               // A value of 0 in any subsystem is for a 'command accepted' event
} tSCRIPT_HDR;


typedef struct
{
    UINT8                   *pParams;           // Function parameters
    UINT8                   paramsLen;

    UINT8                   *pRetP;             // Return Parameters
    UINT8                   retParms[255];      // 

    UINT8                   *pCB;               // Callback Event data pointer
    UINT8                   cbData[1000];       // Callback Event data
} tSCRIPT_CB;

extern tSCRIPT_CB  script_cb;

extern void script_start_wait_timer (UINT32 timeout);
extern void script_stop_wait_timer (void);
extern void script_send_func_return_params(void);

extern void script_post_bt_init();

/* RPC definitions
*/
typedef void (tRPC_FUNC_PROXY)(void);

extern tRPC_FUNC_PROXY *rpc_l2c_proxies[];

//extern UINT8   RPC_ExtractU8(void);
//extern BOOLEAN RPC_ExtractBool(void);
//extern UINT16  RPC_ExtractU16(void);
//extern UINT32  RPC_ExtractU32(void);
//extern void    RPC_Extract(char *format, ...);
//extern void    RPC_ExtractFixedArray(UINT8 *p_arr, int len);
//extern void    RPC_ExtractVariableArray(UINT8 *p_arr, UINT16 *p_len);
//
//extern void RPC_EncodeReturnParams(char *format, ...);
//extern void RPC_EncodeReturnParams_U8(UINT8 u8);
//extern void RPC_EncodeReturnParams_U16(UINT16 u16);
//extern void RPC_EncodeReturnParams_U32(UINT32 u32);
//extern void RPC_EncodeReturnParams_FixedArray(UINT8 *p_arr, int len);
//extern void RPC_EncodeReturnParams_VariableArray(UINT8 *p_arr, UINT16 len);
//
//extern void RPC_EncodeCbackEvt(char *format, ...);
//extern void RPC_EncodeCbackEvt_U8(UINT8 u8);
//extern void RPC_EncodeCbackEvt_U16(UINT16 u16);
//extern void RPC_EncodeCbackEvt_U32(UINT32 u32);
//extern void RPC_EncodeCbackEvt_FixedArray(UINT8 *p_arr, int len);
//extern void RPC_EncodeCbackEvt_VariableArray(UINT8 *p_arr, UINT16 len);
//
//extern void l2c_rpc_return_connected_evt (tL2C_SCRIPT_CONNECTED_PARAMS *p_cbp);
//extern void l2c_rpc_return_rx_data_evt(tL2C_SCRIPT_RX_DATA_PARAMS *p_cbp);
//extern void l2c_rpc_return_disc_ind_evt (tL2C_SCRIPT_DISC_IND_PARAMS *p_cbp);
//extern void l2c_rpc_return_disc_conf_evt (tL2C_SCRIPT_DISC_CNF_PARAMS *p_cbp);
//extern void l2c_rpc_return_ping_rsp_evt(tL2C_SCRIPT_PING_RSP_PARAMS* ping_rsp);
//extern void l2c_rpc_return_fixed_ind_evt(tL2C_SCRIPT_FIXED_CHNL_IND_PARAMS* fixed_ind);
//extern void l2c_rpc_return_rx_fixed_data_evt(tL2C_SCRIPT_RX_FIX_DATA_PARAMS* fixed_data);
//extern void l2c_rpc_return_le_conn_ind_evt(tL2C_SCRIPT_LE_CONNECT_IND_PARAMS* le_con_ind);
//extern void l2c_rpc_return_le_conn_cnf_evt(tL2C_SCRIPT_LE_CONNECT_CNF_PARAMS* le_con_cnf);
//extern void l2c_rpc_return_le_disc_ind_evt(tL2C_SCRIPT_LE_DISCONNECT_IND_PARAMS* le_disc_ind);
//extern void l2c_rpc_return_le_disc_cnf_evt(tL2C_SCRIPT_LE_DISCONNECT_CNF_PARAMS* le_disc_cnf);
//extern void l2c_rpc_return_le_conf_test1_evt(tL2C_SCRIPT_LETPLECFCBV01C_PARAMS* le_conf_test1);
//extern void l2c_rpc_return_le_conf_test6_evt(tL2C_SCRIPT_LETPLECFCBV06C_PARAMS* le_conf_test6);

extern void mesh_client_rpc_return_event(tMESH_CLIENT_SCRIPT_EVENT_PARAMS *p_evt);
#endif  /* _SCRIPT_APP_H */
