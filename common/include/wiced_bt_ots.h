/***************************************************************************//**
* \file <wiced_bt_otp.h_>
*
* \brief
* 	Contains Object Transfer Service - Object Transfer Service APIs and definitions.
*
*//*****************************************************************************
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
*******************************************************************************/
#ifndef WICED_BT_OTS_H
#define WICED_BT_OTS_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
*          Type Definitions
******************************************************************************/
/* GATT handles used by Object Server application and library */

/* Primary Service - Object Transfer Service */
#define HDLS_OBJECT_TRANSFER                                    0x0028

/* Characteristic - OTS Feature */
#define HDLC_OTS_FEATURE                                        0x0029
#define HDLC_OTS_FEATURE_VALUE                                  0x0030

/* Characteristic - Object Name */
#define HDLC_OBJECT_NAME                                        0x0031
#define HDLC_OBJECT_NAME_VALUE                                  0x0032

/* Characteristic - Object Type */
#define HDLC_OBJECT_TYPE                                        0x0033
#define HDLC_OBJECT_TYPE_VALUE                                  0x0034

/* Characteristic - Object Type */
#define HDLC_OBJECT_SIZE                                        0x0035
#define HDLC_OBJECT_SIZE_VALUE                                  0x0036

/* Characteristic - Object First-Created */
#define HDLC_OBJECT_FIRST_CREATED                               0x0037
#define HDLC_OBJECT_FIRST_CREATED_VALUE                         0x0038

/* Characteristic - Object Last-Modified */
#define HDLC_OBJECT_LAST_MODIFIED                               0x0039
#define HDLC_OBJECT_LAST_MODIFIED_VALUE                         0x003a

/* Characteristic - Object ID */
#define HDLC_OBJECT_ID                                          0x003b
#define HDLC_OBJECT_ID_VALUE                                    0x003c

/* Characteristic - Object Properties */
#define HDLC_OBJECT_PROPERTIES                                  0x003d
#define HDLC_OBJECT_PROPERTIES_VALUE                            0x003e

/* Characteristic - OACP */
#define HDLC_OACP                                               0x003f
#define HDLC_OACP_VALUE                                         0x0040
#define HDLC_OACP_CLIENT_CONFIG_DESCRIPTOR                      0x0041

/* Characteristic - OLCP */
#define HDLC_OLCP                                               0x0042
#define HDLC_OLCP_VALUE                                         0x0043
#define HDLC_OLCP_CLIENT_CONFIG_DESCRIPTOR                      0x0044

/* Object Server shall support three instances of the object filter */
/* Characteristic - Object List Filter */
#define HDLC_OBJECT_LIST_FILTER_INSTANCE_1                      0x0045
#define HDLC_OBJECT_LIST_FILTER_INSTANCE_1_VALUE                0x0046

#define HDLC_OBJECT_LIST_FILTER_INSTANCE_2                      0x0047
#define HDLC_OBJECT_LIST_FILTER_INSTANCE_2_VALUE                0x0048

#define HDLC_OBJECT_LIST_FILTER_INSTANCE_3                      0x0049
#define HDLC_OBJECT_LIST_FILTER_INSTANCE_3_VALUE                0x004a

/* Characteristic - Object Changed */
#define HDLC_OBJECT_CHANGED                                     0x004b
#define HDLC_OBJECT_CHANGED_CLIENT_CONFIG_DESCRIPTOR            0x004c

/******************************************************************************
*
* \name wiced_bt_ots_oacp_resp_result_t
*
* \details This callback is invoked when Object Server receives any OACP procedure
*          requests from the Object Client.
*
* @param[in]   procedure        : OACP Procedure
* @param[in]   p_procedure_data : OACP Procedure data
* @param[out]  p_resp           : OACP response value updated by the application if applicable
*
* @return wiced_bt_ots_oacp_resp_result_t : OACP response result code
*
******************************************************************************/
typedef wiced_bt_ots_oacp_resp_result_t (*wiced_bt_ots_oacp_callback_t)(wiced_bt_ots_oacp_procedure_t procedure, void* p_procedure_data, wiced_bt_ots_oacp_response_t* p_resp );

/* Object Configurations to be specified by the application on initialization */
typedef struct
{
    uint32_t                      num_objects;          /* Number of objects that the server should support */
    ots_feature_characteristic_t  feature_support;      /* OTS Features that the server should support */
    uint32_t                      object_properties;    /* Default properties of objects created in the server */
    uint16_t                      mtu;                  /* MTU supported for data transfer over object transfer channel.L2CAP minimum MTU is 48 octets */
}wiced_bt_object_config_t;

/* Callbacks registered by the application */
typedef struct
{
    wiced_bt_ots_oacp_callback_t    p_ots_oacp_cback;   /* OACP procedure callback */
    wiced_bt_otp_channel_callback_t p_otp_channel_cback;/* Object Transfer Channel event callback */
}wiced_bt_ots_callback_t;

/******************************************************************************
*          Function Prototypes
******************************************************************************/

/******************************************************************************
*
* \name wiced_bt_ots_init
*
* \details This API is used by the higher layer application to initialize the
*          Object Server and specify the object configurations that the Server should support.
*
* @param[in]  p_object_cfg : Object configurations
* @param[in]  p_cback      : Callbacks registered by the application to
*                            receive the events from the Object Server
*
* @return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_ots_init( wiced_bt_object_config_t*  p_object_cfg, wiced_bt_ots_callback_t* p_cback );

/******************************************************************************
*
* \name wiced_bt_ots_create_object
*
* \details This API is used by the higher layer application to create an object
*          in the object server.
*
* @param[in]  create       : Create procedure parameters
*
* @return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_ots_create_object( ots_oacp_create_procedure_data_t* create );

/******************************************************************************
*
* \name wiced_bt_ots_write_object_metadata
*
* \details This API is used by the higher layer application to write the metadata
*          characteristics of the object.
*
* @param[in]  meta_data_characteristic : Object metadata characteristic to be written
* @param[in]  p_data                   : Characteristic value
*
* @return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_ots_write_object_metadata( uint8_t meta_data_characteristic, void* p_data );

/******************************************************************************
*
* \name wiced_bt_ots_connection_up
*
* \details This API is called by the higher layer application when any client is connected.
*
* @param[in]  conn_id : GATT connection ID
*
* @return  None.
*
******************************************************************************/
void wiced_bt_ots_connection_up( uint16_t conn_id );

/******************************************************************************
*
* \name wiced_bt_ots_connection_down
*
* \details This API is called by the higher layer application when the client is disconnected.
*
* @param[in]  conn_id : GATT connection ID
*
* @return  None.
*
******************************************************************************/
void wiced_bt_ots_connection_down( uint16_t conn_id );

/******************************************************************************
*
* \name wiced_bt_ots_process_gatt_read_req
*
* \details This API is called by the application to process the GATT read requests from the
*          Object Client to read the OTS service characteristics.
*
* @param[in]  conn_id : GATT connection ID
* @param[in]  p_read  : GATT read request.
*
* @return  Status of the GATT read operation.
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_ots_process_gatt_read_req( uint16_t conn_id, wiced_bt_gatt_read_t *p_read );

/******************************************************************************
*
* \name wiced_bt_ots_process_gatt_write_req
*
* \details This API is called by the application to process the GATT write requests from the
*          Object Client to write to the OTS service characteristics.
*
* @param[in]  conn_id : GATT connection ID
* @param[in]  p_write : GATT write request.
*
* @return  Status of the GATT write operation.
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_ots_process_gatt_write_req( uint16_t conn_id, wiced_bt_gatt_write_t *p_write );

#ifdef __cplusplus
}
#endif

#endif /* _WICED_BT_OTS_H_ */
