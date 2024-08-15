/*
 * Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *  Connection functions
 *
 *  Provides connection methods for use by applications
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_internal.h"
#include "amqp_frame.h"
#include "amqp_link.h"
#include "amqp_manager.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static wiced_result_t get_element_source_target(wiced_amqp_source_target_t*,wiced_bool_t*,wiced_amqp_buffer_t*);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/


wiced_result_t wiced_amqp_detach( wiced_amqp_link_instance *link_instance)
{
    wiced_amqp_delink_instance args;

    memset( &args, 0, sizeof(wiced_amqp_delink_instance) );

    args.detach_args.closed = WICED_TRUE;
    args.detach_args.handle = link_instance->link_endpoint->output_handle;
    args.link = link_instance;
    return amqp_manager( WICED_AMQP_EVENT_DETACH_SENT, &args, 0, link_instance->session->connection_instance );
}

wiced_result_t wiced_amqp_attach( wiced_amqp_link_instance* link_instance, wiced_amqp_session_instance_t* session_instance )
{
    wiced_result_t result = WICED_ERROR;
    amqp_link_endpoints *link_endpoint;

    link_instance->link_args.max_message_size = 1024;
    link_instance->link_args.initial_delivery_count = 0;

    /* create new link endpoint */
    result = amqp_get_buffer( (void**) &link_endpoint, sizeof(amqp_link_endpoints) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_DEBUG(("memory not available to create link endpoint"));
        return WICED_ERROR;
    }

    memset( link_endpoint, 0, sizeof(amqp_link_endpoints) );

    link_endpoint->output_handle = link_instance->link_args.handle;
    session_instance->link_endpoint_count++ ;

    link_endpoint->name = (uint8_t*) link_instance->link_args.name;
    result = linked_list_insert_node_at_rear( &session_instance->link_endpoint, &link_endpoint->this_node );

    /* initialize message delivery list */
    linked_list_init( &link_instance->message_delivery_list );

    link_instance->link_endpoint = link_endpoint;
    link_endpoint->context = (wiced_amqp_link_instance*) link_instance;

    return amqp_manager( WICED_AMQP_EVENT_ATTACH_SENT, link_instance, 0, session_instance->connection_instance );
}

/******************************************************
 *               Protocol Header Frames
 ******************************************************/

wiced_result_t amqp_frame_put_link( wiced_amqp_frame_t *frame,  void* params )
{
    uint8_t count = AMQP_ATTACH_FRAME_LIST_COUNT;
    wiced_amqp_link_instance *args = params;
    uint8_t* temp1 = frame->buffer.data;

    amqp_fixed_frame fixed_frame;

    fixed_frame.channel = args->session->endpoints->outgoing_channel;
    fixed_frame.doff = 0x02;
    fixed_frame.performative_type = AMQP_ATTACH;
    fixed_frame.size = 0;
    fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    fixed_frame.count = count;

    put_fixed_frame( frame, &fixed_frame );

    /* Encode container id */
    if ( args->link_args.name == NULL )
    {
        put_element_bool( 0, WICED_FALSE, &frame->buffer );

    }
    else
    {
        /* As it is container id string-8 type  code : 0xal*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, VARIABLE_TYPE_ONE_BYTE_UNICODE_STRING );
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->link_args.name, strlen( (char* ) args->link_args.name ) );
    }
    /* Encode handle handle as it is uint code : FIXED_TYPE_FOUR_BYTE_UINT */
    put_element_value( args->link_args.handle, WICED_TRUE, &frame->buffer );

    /* Encode role as it is based on boolean sender/recevier (FALSE/TRUE) */
    if ( args->link_args.role == WICED_AMQP_ROLE_RECEIVER )
    {
        put_element_bool( WICED_AMQP_ROLE_RECEIVER, WICED_TRUE, &frame->buffer );
    }
    else if ( args->link_args.role == WICED_AMQP_ROLE_SENDER )
    {
        put_element_bool( WICED_AMQP_ROLE_SENDER, WICED_TRUE, &frame->buffer );

    }
    else
    {
        WPRINT_LIB_DEBUG ((" Invalid value for role of link \r\n"));
    }

    /* Encode snd_settle_mode as it is ubyte code : 0x50 */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x50 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->link_args.snd_settle_mode );

    /* Encode recv_settle_mode as it is ubyte code : 0x50 */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x50 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->link_args.rcv_settle_mode );

    /* Encode source as it is list */
    //TODO: currently handling only first element; need to handle all elements in the list
    if ( args->link_args.source.address == NULL )
    {
        uint32_t temp_size = 0;

        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 ); /*  2  descriptor   */

        AMQP_BUFFER_PUT_OCTET( &frame->buffer, ELEMENT_SOURCE ); /*  1  performative */

        /* To represent list code : COMPOUND_TYPE_ONE_BYTE_LIST */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, COMPOUND_TYPE_ONE_BYTE_LIST );
        /* to represent size of list */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, temp_size + 1 );
        /* to represent count */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );
        /* to represent NULL */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, NULL_BYTE );
    }
    else
    {
        uint32_t temp_size = 0;

        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 ); /*  2  descriptor   */

        AMQP_BUFFER_PUT_OCTET( &frame->buffer, ELEMENT_SOURCE ); /*  1  performative */
        /* To represent list code : COMPOUND_TYPE_ONE_BYTE_LIST */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, COMPOUND_TYPE_ONE_BYTE_LIST );
        /* to represent size of list */
        temp_size++;
        temp_size = temp_size + strlen( (char*) args->link_args.source.address );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, temp_size + 1 );
        /* to represent count */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );
        /* As it is name string-8 type  code : 0xal*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, VARIABLE_TYPE_ONE_BYTE_UNICODE_STRING );
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->link_args.source.address, strlen( (char* )args->link_args.source.address ) );
    }

    /* Encode target as it is list */
    //TODO: currently handling only first element; need to handle all elements in the list
    if ( args->link_args.target.address == NULL )
    {
        uint32_t temp_size = 0;

        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 ); /*  2  descriptor   */

        AMQP_BUFFER_PUT_OCTET( &frame->buffer, ELEMENT_TARGET ); /*  1  performative */

        /* To represent list code : COMPOUND_TYPE_ONE_BYTE_LIST */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, COMPOUND_TYPE_ONE_BYTE_LIST );
        /* to represent size of list */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, temp_size + 1 );
        /* to represent count */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );
        /* to represent NULL */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, NULL_BYTE );
    }
    else
    {
        uint32_t temp_size = 0;

        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 ); /*  2  descriptor   */

        AMQP_BUFFER_PUT_OCTET( &frame->buffer, ELEMENT_TARGET ); /*  1  performative */

        /* To represent list code : COMPOUND_TYPE_ONE_BYTE_LIST */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, COMPOUND_TYPE_ONE_BYTE_LIST );
        /* to represent size of list */
        temp_size++;
        temp_size = temp_size + strlen( (char*) args->link_args.target.address );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, temp_size + 1 );
        /* to represent count */AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );
        /* As it is name string-8 type  code : 0xal*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, VARIABLE_TYPE_ONE_BYTE_UNICODE_STRING );
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->link_args.target.address, strlen( (char* )args->link_args.target.address ) );
    }

    /* Encode unsettled currently not handled so always NULL */
    //TODO : As we are ignoring so always NULL
    put_element_bool( 0, WICED_FALSE, &frame->buffer );

    /* Encode incoming-unsettled currently not handled so always NULL */
    //TODO : As we are ignoring so always NULL
    put_element_bool( 0, WICED_FALSE, &frame->buffer );

    /* Encode initial-delivery-count as it is uint code : FIXED_TYPE_FOUR_BYTE_UINT */
    //TODO : As we are ignoring so always NULL
    if ( args->link_args.role == WICED_AMQP_ROLE_SENDER )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, FIXED_TYPE_FOUR_BYTE_UINT );
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->link_args.initial_delivery_count );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, FIXED_TYPE_EMPTY_BYTE_UINT );
    }

    /* Encode max-message-size as it is ulong code : 0x80 */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x80 );
    AMQP_BUFFER_PUT_LONG_LONG( &frame->buffer, args->link_args.max_message_size );

    /*calculate the size of link frame*/
    calculate_update_size_fields( temp1, temp1 + 12, frame );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_attach_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint32_t temp = 0;
    uint32_t count = 0;
    uint8_t temp_variable = 0;
    wiced_bool_t temp_bool;

    get_fixed_frame( frame, &args->fixed_frame );

    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    if ( temp == COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp ); /*size*/

        AMQP_BUFFER_GET_OCTET( &frame->buffer, count ); /*count*/
    }
    else if ( temp == COMPOUND_TYPE_FOUR_BYTE_LIST )
    {
        AMQP_BUFFER_GET_LONG( &frame->buffer, temp ); /*size*/

        AMQP_BUFFER_GET_LONG( &frame->buffer, count ); /*count*/
    }

    temp_variable = 1;
    do
    {
        switch ( temp_variable )
        {
            case 1:
                get_element_pointer( (uint8_t**) &args->args.link_args.name, (uint32_t*) &args->args.link_args.name_length, &frame->buffer );
                break;
            case 2:
            {
                uint64_t temp_handle = 0;
                get_element_value( &temp_handle, &temp_bool, &frame->buffer );
                args->args.link_args.handle = (uint32_t) temp_handle;
                break;
            }
            case 3:
                get_element_bool( (wiced_bool_t*) &args->args.link_args.role, &temp_bool, &frame->buffer );
                break;
            case 4:
            {
                uint64_t temp_mode = 0;
                get_element_value( &temp_mode, &temp_bool, &frame->buffer );
                args->args.link_args.snd_settle_mode = (uint8_t) temp_mode;
                break;
            }
            case 5:
            {
                uint64_t temp_mode = 0;
                get_element_value( &temp_mode, &temp_bool, &frame->buffer );
                args->args.link_args.rcv_settle_mode = (uint8_t) temp_mode;
                break;
            }
            case 6:
                get_element_source_target( &args->args.link_args.source, &args->args.link_args.source_included, &frame->buffer );
                break;
            case 7:
                get_element_source_target( &args->args.link_args.target, &args->args.link_args.target_included, &frame->buffer );
                break;
            case 8:
                AMQP_BUFFER_GET_OCTET( &frame->buffer, temp ); /*unsettled*/
                break;
            case 9:
                AMQP_BUFFER_GET_OCTET( &frame->buffer, temp ); /*incompete-unsettled*/
                break;
            case 10:
            {
                uint64_t temp_delivery_count = 0;
                get_element_value( &temp_delivery_count, &temp_bool, &frame->buffer );
                args->args.link_args.initial_delivery_count = (uint32_t) temp_delivery_count;
                break;
            }
            case 11:
                get_element_value( &args->args.link_args.max_message_size, &args->args.link_args.max_message_size_included, &frame->buffer );
                break;
            default:
                break;
        }
        temp_variable++;
    } while ( temp_variable <= count );

    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_get_attach_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret = WICED_SUCCESS;

     ret = amqp_manager( WICED_AMQP_EVENT_ATTACH_RCVD, args, 0, conn_instance );

    //TODO : Need to use all the args what we have received
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_ATTACH_RCVD, args, conn_instance );
        }
    }
    return ret;

}

wiced_result_t amqp_connection_backend_put_attach_performative( wiced_amqp_link_instance* link_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send attach packet */
    ret = amqp_frame_create( WICED_AMQP_EVENT_ATTACH_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &link_instance->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_link( &frame, link_instance );
    return amqp_frame_send( &frame, &link_instance->session->connection_instance->conn->socket );
}

wiced_result_t amqp_connection_backend_get_detach_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret = WICED_SUCCESS;

    ret = amqp_manager( WICED_AMQP_EVENT_DETACH_RCVD, args, 0, conn_instance );

    //TODO : Need to use all the args what we have received
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_DETACH_RCVD, args, conn_instance );
        }
    }
    return ret;

}

wiced_result_t amqp_connection_backend_put_detach_performative( wiced_amqp_delink_instance* delink_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_DETACH_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &delink_instance->link->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_detach_performative( &frame, delink_instance );
    return amqp_frame_send( &frame, &delink_instance->link->session->connection_instance->conn->socket );
}

wiced_result_t amqp_frame_get_detach_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint32_t temp = 0;
    uint32_t count = 0;

    get_fixed_frame( frame, &args->fixed_frame );


    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    if ( (temp & UPPER_NIBBLE)== COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        AMQP_BUFFER_GET_OCTET( &frame->buffer, count );
    }
    if ( (temp & UPPER_NIBBLE)== COMPOUND_TYPE_FOUR_BYTE_LIST )
    {
        AMQP_BUFFER_GET_LONG( &frame->buffer, temp );

        AMQP_BUFFER_GET_LONG( &frame->buffer, count );
    }

    /*parse handle*/
    if ( count != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        count--;
        if ( temp == NULL_BYTE )
        {
            args->args.detach_args.handle_is_not_specified = 1;
        }
        else if ( temp == FIXED_TYPE_EMPTY_BYTE_UINT )
        {
            args->args.detach_args.handle = 0;
        }
        else if(temp == 0x52)
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.detach_args.handle );
        }
        else if ( temp == 0x60 )
        {
            AMQP_BUFFER_GET_SHORT( &frame->buffer, args->args.detach_args.handle );
        }
        else if ( temp == FIXED_TYPE_FOUR_BYTE_UINT )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.detach_args.handle );
        }

    }

    /*parse closed*/
    if ( count != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x41 )
        {
            args->args.detach_args.closed = WICED_TRUE;
        }
        else if ( temp == 0x42 )
        {
            args->args.detach_args.closed = WICED_FALSE;
        }
        else
        {

        }
    }

    /*parse error*/
    if ( count != 0)
    {
        AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        if ( temp == ELEMENT_ERROR)
        {
            get_element_error(&args->args.detach_args.error,&frame->buffer);
        }
    }
    return WICED_SUCCESS;
}
wiced_result_t amqp_frame_put_detach_performative( wiced_amqp_frame_t *frame,  void* arg )
{

    uint8_t count = AMQP_DETACH_FRAME_LIST_COUNT;
    wiced_amqp_delink_instance *delink_instance = arg;
    uint8_t* temp1 = frame->buffer.data;

    amqp_fixed_frame fixed_frame;

    fixed_frame.channel = delink_instance->link->session->endpoints->outgoing_channel;
    fixed_frame.doff = 0x02;
    fixed_frame.performative_type = AMQP_DETACH;
    fixed_frame.size = 0;
    fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    fixed_frame.count = count;

    put_fixed_frame( frame, &fixed_frame );

    /* encode handle */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, FIXED_TYPE_FOUR_BYTE_UINT );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, delink_instance->detach_args.handle );

    /* Encode closed filed */
    if ( delink_instance->detach_args.closed == WICED_TRUE )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x41 );

    }
    else if ( delink_instance->detach_args.closed == WICED_FALSE )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x42 );

    }
    else
    {

    }

    /*error should be NULL*/
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, NULL_BYTE );

    /*calculate the size of deatach frame*/
    calculate_update_size_fields( temp1, temp1 + 12, frame );

    return WICED_SUCCESS;
}

wiced_result_t get_element_source_target(wiced_amqp_source_target_t *value,wiced_bool_t * included, wiced_amqp_buffer_t* pointer)
{
    uint64_t temp;
    uint32_t count=0;
    uint8_t temp_variable;
    *included=WICED_TRUE;
    AMQP_BUFFER_GET_OCTET( pointer, temp );
    if ( temp == NULL_BYTE )
    {
        *included=WICED_FALSE;
    }
    else if(temp==0x00)
    {
        AMQP_BUFFER_GET_OCTET( pointer, temp );
        AMQP_BUFFER_GET_OCTET( pointer, temp );

        if((temp==ELEMENT_TARGET) || (temp==ELEMENT_SOURCE))   /*target field*/
        {
            AMQP_BUFFER_GET_OCTET( pointer, temp );
            if((temp & UPPER_NIBBLE)==COMPOUND_TYPE_ONE_BYTE_LIST)
            {
                 AMQP_BUFFER_GET_OCTET( pointer, temp );  /*size*/
                 AMQP_BUFFER_GET_OCTET(pointer, count );    /*count*/
                 temp_variable=1;
                 do
                 {
                     switch(temp_variable)
                     {
                        case 1:
                            get_element_pointer((uint8_t**)&value->address,(uint32_t*)&value->address_length,pointer);
                            break;
                        case 2:
                        {
                            uint64_t temp_durable=0;
                            get_element_value(&temp_durable,&value->durable_included,pointer);
                            value->durable=(uint32_t)temp_durable;
                            break;
                        }
                        case 3:
                            get_element_pointer(&value->expiry_policy,(uint32_t*)&value->expiry_policy_length,pointer);
                            break;
                        case 4:
                        {
                            uint64_t temp_timeout=0;
                            get_element_value(&temp_timeout,&value->timeout_included,pointer);
                            value->timeout=(uint32_t)temp_timeout;
                            break;
                        }
                        case 5:
                            get_element_bool(&value->dynamic,&value->dynamic_included,pointer);
                            break;
                        default:
                            AMQP_BUFFER_GET_OCTET( pointer, temp );  /*skipping rest of the elements*/
                            break;
                     }
                     temp_variable++;
                 }while(temp_variable<=count);
            }
        }
    }

    return WICED_SUCCESS;
}
