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
#include "amqp_frame.h"
#include "amqp_transfer.h"
#include "amqp_manager.h"
#include "amqp.h"
#include "wiced_tls.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define TCP_SERVER_THREAD_PRIORITY          (WICED_DEFAULT_LIBRARY_PRIORITY)
#define TCP_SERVER_STACK_SIZE               (6200)
#define TCP_CLIENT_PORT                     (50007)
#define TCP_CLIENT_INTERVAL                 (2)
#define TCP_CLIENT_CONNECT_TIMEOUT          (500)
#define TCP_CONNECTION_NUMBER_OF_RETRIES    (3)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    wiced_packet_t* packet;
    void* data;
} wiced_amqp_network_message_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_amqp_send( wiced_amqp_link_instance* link_instance, wiced_amqp_message_t* application_message )
{
    wiced_result_t result;
    amqp_delivery_t* delivery;
    amqp_transfer_t transfer_instance;

    if ( link_instance == NULL || link_instance->link_state != AMQP_LINK_STATE_ATTACHED )
    {
        return WICED_ERROR;
    }

    /* make sure we have enough link credit from peer before sending data */
    if ( link_instance->link_credit == 0 )
    {
        return WICED_ERROR;
    }

    /* delivery ID should be unique for particular session */
    link_instance->session->begin_args.next_outgoing_id++;
    /* use delivery count as delivery tag so which can be unique */
    link_instance->delivery_count += 1;
    /* decrement remote session window */
    link_instance->session->begin_args.remote_incoming_window--;

    if ( application_message->settle == WICED_FALSE )
    {

        /* create delivery instance and keep it in link as pending deliveries till we dont get Disposition [ acknowledgement ] */
        result = amqp_get_buffer( (void**) &delivery, sizeof(amqp_delivery_t) );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_LIB_DEBUG(("memory not available to create link endpoint"));
            return WICED_ERROR;
        }

        delivery->delivery_id = link_instance->session->begin_args.next_outgoing_id;
        delivery->link = link_instance;
        delivery->delivery_tag = application_message->delivery_tag;
        /* add delivery information to delivery list which will be used when we get disposition */
        result = linked_list_insert_node_at_rear( &link_instance->message_delivery_list, &delivery->this_node );
    }

    /* Fill parameters needed to send transfer packet */
    transfer_instance.delivery_id = link_instance->session->begin_args.next_outgoing_id;
    transfer_instance.handle = link_instance->link_endpoint->output_handle;
    transfer_instance.message = application_message;
    transfer_instance.link = link_instance;

    /* decrement the link credit */
    link_instance->link_credit--;

    return amqp_manager( WICED_AMQP_EVENT_TRANSFER_SENT, &transfer_instance, 0, link_instance->session->connection_instance );
}

wiced_result_t amqp_connection_backend_put_transfer_performative( amqp_transfer_t* transfer_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_TRANSFER_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &transfer_instance->link->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_transfer( &frame, transfer_instance );
    return amqp_frame_send( &frame, &transfer_instance->link->session->connection_instance->conn->socket );
}

wiced_result_t amqp_connection_backend_put_disposition_performative( amqp_disposition_t* disposition_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_DISPOSITION_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &disposition_instance->link->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_disposition( &frame, disposition_instance );
    return amqp_frame_send( &frame, &disposition_instance->link->session->connection_instance->conn->socket );
}

wiced_result_t amqp_frame_put_transfer( wiced_amqp_frame_t *frame, void* arg )
{
    uint8_t count = AMQP_TRANSFER_FRAME_LIST_COUNT;
    amqp_transfer_t *params = arg;
    uint8_t* temp1 = frame->buffer.data, *temp2 = NULL, *temp3 = NULL;

    uint32_t total_size = 8;
    uint32_t size = 0;
    amqp_fixed_frame fixed_frame;

    fixed_frame.channel = params->link->session->endpoints->outgoing_channel;
    fixed_frame.doff = 0x02;
    fixed_frame.performative_type = AMQP_TRANSFER;
    fixed_frame.size = 0;
    fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    fixed_frame.count = count;

    put_fixed_frame( frame, &fixed_frame );

    temp2 = temp1 + 12;

    /* encoding handle */
    put_element_value( (uint64_t) params->handle, WICED_TRUE, &frame->buffer );

    /* encoding delivery-id */
    put_element_value( (uint64_t) params->delivery_id, WICED_TRUE, &frame->buffer );

    /* encoding delivery-tag */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xa0 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x04 );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, params->message->delivery_tag );

    /* encoding message-format */
    put_element_value( 0, WICED_TRUE, &frame->buffer );

    /* Encoding settle flag */
    put_element_bool( params->message->settle, params->message->settle_included, &frame->buffer );

    /* Encoding more flag */
    put_element_bool( params->message->more, WICED_TRUE, &frame->buffer );

    /*receive settle mode*/
    put_element_value( params->message->receiver_settle_mode, params->message->receiver_settle_mode_included, &frame->buffer );

    /*state*/
    put_element_value( 0, WICED_FALSE, &frame->buffer );

    /*resume*/
    put_element_value( 0, WICED_FALSE, &frame->buffer );

    /*abort*/
    put_element_value( params->message->abort, params->message->abort_inlcuded, &frame->buffer );

    /*batchable*/
    put_element_value( 0, WICED_FALSE, &frame->buffer );

    /*calculate the size of transfer frame*/
    size = (uint32_t) ( frame->buffer.data - temp2 - 1 );
    *temp2 = (uint8_t) size; /*update the size in size field*/
    total_size += size + 3 + 2;

    /********add header section**********/
    if ( params->message->header.included )
    {
        temp2 = frame->buffer.data;
        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, HEADER_SECTION ); /*properties*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 );
        temp3 = frame->buffer.data;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );/*size*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x05 ); /*5 elements*/

        /*durable*/
        put_element_bool( params->message->header.durable, params->message->header.durable_included, &frame->buffer );
        /*priority*/
        put_element_value( params->message->header.priority, params->message->header.priority_included, &frame->buffer );
        /*ttl*/
        put_element_value( params->message->header.ttl, params->message->header.ttl_included, &frame->buffer );
        /*first acquirer*/
        put_element_bool( params->message->header.first_acquirer, params->message->header.first_acquirer_included, &frame->buffer );
        /*delivery count*/
        put_element_value( params->message->header.delivery_count, params->message->header.delivery_count_included, &frame->buffer );

        /*calculate the size of properties section*/
        size = (uint32_t) ( frame->buffer.data - temp3 - 1 );
        *temp3 = (uint8_t) size; /*update the size in size field*/
        total_size += (uint32_t) ( frame->buffer.data - temp2 );
    }

    /**********add properties section**********/
    if ( params->message->properties.included )
    {
        temp2 = frame->buffer.data;
        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, PROPERTIES_SECTION ); /*properties*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 );
        temp3 = frame->buffer.data;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );/*size*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x0d ); /*13 elements*/

        /*message-id*/
        put_element_pointer( params->message->properties.message_id, params->message->properties.message_id_length, &frame->buffer );

        /*user-id*/
        put_element_pointer( params->message->properties.user_id, params->message->properties.user_id_length, &frame->buffer );

        /*to*/
        put_element_pointer( params->message->properties.to, params->message->properties.to_length, &frame->buffer );

        /*subject*/
        put_element_pointer( params->message->properties.subject, params->message->properties.subject_length, &frame->buffer );

        /*reply-to*/
        put_element_pointer( params->message->properties.reply_to, params->message->properties.reply_to_length, &frame->buffer );

        /*correlation-id*/
        put_element_pointer( params->message->properties.correlation_id, params->message->properties.correlation_id_length, &frame->buffer );

        /*content-type*/
        put_element_pointer( params->message->properties.content_type, params->message->properties.content_type_length, &frame->buffer );

        /*content-encoding*/
        put_element_pointer( params->message->properties.content_encoding, params->message->properties.content_encoding_length, &frame->buffer );

        /*absolute-expiry-time*/
        put_element_value( params->message->properties.absolute_expiry_time, params->message->properties.absolute_expiry_time_included, &frame->buffer );

        /*creation-time*/
        put_element_value( (uint64_t) params->message->properties.creation_time, params->message->properties.creation_time_included, &frame->buffer );

        /*group-id*/
        put_element_pointer( params->message->properties.group_id, params->message->properties.group_id_length, &frame->buffer );

        /*group-sequence*/
        put_element_value( params->message->properties.group_sequence, params->message->properties.group_sequence_included, &frame->buffer );

        /*reply-to-group-id*/
        put_element_pointer( params->message->properties.reply_to_group_id, params->message->properties.reply_to_group_id_length, &frame->buffer );

        /*calculate the size of properties section*/
        size = (uint32_t) ( frame->buffer.data - temp3 - 1 );
        *temp3 = (uint8_t) size; /*update the size in size field*/
        total_size += (uint32_t) ( frame->buffer.data - temp2 );
    }

    /******Add application properties section*******/
    if ( params->message->application_properties.included )
    {
        temp2 = frame->buffer.data;
        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, APPLICATION_PROPERTIES_SECTION ); /*properties*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc1 );
        temp3 = frame->buffer.data;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );/*size*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, ( params->message->application_properties.count * 2 ) );

        /*insert key value pairs*/
        for ( uint8_t i = 0; i < params->message->application_properties.count; i++ )
        {
            put_element_pointer( params->message->application_properties.properties[ i ].key, params->message->application_properties.properties[ i ].key_length, &frame->buffer );
            put_element_pointer( params->message->application_properties.properties[ i ].value, params->message->application_properties.properties[ i ].value_length, &frame->buffer );
        }
        size = (uint32_t) ( frame->buffer.data - temp3 - 1 );
        *temp3 = (uint8_t) size; /*update the size in size field*/
        total_size += (uint32_t) ( frame->buffer.data - temp2 );
    }

    /*****add Data section*******/

    if ( params->message->data_lenth )
    {
        temp2 = frame->buffer.data;
        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, DATA_SECTION );
        put_element_pointer( params->message->data, params->message->data_lenth, &frame->buffer );
        size = (uint32_t) ( frame->buffer.data - temp2 );
        total_size += size;
    }

    /*update total frame size*/
    *temp1 = (uint8_t) ( total_size >> 24 );
    temp1++;
    *temp1 = (uint8_t) ( total_size >> 16 );
    temp1++;
    *temp1 = (uint8_t) ( total_size >> 8 );
    temp1++;
    *temp1 = (uint8_t) ( total_size );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_disposition( wiced_amqp_frame_t *frame, void* arg )
{
    uint8_t count = AMQP_DISPOSTION_FRAME_LIST_COUNT;
    amqp_disposition_t *params = arg;
    uint8_t* temp1 = frame->buffer.data;

    amqp_fixed_frame fixed_frame;

    fixed_frame.channel = 0x00;
    fixed_frame.doff = 0x02;
    fixed_frame.performative_type = AMQP_DISPOSITON;
    fixed_frame.size = 0;
    fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    fixed_frame.count = count;

    put_fixed_frame( frame, &fixed_frame );

    /* encoding Role */
    put_element_bool( params->link->link_args.role, WICED_TRUE, &frame->buffer );

    /* encoding first */
    put_element_value( params->first, WICED_TRUE, &frame->buffer );

    /* encoding last */
    put_element_value( params->last, WICED_TRUE, &frame->buffer );

    /* encoding settled */
    put_element_bool( params->settled, WICED_TRUE, &frame->buffer );

    /*state*/
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x53 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x24 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x45 );

    /*calculate the size of disposition frame*/
    calculate_update_size_fields( temp1, temp1 + 12, frame );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_disposition_performative( wiced_amqp_frame_t *frame, void *arg )
{

    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint32_t temp = 0;
    uint32_t count = 0;
    uint8_t temp_variable = 0;

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
                AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                if ( temp == BOOLEAN_TYPE_TRUE )
                {
                    args->args.disposition_args.is_receiver = WICED_TRUE;
                }
                if ( temp == BOOLEAN_TYPE_FALSE )
                {
                    args->args.disposition_args.is_receiver = WICED_FALSE;
                }
                break;
            case 2:
                AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

                if ( temp == FIXED_TYPE_EMPTY_BYTE_UINT )
                {
                    args->args.disposition_args.first = 0;
                }
                if ( temp == FIXED_TYPE_ONE_BYTE_UINT )
                {
                    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.disposition_args.first );
                }
                if ( temp == FIXED_TYPE_FOUR_BYTE_UINT )
                {
                    AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.disposition_args.first );
                }
                break;
            case 3:
                AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                if ( temp == NULL_BYTE )
                {
                    /* If not set, this is taken to be the same as first. */
                    args->args.disposition_args.last = args->args.disposition_args.first;
                }
                if ( temp == FIXED_TYPE_EMPTY_BYTE_UINT )
                {
                    args->args.disposition_args.last = 0;
                }
                if ( temp == FIXED_TYPE_ONE_BYTE_UINT )
                {
                    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.disposition_args.last );
                }
                if ( temp == FIXED_TYPE_FOUR_BYTE_UINT )
                {
                    AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.disposition_args.last );
                }
                break;
            case 4:
                AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                if ( temp == BOOLEAN_TYPE_TRUE )
                {
                    args->args.disposition_args.settled = WICED_TRUE;
                }
                if ( temp == BOOLEAN_TYPE_FALSE )
                {
                    args->args.disposition_args.settled = WICED_FALSE;
                }
                break;
            case 5:
                AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
                AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.disposition_args.delivery_state );
                break;
            default:
                break;
        }
        temp_variable++;
    } while ( temp_variable <= count );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_transfer_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint64_t total_frame_size = 0;
    uint64_t temp = 0;
    uint32_t count = 0;
    uint8_t *temp_pointer = NULL;

    get_fixed_frame( frame, &args->fixed_frame );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    total_frame_size = args->fixed_frame.size - 8 - 4 /* size - 8(doff , type , channel) - 4( performative type) */;

    if ( temp == COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        AMQP_BUFFER_GET_OCTET( &frame->buffer, count );
        total_frame_size -= ( temp + 1 );
    }
    else if ( temp == COMPOUND_TYPE_FOUR_BYTE_LIST )
    {
        AMQP_BUFFER_GET_LONG( &frame->buffer, temp );

        AMQP_BUFFER_GET_LONG( &frame->buffer, count );
        total_frame_size -= ( temp + 4 );
    }
    else
        count = 0;

    /*parse transfer frame contents*/
    if ( count )
    {
        uint8_t temp_count = 1;
        wiced_bool_t temp_bool = WICED_TRUE;
        uint64_t temp_data = 0;
        do
        {
            switch ( temp_count )
            {
                case 1:
                    get_element_value( &temp_data, &temp_bool, &frame->buffer );
                    args->args.transfer_args.handle = (uint32_t) temp_data;
                    break;
                case 2:
                    get_element_value( &temp_data, &temp_bool, &frame->buffer );
                    args->args.transfer_args.delivery_id = (uint32_t) temp_data;
                    break;
                case 3:
                    /*not saving delivery-tag*/
                    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                    if ( temp == 0xa0 )
                    {
                        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                        /*TODO*/
                        frame->buffer.data = &frame->buffer.data[ temp ];
                    }
                    break;
                case 4:
                    get_element_value( &temp_data, &temp_bool, &frame->buffer );
                    args->args.transfer_args.message.message_format = (uint32_t) temp_data;
                    break;
                case 5:
                    get_element_bool( &args->args.transfer_args.message.settle, &temp_bool, &frame->buffer );
                    break;
                case 6:
                    get_element_bool( &args->args.transfer_args.message.more, &temp_bool, &frame->buffer );
                    break;
                case 7:
                    get_element_value( &temp_data, &temp_bool, &frame->buffer );
                    args->args.transfer_args.message.receiver_settle_mode = (uint8_t) temp_data;
                    break;
                case 8:
                    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );/*state*/
                    break;
                case 9:
                    get_element_value( &temp_data, &temp_bool, &frame->buffer );/*resume*/
                    break;
                case 10:
                    get_element_bool( &args->args.transfer_args.message.abort, &args->args.transfer_args.message.abort_inlcuded, &frame->buffer );
                    break;
                case 11:
                    get_element_bool( &temp_bool, &temp_bool, &frame->buffer );/*batchable*/
                    break;
                default:
                    break;
            }
            temp_count++;
        } while ( temp_count <= count );
    }

    if ( total_frame_size )
    {
        do
        {
            AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            total_frame_size = total_frame_size - 3;
            temp_pointer = frame->buffer.data;
            if ( ( 0x70 <= temp ) | ( temp <= 0x78 ) )
            {
                if ( temp == HEADER_SECTION )
                {
                    amqp_message_get_header( &frame->buffer, &args->args.transfer_args.message.header );
                }
                else if ( temp == DELIVERY_ANNOTATIONS_SECTION )
                {
                    amqp_message_get_delivery_message_annotations( &frame->buffer, NULL );/*TODO*/
                }
                else if ( temp == MESSAGE_ANNOTATIONS_SECTION )
                {
                    amqp_message_get_delivery_message_annotations( &frame->buffer, NULL ); /*TODO*/
                }
                else if ( temp == PROPERTIES_SECTION )
                {
                    amqp_message_get_properties( &frame->buffer, &args->args.transfer_args.message.properties );
                }
                else if ( temp == APPLICATION_PROPERTIES_SECTION )
                {
                    amqp_message_get_application_properties( &frame->buffer, &args->args.transfer_args.message.application_properties );
                }
                else if ( temp == DATA_SECTION )
                {
                    amqp_message_get_data( &frame->buffer, &args->args.transfer_args.message );
                }
                else if ( temp == AMQP_VALUE_SECTION )
                {
                    amqp_message_get_data( &frame->buffer, &args->args.transfer_args.message );
                }
                else
                {
                    total_frame_size = 0;/*to break the while loop*/
                }
            }

            total_frame_size = total_frame_size - (uint64_t) ( frame->buffer.data - temp_pointer );
        } while ( total_frame_size > 0 );
    }

    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_get_disposition_performative( wiced_amqp_packet_content *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret;

    ret = amqp_manager( WICED_AMQP_EVENT_DISPOSITION_RCVD, args, 0, conn_instance );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_DISPOSITION_RCVD, args, conn_instance );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_get_transfer_performative( wiced_amqp_packet_content *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret = WICED_SUCCESS;

    ret = amqp_manager( WICED_AMQP_EVENT_TRANSFER_RCVD, args, 0, conn_instance );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_TRANSFER_RCVD, args, conn_instance );
        }
    }
    return ret;
}

wiced_result_t amqp_message_get_header( wiced_amqp_buffer_t* pointer, wiced_amqp_message_header* parsed_data )
{

    uint64_t temp_data = 0;
    uint8_t count = 0;
    uint32_t temp_variable = 0;
    uint64_t temp_value = 0;

    AMQP_BUFFER_GET_OCTET( pointer, temp_data );

    if ( temp_data == 0xc0 )
    {
        AMQP_BUFFER_GET_OCTET( pointer, temp_data ); /*size */
        AMQP_BUFFER_GET_OCTET( pointer, count );/*count*/

        if ( count == 0 )
        {
            parsed_data->included = WICED_FALSE;
            return WICED_SUCCESS;
        }
        else
        {
            parsed_data->included = WICED_TRUE;
        }

        temp_variable = 1;
        do
        {
            temp_value = 0;
            switch ( temp_variable )
            {
                case 1:
                    get_element_bool( &parsed_data->durable, &parsed_data->durable_included, pointer );
                    break;
                case 2:
                    get_element_value( &temp_value, &parsed_data->priority_included, pointer );
                    parsed_data->priority = (uint8_t) temp_value;
                    break;
                case 3:
                    get_element_value( &temp_data, &parsed_data->ttl_included, pointer );
                    parsed_data->ttl = (uint32_t) temp_value;
                    break;
                case 4:
                    get_element_bool( &parsed_data->first_acquirer, &parsed_data->first_acquirer_included, pointer );
                    break;
                case 5:
                    get_element_value( &temp_value, &parsed_data->delivery_count_included, pointer );
                    parsed_data->delivery_count = (uint32_t) temp_value;
                    break;
                default:
                    break;
            }
            temp_variable++;
        } while ( temp_variable <= count );
    }
    /*TODO*/
    return WICED_SUCCESS;
}

wiced_result_t amqp_message_get_delivery_message_annotations( wiced_amqp_buffer_t* pointer, void* parsed_data )
{
    uint64_t temp_data = 0;

    UNUSED_PARAMETER( parsed_data );

    /*skip the bytes related to delivery and message annotations*/
    AMQP_BUFFER_GET_OCTET( pointer, temp_data );

    if ( ( temp_data & UPPER_NIBBLE ) == COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_OCTET( pointer, temp_data );/*size*/
        pointer->data = &pointer->data[ temp_data ];
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == COMPOUND_TYPE_FOUR_BYTE_LIST )
    {
        AMQP_BUFFER_GET_LONG( pointer, temp_data );/*size*/
        pointer->data = &pointer->data[ temp_data ];
    }

    return WICED_SUCCESS;
}

wiced_result_t amqp_message_get_properties( wiced_amqp_buffer_t *pointer, wiced_amqp_message_properties* parsed_data )
{
    uint64_t temp_data;
    uint64_t count = 0;
    uint8_t temp_variable = 0;

    AMQP_BUFFER_GET_OCTET( pointer, temp_data );

    if ( ( temp_data & UPPER_NIBBLE ) == 0xC0 )
    {
        AMQP_BUFFER_GET_OCTET( pointer, temp_data );
        AMQP_BUFFER_GET_OCTET( pointer, count );

        if ( count == 0 )
        {
            parsed_data->included = WICED_FALSE;
            return WICED_SUCCESS;
        }
        else
        {
            parsed_data->included = WICED_TRUE;
        }

        temp_variable = 1;
        do
        {
            switch ( temp_variable )
            {
                case 1:
                    get_element_pointer( &parsed_data->message_id, &parsed_data->message_id_length, pointer );
                    break;
                case 2:
                    get_element_pointer( &parsed_data->user_id, &parsed_data->user_id_length, pointer );
                    break;
                case 3:
                    get_element_pointer( &parsed_data->to, &parsed_data->to_length, pointer );
                    break;
                case 4:
                    get_element_pointer( &parsed_data->subject, &parsed_data->subject_length, pointer );
                    break;
                case 5:
                    get_element_pointer( &parsed_data->reply_to, &parsed_data->reply_to_length, pointer );
                    break;
                case 6:
                    get_element_pointer( &parsed_data->correlation_id, &parsed_data->correlation_id_length, pointer );
                    break;
                case 7:
                    get_element_pointer( &parsed_data->content_type, &parsed_data->content_type_length, pointer );
                    break;
                case 8:
                    get_element_pointer( &parsed_data->content_encoding, &parsed_data->content_encoding_length, pointer );
                    break;
                case 9:
                    get_element_value( &parsed_data->absolute_expiry_time, &parsed_data->absolute_expiry_time_included, pointer );
                    break;
                case 10:
                    get_element_value( &parsed_data->creation_time, &parsed_data->creation_time_included, pointer );
                    break;
                case 11:
                    get_element_pointer( &parsed_data->group_id, &parsed_data->group_id_length, pointer );
                    break;
                case 12:
                {
                    uint64_t temp_group_sequence = 0;
                    get_element_value( &temp_group_sequence, &parsed_data->group_sequence_included, pointer );
                    parsed_data->group_sequence = (uint32_t) temp_group_sequence;
                    break;
                }
                case 13:
                    get_element_pointer( &parsed_data->reply_to_group_id, &parsed_data->reply_to_group_id_length, pointer );
                    break;
                default:
                    break;
            }

            temp_variable++;
        } while ( temp_variable <= count );

    }
    return WICED_SUCCESS;
}

wiced_result_t amqp_message_get_application_properties( wiced_amqp_buffer_t *pointer, wiced_amqp_message_application_properties* parsed_data )
{

    uint32_t temp_data = 0;
    uint32_t count = 0;
    uint8_t temp_variable;

    AMQP_BUFFER_GET_OCTET( pointer, temp_data );
    if ( ( temp_data & UPPER_NIBBLE ) == COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_OCTET( pointer, temp_data );
        AMQP_BUFFER_GET_OCTET( pointer, count );
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == COMPOUND_TYPE_FOUR_BYTE_LIST )
    {
        AMQP_BUFFER_GET_LONG( pointer, temp_data );
        AMQP_BUFFER_GET_LONG( pointer, count );
    }

    if ( count == 0 )
    {
        parsed_data->included = WICED_FALSE;
        return WICED_SUCCESS;
    }
    else
    {
        parsed_data->included = WICED_TRUE;
    }

    temp_variable = 0;
    count /= 2;
    do
    {
        get_element_pointer( &parsed_data->properties[ temp_variable ].key, &parsed_data->properties[ temp_variable ].key_length, pointer );
        get_element_pointer( (uint8_t**) &parsed_data->properties[ temp_variable ].value, &parsed_data->properties[ temp_variable ].value_length, pointer );
        parsed_data->count++;
        temp_variable++;
    } while ( temp_variable < count );
    return WICED_SUCCESS;
}

wiced_result_t amqp_message_get_data( wiced_amqp_buffer_t *pointer, wiced_amqp_message_t * parsed_data )
{
    uint64_t temp_data = 0;
    AMQP_BUFFER_GET_OCTET( pointer, temp_data );
    if ( ( temp_data & UPPER_NIBBLE ) == VARIABLE_TYPE_ONE_BYTE_BINARY )
    {
        AMQP_BUFFER_GET_OCTET( pointer, temp_data );
        parsed_data->data_lenth = (uint32_t) temp_data;
        parsed_data->data = pointer->data;
    }
    else
    {
        /*TODO:handle MAPS also*/
    }
    pointer->data = &pointer->data[ parsed_data->data_lenth ];
    return WICED_SUCCESS;
}
