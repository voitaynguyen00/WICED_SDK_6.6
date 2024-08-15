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
 *  flow functions
 *
 *  Provides flow methods for use by library
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_internal.h"
#include "amqp_frame.h"
#include "amqp_flow.h"
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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/


/******************************************************
 *               Protocol Header Frames
 ******************************************************/

wiced_result_t wiced_amqp_update_link_credit( wiced_amqp_link_instance *link_instance, uint32_t link_credit )
{
     wiced_amqp_flow_instance args;
     args.flow_args.next_incoming_id = link_instance->session->begin_args.next_incoming_id;
     args.flow_args.next_outgoing_id = link_instance->session->begin_args.next_outgoing_id;
     args.flow_args.link_credit = ( link_instance->link_credit + link_credit );
     args.flow_args.incoming_window = link_instance->session->begin_args.incoming_window;
     args.flow_args.handle = link_instance->link_args.handle;
     args.flow_args.outgoing_window = link_instance->session->begin_args.outgoing_window;
     args.flow_args.delivery_count = link_instance->delivery_count;
     args.link_instance = link_instance;

     /*sender link can't provide link credits*/
     if ( link_instance->link_args.role == WICED_AMQP_ROLE_SENDER )
    {
         args.flow_args.link_credit=0;
    }

     return amqp_manager( WICED_AMQP_EVENT_FLOW_SENT, &args, 0, link_instance->session->connection_instance );
}

wiced_result_t amqp_frame_put_flow( wiced_amqp_frame_t *frame,  void* arg )
{
    uint8_t count = AMQP_FLOW_FRAME_LIST_COUNT;
    wiced_amqp_flow_instance *args = arg;
    uint8_t* temp1 = frame->buffer.data;

    amqp_fixed_frame fixed_frame;

    fixed_frame.channel = 0x00;
    fixed_frame.doff = 0x02;
    fixed_frame.performative_type = AMQP_FLOW;
    fixed_frame.size = 0;
    fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    fixed_frame.count = count;

    put_fixed_frame( frame, &fixed_frame );

    put_element_value( args->flow_args.next_incoming_id, WICED_TRUE, &frame->buffer );

    put_element_value( args->flow_args.incoming_window, WICED_TRUE, &frame->buffer );

    put_element_value( args->flow_args.next_outgoing_id, WICED_TRUE, &frame->buffer );

    put_element_value( args->flow_args.outgoing_window, WICED_TRUE, &frame->buffer );

    put_element_value( args->flow_args.handle, WICED_TRUE, &frame->buffer );

    put_element_value( args->flow_args.delivery_count, WICED_TRUE, &frame->buffer );

    put_element_value( args->flow_args.link_credit, WICED_TRUE, &frame->buffer );

    put_element_value( args->flow_args.available, WICED_FALSE, &frame->buffer );

    put_element_bool( args->flow_args.drain, WICED_FALSE, &frame->buffer );

    put_element_bool( args->flow_args.echo, WICED_FALSE, &frame->buffer );

    put_element_bool( 0, WICED_FALSE, &frame->buffer );

    /*calculate the size of flow frame*/
    calculate_update_size_fields( temp1, temp1 + 12, frame );

    return WICED_SUCCESS;
}


wiced_result_t amqp_frame_get_flow_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint32_t temp = 0;
    uint32_t count = 0;
    wiced_bool_t value;
    uint64_t temp_value;

    get_fixed_frame( frame, &args->fixed_frame );

    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    if ( temp == COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        AMQP_BUFFER_GET_OCTET( &frame->buffer, count );
    }
    else if ( temp == COMPOUND_TYPE_FOUR_BYTE_LIST )
    {
        AMQP_BUFFER_GET_LONG( &frame->buffer, temp );

        AMQP_BUFFER_GET_LONG( &frame->buffer, count );
    }
    else
    {
count = 0;
    }


    temp=1;
    do
    {
        temp_value=0;
        switch(temp)
        {
        case 1:
            get_element_value(&temp_value,&value,&frame->buffer);
            args->args.flow_args.next_incoming_id=(uint32_t)temp_value;
            break;
        case 2:
            get_element_value(&temp_value,&value,&frame->buffer);
            args->args.flow_args.incoming_window=(uint32_t)temp_value;
            break;
        case 3:
            get_element_value(&temp_value,&value,&frame->buffer);
            args->args.flow_args.next_outgoing_id=(uint32_t)temp_value;
            break;
        case 4:
            get_element_value(&temp_value,&value,&frame->buffer);
            args->args.flow_args.outgoing_window=(uint32_t)temp_value;
            break;
        case 5:
            get_element_value(&temp_value,&args->args.flow_args.handle_included,&frame->buffer);
            args->args.flow_args.handle=(uint32_t)temp_value;
            break;
        case 6:
            get_element_value(&temp_value,&value,&frame->buffer);
            args->args.flow_args.delivery_count=(uint32_t)temp_value;
            break;
        case 7:
            get_element_value(&temp_value,&value,&frame->buffer);
            args->args.flow_args.link_credit=(uint32_t)temp_value;
            break;
        case 8:
            get_element_value(&temp_value,&value,&frame->buffer);
            args->args.flow_args.available=(uint32_t)temp_value;
            break;
        case 9:
            get_element_bool(&args->args.flow_args.drain,&args->args.flow_args.drain_included,&frame->buffer);
            break;
        case 10:
            get_element_bool(&args->args.flow_args.echo,&args->args.flow_args.echo_included,&frame->buffer);
            break;
        default:
            break;
        }
        temp++;
    }while(temp<=count);

    /*properties--skipping for now*/

    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_put_flow_performative( wiced_amqp_flow_instance* flow_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_FLOW_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &flow_instance->link_instance->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_flow( &frame, flow_instance );
    return amqp_frame_send( &frame, &flow_instance->link_instance->session->connection_instance->conn->socket );
}

wiced_result_t amqp_connection_backend_get_flow_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret = WICED_SUCCESS;

     ret = amqp_manager( WICED_AMQP_EVENT_FLOW_RCVD, args, 0, conn_instance );

    //TODO : Need to use all the args what we have received
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_FLOW_RCVD, args, conn_instance );
        }
    }
    return ret;

}
