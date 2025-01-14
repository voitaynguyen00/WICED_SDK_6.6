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
 *
 */
#include "wiced.h"
#include "bt_packet_internal.h"
#include "bt_transport_driver.h"
#include "bt_transport_thread.h"

/******************************************************
 *                      Macros
 ******************************************************/

#ifdef DEBUG
#define DUMP_PACKET( direction, start, end )                        \
do                                                                  \
{                                                                   \
    if ( bt_transport_enable_packet_dump == WICED_TRUE )            \
    {                                                               \
        uint8_t* current = start;                                   \
        if ( direction == 0 )                                       \
            WPRINT_LIB_INFO(( "\n[BT Transport] TX -----\n" ));     \
        else                                                        \
            WPRINT_LIB_INFO(( "\n[BT Transport] RX -----\n" ));     \
        while ( current != end )                                    \
        {                                                           \
            WPRINT_LIB_INFO(( "%.2X ", (int)*current++ ));          \
            if ( ( current - start ) % 16 == 0 )                    \
            {                                                       \
                WPRINT_LIB_INFO(( "\n" ));                          \
            }                                                       \
        }                                                           \
        WPRINT_LIB_INFO(( "\n-----------------------\n" ));         \
    }                                                               \
}                                                                   \
while (0)
#else
#define DUMP_PACKET( direction, start, end )
#endif

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

static wiced_result_t bt_transport_thread_send_packet_handler   ( void* arg );
static wiced_result_t bt_transport_thread_receive_packet_handler( void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_worker_thread_t                         bt_transport_thread;
static wiced_bool_t                                  bt_transport_thread_initialised      = WICED_FALSE;
static bt_transport_thread_received_packet_handler_t bt_transport_received_packet_handler = NULL;
#ifdef DEBUG
static wiced_bool_t                                  bt_transport_enable_packet_dump      = WICED_FALSE;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t bt_transport_thread_init( bt_transport_thread_received_packet_handler_t handler )
{
    wiced_result_t result;

    if ( bt_transport_thread_initialised == WICED_TRUE )
    {
        return WICED_BT_SUCCESS;
    }

    bt_transport_received_packet_handler = handler;

    /* Create MPAF worker thread */
    result = wiced_rtos_create_worker_thread( &bt_transport_thread, BT_TRANSPORT_THREAD_PRIORITY, BT_TRANSPORT_STACK_SIZE, BT_TRANSPORT_QUEUE_SIZE );
    if ( result == WICED_SUCCESS )
    {
        return WICED_BT_SUCCESS;
    }
    else
    {
        wiced_assert( "Error creating BT transport thread\n", 0!=0 );
        bt_transport_received_packet_handler = NULL;
        return result;
    }
}

wiced_result_t bt_transport_thread_deinit( void )
{
    if ( bt_transport_thread_initialised == WICED_FALSE )
    {
        return WICED_BT_SUCCESS;
    }

    /* Deliberately not checking for result. Deinit always assumed successful */
    wiced_rtos_delete_worker_thread( &bt_transport_thread );
    bt_transport_received_packet_handler = NULL;
    bt_transport_thread_initialised      = WICED_FALSE;

    return WICED_BT_SUCCESS;
}

wiced_result_t bt_transport_thread_send_packet( bt_packet_t* packet )
{
    if ( packet == NULL )
    {
        return WICED_BT_BADARG;
    }

    if ( wiced_rtos_is_current_thread( &bt_transport_thread.thread ) == WICED_SUCCESS )
    {
        return bt_transport_thread_send_packet_handler( (void*)packet );
    }
    else
    {
        return wiced_rtos_send_asynchronous_event( &bt_transport_thread, bt_transport_thread_send_packet_handler, (void*)packet );
    }
}

wiced_result_t bt_transport_thread_notify_packet_received( void )
{
    return wiced_rtos_send_asynchronous_event( &bt_transport_thread, bt_transport_thread_receive_packet_handler, NULL );
}

wiced_result_t bt_transport_thread_execute_callback( bt_transport_thread_callback_handler_t callback_handler, void* arg )
{
    return wiced_rtos_send_asynchronous_event( &bt_transport_thread, callback_handler, arg );
}

wiced_result_t bt_transport_thread_enable_packet_dump( void )
{
#ifdef DEBUG
    bt_transport_enable_packet_dump = WICED_TRUE;
    return WICED_BT_SUCCESS;
#else
    return WICED_BT_UNSUPPORTED;
#endif
}

wiced_result_t bt_transport_thread_disable_packet_dump( void )
{
#ifdef DEBUG
    bt_transport_enable_packet_dump = WICED_FALSE;
    return WICED_BT_SUCCESS;
#else
    return WICED_BT_UNSUPPORTED;
#endif
}

static wiced_result_t bt_transport_thread_send_packet_handler( void* arg )
{
    bt_packet_t* packet = (bt_packet_t*) arg;

    DUMP_PACKET( 0, packet->packet_start, packet->data_end );

    return bt_transport_driver_send_packet( packet );
}

static wiced_result_t bt_transport_thread_receive_packet_handler( void* arg )
{
    bt_packet_t*   packet = NULL;
    wiced_result_t result = bt_transport_driver_receive_packet( &packet );

    wiced_assert( "bt_transport_received_packet_handler isn't set", bt_transport_received_packet_handler != NULL );

    UNUSED_PARAMETER( arg );

    if ( result == WICED_BT_SUCCESS && packet != NULL )
    {
        DUMP_PACKET( 1, packet->packet_start, packet->data_end );
        result = bt_transport_received_packet_handler( packet );
    }

    return result;
}
