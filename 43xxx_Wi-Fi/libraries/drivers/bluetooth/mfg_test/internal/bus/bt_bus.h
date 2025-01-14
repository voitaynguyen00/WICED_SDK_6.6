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

#pragma once

#include "wiced_result.h"
#include "wiced_utilities.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/* Macro for checking if bus is ready */
#define BT_BUS_IS_READY( ) \
do \
{ \
    if ( bt_bus_is_ready( ) == WICED_FALSE ) \
    { \
        wiced_assert( "bus not ready", 0!=0 ); \
        return WICED_ERROR; \
    } \
}while ( 0 )

/* Macro for waiting until bus is ready */
#define BT_BUS_WAIT_UNTIL_READY( ) \
do \
{ \
    while ( bt_bus_is_ready( ) == WICED_FALSE ) \
    { \
        wiced_rtos_delay_milliseconds( 10 ); \
    } \
} while ( 0 )

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t bt_bus_init( void );

wiced_result_t bt_bus_deinit( void );

wiced_result_t bt_bus_transmit( const uint8_t* data_out, uint32_t size );

wiced_result_t bt_bus_receive( uint8_t* data_in, uint32_t size, uint32_t timeout_ms );

wiced_bool_t   bt_bus_is_ready( void );

wiced_bool_t   bt_bus_is_on( void );

#ifdef __cplusplus
} /* extern "C" */
#endif
