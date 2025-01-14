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

#include "besl_host.h"
#include "p2p_host_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

extern besl_result_t            besl_p2p_init                                           ( p2p_workspace_t* workspace, const besl_p2p_device_detail_t* device_details );
extern besl_result_t            besl_p2p_init_common                                    ( p2p_workspace_t* workspace, const besl_p2p_device_detail_t* device_details );
extern besl_result_t            besl_p2p_deinit                                         ( p2p_workspace_t* workspace );
extern besl_result_t            besl_p2p_start                                          ( p2p_workspace_t* workspace );
extern besl_result_t            besl_p2p_listen_start                                   ( p2p_workspace_t* workspace );
extern besl_result_t            besl_p2p_start_negotiation                              ( p2p_workspace_t* workspace );
extern besl_result_t            besl_p2p_find_group_owner                               ( p2p_workspace_t* workspace );
extern besl_result_t            besl_p2p_get_discovered_peers                           ( p2p_workspace_t* workspace, p2p_discovered_device_t** devices, uint8_t* device_count );
extern besl_result_t            besl_p2p_group_owner_start                              ( p2p_workspace_t* workspace );
extern besl_result_t            besl_p2p_client_enable_powersave                        ( p2p_workspace_t* workspace, uint32_t power_save_mode );
extern besl_result_t            besl_p2p_send_action_frame                              ( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, p2p_action_frame_writer_t writer, uint32_t channel, uint32_t dwell_time );
extern besl_result_t            besl_p2p_start_registrar                                ( void );
extern p2p_discovered_device_t* besl_p2p_host_find_device                               ( p2p_workspace_t* workspace, const besl_mac_t* mac );
extern wiced_bool_t             besl_p2p_group_owner_is_up                              ( void );
extern void                     besl_p2p_host_negotiation_complete                      ( p2p_workspace_t* workspace );
extern void                     besl_p2p_register_p2p_device_connection_callback        ( p2p_workspace_t* workspace, void ( *p2p_connection_request_callback)(p2p_discovered_device_t*) );
extern void                     besl_p2p_register_legacy_device_connection_callback     ( p2p_workspace_t* workspace, void ( *p2p_legacy_device_connection_request_callback)(p2p_legacy_device_t*) );
extern void                     besl_p2p_register_group_formation_result_callback       ( p2p_workspace_t* workspace, void ( *p2p_group_formation_result_callback)(void*) );
extern void                     besl_p2p_register_wpa2_client_association_callback      ( p2p_workspace_t* workspace, void ( *p2p_client_wpa2_association_callback)(besl_mac_t*) );
extern void                     besl_p2p_register_wps_enrollee_association_callback     ( p2p_workspace_t* workspace, void ( *p2p_wps_enrollee_association_callback)(besl_mac_t*) );
extern void                     besl_p2p_register_wps_result_callback                   ( p2p_workspace_t* workspace, void (*p2p_wps_result_callback)(wps_result_t*) );
extern void                     besl_p2p_register_p2p_device_disassociation_callback    ( p2p_workspace_t* workspace, void (*p2p_device_disassociation_callback)(besl_mac_t*) );
extern void                     besl_p2p_register_legacy_device_disassociation_callback ( p2p_workspace_t* workspace, void (*p2p_non_p2p_device_disassociation_callback)(besl_mac_t*) );
extern besl_result_t            besl_p2p_get_group_formation_progress                   ( p2p_workspace_t* workspace );
extern besl_result_t            besl_p2p_go_get_client_wps_progress                     ( p2p_workspace_t* workspace );

#ifdef __cplusplus
} /* extern "C" */
#endif
