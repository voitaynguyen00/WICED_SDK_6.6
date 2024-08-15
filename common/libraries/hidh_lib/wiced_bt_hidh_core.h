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
 * WICED HID Host State Machine
 *
 */

#pragma once

#include "wiced_bt_hidh_int.h"
#include "wiced_bt_hidh_con.h"

/*
 * wiced_bt_hidh_con_cback
 */
void wiced_bt_hidh_con_cback (wiced_bt_hidh_con_event_t event, wiced_bt_hidh_con_data_t *p_data);
/*
 * wiced_bt_hidh_core_discovery_done
 */
void wiced_bt_hidh_core_discovery_done (wiced_bt_hidh_dev_t *p_dev, wiced_bt_hidh_status_t status);

/*
 * wiced_bt_hidh_core_descriptor_done
 */
void wiced_bt_hidh_core_descriptor_done(wiced_bt_hidh_dev_t *p_dev, wiced_bt_hidh_status_t status,
        uint8_t *p_descriptor, uint16_t length);

/*
 * wiced_bt_hidh_core_close
 */
wiced_bt_hidh_status_t wiced_bt_hidh_core_close(wiced_bt_hidh_dev_t *p_dev);

/*
 * wiced_bt_hidh_core_set_report
 */
wiced_bt_hidh_status_t wiced_bt_hidh_core_set_report(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_channel_t channel, wiced_bt_hidh_report_type_t type, uint8_t report_id,
        uint8_t *p_data, uint16_t length);

/*
 * wiced_bt_hidh_core_get_report
 */
wiced_bt_hidh_status_t wiced_bt_hidh_core_get_report(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_report_type_t type, uint8_t report_id, uint16_t length);

/*
 * wiced_bt_hidh_core_set_protocol
 */
wiced_bt_hidh_status_t wiced_bt_hidh_core_set_protocol(wiced_bt_hidh_dev_t *p_dev,
        wiced_bt_hidh_protocol_t protocol);

