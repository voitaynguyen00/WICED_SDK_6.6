/*
 *  Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
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

#pragma once

#include "wiced_bt_ble_hidh_core.h"

wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_gattc_search(wiced_bt_ble_hidh_dev_t *p_dev);

wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_gattc_read_descriptor(wiced_bt_ble_hidh_dev_t *p_dev);

wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_gattc_set_report(wiced_bt_ble_hidh_dev_t *p_dev,
        wiced_bt_ble_hidh_report_type_t report_type, uint8_t report_id, uint8_t *p_data,
        uint16_t length);

wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_gattc_set_protocol(wiced_bt_ble_hidh_dev_t *p_dev,
        wiced_bt_ble_hidh_protocol_t protocol);
        
wiced_bt_ble_hidh_gatt_char_t *wiced_bt_ble_hidh_gattc_get_report_id(uint16_t conn_id, uint16_t handle);

wiced_bt_gatt_status_t wiced_bt_ble_hidh_gattc_operation_complete(wiced_bt_ble_hidh_dev_t *p_dev,
        wiced_bt_gatt_operation_complete_t *p_operation_complete);

wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_gattc_get_report(wiced_bt_ble_hidh_dev_t *p_dev,
        wiced_bt_ble_hidh_report_type_t report_type, uint8_t report_id, uint16_t length);

wiced_bt_gatt_status_t wiced_bt_ble_hidh_gattc_discovery_result(wiced_bt_ble_hidh_dev_t *p_dev,
        wiced_bt_gatt_discovery_result_t *p_discovery_result);

wiced_bt_gatt_status_t wiced_bt_ble_hidh_gattc_discovery_complete(wiced_bt_ble_hidh_dev_t *p_dev,
        wiced_bt_gatt_discovery_complete_t *p_discovery_complete);

void wiced_bt_ble_hidh_gattc_dump(wiced_bt_ble_hidh_dev_t *p_dev);

