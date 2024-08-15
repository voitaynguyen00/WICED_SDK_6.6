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

#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble_hidh_int.h"
#include "wiced_bt_ble_hidh_core.h"

/*
 * wiced_bt_ble_hidh_wakeup_pattern_add
 * Add a WakeUp pattern for a device
 */
wiced_bt_ble_hidh_status_t  wiced_bt_ble_hidh_wakeup_pattern_add(wiced_bt_ble_hidh_dev_t *p_dev,
        uint8_t report_id, uint8_t *p_pattern, uint16_t pattern_len);

/*
 * wiced_bt_ble_hidh_wakeup_pattern_check
 * Check if a Report (Id and Data) matches a WakeUp pattern of a device
 * If it matches, assert the WakeUp GPIO and return TRUE (report filtered)
 * If it does not match, return FALSE
 */
wiced_bool_t  wiced_bt_ble_hidh_wakeup_pattern_check(wiced_bt_ble_hidh_dev_t *p_dev,
        uint8_t report_id, uint8_t *p_data, uint16_t data_len);

