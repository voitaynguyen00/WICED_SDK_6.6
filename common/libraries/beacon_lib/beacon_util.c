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

/* 
*  Utility used by beacon functions
*/

#include "wiced_bt_stack.h"
#include "wiced_bt_beacon.h"
#include "string.h"

/*
* This function copies advertiement elements into a flat buffer and return the lenght
*/
void wiced_bt_beacon_set_adv_data(wiced_bt_beacon_ble_advert_elem_t *beacon_adv_elem, uint8_t num_elem, 
                                  uint8_t adv_data[WICED_BT_BEACON_ADV_DATA_MAX], uint8_t *adv_len)
{
    uint8_t i = 0, j = 0;
    for (i = 0; i < num_elem; i++)
    {
        adv_data[j++] = beacon_adv_elem[i].len;
        adv_data[j++] = beacon_adv_elem[i].advert_type;
        memcpy(&adv_data[j], beacon_adv_elem[i].data, (beacon_adv_elem[i].len - 1)); // len -1 for advert_type
        j += beacon_adv_elem[i].len - 1;
    }
    *adv_len = j;
}