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

#include "wiced_bt_ble_hidh_wakeup.h"
#include "wiced_hal_gpio.h"


/*
 * wiced_bt_ble_hidh_wakeup_pattern_add
 * Add a WakeUp pattern for a device
 */
wiced_bt_ble_hidh_status_t  wiced_bt_ble_hidh_wakeup_pattern_add(wiced_bt_ble_hidh_dev_t *p_dev,
        uint8_t report_id, uint8_t *p_pattern_data, uint16_t pattern_data_len)
{
    int pattern_idx;
    wiced_bt_ble_hidh_wakeup_pattern_t *p_pattern;
    wiced_bt_gatt_status_t status;

    WICED_BT_BLE_HIDH_TRACE("Address:%B report_id:%x len:%d\n", p_dev->bdaddr, report_id,
            pattern_data_len);

    if (pattern_data_len > WICED_BT_BLE_HIDH_WAKEUP_PATTERN_LEN_MAX)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Pattern len:%d too long (max:%d)\n", pattern_data_len,
                WICED_BT_BLE_HIDH_WAKEUP_PATTERN_LEN_MAX);
        return WICED_BT_BLE_HIDH_STATUS_INVALID_PARAM;
    }

    /* Search for a free entry */
    p_pattern = &p_dev->wakeup_patterns[0];
    for (pattern_idx = 0; pattern_idx < WICED_BT_BLE_HIDH_WAKEUP_PATTERN_NB_MAX;
            pattern_idx++, p_pattern++)
    {
        /* If the ReportId is 0, it means that the entry in not in use */
        if (p_pattern->report_id == 0)
            break;
    }
    if (pattern_idx >= WICED_BT_BLE_HIDH_WAKEUP_PATTERN_NB_MAX)
    {
        WICED_BT_BLE_HIDH_TRACE_ERR("Pattern full (max:%d)\n",
                WICED_BT_BLE_HIDH_WAKEUP_PATTERN_NB_MAX);
        return WICED_BT_BLE_HIDH_STATUS_MEM_FULL;
    }

    /* Add the WakeUp pattern */
    WICED_BT_BLE_HIDH_TRACE("Adding WakeUp Pattern index:%d\n", pattern_idx);
    p_pattern->report_id = report_id;
    p_pattern->length = pattern_data_len;
    memcpy(p_pattern->pattern, p_pattern_data, pattern_data_len);

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

/*
 * wiced_bt_ble_hidh_wakeup_pattern_check
 * Check if a Report (Id and Data) matches a WakeUp pattern of a device
 * If it matches, assert the WakeUp GPIO and return TRUE (report filtered)
 * If it does not match, return FALSE
 */
wiced_bool_t  wiced_bt_ble_hidh_wakeup_pattern_check(wiced_bt_ble_hidh_dev_t *p_dev,
        uint8_t report_id, uint8_t *p_data, uint16_t data_len)
{
    int pattern_idx;
    wiced_bt_ble_hidh_wakeup_pattern_t *p_pattern;

    WICED_BT_BLE_HIDH_TRACE("report_id:0x%x len:%d data:%02X %02X %02X %02X %02X %02X %02X %02X\n",
            report_id, data_len, p_data[0], p_data[1], p_data[2], p_data[3],
            p_data[4], p_data[5], p_data[6], p_data[7]);

    /* WakeUp not enabled */
    if (wiced_bt_ble_hidh_cb.wakeup_control.enable != WICED_TRUE)
        return WICED_FALSE;

    if (data_len > WICED_BT_BLE_HIDH_WAKEUP_PATTERN_LEN_MAX)
        return WICED_FALSE;

    p_pattern = &p_dev->wakeup_patterns[0];
    for (pattern_idx = 0; pattern_idx < WICED_BT_BLE_HIDH_WAKEUP_PATTERN_NB_MAX;
            pattern_idx++, p_pattern++)
    {
        WICED_BT_BLE_HIDH_TRACE("testing against rid:0x%x len:%d data:%02X %02X %02X %02X %02X %02X %02X %02X\n",
                p_pattern->report_id, p_pattern->length,
                p_pattern->pattern[0], p_pattern->pattern[1], p_pattern->pattern[2], p_pattern->pattern[3],
                p_pattern->pattern[4], p_pattern->pattern[5], p_pattern->pattern[6], p_pattern->pattern[7]);

        if ((p_pattern->report_id == report_id) &&
            (p_pattern->length == data_len) &&
            (memcmp(p_pattern->pattern, p_data, data_len) == 0))
        {
            WICED_BT_BLE_HIDH_TRACE("WakeUp Pattern match. Assert WakeUp GPIO:%d\n",
                    wiced_bt_ble_hidh_cb.wakeup_control.gpio_num);

            wiced_hal_gpio_set_pin_output(wiced_bt_ble_hidh_cb.wakeup_control.gpio_num,
                    wiced_bt_ble_hidh_cb.wakeup_control.polarity);
            return WICED_TRUE;
        }
    }

    return WICED_FALSE;
}

