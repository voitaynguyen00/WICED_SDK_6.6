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
 * This file provides definitions and function prototypes for Hello Sensor
 * device
 *
 */
#include "wiced_bt_dev.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
 *                                Constants
 ******************************************************************************/

/******************************************************************************
 *                          Constants
 ******************************************************************************/

/*
 *  TODO: Define the UUIDs of your services/characteristics here
 */

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_BASICPERI_GATT_SERVICE = 0x1,

    HANDLE_BASICPERI_GAP_SERVICE = 0x14,
        HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_NAME,
        HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_NAME_VAL,

        HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_APPEARANCE,
        HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,


    /*
     *
     * TODO: Define your Custom Service Handles
     */

}simple_ble_peri_db_tags;

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

