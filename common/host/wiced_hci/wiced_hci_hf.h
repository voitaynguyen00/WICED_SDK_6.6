
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
 * Definitions for WICED HCI
 */

#ifndef WICED_HCI_HF_H
#define WICED_HCI_HF_H

#include "wiced_types.h"


// WICED HCI command
bool wiced_hci_send_command(uint16_t command, uint8_t * payload, uint32_t len);

// Handsfree
typedef struct
{
	uint8_t m_address[6];
} wiced_hci_bt_hf_connect_data_t;

typedef struct
{
    uint16_t handle;
} wiced_hci_bt_hf_disconnect_data_t;

typedef struct
{
    uint16_t handle;
} wiced_hci_bt_hf_open_audio_data_t;

typedef struct
{
    uint16_t handle;
} wiced_hci_bt_hf_close_audio_data_t;


typedef struct
{
    uint16_t handle;
    uint16_t num;
    char * atStr;
    uint16_t nAtCmd;
} wiced_hci_bt_hf_at_command_data_t;

bool wiced_hci_hf_connect (wiced_hci_bt_hf_connect_data_t * data);
bool wiced_hci_hf_disconnect (wiced_hci_bt_hf_disconnect_data_t * data);
bool wiced_hci_hf_open_audio (wiced_hci_bt_hf_open_audio_data_t * data);
bool wiced_hci_hf_close_audio (wiced_hci_bt_hf_close_audio_data_t * data);
bool wiced_hci_hf_at_command (wiced_hci_bt_hf_at_command_data_t * data);

bool wiced_hci_hf_connect (wiced_hci_bt_hf_connect_data_t * data);

typedef struct {
    uint16_t sco_index;
} wiced_hci_bt_hf_audio_accept_data_t;

bool wiced_hci_hf_audio_accept_conn (wiced_hci_bt_hf_audio_accept_data_t * data);


#endif // WICED_HCI_H


