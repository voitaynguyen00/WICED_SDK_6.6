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

#include "wiced_hci_hidd.h"
#include "hci_control_api.h"
#include "string.h"

bool wiced_hci_hidd_disconnect()
{
    return wiced_hci_send_command(HCI_CONTROL_HIDD_COMMAND_DISCONNECT, 0, 0);
}

bool wiced_hci_hidd_connect()
{
    return wiced_hci_send_command(HCI_CONTROL_HID_COMMAND_CONNECT, 0, 0);
}

bool wiced_hci_hidd_send_report(wiced_hci_bt_hidd_report_t * data)
{	
	uint8_t cmd[60];

	cmd[0] = data->channel;
	cmd[1] = data->report_id;
	memcpy(&(cmd[2]), data->report, data->report_len);

	return wiced_hci_send_command(HCI_CONTROL_HID_COMMAND_SEND_REPORT, cmd, 2 + data->report_len);
}

bool wiced_hci_hidd_pairing_mode(wiced_hci_bt_hidd_paring_mode_data_t * data)
{
    uint8_t    cmd[1];
	cmd[0] = data->pairing_mode;
    return wiced_hci_send_command(HCI_CONTROL_HID_COMMAND_ACCEPT_PAIRING, cmd, 1);
}
	
bool wiced_hci_bt_hidd_send_key(wiced_hci_bt_hidd_send_key_data_t * data)
{
    uint8_t  cmd[60] = { 0 };
    int i = 0;
    cmd[0] = HCI_CONTROL_HID_REPORT_CHANNEL_INTERRUPT;
    cmd[1] = HCI_CONTROL_HID_REPORT_TYPE_INPUT;
    cmd[2] = HCI_CONTROL_HID_REPORT_ID;
    cmd[3] = data->cap_lock ? 0x20 : 0x00 |
        data->ctrl_key ? 0x80 : 0x00 |
        data->alt_key ? 0x40 : 0x00;

    for (i = 0; (i < 6) && (data->buffer[i] != 0); i++)
    {
        if ((data->buffer[i] >= '1') && (data->buffer[i] <= '9'))
            cmd[5 + i] = 0x1e + data->buffer[i] - '1';
        if (data->buffer[i] == '0')
            cmd[5 + i] = 0x27;
        if ((data->buffer[i] >= 'a') && (data->buffer[i] <= 'z'))
            cmd[5 + i] = 0x04 + data->buffer[i] - 'a';
    }
 
	bool rval = wiced_hci_send_command(HCI_CONTROL_HID_COMMAND_SEND_REPORT, cmd, 11);

    if (data->btn_up)
    {
        for (i = 0; i < 7; i++)
            cmd[4 + i] = 0;
        rval &= wiced_hci_send_command(HCI_CONTROL_HID_COMMAND_SEND_REPORT, cmd, 11);
    }
	
	return rval;
}

bool wiced_hci_bt_hidd_cap_lock(wiced_hci_bt_hidd_cap_lock_data_t *data)
{
    uint8_t cmd[60] = { 0 };
    int i = 0;
    cmd[0] = HCI_CONTROL_HID_REPORT_CHANNEL_INTERRUPT;
    cmd[1] = HCI_CONTROL_HID_REPORT_TYPE_INPUT;
    cmd[2] = HCI_CONTROL_HID_REPORT_ID;
    cmd[3] = data->cap_lock ? 0x20 : 0x00 |
        data->ctrl_key ? 0x80 : 0x00 |
        data->alt_key ? 0x40 : 0x00;
    for (i = 0; i < 7; i++)
        cmd[4 + i] = 0;

    return wiced_hci_send_command(HCI_CONTROL_HID_COMMAND_SEND_REPORT, cmd, 11);
}

bool wiced_hic_bt_hidd_virtual_unplug()
{
    return wiced_hci_send_command(HCI_CONTROL_HIDD_COMMAND_VIRTUAL_UNPLUG, 0, 0);
}	
	
	
	
