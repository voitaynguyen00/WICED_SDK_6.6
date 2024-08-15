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
 * Sample MCU application for BLE or BR/EDR HID Device using WICED HCI protocol.
 */

#include "app_include.h"

extern "C"
{
#include "app_host_hidd.h"
}

// Initialize app
void MainWindow::InitBLEHIDD()
{
    m_pairing_mode_active = FALSE;
    m_b_is_hidd = false;
    ui->cbBLEHIDInterupt->clear();
    ui->cbBLEHIDReport->clear();

    ui->cbBLEHIDInterupt->addItem("Control Channel", HCI_CONTROL_HID_REPORT_CHANNEL_CONTROL);
    ui->cbBLEHIDInterupt->addItem("Interrupt Channel", HCI_CONTROL_HID_REPORT_CHANNEL_INTERRUPT);

    ui->cbBLEHIDReport->addItem("Other", HCI_CONTROL_HID_REPORT_TYPE_OTHER);
    ui->cbBLEHIDReport->addItem("Input", HCI_CONTROL_HID_REPORT_TYPE_INPUT);
    ui->cbBLEHIDReport->addItem("Output", HCI_CONTROL_HID_REPORT_TYPE_OUTPUT);
    ui->cbBLEHIDReport->addItem("Feature", HCI_CONTROL_HID_REPORT_TYPE_FEATURE);

    ui->cbBLEHIDInterupt->setCurrentIndex(1);
    ui->cbBLEHIDReport->setCurrentIndex(1);
}

// Connect to peer device
void MainWindow::on_btnBLEHIDConnect_clicked()
{
    Log("Sending HID Connect Command");
    app_host_hidd_connect();
}

// Disconnect from peer device
void MainWindow::on_btnBLEHIDDisconnect_clicked()
{
    Log("Sending HID Disconnect Command");
    app_host_hidd_disconnect();
}

// Send HID report
void MainWindow::on_btnBLEHIDSendReport_clicked()
{
   char szLog[80] = { 0 };
   uint8_t report[50];
   uint8_t report_len = 0;

   QVariant v1 = ui->cbBLEHIDInterupt->currentData();
   uint8_t channel = (BYTE)v1.toUInt();

   QVariant v2 = ui->cbBLEHIDReport->currentData();
   uint8_t report_id = (BYTE)v2.toUInt();

   QString str = ui->lineEditBLEHIDSendText->text();

   report_len = GetHexValue(&(report[0]), 50, str);

   for (int i = 0; i < report_len; i++)
       sprintf(&szLog[strlen(szLog)], "%02x ", report[i]);

   Log("Sending HID Report: channel %d, report %d, %s",  channel, report_id, szLog);
   app_host_hidd_send_report(channel, report_id, report, report_len);
}

// Enter pairing mode
void MainWindow::on_btnBLEHIDPairingMode_clicked()
{
    if (!m_pairing_mode_active)
    {
        ui->btnBLEHIDPairingMode->setText("Exit Pairing Mode");
        m_pairing_mode_active = TRUE;
    }
    else
    {
        ui->btnBLEHIDPairingMode->setText("Enter Pairing Mode");
        m_pairing_mode_active = FALSE;
    }

    Log("Issue %s pairing mode command", m_pairing_mode_active ? "set" : "exit");
    app_host_hidd_pairing_mode(m_pairing_mode_active);
}

// Send HID report key
void MainWindow::on_btnBLEHIDSendKey_clicked()
{
    char buf[100] = { 0 };
    QString str = ui->cbBLEHIDKey->currentText();
    strcpy(buf, str.toStdString().c_str());    

    uint8_t cap_lock = ui->cbBLEHIDCapLock->isChecked();
    uint8_t ctrl_key = ui->cbBLEHIDCtrl->isChecked() ;
    uint8_t alt_key = ui->cbBLEHIDAlt->isChecked() ;
    char trace[256];
    sprintf(trace, "Send HID Report Type:input ID:1 cap:%02x ctrl:%02x alt:%02x %02x %02x %02x %02x %02x %02x",
            cap_lock, ctrl_key, alt_key, buf[5], buf[6], buf[7], buf[8], buf[9], buf[10]);
    Log(trace);    

    uint8_t btn_up = ui->cbBLEHIDBtnUp->isChecked();
    app_host_hidd_send_key(cap_lock, ctrl_key, alt_key, buf, btn_up);
}

void MainWindow::on_cbBLEHIDCapLock_clicked()
{
    uint8_t cap_lock = ui->cbBLEHIDCapLock->isChecked();
    uint8_t ctrl_key = ui->cbBLEHIDCtrl->isChecked() ;
    uint8_t alt_key = ui->cbBLEHIDAlt->isChecked() ;

    app_host_hidd_cap_lock(cap_lock, ctrl_key, alt_key);
}

void MainWindow::on_btnBLEHIDDVirtualUnplug_clicked()
{
    Log("Sending HIDD Virtual Unplug Command");
    app_host_hidd_virtual_unplug();
}


void MainWindow::on_cbBLEHIDCtrl_clicked()
{
    on_cbBLEHIDCapLock_clicked();
}

void MainWindow::on_cbBLEHIDAlt_clicked()
{
    on_cbBLEHIDCapLock_clicked();
}

// Handle WICED HCI events for BLE/BR HID device
void MainWindow::onHandleWicedEventBLEHIDD(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    char   trace[1024];

    switch (opcode)
    {
        case HCI_CONTROL_EVENT_DEVICE_STARTED:
            m_pairing_mode_active = FALSE;
            ui->btnBLEHIDPairingMode->setText("Enter Pairing Mode");
            break;

        case HCI_CONTROL_HID_EVENT_OPENED:
            if (p_data)
                Log("HID connection opened with %02X:%02X:%02X:%02X:%02X:%02X",p_data[5],p_data[4],p_data[3],p_data[2],p_data[1],p_data[0]);
            else
                Log("HID connection opened");
            ui->btnBLEHIDPairingMode->setText("Enter Pairing Mode");
            m_pairing_mode_active = FALSE;
            break;
        case HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE:
            sprintf(trace, "Advertisement state:%d", p_data[0]);
            Log(trace);
            if (p_data[0] == 0)
            {
                ui->btnBLEHIDPairingMode->setText("Enter Pairing Mode");
                m_pairing_mode_active = FALSE;
            }
            break;
        case HCI_CONTROL_HID_EVENT_VIRTUAL_CABLE_UNPLUGGED:
            Log("HID Virtual Cable Unplugged");
            break;
        case HCI_CONTROL_HID_EVENT_DATA:
            sprintf(trace, "Recv HID Report type:%d ", p_data[0]);
            for (uint i = 0; i < len - 1; i++)
                sprintf(&trace[strlen(trace)], "%02x ", p_data[i + 1]);
            Log(trace);
            break;
        case HCI_CONTROL_HID_EVENT_CLOSED:
            sprintf(trace, "HID Connection down reason: %d ", p_data[0]);
            Log(trace);
            break;
    }
}


void MainWindow::on_btnHelpHIDD_clicked()
{
    onClear();
    Log("HID Device help topic:");
    Log("");

    Log("WICED Platforms : 20706-A2, 20719-B1, 20735-B1, 20819-A1");
    Log("Apps : use app under demo/hid on 20735-B1. Use hci_hid_host for BR-EDR or ");
    Log("       'hci_ble_hid_host' for BLE HOGP on 20706-A2 and 20719-B1");
    Log("Peer device : Windows PC or any HID host");
    Log("");

    Log("Note: For 20735-B1 and 20819-A1 apps, see flag TESTING_USING_HCI in the app makefile");
    Log("");

    Log("- Enter Pairing Mode");
    Log("  Sets the local device to pair-able");
    Log("- Connect");
    Log("  Connect with a HID host");
    Log("- Send Key");
    Log("  Sends the specified key from the drop down with options such as button up,");
    Log("  Caps Lock, etc.");
    Log("- Send Report");
    Log("  Send report for Interrupt or Control channel.");
    ScrollToTop();

}
