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
* Bluetooth (BT) Classic Keyboard
*
* The BT Keyboard application is a single chip SoC.  It provides a turnkey solution
* using on-chip keyscan HW component and is compliant with HID Profile.
*
* During initialization the app registers with BT stack, WICED HID Device Library and
* keyscan HW to receive various notifications including pairing complete, HID connection
* status change, HIDD events and interrupts for key pressed/released.
* Press any key will enter discoverable, i.e inquiry scan and page scan enabled. 
* When device is successfully bonded, the app saves bonded host's information in the NVRAM. 
* When user presses/releases key, a key report will be sent to the host.
* On connection up or battery level changed, a battery report will be sent to the host.
* When battery level is below shutdown voltage, device will do critical shutdown.
* Host can send LED report to the device to control LED.
*
* Features demonstrated
*  - SDP database and Device configuration initialization
*  - Registration with BT stack for various events 
*  - Sending HID reports to the host
*  - Receiving HID reports from the host
*  - Low power management
*
* To demonstrate the app, walk through the following steps.
* 1. Plug the CYW920739FCBGA120 board or 20739B1 Keyboard HW into your computer
* 2. Build and download the application (to the EVAL board or Keyboard HW) as below:
*    demo.hid.bt_keyboard-CYW920719Q40EVB_01 download UART=COMxx
* 3. Unplug the EVAL board or Keyboard HW from your computer and power cycle the EVAL board or keyboard HW
* 4. Press any key to enter discoverable, then pair with a PC or Tablet
*     If using the CYW920739FCBGA120 board, use a fly wire to connect GPIO P0 and P11 to simulate key 'r' press,
*     and remove the wire to simulate key release.
* 5. Once connected, it becomes the keyboard of the PC or Tablet.
*
* !!! In case you don't have the right board, i.e. CYW920739FCBGA120, which is required to support the 8*15
* key matrix used in the keyboard application. And you only have CYW920719Q40EVB_01 board.
* There is a ClientControl tool in the apps\host\client_control that you can use to test the basic BLE functions.
* NOTE!!!Make sure you include "TESTING_USING_HCI=1" in make target:
*     demo.hid.bt_keyboard-CYW920719Q40EVB_01 download UART=COMxx TESTING_USING_HCI=1
*
* 1. Plug the WICED EVAL board into your computer
* 2. Build and download the application (to the WICED board) as below:
*    demo.hid.bt_keyboard-CYW920719Q40EVB_01 download UART=COMxx TESTING_USING_HCI=1
* 3. Run ClientControl.exe
* 4. Choose 115200 baudrate and select the "COM Port" in ClientControl tool window.
* 5. Press "Enter Pairing Mode"or "Connect" to enter discoverable, then pair with a PC or Tablet
* 6. Once connected, it becomes the keyboard of the PC or Tablet.
*  - Select Interrupt channel, Input report, enter the contents of the report
*    and click on the Send button, to send the report.  For example to send
*    key down event when key '1' is pushed, report should be
*    01 00 00 1e 00 00 00 00 00.  All keys up 01 00 00 00 00 00 00 00 00.
*    Please make sure you always send a key up report following key down report.
*/
#include "spar_utils.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_mia.h"
//#include "scrolldriver.h"
#include "keyboard.h"
#include "wiced_bt_sdp.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_hidd.h"
#include "wiced_hal_batmon.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_adc.h"
#include "wiced_hidd_lib.h"
#include "wiced_memory.h"
#include "bthidlink.h"
#ifdef LED_USE_PWM
 #include "wiced_hal_aclk.h"
 #include "wiced_hal_pwm.h"
#endif



#define VC_UNPLUG_ON_CONNECT_BUTTON_PRESS   1
#define BECOME_DISCOVERABLE_ON_CONNECT_BUTTON_PRESS     1
//////////////////////////////////////////////////////////////////////////////
//                      local interface declaration
//////////////////////////////////////////////////////////////////////////////
tKbAppState keyboard_application_state = {0, };
tKbAppState *kbAppState = &keyboard_application_state;


uint8_t   kbapp_protocol = PROTOCOL_REPORT;


extern uint8_t timedWake_SDS;
uint8_t firstTransportStateChangeNotification = 1;
wiced_timer_t kb_allow_sleep_timer;

PLACE_DATA_IN_RETENTION_RAM uint8_t  kbapp_funcLock_state; // function lock state

extern KbAppConfig kbAppConfig;
extern KbKeyConfig kbKeyConfig[];
extern uint8_t bthid_kbKeyConfig_size;

extern wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;




wiced_bt_hidd_link_app_callback_t kbAppCallbacks =
{
    NULL, //kbapp_write_eir,                    //   *p_app_write_eir_data;
    kbapp_pollReportUserActivity,       //   *p_app_poll_user_activities;
    kbapp_connectFailedNotification,    //   *p_app_connection_failed_notification;

    kbapp_enterPinCodeEntryMode,        //   *p_app_enter_pincode_entry_mode;  
    kbapp_enterPassCodeEntryMode,       //   *p_app_enter_passcode_entry_mode;  
    kbapp_exitPinAndPassCodeEntryMode,  //   *p_app_exit_pin_and_passcode_entry_mode;

    kbapp_getIdleRate,                  //  *p_app_get_idle;
    kbapp_setIdleRate,                  //  *p_app_set_idle;
    kbapp_getProtocol,                  //  *p_app_get_protocol;
    kbapp_setProtocol,                  //  *p_app_set_protocol;
    kbapp_getReport,                    //  *p_app_get_report;
    kbapp_setReport,                    //  *p_app_set_report;
    kbapp_rxData,                       //  *p_app_rx_data;

};

/// Translation table for func-lock dependent keys. 
KbFuncLockDepKeyTransTab kbFuncLockDepKeyTransTab[KB_MAX_FUNC_LOCK_DEP_KEYS] = 
{
    // Home/F1
    {0x03, USB_USAGE_F1},
    // Lock/F2
    {0x05, USB_USAGE_F2},
    // Siri/F3
    {0x08, USB_USAGE_F3},
    // Search/F4
    {0x06, USB_USAGE_F4},

    // Language/F5
    {0x09, USB_USAGE_F5},
    // Eject/F6
    {0x0D, USB_USAGE_F6},
    // Previous Track/F7
    {0x0B, USB_USAGE_F7},
    // Play-Pause/F8
    {0x0E, USB_USAGE_F8},

    // Next Track/F9
    {0x0C, USB_USAGE_F9},
    // Mute/F10
    {0x11, USB_USAGE_F10},
    // Vol-Down/F11
    {0x10, USB_USAGE_F11},
    // Vol-Up/F12
    {0x0F, USB_USAGE_F12},
    
    // Power/Power
    {0x0, USB_USAGE_POWER}
};

uint8_t pinCodeEventTransTab[PASS_CODE_ENTRY_PROGRESS_MAX][KEY_ENTRY_EVENT_MAX] = 
{
    {
    (uint8_t)PIN_ENTRY_EVENT_INVALID, 
    (uint8_t)PIN_ENTRY_EVENT_CHAR, 
    (uint8_t)PIN_ENTRY_EVENT_BACKSPACE, 
    (uint8_t)PIN_ENTRY_EVENT_RESTART, 
    (uint8_t)PIN_ENTRY_EVENT_INVALID
    },
    {
    (uint8_t)PASS_KEY_ENTRY_EVENT_START, 
    (uint8_t)PASS_KEY_ENTRY_EVENT_CHAR, 
    (uint8_t)PASS_KEY_ENTRY_EVENT_BACKSPACE, 
    (uint8_t)PASS_KEY_ENTRY_EVENT_RESTART, 
    (uint8_t)PASS_KEY_ENTRY_EVENT_STOP
    }
};

UINT32 mpaf_cfa_mm_MemFreeBytes(void);
void wiced_hal_mia_pollHardware(void);
INT16 scroll_getCount(void);
wiced_bool_t wiced_hal_keyscan_is_any_key_pressed(void);

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allow_sleep_timer
////////////////////////////////////////////////////////////////////////////////
void kbapp_allowsleeptimerCb( uint32_t arg)
{

    WICED_BT_TRACE("allow SDS\n");
    
    kbAppState->allowSDS = WICED_TRUE;
}

void kbapp_create(void)
{
    WICED_BT_TRACE("KBCreated\n");


    //battery monitoring configuraion
    wiced_hal_batmon_config(ADC_INPUT_VDDIO,    // ADC input pin
                            3000,               // Period in millisecs between battery measurements
                            8,                  // Number of measurements averaged for a report, max 16
                            3200,               // The full battery voltage in mili-volts
                            1800,               // The voltage at which the batteries are considered drained (in milli-volts)
                            1700,               // System should shutdown if it detects battery voltage at or below this value (in milli-volts)
                            100,                // battery report max level
                            BATTERY_REPORT_ID,  // battery report ID
                            1,                  // battery report length
                            1);                 // Flag indicating that a battery report should be sent when a connection is established

#ifndef  TESTING_USING_HCI
    //keyscan configuration 
    wiced_hal_keyscan_configure(NUM_KEYSCAN_ROWS, NUM_KEYSCAN_COLS);
    wiced_hal_keyscan_init();
#endif

#ifdef __KB_SCROLL_REPORT__
    quadratureConfig.port0PinsUsedAsQuadratureInput=0;
    quadratureConfig.configureP26AsQOC0=0;
    quadratureConfig.ledEnableDisableControls=0;
    quadratureConfig.scanPeriod=0xff00;

    quadratureConfig.togglecountLed0=0xfff0;
    quadratureConfig.togglecountLed1=0xfff0;
    quadratureConfig.togglecountLed2=0xfff0;
    quadratureConfig.togglecountLed3=0xfff0;

    quadratureConfig.sampleInstantX=0xfff8;
    quadratureConfig.sampleInstantY=0xfff8;
    quadratureConfig.sampleInstantZ=0xfff8;

    quadratureConfig.channelEnableAndSamplingRate=0x88;

    quadratureConfig.pollXAxis=0;
    quadratureConfig.pollYAxis=1;
    quadratureConfig.pollZAxis=0;

    scroll_init();
#endif

    kbapp_LED_init();

    wiced_hidd_event_queue_init(&kbAppState->eventQueue, (uint8_t *)wiced_memory_permanent_allocate(kbAppConfig.maxEventNum * kbAppConfig.maxEventSize),
                    kbAppConfig.maxEventSize, kbAppConfig.maxEventNum);
    
    kbapp_init();
    
    WICED_BT_TRACE("Free RAM bytes=%d bytes\n", mpaf_cfa_mm_MemFreeBytes());
}

////////////////////////////////////////////////////////////////////////////////
/// This function is called when battery voltage drops below the configured threshold.
////////////////////////////////////////////////////////////////////////////////
void kbapp_shutdown(void)
{
    WICED_BT_TRACE("kbapp_shutdown\n");

    kbapp_flushUserInput();

#ifdef __KB_SCROLL_REPORT__
    // Disable the scroll HW
    scroll_turnOff();
#endif
#ifndef  TESTING_USING_HCI
    // Disable key detection
    wiced_hal_keyscan_turnOff();
#endif
    if(wiced_bt_hidd_link_is_connected())
    {
        wiced_bt_hidd_link_disconnect();
    }

    // Disable Interrupts
    wiced_hal_mia_enable_mia_interrupt(FALSE);
    wiced_hal_mia_enable_lhl_interrupt(FALSE);

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void kbapp_batLevelChangeNotification(uint32_t newLevel)
{
    WICED_BT_TRACE("bat level changed to %d\n", newLevel);

    if (kbapp_protocol == PROTOCOL_REPORT)
    {
        kbAppState->batRpt.level[0] = newLevel;
        kbapp_batRptSend();
    }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void kbapp_init(void)
{
    //timer to allow SDS
    wiced_init_timer( &kb_allow_sleep_timer, kbapp_allowsleeptimerCb, 0, WICED_MILLI_SECONDS_TIMER );

    // Determine the size of the standard report. 
    kbAppState->stdRptSize = kbAppConfig.maxKeysInStdRpt +
        (sizeof(KeyboardStandardReport) - sizeof(kbAppState->stdRpt.keyCodes));

    // Determine the size of the Battery
    kbAppState->batRpt.reportID = BATTERY_REPORT_ID;

    // Determine the size of the bit mapped report.
    // and round up to the next largest integer
    kbAppState->bitReportSize = (kbAppConfig.numBitMappedKeys + 7)/8 + 2;

#ifndef  TESTING_USING_HCI
    wiced_hal_keyscan_register_for_event_notification(kbapp_userKeyPressDetected, NULL);
#endif
#ifdef __KB_SCROLL_REPORT__
    quad_registerForEventNotification(kbapp_userScrollDetected, NULL);
#endif

    // Set initial func-lock state. If we came out of shutdown sleep (SDS), use the
    // last preserved state, otherwise use the default state
    if (wiced_hal_mia_is_reset_reason_por())
    {
        kbapp_funcLock_state = kbAppState->funcLockInfo.state = kbAppConfig.defaultFuncLockState;
    }

    // Set func lock key as up
    kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_UP;

    // The following flag applies when func-lock is used in combo with another key. Start it off as FALSE
    kbAppState->funcLockInfo.toggleStateOnKeyUp = WICED_FALSE;

    // Initialize temporaries used for events
    kbAppState->keyEvent.eventInfo.eventType = HID_EVENT_KEY_STATE_CHANGE;
    kbAppState->scrollEvent.eventInfo.eventType = HID_EVENT_MOTION_AXIS_0;


    kbapp_stdRptRolloverInit();

    kbapp_ledRptInit();

    kbapp_funcLockRptInit();

    kbapp_clearAllReports();

    //add battery observer
    wiced_hal_batmon_add_battery_observer(kbapp_batLevelChangeNotification);

    //register App low battery shut down handler
    wiced_hal_batmon_register_low_battery_shutdown_cb(kbapp_shutdown);

    wiced_bt_hidd_link_add_state_observer(kbapp_stateChangeNotification);

    wiced_bt_hidd_link_register_app_callback(&kbAppCallbacks);

    wiced_bt_hidd_link_register_sleep_permit_handler(kbapp_sleep_handler);

#ifdef __KB_SCROLL_REPORT__
    wiced_hal_mia_notificationRegisterQuad();
#endif
    wiced_bt_hidd_link_init();

    wiced_hal_mia_enable_mia_interrupt(TRUE);
    wiced_hal_mia_enable_lhl_interrupt(TRUE);//GPIO interrupt
}

/////////////////////////////////////////////////////////////////////////////////
/// When a paging the bonded host(s) fails, we have nothing to do but to
/// flush all events from the event queue.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_connectFailedNotification(void)
{   
    // Flush all user inputs.
    kbapp_flushUserInput();
}

////////////////////////////////////////////////////////////////////////////////
/// This function will poll user activities and send reports
////////////////////////////////////////////////////////////////////////////////
void kbapp_pollReportUserActivity(void)
{
    uint8_t activitiesDetectedInLastPoll;

    kbAppState->pollSeqn++;

    if((kbAppState->pollSeqn % 64) == 0)
    {
        WICED_BT_TRACE(".");
    }

    activitiesDetectedInLastPoll = kbapp_pollActivityUser();

    // If there was an activity and the transport is not connected
    if (activitiesDetectedInLastPoll != BTHIDLINK_ACTIVITY_NONE &&
        !wiced_bt_hidd_link_is_connected())
    {
        // ask the transport to connect.
        wiced_bt_hidd_link_connect();
    }

    if(wiced_bt_hidd_link_is_connected())
    {
        // Generate a report
        if(wiced_bt_hid_cfg_settings.security_requirement_mask)
        {
            if (bt_hidd_link.encrypt_status.encrypted)
            {
                kbapp_generateAndTxReports();
            }
        }
        else
        {
            kbapp_generateAndTxReports();
        }

        wiced_hal_batmon_poll_monitor();  // Poll the battery monitor
    }
    
}

////////////////////////////////////////////////////////////////////////////////
/// This function will poll HW for user activies
////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_pollActivityUser(void)
{
    // Poll the hardware for events
    wiced_hal_mia_pollHardware();

    // Poll and queue key activity
    kbapp_pollActivityKey();

#ifdef __KB_SCROLL_REPORT__
    // Poll and queue scroll activity
    kbapp_pollActivityScroll();
#endif

    // Check if we are in pin code entry mode. If so, call the pin code entry processing function
    if (kbAppState->pinCodeEntryInProgress != PIN_ENTRY_MODE_NONE)
    {
        kbapp_handlePinEntry();

        // Always indicate reportable and non-reportable activity when doing pin code entry
        return BTHIDLINK_ACTIVITY_REPORTABLE | BTHIDLINK_ACTIVITY_NON_REPORTABLE;
    }
    else
    {
        // For all other cases, return value indicating whether any event is pending or
        return (wiced_hidd_event_queue_get_num_elements(&kbAppState->eventQueue) ? BTHIDLINK_ACTIVITY_REPORTABLE : BTHIDLINK_ACTIVITY_NONE) |
               ((kbAppState->modKeysInStdRpt || kbAppState->keysInStdRpt || kbAppState->keysInBitRpt || kbAppState->slpRpt.sleepVal)?
                BTHIDLINK_ACTIVITY_NON_REPORTABLE : BTHIDLINK_ACTIVITY_NONE);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls for key activity and queues any key events in the
/// FW event queue. Events from the keyscan driver are processed until the driver
/// runs out of events. Connect button events are seperated out and handled here
/// since we don't want them to go through the normal event queue. If necessary,
/// the end of scan cycle event after the connect button is supressed. Also
/// note that connect button events are supressed during recovery to eliminate
/// spurious connect button events.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_pollActivityKey(void)
{
#ifndef  TESTING_USING_HCI
    uint8_t suppressEndScanCycleAfterConnectButton;

    // Assume that end-of-cycle event suppression is on
    suppressEndScanCycleAfterConnectButton = TRUE;

    // Process all key events from the keyscan driver
    while (wiced_hal_keyscan_get_next_event(&kbAppState->keyEvent.keyEvent))
    {
        //WICED_BT_TRACE("kc_: %d, U/D: %d\n", kbAppState->keyEvent.keyEvent.keyCode, kbAppState->keyEvent.keyEvent.upDownFlag);
        // Check for connect button
        if (kbAppState->keyEvent.keyEvent.keyCode == kbAppConfig.connectButtonScanIndex)
        {
            // Ignore connect button in recovery
            if (!kbAppState->recoveryInProgress)
            {
                // Pass current connect button state to connect button handler
                kbapp_connectButtonHandler(
                    ((kbAppState->keyEvent.keyEvent.upDownFlag == KEY_DOWN)?
                     CONNECT_BUTTON_DOWN:CONNECT_BUTTON_UP));
            }
        }
        else
        {
            // Check if this is an end-of-scan cycle event
            if (kbAppState->keyEvent.keyEvent.keyCode == END_OF_SCAN_CYCLE)
            {
                // Yes. Queue it if it need not be suppressed
                if (!suppressEndScanCycleAfterConnectButton)
                {
                    wiced_hidd_event_queue_add_event_with_overflow(&kbAppState->eventQueue, &kbAppState->keyEvent.eventInfo, sizeof(kbAppState->keyEvent), kbAppState->pollSeqn);
                }

                // Enable end-of-scan cycle supression since this is the start of a new cycle
                suppressEndScanCycleAfterConnectButton = TRUE;
            }
            else
            {
                // No. Queue the key event
                wiced_hidd_event_queue_add_event_with_overflow(&kbAppState->eventQueue, &kbAppState->keyEvent.eventInfo, sizeof(kbAppState->keyEvent), kbAppState->pollSeqn);

                // Disable end-of-scan cycle supression
                suppressEndScanCycleAfterConnectButton = FALSE;
            }
        }
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls the scroll interface to get any newly detected
/// scroll count. It negates the data and performs any scaling if configured to do so.
/// If configured to do so, it discards any fractional value after the configured
/// number of polls. If any non-fractional scroll activity is accumulated,
/// it queues a scroll event.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_pollActivityScroll(void)
{
    int16_t scrollCurrent = scroll_getCount();

    // Check for scroll
    if (scrollCurrent)
    {
        // Negate scroll value if enabled
        if (kbAppConfig.negateScroll)
        {
            scrollCurrent = -scrollCurrent;
        }

        // Check if scroll scaling is enabled
        if (kbAppConfig.scrollScale)
        {
            // Yes. Add the current scroll count to the fractional count
            kbAppState->scrollFractional += scrollCurrent;

            // Scale and adjust accumulated scroll value. Fractional value will be
            // left in the factional part. Place the whole number in the scroll
            // event
            kbAppState->scrollEvent.motion =
                kbapp_scaleValue(&kbAppState->scrollFractional, kbAppConfig.scrollScale);

            // Reset the scroll discard counter
            kbAppState->pollsSinceScroll = 0;
        }
        else
        {
            // No scaling is required. Put the data in the scroll event
            kbAppState->scrollEvent.motion = scrollCurrent;
        }

        // Queue scroll event with the proper seqn
        wiced_hidd_event_queue_add_event_with_overflow(&kbAppState->eventQueue, &kbAppState->scrollEvent.eventInfo, sizeof(kbAppState->scrollEvent), kbAppState->pollSeqn);
    }
    else
    {
        // If scroll scaling timeout is not infinite, bump up the
        // inactivity counter and check if we have crossed the threshold.
        if (kbAppConfig.pollsToKeepFracScrollData &&
            ++kbAppState->pollsSinceScroll >= kbAppConfig.pollsToKeepFracScrollData)
        {
            // We have. Discard any fractional scroll data
            kbAppState->scrollFractional = 0;

            // Reset the scroll discard counter
            kbAppState->pollsSinceScroll = 0;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
///   This function scales (divides by a power of 2) a value, returns the quotient
/// and leaves the remainder in the value. It handles positive and negative
/// numbers
///
/// \param val -Pointer to value. It outputs the remainder value
/// \param scaleFactor -Number of bits to scale by (shift right)
///
///
/// \return
///   The whole number after the scaling.
/////////////////////////////////////////////////////////////////////////////////
int16_t kbapp_scaleValue(int16_t *val, uint8_t scaleFactor)
{
    int16_t result;

    // Get the mod of the value
    if (*val < 0)
    {
        result = - *val;
    }
    else
    {
        result = *val;
    }

    // Now scale it by the given amount
    result >>= scaleFactor;

    // Check if we have anything left
    if (result)
    {
        // Yes. Now we have to adjust the sign of the result
        if (*val < 0)
        {
            // So we had a negative value. Adjust result accordingly
            result = -result;
        }

        // Now adjust the actual value
        *val -= (result << scaleFactor);
    }

    // Return the scaled value
    return result;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function performs connection button processing. It should be called with
/// the current state of the connect button. This function generates a
/// become discoverable event to the BT Transport if the connect button is
/// held for the configured duration. The configured duration may be 0
/// in which case an instantaneous press of the button causes the device
/// to become discoverable. Once a "become disoverable" event has
/// been generated, not further events will be generated until after the
/// button has been released
///
/// \param connectButtonPosition current position of the connect button, up or down
/////////////////////////////////////////////////////////////////////////////////
void kbapp_connectButtonHandler(ConnectButtonPosition connectButtonPosition)
{
    // The connect button was not pressed. Check if it is now pressed
    if (connectButtonPosition == CONNECT_BUTTON_DOWN)
    {
        WICED_BT_TRACE("Connect Btn Pressed\n");
        kbapp_connectButtonPressed();
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This method handles connect button pressed events. It performs the following
/// actions in order:
///  - If we are configured to generate a VC unplug on connect button press
///    it generates a VC unplug to the BT transport
///  - If we are configured to become discoverable on connect button press
///    it tells the BT transport to become discoverable
/////////////////////////////////////////////////////////////////////////////////
void kbapp_connectButtonPressed(void)
{
    // Generate VC unplug to BT transport if configured to do so
    if (VC_UNPLUG_ON_CONNECT_BUTTON_PRESS)
    {
        wiced_bt_hidd_link_virtual_cable_unplug();
    }

    // Tell BT transport to become discoverable if configured to do so
    if (BECOME_DISCOVERABLE_ON_CONNECT_BUTTON_PRESS)
    {
        wiced_bt_hidd_link_enter_discoverable();
    }

}

/////////////////////////////////////////////////////////////////////////////////
/// Process get idle rate. Generates an idle rate report on the control channel
/// of the given transport
/// \return idle rate
/////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_getIdleRate(void)
{
    return kbAppState->idleRate;
}

/////////////////////////////////////////////////////////////////////////////////
/// Set the idle rate. Converts the idle rate to BT clocks and saves the value
/// for later use.
/// \param idleRateIn4msUnits 0 means infinite idle rate
/// \return 
/////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_setIdleRate(uint8_t idleRateIn4msUnits)
{
    // Save the idle rate in units of 4 ms
    kbAppState->idleRate = idleRateIn4msUnits;

    // Convert to BT clocks for later use. Formula is ((Rate in 4 ms)*192)/15
    kbAppState->idleRateInBtClocks = idleRateIn4msUnits*192;
    kbAppState->idleRateInBtClocks /= 15;

    return HID_PAR_HANDSHAKE_RSP_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function performs idle rate processing for the standard keyboard report.
/// It will transmit the old standard report under the following conditions:
///     - Idle rate is non-zero
///     - We are not in the middle of a recovery
///     - At least one key is down in the standard report, either normal key
///       or a modifier key.
///     - No events are pending
///     - The active transport is willing to accept a report
///     - Required time has elapsed since the last time the standard report was sent
/////////////////////////////////////////////////////////////////////////////////
void kbapp_idleRateProc(void)
{
    // Send the standard report again if the above criteria is satisfied
    if (kbAppState->idleRate && 
        !kbAppState->recoveryInProgress &&
        (kbAppState->keysInStdRpt || kbAppState->modKeysInStdRpt) &&
        !wiced_hidd_event_queue_get_num_elements(&kbAppState->eventQueue) &&
        (wiced_bt_buffer_poolutilization(HCI_ACL_POOL_ID) < 80) &&
        (wiced_hidd_get_bt_clocks_since(kbAppState->stdRptTxInstant) >= kbAppState->idleRateInBtClocks))
    {
        kbapp_stdRptSend();
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function provides an implementation for the generateAndTxReports() function
/// defined by the HID application. This function is only called when the active transport
/// is connected. This function performs the following actions:
///  - When pin code entry is in progress, the behavior of this function is changed.
///    It only checks and transmits the pin code report; normal event processing is
///    suspended.
///  - If the number of packets in the hardware fifo is less than the report generation
///    threshold and the event queue is not empty, this function will process events
///    by calling the event rpocessing functions, e.g. kbapp_procEvtKey(),
///    kbapp_procEvtScroll()
///  - This function also tracks the recovery period after an error. If
///    the recovery count is non-zero, it is decremented as long as there is room
///    for one report in the transport
/////////////////////////////////////////////////////////////////////////////////
void kbapp_generateAndTxReports(void)
{
    HidEvent *curEvent;

    // Check if we are in pin code processing state
    if (kbAppState->pinCodeEntryInProgress != PIN_ENTRY_MODE_NONE)
    {
        // Transmit the pin code entry report if it has changed since last time
        if (kbAppState->pinRptChanged)
        {
            kbapp_pinRptSend();
        }
    }
    else
    {
        // Normal state

        // If we are recovering from an error, decrement the recovery count as long as the transport
        // has room. Avoid the case where no event processing is done during recovery because
        // transport is full, as the failure might be a non-responding transport.
        if (kbAppState->recoveryInProgress)
        {
            // If recovery is complete, transmit any modified reports that we have been hoarding
            if (!--kbAppState->recoveryInProgress)
            {
                kbapp_txModifiedKeyReports();
            }
        }
        // Continue report generation as long as the transport has room and we have events to process
        while ((wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) && 
               ((curEvent = (HidEvent *)wiced_hidd_event_queue_get_current_element(&kbAppState->eventQueue)) != NULL))
        {
            // Further processing depends on the event type
            switch (curEvent->eventType)
            {
                case HID_EVENT_KEY_STATE_CHANGE:
                    kbapp_procEvtKey();
                    break;
                case HID_EVENT_MOTION_AXIS_0:
                    kbapp_procEvtScroll();
                    break;
                case HID_EVENT_EVENT_FIFO_OVERFLOW:
                    // Call event queue error handler
                    kbapp_procErrEvtQueue();
                    break;
                default:
                    kbapp_procEvtUserDefined();
                    break;
            }
            
            // The current event should be deleted by the event processing function.
            // Additional events may also be consumed but we don't care about that
        }

        // Do idle rate processing
        kbapp_idleRateProc();
    }
}

void kbapp_procEvtUserDefinedKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    //doing nothing
}

void kbapp_procEvtUserDefined(void)
{
    // Deliberately nothing
}

/////////////////////////////////////////////////////////////////////////////////
/// This function processes keys from the event queue until the end of scan cycle event is seen.
/// During this process it accumulates changes to key reports. Once the end-of-scan-cycle
/// event is seen, it generates any modified reports. If any errors are detected
/// during the processing of events it calls one of the error handlers. Note that
/// if this function is called, there should be at least one event in the event queue
/////////////////////////////////////////////////////////////////////////////////
void kbapp_procEvtKey(void)
{
    uint8_t keyCode=KB_MAX_KEYS;
    uint8_t upDownFlag;
    uint8_t keyType, keyTranslationCode;
    HidEventKey *keyEvent;

    // Process events until we get an end-of cycle event
    // or an error (which doubles as an end-of-scan cycle event)
    // or we run out of events
    do
    {
        // Grab the next event. The first time we enter this loop we will have at least one
        // event. Subsequent iterations may run out of events
        if ((keyEvent = (HidEventKey *)wiced_hidd_event_queue_get_current_element(&kbAppState->eventQueue)) != NULL)
        {
            // Verify that the next event is a key event. Note that
            // an end of cycle key event is always present except when the event fifo overflows
            // We can assume that we have an overflow if the next event is not a key event
            if (keyEvent->eventInfo.eventType == HID_EVENT_KEY_STATE_CHANGE)
            {
                // Get the current key event and up/down flag
                upDownFlag = keyEvent->keyEvent.upDownFlag;
                keyCode = keyEvent->keyEvent.keyCode;

                // Check if we have a valid key
                if (keyCode < bthid_kbKeyConfig_size) //(keyCode < KB_MAX_KEYS)
                {
                    // This is a normal key event. Translate it to event type and tranlation code
                    keyType = kbKeyConfig[keyCode].type;
                    keyTranslationCode = kbKeyConfig[keyCode].translationValue;

                    // Depending on the key type, call the appropriate function for handling
                    // Pass unknown key types to user function
                    switch(keyType)
                    {
                        case KEY_TYPE_STD:
                            kbapp_stdRptProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_MODIFIER:
                            kbapp_stdRptProcEvtModKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_BIT_MAPPED:
                            kbapp_bitRptProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_SLEEP:
                            kbapp_slpRptProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_FUNC_LOCK:
                            kbapp_funcLockProcEvtKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_FUNC_LOCK_DEP:
                            kbapp_funcLockProcEvtDepKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                        case KEY_TYPE_NONE:
                            //do nothing
                            break;
                        default:
                            kbapp_procEvtUserDefinedKey(upDownFlag, keyCode, keyTranslationCode);
                            break;
                    }
                }
                // Check if we have an end of scan cycle event
                else if (keyCode == END_OF_SCAN_CYCLE)
                {
                    kbapp_txModifiedKeyReports();
                }
                else
                {
                    // Call error handler for all other events
                    kbapp_procErrKeyscan();
                    break;
                }
            }
            else
            {
                // We probably have event queue overflow. Call the event queue error handler
                kbapp_procErrEvtQueue();
                break;
            }

            // Delete the current event since we have consumed it
            wiced_hidd_event_queue_remove_current_element(&kbAppState->eventQueue);
        }
        else
        {
            // We ran out of events before we saw an end-of-scan-cycle event
            // Call error handler and manually exit the loop
            kbapp_procErrEvtQueue();
            break;
        }
    } while (keyCode < KB_MAX_KEYS);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles func lock key events. Func-lock events are ignored
/// during recovery and in boot mode. On func-lock down, it performs
/// the following actions:
///     - It toggles the func lock state and clears the toggleStateOnKeyUp flag
///       By default, func lock state will not be toggled when the key goes up
///       unless this flag is cleared. Typically, this flags is set if
///       a func-lock dependent key is detected while func-lock is down
///     - It updates the func lock report with the current func-lock state
///       but does not send it
/// On func-lock up, it performs the following actions:
///     - If the toggleStateOnKeyUp flag is set, it toggles func-lock state
///       and updates the func lock report with the new state and event flag.
///       It does not send the report
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode associated with the func-lock key. Unused
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    // Process the event only if we are not in recovery
    if (!kbAppState->recoveryInProgress && kbapp_protocol == HID_PAR_PROTOCOL_REPORT)
    {
        // Check if this is a down key or up key
        if (upDownFlag == KEY_DOWN)
        {
            // Only process further if we think the func-lock key state
            // is up
            if (kbAppState->funcLockInfo.kepPosition == FUNC_LOCK_KEY_UP)
            {
                // Flag that the func lock key is down
                kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_DOWN;

                // Toggle the func lock state and update the func lock report
                kbapp_funcLockToggle();

                // Clear the toggleStateOnKeyUp flag.
                kbAppState->funcLockInfo.toggleStateOnKeyUp = FALSE;
            }
        }
        else
        {
            // Key up. Only process further if we think the func-lock key state
            // is down
            if (kbAppState->funcLockInfo.kepPosition == FUNC_LOCK_KEY_DOWN)
            {
                // Flag that the func lock key is up
                kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_UP;

                // Check if we need to toggle func-lock
                if (kbAppState->funcLockInfo.toggleStateOnKeyUp)
                {
                    // Toggle the func lock state and update the func lock report
                    kbapp_funcLockToggle();
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles sleep key events. It updates the sleep report with
/// the new value of the sleep bit.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param slpBitMask location of the sleep bit in the sleep report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_slpRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t slpBitMask)
{
    // Check if this is a down key or up key
    if (upDownFlag == KEY_DOWN)
    {
        // Key down. Update report only if the key state has changed
        if (!(kbAppState->slpRpt.sleepVal & slpBitMask))
        {
            // Mark the appropriate key as down in the sleep report
            kbAppState->slpRpt.sleepVal |= slpBitMask;

            // Flag that the sleep report has changed
            kbAppState->slpRptChanged = TRUE;
        }
    }
    else
    {
        // Key up. Update report only if the key state has changed
        if (kbAppState->slpRpt.sleepVal & slpBitMask)
        {
            // Mark the appropriate key as up in the sleep report
            kbAppState->slpRpt.sleepVal &= ~slpBitMask;

            // Flag that the sleep report has changed
            kbAppState->slpRptChanged = TRUE;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function handles bit mapped key events. It updates the bit associated
/// with the key in the bit mapped key report.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param rowCol row/col of the associated bit in the report. The col is in the
///               last 3 bits and defines the bit offset while the row defines
///               the byte offset in the bit mapped report array.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_bitRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rowCol)
{
    uint8_t row, col, keyMask;

    // Bit translation table
    const uint8_t bitOffsetToByteValue[8] =
    {
        0x01, 0x02, 0x04, 0x08,
        0x10, 0x20, 0x40, 0x80
    };

    // Only process the key if it is in range. Since the row/col value comes from the user
    // we don't want a bad index to crash the system
    if (rowCol < kbAppConfig.numBitMappedKeys)
    {
        // Extract the row/col from the input argument
        row = (rowCol >> 3);
        col = (rowCol & 0x07);

        // Convert col to bit mask
        keyMask = bitOffsetToByteValue[col];

        // Check if this is a down key or up key
        if (upDownFlag == KEY_DOWN)
        {
            // Key down. Update the report only if the state of the key changed
            if (!(kbAppState->bitMappedReport.bitMappedKeys[row] & keyMask))
            {
                // The following code is funky because compiler will not allow a simple expression
                kbAppState->bitMappedReport.bitMappedKeys[row] |= keyMask;

                // Increment the number of keys in the bit report
                kbAppState->keysInBitRpt++;

                // Flag that the bit report has changed
                kbAppState->bitRptChanged = TRUE;
            }
        }
        else
        {
            // Key up.  Update the report only if the state of the key changed
            if (kbAppState->bitMappedReport.bitMappedKeys[row] & keyMask)
            {
                // The following code is funky because compiler will not allow a simple expression

                kbAppState->bitMappedReport.bitMappedKeys[row] &= ~keyMask;

                // Decrement the number of keys in the bit report
                kbAppState->keysInBitRpt--;

                // Flag that the bit report has changed
                kbAppState->bitRptChanged = TRUE;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////
/// This function retrieves the scroll event, combines it with
/// other scroll events if configured to do so and then generates reports as
/// necessary
/////////////////////////////////////////////////////////////////////////////////
void kbapp_procEvtScroll(void)
{
    HidEventMotionSingleAxis *scrollEvent;

    // Clear the scroll count
    kbAppState->scrollReport.motionAxis0 = 0;

    // Go through all scroll events
    while (((scrollEvent = (HidEventMotionSingleAxis *)wiced_hidd_event_queue_get_current_element(&kbAppState->eventQueue))!= NULL) &&
           (scrollEvent->eventInfo.eventType == HID_EVENT_MOTION_AXIS_0))
    {
        // Add new scroll value to the scroll report
        kbAppState->scrollReport.motionAxis0 += scrollEvent->motion;

        // We are done with this event. Delete it
        wiced_hidd_event_queue_remove_current_element(&kbAppState->eventQueue);

        // If report combining is not enabled, get out
        if (!kbAppConfig.scrollCombining)
        {
            break;
        }
    }

    // If the accumulated motion is non-zero, flag that the scroll report has not been sent
    if (kbAppState->scrollReport.motionAxis0)
    {
        kbAppState->scrollRptChanged = TRUE;
    }

    // Now transmit modified reports. This will generate and transmit scroll report when appropriate
    kbapp_txModifiedKeyReports();
}

////////////////////////////////////////////////////////////////////////////////
/// This function provides a standard response identical to kbAppStdErrorResponse.
/// In addition, it also performs the following actions:
///     - All pending events are flushed
///     - The keyscan HW is reset
/// This function is typically used when the FW itself is (or involved in) in error.
/// In such cases the FW no longer has the correct state of anything and we
/// must resort to a total reset
////////////////////////////////////////////////////////////////////////////////
void kbapp_stdErrRespWithFwHwReset(void)
{
    // Provide standard error response
    kbapp_stdErrResp();

    // Flush the event fifo
    wiced_hidd_event_queue_flush(&kbAppState->eventQueue);

#ifndef  TESTING_USING_HCI
    // Reset the keyscan HW
    wiced_hal_keyscan_reset();
    
    // Configure GPIOs for keyscan operation
    wiced_hal_keyscan_config_gpios();
#endif
}

////////////////////////////////////////////////////////////////////////////////
/// This function handles error events reported by the keyscan HW. Typically
/// these would be ghost events. This function calls stdErrResp() to
/// handle the error.
////////////////////////////////////////////////////////////////////////////////
void kbapp_procErrKeyscan(void)
{
    WICED_BT_TRACE("KSErr\n");
    kbapp_stdErrResp();
}

////////////////////////////////////////////////////////////////////////////////
/// This function handles event queue errors. This includes event queue overflow
/// unexpected events, missing expected events, and events in unexpected order.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem. A user defined implementation should at least
/// remove the first element in the queue if this event is an overflow event
////////////////////////////////////////////////////////////////////////////////
void kbapp_procErrEvtQueue(void)
{
    WICED_BT_TRACE("KSQerr\n");
    kbapp_stdErrRespWithFwHwReset();
}

////////////////////////////////////////////////////////////////////////////////
/// This function provides a standard response for errors. The response is:
///     - A rollover report is sent to the host if we are not already recovering from an error
///     - All reports are cleared and marked as modified. They will be sent once
///       we have recovered from the error.
///     - Marks the func-lock key as up but dow not toggle its state even if
///       the associated toggle flag is set. This allows for proper reconstruction
///       of the keyboard state including func-lock dependent keys after the recovery
///     - The recovery poll count is also set to the configured value.
///     - Connect button state is cleared since we don't know if the connect button press
///       is valid
////////////////////////////////////////////////////////////////////////////////
void kbapp_stdErrResp(void)
{
    // Clear all reports unconditionally
    kbapp_clearAllReports();

    // Mark the func-lock key as up.
    kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_UP;

    // Send a rollover report if
    //   - we are not already in the middle of a recovery OR
    //   - the rollover report generation period is non-zero and we have exceeded that period
    if (!kbAppState->recoveryInProgress)
    {
        // Send rollover report
        kbapp_stdRptRolloverSend();
    }

    // Reset recovery timeout
    kbAppState->recoveryInProgress = kbAppConfig.recoveryPollCount;

    // Mark all reports as not sent. This ensures that all reports will be sent once
    // the recovery is complete
    kbAppState->slpRptChanged = kbAppState->bitRptChanged = kbAppState->stdRptChanged = TRUE;

    // Assume connect button is now up
    kbapp_connectButtonHandler(CONNECT_BUTTON_UP);
}


/////////////////////////////////////////////////////////////////////////////////
/// This function sends a rollover report. It assumes that the rollover report
/// has already been initialized. It also snaps the current BT clock and
/// places it in keyboardAppData.rolloverRptTxInstant. This
/// can be used to reapeat the rollover report if desired
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptRolloverSend(void)
{
    // Tx rollover report
    WICED_BT_TRACE("RollOverRpt\n");

    wiced_bt_hidd_send_data(WICED_FALSE, HID_PAR_REP_TYPE_INPUT, (uint8_t *)&kbAppState->rolloverRpt, kbAppState->stdRptSize);

    // Snap the current BT clock for idle rate
    // rolloverRptTxInstant = wiced_hidd_get_current_native_bt_clock();
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles key events targetted for the standard key report.
/// It updates the standard report with the given event.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode information on how the scan code is translated to a reported value
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    // Processing depends on whether the event is an up or down event
    if (upDownFlag == KEY_DOWN)
    {
        kbapp_stdRptProcEvtKeyDown(upDownFlag, keyCode, translationCode);
    }
    else
    {
        kbapp_stdRptProcEvtKeyUp(upDownFlag, keyCode, translationCode);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles a key down event for the standard key report. It adds the
/// given key to the report if it is not already present.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode information on how the scan code is translated to a reported value
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    uint8_t i;

    // Check if the key is already in the report
    for (i=0; i < kbAppState->keysInStdRpt; i++)
    {
        if (kbAppState->stdRpt.keyCodes[i] == translationCode)
        {
            // It is. Ignore the event
            return;
        }
    }

    // Check if the std report has room
    if (i < kbAppConfig.maxKeysInStdRpt)
    {
        // Add the new key to the report
        kbAppState->stdRpt.keyCodes[i] = translationCode;

        // Update the number of keys in the report
        kbAppState->keysInStdRpt++;

        // Flag that the standard key report has changed
        kbAppState->stdRptChanged = TRUE;
    }
    else
    {
        // No room in report. Call error handler
        kbapp_stdRptProcOverflow();
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles a key up event for the standard key report. It removes
/// the key from the report if it is already present. Otherwise it does nothing.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode information on how the scan code is translated to a reported value
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    uint8_t i;

    // Find the key in the current standard report
    for (i=0; i < kbAppState->keysInStdRpt; i++)
    {
        if (kbAppState->stdRpt.keyCodes[i] == translationCode)
        {
            // Found it. Remvove it by replacing it with the last key and
            // reducing the key count by one. We can do this because the
            // order of keys in the report is not important.
            kbAppState->keysInStdRpt--;
            // The following code is funky because compiler will not allow a simple expression
            kbAppState->stdRpt.keyCodes[i] = kbAppState->stdRpt.keyCodes[kbAppState->keysInStdRpt];
            // Clear the last key
            kbAppState->stdRpt.keyCodes[kbAppState->keysInStdRpt] = 0;

            // Flag that the standard key report has changed
            kbAppState->stdRptChanged = TRUE;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles overflow of the standard key report. This happens when
/// more than 6 (or the configured number of) standard keys are pressed at a time.
/// This function does a FW/HW reset in response.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcOverflow(void)
{
    WICED_BT_TRACE("OverFlow\n");
    kbapp_stdErrRespWithFwHwReset();
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles modifier key events. It updates the modifier key bits
/// in the standard report structure.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode bitmap of the modifier key used for report generation
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptProcEvtModKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode)
{
    // Process the key event and update the modifier key bits in the standard report

    // Check if this is a down key or up key
    if (upDownFlag == KEY_DOWN)
    {
        // Key down. Update report only if the key state has changed
        if (!(kbAppState->stdRpt.modifierKeys & translationCode))
        {
            // Mark the appropriate modifier key as down
            kbAppState->stdRpt.modifierKeys |= translationCode;

            // Flag that the standard key report has changed
            kbAppState->stdRptChanged = TRUE;

            // Increment the number of mod keys that are down
            kbAppState->modKeysInStdRpt++;
        }
    }
    else
    {
        // Key up. Update report only if the key state has changed
        if (kbAppState->stdRpt.modifierKeys & translationCode)
        {
            // Mark the appropriate modifier key as down
            kbAppState->stdRpt.modifierKeys &= ~translationCode;

            // Flag that the standard key report has changed
            kbAppState->stdRptChanged = TRUE;

            // Decrement the number of mod keys that are down
            kbAppState->modKeysInStdRpt--;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits all modified key reports as long as we are not trying to
/// recover from an error. Note that it only transmits the standard report
/// in boot mode.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_txModifiedKeyReports(void)
{
    // Only transmit reports if recovery is not in progress
    if (!kbAppState->recoveryInProgress)
    {
        // Transmit standard report
        if (kbAppState->stdRptChanged)
        {
            kbapp_stdRptSend();
        }

        // Transmit the rest of the reports only in report mode
        if (kbapp_protocol == HID_PAR_PROTOCOL_REPORT)
        {
            // Transmit bit mapped report
            if (kbAppState->bitRptChanged)
            {
                kbapp_bitRptSend();
            }

            // Transmit sleep report
            if (kbAppState->slpRptChanged)
            {
                kbapp_slpRptSend();
            }

            // Transmit the func-lock report
            if (kbAppState->funcLockRptChanged)
            {
                kbapp_funcLockRptSend();
            }

            // Transmit scroll report
            if (kbAppState->scrollRptChanged)
            {
                kbapp_scrollRptSend();
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the scroll report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_scrollRptSend(void)
{
    // Flag that the scroll report has not changed since it was sent the last time
    kbAppState->scrollRptChanged = FALSE;
    WICED_BT_TRACE("ScrollRpt\n");

    wiced_bt_hidd_send_data(WICED_FALSE, HID_PAR_REP_TYPE_INPUT, (uint8_t *)&kbAppState->scrollReport, kbAppConfig.scrollReportLen);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the func-lock report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockRptSend(void)
{
    // Flag that the func-lock report has not changed since it was sent the last time
    kbAppState->funcLockRptChanged = FALSE;
    WICED_BT_TRACE("FuncLockRpt\n");

    wiced_bt_hidd_send_data(WICED_FALSE, HID_PAR_REP_TYPE_INPUT, (uint8_t *)&kbAppState->funcLockRpt, sizeof(KeyboardFuncLockReport));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the sleep report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_slpRptSend(void)
{
    // Flag that the sleep report has not changed since it was sent the last time
    kbAppState->slpRptChanged = FALSE;
    WICED_BT_TRACE("SleepRpt\n");

    wiced_bt_hidd_send_data(WICED_FALSE, HID_PAR_REP_TYPE_INPUT, (uint8_t *)&kbAppState->slpRpt, sizeof(KeyboardSleepReport));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the bit mapped report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_bitRptSend(void)
{
    // Flag that the bit mapped key report has not changed since it was sent the last time
    kbAppState->bitRptChanged = FALSE;
    WICED_BT_TRACE("BitRpt\n");

    wiced_bt_hidd_send_data(WICED_FALSE, HID_PAR_REP_TYPE_INPUT, (uint8_t *)&kbAppState->bitMappedReport, kbAppState->bitReportSize);
}


/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the battery report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_batRptSend(void)
{
    WICED_BT_TRACE("BASRpt\n");

    if (WICED_SUCCESS == wiced_bt_hidd_send_data(WICED_FALSE, 
                                            HID_PAR_REP_TYPE_INPUT, 
                                            (uint8_t *)&kbAppState->batRpt, 
                                            sizeof(KeyboardBatteryReport)))
    {
        wiced_hal_batmon_set_battery_report_sent_flag(WICED_TRUE);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the standard report over the interrupt channel and
/// marks internally that the report has been sent
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptSend(void)
{
    // Flag that the standard key report ha not changed since it was sent the last time
    kbAppState->stdRptChanged = FALSE;
    
    wiced_bt_hidd_send_data(WICED_FALSE, HID_PAR_REP_TYPE_INPUT, (uint8_t *)&kbAppState->stdRpt, kbAppState->stdRptSize);

    // Snap the current BT clock for idle rate
    kbAppState->stdRptTxInstant = wiced_hidd_get_current_native_bt_clock();
}

// Keyscan interrupt
void kbapp_userKeyPressDetected(void* unused)
{
    //WICED_BT_TRACE("kbapp_userKeyPressDetected\n");
    kbAppState->keyInterrupt_On = wiced_hal_keyscan_is_any_key_pressed();
    
    // Poll the app.
    kbapp_pollReportUserActivity();
}


// Scroll/Quadrature interrupt
void kbapp_userScrollDetected(void* unused)
{
    //WICED_BT_TRACE("kbapp_userScrollDetected\n");
    //Poll the app.
    kbapp_pollReportUserActivity();
}

void kbapp_stateChangeNotification(uint32_t newState)
{
    WICED_BT_TRACE("Transport state changed to %d\n", newState);

     //disable Shut Down Sleep (SDS) timed wake by default
    timedWake_SDS = WICED_FALSE;
    
    kbAppState->allowSDS = WICED_FALSE;

    //stop allow Shut Down Sleep (SDS) timer
    if (wiced_is_timer_in_use(&kb_allow_sleep_timer))
    {
        wiced_stop_timer(&kb_allow_sleep_timer);
    }

    kbapp_LED_off(BLEKB_LED_BLUE);

    if(newState == BTHIDLINK_CONNECTED)
    {
        //if connected, SDS timed wake must be used for uBCS mode
        timedWake_SDS = 1;

#ifndef  TESTING_USING_HCI
        // enable ghost detection
        wiced_hal_keyscan_enable_ghost_detection(TRUE);
#endif
        wiced_bt_hidd_link_enable_poll_callback(WICED_TRUE);

#if 0
        if(firstTransportStateChangeNotification == TRUE)
        {
            //Wake up from HID Off and already have a connection then allow HID Off in 1 second
            //This will allow time to send a key press.
            //To do need to check if key event is in the queue at lpm query
            wiced_start_timer(&kb_allow_sleep_timer,1000); // 1 second. timeout in ms
        }
        else
        {
            //We connected after power on reset or HID off recovery.
            //Start 20 second timer to allow time to setup connection encryption
            //before allowing HID Off/Micro-BCS.
            wiced_start_timer(&kb_allow_sleep_timer,20000); //20 seconds. timeout in ms
        }
#endif      
    }
    else if (newState == BTHIDLINK_DISCONNECTED)
    {    
        //allow SDS 
        wiced_start_timer(&kb_allow_sleep_timer,2000); // 2 seconds. timeout in ms

#ifndef  TESTING_USING_HCI
        // disable Ghost detection
        wiced_hal_keyscan_enable_ghost_detection(FALSE);
#endif

        // Tell the transport to stop polling
        wiced_bt_hidd_link_enable_poll_callback(WICED_FALSE);
    }
    else if (newState == BTHIDLINK_DISCOVERABLE)
    {
        kbapp_LED_on(BLEKB_LED_BLUE);
        // Tell the transport to stop polling
        wiced_bt_hidd_link_enable_poll_callback(WICED_FALSE);
    }

    if(firstTransportStateChangeNotification)
        firstTransportStateChangeNotification = FALSE;
}


/////////////////////////////////////////////////////////////////////////////////
/// This function clears all dynamic reports defined by the standard keyboard application
/// except the func-lock report. These are
///     - Standard report
///     - Bit mapped report
///     - Sleep report
///     - Pin code report
///     - Scroll report
/// The reports are also flagged as unchanged since last transmission.
/// The func-lock report is not cleared since it is a "state of func-lock" report
/// rather than the "state of func-lock key" report. It is prepared and sent
/// whenever the func-lock state changes and doesn't have the current state of the
/// func-lock key that needs to be cleared.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_clearAllReports(void)
{
    kbapp_stdRptClear();
    kbapp_bitRptClear();
    kbapp_slpRptClear();
    kbapp_pinRptClear();
    kbapp_scrollRptClear();

    // Flag that the reports have not been sent
    kbAppState->bitRptChanged = kbAppState->slpRptChanged = kbAppState->stdRptChanged =
        kbAppState->pinRptChanged = kbAppState->scrollRptChanged =
        kbAppState->funcLockRptChanged = FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// This function flushes all queued events and unprocessed fractional scroll
/// activity. It also clears all reports.
////////////////////////////////////////////////////////////////////////////////
void kbapp_flushUserInput(void)
{
    // Flush any partial scroll count
    kbAppState->scrollFractional = 0;

    // Flag that recovery is no longer in progress
    kbAppState->recoveryInProgress = 0;

    // Clear all dynamic reports
    kbapp_clearAllReports();

    // Flush the event fifo
    wiced_hidd_event_queue_flush(&kbAppState->eventQueue);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the led report.
/////////////////////////////////////////////////////////////////////////////////
void kbapp_ledRptInit(void)
{
    kbAppState->ledReport.reportID = kbAppConfig.ledReportID;
    kbAppState->ledReport.ledStates = kbAppConfig.defaultLedState;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the standard key report including internal count of
/// standard and modifier keys
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptClear(void)
{
    // Indicate that there are no keys in the standard report
    kbAppState->modKeysInStdRpt = kbAppState->keysInStdRpt = 0;

    // Initialize the std report completely
    kbAppState->stdRpt.reportID = kbAppConfig.stdRptID;
    kbAppState->stdRpt.modifierKeys = 0;
    kbAppState->stdRpt.reserved = 0;
    memset((void*)&kbAppState->stdRpt.keyCodes, 0, sizeof(kbAppState->stdRpt.keyCodes));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the rollover report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_stdRptRolloverInit(void)
{
    kbAppState->rolloverRpt.reportID = kbAppConfig.stdRptID;
    kbAppState->rolloverRpt.modifierKeys = 0;
    kbAppState->rolloverRpt.reserved = 0;
    memset((void*)&kbAppState->rolloverRpt.keyCodes,
           KEYRPT_CODE_ROLLOVER,
           sizeof(kbAppState->rolloverRpt.keyCodes));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the sleep report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_slpRptClear(void)
{
    // Initialize the sleep report completely
    kbAppState->slpRpt.reportID = kbAppConfig.sleepReportID;
    kbAppState->slpRpt.sleepVal = 0;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the bitmapped key report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_bitRptClear(void)
{
    // Indicate that there are no keys in the bit report
    kbAppState->keysInBitRpt = 0;

    // Initialize the bit mapped report completely
    kbAppState->bitMappedReport.reportID = kbAppConfig.bitReportID;
    memset((void*)&kbAppState->bitMappedReport.bitMappedKeys, 0, sizeof(kbAppState->bitMappedReport.bitMappedKeys));
}


/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the func-lock report. The header and report ID are
/// set and the status field is set based on the current state of func-lock
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockRptInit(void)
{
    // Set the report ID to the configured value
    kbAppState->funcLockRpt.reportID = kbAppConfig.funcLockReportID;

    // Set the current state of func-lock as well as the event flag
    kbAppState->funcLockRpt.status = kbAppState->funcLockInfo.state | 2;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles toggles the func lock state and updates the func-lock
/// report but doesn't send it. Note that it assumes that the func lock report
/// is sent in a specific format
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockToggle(void)
{
    // Toggle func lock state
    kbapp_funcLock_state = kbAppState->funcLockInfo.state = (kbAppState->funcLockInfo.state == FUNC_LOCK_STATE_OFF?
                                            FUNC_LOCK_STATE_ON:FUNC_LOCK_STATE_OFF);

    // Update the func lock report. Always set the func-lock event flag
    kbAppState->funcLockRpt.status = kbAppState->funcLockInfo.state | 2;

    // Mark the funct-lock report as changed
    kbAppState->funcLockRptChanged = TRUE;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles func-lock dependent key events. It uses the current
/// func-lock state to determine whether the key should be sent to the bit
/// key handler or the std key handler. Note that up keys
/// are sent to both handlers to ensure that 
/// up keys are not lost after a boot<->report protocol switch. Also
/// note that func-lock is assumed to be down in boot mode. Also
/// note that a down key set the func-lock toglle on key up flag 
/// unconditionally. This allows func-lock to be used as a temporary override of
/// its own state.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key 
/// \param funcLockDepKeyTableIndex index in the func-lock dependent key 
///        description table
/////////////////////////////////////////////////////////////////////////////////
void kbapp_funcLockProcEvtDepKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t funcLockDepKeyTableIndex)
{
    // Check if this is a down key or up key
    if (upDownFlag == KEY_DOWN)
    {
        // Check if we are in boot mode or the func-lock state is down
        if (kbAppState->funcLockInfo.state == FUNC_LOCK_STATE_ON ||
            kbapp_protocol == HID_PAR_PROTOCOL_BOOT_MODE)
        {
            // Pass this to the standard report handler
            kbapp_stdRptProcEvtKey(upDownFlag, keyCode, kbFuncLockDepKeyTransTab[funcLockDepKeyTableIndex].stdRptCode);
        }
        else
        {
            // Pass it to the bit report handler
            kbapp_bitRptProcEvtKey(upDownFlag, keyCode, kbFuncLockDepKeyTransTab[funcLockDepKeyTableIndex].bitRptCode);
        }

        // Flag that we had a func lock dependent key pressed. Note that this will only be used
        // if func-lock is down so we don't need to check for it.
        kbAppState->funcLockInfo.toggleStateOnKeyUp = TRUE;
    }
    else
    {
        // Key up. Send it to both the standard and bit mapped report handler
        kbapp_stdRptProcEvtKey(upDownFlag, keyCode, kbFuncLockDepKeyTransTab[funcLockDepKeyTableIndex].stdRptCode);
        kbapp_bitRptProcEvtKey(upDownFlag, keyCode, kbFuncLockDepKeyTransTab[funcLockDepKeyTableIndex].bitRptCode);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function initializes the scroll report. The header and report ID are
/// set and the rest of the report is set to 0
/////////////////////////////////////////////////////////////////////////////////
void kbapp_scrollRptClear(void)
{
    // Initialize the scroll report
    memset(&kbAppState->scrollReport, 0, sizeof(kbAppState->scrollReport));

    // Fill in the report ID information
    kbAppState->scrollReport.reportID = kbAppConfig.scrollReportID;

    // Flag that the scroll report has not changed since it was sent the last time
    kbAppState->scrollRptChanged = FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function clears the pin code entry report
/////////////////////////////////////////////////////////////////////////////////
void kbapp_pinRptClear(void)
{
    // Initialize the pin report
    kbAppState->pinReport.reportID = kbAppConfig.pinReportID;
    kbAppState->pinReport.reportCode = 0;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the pin code report over the interrupt channel
/////////////////////////////////////////////////////////////////////////////////
void kbapp_pinRptSend(void)
{
    // Flag that the pin report has not changed since it was sent the last time
    kbAppState->pinRptChanged = FALSE;
    
    // Queue the pin report for transmission through the authenticating transport
    wiced_bt_hidd_send_data(WICED_FALSE, HID_PAR_REP_TYPE_INPUT, (uint8_t *)&kbAppState->pinReport, sizeof(KeyboardPinEntryReport));
}

/////////////////////////////////////////////////////////////////////////////////
/// This function updates the pin report and flags it as changed since the last
/// transmission
/////////////////////////////////////////////////////////////////////////////////
void kbapp_pinRptUpdate(uint8_t pinEntryCode)
{
    // Update the report with the new code
    kbAppState->pinReport.reportCode = pinEntryCode;

    // Flag that the pin report has changed
    kbAppState->pinRptChanged = TRUE;
}

/////////////////////////////////////////////////////////////////////////////////
/// Process get current protocol request. Sends a data transaction over
/// the control channel of the given transport with the current protocol
/// \param none
/// \return current protocol
/////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_getProtocol(void)
{
    return kbapp_protocol;
}

////////////////////////////////////////////////////////////////////////////////
/// Handles set protocol from the host. Uses the default hid application function
/// for setting the protocol. In addition, if the protocol changes and the new protocol
/// is report, it:
///     - clears the bit mapped report
///     - clears the sleep report
///     - sets the func-lock key as up regardless of its current state
/// \param newProtocol requested protocol
/// \return HID_PAR_HANDSHAKE_RSP_SUCCESS
////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_setProtocol(uint8_t newProtocol)
{
    // Check if the protocol was changed and the new protocol is report
    if ((kbapp_protocol != newProtocol) && (newProtocol == HID_PAR_PROTOCOL_REPORT))
    {
        // Clear reports which are only sent in report mode. This ensures garbage is not sent 
        // after the mode switch
        kbapp_bitRptClear();
        kbapp_slpRptClear();
        kbapp_scrollRptClear();

        // Mark the func-lock key as up.
        kbAppState->funcLockInfo.kepPosition = FUNC_LOCK_KEY_UP;
    }

    kbapp_protocol = newProtocol;

    return HID_PAR_HANDSHAKE_RSP_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function implements the rxGetReport() function defined by
/// the HID application used to handle "Get Report" requests. 
/// \param hidTransport pointer to transport on which to send the response, if any
/// \param reportType type of the requested report, e.g. feature
/// \param reportId the report being requested
/// \return HID_PAR_HANDSHAKE_RSP_SUCCESS on success, and a DATA message will be sent out
///            HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM on failure.  It is assumed that the 
///             caller will generate an error response
/////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_getReport( uint8_t reportType, uint8_t reportId)
{
    uint8_t size;
    void *reportPtr = 0;

    // We only handle input/output reports. 
    if (reportType == HID_PAR_REP_TYPE_INPUT)
    {
        // Ensure that one of the valid keyboard input reports is being requested 
        // Also grab its length and pointer
        // Note that the configured size includes the DATA header
        // Remove it from the calculation. It will be added later
        if (reportId == kbAppConfig.stdRptID)
        {
            size = kbAppState->stdRptSize;
            reportPtr = &kbAppState->stdRpt;
        }
        else if (reportId == kbAppConfig.bitReportID)
        {
            size = sizeof(KeyboardBitMappedReport);
            reportPtr = &kbAppState->bitMappedReport;
        }
        else if (reportId == FUNC_LOCK_REPORT_ID)
        {
            size = sizeof(KeyboardFuncLockReport);
            reportPtr = &kbAppState->funcLockRpt;
        }
        else if (reportId == kbAppConfig.sleepReportID)
        {
            size = sizeof(KeyboardSleepReport);
            reportPtr = &kbAppState->slpRpt;
        }
        else if (reportId == BATTERY_REPORT_ID)
        {
            size = sizeof(KeyboardBatteryReport);
            reportPtr = &kbAppState->batRpt;
        }
    }
    else if (reportType == HID_PAR_REP_TYPE_OUTPUT)
    {
        // Ensure that one of the valid keyboard output reports is being requested
        // Also grab its length and pointer
        if (reportId == kbAppConfig.ledReportID)
        {
            size = sizeof(KeyboardLedReport);
            reportPtr = &kbAppState->ledReport;
        }
    }
    
    // We do not understand this, pass this to the base class.
    if (!reportPtr)
    {
        return HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
    }

    wiced_bt_hidd_send_data(WICED_TRUE, reportType , reportPtr, size);
    
    // Done!
    return HID_PAR_HANDSHAKE_RSP_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function processes an incoming LED output report. It verifies that the
/// report length is valid and then proceeds to update the internal state of the
/// keyboard LEDs. As the keyboard doesn't really have LEDs, this simply updates
/// the internal state of the LEDs
/// \param incomingLedReport the LED report to process
/// \param incomingLedReportSize size of the incoming report
/// \return HID_PAR_HANDSHAKE_RSP_SUCCESS if the report is processed correctly, TSC_ERR* on error
/////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_procLedRpt(KeyboardLedReport *incomingLedReport, 
                  UINT16 incomingLedReportSize)
{
    // Verify the report size
    if (incomingLedReportSize == sizeof(KeyboardLedReport))
    {
        // We are OK. Extract the LED states. Note that we dont access the 
        // reportId part of the report because it may have been skipped.
        kbAppState->ledReport.ledStates  = incomingLedReport->ledStates;
        WICED_BT_TRACE("KB LED report : %d\n", kbAppState->ledReport.ledStates);

#ifdef LED_USE_PWM
        if (kbAppState->ledReport.ledStates & 0x01)
        {
            kbapp_LED_on(BLEKB_PWM_LED1);
        }
        else
        {
            kbapp_LED_off(BLEKB_PWM_LED1);
        }
        
        if (kbAppState->ledReport.ledStates & 0x02)
        {
            kbapp_LED_on(BLEKB_PWM_LED2);
        }
        else
        {
            kbapp_LED_off(BLEKB_PWM_LED2);
        }
#else
        //CAPS LED
        if (kbAppState->ledReport.ledStates & 0x02)
        {
            kbapp_LED_on(BLEKB_LED_CAPS);
        }
        else
        {
            kbapp_LED_off(BLEKB_LED_CAPS);
        }
#endif

        // Done!
        return HID_PAR_HANDSHAKE_RSP_SUCCESS;
    }

    // Invalid length
    return HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function implements the SetReport function defined by
/// the HID application to handle "Set Report" messages.
/// This function looks at the report ID and passes the message to the
/// appropriate handler.
/// \param reportType type of incoming report, e.g. feature
/// \param payload pointer to data that came along with the set report request
///          including the report ID
/// \param payloadSize size of the payload including the report ID
/// \return handshake result code
/////////////////////////////////////////////////////////////////////////////////
uint8_t kbapp_setReport(uint8_t reportType, uint8_t *payload, uint16_t payloadSize)
{
    uint8_t result = HID_PAR_HANDSHAKE_RSP_SUCCESS;

    // We only handle output report types
    if (reportType == HID_PAR_REP_TYPE_OUTPUT)
    {
        // Pass to handler based on report ID. Ensure that report ID is in the payload
        if (payloadSize >= 2)
        {
            // Demux on report ID
            if(payload[0] == kbAppConfig.ledReportID)
            {

                return kbapp_procLedRpt((KeyboardLedReport *)payload, payloadSize);
            }
            else
            {
                result = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID;
            }
        }
        else
        {
            result = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
        }
    }
    else
    {
        result = HID_PAR_HANDSHAKE_RSP_ERR_UNSUPPORTED_REQ;
    }

    return result;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function implements the rxData function defined by
/// the HID application used to handle the "Data" message. 
/// The data messages are output reports. 
/// This function looks at the report ID and passes the message to the
/// appropriate handler. 
/// \param reportType reportType extracted from the header
/// \param payload pointer to the data message 
/// \param payloadSize size of the data message
/////////////////////////////////////////////////////////////////////////////////
void kbapp_rxData(uint8_t reportType, uint8_t *payload,  uint16_t payloadSize)
{
    // Demux on report type
    if (reportType == HID_PAR_REP_TYPE_OUTPUT)
    {
        // Pass to handler based on report ID. Ensure that report ID remains in the payload
        if((payload[0] == kbAppConfig.ledReportID) && (payloadSize >= 1))
        {
            kbapp_procLedRpt((KeyboardLedReport *)payload, payloadSize);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// This function demultiplexes pin/pass code entry based on the current valus of 
/// pinCodeEntryInProgress to either send out legacy HID report or notifies
/// the transport of a key event
////////////////////////////////////////////////////////////////////////////////
void kbapp_pinEntryEvent(uint8_t keyEntry)
{
    if((kbAppState->pinCodeEntryInProgress != PIN_ENTRY_MODE_NONE) 
       && (keyEntry < KEY_ENTRY_EVENT_MAX))
    {
        uint8_t newCode = pinCodeEventTransTab[kbAppState->pinCodeEntryInProgress & 0x01][keyEntry];
        
        switch(kbAppState->pinCodeEntryInProgress)
        {
        case LEGACY_PIN_ENTRY_IN_PROGRESS:
            if(newCode != PIN_ENTRY_EVENT_INVALID)
            {
                // Update the pin code report if it needs to be sent
                kbapp_pinRptUpdate(newCode);
            }
            break;
        case PASS_KEY_ENTRY_IN_PROGRESS:
            // Tell the transport about the key press event
            bthidlink_passCodeKeyPressReport(newCode);
            break;
        default:
            break;
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
/// This function provides pin code entry functionality on the keyboard. 
/// It processes all pending events in the event fifo and uses them to construct
/// the pin code. All non-key events will be thrown away, as well as any unrecognized keys
/// This function uses the translation code of each key and assumes that the translation
/// code will match the USB usage. The following USB usage codes are understood:
///     0-9, Enter, Key Pad Enter, Backspace, Delete (works like backspace), 
///     Escape (resets pin entry)
////////////////////////////////////////////////////////////////////////////////
void kbapp_handlePinEntry(void)
{
    HidEventKey *curEvent;
    BYTE upDownFlag, keyCode, usbUsageCode, numericKeyAsciiCode;

    // Process events until the FIFO is empty
    while (curEvent = (HidEventKey *)wiced_hidd_event_queue_get_current_element(&kbAppState->eventQueue))
    {
        // We only process key events here
        if (curEvent->eventInfo.eventType == HID_EVENT_KEY_STATE_CHANGE)
        {
            // Get the current key event
            upDownFlag = curEvent->keyEvent.upDownFlag;
            keyCode = curEvent->keyEvent.keyCode;

            // We only deal with key down events
            if (upDownFlag == KEY_DOWN &&  !kbAppState->enterKeyPressed )
            {
                // Only process it if it is a standard key
                if (kbKeyConfig[keyCode].type == KEY_TYPE_STD)
                {
                    // Translate into USB usage code
                    usbUsageCode = kbKeyConfig[keyCode].translationValue;

                    switch (usbUsageCode)
                    {
                        // Backspace and delete are handled the same way
                        case USB_USAGE_BACKSPACE:
                        case USB_USAGE_DELETE:
                            // Check if we have any accumulated digits
                            if (kbAppState->pinCodeSize)
                            {
                                // Kill the previous character
                                kbAppState->pinCodeSize--;

                                // Update the pin code report
                                kbapp_pinEntryEvent(KEY_ENTRY_EVENT_BACKSPACE);
                            }
                            break;
                        case USB_USAGE_ESCAPE:
                            // Clear the pin code buffer
                            kbAppState->pinCodeSize = 0;

                            // Update the pin code report
                            kbapp_pinEntryEvent(KEY_ENTRY_EVENT_RESTART);
                            
                            break;
                        case USB_USAGE_ENTER:
                        case USB_USAGE_KP_ENTER:
                            // Check if this is legacy pin entry mode
                            kbAppState->enterKeyPressed = usbUsageCode;
                            break;   
                        default:
                            // Handle the numbers in the default case

                            // Assume key is not valid
                            numericKeyAsciiCode = 0xff;

                            // 0 is special; handle it seperately
                            if (usbUsageCode == USB_USAGE_0 || usbUsageCode == USB_USAGE_KP_0)
                            {
                                numericKeyAsciiCode = 0;
                            }
                            // Check for keyboard 1-9
                            else if (usbUsageCode >= USB_USAGE_1 && usbUsageCode <= USB_USAGE_9)
                            {
                                numericKeyAsciiCode = usbUsageCode - USB_USAGE_1 + 1;
                            }
                            // Check for numpad 1-9
                            else if (usbUsageCode >= USB_USAGE_KP_1 && usbUsageCode <= USB_USAGE_KP_9)
                            {
                                numericKeyAsciiCode = usbUsageCode - USB_USAGE_KP_1 + 1;
                            }

                            // Check if we got a valid digit
                            if (numericKeyAsciiCode != 0xff)
                            {
                                // Add ASCII '0' to get the ASCII pin code
                                numericKeyAsciiCode += '0';

                                // Add it to the existing pin code if there is room
                                if (kbAppState->pinCodeSize < kbAppState->maxPinCodeSize)
                                {
                                    kbAppState->pinCodeBuffer[kbAppState->pinCodeSize] = numericKeyAsciiCode;
                                    kbAppState->pinCodeSize++;

                                    // Update the pin code report
                                    kbapp_pinEntryEvent(KEY_ENTRY_EVENT_CHAR);
                                }
                            }

                            // Done
                            break;
                    }
                }
            }
            else
            {
                 if (kbAppState->enterKeyPressed && (kbKeyConfig[keyCode].translationValue == kbAppState->enterKeyPressed))
                 {
                            if(kbAppState->pinCodeEntryInProgress == LEGACY_PIN_ENTRY_IN_PROGRESS)
                            {
                                // Pass the pin code on to the authenticating transport
                                bthidlink_pinCode(kbAppState->pinCodeSize, kbAppState->pinCodeBuffer);
                            }
                            else  // Has to be pass code entry mode
                            {
                                // Indicate end of pass code entry to peer
                                kbapp_pinEntryEvent(KEY_ENTRY_EVENT_STOP);

                                // Null terminate the buffer
                                kbAppState->pinCodeBuffer[kbAppState->pinCodeSize] = 0;

                                // Pass the pass key on to the authenticating transport
                                bthidlink_passCode(kbAppState->pinCodeSize, kbAppState->pinCodeBuffer);
                            }

                            // Flag that pin/pass code entry has completed
                            kbAppState->pinCodeEntryInProgress = PIN_ENTRY_MODE_NONE;

                            // Flush everything
                            kbapp_flushUserInput();

                            // Now get out of this function.
                            return;
                 }                            
            }
        }

        // We have consumed the current event
        wiced_hidd_event_queue_remove_current_element(&kbAppState->eventQueue);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// The keyboard application responds to a pin code entry request as follows:
/// - If user needs does not need to be prompted to enter pincode or user needs
///   a prompt and app is capable of prompting (through dosplay), flush any pending user input 
///   and enter pin code entry mode. This is done by setting the flag
///   pinCodeEntryInProgress to LEGACY_PIN_ENTRY_IN_PROGRESS. 
/// - Else it rejects the request and tells the BT transport to disconnect.
////////////////////////////////////////////////////////////////////////////////
void kbapp_enterPinCodeEntryMode(void)
{
    // If we are not already in some pin entry mode
    if(kbAppState->pinCodeEntryInProgress == PIN_ENTRY_MODE_NONE)
    {
        kbAppState->enterKeyPressed = 0;
    
        // Flush any pending user input
        kbapp_flushUserInput();
        
        // Clear any previous pin code
        kbAppState->pinCodeSize = 0;

        // Max pin code size is the size of the buffer
        kbAppState->maxPinCodeSize = MAX_PIN_SIZE;

        // Clear buffer
        memset(kbAppState->pinCodeBuffer, 0, MAX_PIN_SIZE);
        
        // Flag that pin code entry is in progress
        kbAppState->pinCodeEntryInProgress = LEGACY_PIN_ENTRY_IN_PROGRESS;
    }
    else
    {
        // Some pin code request pending, disconnect
        wiced_bt_hidd_link_disconnect();
    }
}

////////////////////////////////////////////////////////////////////////////////
/// The keyboard application responds to an exit pin code entry by clearing
/// the flag pinCodeEntryInProgress, i.e. setting it to 0. No other
/// action is taken. This method is safe to call anytime per the requirements
/// of this interface, even if pin code entry was never initiated.
////////////////////////////////////////////////////////////////////////////////
void kbapp_exitPinAndPassCodeEntryMode(void)
{
    // Flag that pin code entry is not in progress. This can be done any time
    // even when pin code entry was never initiated
    kbAppState->pinCodeEntryInProgress = PIN_ENTRY_MODE_NONE;
}

////////////////////////////////////////////////////////////////////////////////
/// The KB app responds to pass code request as follows:
/// - if no other pin/pass code request is pending, flush any pending user input 
///   and enter pin code entry mode. This is done by setting the flag
///   pinCodeEntryInProgress to PASS_KEY_ENTRY_IN_PROGRESS
/// - Else it rejects the request and tells the BT transport to disconnect.
////////////////////////////////////////////////////////////////////////////////
void kbapp_enterPassCodeEntryMode(void)
{
    // If we are not already in some pin entry mode
    if(kbAppState->pinCodeEntryInProgress == PIN_ENTRY_MODE_NONE)
    {
        kbAppState->enterKeyPressed = 0;
        
        // Flush any pending user input
        kbapp_flushUserInput();
        
        // Clear any previous pin code
        kbAppState->pinCodeSize = 0;

        // Max pass code size allowed is
        kbAppState->maxPinCodeSize = MAX_PASS_SIZE;

        // Clear buffer
        memset(kbAppState->pinCodeBuffer, 0, sizeof(kbAppState->pinCodeBuffer));

        // Flag that pass code entry is in progress
        kbAppState->pinCodeEntryInProgress = PASS_KEY_ENTRY_IN_PROGRESS;

        // Indicate pin entry start to the peer
        kbapp_pinEntryEvent(KEY_ENTRY_EVENT_START);
    }
    else
    {
        /// Some pin code request pending, disconnect
        wiced_bt_hidd_link_disconnect();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sleep permit query to check if sleep (normal or SDS) is allowed and sleep time
///
/// \param type - sleep poll type
///
/// \return   sleep permission or sleep time, depending on input param
////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t kbapp_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#ifndef  TESTING_USING_HCI
    // Check if keys are pressed before deciding whether sleep is allowed
    kbAppState->keyInterrupt_On = wiced_hal_keyscan_is_any_key_pressed();
#endif        

    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;  
            
            //if we are in the middle of kescan recovery, no sleep
            if (kbAppState->recoveryInProgress)
            {
                ret = 0;
            }               
                
            break;
            
        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
            
            if (!kbAppState->allowSDS)
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            
            //if key is not released, no Shut Down Sleep (SDS)
            if (kbAppState->keyInterrupt_On)
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }
            
            break;
            
    }

    return ret;
}


////////////////////////////////////////////////////////////////////////////////////
/// restore contents from Always On Memory. This should be called when wake up from SDS
///////////////////////////////////////////////////////////////////////////////////
void kbapp_aon_restore(void)
{
    if (!wiced_hal_mia_is_reset_reason_por())
    {
        kbAppState->funcLockInfo.state = kbapp_funcLock_state;
        wiced_bt_hidd_link_aon_action_handler(BTHIDLINK_RESTORE_FROM_AON);
    }
}



void kbapp_LED_init(void)
{
#ifdef LED_USE_PWM
    uint16_t init_value = 0x3FF - BLEKB_PWM_STEPS;
    uint16_t toggle_val = 0x3FF - 0;

    wiced_hal_gpio_configure_pin(BLEKB_PWM_LED1, GPIO_OUTPUT_ENABLE, 0);
    wiced_hal_gpio_configure_pin(BLEKB_PWM_LED2, GPIO_OUTPUT_ENABLE, 0);
    
    // base clock 24MHz,
    wiced_hal_aclk_enable(256000, ACLK1, ACLK_FREQ_24_MHZ);
    wiced_hal_pwm_start(BLEKB_PWM_LED1 - BLEKB_PWM_LED_BASE , PMU_CLK, toggle_val, init_value, 0);
    wiced_hal_pwm_start(BLEKB_PWM_LED2 - BLEKB_PWM_LED_BASE , PMU_CLK, toggle_val, init_value, 0);
#else
    if (wiced_hal_mia_is_reset_reason_por())
    {
        WICED_BT_TRACE("cold boot\n");
        wiced_hal_gpio_configure_pin(BLEKB_LED_CAPS, GPIO_OUTPUT_ENABLE, 1);
        wiced_hal_gpio_configure_pin(BLEKB_LED_BLUE, GPIO_OUTPUT_ENABLE, 1);
        wiced_hal_gpio_configure_pin(BLEKB_LED_GREEN, GPIO_OUTPUT_ENABLE, 0);    
        wiced_hal_gpio_configure_pin(BLEKB_LED_RED, GPIO_OUTPUT_ENABLE, 1);
    }
    else
        WICED_BT_TRACE("wake from SDS\n");

    //maintain LED state during SDS
    wiced_hal_gpio_slimboot_reenforce_cfg(BLEKB_LED_CAPS, GPIO_OUTPUT_ENABLE);
    wiced_hal_gpio_slimboot_reenforce_cfg(BLEKB_LED_BLUE, GPIO_OUTPUT_ENABLE);
    wiced_hal_gpio_slimboot_reenforce_cfg(BLEKB_LED_GREEN, GPIO_OUTPUT_ENABLE);
    wiced_hal_gpio_slimboot_reenforce_cfg(BLEKB_LED_RED, GPIO_OUTPUT_ENABLE);
#endif    
}


void kbapp_LED_on(uint8_t gpio)
{
#ifdef LED_USE_PWM
    /*
     * turn on LED by setting toggle value value to (0x3FF - BLEKB_PWM_STEPS/2)
     * BLEKB_PWM_STEPS(1000) to makes 48 kHz
    */
    uint16_t init_value = 0x3FF - BLEKB_PWM_STEPS;
    uint16_t toggle_val = 0x3FF - BLEKB_PWM_STEPS/10; // On(0): 10%, off(1):90%

    wiced_hal_pwm_change_values(gpio - BLEKB_PWM_LED_BASE, toggle_val,init_value);
#else    
    wiced_hal_gpio_set_pin_output(gpio, 0);
#endif    
}


void kbapp_LED_off(uint8_t gpio)
{
#ifdef LED_USE_PWM
    /*
     * turn off LED by setting toggle value to max(0x3FF)
     * GPIO will be set 0.
    */
    uint16_t init_value = 0x3FF - BLEKB_PWM_STEPS;
    uint16_t toggle_val = 0x3FF - 0;

    wiced_hal_pwm_change_values(gpio - BLEKB_PWM_LED_BASE, toggle_val,init_value);
#else    
    wiced_hal_gpio_set_pin_output(gpio, 1);
#endif    
}

#define WICED_HID_EIR_BUF_MAX_SIZE      264
////////////////////////////////////////////////////////////////////////////////
//Prepare extended inquiry response data.  Current version HID service.
////////////////////////////////////////////////////////////////////////////////
void kbapp_write_eir(void)
{
    uint8_t pBuf[WICED_HID_EIR_BUF_MAX_SIZE] = {0, };
    uint8_t *p = NULL;
    uint8_t length = strlen((char *) wiced_bt_hid_cfg_settings.device_name);

    p = pBuf;

    /* Update the length of the name (Account for the type field(1 byte) as well) */
    *p++ = (1 + length);
    *p++ = 0x09;            // EIR type full name

    /* Copy the device name */
    memcpy(p, wiced_bt_hid_cfg_settings.device_name, length);
    p += length;


    *p++ = ( 1 * 2 ) + 1;     // length of services + 1
    *p++ = 0x02;            // EIR type full list of 16 bit service UUIDs
    *p++ = UUID_SERVCLASS_HUMAN_INTERFACE & 0xff;
    *p++ = ( UUID_SERVCLASS_HUMAN_INTERFACE >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    wiced_bt_trace_array("EIR :", (uint8_t*) (pBuf + 1), MIN(p - (uint8_t*) pBuf, 100));
    wiced_bt_dev_write_eir(pBuf, (uint16_t) (p - pBuf));

    return;
}



