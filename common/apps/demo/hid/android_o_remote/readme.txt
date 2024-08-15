-------------------------------------------------------------------------------
Android O Remote Control
-------------------------------------------------------------------------------

Overview
--------

The Android O Remote Control application is a single chip SoC compliant with HID over GATT Profile (HOGP).
Supported features include key, voice (Android TV Voice Service), Infrared Transmit (IR TX).

During initialization the app registers with LE stack, WICED HID Device Library and
keyscan and external HW peripherals to receive various notifications including
bonding complete, connection status change, peer GATT request/commands and
interrupts for key pressed/released, ADC audio.
Press any key will start LE advertising. When device is successfully bonded, the app
saves bonded host's information in the NVRAM.
When user presses/releases any key, a key report will be sent to the host.
On connection up or battery level changed, a battery report will be sent to the host.
When battery level is below shutdown voltage, device will critical shutdown.
When user presses microphone key, voice streaming starts until TV host stops it
When user presses and holds power key, IR TX starts until power key is released.

Features demonstrated
---------------------
 - GATT database and Device configuration initialization
 - Registration with LE stack for various events
 - Sending HID reports to the host
 - Processing write requests from the host
 - Low power management
 - Over the air firmware update (OTAFWU)

 Instructions
-------------
To demonstrate the app, walk through the following steps -
1. Plug the CYW920735EVB_01 board or the 20735B1 Remote Control HW into your computer
2. Build and download the application (to the EVAL board or Remote Control HW).
3. Power cycle the EVAL board or Remote Control HW.
4. Press any key to start LE advertising, then pair with a TV
   If using the CYW920735EVB_01 board, use a fly wire to connect GPIO P0 and P11 to simulate key '0' press,
    and remove the wire to simulate key release.
5. Once connected, it becomes the remote control of the TV.
6. If you have the 20735B1 Remote Control HW:
    - Press and hold microphone key, voice streaming starts until TV host stops it.

In case what you have is the WICED EVAL board, you can either use fly wire to connect to GPIOs to simulate key press and release.
Or using the ClientControl tool in the tools to simulate key press and release.
1. Plug the WICED EVAL board into your computer
2. Build and download the application (to the WICED board).
3. If failed to download due to device not detected, just repeat step 2 again.
4. Press any key to start LE advertising, then pair with a TV
    Use a fly wire to connect GPIO P0 and P11 to simulate key '0' press,
    and remove the wire to simulate key release.
5. Once connected, it becomes the remote control of the TV.


To use ClientControl tool + WICED EVAL board to simulate key press and release.
NOTE: Make sure you use "TESTING_USING_HCI=1" in application settings.
In ModusToolbox, select right click on app and select 'Change Application Settings'

1~3. same download procedure as above
5. Run ClientControl.exe
6. Select the "COM Port" in ClientControl tool window.
7. Press "Enter Pairing Mode"or "Connect" to start LE advertising, then pair with a PC or Tablet
8. Once connected, it becomes the remote control of the TV.
 - Select Interrupt channel, Input report, enter the contents of the report
   and click on the Send button, to send the report.  For example to send
   key down event when key '1' is pushed, report should be
   01 00 00 1e 00 00 00 00 00.  All keys up 01 00 00 00 00 00 00 00 00.
   Please make sure you always send a key up report following key down report.

Notes
-----
The application GATT database is located in wiced_bt_cfg.c
If you create a GATT database using Bluetooth Configurator, update the
GATT database in the location mentioned above.

Application Settings
--------------------
Application specific settings are -
TESTING_USING_HCI
    Use this option for testing with Bluetooth Profile Client Control. The Client Control
    UI can be used to provide input.
OTA_FW_UPGRADE
    Use this option for Over The Air (OTA) upgrade
OTA_SEC_FW_UPGRADE
    Use this option for secure OTA firmware upgrade. When this option is used
    the above option for OTA_FW_UPGRADE shuold also be used.
ENABLE_AUDIO
    Use this option to enable audio over HID.
ENABLE_IR
    Use this option to allow IR.
OPUS_CELT_ENCODER
    Use this option to enable Opus Celt encoder.
ADPCM_ENCODER
    Use this option to enable ADPCM encoder.
AUTO_RECONNECT
    Use this option to enable auto reconnect.
SKIP_PARAM_UPDATE
    Use this option to skip paramter update.
START_ADV_ON_POWERUP
    Use this option to enable motion.
ENABLE_CONNECTED_ADV
    Use this option to enable connected advertisement.
DISCONNECTED_ENDLESS_ADV
    Use this option to enable disconnected endless advertisement.
ASSYMETRIC_SLAVE_LATENCY
    Use this option to enable assymetric slave latency.

-------------------------------------------------------------------------------
