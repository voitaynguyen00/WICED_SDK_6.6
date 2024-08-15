 Bluetooth Over the Air (OTA) Firmware Upgrade Peer Applications
 ---------------------------------------------------------------

  Applications to perform and verify OTA FW upgrade are provided for Windows 10 and Android platforms
  Executable and source code are provided
  Windows Peer App: WsOTAUpgrade.exe
  Android Peer App: LeOTAApp (app-debug.apk)
  Windows and Android applications can be used to perform and verify secure (signed) and unsecured (unsigned) OTA upgrade
  Both applications accept a secure (signed) or unsecured (unsigned) OTA binary images as input (*.ota.bin & *.ota.bin.signed)

 To demonstrate the app, work through the following steps.
 
 To perform OTA upgrade -
 1. Plug the WICED eval board into your computer
 2. Build and download the application (to the WICED board)
 3. Copy the mainapp_download.ota.bin to the same folder as WsOtaUpgrade.exe for your OS.
 4. Launch appropriate version of WsOtaUpgrade.exe located at
    ModusToolbox - C:\Users\<user name>\ModusToolbox_X.X\libraries\wiced_base-1.0\examples\BT-SDK\
    WICED Studio - C:\Users\<name>\Documents\WICED-Studio-X.X\
    \common\peer_apps\ota_firmware_upgrade\Windows\WsOtaUpgrade\Release\
 5. Use WsOtaUpgrade application to try over the air upgrade.  Pass the filename of the ota binary created in the build as
    the command-line argument C:..\> WsOtaUpgrade.exe mainapp_download.ota.bin
 6. Select the peer device and click on start button to initiate OTA process

 To perform Secure OTA upgrade -
 1. Use 'Change Application Settings...' menu to select OTA_SEC_FW_UPGRADE=1 
 2. Open a Command Prompt and change directory to ModusToolbox installation directory
    ModusToolbox - C:\Users\<user name>\ModusToolbox_X.X\tools\wiced-tools-1.0\BT\ecdsa256\bin
    WICED Studio - C:\Users\<name>\Documents\WICED-Studio-X.X\wiced_tools\ecdsa256\<OS>
 3. Generate private/public key pair by running ecdsa_genkey application
 4. Copy generated ecdsa256_pub.c to the application directory. For example
    C:\Users\<user name>\mtw\OTAFirmwareUpgrade_mainapp\secure\ecdsa256_pub.c
 5. Rebuild the project and download to the board.
 6. Copy generated mainapp_download.ota.bin file back to ecdsa256\bin (or ecdsa256\<OS>) directory. For example on ModusToolbox
    C:\Users\<user name>\ModusToolbox_X.X\tools\wiced-tools-1.0\BT\ecdsa256\bin\mainapp_download.ota.bin
 7. Sign the binary .ota.bin file with the public key using ecdsa_sign application in ecdsa256\bin (or ecdsa256\<OS>) folder
      >ecdsa_sign.exe mainapp_download.ota.bin
 Android Device -
   8. Connect Android phone to your PC and install LE OTA App
      ModusToolbox - C:\Users\<user name>\ModusToolbox_X.X\libraries\wiced_base-1.0\examples\BT-SDK\
      WICED Studio - C:\Users\<name>\Documents\WICED-Studio-X.X\
       \common\peer_apps\ota_firmware_upgrade\Android\LeOTAApp\app\build\outputs\apk>adb install -r app-debug.apk
   9. Copy signed ota image to your Android device
   10. Start LEOTAApp application on Android device
   11. Click on <Tap to select OTA image> and select the mainapp_download.ota.bin.signed that you copied in step 9.
   12. Click on <Tap to select a device> and select OTA FW Upgrade device
   13. Tap on Connect button
   14. Tap on Upgrade
 Windows -
   8. Copy the mainapp_download.ota.bin.signed to the same folder as WsOtaUpgrade.exe for your OS.
   9. Launch appropriate version of WsOtaUpgrade.exe located at
      \common\peer_apps\ota_firmware_upgrade\Windows\WsOtaUpgrade\Release\
   10. Use WsOtaUpgrade application to try over the air upgrade.  Pass the filename of the ota binary created in the build as
       the command-line argument C:..\> WsOtaUpgrade.exe mainapp_download.ota.bin.signed
   11. Select the peer device and click on start button to initiate OTA process

Note: On some older Bluetooth Devices on Windows, the OTA might fail because of improper MTU. To work around this,
change the code as below and rebuild the Windows application.
#define MAX_MTU 256 // (instead of 512)
in .\Windows\WsOtaUpgrade\WsOtaDownloader.cpp

 