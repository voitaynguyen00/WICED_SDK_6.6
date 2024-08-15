===============================================================================
Cypress WICED Studio Software Development Kit - README
===============================================================================

WICED Studio provides systems and APIs needed to build, design and implement
applications for Wi-Fi, Bluetooth Classic (BR/EDR), and Bluetooth low energy
(BLE) devices.

WICED Studio platforms include support for -
- 20706-A2, 20735-B1, and 20719-B1 based Bluetooth platforms.
- Support for various Cypress Wi-Fi & combo chips
   - 4390X (43909, 43907 and 43903) integrated MCU + Wi-Fi SoC
   - 4336X (43362, 43364) Wi-Fi SoC
   - 4343X (43438, 4343W) Wi-Fi SoC
   - 43340 Wi-Fi + Bluetooth combo SoC
   - 43012 Wi-Fi + Bluetooth combo SoC

Bluetooth Features:
- Bluetooth stack included the ROM.
- BT stack and profile level APIs for embedded BT application development.
- WICED HCI protocol to simplify host/MCU application development.
- APIs and drivers to access on board peripherals like SPI, UART, 
  ADC, PWM, Keyscan and IR HW blocks.
- Bluetooth protocols include GAP, GATT, SMP, RFCOMM, SDP,
  AVDTP, AVTCP and OBEX.
- Bluetooth profiles include A2DP source and sink, AVRCP controller
  and target, HF and AG, HID device and host, HOGP, PBAP, SPP.
- BR/EDR embedded sample apps for BT profiles.
- BLE embedded sample apps for BLE client and server, beacon, AMS, ANCS, 
  HID over GATT, Serial port profile over GATT.
- Support for Apple iAP2 protocol and sample applications.
- Support for BLE Mesh.
- MCU sample application running on host OS for controlling embedded
  BT applications over WICED HCI protocol.
- Sample apps for hardware interfaces including ADC, PUART, 
  GPIO, PWM.
- Support for Over-The-Air (OTA) upgrade.
- BTSpy application for viewing embedded BT application and 
  HCI protocol traces.
- Manufacturing tools to verify RF performance (mbt and wmbt). 
- WICED Bluetooth Designer wizard for quickly creating Bluetooth
  applications.
  (See Eclipse IDE menu -> File -> New -> WICED Bluetooth Designer)
- Segger J-Link debugger using J-Link GDB server and GCC GDB client.
- Test tool for automated testing. 

Wi-Fi Features:
  - Low-footprint embedded Wi-Fi Driver with Client (STA), softAP and Wi-Fi Direct
  - Wi-Fi <-> Bluetooth Internet Gateway 
  - Various RTOS/TCP stack options including
    - ThreadX/NetX (IPv4), ThreadX/NetX Duo (IPv6)
    - FreeRTOS/LwIP
  - Support for various Cypress Wi-Fi & combo chips
    - 4390X (43909, 43907 and 43903) integrated MCU + Wi-Fi SoC
    - 4336X (43362, 43364) Wi-Fi SoC
    - 4343X (43438, 4343W) Wi-Fi SoC
    - 43340 Wi-Fi + Bluetooth combo SoC
  - Support for various MCU host platforms
    - ST Microelectronics : STM32F2xx, STM32F4xx
    - Cypress: FM4
    - Atmel : AT91SAM4S16B
    - Freescale : K61
    - NXP : LPC17xx, LPC18xx
  - RTOS & Network abstraction layer with a simple API for UDP, TCP, HTTP, 
    HTTPS communications
  - SSL/TLS Security Library integrated with an HTTPS library for secure web 
    transactions
  - DTLS security library integrated with CoAP library
  - IoT protocols - HTTP/HTTPS, CoAP, AMQP v1.0 and MQTT
  - WICED Application Framework including Bootloader, OTA Upgrade and Factory Reset
    - Second flavor of OTA and Factory Reset (called OTA2)
      - OTA2 Failsafe - if OTA2 extraction is interrupted, the update will recover 
  - Automated Wi-Fi Easy Setup using one of several methods
    - SoftAP & Secure HTTP server
    - Wi-Fi Protected Setup
    - Apple Wireless Accessory Configuration (WAC) Protocol
  - Simple API to provide access to MCU peripherals including UART, SPI, I2C, 
    Timers, RTC, ADCs, DACs, etc
  - Support for multiple toolchains including GNU and IAR
  - Audio Applications for Internet streaming and BT/Wi-Fi Rebroadcast
  - Unified Bluetooth BTEWICED stack for Dual Mode and Low-Enery only modes 
  - Applications may pick the desired Bluetooth stack binary at link time.
  - Integrated Bluetooth embedded stack support for BCM20706A2 chips 
    (when used in conjunction with the BCM43907 SoC on CY943907WAE3 platform)
    - Support for A2DP, AVRCP, HFP and GATT profiles support with BCM20706A2 
      embedded Bluetooth stack on CY943907WAE3 platforms
  - BLE mesh gateway support to BIG for CYW43907WAE platform with BLE mesh library
    running on BCM20703A2 embedded mode
  - Enhanced AMQP to support SASL plain authentication to enable connectivity 
    to Azure cloud service
  - Verified PEAPv0 (with MSCHAPv2) and EAP-TLS Enterprise security protocol 
    with FreeRadius, IAS and ACS RADIUS servers
  - Added gSPI protocol support (slave) on BCM4390x
  - Implemented USB device mode support on BCM43907
  - Alexa Voice Service (AVS) API and sample application for custom Alexa Voice
    service applications using WICED HTTP 2.0 client library
  - Added XIP Support to 4390x/5490x platforms to allow instructions to be
    executed in sflash.


WICED Studio is structured as follows:
apps               : Example & Test Applications
doc                : API & Reference Documentation, Eval Board & Module Schematics
drivers            : Drivers for USB serial converter
include            : WICED API, constants, and defaults
libraries          : Bluetooth libraries, daemons, drivers, file systems, inputs,
                     and protocols
platforms          : Configuration files and information for supported hardware 
                     platforms
resources          : Binary and text based objects including scripts, images, 
                     and certificates
test               : Tools provided for automation testing
tools, wiced_tools : Build tools, compilers, programming tools etc.
WICED              : Core WICED components (Bluetooth, Wi-Fi, etc)
README.txt         : WICED Platform specifc README file
version.txt        : Version of WICED Studio


Getting Started
-------------------------------------------------------------------------------
If you are unfamiliar with WICED Studio, please refer to the WICED Studio
Quick Start Guide or Kit Guide for your WICED Platform under the Eclipse IDE 
'Project Explorer', located here: <WICED Platform>/doc/
For example: 
20706-A2_Bluetooth/doc/CYW920706WCDEVAL-Kit-Guide.pdf or 
43xxx_Wi-Fi/doc/WICED-QSG204.pdf
The WICED Studio Quick Start or Kit Guide documents the process to setup a computer 
for use with the WICED Studio IDE and WICED Evaluation Board. 

The currently active project, including associated support files and make
targets specific to each platfrom, may be switched using the "WICED Platform"
drop down menu of the Ecliipse IDE. 

Please see the  <WICED Platform>/README.txt for each WICED Platform to get 
detailed descriptions for each platform.

WICED Studio includes lots of sample applications in the <WICED Platform>/apps
directory.  See the <WICED Platform>/apps/README.txt for more detailed
descriptions of the apps.


Tools
-------------------------------------------------------------------------------
The GNU ARM toolchain is from Yagarto, http://yagarto.de

WICED Studio also supports ARM RealView 4.1 and above compiler toolchain:
http://www.arm.com

The standard WICED Evaluation board provides single USB-serial port for programming.

The debug interface is ARM Serial Wire Debug (SWD) and shares pins with download
serial lines TXd (SWDCLK) and RXd (SWDIO).

Building, programming and debugging of applications is achieved using either a 
command line interface or the WICED Studio IDE as described in the Quick Start
Guide or Kit Guide.
    

WICED Technical Support
-------------------------------------------------------------------------------
WICED support is available on the Cypress forum at 
https://community.cypress.com/welcome
Access to the WICED forum is restricted to bona-fide WICED customers only.

Cypress provides customer access to a wide range of additional 
information, including technical documentation, schematic diagrams, 
product bill of materials, PCB layout information, and software 
updates. Please contact your Cypress Sales or Engineering support 
representative or Cypress support at http://www.cypress.com/support.


Further Information
-------------------------------------------------------------------------------
Further information about WICED and the WICED Development System is
available on the WICED website at 
http://www.cypress.com/products/wireless-connectivity
or by contacting Cypress support at http://www.cypress.com/support
