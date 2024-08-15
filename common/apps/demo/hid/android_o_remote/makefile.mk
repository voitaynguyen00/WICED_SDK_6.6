#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
#  Corporation. All rights reserved. This software, including source code, documentation and  related 
# materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
#  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
# (United States and foreign), United States copyright laws and international treaty provisions. 
# Therefore, you may use this Software only as provided in the license agreement accompanying the 
# software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
# hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
# compile the Software source code solely for use in connection with Cypress's  integrated circuit 
# products. Any reproduction, modification, translation, compilation,  or representation of this 
# Software except as specified above is prohibited without the express written permission of 
# Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
# OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
# AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
# the Software without notice. Cypress does not assume any liability arising out of the application 
# or use of the Software or any product or circuit  described in the Software. Cypress does 
# not authorize its products for use in any products where a malfunction or failure of the 
# Cypress product may reasonably be expected to result  in significant property damage, injury 
# or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
#  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
# to indemnify Cypress against all liability.
#

NAME := android_o_remote

ENABLE_AUDIO = y
ENABLE_IR = n

APP_SRC = appinit_ble_remote.c
APP_SRC += wiced_bt_cfg.c
APP_SRC += ble_remote.c

APP_PATCHES_AND_LIBS += wiced_hidd_lib.a

#enable OTA Firmware Update
ifeq ($(OTA_SEC_FW_UPGRADE), 1)
APP_SRC += ecdsa256_pub.c
C_FLAGS += -DOTA_SECURE_FIRMWARE_UPGRADE
C_FLAGS += -DOTA_FIRMWARE_UPGRADE
C_FLAGS += -DOTA_SKIP_CONN_PARAM_UPDATE
$(NAME)_COMPONENTS += fw_upgrade_lib.a
INCS += ../../common/libraries/fw_upgrade_lib
endif

ifeq ($(OTA_FW_UPGRADE), 1)
C_FLAGS += -DOTA_FIRMWARE_UPGRADE
C_FLAGS += -DOTA_SKIP_CONN_PARAM_UPDATE
$(NAME)_COMPONENTS += fw_upgrade_lib.a
INCS += ../../common/libraries/fw_upgrade_lib
endif

#include HIDD library w/ LE HIDD only flag
APPLIBDIR := ../../common/libraries/hidd_lib
include $(APPLIBDIR)/makefile.mk

APP_SRC += $(LIB_SRC)
INCS += $(APPLIBDIR)

C_FLAGS += -DLE_HIDD_ONLY

# if 256 Kbyte/2M Bit Sflash is used, this flag needs to be set. Otherwise, 512 Kbyte/4M Bit Sflash is default. 
#C_FLAGS += -DSFLASH_SIZE_2M_BITS

ifeq ($(ENABLE_AUDIO), y)
 ifeq ($(CHIP), 20819)
  APP_PATCHES_AND_LIBS += adc_audio_lib.a
 endif
 
 #enable audio microphone
 C_FLAGS += -DSUPPORT_AUDIO
 
 #send audio data as 1 big gatt packet
 #C_FLAGS += -DATT_MTU_SIZE_180
 
 #enabled audio enhancement
 C_FLAGS += -DENABLE_ADC_AUDIO_ENHANCEMENTS
 
 #Press & Hold audio key to enable audio, release audio key to stop audio
 #C_FLAGS += -DAUDIO_KEY_PRESS_AND_HOLD

 #select encoder method
 ifeq ($(OPUS_CELT_ENCODER), 1)
  #use OPUS CELT encoder
  C_FLAGS += -DCELT_ENCODER
  ifeq (A_20819A1,$(BLD))
   APP_PATCHES_AND_LIBS += celt_lib.a
  endif
 else
  ifeq ($(ADPCM_ENCODER), 1)
   C_FLAGS += -DADPCM_ENCODER
   APP_PATCHES_AND_LIBS += adpcm_lib.a
  else
   #use mSBC encoder
   C_FLAGS += -DSBC_ENCODER
  endif
 endif 
 APP_SRC += android_voice.c
endif

ifeq ($(ENABLE_IR), y)
 #enable IR
 C_FLAGS += -DSUPPORT_IR
 
 APP_SRC += bleapp_appirtx.c 
endif

#start advertising when no connected on power up
#C_FLAGS += -DSTART_ADV_WHEN_POWERUP_NO_CONNECTED

#support advertising while connected
#C_FLAGS += -DCONNECTED_ADVERTISING_SUPPORTED

#auto reconnect when connection drops
#C_FLAGS += -DAUTO_RECONNECT

#endless LE advertising while disconnected and bonded
#C_FLAGS += -DENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED

#skip sending connect param request when received LE Connection Update Complete event w/ non prefered connection params
C_FLAGS += -DSKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED

#enable debug traces
C_FLAGS += -DWICED_BT_TRACE_ENABLE

#enable HCI traces (for debugging purpose)
#C_FLAGS += -DHCI_TRACES_ENABLED

ifeq ($(TESTING_USING_HCI), 1)
 #!!!only enabled for testing via HCI UART using ClientControl tool.
 #It is only for the case that the WICED BOARD you get doesn't have any GPIO pins 
 #exposed, you can't use any fly wire to connect to GPIOs to simulate key press 
 #and key release to test out the application. ClientControl tool can help you
 #simulate key press and release by sending key report via HCI to the WICED BOARD
 C_FLAGS += -DTESTING_USING_HCI
endif
