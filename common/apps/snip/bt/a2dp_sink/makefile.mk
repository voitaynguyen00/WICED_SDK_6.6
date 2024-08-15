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

NAME := a2dp_sink

########################################################################
# Add Application sources here.
########################################################################

APP_SRC  = wiced_bt_cfg.c 
APP_SRC += a2dp_sink_main.c
APP_SRC += a2dp_sink.c

########################################################################
# Component(s) needed
########################################################################
$(NAME)_COMPONENTS   := a2dp_sink_profile.a

ifneq ($(CHIP), 43012)

ifeq ($(CHIP), 20703)
APP_PATCHES_AND_LIBS += wiced_audio_sink.a
else
#By default external Codec support is disabled in SDK
#Required libraries for external codec support is not available in this version of the SDK.
A2DP_EXT_CODEC=0

ifeq ($(A2DP_EXT_CODEC),1)
APP_PATCHES_AND_LIBS +=wiced_audio_sink_ext_lib.a
$(NAME)_COMPONENTS += AAC_codec_lib.a
else
APP_PATCHES_AND_LIBS += wiced_audio_sink_lib.a
endif
endif

endif

########################################################################
# C flags
########################################################################
C_FLAGS += -DWICED_BT_TRACE_ENABLE
C_FLAGS += -DA2DP_SINK_ENABLE_CONTENT_PROTECTION # Enables the content protection. Supports SCMS-T
ifneq ($(CHIP), 43012)
#C_FLAGS += -DWICED_COEX_ENABLE
endif

#AAC Passthrough flags. AAC Data can be routed either to the app or to the transport.
#C_FLAGS += -DA2DP_SINK_AAC_ENABLED

#Enable this flag, if external codec library is included
ifeq ($(A2DP_EXT_CODEC),1)
C_FLAGS += -DWICED_A2DP_EXT_CODEC=1
else
C_FLAGS += -DWICED_A2DP_EXT_CODEC=0
endif

########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################
