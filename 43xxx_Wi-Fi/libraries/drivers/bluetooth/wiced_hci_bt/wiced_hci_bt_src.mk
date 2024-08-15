#
# Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

BLUETOOTH_WICED_HCI_DIR :=  $(BLUETOOTH_ROOT_DIR)wiced_hci_bt/
$(info $(BLUETOOTH_WICED_HCI_DIR))
################################################################################
# Supported variants                                                           #
################################################################################

SUPPORTED_BT_CHIPS           := 20702B0 43341B0 43438A1

################################################################################
# Default settings                                                             #
################################################################################

COMPLETE_BT_CHIP_NAME := BCM$(BT_CHIP)$(BT_CHIP_REVISION)

$(NAME)_INCLUDES   += $(SOURCE_ROOT)libraries/protocols/wiced_hci \
                      $(BLUETOOTH_BTE_DIR)WICED \
                      $(BLUETOOTH_ROOT_DIR)include \
                      $(BLUETOOTH_BTE_DIR)Components/stack/include_wiced \
                      $(BLUETOOTH_BTE_DIR)Components/fw2/mpaf/components/stack/include \
                      $(BLUETOOTH_BTE_DIR)Components/gki/common \
                      $(BLUETOOTH_BTE_DIR)Projects/bte/main \
                      $(BLUETOOTH_WICED_HCI_DIR)internal


HCIBTSOURCES :=   $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_dm.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_ble.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_gatt.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_sdp.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_a2dp.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_hfp.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_avrcp.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_rfcomm.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_mesh.c



$(NAME)_SOURCES    :=      $(HCIBTSOURCES)
$(NAME)_COMPONENTS := libraries/protocols/wiced_hci


ifneq ($(IAR),)
# IAR toolchain

# Disable "pointless integer comparison" because macros are
# used to test integer variable ranges.
BTE_IAR_DIAG_SUPPRESSIONS += --diag_suppress Pa084

# Disable "typedef name has already been declared" because
# multiply defined typedefs are used extensively.
BTE_IAR_DIAG_SUPPRESSIONS += --diag_suppress Pe301

$(NAME)_CFLAGS   += $(BTE_IAR_DIAG_SUPPRESSIONS)
$(NAME)_CXXFLAGS += $(BTE_IAR_DIAG_SUPPRESSIONS)

endif
