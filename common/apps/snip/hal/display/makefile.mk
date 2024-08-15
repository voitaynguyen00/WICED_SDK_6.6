#
# $ Copyright Cypreess Semiconductor $
#

########################################################################
# Add Application sources here.
########################################################################
APP_SRC = display_demo.c
########################################################################
# these two drivers below support CY enhancements
# if you want to use other drivers, you have to port enhancements over
# compare original u8g_dev_ssd1351_128x96.c on u8g project file to see the changes
APP_SRC           += drivers/u8g_dev_ssd1351_128x128_cy.c #this driver is for Freetronic 128x128 display
#APP_SRC           += drivers/u8g_dev_ssd1351_128x96_cy.c #this driver is for NHD 128x96 display
#C_FLAGS += -DNHD # this is needed to call NHD's display driver (instead of Freetronic's) initialization
########################################################################

########################################################################
# Component(s) needed
########################################################################

APP_PATCHES_AND_LIBS += graphic_lib.a

########################################################################
# C flags
########################################################################

#C_FLAGS += -DWICED_SMART_READY=TRUE
C_FLAGS += -DWICED_BT_TRACE_ENABLE
C_FLAGS += -DDISPLAY_ONLY
#next flag enable pop-up screens with messages
C_FLAGS += -DPOP_UP_SIMUL
#next flag enable dimming of the stocks screen
C_FLAGS += -DCONTRAST_DEMO

#NEED_STD_LIBS=y

########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################


