#!/usr/bin/python

from library.wiced_bt import wiced_bt_class
from library.common import *
import time


###############################################################################
# Global variables 
local_bda   = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]
test_result = "FAIL"
rx_channel  = 1


###############################################################################
# script main

print "---------- Running Receiver Test Test ----------\n"

# Instantiate wiced_bt_class passing WICED COM port
wiced = wiced_bt_class( "COM12" )

if( wiced.started == "false" ):
    print "Failed to init COM port, quitting test\n"
    quit()    
    
# Download application image
if( wiced.download( "hci_av_source_plus-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd" ) == "success" ):
    print "Firmware download SUCCESS\n"
else:
    print "Firmware download ERROR\n"
    end_test( wiced, test_result )

# Wait for Device Started Event
if( wiced.getWait( "EventDeviceStarted", 5 ) != "success" ):
    end_test( wiced, test_result )

time.sleep( 2 )

# Set local BD_ADDR
if( wiced.CommandDeviceSetLocalBDA( local_bda ) != "success" ):
    end_test( wiced, test_result )

if( wiced.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_test( wiced, test_result )

# Send LE Transmitter Test command
if( wiced.CommandTestLEReceiver( rx_channel ) != "success" ):
    end_test( wiced, test_result )

if( wiced.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_test( wiced, test_result )

print "Waiting for 10 seconds\n"
time.sleep(10)

print "Ending LE Receiver test\n"

# Send LE Test End command
if( wiced.CommandTestLEEnd() != "success" ):
    end_test( wiced, test_result )

if( wiced.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_test( wiced, test_result )

test_result = "PASS"

end_test( wiced, test_result )
