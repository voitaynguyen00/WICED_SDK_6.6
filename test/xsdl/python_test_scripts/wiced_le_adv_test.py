#!/usr/bin/python

from library.wiced_bt import wiced_bt_class
from library.common import *
import time


###############################################################################
# Global variables 
local_bda   = [ 0x20, 0x70, 0x3A, 0x10, 0x19, 0x26 ]
test_result = "FAIL"
com_port    = "COM48"
hcd_file    = "hci_av_source_plus-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd"


###############################################################################
# script main

print "---------- Running LE Advertisement Test ----------\n"

# Instantiate wiced_bt_class passing WICED COM port
wiced = wiced_bt_class( com_port )

if( wiced.started == "false" ):
    print "Failed to init COM port, quitting test\n"
    quit() 

# Download application image
if( wiced.download( hcd_file ) == "success" ):
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

# Enable BLE Advertisements
if( wiced.CommandLeAdvertise( "true" ) != "success" ):
    end_test( wiced, test_result )

print "Waiting for LE Advertisement Events\n"

# Wait for LE Advert State Event
if( wiced.getWait( "EventLeAdvertState", 10 ) != "success" ):
    end_test( wiced, test_result )
    
print "Disabling LE Advertisements\n"

# Disable BLE Advertisements
if( wiced.CommandLeAdvertise( "false" ) != "success" ):
    end_test( wiced, test_result )

print "Waiting for LE Advertisement Events\n"

# Wait for LE Advert State Event
if( wiced.getWait( "EventLeAdvertState", 10 ) != "success" ):
    end_test( wiced, test_result )
    
test_result = "PASS"

end_test( wiced, test_result )
