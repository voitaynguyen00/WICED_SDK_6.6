#!/usr/bin/python

from library.wiced_bt import wiced_bt_class
from library.common import *
import time


###############################################################################
# Global variables 
local_bda       = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]
test_result     = "FAIL"
carrier         = 1
tx_frequency    = 2402
mode            = 0
modulation_type = 0
tx_power        = -10


###############################################################################
# script main

print "---------- Running Continuous Tx Test ----------\n"

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

# Start LE Continuous Transmit
if( wiced.CommandTestLEContinuousTx( carrier, tx_frequency, mode, modulation_type, tx_power ) != "success" ):
    end_test( wiced, test_result )
    
if( wiced.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_test( wiced, test_result )

print "Waiting for 10 seconds, verify the device is transmitting via the Spectrum Analyzer\n"
time.sleep(10)

print "Stopping LE Continuous Tx test\n"

# Stop LE Continuous Transmit
if( wiced.CommandTestLEContinuousTx( 0, tx_frequency, mode, modulation_type, tx_power ) != "success" ):
    end_test( wiced, test_result )
    
if( wiced.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_test( wiced, test_result )


test_result = "PASS"

end_test( wiced, test_result )