#!/usr/bin/python

from library.wiced_bt import wiced_bt_class
from library.common import *
import time


###############################################################################
# Global variables 
local_bda   = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]
data        = [0x4c, 0xfb, 0x5c, 0x2b, 0x02, 0xef]
data_verify = [0] * 6
test_result = "FAIL"
com_port    = "COM48"
hcd_file    = "hci_av_source_plus-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd"


###############################################################################
# script main 

print "---------- Running Ping Test ----------\n"

# Instantiate wiced_bt_class passing WICED COM port
wiced = wiced_bt_class( com_port )

if( wiced.started == "false" ):
    print "Failed to init COM port, quitting test\n"
    quit()    

# Download application image
if( wiced.download( hcd_file ) == "success" ):
    print "fw download SUCCESS\n"
else:
    print "fw download ERROR\n"
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
    
# Ping the 20707 device to check that it is still responsive
if( wiced.CommandMiscPingReq( data ) != "success" ):
    end_test( wiced, test_result )

# Get Ping Reqsponse
if( wiced.getWait( "EventMiscPingRes", 5 ) != "success" ):
    end_test( wiced, test_result )

# Retrieve the data
for i in range( len( data ) ):
    data_verify[i] = wiced.getData( "Data[%s]" % i )

# Check if data is correct
if( data == data_verify ):
    print "Data Matches\n"
    test_result = "PASS"
else:
    print "Data Does Not Match\n"
    
end_test( wiced, test_result )
