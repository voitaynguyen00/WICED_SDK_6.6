#!/usr/bin/python

from library.wiced_bt import wiced_bt_class
from library.common import *
import time


###############################################################################
# Global variables 
local_bda   = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]
#remote_bda  = [0x8c, 0xde, 0x52, 0x1b, 0x8c, 0x5c]
remote_bda  = [0x5c, 0x8c, 0x1b, 0x52, 0xde, 0x8c]
peer_bda    = [0] * 6
test_result = "FAIL"
com_port    = "COM48"
hcd_file    = "hci_av_source_plus-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd"


###############################################################################
# script main 

print "---------- Running Inquiry Test ----------\n"

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

# Send Inquiry command
if( wiced.CommandDeviceInquiry( "start" ) != "success" ):
    end_test( wiced, test_result )

# Get Command Status Event
if( wiced.getWait( "EventDeviceCommandStatus", 10 ) != "success" ):
    end_test( wiced, test_result )

print "Waiting for Inquiry Reports\n"

inquiry_reports = 0

# Wait for desired Inquiry Data
for i in range(10):
    # Wait for Inquiry Result Event
    if( wiced.getWait( "EventDeviceInquiryResult", 10 ) == "success" ):
        inquiry_reports = inquiry_reports + 1
        wiced.ProcessAdvData()
    else:
        if( inquiry_reports == 0 ):
            print "ERROR No Inquiry Reports received\n"
        else:
            print "Did not receive the adv data from [%02X:%02X:%02X:%02X:%02X:%02X]\n" % ( remote_bda[0], remote_bda[1], remote_bda[2], remote_bda[3], remote_bda[4], remote_bda[5] )
        end_test( wiced, test_result )

    print "peer_bda = [%02X:%02X:%02X:%02X:%02X:%02X]\n" % ( wiced.peer_bda[0], wiced.peer_bda[1], wiced.peer_bda[2], wiced.peer_bda[3], wiced.peer_bda[4], wiced.peer_bda[5] )
    print "rssi = %s\n" % wiced.rssi

    if( wiced.peer_bda == remote_bda ):
        print "SUCCESS found my device\n"
        test_result = "PASS"
        break
    else:
        print "Device Not Found, wait for more Inquiry Reports\n"


end_test( wiced, test_result )
