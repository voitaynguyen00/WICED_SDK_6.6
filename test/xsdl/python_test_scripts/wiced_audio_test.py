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
peer_found  = "false"
route       = "sine"
Freq        = "48"
Mode        = "stereo"
test_result = "FAIL"
com_port    = "COM48"
hcd_file    = "hci_av_source_plus-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd"


###############################################################################
# script main 

print "---------- Running Audio Test ----------\n"

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
    
# Enable Pairing
if( wiced.CommandDeviceSetPairingMode ( "true" ) != "success" ):
    end_test( wiced, test_result )

time.sleep( 2 )
    
# Attempt to connect
print "Attempting to connect to [%0.2x:%0.2x:%0.2x:%0.2x:%0.2x:%0.2x]\n" % ( remote_bda[0], remote_bda[1], remote_bda[2], remote_bda[3], remote_bda[4], remote_bda[5] )
    
if( wiced.CommandAudioConnect( remote_bda, route ) != "success" ):
    end_test( wiced, test_result )

# Get Command Status Event
if( wiced.getWait( "EventAudioCommandStatus", 5 ) != "success" ):
    end_test( wiced, test_result )

# Get Command Status Code
status = wiced.getData( "status" )

print "Received EventAudioCommandStatus status = %d\n" % status

# status of '0' means success
if( status != 0 ):
    end_test( wiced, test_result )

print "Waiting for Audio Connected Event\n"

# Wait for Audio Connect Event
if( wiced.getWait( "EventAudioConnect", 20 ) != "success" ):
    end_test( wiced, test_result )

# Get Connection Handle
Handle = wiced.getData( "Handle" )

if Handle == 0:
    print "Invalid Connection Handle\n"
    end_test( wiced, test_result )

# Start Audio Streaming
if( wiced.CommandAudioStart( Handle, Freq, Mode ) != "success" ):
    end_test( wiced, test_result )

# Wait for Audio Started Event
if( wiced.getWait( "EventAudioStarted", 5 ) != "success" ):
    end_test( wiced, test_result )

print "Streaming Audio for 10 seconds...\n"
time.sleep(10)

# Stop the Audio Streaming
if( wiced.CommandAudioStop( Handle ) != "success" ):
    end_test( wiced, test_result )
    
# Send Audio Disconnect Command
if( wiced.CommandAudioDisconnect( Handle ) != "success" ):
    end_test( wiced, test_result )

time.sleep(3)

test_result = "PASS"

end_test( wiced, test_result )
