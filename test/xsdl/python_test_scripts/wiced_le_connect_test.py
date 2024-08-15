#!/usr/bin/python

from library.wiced_bt import wiced_bt_class
from library.common import *
import time


###############################################################################
# Global variables
local_bda   = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]
remote_bda  = [0x20, 0x70, 0x60, 0xbe, 0xef, 0x11]
peer_bda     = [0] * 6
test_result = "FAIL"
com_port    = "COM48"
hcd_file    = "hci_av_source_plus-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd"


###############################################################################
# script main

print "---------- Running LE Connect Test ----------\n"

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

# Start LE Scan
if( wiced.CommandLeScan( "start" ) != "success" ):
    end_test( wiced, test_result )

# Get LE status
if( wiced.getWait( "EventLeScanStatus", 5 ) != "success" ):
    end_test( wiced, test_result )

print "Waiting for LE Advertiment Reports\n"

adv_reports = 0

# Wait for desired Adv Data
peer_found = 0
for i in range( 50 ):
    # Get LE Adv Event
    if( wiced.getWait( "EventLeAdvertReport", 10 ) == "success" ):
        adv_reports = adv_reports + 1
        wiced.ProcessAdvData()
    else:
        if( adv_reports == 0 ):
            print "ERROR No Advertiment Reports received\n"
        else:
            print "Did not receive the adv data from [%02X:%02X:%02X:%02X:%02X:%02X]\n" % ( remote_bda[0], remote_bda[1], remote_bda[2], remote_bda[3], remote_bda[4], remote_bda[5] )
        end_test( wiced, test_result )

    print "peer_bda = [%02X:%02X:%02X:%02X:%02X:%02X]\n" % ( wiced.peer_bda[0], wiced.peer_bda[1], wiced.peer_bda[2], wiced.peer_bda[3], wiced.peer_bda[4], wiced.peer_bda[5] )
    print "rssi = %s\n" % wiced.rssi

    if( wiced.peer_bda == remote_bda ):
        print "SUCCESS found my device\n"
        #test_result = "PASS"
        peer_found = 1

        break
    else:
        print "Device Not Found, wait for more Adv Reports\n"

# Stop LE Scan
wiced.CommandLeScan( "stop" )

if( peer_found == 0 ):
    print "Peer device not found!\n"
    end_test( wiced, test_result )

# Connect to peer device
print "Connect to peer device\n"
if( wiced.CommandLeConnect( remote_bda ) != "success" ):
    print "fail - starting to Connect to peer device\n"
    end_test( wiced, test_result )

print "Waiting for GATT Command Status\n"
# Get GATT Command Status
if( wiced.getWait( "EventGattCommandStatus", 5 ) != "success" ):
    end_test( wiced, test_result )

# Get LE status
print "Waiting for Connection Reports\n"
if( wiced.getWait( "EventLeConnected", 10 ) == "success" ):
    print "success - LE Connect to peer device\n"

    # Get params from EventLeConnected
    AddrType = wiced.getData( "AddrType" )
    for i in range(6):
        peer_bda[ 5 - i ] = wiced.getData( "bda[%x]" %i )
    connection_handle = wiced.getData( "Handle" )
    Role = wiced.getData( "Role" )

    print "--EventLeConnected--"
    print "  [AddrType          = 0x%02X]  (0x00: public, 0x01: random)" % AddrType
    print "  [peer_bda          = %02X:%02X:%02X:%02X:%02X:%02X]" % ( peer_bda[0], peer_bda[1], peer_bda[2], peer_bda[3], peer_bda[4], peer_bda[5] )
    print "  [connection_handle = 0x%04X]" % connection_handle
    print "  [Role              = 0x%02X] (0x00: master/central, 0x01: slave/peripheral)\n" % Role

    if connection_handle == 0:
        print "Invalid Connection Handle\n"
        end_test( wiced, test_result )
    else:
        test_result = "PASS"
else:
    print "fail - Waiting for LE Connect Events\n"
    end_test( wiced, test_result )

# Get Pairing status
print "Waiting for Pairing Complete event"
if( wiced.getWait( "EventDevicePairingComplete", 20 ) == "success"):
    pairing_status = wiced.getData( "Status" )
    print "Pairing status = %s" % pairing_status 
    if( pairing_status != 0):
        print "Pairing Failed!"
        end_test( wiced, test_result )
else:
    print "fail - Waiting for LE Connect Events\n"
    end_test( wiced, test_result )

time.sleep( 5 )

if( wiced.CommandLeDisconnect( connection_handle ) != "success" ):
    print "fail - starting to Disconnect from peer device\n"
    test_result = "FAIL"
    end_test( wiced, test_result )

end_test( wiced, test_result )
