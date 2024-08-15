#!/usr/bin/python

from library.wiced_bt import wiced_bt_class
from library.common import *
import time


###############################################################################
# Global variables 
local_bda1  = [ 0x20, 0x70, 0x60, 0xbe, 0xef, 0x01 ]
local_bda2  = [ 0x20, 0x70, 0x70, 0xbe, 0xef, 0x02 ]
test_result = "FAIL"
com_port1   = "COM48"
com_port2   = "COM50"
hcd_file    = "hci_av_source_plus-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd"


###############################################################################
# script main

print "---------- Running E2E LE Scan Test ----------\n"


print "Starting 1st device\n"

# Instantiate wiced_bt_class for 1st device
wiced1 = wiced_bt_class( com_port1, log_file="log1.txt" )

if( wiced1.started == "false" ):
    print "Failed to init COM port, quitting test\n"
    quit() 
    
# Download application image
if( wiced1.download( hcd_file ) == "success" ):
    print "Firmware download SUCCESS\n"
else:
    print "Firmware download ERROR\n"
    end_e2e_test( wiced1, wiced2, test_result )

# Wait for Device Started Event
if( wiced1.getWait( "EventDeviceStarted", 5 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

time.sleep( 2 )

# Set local BD_ADDR
if( wiced1.CommandDeviceSetLocalBDA( local_bda1 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

if( wiced1.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )
    
    
print "Starting 2nd device\n"

# Instantiate wiced_bt_class for 2nd device
wiced2 = wiced_bt_class( com_port2, log_file="log2.txt" )

if( wiced2.started == "false" ):
    print "Failed to init COM port, quitting test\n"
    end_e2e_test( wiced1, wiced2, test_result )   

# Download application image
if( wiced2.download( hcd_file ) == "success" ):
    print "Firmware download SUCCESS\n"
else:
    print "Firmware download ERROR\n"
    end_e2e_test( wiced1, wiced2, test_result )

# Wait for Device Started Event
if( wiced2.getWait( "EventDeviceStarted", 5 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

time.sleep( 2 )

# Set local BD_ADDR
if( wiced2.CommandDeviceSetLocalBDA( local_bda2 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

if( wiced2.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )


print "Configuring 1st device to enable BLE Advertisements\n"

# Enable BLE Advertisements
if( wiced1.CommandLeAdvertise( "true" ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

print "Waiting for LE Advertisement Events\n"

# Wait for LE Advert State Event
if( wiced1.getWait( "EventLeAdvertState", 10 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )


print "Configuring 2nd device to Scan for 1st device\n"

# Start LE Scan
if( wiced2.CommandLeScan( "start" ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

# Get LE status
if( wiced2.getWait( "EventLeScanStatus", 5 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )
    
print "Waiting for LE Advertiment Reports\n"

adv_reports = 0

# Wait for desired Adv Data
for i in range( 50 ):
    # Get LE Adv Event
    if( wiced2.getWait( "EventLeAdvertReport", 10 ) == "success" ):
        adv_reports = adv_reports + 1
        wiced2.ProcessAdvData()
    else:
        if( adv_reports == 0 ):
            print "ERROR No Advertiment Reports received\n"
        else:
            print "Did not receive the adv data from [%02X:%02X:%02X:%02X:%02X:%02X]\n" % ( local_bda1[0], local_bda1[1], local_bda1[2], local_bda1[3], local_bda1[4], local_bda1[5] )
        end_e2e_test( wiced1, wiced2, test_result )

    print "peer_bda = [%02X:%02X:%02X:%02X:%02X:%02X]\n" % ( wiced2.peer_bda[0], wiced2.peer_bda[1], wiced2.peer_bda[2], wiced2.peer_bda[3], wiced2.peer_bda[4], wiced2.peer_bda[5] )
    print "rssi = %s\n" % wiced2.rssi

    if( wiced2.peer_bda == local_bda1 ):
        print "SUCCESS found my device\n"
        test_result = "PASS"
        break
    else:
        print "Device Not Found, wait for more Adv Reports\n"

# Stop LE Scan
wiced2.CommandLeScan( "stop" )

end_e2e_test( wiced1, wiced2, test_result )
