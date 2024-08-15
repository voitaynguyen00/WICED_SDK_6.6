#!/usr/bin/python

from library.wiced_bt import wiced_bt_class
from library.common import *
import time


###############################################################################
# Global variables
local_bda_1   = [0x11, 0x22, 0x11, 0x22, 0x11, 0x22]
local_bda_2   = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]
remote_bda	  = [0x11, 0x22, 0x11, 0x22, 0x11, 0x22]
peer_bda      = [0] * 6
peer_found    = "false"
test_result   = "FAIL"
#Data          = [ 0x01 ]
Data          = [ 0x01, 0x02, 0x03, 0x04, 0x05 ]
visibility    = 0x01
log_file1 = "log_wiced_1_server.txt"
log_file2 = "log_wiced_2_client.txt"
com1 = "COM48"
com2 = "COM50"
hcd_file    = "hci_av_source_plus-CYW920706WCDEVAL-rom-ram-Wiced-release.hcd"


###############################################################################
# script main

print "---------- Running SPP Test ----------\n"

# Instantiate wiced_bt_class passing WICED COM port
wiced_1 = wiced_bt_class( com1, log_file=log_file1 )

if( wiced_1.started == "false" ):
    print "Failed to init COM port, quitting test\n"
    quit()
    
wiced_2 = wiced_bt_class( com2, log_file=log_file2 )

if( wiced_2.started == "false" ):
    print "Failed to init COM port, quitting test\n"
    quit()

# Download application image to first board
if( wiced_1.download( hcd_file ) == "success" ):
    print "Firmware download SUCCESS in board1\n"
else:
    print "Firmware download ERROR\n"
    end_e2e_test( wiced_1, wiced_2, test_result )

# Wait for Device Started Event
if( wiced_1.getWait( "EventDeviceStarted", 10 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

time.sleep( 2 )

# Set local BD_ADDR for the first board
if( wiced_1.CommandDeviceSetLocalBDA( local_bda_1 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

if( wiced_1.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

time.sleep( 1 )

# Download application image to second board
if( wiced_2.download( hcd_file ) == "success" ):
    print "Firmware download SUCCESS in board2\n"
else:
    print "Firmware download ERROR\n"
    end_e2e_test( wiced_1, wiced_2, test_result )

# Wait for Device Started Event
if( wiced_2.getWait( "EventDeviceStarted", 10 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

time.sleep( 2 )

# Set local BD_ADDR for the second board
if( wiced_2.CommandDeviceSetLocalBDA( local_bda_2 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

if( wiced_2.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

time.sleep( 2 )

# Set pairing mode in first board
print "set pairing mode as pairable in board 1\n"
if(wiced_1.CommandDeviceSetPairingMode( "true" ) != "success"):
    end_e2e_test( wiced_1, wiced_2, test_result )

time.sleep( 2 )

# Set pairing mode in second board
print "set pairing mode as pairable in board 2\n"
if(wiced_2.CommandDeviceSetPairingMode( "true" ) != "success"):
    end_e2e_test( wiced_1, wiced_2, test_result )
 
time.sleep( 2 )
    
# Set visibility in first board
if( wiced_1.CommandDeviceSetVisibility( visibility, visibility ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )
    
if( wiced_1.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )
 
time.sleep( 2 )

# Start BR/EDR scan from second board
# Send Inquiry command
if( wiced_2.CommandDeviceInquiry( "start" ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

# Get Command Status Event
if( wiced_2.getWait( "EventDeviceCommandStatus", 5 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )
 
print "Waiting for Inquiry Reports\n"
 
inquiry_reports = 0
 
# Wait for desired Inquiry Data
for i in range(50):
    # Wait for Inquiry Result Event
    if( wiced_2.getWait( "EventDeviceInquiryResult", 10 ) == "success" ):
        inquiry_reports = inquiry_reports + 1
        wiced_2.ProcessAdvData()
    else:
        if( inquiry_reports == 0 ):
            print "ERROR No Inquiry Reports received\n"
        else:
            print "Did not receive the adv data from [%02X:%02X:%02X:%02X:%02X:%02X]\n" % ( remote_bda[0], remote_bda[1], remote_bda[2], remote_bda[3], remote_bda[4], remote_bda[5] )
            end_e2e_test( wiced_1, wiced_2, test_result )
 
    print "peer_bda = [%02X:%02X:%02X:%02X:%02X:%02X]\n" % ( wiced_2.peer_bda[0], wiced_2.peer_bda[1], wiced_2.peer_bda[2], wiced_2.peer_bda[3], wiced_2.peer_bda[4], wiced_2.peer_bda[5] )
    print "rssi = %s\n" % wiced_2.rssi
 
    if( wiced_2.peer_bda == remote_bda ):
        print "SUCCESS found my device\n"
        peer_found = "true"
        wiced_2.CommandDeviceInquiry( "stop" )
        break
    else:
        print "Device Not Found, wait for more Inquiry Reports\n"
 
# If peer device is found, initiate a connection
if( peer_found != "true" ):
    end_e2e_test( wiced_1, wiced_2, test_result )
 
if( wiced_2.CommandSPPConnect( remote_bda ) != "success" ):     
    end_e2e_test( wiced_1, wiced_2, test_result )

# Get Command Status Event
if( wiced_2.getWait( "EventSPPConnected", 5 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )
	
# Get Connection Handle
ConnectionHandle = wiced_2.getData( "connection_handle" )
print "Handle value as got it from getData = %s\n" % ConnectionHandle
 
if ConnectionHandle == 0:
    print "Invalid Connection Handle\n"
    end_e2e_test( wiced_1, wiced_2, test_result )

time.sleep(3)

# Sending SPPTx Data
if(wiced_2.CommandSPPData( ConnectionHandle, Data )!="success"):
 	end_e2e_test( wiced_1, wiced_2, test_result )

# Waiting for the TX event
if( wiced_2.getWait( "EventSPPTXComplete", 10 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

# Get data on Rx side
if( wiced_1.getWait( "EventSPPRXData", 10 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

# Retrieve the value received from EventSPPRXData so we can compare to data that was transmitted
data_verify = []
length = wiced_1.getData( "Length" )
for i in range(length - 2):                  # -2 Bytes for the conn_handle
    data_verify.append( wiced_1.getData( "data[%s]" %i ) )

print "Received SPP data = " + str( data_verify ) + "\n"

# Disconnect the SPP connection
if( wiced_2.CommandSPPDisconnect( ConnectionHandle ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

# Wait for the Disconnection event
if( wiced_2.getWait( "EventSPPDisconnected", 5 ) != "success" ):
    end_e2e_test( wiced_1, wiced_2, test_result )

test_result = "PASS"

end_e2e_test( wiced_1, wiced_2, test_result )
