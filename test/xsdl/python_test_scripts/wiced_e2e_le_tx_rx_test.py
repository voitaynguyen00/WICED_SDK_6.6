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

# LE_Transmitter_Test HCI Command
le_transmitter_test = [0x01, 0x1E, 0x20, 0x03, 0x00, 0x00, 0x00]

tx_freq_mhz  = 2406
tx_channel   = (tx_freq_mhz - 2402) / 2
data_length  = 10
data_pattern = 4


# LE_Receiver_Test HCI Command
le_receiver_test = [0x01, 0x1D, 0x20, 0x01, 0x00 ]

rx_channel = tx_channel


# LE_Test_End
le_test_end = [0x01, 0x1F, 0x20, 0x00]


###############################################################################
# script main

print "---------- Running E2E LE Tx/Rx Test ----------\n"

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

# Format le_transmitter_test hci command
le_transmitter_test[4] = tx_channel
le_transmitter_test[5] = data_length
le_transmitter_test[6] = data_pattern

# Send LE_Transmitter_Test command to 1st DUT
if( wiced1.CommandTestHCIEcapsulated( le_transmitter_test ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

# Wait for Command Complete Event
if( wiced1.getWait( "EventTestEncapsulatedHCI", 5 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

length = wiced1.getData( "Length" )
print "Received hci event length = %d\n" % length

# Get hci event data from EventTestEncapsulatedHCI
hci_event = []
for i in range(length):
    hci_event.append( wiced1.getData( "hci_event[%s]" %i ) )

print "HCI HDR: [%02x %02x %02x %02x %02x ]\n" % ( hci_event[0], hci_event[1], hci_event[2], hci_event[3], hci_event[4]  )
print "HCI Event Status: %02x\n" % hci_event[5]

hci_event_payload = []
if ( (length - 6) > 0 ):
    for i in range( length - 6):
         hci_event_payload.append( hci_event[i + 6] )
     
    print "HCI Event Payload: %s\n" % hci_event_payload
else:
    print "HCI Event has no payload other than the Command Complete Status\n"


# Format le_receiver_test hci command
le_receiver_test[4] = rx_channel

# Send LE_Receiver_Test command to 2nd DUT
if( wiced2.CommandTestHCIEcapsulated( le_receiver_test ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )
    
# Wait for Command Complete Event
if( wiced2.getWait( "EventTestEncapsulatedHCI", 5 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

length = wiced2.getData( "Length" )
print "Received hci event length = %d\n" % length

# Get hci event data from EventTestEncapsulatedHCI
hci_event = []
for i in range(length):
    hci_event.append( wiced2.getData( "hci_event[%s]" %i ) )

print "HCI HDR: [%02x %02x %02x %02x %02x ]\n" % ( hci_event[0], hci_event[1], hci_event[2], hci_event[3], hci_event[4]  )
print "HCI Event Status: %02x\n" % hci_event[5]

hci_event_payload = []
if ( (length - 6) > 0 ):
    for i in range( length - 6):
         hci_event_payload.append( hci_event[i + 6] )
     
    print "HCI Event Payload: %s\n" % hci_event_payload
else:
    print "HCI Event has no payload other than the Command Complete Status\n"

# Wait 10 seconds for test to run
print "Running LE tx/rx test for 10 seconds...\n"

time.sleep( 10 )

# Send LE_Test_End command to 2nd DUT
print "Sending LE_Test_End command\n"
if( wiced2.CommandTestHCIEcapsulated( le_test_end ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )
    
# Wait for Command Complete Event
if( wiced2.getWait( "EventTestEncapsulatedHCI", 5 ) != "success" ):
    end_e2e_test( wiced1, wiced2, test_result )

length = wiced2.getData( "Length" )
print "Received hci event length = %d\n" % length

# Get hci event data from EventTestEncapsulatedHCI
hci_event = []
for i in range(length):
    hci_event.append( wiced2.getData( "hci_event[%s]" %i ) )

print "HCI HDR: [%02x %02x %02x %02x %02x ]\n" % ( hci_event[0], hci_event[1], hci_event[2], hci_event[3], hci_event[4]  )
print "HCI Event Status: %02x\n" % hci_event[5]

hci_event_payload = []
if( (length - 6) > 0 ):
    for i in range( length - 6):
         hci_event_payload.append( hci_event[i + 6] )
     
    print "HCI Event Payload: %s\n" % hci_event_payload
else:
    print "HCI Event has no payload other than the Command Complete Status\n"

num_packets = hci_event_payload[0] + ( hci_event_payload[1] << 8 )

print "Number of LE packets received = %d\n" % num_packets

if( num_packets > 0 ):
    test_result = "PASS"

wiced1.CommandDeviceReset()
wiced2.CommandDeviceReset()

end_e2e_test( wiced1, wiced2, test_result )
