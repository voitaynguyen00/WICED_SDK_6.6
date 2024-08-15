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

number_of_rx_stats = 20

# Radio_Tx_Test HCI Command
radio_tx_test = [ 0x01, 0x51, 0xFC, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00 ]

tx_freq_mhz     = 2402
tx_channel      = tx_freq_mhz - 2402
modulation_type = 2 # 0: 0x00 8-bit Pattern, 1: 0xFF 8-bit Pattern, 2: 0xAA 8-bit Pattern, 3: 0xF0 8-bit Pattern, 4: PRBS9 Pattern
logical_channel = 1 # 0:ACL EDR, 1:ACL Basic
bb_packet_type  = 3 # 3: DM1
packet_length   = 17 
tx_power        = -3


# Radio_Rx_Test HCI Command
radio_rx_test = [ 0x01, 0x52, 0xFC, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ]

rx_channel = tx_channel


###############################################################################
# script main

print "---------- Running E2E BT Connectionless Tx/Rx Test ----------\n"

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

# Format radio_tx_test hci command
radio_tx_test[4:9] = local_bda1
radio_tx_test[10] = 0x1 # 1: Single Frequency
radio_tx_test[11] = tx_channel
radio_tx_test[12] = modulation_type
radio_tx_test[13] = logical_channel
radio_tx_test[14] = bb_packet_type
radio_tx_test[15] = packet_length & 0xff
radio_tx_test[16] = (packet_length >> 8) & 0xff
radio_tx_test[17] = 0x8 # 8: Specify power in dBm
radio_tx_test[18] = ( 256 + tx_power ) if ( tx_power < 0 ) else tx_power


# Send LE_Transmitter_Test command to 1st DUT
if( wiced1.CommandTestHCIEcapsulated( radio_tx_test ) != "success" ):
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

print "HCI HDR: [ %02x %02x %02x %02x %02x ]\n" % ( hci_event[0], hci_event[1], hci_event[2], hci_event[3], hci_event[4]  )
print "HCI Event Status: %02x\n" % hci_event[5]

hci_event_payload = []
if ( (length - 6) > 0 ):
    for i in range( length - 6):
         hci_event_payload.append( hci_event[i + 6] )
     
    print "HCI Event Payload: %s\n" % hci_event_payload
else:
    print "HCI Event has no payload other than the Command Complete Status\n"


# Format radio_rx_test hci command
radio_rx_test[4:9] = local_bda1
radio_rx_test[10] = 0xE8 # low byte of report perioe in ms (1sec = 1000ms, 0x03e8)
radio_rx_test[11] = 0x03 # high byte
radio_rx_test[12] = rx_channel
radio_rx_test[13] = modulation_type
radio_rx_test[14] = logical_channel
radio_rx_test[15] = bb_packet_type
radio_rx_test[16] = packet_length & 0xff
radio_rx_test[17] = (packet_length >> 8) & 0xff

# Send LE_Receiver_Test command to 2nd DUT
if( wiced2.CommandTestHCIEcapsulated( radio_rx_test ) != "success" ):
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

print "HCI HDR: [ %02x %02x %02x %02x %02x ]\n" % ( hci_event[0], hci_event[1], hci_event[2], hci_event[3], hci_event[4]  )
print "HCI Event Status: %02x\n" % hci_event[5]

hci_event_payload = []
if ( (length - 6) > 0 ):
    for i in range( length - 6):
         hci_event_payload.append( hci_event[i + 6] )
     
    print "HCI Event Payload: %s\n" % hci_event_payload
else:
    print "HCI Event has no payload other than the Command Complete Status\n"

# Wait 10 seconds for test to run
print "Waiting for Rx Statistics Report...\n"

count = 0
while( count < number_of_rx_stats ):
    
    # Wait for Command Complete Event
    if( wiced2.getWait( "EventTestEncapsulatedHCI", 5 ) != "success" ):
        end_e2e_test( wiced1, wiced2, test_result )
    
    length = wiced2.getData( "Length" )
    
    # Get hci event data from EventTestEncapsulatedHCI
    hci_event = []
    for i in range(length):
        hci_event.append( wiced2.getData( "hci_event[%s]" %i ) )
    
    if( (hci_event[0] == 0xff) & (hci_event[1] == 0x21) & (hci_event[2] == 0x07) ):
        print "Rx Statistics Report received:\n"
        print "    Sync_Timeout_Count:     0x%x\n" % (hci_event[3] + (hci_event[4] << 8) + (hci_event[5] << 16) + (hci_event[6] << 24))
        print "    HEC_Error_Count:        0x%x\n" % (hci_event[7] + (hci_event[8] << 8) + (hci_event[9] << 16) + (hci_event[10] << 24))
        print "    Total_Received_Packets: 0x%x\n" % (hci_event[11] + (hci_event[12] << 8) + (hci_event[13] << 16) + (hci_event[14] << 24))
        print "    Good_Packets:           0x%x\n" % (hci_event[15] + (hci_event[16] << 8) + (hci_event[17] << 16) + (hci_event[18] << 24))
        print "    CRC_Error_Packets:      0x%x\n" % (hci_event[19] + (hci_event[20] << 8) + (hci_event[21] << 16) + (hci_event[22] << 24))
        print "    Total_Received_Bits:    0x%x\n" % (hci_event[23] + (hci_event[24] << 8) + (hci_event[25] << 16) + (hci_event[26] << 24))
        print "    Good_Bits:              0x%x\n" % (hci_event[27] + (hci_event[28] << 8) + (hci_event[29] << 16) + (hci_event[30] << 24))
        print "    Error_Bits:             0x%x\n" % (hci_event[31] + (hci_event[32] << 8) + (hci_event[33] << 16) + (hci_event[34] << 24))
    
    count += 1

test_result = "PASS"

wiced1.CommandDeviceReset()
wiced2.CommandDeviceReset()

end_e2e_test( wiced1, wiced2, test_result )
