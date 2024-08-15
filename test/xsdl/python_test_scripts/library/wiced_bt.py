import os
import subprocess
import time

audio_route = { 'i2s': 0, 'uart': 1, 'sine': 2 }
audio_freq  = { '16': 0, '32': 1, '44.1': 2, '48': 3 }
audio_mode  = { 'mono': 0, 'stereo': 1 }

###############################################################################
#
# wiced_bt_class Python Class for Automation Testing
#
# - Assumes standard WICED COM Port for interfacing to BCM2070x
#
class wiced_bt_class:
    peer_bda = [0] * 6
    rssi     = 0
    started  = "true"

    def __init__( self, com, baudrate="115200", baudratedwnld="4000000", log="true", log_file="log.txt", flowcontrol="Yes" ):
        self.com_port = com
        self.baudrate = baudrate
        self.baudratedwnld = baudratedwnld
        
        if( flowcontrol == "No" ):
            self.flowcontrol = "-RTS 1 -CTS 0"
        else:
            self.flowcontrol = ""
        
        #Initialize com port
        if( log == "true" ):
            self.log_file = log_file
        else:
            self.log_file = None
        
        self.__open()
        
    def __open( self ):
        if( self.log_file != None ):
            if os.name == 'nt':
                self.subP = subprocess.Popen([r'..\bin\Win32\hwtalk.exe', "-PORT", self.com_port, "-BAUDRATE", self.baudrate, "-BAUDRATEDWNLD", self.baudratedwnld, "-MSGCMDDEFS", "..\wicedhcicmd.sdl", "-MSGEVTDEFS", "..\wicedhcievt.sdl", "-LOGTO", self.log_file, self.flowcontrol], shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE )
            else:
                self.subP = subprocess.Popen( [ "../bin/Win32/hwtalk.exe -PORT %s -BAUDRATE %s -BAUDRATEDWNLD %s -MSGCMDDEFS ../wicedhcicmd.sdl -MSGEVTDEFS ../wicedhcievt.sdl -LOGTO %s %s" % ( self.com_port, self.baudrate, self.baudratedwnld, self.log_file, self.flowcontrol ) ], shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE )
            
            time.sleep(2)
            f = open( self.log_file, 'r' )
            for line in f:
                if "Failed to open the transport connection" in line:
                    self.started = "false"
            f.close()
        else:
            if os.name == 'nt':
                self.subP = subprocess.Popen([r'..\bin\Win32\hwtalk.exe', "-PORT", self.com_port, "-BAUDRATE", self.baudrate, "-BAUDRATEDWNLD", self.baudratedwnld, "-MSGCMDDEFS", "..\wicedhcicmd.sdl", "-MSGEVTDEFS", "..\wicedhcievt.sdl", self.flowcontrol], shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE )
            else:
                self.subP = subprocess.Popen( [ "../bin/Win32/hwtalk.exe -PORT %s -BAUDRATE %s -BAUDRATEDWNLD %s -MSGCMDDEFS ../wicedhcicmd.sdl -MSGEVTDEFS ../wicedhcievt.sdl %s" % ( self.com_port, self.baudrate, self.baudratedwnld, self.flowcontrol ) ], shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE )        
        
    def write( self, command ):
        self.subP.stdin.write( "%s\n" % command )
        self.subP.stdin.flush()
        
        result = self.subP.stdout.readline().strip()
        self.subP.stdout.flush()
        
        return result

    def getData( self, data ):
        #print "Getting %s from received data\n" % data
        return int( self.write( "get %s" % data ) )
        
    def getWait( self, wait_for_event, timeout ):
        start_time = time.time()
	while( ( time.time() - start_time ) < timeout ):
            #print "Writing next\n"
            event = self.write( "next" )

            if ( event == wait_for_event ):
                print "EVT_SUCCESS: %s received!\n" % wait_for_event
                return "success"

        print "Timeout! Expected %s event not received\n" % wait_for_event
        return "timeout"
                
    def download( self, hcd_file ):
        print "Sending download command\n"
        status = self.write( "download %s" % hcd_file )
        
        if( status == "error" ):
            print "Firmware download error - retrying...\n"
            self.close()
            time.sleep(2)
            print "Reopening %s\n" % self.com_port
            self.__open()
            self.started = "true"
            print "Sending download command\n"
            status = self.write( "download %s" % hcd_file )
        return status

    def close( self ):
        print "Sending Exit command\n"
        self.subP.stdin.write( "exit\n" )
        self.started = "false"
    
    def CommandDeviceReset( self ):
        print "Sending CommandDeviceReset\n" 
        status = self.write( "send HCI.CommandDeviceReset" )
        return status

    def CommandDeviceSetPairingMode( self, command):
        print "Sending CommandDeviceSetPairingMode %s\n" % command
        if( command == "true" ):
            print "Enabling Pairing\n"
            status = self.write( "send HCI.CommandDeviceSetPairingMode PairingMode=1" )
        else:
            print "Disabling Pairing\n"
            status = self.write( "send HCI.CommandDeviceSetPairingMode PairingMode=0" )
        return status
            
    def CommandDeviceSetLocalBDA( self, bda ):
        print "sending CommandDeviceSetLocalBDA command\n"
        local_bda = "bda[0]=0x%x, bda[1]=0x%x, bda[2]=0x%x, bda[3]=0x%x, bda[4]=0x%x, bda[5]=0x%x" % ( bda[5], bda[4], bda[3], bda[2], bda[1], bda[0] )
        status = self.write( "send HCI.CommandDeviceSetLocalBDA %s" % local_bda )
        return status
    
    def CommandDeviceInquiry( self, command ):
        print "Sending CommandDeviceInquiry %s\n" % command
        if( command == "start" ):
            print "Starting Inquiry\n"
            status = self.write( "send HCI.CommandDeviceInquiry Cmd=1" )
        else:
            print "Stopping Inquiry\n"
            status = self.write( "send HCI.CommandDeviceInquiry Cmd=0" )
        return status
    
    def CommandAudioConnect( self, bda, route ):
        print "Sending CommandAudioConnect command\n"
        #remote_bda = "bda[0]=0x%x, bda[1]=0x%x, bda[2]=0x%x, bda[3]=0x%x, bda[4]=0x%x, bda[5]=0x%x" % ( bda[5], bda[4], bda[3], bda[2], bda[1], bda[0] )
        remote_bda = "bda[0]=0x%x, bda[1]=0x%x, bda[2]=0x%x, bda[3]=0x%x, bda[4]=0x%x, bda[5]=0x%x" % ( bda[0], bda[1], bda[2], bda[3], bda[4], bda[5] )
        status = self.write( "send HCI.CommandAudioConnect %s, Route=%s" % ( remote_bda, audio_route[ route ] ) )
        return status
    
    def CommandAudioDisconnect( self, Handle ):
        print "Sending CommandAudioDisconnect 0x%x\n" % Handle
        status = self.write( "send HCI.CommandAudioDisconnect Handle=0x%x" % Handle )
        return status
    
    def CommandAudioStart( self, Handle, Freq, Mode ):
        print "Sending CommandAudioStart 0x%x, %s, %s\n" % ( Handle, Freq, Mode )
        status = self.write( "send HCI.CommandAudioStart Handle=0x%x, Freq=%s, Mode=%s" % ( Handle, audio_freq[ Freq ], audio_mode[ Mode ] ) )
        return status    
    
    def CommandAudioStop( self, Handle):
        print "Sending CommandAudioStop 0x%x\n" % Handle
        status = self.write( "send HCI.CommandAudioStop Handle=0x%x" % Handle )
        return status
        
    def CommandLeScan( self, command, FltrDup=0 ):
        print "Sending CommandLeScan %s\n" % command
        if( command == "start" ):
            print "Starting BLE Scan\n"
            status = self.write( "send HCI.CommandLeScan Cmd=1, FltrDup=%s" % FltrDup )
        else:
            print "Stopping BLE Scan\n"
            status = self.write( "send HCI.CommandLeScan Cmd=0, FltrDup=%s" % FltrDup )
        return status
    
    def ProcessAdvData( self ):
        print "Retrieving bda from adv data\n"
        for i in range(6):
            #self.peer_bda[ 5 - i ] = int( self.write( "get bda[%s]" % i ) )
            self.peer_bda[ i ] = int( self.write( "get bda[%s]" % i ) )
        
        self.rssi = int( self.write( "get rssi" ) )
    
    def CommandLeAdvertise( self, command ):
        print "Sending CommandLeAdvertise %s\n" % command
        if( command == "true" ):
            print "Starting BLE Adv\n"
            status = self.write( "send HCI.CommandLeAdvertise Cmd=1" )
        else:
            print "Stopping BLE Adv\n"
            status = self.write( "send HCI.CommandLeAdvertise Cmd=0" )
        return status
        
    def CommandLeConnect( self, bda ):
        print "Sending CommandLeConnect\n"
        #remote_bda = "bda[0]=0x%x, bda[1]=0x%x, bda[2]=0x%x, bda[3]=0x%x, bda[4]=0x%x, bda[5]=0x%x" % ( bda[5], bda[4], bda[3], bda[2], bda[1], bda[0] )
        remote_bda = "bda[0]=0x%x, bda[1]=0x%x, bda[2]=0x%x, bda[3]=0x%x, bda[4]=0x%x, bda[5]=0x%x" % ( bda[0], bda[1], bda[2], bda[3], bda[4], bda[5] )
        status = self.write( "send HCI.CommandLeConnect AddrType=%s, %s" % ( 0, remote_bda ) )
        return status

    def CommandLeDisconnect( self, connection_handle ):
        print "Sending CommandLeDisconnect\n"
        status = self.write( "send HCI.CommandLeDisconnect connection_handle=%s" % ( connection_handle ) )
        return status

    def CommandGattDiscoverServices( self, connection_handle, start_handle, end_handle ):
        print "Sending CommandGattDiscoverServices\n"
        status = self.write( "send HCI.CommandGattDiscoverServices connection_handle=0x%x, start_handle=0x%x, end_handle=0x%x" % ( connection_handle, start_handle, end_handle ) )
        return status

    def CommandGattDiscoverCharacteristics( self, connection_handle, start_handle, end_handle ):
        print "Sending CommandGattDiscoverCharacteristics\n"
        status = self.write( "send HCI.CommandGattDiscoverCharacteristics connection_handle=0x%x, start_handle=0x%x, end_handle=0x%x" % ( connection_handle, start_handle, end_handle ) )
        return status

    def CommandGattDiscoverDescriptors( self, connection_handle, start_handle, end_handle ):
        print "Sending CommandGattDiscoverCharacteristics\n"
        status = self.write( "send HCI.CommandGattDiscoverDescriptors connection_handle=0x%x, start_handle=0x%x, end_handle=0x%x" % ( connection_handle, start_handle, end_handle ) )
        return status
        
    def CommandGattWriteCharacteristic( self, connection_handle, characteristic_handle, data=None ):
        print "Sending CommandGattWriteCharacteristic\n"
        if( data != None):
            write_data = ""
            write_data_len = len( data )
            for n in range( write_data_len - 1 ):
                write_data = write_data + ( "data[%s]=0x%x, " % ( n, data[n] ) )
            write_data = write_data + ( "data[%s]=0x%x" % ( n + 1, data[n + 1] ) )
        status = self.write( "send HCI.CommandGattWriteCharacteristic Length=%s, connection_handle=0x%x, characteristic_handle=0x%x, %s" % ( write_data_len + 4, connection_handle, characteristic_handle, write_data ) )
        return status

    def CommandGattWriteReqCharacteristic( self, connection_handle, characteristic_handle, data=None ):
        print "Sending CommandGattWriteReqCharacteristic\n"
        if( data != None):
            write_data = ""
            write_data_len = len( data )
            for n in range( write_data_len - 1 ):
                write_data = write_data + ( "data[%s]=0x%x, " % ( n, data[n] ) )
            write_data = write_data + ( "data[%s]=0x%x" % ( n + 1, data[n + 1] ) )
        status = self.write( "send HCI.CommandGattWriteReqCharacteristic Length=%s, connection_handle=0x%x, characteristic_handle=0x%x, %s" % ( write_data_len + 4, connection_handle, characteristic_handle, write_data ) )
        return status

    def CommandGattReadCharacteristic( self, connection_handle, characteristic_handle ):
        print "Sending CommandGattReadCharacteristic\n"
        status = self.write( "send HCI.CommandGattReadCharacteristic connection_handle=0x%x, characteristic_handle=0x%x" % ( connection_handle, characteristic_handle ) )
        return status

    def CommandSPPConnect(self,bda):
        print "Sending CommandSPPConnect command\n"
        #remote_bda = "bda[0]=0x%x, bda[1]=0x%x, bda[2]=0x%x, bda[3]=0x%x, bda[4]=0x%x, bda[5]=0x%x" % ( bda[5], bda[4], bda[3], bda[2], bda[1], bda[0] )
        remote_bda = "bda[0]=0x%x, bda[1]=0x%x, bda[2]=0x%x, bda[3]=0x%x, bda[4]=0x%x, bda[5]=0x%x" % ( bda[0], bda[1], bda[2], bda[3], bda[4], bda[5] )
        status = self.write( "send HCI.CommandSPPConnect %s" %remote_bda)
        return status
    
    def CommandSPPData(self, connection_handle, data=None):
        print "Sending CommandSPPData command\n"
        if( data != None):
            write_data = ""
            write_data_len = len( data )
            for n in range( write_data_len - 1 ):
                write_data = write_data + ( "data[%s]=0x%x, " % ( n, data[n] ) )
            if( write_data_len > 1 ):
                write_data = write_data + ( "data[%s]=0x%x" % ( n + 1, data[n + 1] ) )
            else:
                write_data = "data[0]=0x%x" % ( data[0] )
        
        status=self.write("send HCI.CommandSPPData Length=%s, connection_handle=0x%x, %s" % ( write_data_len + 2, connection_handle, write_data ))
        return status
    
    def CommandDeviceSetVisibility(self, discoverability, connectability):
        print "sending CommandDeviceSetVisibility command\n"
        status=self.write( "send HCI.CommandDeviceSetVisibility Discoverability=0x%x, Connectability=0x%x" % ( discoverability, connectability) )
        return status
    
    def CommandSPPDisconnect(self, connection_handle):
        print "Sending CommandSPPDisconnect command\n"
        status=self.write("send HCI.CommandSPPDisconnect connection_handle=%s" % connection_handle )
        return status

    def CommandTestLEReceiver( self, rx_channel ):
        print "Sending CommandTestLEReceiver %s\n" % rx_channel
        status = self.write( "send HCI.CommandTestLEReceiver rx_channel=%s" % rx_channel )
        return status
    
    def CommandTestLETransmitter( self, tx_channel, data_length, data_pattern ):
        print "Sending CommandTestLETransmitter %s %s %s\n" % ( tx_channel, data_length, data_pattern )
        status = self.write( "send HCI.CommandTestLETransmitter tx_channel=%s, data_length=%s, data_pattern=%s" % ( tx_channel, data_length, data_pattern ) )
        return status        
        
    def CommandTestLEEnd( self ):
        print "Sending CommandTestLEEnd\n"
        status = self.write( "send HCI.CommandTestLEEnd" )
        return status
    
    def CommandTestLEContinuousTx( self, carrier, tx_frequency, mode, modulation_type, tx_power ):
        print "Sending CommandTestLEContinuousTx %s %s %s %s %s\n" % ( carrier, tx_frequency, mode, modulation_type, tx_power )
        status = self.write( "send HCI.CommandTestLEContinuousTx carrier=%s, tx_frequency=%s, mode=%s, modulation_type=%s, tx_power=%s" % ( carrier, tx_frequency, mode, modulation_type, tx_power ) )
        return status

    def CommandTestLEReceiveOnly( self, rx_frequency ):
        print "Sending CommandTestLEReceiveOnly %s\n" % rx_frequency
        status = self.write( "send HCI.CommandTestLEReceiveOnly rx_frequency=%s" % rx_frequency )
        return status
    
    def CommandTestHCIEcapsulated( self, hci_command ):
        print "Sending CommandTestHCIEcapsulated command\n"
        
        hci_command_length = len( hci_command );
        
        encapsulated_command = ""
        for n in range( hci_command_length - 1 ):
            encapsulated_command = encapsulated_command + ( "hci_command[%s]=0x%x, " % ( n, hci_command[n] ) )
        encapsulated_command = encapsulated_command + ( "hci_command[%s]=0x%x" % ( n + 1, hci_command[n + 1] ) )
        
        status = self.write( "send HCI.CommandTestHCIEcapsulated Length=%s, %s" % ( hci_command_length, encapsulated_command ) )
        return status
        
    def CommandMiscPingReq( self, data=None ):
        print "Sending CommandMiscPingReq command\n"
        if( data != None):
            ping_data = ""
            ping_data_len = len( data )
            
            for n in range( ping_data_len - 1 ):
	        ping_data = ping_data + ( "data[%s]=0x%x, " % ( n, data[n] ) )
	    ping_data = ping_data + ( "data[%s]=0x%x" % ( n + 1, data[n + 1] ) )
            
            status = self.write( "send HCI.CommandMiscPingReq Length=%s, %s" % ( ping_data_len, ping_data ) )
        else:
            status = self.write( "send HCI.CommandMiscPingReq" )
            
        return status