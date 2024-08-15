/******************************************************************************
 *
 *  Copyright (C) 2013-2014 Cypress Semiconductor
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/
package com.broadcom.app.lebsgclient;

import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Random;
import java.util.UUID;

import com.broadcom.app.ledevicepicker.DeviceListFragment.Callback;
import com.broadcom.app.ledevicepicker.DevicePickerFragment;
import com.broadcom.app.lebsgclient.GattUtils.RequestQueue;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.os.Bundle;
import android.os.Handler;
import android.text.Layout;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;


class serial_gatt_state_t {
    byte    tx_credits;             // number of credits we have to send OTA data
    byte    rx_credits_max;         // maximum number of credits to give to peer
    byte    rx_credits;             // current number of credits to give to peer
    short   peer_mtu;               // negotiated MTU
    short   conn_id;                // connection ID to send data
    short   tx_len;                 // Length of the data in the TX buffer
    short   tx_offset;              // Offset of the data in the TX buffer
    byte[]  p_tx_buffer;            // buffer currently being transmitted
    long    bytes_rx;               // bytes received
    long    bytes_tx;               // bytes transmitted

    short   static_tx_len;          // Length of the data in the TX buffer
    short   count;                  // Count for thruput testing

    boolean mtu_configured;
}

/**
 * Main activity for the the BSG Client application
 */
/*vz
public class MainActivity extends Activity implements OnClickListener, Callback,
        OnItemSelectedListener {
        */
public class MainActivity extends Activity implements OnClickListener, Callback {
    private static final String TAG = Constants.TAG_PREFIX + "MainActivity";
    private static final String FRAGMENT_DEVICE_PICKER = "DevicePickerDialog";

    // Used to format timestamp for notification entries in the notification
    // view
    private static final SimpleDateFormat mFormatter = new SimpleDateFormat(
            "yyyy-MM-dd HH:mm:ss.SSS");

    private static final int SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT = 0x01;
    private static final int SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT    = 0x02;
    private static final int SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT   = 0x04;
    
    private static final int SERIAL_GATT_MAX_CREDITS                = 255;
    private static final int SERIAL_GATT_DEFAULT_MTU                = 23;
    private static final int SERIAL_GATT_REQUEST_MTU                = 158;
    private static final int SERIAL_THROUGHPUT_TEST_SIZE            = 1024;
    private static final int SERIAL_THROUGHPUT_TEST_MAX_ITERATIONS  = 1000;

    private int rx_credit = 0;
    private int tx_credit_max = 64;
    private int tx_credit = tx_credit_max;
    private int mtuUsed  = SERIAL_GATT_DEFAULT_MTU;
    private int lastNotificationState = 0;
    private boolean running = false;

    private int mBytesSent;
    private int mBytesReceived;

    // Create the Handler object (on the main thread by default)
    Handler handler = new Handler();
    // Define the code block to be executed
    private Runnable runnableCode = new Runnable() {
        @Override
        public void run() {
            mBytesReceived = 0;
            mBytesSent = 0;

            // Repeat this the same runnable code block again another 10 seconds
            handler.postDelayed(runnableCode, 10000);
        }
    };

    serial_gatt_state_t serial_gatt_state = new serial_gatt_state_t();

    char[] serial_gatt_dump_hex_string(byte[] data)
    {
        char[] hexArray = "0123456789ABCDEF".toCharArray();
        char[] hexChars = new char[data.length * 2];
        for ( int j = 0; j < data.length; j++ ) {
            int v = data[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }

        return hexChars;
    }


    private int serial_gatt_client_process_forward_data(byte[] p_buffer, short offset, short length)
    {
        if (serial_gatt_state.p_tx_buffer != null)
        {
            if (serial_gatt_state.static_tx_len != SERIAL_THROUGHPUT_TEST_SIZE) {
                Log.d(TAG, "error fwd data with buffer present");
                return 1;
            }
        }
        serial_gatt_state.p_tx_buffer = p_buffer;
        serial_gatt_state.tx_offset = offset;
        serial_gatt_state.tx_len = length;

        String s = new String(serial_gatt_dump_hex_string(p_buffer));
        Log.d(TAG, "serial_gatt_client_process_forward_data p_buffer " + s );
        Log.d(TAG, "serial_gatt_client_process_forward_data offset " + offset + " length " + length);

        return (serial_gatt_client_send_data(0));
    }

    private int serial_gatt_client_send_serial_data() {

        if ((serial_gatt_state.tx_credits > 0) && (serial_gatt_state.p_tx_buffer != null) && (serial_gatt_state.tx_len > 0)) {
            Log.d(TAG, "serial_gatt_client_send_data while tx_credits " + serial_gatt_state.tx_credits);
            Log.d(TAG, "serial_gatt_client_send_data while tx_len " + serial_gatt_state.tx_len);
            Log.d(TAG, "serial_gatt_client_send_data while tx_offset " + serial_gatt_state.tx_offset);
            Log.d(TAG, "serial_gatt_client_send_data while serial_gatt_state.peer_mtu " + serial_gatt_state.peer_mtu);

            // send up to peer MTU number of bytes
            short bytes_to_send;

            if (serial_gatt_state.rx_credits != 0) {
                // need to add 2 bytes header (flags and credits), leave 3 bytes for the ATT header
                if (serial_gatt_state.tx_len < (serial_gatt_state.peer_mtu - 5))
                    bytes_to_send = serial_gatt_state.tx_len;
                else
                    bytes_to_send = (short) (serial_gatt_state.peer_mtu - 5);

                serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 2] = (SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT | SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT);
                serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 1] = serial_gatt_state.rx_credits;

                int length = bytes_to_send + 2;
                byte[] value = new byte[length];
                for (int i = 0; i < length; i++) {
                    value[i] = serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 2 + i];
                }
                writeBSGConfigurationCharacteristic(value);

                serial_gatt_state.rx_credits = 0;
            } else {
                // need to add 1 bytes header (flags)
                if (serial_gatt_state.tx_len < (serial_gatt_state.peer_mtu - 4))
                    bytes_to_send = serial_gatt_state.tx_len;
                else
                    bytes_to_send = (short) (serial_gatt_state.peer_mtu - 4);

                serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 1] = SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT;

                int length = bytes_to_send + 1;
                byte[] value = new byte[length];
                for (int i = 0; i < length; i++) {
                    value[i] = serial_gatt_state.p_tx_buffer[serial_gatt_state.tx_offset - 1 + i];
                }
                writeBSGConfigurationCharacteristic(value);
            }

            serial_gatt_state.tx_credits--;
            serial_gatt_state.tx_len -= bytes_to_send;
            serial_gatt_state.tx_offset += bytes_to_send;

            Log.d(TAG, "serial_gatt_client_send_data while END bytes_to_send " + bytes_to_send);
            Log.d(TAG, "serial_gatt_client_send_data while END tx_credits " + serial_gatt_state.tx_credits);
            Log.d(TAG, "serial_gatt_client_send_data while END tx_len " + serial_gatt_state.tx_len);
            Log.d(TAG, "serial_gatt_client_send_data while END tx_offset " + serial_gatt_state.tx_offset);

            // if we are done with this buffer, send notification to the MCU and release the buffer
            if (serial_gatt_state.tx_len == 0) {
                if (serial_gatt_state.static_tx_len == SERIAL_THROUGHPUT_TEST_SIZE) {
                    if (serial_gatt_state.count == SERIAL_THROUGHPUT_TEST_MAX_ITERATIONS) {
                        serial_gatt_state.p_tx_buffer = null;
                    } else {
                        serial_gatt_state.count += 1;
                        Log.d(TAG, "serial_gatt_client_send_data serial_gatt_state.count " + serial_gatt_state.count);
                        serial_gatt_client_process_forward_data(serial_gatt_state.p_tx_buffer, (short) 2, serial_gatt_state.static_tx_len);
                    }
                } else
                    serial_gatt_state.p_tx_buffer = null;
            }
        }

        Log.d(TAG, "serial_gatt_client_send_data while tx_credits " + serial_gatt_state.tx_credits);

        return 0;
    }

    /*
* if we have credits and there is a tx buffer, send it now.  Return TRUE if buffer was sent out.
* Parameter return_credits is set to TRUE if it is time to send credits to peer
*/
    private int serial_gatt_client_send_data(int return_credits)
    {
        Log.d(TAG, "serial_gatt_client_send_data returncredits " + return_credits);

        // if tx buffer is empty, we may still need to send credits
        if (serial_gatt_state.p_tx_buffer == null)
        {
            if ((serial_gatt_state.rx_credits != 0) &&
                    ((return_credits > 0) || ((serial_gatt_state.rx_credits >= (serial_gatt_state.rx_credits_max / 2)))))
            {
                byte[] buffer =  new byte[2];

                buffer[0] = SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT;
                buffer[1] = serial_gatt_state.rx_credits;

                writeBSGConfigurationCharacteristic(buffer);
                serial_gatt_state.rx_credits = 0;
            }
            return 0;
        }

        serial_gatt_client_send_serial_data();
        return 0;
    }

    /**
     * Callback object that the LE Gatt service calls to report callback events
     * that occur
     *
     * @author fredc
     *
     */
    private class GattCallback extends BluetoothGattCallback {

        /**
         * Callback invoked by Android framework and a LE connection state
         * change occurs
         */
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            Log.d(MainActivity.TAG, "onConnectionStateChange(): address=="
                    + gatt.getDevice().getAddress() + ", status = " + status + ", state="
                    + newState);
            boolean isConnected = (newState == BluetoothAdapter.STATE_CONNECTED);

            boolean isOk = (status == 0);

            Log.d(MainActivity.TAG, "onConnectionStateChange(): isConnected ="
                    + isConnected + ", isOk = " + isOk );

            if (isConnected && isOk) {
                Log.d(TAG, "Request MTU");
                gatt.requestMtu(SERIAL_GATT_REQUEST_MTU); //SERIAL_GATT_DEFAULT_MTU
            }

            // If we got here, this is a disconnect with or without error
            // close gatt connection
            if (!isConnected) {
                Log.d(MainActivity.TAG, "onConnectionStateChange(): Connection and GATT closed");

                serial_gatt_state.mtu_configured = false;
                closeDevice();
                gatt.close();
                processConnectionStateChanged(false, false);
            }
        }

        /**
         * Callback invoked by Android framework when LE service discovery
         * completes
         */
        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {

            Log.d(TAG, "onServicesDiscovered");
            if (status != 0) {
                // Error occurred. close the ocnnection and return a
                // disconnected status
                gatt.close();
                try {
                    processConnectionStateChanged(false, true);
                } catch (Throwable t) {
                    Log.e(TAG, "error", t);
                }
            } else {
                try {
                    serial_gatt_state.tx_credits = 0;

                    serial_gatt_state.peer_mtu = (byte) 23;
                    if((byte)(1024 / serial_gatt_state.peer_mtu) < 20)
                        serial_gatt_state.rx_credits_max = (byte) (1024 / serial_gatt_state.peer_mtu);
                    else
                        serial_gatt_state.rx_credits_max = 20;

                    serial_gatt_state.rx_credits = serial_gatt_state.rx_credits_max;

                    processConnectionStateChanged(true, false);

                    RegisterBSGNotification(true);

                    serial_gatt_client_send_data(1);
                } catch (Throwable t) {
                    Log.e(TAG, "error", t);
                }
            }
        }

        /**
         * Callback invoked by Android framework when a characteristic read
         * completes
         */
        @Override
        public void onCharacteristicRead(BluetoothGatt gatt,
                                         BluetoothGattCharacteristic characteristic, int status) {
            if (status == 0) {
                try {
                    processCharacteristicRead(characteristic);
                } catch (Throwable t) {
                    Log.e(TAG, "error", t);
                }
            }
            mRequestQueue.next();// Execute the next queued request, if
        }

        /**
         * Callback invoked by Android framework when a descriptor read
         * completes
         */
        @Override
        public void onDescriptorRead(BluetoothGatt gatt, BluetoothGattDescriptor descriptor,
                                     int status) {
            /*
            if (status == 0) {
                try {
                    processDescriptorRead(gatt, descriptor);
                } catch (Throwable t) {
                    Log.e(TAG, "error", t);
                }
            }
            mRequestQueue.next();// Execute the next queued request, if any
            */
        }

        /**
         * Callback invoked by Android framework when a characteristic
         * notification occurs
         */
        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt,
                                            BluetoothGattCharacteristic characteristic) {
            try {
                processCharacteristicNotification(characteristic);
            } catch (Throwable t) {
                Log.e(TAG, "error", t);
            }

            mRequestQueue.next();// Execute the next queued request, if any
        }

        /**
         * Callback invoked by Android framework when a descriptor write
         * completes
         */
        @Override
        public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor,
                                      int status) {
            if (status == 0) {
                try {
                    /*vz
                    processDescriptorRead(gatt, descriptor);
                    */
                    processDescriptorWrite(gatt, descriptor);
                } catch (Throwable t) {
                    Log.e(TAG, "error", t);
                }
            }

            mRequestQueue.next();// Execute the next queued request, if any
        }

        /**
         * Callback invoked by Android framework when a descriptor write
         * completes
         */
        @Override
        public void onMtuChanged(BluetoothGatt gatt, int mtu, int status) {
            Log.d(TAG, "onMtuChanged  mtu = " + mtu);

            mtuUsed = mtu;

            if (status == 0) {
                serial_gatt_state.peer_mtu = (short) mtu;
                // Discover services, and return connection state = connected
                // after services discovered
                boolean isOk = gatt.discoverServices();

                if (isOk) {
                    return;
                }
                else {
                    gatt.close();
                    processConnectionStateChanged(false, false);
                }
/*
                try {

                    serial_gatt_state.peer_mtu = (short) mtu;
                    if((byte)(1024 / serial_gatt_state.peer_mtu) < 20)
                        serial_gatt_state.rx_credits_max = (byte) (1024 / serial_gatt_state.peer_mtu);
                    else
                        serial_gatt_state.rx_credits_max = 20;

                    serial_gatt_state.rx_credits = serial_gatt_state.rx_credits_max;

                    Log.d(TAG, "Send Rx credits to peer");
                    serial_gatt_client_send_data(1);

                    serial_gatt_state.mtu_configured = true;

                } catch (Throwable t) {
                    Log.e(TAG, "error", t);
                }
*/
            }

            mRequestQueue.next();// Execute the next queued request, if any
        }

        /**
         * Callback invoked by Android framework when a characteristic write
         * completes
         */
        @Override
        public void onCharacteristicWrite(BluetoothGatt gatt,
                                          BluetoothGattCharacteristic characteristic, int status) {
            super.onCharacteristicWrite(gatt, characteristic, status);
            if (status == 0) {
                try {
                    processCharacteristicWrite(characteristic);
                } catch (Throwable t) {
                    Log.e(TAG, "error", t);
                }
            }

            mRequestQueue.next();// Execute the next queued request, if any
        }
    }

    // UI Components
    // Device picker components
    private LinearLayout mButtonSelectDevice; // Button to start device picker
    private DevicePickerFragment mDevicePicker;
    private TextView mTextDeviceName; // Displays device's name
    private TextView mTextDeviceAddress; // Displays device's address

    // Connection components
    private Button mButtonConnect; // Button to connect to a device
    private Button mButtonDisconnect; // Button to connect from a device
    private TextView mTextConnectionState; // Displays current connection state

    // BSG service components
    private TextView mLabelBSGConfiguration;
    private EditText mEditTextBSGConfiguration;
    private Button mButtonWriteBSGConfiguration;
    private Button mButtonBSGStart;
    private TextView mLabelBSGThroughput;

    // Notification components
    private TextView mTextAreaNotification; // Displays notifications
    private Button mButtonClear; // Button to clear the notifications

    private final GattCallback mGattCallback = new GattCallback();
    private final RequestQueue mRequestQueue = GattUtils.createRequestQueue();
    private BluetoothAdapter mBtAdapter;
    private BluetoothDevice mPickedDevice;
    private BluetoothGatt mPickedDeviceGatt;
    private boolean mPickedDeviceIsConnected;
    private boolean mSyncNotificationSetting;

    /**
     * Helper function to show a toast notification message
     *
     * @param msg
     */
    private void showMessage(String msg) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
    }

    /**
     * Check Bluetooth is available and enabled, and initialize Bluetooth
     * adapter
     *
     * @return
     */
    private boolean checkAndInitBluetooth() {
        mBtAdapter = BluetoothAdapter.getDefaultAdapter();
        if (mBtAdapter == null || !mBtAdapter.isEnabled()) {
            return false;
        }
        return true;
    }

    /**
     * Initialize the device picker
     *
     * @return
     */
    private void initDevicePicker() {
        mDevicePicker = DevicePickerFragment.createDialog(this, null, true);
    }

    /**
     * Cleanup the device picker
     */
    private void cleanupDevicePicker() {
        if (mDevicePicker != null) {
            mDevicePicker = null;
        }
    }

    private void closeDevice() {
        if (mPickedDeviceGatt != null) {
            mPickedDeviceGatt.close();
            mPickedDeviceGatt = null;
        }
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Check bluetooth is available. If not, exit
        if (!checkAndInitBluetooth()) {
            showMessage(getString(R.string.error_bluetooth_not_available));
            finish();
            return;
        }

        // Initialize the UI components, and register a listeners
        setContentView(R.layout.main);

        // Load device picker components
        mButtonSelectDevice = (LinearLayout) findViewById(R.id.btn_selectdevice);
        mButtonSelectDevice.setOnClickListener(this);
        mTextDeviceName = (TextView) findViewById(R.id.deviceName);
        mTextDeviceAddress = (TextView) findViewById(R.id.deviceAddress);

        // Load connection components
        mButtonConnect = (Button) findViewById(R.id.btn_connect);
        mButtonConnect.setOnClickListener(this);
        mButtonDisconnect = (Button) findViewById(R.id.btn_disconnect);
        mButtonDisconnect.setOnClickListener(this);
        mTextConnectionState = (TextView) findViewById(R.id.connectionState);

        mLabelBSGConfiguration = (TextView) findViewById(R.id.label_bsg_configuration);
        mLabelBSGConfiguration.setText(getString(R.string.label_bsg_configuration,
                Constants.BSG_CHARACTERISTIC_CONFIGURATION_UUID));
        mEditTextBSGConfiguration = (EditText) findViewById(R.id.value_bsg_configuration);
        mLabelBSGThroughput = (TextView) findViewById(R.id.label_bsg_throughput);
        mLabelBSGThroughput.setText("");

        mButtonWriteBSGConfiguration = (Button) findViewById(R.id.btn_write_bsg_configuration);
        mButtonWriteBSGConfiguration.setOnClickListener(this);

        mButtonBSGStart = (Button) findViewById(R.id.btn_bsg_start);
        mButtonBSGStart.setOnClickListener(this);

        // Load notification components
        mTextAreaNotification = (TextView) findViewById(R.id.value_notifications);
        mTextAreaNotification.setMovementMethod(ScrollingMovementMethod.getInstance());
        mButtonClear = (Button) findViewById(R.id.btn_clear);
        mButtonClear.setOnClickListener(this);

        // Initialize the device picker UI fragment
        initDevicePicker();

        // refresh the UI component states
        updateWidgets();
    }

    /**
     * Updates the UI widgets based on the latest connection state
     */
    private void updateWidgets() {
        if (mPickedDevice == null) {
            // No devices selected: set initial state
            mButtonConnect.setEnabled(false);
            mButtonDisconnect.setEnabled(false);
            mButtonSelectDevice.setEnabled(true);
            mTextDeviceName.setText(R.string.no_device);
            mTextDeviceAddress.setText("");
        } else {
            // Device picked, always set the connect/disconnect buttons enabled
            mButtonConnect.setEnabled(true);
            mButtonDisconnect.setEnabled(true);

            if (mPickedDeviceIsConnected) {
                // Set resources when connected
                mBytesSent = 0;
                mBytesReceived = 0;
                mLabelBSGThroughput.setText("bytes sent " + mBytesSent + "    received " + mBytesReceived);

                // Disable selecting new device when connected
                mButtonSelectDevice.setEnabled(false);

                // Set the connection state status
                mTextConnectionState.setText(getString(R.string.connected));

                // Update bsg service components
                mEditTextBSGConfiguration.setEnabled(true);
                mButtonWriteBSGConfiguration.setEnabled(true);
                mButtonBSGStart.setEnabled(true);

                // Update notification components
                mTextAreaNotification.setEnabled(true);
                mButtonClear.setEnabled(true);

            } else {
                // Update resources when disconnected

                // Enable selecting new device when connected
                mButtonSelectDevice.setEnabled(true);

                // Set the connection state status
                mTextConnectionState.setText(getString(R.string.disconnected));

                // Update bsg service components
                mEditTextBSGConfiguration.setEnabled(false);
                mEditTextBSGConfiguration.setText("8");
                mButtonWriteBSGConfiguration.setEnabled(false);
                mButtonBSGStart.setEnabled(false);

                // Update notification components
                mTextAreaNotification.setEnabled(false);
                mTextAreaNotification.setText("");
                mButtonClear.setEnabled(false);
            }
        }
    }

    @Override
    public void onDestroy() {
        closeDevice();
        cleanupDevicePicker();
        super.onDestroy();
    }

    /**
     * Callback invoked when buttons/switches clicked
     */
    @Override
    public void onClick(View v) {
        if (v == mButtonSelectDevice) {
            // Start the device selector
            mDevicePicker.show(getFragmentManager(), FRAGMENT_DEVICE_PICKER);
        } else if (v == mButtonConnect) {
            // Start device connection
            connect();
        } else if (v == mButtonDisconnect) {
            // Start device disconnect
            disconnect();
        } else if (v == mButtonWriteBSGConfiguration) {
            // Start writing the bsg configuration characteristics
            onWriteBSGConfigurationCharacteristic();
        } else if (v == mButtonBSGStart) {
            if (running) {
                running = false;
                mButtonBSGStart.setText("Start");
            } else {
                // Start sending data
                running = true;
                mButtonBSGStart.setText("Stop");
                mBytesSent = 0;
                mBytesReceived = 0;
                mLabelBSGThroughput.setText("bytes sent " + mBytesSent + "    received " + mBytesReceived);
                onWriteBSGConfigurationCharacteristic();
            }
        } else if (v == mButtonClear) {
            // Clear the notification area
            mTextAreaNotification.setText("");
            mBytesSent = 0;
            mBytesReceived = 0;
            mLabelBSGThroughput.setText("bytes sent " + mBytesSent + "    received " + mBytesReceived);
        }
    }

    /**
     * Callback invoked when a device was picked from the device picker
     *
     * @param device
     */
    @Override
    public void onDevicePicked(BluetoothDevice device) {
        Log.d(TAG, "onDevicePicked: " + device == null ? "" : device.getAddress());
        // Close any outstanding connections to remote devices
        closeDevice();

        // Get the remote device object
        String address = device.getAddress();
        mPickedDevice = mBtAdapter.getRemoteDevice(address);

        // Get the name
        String name = mPickedDevice.getName();
        if (name == null || name.isEmpty()) {
            name = address;
        }

        // Set UI resources
        mTextDeviceName.setText(name);
        mTextDeviceAddress.setText(address);
        // Update the connect widget
        mButtonConnect.setEnabled(true);
        mButtonDisconnect.setEnabled(true);
    }

    /**
     * Callback invoked when a devicepicker was dismissed without a device
     * picked
     */
    @Override
    public void onDevicePickError() {
        Log.d(TAG, "onDevicePickError");
    }

    /**
     * Callback invoked when a devicepicker encountered an unexpected error
     */
    @Override
    public void onDevicePickCancelled() {
        Log.d(TAG, "onDevicePickCancelled");
    }

    /**
     * Connect to the picked device
     */
    private void connect() {
        if (mPickedDevice == null) {
            showMessage(getString(R.string.error_connect, mPickedDevice.getName(),
                    mPickedDevice.getAddress()));
            return;
        }

        mPickedDeviceGatt = mPickedDevice.connectGatt(this, false, mGattCallback);

        if (mPickedDeviceGatt == null) {
            showMessage(getString(R.string.error_connect, mPickedDevice.getName(),
                    mPickedDevice.getAddress()));
        }
    }

    /**
     * Disconnects the picked device
     */
    private void disconnect() {
        if (mPickedDeviceGatt != null) {
            mPickedDeviceGatt.disconnect();
           // closeDevice();
        }
    }

    /**
     * Called when a gatt connection state changes. This function updates the UI
     *
     * @param
     */
    private void processConnectionStateChanged(final boolean isConnected, final boolean hasError) {
        runOnUiThread(new Runnable() {

            @Override
            public void run() {
                if (hasError) {
                    showMessage(getString(R.string.error_connect, mPickedDevice.getName(),
                            mPickedDevice.getAddress()));
                }
                mPickedDeviceIsConnected = isConnected;
                updateWidgets();

                // Refresh the device information
                if (mPickedDeviceIsConnected) {
                    mSyncNotificationSetting = true;
                    //readEverything();
                }
            }
        });
    }

    /**
     * Reads the bsg input characteristic and updates the UI components
     */
    private void readBSGInputCharacteristic() {
        // Get all readable characteristics and descriptors of interest and add
        // request to a request queue
        Log.d(TAG, "readBSGInputCharacteristic");
        BluetoothGattDescriptor descriptor = null;

        // Get client config descriptor: enable/disable notification
        descriptor = GattUtils.getDescriptor(mPickedDeviceGatt, Constants.BSG_SERVICE_UUID,
                Constants.BSG_CHARACTERISTIC_CONFIGURATION_UUID, Constants.CLIENT_CONFIG_DESCRIPTOR_UUID);
        mRequestQueue.addReadDescriptor(mPickedDeviceGatt, descriptor);
        mRequestQueue.execute();
    }

    /**
     * Read every characteristic on the device
     */
    private void readEverything() {
        readBSGInputCharacteristic();
    }

    /**
     * Write the bsg configuration characteristic to the device
     */
    private void onWriteBSGConfigurationCharacteristic() {
        BluetoothGattCharacteristic characteristic = null;

        short  randomLength = 10;
        String str = mEditTextBSGConfiguration.getText().toString();
        if(str.length() > 0)
            randomLength = (short) Integer.parseInt(str);

        byte[] tx_buffer = new byte[randomLength + 2];

        tx_buffer[0] = 0;
        tx_buffer[1] = 0;

        char[] hexArray = "0123456789ABCDEF".toCharArray();
        for(int i = 0; i < randomLength; i++) {
            tx_buffer[i + 2] = (byte) (i%16);
        }

        serial_gatt_state.count = 0;
        serial_gatt_state.static_tx_len = randomLength;

        serial_gatt_client_process_forward_data(tx_buffer, (short) 2, randomLength);
    }

    private void onBsgStart(){

    }

    /**
     * Write the bsg configuration characteristic to the device
     */
    private void writeBSGConfigurationCharacteristic(byte[] charValue) {
        BluetoothGattCharacteristic characteristic = null;
        Log.d(TAG, "writeBSGConfigurationCharacteristic");
        String s = new String(serial_gatt_dump_hex_string(charValue));
        Log.d(TAG, "writeBSGConfigurationCharacteristic value " + s);

        try {
            characteristic = GattUtils.getCharacteristic(mPickedDeviceGatt, Constants.BSG_SERVICE_UUID,
                    Constants.BSG_CHARACTERISTIC_CONFIGURATION_UUID);
            mRequestQueue.addWriteCharacteristic(mPickedDeviceGatt, characteristic, charValue);
            mRequestQueue.execute();
            mBytesSent += charValue.length;

        } catch (Throwable t) {
            Log.w(TAG, "invalid number of notifications");
            mEditTextBSGConfiguration.setText("8");
        }
    }

    /**
     * Write the bsg input descriptor to the device
     */
    private void RegisterBSGNotification(boolean notify) {
        Log.d(TAG, "RegisterBSGNotification = " +  notify);
        // Set the enable/disable notification settings
        BluetoothGattCharacteristic notifyCharacteristic = GattUtils.getCharacteristic(
                mPickedDeviceGatt, Constants.BSG_SERVICE_UUID,
                Constants.BSG_CHARACTERISTIC_CONFIGURATION_UUID);
        BluetoothGattDescriptor descriptor = GattUtils.getDescriptor(mPickedDeviceGatt,
                Constants.BSG_SERVICE_UUID, Constants.BSG_CHARACTERISTIC_CONFIGURATION_UUID,
                Constants.CLIENT_CONFIG_DESCRIPTOR_UUID);
        byte[] value = new byte[2];
        if (notify == true) {
            value[0] = 1;
            value[1] = 0;
            lastNotificationState = 1;
            mPickedDeviceGatt.setCharacteristicNotification(notifyCharacteristic, true);
        } else {
            value[0] = 0;
            value[1] = 0;
            lastNotificationState = 0;
            mPickedDeviceGatt.setCharacteristicNotification(notifyCharacteristic, false);
        }
        Log.d(TAG, "RegisterBSGNotification - addWriteDescriptor");
        mRequestQueue.addWriteDescriptor(mPickedDeviceGatt, descriptor, value);
        mRequestQueue.execute();

    }

    /**
     * Write the bsg input descriptor to the device
     */
    private void writeBSGInputCharacteristic() {

        Log.d(TAG, "writeBSGInputCharacteristic");
        // Set the enable/disable notification settings
        BluetoothGattCharacteristic notifyCharacteristic = GattUtils.getCharacteristic(
                mPickedDeviceGatt, Constants.BSG_SERVICE_UUID,
                Constants.BSG_CHARACTERISTIC_CONFIGURATION_UUID);

        int notificationState = 1;

        Log.d(TAG, "writeBSGInputCharacteristic - notificationState " +  notificationState + "lastNotificationState" + lastNotificationState);
        if(notificationState == lastNotificationState)
        {
            Log.d(TAG, "writeBSGInputCharacteristic - Notification value did not change- ignore");
            return;
        }

        if (notificationState >= 0 && notificationState <= 2) {
            lastNotificationState = notificationState;

            BluetoothGattDescriptor descriptor = GattUtils.getDescriptor(mPickedDeviceGatt,
                    Constants.BSG_SERVICE_UUID, Constants.BSG_CHARACTERISTIC_CONFIGURATION_UUID,
                    Constants.CLIENT_CONFIG_DESCRIPTOR_UUID);

            byte[] value = new byte[2];
            if (notificationState > 0) {
                mPickedDeviceGatt.setCharacteristicNotification(notifyCharacteristic, true);
                if (notificationState == 1) {
                    value[0] = 1;
                    value[1] = 0;
                }
            } else {
                value[0] = 0;
                value[1] = 0;
                mPickedDeviceGatt.setCharacteristicNotification(notifyCharacteristic, false);
            }
            Log.d(TAG, "writeBSGInputCharacteristic - addWriteDescriptor notificationState " +  notificationState + "lastNotificationState" + lastNotificationState);
            mRequestQueue.addWriteDescriptor(mPickedDeviceGatt, descriptor, value);
            mRequestQueue.execute();
        }
    }

    /**
     * Callback invoked by the Android framework when a read characteristic
     * successfully completes
     *
     * @param characteristic
     */
    private void processCharacteristicRead(final BluetoothGattCharacteristic characteristic) {
        runOnUiThread(new Runnable() {

            @Override
            public void run() {
                UUID uuid = characteristic.getUuid();
                if (Constants.BSG_CHARACTERISTIC_CONFIGURATION_UUID.equals(uuid)) {
                    int blinks = characteristic.getIntValue(
                            BluetoothGattCharacteristic.FORMAT_UINT8, 0);
                    mEditTextBSGConfiguration.setText(String.valueOf(blinks));
                }
            }
        });
    }

    /**
     * Callback invoked by the Android framework when a write descriptor
     * successfully completes
     *
     * @param descriptor
     */
    private void processDescriptorWrite(final BluetoothGatt gatt, final BluetoothGattDescriptor descriptor) {
        runOnUiThread(new Runnable() {

            @Override
            public void run() {
            }
        });
    }

    /**
     * Callback invoked by the Android framework when a write characteristic
     * successfully completes
     *
     * @param characteristic
     */
    private void processCharacteristicWrite(final BluetoothGattCharacteristic characteristic) {
        runOnUiThread(new Runnable() {

            @Override
            public void run() {
                mLabelBSGThroughput.setText("");
                // Continue if there is data to be sent
                serial_gatt_client_send_serial_data();
            }
        });
    }

    void serial_gatt_client_process_notification(short conn_id, byte[] p_data, short len)
    {
        byte   flags;
        byte   credit = 0;
        byte[] data = null;
        short  mtu = 0;
        short  index = 0;

        // parse data received from the peer
        if (len < 2)
        {
            Log.d(TAG, "illegal data len\n");
            return;
        }

        // ignore bits not defined in the current protocol
        flags = (byte) (p_data[0] & (SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT | SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT | SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT));

        index++;
        len -= 1;

        if((flags & SERIAL_GATT_FLAGS_CREDIT_FIELD_PRESENT) > 0)
        {
            credit = p_data[index];
            Log.d(TAG, "notification  credit = " + credit);

            index++;
            len -= 1;
        }

        if((flags & SERIAL_GATT_FLAGS_MTU_FIELD_PRESENT) > 0)
        {
            serial_gatt_state.peer_mtu = (short)((int) ((p_data[index] & 0xFF) | ((p_data[index+1] & 0xFF) << 8)));

            Log.d(TAG, "notification  mtu = " + serial_gatt_state.peer_mtu);

            // We are not doing anything with this mtu info from server in this Android
            // App as we are setting the MTU and we've already sent credits based on that
            // in the onMTUChanged handler

            index +=2;
            len -= 2;
        }

        // if we received data send it to the app
        if ( (len > 0) && ((flags & SERIAL_GATT_FLAGS_DATA_FIELD_PRESENT) > 0))
        {
            data = new byte[len];

            for(int i = 0; i < len; i++)
                data[i] = p_data[index + i];

            serial_gatt_state.bytes_rx += len;

            String s = new String(serial_gatt_dump_hex_string(data));
            Log.d(TAG, s);

            mTextAreaNotification.append(s);
            mTextAreaNotification.append("\n");

            // Auto scroll to the bottom
            Layout l = mTextAreaNotification.getLayout();
            if (l != null) {
                int scrollAmount = l.getLineTop(mTextAreaNotification.getLineCount())
                        - mTextAreaNotification.getHeight();
                // if there is no need to scroll, scrollAmount will be <=0
                if (scrollAmount > 0)
                    mTextAreaNotification.scrollTo(0, scrollAmount);
                else
                    mTextAreaNotification.scrollTo(0, 0);
            }

            // increase number of credits we need to deliver to the peer.  serial_gatt_client_send_data
            // will send credits if needed.
            serial_gatt_state.rx_credits++;
        }

        // process number of credits if received from the peer
        if (credit + serial_gatt_state.tx_credits > SERIAL_GATT_MAX_CREDITS)
        {
            Log.d(TAG, "illegal credits");
            return;
        }

        // tx_credits indicates how many packets we can send to the peer.
        serial_gatt_state.tx_credits += credit;

        // as we got more credits, we might be able to send data out, or may need to send credits.
        serial_gatt_client_send_data(0);

        // if ack timer is not running, start one now.
        //StartTimer(100, serial_gatt_client_fine_timer_callback);
    }

    /**
     * Callback invoked by the Android framework when a characteristic
     * notification is received
     *
     * @param characteristic
     */
    private void processCharacteristicNotification(final BluetoothGattCharacteristic characteristic) {
        runOnUiThread(new Runnable() {

            @Override
            public void run() {
                mBytesReceived += characteristic.getValue().length;
                if (running) {
                    onWriteBSGConfigurationCharacteristic();
                }
                mLabelBSGThroughput.setText("bytes sent " + mBytesSent + "    received " + mBytesReceived);
                String s = characteristic.getStringValue(0);
                byte[] val = characteristic.getValue();
                byte[] data;
                short len = (short) val.length;
                short conn_id = 1;
                serial_gatt_client_process_notification(conn_id, val, len);
            }
        });
    }

}
