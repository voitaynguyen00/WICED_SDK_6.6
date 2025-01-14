/*
* Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
*/
package com.cypress.le.mesh.meshframework;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.Build;
import android.os.Environment;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.support.annotation.RequiresApi;
import android.util.Log;

import com.cypress.le.mesh.meshcore.IMeshNativeCallback;
import com.cypress.le.mesh.meshcore.MeshNativeHelper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;


@RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
public class MeshService extends Service implements IMeshGattClientCallback {

    private static String TAG = "MeshService";

    public static final int TRANSPORT_GATT = 1;
    public static final int TRANSPORT_GATEWAY = 2;
    private static boolean USE_INTERNAL_STORAGE = true;
    public static final boolean WICED_MESH_DFU_ENABLED = true;

    public static final int MESH_CLIENT_ERR_NOT_CONNECTED = 2;
    public static final int MESH_CLIENT_ERR_INVALID_ARGS = 8;

    static String BIGBDaddr = null;
    static String BigBdAddr = "";

    //tracking states
    private static final int STATE_IDLE = 0;
    private static final int STATE_TRACKING = 1;

    private static int CURRENT_TRACKING_STATE = STATE_IDLE;

    private TrackingHelper mTrackingHelper = null;

    //Enable TRANSPORT_MESH_CONTROLLER_GATT to choose controller GATT
    private static int mTransport              = TRANSPORT_GATT;//TRANSPORT_MESH_CONTROLLER_GATT;
    private static MeshGattClient mMeshGattClient = null;

    static MeshNativeHelper mMeshHelper = null;
    static Timer mTrackingTimer = null;

    private boolean provision_connecting = false;
    private int retries = 0;

    /* Mesh Client changes*/
    private String mCurrNetwork = null;
    IMeshControllerCallback mMeshControllerCb = null;

    private int mConnectionId = 0;

    // Binder given to clients
    private final IBinder mBinder = new LocalBinder();
    private String mFileName = null;
    private String mMetadataFile = null;


    void register(IMeshControllerCallback cb){
        infoLog("register register");
        mMeshControllerCb = cb;
    }

    void stopTracking() {
        //stop hsl timer
        CURRENT_TRACKING_STATE = STATE_IDLE;
        mTrackingHelper.stopTracking();
        if(mTrackingTimer != null)
        {
        mTrackingTimer.cancel();
        mTrackingTimer.purge();
        mTrackingTimer = null;
        }
    }

    void startTracking() {
        //start hsl timer
        mTrackingHelper = new TrackingHelper();
        CURRENT_TRACKING_STATE = STATE_TRACKING;
        mTrackingHelper.startTracking();
        if(mTrackingTimer == null)
            mTrackingTimer = new Timer();
        mTrackingTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                mTrackingHelper.execute(false);
            }
        }, 0, 100);
    }

    int isNetworkExist(String meshName) {
        return mMeshNativeHelper.meshClientNetworkExists(meshName);
    }

    int setDeviceConfig(String deviceName, int isGattProxy, int isFriend, int isRelay, int sendNetBeacon, int relayXmitCount, int relayXmitInterval, int defaultTtl, int netXmitCount, int netXmitInterval) {
        if(deviceName == null || deviceName.equals(""))
            deviceName = "";
        return mMeshNativeHelper.meshClientSetDeviceConfig(deviceName, isGattProxy, isFriend, isRelay, sendNetBeacon, relayXmitCount, relayXmitInterval, defaultTtl, netXmitCount, netXmitInterval);
    }

    int setPublicationConfig(int publishCredentialFlag, int publishRetransmitCount, int publishRetransmitInterval, int publishTtl) {
        return mMeshNativeHelper.meshClientSetPublicationConfig(publishCredentialFlag, publishRetransmitCount, publishRetransmitInterval, publishTtl);
    }

    int resetDevice(String componentName) {
        return mMeshNativeHelper.meshClientResetDevice(componentName);
    }

    int vendorDataSet(String deviceName, byte[] buffer, short len) {
        if(!isConnectedToNetwork())
            return MESH_CLIENT_ERR_NOT_CONNECTED;
        return mMeshNativeHelper.meshClientVendorDataSet(deviceName, buffer, len);
    }

    int identify(String name, byte duration) {
        if(!isConnectedToNetwork())
            return MESH_CLIENT_ERR_NOT_CONNECTED;
        return mMeshNativeHelper.meshClientIdentify(name,duration);
    }

    int lightnessGet(String deviceName) {
        if (!isConnectedToNetwork())
            return MESH_CLIENT_ERR_NOT_CONNECTED;
        return mMeshNativeHelper.meshClientLightnessGet(deviceName);
    }

    int lightnessSet(String name, int lightness, boolean reliable, int transitionTime, short delay) {
        int res;
        if(!isConnectedToNetwork())
            return MESH_CLIENT_ERR_NOT_CONNECTED;
        if(CURRENT_TRACKING_STATE == STATE_TRACKING) {
            mTrackingHelper.setTrackingType(TrackingHelper.TRACK_LIGHTNESS_SET);
            mTrackingHelper.LightnessSetMessage(name, lightness, transitionTime,delay);
            res = 1;
        } else {
            res = mMeshNativeHelper.meshClientLightnessSet(name, lightness, reliable, transitionTime,delay);
        }
        return res;
    }

    int ctlGet(String deviceName) {
        return mMeshNativeHelper.meshClientCtlGet(deviceName);
    }

    int ctlSet(String name, int lightness, short temperature, short deltaUv, boolean reliable, int transitionTime, short delay) {
        int res;
        if(!isConnectedToNetwork())
            return MESH_CLIENT_ERR_NOT_CONNECTED;
        if(CURRENT_TRACKING_STATE == STATE_TRACKING) {
            mTrackingHelper.setTrackingType(TrackingHelper.TRACK_CTL_SET);
            mTrackingHelper.CtlSetMessage(name, lightness, temperature, deltaUv, transitionTime,delay);
            res = 1;
        } else {
            res = mMeshNativeHelper.meshClientCtlSet(name, lightness, temperature, deltaUv, reliable, transitionTime,delay);
        }
        return res;
    }

    @Override
    public void onNetworkConnectionStateChange() {
        if(mMeshControllerCb != null) {
            Log.d(TAG,"onNetworkConnectionStateChange status:"+isConnectedToNetwork());
            mMeshControllerCb.onNetworkConnectionStatusChanged((byte)mTransport, isConnectedToNetwork()?IMeshControllerCallback.NETWORK_CONNECTION_STATE_CONNECTED:IMeshControllerCallback.NETWORK_CONNECTION_STATE_DISCONNECTED);
        }
    }

    @Override
    public void onOTAUpgradeStatusChanged(byte status, int percent) {
        mMeshControllerCb.onOTAUpgradeStatus(status, percent);
    }

    String[] getTargetMethods(String componentName) {
         return mMeshNativeHelper.meshClientGetTargetMethods(componentName);
    }

    String[] getControlMethods(String componentName) {
        return mMeshNativeHelper.meshClientGetControlMethods(componentName);
    }

    byte connectComponent(String componentName, byte scanDuration) {
        return mMeshNativeHelper.meshConnectComponent(componentName, (byte) TRANSPORT_GATT, scanDuration);
    }

    public int startOtaUpgrade(String componentName, byte upgradeType) {
        Log.d(TAG,"otaUpgrade fileName:"+mFileName + ", upgradeType = " + upgradeType);
        if(upgradeType == MeshController.DFU_METHOD_APP_TO_ALL) {
            return mMeshNativeHelper.meshClientDfuStart(upgradeType, componentName);
        } else if (upgradeType == MeshController.DFU_METHOD_APPLY) {
            mMeshGattClient.otaUpgradeApply();
            return 0;
        } else {
            mMeshGattClient.startOtaUpgrade(componentName, mFileName, mMetadataFile, upgradeType);
            return 0;
        }
    }

    public void setOTAFiles(String fwFile, String metadataFile) {
        Log.d(TAG,"setOTaFiles fw file :"+fwFile+" metadataFile :"+metadataFile);
        mFileName = fwFile;
        mMetadataFile = metadataFile;
        if (MeshService.WICED_MESH_DFU_ENABLED) {
            if(fwFile != null && metadataFile != null) {
                mMeshNativeHelper.meshClientSetDfuFiles(fwFile, metadataFile);
            }
        } else {
            if(metadataFile != null) {
                mMeshNativeHelper.meshClientSetDfuFiles(fwFile, metadataFile);
            }
        }

    }

    public int stopOtaUpgrade() {
        mMeshGattClient.stopOtaUpgrade();
        return mMeshNativeHelper.meshClientDfuStop();
    }

    boolean sendReceivedData(byte[] data) {
        if(mTransport == TRANSPORT_GATEWAY) {
            mMeshNativeHelper.SendRxProxyPktToCore(data, data.length);
            return true;
        } else {
            Log.d(TAG,"Invalid transport!!!");
            return false;
        }
    }

    String importNetwork(String provisionerName, String jsonString) {
        infoLog("importNetwork ");
        if((provisionerName == null || provisionerName.equals("") && (jsonString == null || jsonString.equals(""))))
            return null;

        if(isConnectedToNetwork()) {
            disconnectNetwork();

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        return mMeshNativeHelper.meshClientNetworkImport(provisionerName, jsonString);
    }

    String exportNetwork(String meshName) {
        infoLog("exportNetwork ");
        if((meshName == null || meshName.equals("")))
            return null;
        return mMeshNativeHelper.meshClientNetworkExport(meshName);
    }

    int deleteNetwork(String provisionerName, String meshName) {
        if((provisionerName == null || provisionerName.equals("") && (meshName == null || meshName.equals(""))))
            return MESH_CLIENT_ERR_INVALID_ARGS;
        return mMeshNativeHelper.meshClientNetworkDelete(provisionerName, meshName);
    }

    byte getComponentInfo(String componentName) {
        if(componentName == null || componentName.equals(""))
            return MESH_CLIENT_ERR_INVALID_ARGS;
        return mMeshNativeHelper.meshClientGetComponentInfo(componentName);
    }

     String[] getComponentGroupList(String componentName) {
        if(componentName == null || componentName.equals("")) {
            Log.d(TAG, "componentName is not valid");
            return null;
        }
        return mMeshNativeHelper.meshClientGetComponentGroupList(componentName);
    }

    int removeComponentFromGroup(String componentName, String groupName) {
        if(componentName == null || componentName.equals("") || groupName == null || groupName.equals(""))
            return MESH_CLIENT_ERR_INVALID_ARGS;
        return mMeshNativeHelper.meshClientRemoveComponentFromGroup(componentName, groupName);
    }

    int addComponentToGroup(String componentName, String groupName) {
        if(componentName == null || componentName.equals("") || groupName == null || groupName.equals(""))
            return MESH_CLIENT_ERR_INVALID_ARGS;
        return mMeshNativeHelper.meshClientAddComponentToGroup(componentName, groupName);
    }

    int dfuGetStatus(String componentName) {
        return mMeshNativeHelper.meshClientDfuGetStatus(componentName);
    }

    int networkConnectionChanged(int connId) {
        int res;
        mTransport = MeshController.TRANSPORT_IP;
        res = mMeshNativeHelper.meshClientNetworkConnectionChanged(connId);
        mMeshNativeHelper.meshClientSetGattMtu(Constants.MTU_SIZE_REST);
        return res;
    }

    int sensorCadenceSet(String componentName, int propertyId, short fastCadencePeriodDivisor, boolean triggerType, int triggerDeltaDown, int triggerDeltaUp, int minInterval, int fastCadenceLow, int fastCadenceHigh) {
        if(componentName == null || componentName.equals("") || propertyId == 0)
        {
            Log.d(TAG, "componentName is not valid");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        return mMeshNativeHelper.meshClientSensorCadenceSet(componentName, propertyId, fastCadencePeriodDivisor, triggerType,
                triggerDeltaDown, triggerDeltaUp, minInterval, fastCadenceLow, fastCadenceHigh);
    }

    short[] sensorSettingsGet(String componentName, int propertyId) {
        if(componentName == null || componentName.equals("") || propertyId == 0)
        {
            Log.d(TAG, "componentName is not valid");
            return null;
        }
        return mMeshNativeHelper.meshClientSensorSettingsGetPropIds(componentName, propertyId);
    }

    int[] sensorPropertyListGet(String componentName) {
        if(componentName == null || componentName.equals(""))
        {
            Log.d(TAG, "componentName is not valid");
            return null;
        }
        return mMeshNativeHelper.meshClientSensorPropertyListGet(componentName);
    }

    int sensorSettingSet(String componentName, int propertyId, short settingPropertyId, byte[] val) {
        if(componentName == null || componentName.equals("") || propertyId == 0 || val == null)
        {
            Log.d(TAG, "componentName is not valid");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        return mMeshNativeHelper.meshClientSensorSettingSet(componentName, propertyId, settingPropertyId, val);

    }

    int sensorGet(String componentName, int propertyId) {
        if(componentName == null || componentName.equals("") && propertyId == 0)
        {
            Log.d(TAG, "params is not valid");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        return mMeshNativeHelper.meshClientSensorGet(componentName, propertyId);
    }

    int listenForAppGroupBroadcasts(String controlMethod, String groupName, boolean startListening) {
        if(controlMethod == null)
            controlMethod = "";
        if(groupName == null)
            groupName = "";
        return mMeshNativeHelper.meshClientListenForAppGroupBroadcasts(controlMethod, groupName, startListening);
    }

    String getPublicationTarget(String componentName, boolean isClient, String method) {
        if(componentName == null || componentName.equals("") ||
                method == null || method.equals(""))
        {
            Log.d(TAG, "params is not valid");
            return null;
        }
        return mMeshNativeHelper.meshClientGetPublicationTarget(componentName, isClient == true ?(byte)1:0, method);
    }

    int getPublicationPeriod(String componentName, boolean isClient, String method) {
        if(componentName == null || componentName.equals("") ||
                method == null || method.equals(""))
        {
            Log.d(TAG, "params is not valid");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        return mMeshNativeHelper.meshClientGetPublicationPeriod(componentName, isClient == true ?(byte)1:0, method);
    }

    /**
     * Class used for the client Binder.  Because we know this service always
     * runs in the same process as its clients, we don't need to deal with IPC.
     */
    public class LocalBinder extends Binder {
        MeshService getService() {
            // Return this instance of LocalService so clients can call public methods
            return MeshService.this;
        }
    }

    boolean isConnectedToNetwork() {

        boolean res = false;
        if(mTransport == MeshController.TRANSPORT_IP)
            res = true;
        else
            res = mMeshNativeHelper.meshClientIsConnectedToNetwork();
        infoLog("isConnectedToNetwork : "+res);
        return res;
    }


    boolean scanMeshDevices(boolean start) {
        boolean ret = true;
        infoLog("scanMeshDevices start:" + start + " ret:" + ret);

        mMeshNativeHelper.meshClientScanUnprovisioned(start?1:0);
        return ret;
    }

    int getTransport() {
        return mTransport;
    }



    @Override
    public void onCreate() {
        debugLog("onCreate");
        super.onCreate();

        mMeshHelper = MeshNativeHelper.getInstance();
        mMeshHelper.registerNativeCallback(nativeCallback);
        mMeshHelper.meshClientInit();

        if(USE_INTERNAL_STORAGE)
            mMeshHelper.setFileStorge(getFilesDir().getAbsolutePath());
        else
            mMeshHelper.setFileStorge(Environment.getExternalStorageDirectory().getAbsolutePath());

        mMeshGattClient = MeshGattClient.getInstance(this);

        mMeshGattClient.init(this);
        infoLog("on create end");

    }

    static String toHexString(byte[] bytes) {
        int len = bytes.length;
        if(len == 0)
            return null;

        char[] buffer = new char[len * 3 - 1];

        for (int i = 0, index = 0; i < len; i++) {
            if (i > 0) {
                buffer[index++] = ' ';
            }

            int data = bytes[i];
            if (data < 0) {
                data += 256;
            }

            byte n = (byte) (data >>> 4);
            if (n < 10) {
                buffer[index++] = (char) ('0' + n);
            }
            else {
                buffer[index++] = (char) ('A' + n - 10);
            }

            n = (byte) (data & 0x0F);
            if (n < 10) {
                buffer[index++] = (char) ('0' + n);
            }
            else {
                buffer[index++] = (char) ('A' + n - 10);
            }
        }
        return new String(buffer);
    }

    @Override
    public void onDestroy() {
        debugLog("onDestroy");
        mMeshGattClient = null;
        mMeshHelper = null;
        mMeshNativeHelper = null;
        mMeshControllerCb = null;
        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        debugLog("onBind intent:" + intent);
        return (mBinder);
    }

    @Override
    public boolean onUnbind(Intent intent) {
        debugLog("onUnbind");
        return super.onUnbind(intent);
    }

    /**
     * DeathReceipient handlers used to unregister applications that
     * die ungracefully (ie. crash or forced close).
     */
    class MeshDeathRecipient implements IBinder.DeathRecipient {

        public void binderDied() {
            errorLog("Binder is dead - closing network (" + mCurrNetwork + ")!");
            if (mCurrNetwork == null) return;
        }
    }

    // -- MeshController apis --
     MeshNativeHelper mMeshNativeHelper = MeshNativeHelper.getInstance();

    int createNetwork(String provisionerName, String meshName) {
        if((provisionerName == null || provisionerName.equals("") && (meshName == null || meshName.equals(""))))
            return MESH_CLIENT_ERR_INVALID_ARGS;
        return mMeshNativeHelper.meshClientNetworkCreate(provisionerName, meshName);
    }

    int openNetwork(String provisionerName, String meshName) {
        mCurrNetwork = meshName;
        int res;
        if(provisionerName == null || provisionerName.equals(""))
            return MESH_CLIENT_ERR_INVALID_ARGS;

        ArrayList<String> networks = new ArrayList<String>(Arrays.asList(mMeshNativeHelper.meshClientGetAllNetworks()));
        if(networks.contains(meshName)) {
            res = mMeshNativeHelper.meshClientNetworkOpen(provisionerName, meshName);
        } else {
            res = MESH_CLIENT_ERR_INVALID_ARGS;
        }

        return res;
    }

    void closeNetwork() {
        mMeshNativeHelper.meshClientNetworkClose();
    }

    int createGroup(String groupName, String parentGroupName) {
        if(groupName == null || groupName.equals("")){
            Log.d(TAG,"returning MESH_CLIENT_ERR_INVALID_ARGS because groupName was emtpy !!!");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        if(parentGroupName == null)
            parentGroupName = "";
        return mMeshNativeHelper.meshClientGroupCreate(groupName, parentGroupName);
    }

    int deleteGroup(String groupName) {

        if(groupName == null || groupName.equals(""))
            return MESH_CLIENT_ERR_INVALID_ARGS;

        return mMeshNativeHelper.meshClientGroupDelete(groupName);
    }

    String[] getAllNetworks() {
        return mMeshNativeHelper.meshClientGetAllNetworks();
    }

    String[] getAllGroups(String inGroup) {
        if(inGroup == null || inGroup.equals(""))
            return null;
        return mMeshNativeHelper.meshClientGetAllGroups(inGroup);
    }

    String[] getAllProvisioners() {
        return mMeshNativeHelper.meshClientGetAllProvisioners();
    }

    String[] getDeviceComponents(byte[] uuid) {
        if(uuid == null) {
            Log.d(TAG, "uuid is null");
            return null;
        }

        return mMeshNativeHelper.meshClientGetDeviceComponents(uuid);
    }

    String[] getGroupComponents(String groupName) {
        if(groupName == null || groupName.equals("")) {
            Log.d(TAG, "groupname is not valid");
            return null;
        }
        return mMeshNativeHelper.meshClientGetGroupComponents(groupName);
    }

    byte getComponentType(String componentName) {
        if(componentName == null || componentName.equals("")) {
            Log.d(TAG, "groupname is not valid");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        return mMeshNativeHelper.meshClientGetComponentType(componentName);
    }

    int rename(String oldName, String newName) {
        if((oldName == null || oldName.equals("")) || (newName == null || newName.equals(""))) {
            Log.d(TAG, "invalid params one of the param is null");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        return mMeshNativeHelper.meshClientRename(oldName, newName);
    }

    int moveComponentToGroup(String componentName, String fromGroupName, String toGroupName) {
        if((componentName == null || componentName.equals("")) ||
                (fromGroupName == null || fromGroupName.equals("")) ||
                (toGroupName == null || toGroupName.equals(""))) {
            Log.d(TAG, "invalid params one of the param is null");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        return mMeshNativeHelper.meshClientMoveComponentToGroup(componentName, fromGroupName, toGroupName);
    }

    int configurePublication(String componentName, boolean isClient, String method, String targetName, int publishPeriod) {
        if((componentName == null || componentName.equals("")) || (targetName == null || targetName.equals(""))) {
            Log.d(TAG, "invalid params one of the param is null");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        return mMeshNativeHelper.meshClientConfigurePublication(componentName, isClient == true ?(byte)1:0 , method, targetName, publishPeriod);
    }

    byte provision(final String deviceName ,final String groupName, final UUID deviceUUID, final byte identifyDuration) {

        if (mCurrNetwork == null){
            errorLog("error provision node. active network not set.");
            return MESH_CLIENT_ERR_INVALID_ARGS;
        }
        provision_connecting = true;
        retries = 3;

        UUID   uuid =  deviceUUID;
        final byte[] buffer = new byte[16];
        int index = 0;

        long msb = uuid.getMostSignificantBits();
        for (int i = 0; i < 8; i++) {
            buffer[index + 7 - i] = (byte)(msb & 0xff);
            msb = (msb >> 8);
        }
        index = index + 8;
        long lsb = uuid.getLeastSignificantBits();
        for (int i = 0; i < 8; i++) {
            buffer[index + 7 - i] = (byte)(lsb & 0xff);
            lsb = (lsb >> 8);
        }
        Log.d(TAG,"uuid : "+toHexString(buffer));

//        scanMeshDevices(false);

        new Handler(Looper.getMainLooper()).postDelayed(new Runnable() {
            @Override
            public void run() {
                mMeshNativeHelper.meshClientProvision(deviceName, groupName, buffer, identifyDuration);
            }
        }, 100);

        return MeshController.MESH_CLIENT_SUCCESS;
    }

    byte connectNetwork( byte scanDuration) {
        mTransport = TRANSPORT_GATT;
        return mMeshNativeHelper.meshClientConnectNetwork((byte) TRANSPORT_GATT, scanDuration);
    }

    byte disconnectNetwork() {

        if(mTransport == TRANSPORT_GATT){
            return mMeshNativeHelper.meshClientDisconnectNetwork((byte) TRANSPORT_GATT);
        } else {
            return MeshController.MESH_CLIENT_ERR_INVALID_ARGS;
        }
    }

    int onoffGet(String deviceName) {
        if(isConnectedToNetwork())
            return mMeshNativeHelper.meshClientonoffGet(deviceName);
        else
            return MESH_CLIENT_ERR_NOT_CONNECTED;
    }

    int onoffSet(String deviceName, boolean onoff, boolean reliable, int transitionTime, short delay) {
        if(isConnectedToNetwork())
            return mMeshNativeHelper.meshClientonoffSet(deviceName, (byte) ((onoff==true)?0x01:0x00), reliable, transitionTime, delay);
        else
            return MESH_CLIENT_ERR_NOT_CONNECTED;
    }

    int levelGet(String deviceName) {
        if(isConnectedToNetwork())
            return mMeshNativeHelper.meshClientLevelGet(deviceName);
        else
            return MESH_CLIENT_ERR_NOT_CONNECTED;
    }

    int levelSet(String deviceName, short level, boolean reliable, int transitionTime, short delay) {
        int res;
        if(!isConnectedToNetwork())
            return MESH_CLIENT_ERR_NOT_CONNECTED;
        if(CURRENT_TRACKING_STATE == STATE_TRACKING) {
            mTrackingHelper.setTrackingType(TrackingHelper.TRACK_LEVEL_SET);
            mTrackingHelper.levelSetMessage(deviceName, level, transitionTime, delay);
            res = 1;
        } else {
            res = mMeshNativeHelper.meshClientLevelSet(deviceName, level, reliable, transitionTime, delay);
        }
        return res;
    }

    int hslGet(String deviceName) {
        if(!isConnectedToNetwork())
            return MESH_CLIENT_ERR_NOT_CONNECTED;
        return mMeshNativeHelper.meshClientHslGet(deviceName);
    }

    int hslSet(String deviceName, int lightness, int hue, int saturation, boolean reliable, int transitionTime, short delay) {
        int res;
        if(!isConnectedToNetwork())
            return MESH_CLIENT_ERR_NOT_CONNECTED;
        if(CURRENT_TRACKING_STATE == STATE_TRACKING) {
            mTrackingHelper.setTrackingType(TrackingHelper.TRACK_HSL_SET);
            mTrackingHelper.hslSetMessage(deviceName, (short)lightness, (short)hue, (short)saturation, transitionTime, delay);
            res = 1;
        } else {
            res = mMeshNativeHelper.meshClientHslSet(deviceName, lightness, hue, saturation, reliable, transitionTime, delay);
        }
        return res;
    }

     // ++ MeshController apis ++

    void Sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private  IMeshNativeCallback nativeCallback = new IMeshNativeCallback() {
        //Native callbacks
        @Override
        public void onProvGattPktReceivedCallback(byte[] p_data, int len) {
            mMeshGattClient.sendProvisionPacket(p_data,len);

        }

        @Override
        public void onProxyGattPktReceivedCallback( byte[] p_data, int len) {
            if(mTransport == TRANSPORT_GATEWAY)
                mMeshControllerCb.onReceivedProxyPktFromCore(p_data, len);
            else
                mMeshGattClient.sendProxyPacket(p_data,len);
        }

        @Override
        public void onDeviceFound(UUID uuid, String name) {
            mMeshControllerCb.onDeviceFound(uuid, name);
        }

        @Override
        public void onLinkStatus(byte isConnected, int connId, short addr, byte isOverGatt) {
            byte connStatus;
            Log.d(TAG,"onLinkStatusCallback" + isConnected+ "  isOverGatt"+isOverGatt);
//            if(mMeshControllerCb != null && isConnected == 0) {
//                mMeshControllerCb.onOTAUpgradeStatus(Constants.OTA_UPGRADE_STATUS_DISCONNECTED, 0);
//            }

            if(isConnected != 0)
                connStatus = IMeshControllerCallback.NETWORK_CONNECTION_STATE_CONNECTED;
            else
                connStatus = IMeshControllerCallback.NETWORK_CONNECTION_STATE_DISCONNECTED;

			// on receiving link status notify application network connections status
            if(mMeshControllerCb != null){
                Log.d(TAG, "Notifying application that networkconnection state is :"+connStatus);
                mMeshControllerCb.onNetworkConnectionStatusChanged((byte)mTransport, (byte)connStatus);
            }


        }

        @Override
        public void onNetworkOpenedCb(byte status) {
            if(mMeshControllerCb != null ) {
                mMeshControllerCb.onNetworkOpenedCallback(status);
            }
        }

        @Override
        public void onDatabaseChangedCallback(String meshName) {
            if(mMeshControllerCb != null) {
                mMeshControllerCb.onDatabaseChanged(meshName);
            }
        }

        @Override
        public void onComponentInfoStatus(byte status, String componentName, String componentInfo) {
            if(mMeshControllerCb != null) {
                mMeshControllerCb.onComponentInfoStatus(status, componentName, componentInfo);
            }
        }

        @Override
        public void meshClientProvisionCompletedCb(byte status, byte[] uuid) {
            long msb = 0;
            long lsb = 0;
            for (int i = 0; i < 6; i++)
                msb = (msb << 8) | (uuid[i] & 0xff);
            for (int i = 6; i < 12; i++)
                lsb = (lsb << 8) | (uuid[i] & 0xff);
            UUID uuid_res = new UUID(msb, lsb);
            mMeshControllerCb.onProvisionComplete(uuid_res, status);
        }

        @Override
        public void meshClientOnOffStateCb(String deviceName, byte onOff) {
            mMeshControllerCb.onOnOffStateChanged(deviceName, onOff);
        }

        @Override
        public void meshClientLevelStateCb(String deviceName, short level) {
            mMeshControllerCb.onLevelStateChanged(deviceName, level);
        }

        @Override
        public void meshClientHslStateCb(String deviceName, int lightness, int hue, int saturation) {
            mMeshControllerCb.onHslStateChanged(deviceName, lightness, hue, saturation);
        }

        @Override
        public void meshClientCtlStateCb(String deviceName, int presentLightness, short presentTemperature, int targetLightness, short targetTemperature, int remainingTime) {
            mMeshControllerCb.onCtlStateChanged(deviceName, presentLightness, presentTemperature, targetLightness, targetTemperature, remainingTime);
        }

        @Override
        public void meshClientLightnessStateCb(String deviceName, int target, int present, int remainingTime) {
            mMeshControllerCb.onLightnessStateChanged(deviceName, target, present, remainingTime);
        }

        @Override
        public void meshClientDfuStatusCb(byte status, int currentBlockNumber, int totalBlocks) {
            mMeshControllerCb.onDfuStatus(status, currentBlockNumber, totalBlocks);
        }

        @Override
        public void meshClientSensorStatusCb(String componentName, int propertyId, byte[] data) {
            mMeshControllerCb.onSensorStatusCb(componentName, propertyId, data);
        }

        @Override
        public void meshClientAdvScanStartCb() {
            if(mMeshGattClient != null) {
              //  mMeshControllerCb.onNetworkConnectionStatusChanged((byte) mTransport, IMeshControllerCallback.NETWORK_CONNECTION_STATE_CONNECTING);
                mMeshGattClient.meshAdvScan(true);
            }
        }

        @Override
        public void meshClientAdvScanStopCb() {
            if(mMeshGattClient != null) {
                mMeshGattClient.meshAdvScan(false);
            }
        }

        @Override
        public void meshClientConnect(byte[] bdaddr) {

            if(mMeshGattClient != null)
                mMeshGattClient.connect(bdaddr);
        }

        @Override
        public void meshClientDisconnect(short connId) {
            if(mMeshGattClient != null)
                mMeshGattClient.disconnect(connId);
        }

        @Override
        public void meshClientNodeConnectStateCb(byte status, String componentName) {
           Log.d(TAG,"meshClientNodeConnectStateCb status:"+status);

            mMeshControllerCb.onNodeConnectionStateChanged(status, componentName);
            //TODO remove this once NodeconnectionStateChange is fixed
            //Sleep(200);

            //TODO : Check with Victor startOTAUpgrade public api cannot call connect component internally
//           if(status == Constants.MESH_CLIENT_NODE_CONNECTED) {
//               mMeshControllerCb.onOTAUpgradeStatus(Constants.OTA_UPGRADE_STATUS_CONNECTED, 0);
//               mMeshGattClient.startOtaUpgrade(mFileName);
//           }
        }

        @Override
        public void meshClientSetScanTypeCb(byte scanType) {

        }

    };

    private static void infoLog(String msg) {
        if (DebugUtils.VDBG) Log.i(TAG, msg);
    }

    private void debugLog(String msg) {
        if (DebugUtils.DBG) infoLog(msg);
    }

    private void errorLog(String msg) {
        if (DebugUtils.ERR) Log.e(TAG, msg);
    }

}

