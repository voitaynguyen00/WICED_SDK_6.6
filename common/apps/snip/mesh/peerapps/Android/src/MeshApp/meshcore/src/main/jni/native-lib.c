#include <jni.h>
#include <android/log.h>
#include <platform.h>
#include <wiced_bt_mesh_core.h>
#include <wiced_bt_mesh_provision.h>
#include <stdlib.h>
#include "malloc.h"
#include "trace.h"
#include "wiced_mesh_client.h"
#include "mesh_main.h"
#include <unistd.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <time.h>
#include <pthread.h>
#include <p_256_types.h>

//#define LOG_TAG "Jni"
#define  Log(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define UUID_LEN 16
static jclass jniWrapperClass;
static JNIEnv *sCallbackEnv = NULL;
static JavaVM   *svm;
static jmethodID processDataCb;
static jmethodID processGattPktCb;

static jmethodID meshClientUnProvisionedDeviceCb;
static jmethodID meshClientProvisionCompletedCb;
static jmethodID meshGattDisconnectCb;
static jmethodID meshGattAdvScanStartCb;
static jmethodID meshGattAdvScanStopCb;
static jmethodID meshGattProvisSendCb;
static jmethodID meshGattProxySendCb;
static jmethodID meshGattConnectCb;
static jmethodID meshGattSetScanTypeCb;
static jmethodID onOffStateCb;
static jmethodID levelStateCb;
static jmethodID meshClientHslStateCb;
static jmethodID meshClientLightnessStateCb;
static jmethodID meshClientCtlStateCb;
static jmethodID meshClientNodeConnectStateCb;
static jmethodID meshClientDbStateCb;
static jmethodID meshClientDfuStatusCb;
static jmethodID meshClientLinkStatusCb;
static jmethodID meshClientNetworkOpenedCb;
static jmethodID meshClientComponentInfoCb;
static jmethodID startTimercb;
static jmethodID stopTimercb;
static jmethodID sensorStatuscb;
char pathname[100];
char provisioner_uuid[50];
static uint8_t WICED_MESH_TRANSPORT_GATEWAY = 2;
char fw_file_name[100];
char fw_metadata_file_name[100];

extern void mesh_application_init();
extern uint8_t mesh_client_is_connecting_provisioning();
extern void mesh_client_advert_report(uint8_t *bd_addr, uint8_t addr_type, int8_t rssi, uint8_t *adv_data);
extern void MeshTimerFunc(long timer_id);

static uint32_t timer_id = 1;
extern pthread_mutex_t cs;
void create_prov_uuid();
wiced_bool_t get_firmware_id_from_file(char* sFilePath, mesh_dfu_fw_id_t *fwID);

void unprovisioned_device(uint8_t *p_uuid, uint16_t oob, uint8_t *name, uint8_t name_len);
typedef struct
{
    int is_gatt_proxy;
    int is_friend;
    int is_relay;
    int send_net_beacon;
    int relay_xmit_count;
    int relay_xmit_interval;
    int default_ttl;
    int net_xmit_count;
    int net_xmit_interval;
    int publish_credential_flag;       ///< Value of the Friendship Credential Flag
    int publish_ttl;                   ///< Default TTL value for the outgoing messages
    int publish_period;                ///< Period for periodic status publishing
    int publish_retransmit_count;      ///< Number of retransmissions for each published message
    int publish_retransmit_interval;   ///< Interval in milliseconds between retransmissions
} device_config_params_t;

device_config_params_t DeviceConfig = { 1, 1, 1, 1, 3, 100, 63, 3, 100, 0, 63, 0, 0, 500 };

#define MESH_COMMAND_GATT_PROXY_PKT      = 0;
#define MESH_COMMAND_GATT_PROVISION_PKT  = 1;

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshservice_JNIWrapper_cleanupNative(JNIEnv *env, jclass type) {

    if (jniWrapperClass != NULL) {
        (*env)->DeleteGlobalRef(env,jniWrapperClass);
        jniWrapperClass = NULL;
    }
}
JNIEnv* AttachJava()
{
    JavaVMAttachArgs args = {JNI_VERSION_1_2, 0, 0};
    JNIEnv* java;
    (*svm)->AttachCurrentThread(svm,(void**) &java, &args);
    return java;
}

void JNI_OnUnload(JavaVM *vm, void *reserved)
{
    Log("JNI_OnUnload");
    JNIEnv  *env;
    (*vm)->GetEnv(vm, (void **)&env, JNI_VERSION_1_6);
    (*env)->DeleteGlobalRef(env, jniWrapperClass);
}


int cdToExtStorage(void) {

    // Make JNI calls to get the external storage directory, and cd to it.

    // To begin, get a reference to the env and attach to it.
    JNIEnv *env;
    int isAttached = 0;
    int ret = 0;
    struct stat st = {0};


    jthrowable exception;
    Log("directory :cdToExtStorage");
    if (((*svm)->GetEnv(svm, (void**)&env, JNI_VERSION_1_6)) < 0) {
        // Couldn't get JNI environment, so this thread is native.
        if (((*svm)->AttachCurrentThread(svm, &env, NULL)) < 0) {
            fprintf(stderr, "Error: Couldn't attach to Java VM.\n");
            return (-1);
        }
        isAttached = 1;
    }
/*
    //internal storage --
    jclass classEnvironment = (*sCallbackEnv)->FindClass(sCallbackEnv,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    if (!classEnvironment) goto bailAndroid;
    jmethodID methodIDgetExternalStorageDirectory = (*env)->GetMethodID(env, classEnvironment, "getFilesDir", "()Ljava/io/File;"); // public static File internalStorageDirectory()
    //internal storage ++
*/

    //external storage --
    // Get File object for the external storage directory.
    jclass classEnvironment = (*env)->FindClass(env, "android/os/Environment");
    if (!classEnvironment) goto bailAndroid;
    jmethodID methodIDgetExternalStorageDirectory = (*env)->GetStaticMethodID(env, classEnvironment, "getExternalStorageDirectory", "()Ljava/io/File;"); // public static File getExternalStorageDirectory ()
    //external storage ++
    if (!methodIDgetExternalStorageDirectory) goto bailAndroid;
    jobject objectFile = (*env)->CallStaticObjectMethod(env, classEnvironment, methodIDgetExternalStorageDirectory);
    exception = (*env)->ExceptionOccurred(env);
    if (exception) {
        (*env)->ExceptionDescribe(env);
        (*env)->ExceptionClear(env);
    }

    // Call method on File object to retrieve String object.
    jclass classFile = (*env)->GetObjectClass(env, objectFile);
    if (!classFile) goto bailAndroid;
    jmethodID methodIDgetAbsolutePath = (*env)->GetMethodID(env, classFile, "getAbsolutePath", "()Ljava/lang/String;");
    if (!methodIDgetAbsolutePath) goto bailAndroid;
    jstring stringPath = (*env)->CallObjectMethod(env, objectFile, methodIDgetAbsolutePath);
    exception = (*env)->ExceptionOccurred(env);
    if (exception) {
        (*env)->ExceptionDescribe(env);
        (*env)->ExceptionClear(env);
    }

    memset(pathname,0,100);

    // Extract a C string from the String object, and chdir() to it.
    const char *wpath3 = (*env)->GetStringUTFChars(env, stringPath, NULL);
    strcpy(pathname, wpath3);
    strcat(pathname,"/mesh");
    Log("path %s.\n", pathname);

    if (stat(pathname, &st) == -1) {
        mkdir(pathname, 0777);
    }
    if (chdir(pathname) != 0) {
        Log("Error: Unable to change working directory to %s\n", pathname);
        perror(NULL);
    }
//    else if (path) {
//        if (chdir(path) != 0) {
//            fprintf(stderr, "Error: Unable to change working directory to %s.\n", path);
//            perror(NULL);
//        }
//    }

    Log("directory : %s",pathname);

    create_prov_uuid();
    (*env)->ReleaseStringUTFChars(env, stringPath, wpath3);

    goto retAndroid;

    bailAndroid:
    Log("Error: JNI call failure.\n");
    ret = -1;
    retAndroid:
    if (isAttached) (*svm)->DetachCurrentThread(svm); // Clean up.
    return (ret);
}

wiced_bool_t mesh_adv_scan_start(void)
{
    Log("mesh_adv_scan_start\n");
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    (*env)->CallStaticVoidMethod(env, cls2, meshGattAdvScanStartCb);
    return WICED_TRUE;
}

wiced_bool_t mesh_set_scan_type(uint8_t is_active)
{
    Log("mesh_set_adv_scan_type\n");
    JNIEnv *env = AttachJava();
    jbyte isactive = is_active;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    (*env)->CallStaticVoidMethod(env, cls2, meshGattSetScanTypeCb, isactive);
    return WICED_TRUE;
}



void mesh_adv_scan_stop(void)
{
    Log("mesh_adv_scan_stop\n");
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    (*env)->CallStaticVoidMethod(env, cls2, meshGattAdvScanStopCb);
}

wiced_bool_t mesh_bt_gatt_le_connect(wiced_bt_device_address_t bd_addr, wiced_bt_ble_address_type_t bd_addr_type,
                                            wiced_bt_ble_conn_mode_t conn_mode, wiced_bool_t is_direct)
{
    Log("mesh_bt_gatt_le_connect\n");
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyteArray  bda = (*env)->NewByteArray(env ,6);
    (*env)->SetByteArrayRegion(env,bda,0,6,bd_addr);
    (*env)->CallStaticVoidMethod(env, cls2, meshGattConnectCb, bda);
    return WICED_TRUE;
}

wiced_bool_t mesh_bt_gatt_le_disconnect(uint32_t conn_id)
{
    Log("mesh_bt_gatt_le_disconnect\n");
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    (*env)->CallStaticVoidMethod(env, cls2, meshGattDisconnectCb, conn_id);
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_SendGattPktToCore(JNIEnv *env, jclass type, jshort opcode_,
                                                                  jbyteArray p_data_, jint length_) {
    jbyte *p_data = (*env)->GetByteArrayElements(env, p_data_, NULL);
    uint16_t  opcode = opcode_;
    uint32_t length = length_;
   
    int i;
    Log("SendGattPktToCore\n");
   
    if(opcode == 0)//proxy gatt packet
    {
        pthread_mutex_lock(&cs);
        mesh_client_proxy_data((const uint8_t*)p_data,length);
        pthread_mutex_unlock(&cs);
    } else if (opcode == 1)
    {
        pthread_mutex_lock(&cs);
        mesh_client_provisioning_data(WICED_TRUE,(const uint8_t*)p_data,length);
        pthread_mutex_unlock(&cs);
    }
    else
    {
        Log("ignore the packet as it was neither proxy nor provis");
    }
    (*env)->ReleaseByteArrayElements(env, p_data_, p_data, 0);

}

wiced_result_t wiced_send_gatt_packet( uint16_t opcode, const uint8_t* p_data, uint16_t length ) {
    Log("wiced_send_gatt_packet\n");
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jshort event = opcode;
    jint lengthx = length;
    jbyteArray  data = (*env)->NewByteArray(env ,length);
    (*env)->SetByteArrayRegion(env,data,0,length,p_data);
    (*env)->CallStaticVoidMethod(env, cls2, processGattPktCb, event, data, lengthx);
    return WICED_TRUE;
}

void unprovisioned_device(uint8_t *p_uuid, uint16_t oob, uint8_t *name, uint8_t name_len)
{
    Log("unprovisioned_device\n");
    jstring deviceName;
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyteArray  data = (*env)->NewByteArray(env ,17);
    (*env)->SetByteArrayRegion(env,data,0,16,p_uuid);
    deviceName = (*env)->NewStringUTF(env, name);
    (*env)->CallStaticVoidMethod(env, cls2, meshClientUnProvisionedDeviceCb, data, oob, deviceName);
}

void meshClientProvisionCompleted(uint8_t is_success, uint8_t *p_uuid) {
    Log("meshClientProvisionCompleted\n");
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyte isSuccess = is_success;
    jint lengthx = 16;
    jbyteArray  data = (*env)->NewByteArray(env ,lengthx);
    (*env)->SetByteArrayRegion(env,data,0,lengthx,p_uuid);
    (*env)->CallStaticVoidMethod(env, cls2, meshClientProvisionCompletedCb, isSuccess, data);
}

void mesh_client_dfu_status(uint8_t status, int cur_block_number, int total_blocks)
{
    Log("mesh_client_dfu_status status:%x cur_block_number:%d", status, cur_block_number);
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyte statusval = status;
    (*env)->CallStaticVoidMethod(env, cls2, meshClientDfuStatusCb, statusval, cur_block_number, total_blocks);
}

/*
 * in general the application knows better when connection to the proxy is established or lost.
 * The only case when this function is called, when search for a node or a network times out.
 */
void linkStatus(uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt)
{
    Log("linkStatus is connected %x",is_connected);

    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyte isConnected = is_connected;
    jint connId = conn_id;
    jshort address = addr;
    jbyte isOverGatt = is_over_gatt;
    (*env)->CallStaticVoidMethod(env, cls2, meshClientLinkStatusCb, isConnected, connId, address, isOverGatt);
}

void meshClientNetworkOpened(uint8_t status)
{
    Log("meshClientNetworkOpened %x",status);

    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyte statusval = status;

    (*env)->CallStaticVoidMethod(env, cls2, meshClientNetworkOpenedCb, statusval);
}

void meshComponentInfoStatus(uint8_t status, char *component_name, char *component_info)
{
    Log("meshComponentInfoStatus %x",status);
    JNIEnv *env = AttachJava();
    jstring componentName;
    jstring componentInfo;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyte statusval = status;
    componentName = (*env)->NewStringUTF(env, component_name);
    componentInfo = (*env)->NewStringUTF(env, component_info);
    (*env)->CallStaticVoidMethod(env, cls2, meshClientComponentInfoCb, statusval, componentName, componentInfo);
}

void meshClientOnOffState(const char *device_name, uint8_t target, uint8_t present, uint32_t remaining_time) {
    Log("meshClientOnOffState\n");
    JNIEnv *env = AttachJava();
    jstring deviceName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyte onoff = target;
    deviceName = (*env)->NewStringUTF(env, device_name);
    (*env)->CallStaticVoidMethod(env, cls2, onOffStateCb, deviceName, onoff);
}

void meshClientSensorState(const char *device_name, int property_id, uint8_t length, uint8_t *value)
{
    Log("meshClientSensorStatus\n");
    JNIEnv *env = AttachJava();
    jstring deviceName;
    jstring targetName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyteArray  data = (*env)->NewByteArray(env ,length);
    (*env)->SetByteArrayRegion(env, data, 0, length, value);
    deviceName = (*env)->NewStringUTF(env, device_name);
    Log("meshClientSensorState status :%x\n",value[0]);
    (*env)->CallStaticVoidMethod(env, cls2, sensorStatuscb, deviceName, property_id, data);
}


void meshClientNodeConnectionState(uint8_t status, char *p_name) {
    Log("meshClientNodeConnectionState status :%x\n",status);
    JNIEnv *env = AttachJava();
    jstring deviceName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jbyte status_x = status;
    deviceName = (*env)->NewStringUTF(env, p_name);
    (*env)->CallStaticVoidMethod(env, cls2, meshClientNodeConnectStateCb, status_x, deviceName);
}

void meshClientDbChangedState(char *mesh_name) {
    Log("meshClientDatabaseChangedState status :%s\n",mesh_name);
    JNIEnv *env = AttachJava();
    jstring meshName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    meshName = (*env)->NewStringUTF(env, mesh_name);
    (*env)->CallStaticVoidMethod(env, cls2, meshClientDbStateCb, meshName);
}

void meshClientLevelState(const char *device_name, uint16_t target, uint16_t present, uint32_t remaining_time) {
    Log("meshClientLevelState\n");
    JNIEnv *env = AttachJava();
    jstring deviceName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jshort level_val = target;
    deviceName = (*env)->NewStringUTF(env, device_name);
    (*env)->CallStaticVoidMethod(env, cls2, levelStateCb, deviceName, level_val);
}


void meshClientHslState(const char *device_name, uint16_t lightness, uint16_t hue, uint16_t saturation, uint32_t remaining_time) {
    Log("meshClientHslState\n");
    JNIEnv *env = AttachJava();
    jstring deviceName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jint lightness_val = lightness;
    jint hue_val = hue;
    jint saturation_val = saturation;
    deviceName = (*env)->NewStringUTF(env, device_name);
    (*env)->CallStaticVoidMethod(env, cls2, meshClientHslStateCb, deviceName, lightness_val, hue_val, saturation_val);
}

void meshClientCtlState(const char *device_name, uint16_t present_lightness, uint16_t present_temperature, uint16_t target_lightness, uint16_t target_temperature, uint32_t remaining_time) {
    Log("meshClientCtlState\n");
    JNIEnv *env = AttachJava();
    jstring deviceName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jint presentLightness = present_lightness;
    jshort presentTemperature = present_temperature;
    jint targetLightness = target_lightness;
    jshort targetTemperature = target_temperature;
    jint remainingTime =  remaining_time;
    deviceName = (*env)->NewStringUTF(env, device_name);
    (*env)->CallStaticVoidMethod(env, cls2, meshClientCtlStateCb, deviceName, presentLightness, presentTemperature, targetLightness,targetTemperature, remainingTime);
}

void meshClientLightnessState(const char *device_name, uint16_t target, uint16_t present, uint32_t remaining_time) {
    Log("meshClientLightnessState\n");
    JNIEnv *env = AttachJava();
    jstring deviceName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jint presentval = present;
    jint targetval = target;
    jint remainingtime = remaining_time;
    deviceName = (*env)->NewStringUTF(env, device_name);
    (*env)->CallStaticVoidMethod(env, cls2, meshClientLightnessStateCb, deviceName, presentval, targetval, remainingtime);

}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientNetworkExists(JNIEnv *env, jclass type,
                                                                           jstring meshName_) {
    jint return_val;
    char *meshName = (*env)->GetStringUTFChars(env, meshName_, 0);


    pthread_mutex_lock(&cs);
    return_val = mesh_client_network_exists(meshName);
    pthread_mutex_unlock(&cs);
    (*env)->ReleaseStringUTFChars(env, meshName_, meshName);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientNetworkCreate(JNIEnv *env, jclass type,
                                                                           jstring provisionerName_,
                                                                           jstring meshName_) {
    char* provisioner_name;
    char* mesh_name;
    const char *provisionerName = (*env)->GetStringUTFChars(env, provisionerName_, 0);
    const char *meshName = (*env)->GetStringUTFChars(env, meshName_, 0);
    jint return_val;
    size_t prov_name_length = strlen(provisionerName);
    size_t mesh_name_length = strlen(meshName);

    provisioner_name = (char*)malloc(prov_name_length+1);
    mesh_name = (char*)malloc(mesh_name_length+1);
    strncpy(provisioner_name,provisionerName,prov_name_length);
    strncpy(mesh_name,meshName,mesh_name_length);

    provisioner_name[prov_name_length] = 0;
    mesh_name[mesh_name_length] = 0;
	// ++++
    pthread_mutex_lock(&cs);
    return_val = mesh_client_network_create(provisioner_name,provisioner_uuid,mesh_name);
    pthread_mutex_unlock(&cs);
	// ----

    (*env)->ReleaseStringUTFChars(env, provisionerName_, provisionerName);
    (*env)->ReleaseStringUTFChars(env, meshName_, meshName);
    free(provisioner_name);
    free(mesh_name);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientNetworkOpen(JNIEnv *env, jclass type,
                                                                         jstring provisionerName_,
                                                                         jstring meshName_) {
    char* provisioner_name;
    char* mesh_name;
    const char *provisionerName = (*env)->GetStringUTFChars(env, provisionerName_, 0);
    const char *meshName = (*env)->GetStringUTFChars(env, meshName_, 0);
    jint return_val;
    size_t prov_name_length = strlen(provisionerName);
    size_t mesh_name_length = strlen(meshName);

 

    provisioner_name = (char*)malloc(prov_name_length+1);
    mesh_name = (char*)malloc(mesh_name_length+1);

    strncpy(provisioner_name,provisionerName,prov_name_length);
    strncpy(mesh_name,meshName,mesh_name_length);

    provisioner_name[prov_name_length] = 0;
    mesh_name[mesh_name_length] = 0;
   Log("meshClientNetworkOpen %s", mesh_name);
 
	// ++++
    pthread_mutex_lock(&cs);
    return_val = mesh_client_network_open(provisioner_name, provisioner_uuid, mesh_name, meshClientNetworkOpened);
    pthread_mutex_unlock(&cs);
	// ----
    Log("return_val : %d\n", return_val);
    (*env)->ReleaseStringUTFChars(env, provisionerName_, provisionerName);
    (*env)->ReleaseStringUTFChars(env, meshName_, meshName);
    free(provisioner_name);
    free(mesh_name);
    return return_val;
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientNetworkClose(JNIEnv *env,
                                                                        jclass type) {
    pthread_mutex_lock(&cs);
    mesh_client_network_close();
    pthread_mutex_unlock(&cs);

}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGroupCreate(JNIEnv *env, jclass type,
                                                                         jstring groupName_,
                                                                         jstring parentGroupName_) {
    char* group_name;
    char* parent_group_name;
    const char *groupName = (*env)->GetStringUTFChars(env, groupName_, 0);
    const char *parentGroupName = (*env)->GetStringUTFChars(env, parentGroupName_, 0);
    jint return_val;
    size_t group_name_length = strlen(groupName);
    size_t parent_group_name_length = strlen(parentGroupName);

    group_name = (char*)malloc(group_name_length+1);
    parent_group_name = (char*)malloc(parent_group_name_length+1);


    strncpy(group_name,groupName,group_name_length);
    strncpy(parent_group_name,parentGroupName,parent_group_name_length);

    group_name[group_name_length] = 0;
    parent_group_name[parent_group_name_length] = 0;
    Log("Group Name :%s", group_name);
	
	// ++++
    pthread_mutex_lock(&cs);
    return_val = mesh_client_group_create(group_name,parent_group_name);
    pthread_mutex_unlock(&cs);
	// ----
	
    (*env)->ReleaseStringUTFChars(env, groupName_, groupName);
    (*env)->ReleaseStringUTFChars(env, parentGroupName_, parentGroupName);
    free(group_name);
    free(parent_group_name);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGroupDelete(JNIEnv *env, jclass type,
                                                                         jstring groupName_) {
    char* group_name;
    jint return_val;
    const char *groupName = (*env)->GetStringUTFChars(env, groupName_, 0);

    size_t group_name_length = strlen(groupName);
    group_name = (char*)malloc(group_name_length+1);
    strncpy(group_name,groupName,group_name_length);

    group_name[group_name_length] = 0;
	
	// ++++
    pthread_mutex_lock(&cs);
    return_val = mesh_client_group_delete(group_name);
    pthread_mutex_unlock(&cs);
	// ----
	
    (*env)->ReleaseStringUTFChars(env, groupName_, groupName);
    free(group_name);
    return return_val;
}

JNIEXPORT jobjectArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetAllNetworks(JNIEnv *env,
                                                                            jclass type) {
    int val =0,i;

    pthread_mutex_lock(&cs);
    char* networks = mesh_client_get_all_networks();
    pthread_mutex_unlock(&cs);

    Log("Name***:%s", networks);
    for (char *p_component_name = networks; p_component_name != NULL && *p_component_name != 0; p_component_name += (strlen(p_component_name) + 1), i++)
    {
        Log("Namefff:%s", p_component_name);
        val++;
    }

    jobjectArray networks_array;
    const jint ntwArraySize = val;
    jclass stringObject = (*env)->FindClass(env, "java/lang/String");

    networks_array = (*env)->NewObjectArray(env, ntwArraySize, stringObject, (*env)->NewStringUTF(env, NULL));
    for (char *p_component_name = networks, i=0; p_component_name != NULL && *p_component_name != 0; p_component_name += (strlen(p_component_name) + 1),i++)
    {
        (*env)->SetObjectArrayElement(env, networks_array, i, (*env)->NewStringUTF(env, p_component_name));
    }
    free(networks);

    return networks_array;
}


JNIEXPORT jobjectArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetAllGroups(JNIEnv *env, jclass type,
                                                                          jstring inGroup_) {
    char* ret_groups;
    int i,val =0;

    char *inGroup = (*env)->GetStringUTFChars(env, inGroup_, 0);

    pthread_mutex_lock(&cs);
    ret_groups = mesh_client_get_all_groups(inGroup);
    pthread_mutex_unlock(&cs);

    for (char *p_group_name = ret_groups; p_group_name != NULL && *p_group_name != 0; p_group_name += (strlen(p_group_name) + 1), i++)
    {
        val++;
    }
    Log("no of groups:%d", val);

    jobjectArray groups_array;
    const jint group_array_size = val;
    jclass stringObject = (*env)->FindClass(env, "java/lang/String");

    groups_array = (*env)->NewObjectArray(env, group_array_size, stringObject, (*env)->NewStringUTF(env, NULL));
    for (char *p_group_name = ret_groups, i=0; p_group_name != NULL && *p_group_name != 0; p_group_name += (strlen(p_group_name) + 1),i++)
    {
        (*env)->SetObjectArrayElement(env, groups_array, i, (*env)->NewStringUTF(env, p_group_name));
    }
    (*env)->ReleaseStringUTFChars(env, inGroup_, inGroup);
    free(ret_groups);
    return groups_array;

}


JNIEXPORT jobjectArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetAllProvisioners(JNIEnv *env,
                                                                                jclass type) {
    char* provisioners;
    int i;
    char* ntw = NULL;
    jobjectArray prov_array;
    jint ntwArraySize = 0;

    pthread_mutex_lock(&cs);
    provisioners = mesh_client_get_all_provisioners();
    pthread_mutex_unlock(&cs);

    for (char *p_prov_name = provisioners; p_prov_name != NULL && *p_prov_name != 0; p_prov_name += (strlen(p_prov_name) + 1), i++)
    {
        ntwArraySize++;
    }
    Log("no of provisioners:%d", ntwArraySize);

    jclass stringObject = (*env)->FindClass(env, "java/lang/String");

    prov_array = (*env)->NewObjectArray(env, ntwArraySize, stringObject, (*env)->NewStringUTF(env, NULL));
    for (char *p_prov_name = provisioners, i=0; p_prov_name != NULL && *p_prov_name != 0; p_prov_name += (strlen(p_prov_name) + 1),i++)
    {
        Log("prov:%s", p_prov_name);
        (*env)->SetObjectArrayElement(env, prov_array, i, (*env)->NewStringUTF(env, p_prov_name));
    }
    free(provisioners);
    return prov_array;
}

JNIEXPORT jobjectArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetDeviceComponents(JNIEnv *env,
                                                                                 jclass type,
                                                                                 jbyteArray p_uuid_) {

    int i;
    char* components;
    jobjectArray comp_array;
    jint comp_array_size = 0;
    jbyte *p_uuid = (*env)->GetByteArrayElements(env, p_uuid_, NULL);

    pthread_mutex_lock(&cs);
    components = mesh_client_get_device_components((const uint8_t*)p_uuid);
    pthread_mutex_unlock(&cs);

    for (char *p_component_name = components; p_component_name != NULL && *p_component_name != 0; p_component_name += (strlen(p_component_name) + 1), i++)
    {
        comp_array_size++;
    }
    Log("no of components:%d", comp_array_size);

    jclass stringObject = (*env)->FindClass(env, "java/lang/String");

    comp_array = (*env)->NewObjectArray(env, comp_array_size, stringObject, (*env)->NewStringUTF(env, NULL));
    for (char *p_component_name = components, i=0; p_component_name != NULL && *p_component_name != 0; p_component_name += (strlen(p_component_name) + 1),i++)
    {
        Log("component:%s", p_component_name);
        (*env)->SetObjectArrayElement(env, comp_array, i, (*env)->NewStringUTF(env, p_component_name));
    }
    (*env)->ReleaseByteArrayElements(env, p_uuid_, p_uuid, 0);
    free(components);
    return comp_array;
}

JNIEXPORT jobjectArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetGroupComponents(JNIEnv *env,
                                                                                jclass type,
                                                                                jstring groupName_) {
    char* ret_groups;
    int val =0,i;
    const char *groupName = (*env)->GetStringUTFChars(env, groupName_, 0);

    pthread_mutex_lock(&cs);
    ret_groups = mesh_client_get_group_components(groupName);
    pthread_mutex_unlock(&cs);

    for (char *p_component_name = ret_groups; p_component_name != NULL && *p_component_name != 0; p_component_name += (strlen(p_component_name) + 1), i++)
    {
        val++;
    }

    jobjectArray component_array;
    const jint cmpArraySize = val;
    jclass stringObject = (*env)->FindClass(env, "java/lang/String");

    component_array = (*env)->NewObjectArray(env, cmpArraySize, stringObject, (*env)->NewStringUTF(env, NULL));
    for (char *p_component_name = ret_groups, i=0; p_component_name != NULL && *p_component_name != 0; p_component_name += (strlen(p_component_name) + 1),i++)
    {
        (*env)->SetObjectArrayElement(env, component_array, i, (*env)->NewStringUTF(env, p_component_name));
    }
    (*env)->ReleaseStringUTFChars(env, groupName_, groupName);
    free(ret_groups);
    return component_array;
}

JNIEXPORT jbyte JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetComponentType(JNIEnv *env,
                                                                              jclass type,
                                                                              jstring componentName_) {
    int comp_type;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);

    pthread_mutex_lock(&cs);
    comp_type = mesh_client_get_component_type(componentName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);

    return comp_type;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientRename(JNIEnv *env,
                                                                             jclass type,
                                                                             jstring oldName_,
                                                                             jstring newName_) {

    jint return_val;
    const char *oldName = (*env)->GetStringUTFChars(env, oldName_, 0);
    const char *newName = (*env)->GetStringUTFChars(env, newName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_rename(oldName,newName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, oldName_, oldName);
    (*env)->ReleaseStringUTFChars(env, newName_, newName);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientMoveComponentToGroup(JNIEnv *env,
                                                                                  jclass type,
                                                                                  jstring componentName_,
                                                                                  jstring fromGroupName_,
                                                                                  jstring toGroupName_) {

    jint return_val;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    const char *fromGroupName = (*env)->GetStringUTFChars(env, fromGroupName_, 0);
    const char *toGroupName = (*env)->GetStringUTFChars(env, toGroupName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_move_component_to_group(componentName,fromGroupName, toGroupName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseStringUTFChars(env, fromGroupName_, fromGroupName);
    (*env)->ReleaseStringUTFChars(env, toGroupName_, toGroupName);

    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientConfigurePublication(JNIEnv *env,
                                                                              jclass type,
                                                                              jstring componentName_,
                                                                              jbyte isClient_,
                                                                              jstring method_,
                                                                              jstring targetName_,
                                                                              jint publishPeriod) {

    jint return_val;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    const char *methodName = (*env)->GetStringUTFChars(env, method_, 0);
    const char *targetName = (*env)->GetStringUTFChars(env, targetName_, 0);
    uint32_t publish_period = publishPeriod;

    pthread_mutex_lock(&cs);
    return_val = mesh_client_configure_publication(componentName, isClient_, methodName, targetName, publish_period);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseStringUTFChars(env, targetName_, targetName);
    (*env)->ReleaseStringUTFChars(env, method_, methodName);

    return return_val;
}

JNIEXPORT jbyte JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientProvision(JNIEnv *env, jclass type,
                                                                       jstring deviceName_,
                                                                       jstring groupName_,
                                                                       jbyteArray uuid_,
                                                                       jbyte identifyDuration) {

    int i;
    jbyte ret;
    const char *groupName = (*env)->GetStringUTFChars(env, groupName_, 0);
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);
    jbyte *p_uuid = (*env)->GetByteArrayElements(env, uuid_, NULL);

    pthread_mutex_lock(&cs);
    ret = mesh_client_provision(deviceName, groupName, (const uint8_t*)p_uuid,identifyDuration);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, groupName_, groupName);
    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);
    (*env)->ReleaseByteArrayElements(env, uuid_, p_uuid, 0);
    return ret;
}

JNIEXPORT jbyte JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientConnectNetwork(JNIEnv *env,
                                                                            jclass type,
                                                                            jbyte useGattProxy,
                                                                            jbyte scanDuration) {
    jbyte ret;
    pthread_mutex_lock(&cs);
    ret = mesh_client_connect_network(useGattProxy, scanDuration);
    pthread_mutex_unlock(&cs);
    return ret;
}

JNIEXPORT jbyte JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientDisconnectNetwork(JNIEnv *env,
                                                                               jclass type,
                                                                               jbyte useGattProxy) {
    pthread_mutex_lock(&cs);
    jbyte ret = mesh_client_disconnect_network();
    pthread_mutex_unlock(&cs);
    return ret;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientonoffGet(JNIEnv *env, jclass type,
                                                                      jstring deviceName_) {

    int return_val;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_on_off_get(deviceName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);

    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientonoffSet(JNIEnv *env, jclass type,
                                                                      jstring deviceName_,
                                                                      jbyte onoff, jboolean reliable,jint transitionTime, jshort delay) {

    int return_val;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_on_off_set(deviceName, onoff, reliable, transitionTime, delay);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientLevelGet(JNIEnv *env, jclass type,
                                                                      jstring deviceName_) {

    int return_val;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_level_get(deviceName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientLevelSet(JNIEnv *env, jclass type,
                                                                      jstring deviceName_,
                                                                      jshort level, jboolean reliable,
                                                                      jint transitionTime, jshort delay) {

    int return_val;
    int16_t level_val;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);
    level_val = level;

    pthread_mutex_lock(&cs);
    return_val = mesh_client_level_set(deviceName, level_val, reliable, transitionTime, delay);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientHslGet(JNIEnv *env, jclass type,
                                                                    jstring deviceName_) {
    int return_val;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_hsl_get(deviceName);
    pthread_mutex_unlock(&cs);
	
    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientHslSet(JNIEnv *env, jclass type,
                                                                    jstring deviceName_,
                                                                    jint lightness, jint hue,
                                                                    jint saturation, jboolean reliable,
                                                                    jint transitionTime, jshort delay) {

    int return_val;
    uint32_t lightness_val;
    uint32_t hue_val;
    uint32_t saturation_val;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);

    lightness_val = lightness;
    hue_val = hue;
    saturation_val = saturation;

    pthread_mutex_lock(&cs);
    return_val = mesh_client_hsl_set(deviceName, lightness_val, hue_val, saturation_val, reliable, transitionTime, delay);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);

    return return_val;
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_timerCallback(JNIEnv *env, jclass type,
                                                                 jlong timerid) {
    MeshTimerFunc(timerid);
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSetDeviceConfig(JNIEnv *env,
                                                                             jclass type,
                                                                             jstring deviceName_,
                                                                             jint isGattProxy,
                                                                             jint isFriend,
                                                                             jint isRelay,
                                                                             jint beacon_,
                                                                             jint relayXmitCount,
                                                                             jint relayXmitInterval,
                                                                             jint defaultTtl,
                                                                             jint netXmitCount,
                                                                             jint netXmitInterval) {

    jint return_val;
    uint32_t is_gatt_proxy = isGattProxy;
    uint32_t is_friend = isFriend;
    uint32_t is_relay = isRelay;
    uint32_t beacon = beacon_;
    uint32_t relay_xmit_count = relayXmitCount;
    uint32_t relay_xmit_interval = relayXmitInterval;
    uint32_t default_ttl = defaultTtl;
    uint32_t net_xmit_count = netXmitCount;
    uint32_t net_xmit_interval = netXmitInterval;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_set_device_config(
            strcmp("",deviceName) == 0 ? NULL:deviceName,
            is_gatt_proxy,
            is_friend,
            is_relay,
            beacon,
            relay_xmit_count,
            relay_xmit_interval,
            default_ttl,
            net_xmit_count,
            net_xmit_interval);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);

    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSetPublicationConfig(JNIEnv *env,
                                                                                  jclass type,
                                                                                  jint publishCredentialFlag,
                                                                                  jint publishRetransmitCount,
                                                                                  jint publishRetransmitInterval,
                                                                                  jint publishTtl) {

    jint return_val;
    uint32_t publish_credential_flag = publishCredentialFlag;
    uint32_t publish_retransmit_count = publishRetransmitCount;
    uint32_t publish_retransmit_interval = publishRetransmitInterval;
    uint32_t publish_ttl = publishTtl;


    pthread_mutex_lock(&cs);
    return_val = mesh_client_set_publication_config(
            publish_credential_flag,
            publish_retransmit_count,
            publish_retransmit_interval,
            publish_ttl);
    pthread_mutex_unlock(&cs);

    return return_val;
}

JNIEXPORT jbyte JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientResetDevice(JNIEnv *env, jclass type,
                                                                         jstring componentName_) {

    jbyte return_val;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_reset_device(componentName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);

    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientVendorDataSet(JNIEnv *env, jclass type,
                                                                           jstring deviceName_,
                                                                           jbyteArray buffer_,
                                                                           jshort len) {


    jint return_val;
    int i=0;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);
    jbyte *buffer = (*env)->GetByteArrayElements(env, buffer_, NULL);



    pthread_mutex_lock(&cs);
    return_val = mesh_client_vendor_data_set(deviceName, (const uint8_t*)buffer, len);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);
    (*env)->ReleaseByteArrayElements(env, buffer_, buffer, 0);

    return return_val;
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientScanUnprovisioned(JNIEnv *env,
                                                                               jclass type,
                                                                               jint start) {
    Log("MeshClientScanUnprovisioned %d",start);

	pthread_mutex_lock(&cs);
    mesh_client_scan_unprovisioned(start);
    pthread_mutex_unlock(&cs);

}

JNIEXPORT jboolean JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientIsConnectingProvisioning(JNIEnv *env,
                                                                                      jclass type) {
    Log("meshClientIsConnectingProvisioning");
    jboolean  res;

    pthread_mutex_lock(&cs);
    res = mesh_client_is_connecting_provisioning();
    pthread_mutex_unlock(&cs);
    return  res;

}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientConnectionStateChanged(JNIEnv *env,
                                                                                    jclass type,
                                                                                    jshort conn_id,
                                                                                    jshort mtu) {
    Log("meshClientConnectionStateChanged");
    mesh_client_connection_state_changed(conn_id, mtu);
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientAdvertReport(JNIEnv *env, jclass type,
                                                                          jbyteArray bdaddr_,
                                                                          jbyte addrType,
                                                                          jbyte rssi,
                                                                          jbyteArray advData_,
                                                                          jint advLen_) {
    Log("meshClientAdvertReport begin");

    jbyte *bdaddr = (*env)->GetByteArrayElements(env, bdaddr_, NULL);
    jbyte *advData = (*env)->GetByteArrayElements(env, advData_, NULL);

    pthread_mutex_lock(&cs);
    mesh_client_advert_report((const uint8_t*)bdaddr, (uint8_t)addrType, (int8_t)rssi, (const uint8_t*)advData);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseByteArrayElements(env, bdaddr_, bdaddr, 0);
    (*env)->ReleaseByteArrayElements(env, advData_, advData, 0);
    Log("meshClientAdvertReport end");

}
void mesh_provision_gatt_send(uint16_t conn_id, const uint8_t *packet, uint32_t packet_len){
    // send this packet to JAVA
    Log("\n mesh_provision_gatt_send");
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
     jint lengthx = packet_len;
    jbyteArray  data = (*env)->NewByteArray(env ,packet_len);
    (*env)->SetByteArrayRegion(env,data,0,packet_len,packet);
    (*env)->CallStaticVoidMethod(env, cls2, meshGattProvisSendCb,data, lengthx);
}

void proxy_gatt_send_cb(uint32_t conn_id, uint32_t ref_data, const uint8_t *packet, uint32_t packet_len){
    // send this packet to JAVA
    Log("\n proxy_gatt_send_cb");
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jint lengthx = packet_len;
    jbyteArray  data = (*env)->NewByteArray(env ,packet_len);
    (*env)->SetByteArrayRegion(env,data,0,packet_len,packet);
    (*env)->CallStaticVoidMethod(env, cls2, meshGattProxySendCb,data, lengthx);
}

uint32_t start_timer(uint32_t timeout, uint16_t type) {
    JNIEnv *env = AttachJava();
    jstring deviceName;
    uint32_t curr_timerid = timer_id;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jint time = timeout;
    Log("start_timer timer_id:%x\n",curr_timerid);
    (*env)->CallStaticVoidMethod(env, cls2, startTimercb, curr_timerid, time, type);
    timer_id++;
    return curr_timerid;
}

uint32_t restart_timer(uint32_t timeout, uint32_t timerId ) {
    Log("restart_timer timer_id:%x timeout:%d\n", timerId, timeout);
    JNIEnv *env = AttachJava();
    jstring deviceName;
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jint time = timeout;
    (*env)->CallStaticVoidMethod(env, cls2, startTimercb, timerId, time);
}
void stop_timer(uint32_t timerId){
    Log("stop_timer timer_id:%x\n",timerId);
    JNIEnv *env = AttachJava();
    jclass cls2 = (*env)->FindClass(env,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    jint time_id = timerId;
    (*env)->CallStaticVoidMethod(env, cls2, stopTimercb, time_id);
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientIdentify(JNIEnv *env, jclass type,
                                                                      jstring name_,
                                                                      jbyte duration_) {

    jbyte duration;
    jint ret_value;
    const char *p_name = (*env)->GetStringUTFChars(env, name_, 0);

    duration = duration_;

    pthread_mutex_lock(&cs);
    ret_value = mesh_client_identify(p_name, duration);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, name_, p_name);
    return ret_value;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientLightnessGet(JNIEnv *env, jclass type,
                                                                          jstring deviceName_) {

    jint ret_value;
    const char *p_name = (*env)->GetStringUTFChars(env, deviceName_, 0);

    pthread_mutex_lock(&cs);
    ret_value = mesh_client_lightness_get(p_name);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, p_name);

    return ret_value;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientLightnessSet(JNIEnv *env, jclass type,
                                                                          jstring deviceName_,
                                                                          jint lightness_,
                                                                          jboolean interim_,
                                                                          jint transition_time_,
                                                                          jshort delay_) {

    jint ret_value;
    uint32_t lightness;
    uint16_t delay;
    uint32_t transition_time;
    wiced_bool_t interim;
    const char *p_name = (*env)->GetStringUTFChars(env, deviceName_, 0);
    lightness = lightness_;
    interim = interim_;
    transition_time = transition_time_;
    delay = delay_;


    pthread_mutex_lock(&cs);
    ret_value = mesh_client_lightness_set(p_name,lightness, interim, transition_time, delay);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, p_name);
    return ret_value;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientCtlGet(JNIEnv *env, jclass type,
                                                                    jstring deviceName_)
{

    jint ret_value;
    const char *p_name = (*env)->GetStringUTFChars(env, deviceName_, 0);

    pthread_mutex_lock(&cs);
    ret_value = mesh_client_ctl_get(p_name);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, p_name);
    return ret_value;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientCtlSet(JNIEnv *env, jclass type,
                                                                    jstring deviceName_,
                                                                    jint lightness_,
                                                                    jshort temperature_,
                                                                    jshort deltaUv_,
                                                                    jboolean reliable_,
                                                                    jint transition_time_,
                                                                    jshort delay_) {

    jint ret_value;
    uint16_t lightness;
    uint16_t delay;
    uint32_t transition_time;
    wiced_bool_t reliable;
    uint16_t temperature;
    uint16_t delta_uv;
    const char *p_name = (*env)->GetStringUTFChars(env, deviceName_, 0);
    lightness = lightness_;
    reliable = reliable_;
    transition_time = transition_time_;
    delay = delay_;
    temperature = temperature_;
    delta_uv = deltaUv_;

    pthread_mutex_lock(&cs);
    ret_value = mesh_client_ctl_set(p_name,lightness, temperature,delta_uv, reliable, transition_time, delay);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, deviceName_, p_name);
    return ret_value;
}

JNIEXPORT jobjectArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetTargetMethods(JNIEnv *env,
                                                                              jclass type,
                                                                              jstring componentName_) {

    char* ret_target_methods;
    int val =0,i;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);


    pthread_mutex_lock(&cs);
    ret_target_methods = mesh_client_get_target_methods(componentName);
    pthread_mutex_unlock(&cs);

    for (char *p_method_name = ret_target_methods; p_method_name != NULL && *p_method_name != 0; p_method_name += (strlen(p_method_name) + 1), i++)
    {
        Log("Target methods:%s", p_method_name);
        val++;
    }

    jobjectArray method_array;
    const jint cmpArraySize = val;
    jclass stringObject = (*env)->FindClass(env, "java/lang/String");

    method_array = (*env)->NewObjectArray(env, cmpArraySize, stringObject, (*env)->NewStringUTF(env, NULL));
    for (char *p_method_name = ret_target_methods, i=0; p_method_name != NULL && *p_method_name != 0; p_method_name += (strlen(p_method_name) + 1),i++)
    {
        (*env)->SetObjectArrayElement(env, method_array, i, (*env)->NewStringUTF(env, p_method_name));
    }
    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);

    free(ret_target_methods);
    return method_array;
}

JNIEXPORT jobjectArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetControlMethods(JNIEnv *env,
                                                                               jclass type,
                                                                               jstring componentName_) {
    char* ret_control_methods;
    int val =0,i;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);

    pthread_mutex_lock(&cs);
    ret_control_methods = mesh_client_get_control_methods(componentName);
    pthread_mutex_unlock(&cs);

    for (char *p_method_name = ret_control_methods; p_method_name != NULL && *p_method_name != 0; p_method_name += (strlen(p_method_name) + 1), i++)
    {
        Log("control methods:%s", p_method_name);
        val++;
    }

    jobjectArray method_array;
    const jint cmpArraySize = val;
    jclass stringObject = (*env)->FindClass(env, "java/lang/String");

    method_array = (*env)->NewObjectArray(env, cmpArraySize, stringObject, (*env)->NewStringUTF(env, NULL));
    for (char *p_method_name = ret_control_methods, i=0; p_method_name != NULL && *p_method_name != 0; p_method_name += (strlen(p_method_name) + 1),i++)
    {
        (*env)->SetObjectArrayElement(env, method_array, i, (*env)->NewStringUTF(env, p_method_name));
    }
    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    free(ret_control_methods);
    return method_array;
}

JNIEXPORT jbyte JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshConnectComponent(JNIEnv *env, jclass type,
                                                                        jstring componentName_,
                                                                        jbyte useProxy,
                                                                        jbyte scanDuration) {

    jint ret_value;
    const char *p_name = (*env)->GetStringUTFChars(env, componentName_, 0);

    pthread_mutex_lock(&cs);
    ret_value = mesh_client_connect_component(p_name, useProxy, scanDuration);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, p_name);
    return ret_value;
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_setFileStorge(JNIEnv *env, jobject instance,
                                                                 jstring fileStorge_) {

    struct stat st = {0};
    memset(pathname,0,100);

    // Extract a C string from the String object, and chdir() to it.
    const char *wpath3 = (*env)->GetStringUTFChars(env, fileStorge_, NULL);
    strcpy(pathname, wpath3);
    strcat(pathname,"/mesh");
    Log("path %s.\n", pathname);

    if (stat(pathname, &st) == -1) {
        mkdir(pathname, 0777);
    }
    if (chdir(pathname) != 0) {
        Log("Error: Unable to change working directory to %s\n", pathname);
        perror(NULL);
    }

    create_prov_uuid();
    (*env)->ReleaseStringUTFChars(env, fileStorge_, wpath3);

}

JNIEXPORT jstring JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientNetworkImport(JNIEnv *env, jclass type,
                                                                           jstring provName_,
                                                                           jstring jsonStr_) {
    char* network_name;
    const char *p_prov_name = (*env)->GetStringUTFChars(env, provName_, 0);
    const char *p_jstr = (*env)->GetStringUTFChars(env, jsonStr_, 0);

    pthread_mutex_lock(&cs);
    network_name = mesh_client_network_import(p_prov_name, provisioner_uuid, p_jstr, meshClientNetworkOpened);
    pthread_mutex_unlock(&cs);

    jstring networkName = (*env)->NewStringUTF(env, network_name);

    (*env)->ReleaseStringUTFChars(env, jsonStr_, p_jstr);
    (*env)->ReleaseStringUTFChars(env, provName_, p_prov_name);

    return networkName;
}

JNIEXPORT jstring JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientNetworkExport(JNIEnv *env, jclass type,
                                                                           jstring meshName_) {

    char* json_str;
    const char *p_name = (*env)->GetStringUTFChars(env, meshName_, 0);

    pthread_mutex_lock(&cs);
    json_str = mesh_client_network_export(p_name);
    pthread_mutex_unlock(&cs);

    jstring jsonStr = (*env)->NewStringUTF(env, json_str);
    (*env)->ReleaseStringUTFChars(env, meshName_, p_name);
    free(json_str);
    return jsonStr;
}
void create_prov_uuid()
{
    FILE *fp = fopen("prov_uuid.bin", "rb");
    if (fp == NULL)
    {
        provisioner_uuid[32] = 0;
        Log("create prov_uuid.bin");
        fp = fopen("prov_uuid.bin", "wb");
        for (int i = 0; i < 8; i++)
            sprintf(&provisioner_uuid[i * 4], "%04X", rand());
        fwrite(provisioner_uuid, 1, 33, fp);
    }else{
        Log("prov_uuid.bin already exists");
        fread(provisioner_uuid,sizeof(provisioner_uuid),1,fp);
    }
    fclose(fp);
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientNetworkDelete(JNIEnv *env, jclass type,
                                                                           jstring provisionerName_,
                                                                           jstring meshName_) {
    int return_val;
    const char *provisionerName = (*env)->GetStringUTFChars(env, provisionerName_, 0);
    const char *meshName = (*env)->GetStringUTFChars(env, meshName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_network_delete(provisionerName,provisioner_uuid,meshName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, provisionerName_, provisionerName);
    (*env)->ReleaseStringUTFChars(env, meshName_, meshName);

    return return_val;
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSetGattMtu(JNIEnv *env, jclass type,
                                                                        jint mtu) {

    uint32_t mtu_size = mtu;
    Log("meshClientSetGattMtu");
    pthread_mutex_lock(&cs);
    wiced_bt_mesh_core_set_gatt_mtu(mtu_size);
    pthread_mutex_unlock(&cs);
}

JNIEXPORT jboolean JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientIsConnectedToNetwork(JNIEnv *env,
                                                                                  jclass type) {
    return mesh_client_is_proxy_connected();
}

JNIEXPORT jbyte JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetComponentInfo(JNIEnv *env,
                                                                              jclass type,
                                                                              jstring componentName_) {
    jbyte result;
    const char *p_name = (*env)->GetStringUTFChars(env, componentName_, 0);
    pthread_mutex_lock(&cs);
    result = mesh_client_get_component_info(p_name, &meshComponentInfoStatus);
    pthread_mutex_unlock(&cs);
    (*env)->ReleaseStringUTFChars(env, componentName_, p_name);
    return result;

}

JNIEXPORT jbyteArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientOTADataEncrypt(JNIEnv *env,
                                                                            jclass type,
                                                                            jstring componentName_,
                                                                            jbyteArray buffer_,
                                                                            jint len) {
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    jbyte *p_buffer = (*env)->GetByteArrayElements(env, buffer_, NULL);

    Log("meshClientOTADataEncrypt");
    // The output buffer should be at least 17 bytes larger than input buffer
    uint8_t* p_out_buffer = (char*) malloc(len + 17);
    int out_buf_len ;

    pthread_mutex_lock(&cs);
    out_buf_len = mesh_client_ota_data_encrypt(componentName, (const uint8_t*)p_buffer,len, p_out_buffer,(len+17));
    pthread_mutex_unlock(&cs);

    jbyteArray result=(*env)->NewByteArray(env, out_buf_len);
    (*env)->SetByteArrayRegion(env, result, 0, out_buf_len, p_out_buffer);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseByteArrayElements(env, buffer_, p_buffer, 0);
    free(p_out_buffer);
    p_out_buffer = NULL;
    return  result;
}


JNIEXPORT jbyteArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientOTADataDecrypt(JNIEnv *env,
                                                                            jclass type,
                                                                            jstring componentName_,
                                                                            jbyteArray buffer_,
                                                                            jint len) {
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    jbyte *p_buffer = (*env)->GetByteArrayElements(env, buffer_, NULL);

    Log("meshClientOTADataDecrypt");

    uint8_t* p_out_buffer = (char*) malloc(len + 17);
    int out_buf_len ;

    pthread_mutex_lock(&cs);
    out_buf_len = mesh_client_ota_data_decrypt(componentName, (const uint8_t*)p_buffer,len, p_out_buffer,(len+17));
    pthread_mutex_unlock(&cs);
    jbyteArray result=(*env)->NewByteArray(env, out_buf_len);
    (*env)->SetByteArrayRegion(env, result, 0, out_buf_len, p_out_buffer);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseByteArrayElements(env, buffer_, p_buffer, 0);
     free(p_out_buffer);
     p_out_buffer = NULL;
    return  result;
}

mesh_client_init_t mesh_client_init_callbacks =
        {
                unprovisioned_device,
                meshClientProvisionCompleted,
                linkStatus,
                meshClientNodeConnectionState,
                meshClientDbChangedState,
                meshClientOnOffState,
                meshClientLevelState,
                meshClientLightnessState,
                meshClientHslState,
                meshClientCtlState,
                meshClientSensorState
        };



jint
JNI_OnLoad(JavaVM *vm, void *reserved)
{
    Log("OnLoad");
    JNIEnv  *env;
    svm = vm;
    if ((*vm)->GetEnv(vm, (void **)&env, JNI_VERSION_1_6) != JNI_OK)
        return -1;

    sCallbackEnv = env;
    jniWrapperClass = (*sCallbackEnv)->FindClass(sCallbackEnv,"com/cypress/le/mesh/meshcore/MeshNativeHelper");
    if (jniWrapperClass == 0) {
        Log("NO CLASS");
    }

    processDataCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "ProcessData", "(S[BI)V");
    if(processDataCb == NULL) Log("processDataCb is null");

    processGattPktCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "ProcessGattPacket", "(S[BI)V");
    if(processGattPktCb == NULL) Log("processGattPktCb is null");

    meshClientUnProvisionedDeviceCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientUnProvisionedDeviceCb", "([BILjava/lang/String;)V");
    if(meshClientUnProvisionedDeviceCb == NULL) Log("meshClientUnProvisionedDeviceCb is null");

    meshGattAdvScanStartCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientAdvScanStartCb", "()V");
    if(meshGattAdvScanStartCb == NULL) Log("meshGattAdvScanStartCb is null");

    meshGattSetScanTypeCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientSetAdvScanTypeCb", "(B)V");
    if(meshGattAdvScanStartCb == NULL) Log("meshGattSetScanTypeCb is null");

    meshGattAdvScanStopCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientAdvScanStopCb", "()V");
    if(meshGattAdvScanStopCb == NULL) Log("meshGattAdvScanStopCb is null");
    meshGattProvisSendCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientProvSendCb", "([BI)V");
    if(meshGattProvisSendCb == NULL) Log("meshGattProvisSendCb is null");

    meshGattProxySendCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientProxySendCb", "([BI)V");
    if(meshGattProxySendCb == NULL) Log("meshGattProxySendCb is null");

    meshGattConnectCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientConnectCb", "([B)V");
    if(meshGattConnectCb == NULL) Log("meshGattConnectCb is null");

    meshGattDisconnectCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientDisconnectCb", "(S)V");
    if(meshGattDisconnectCb == NULL) Log("meshGattDisconnectCb is null");

    meshClientProvisionCompletedCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientProvisionCompletedCb", "(B[B)V");
    if(meshClientProvisionCompletedCb == NULL) Log("provisionCompletedCb is null");

    onOffStateCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientOnOffStateCb", "(Ljava/lang/String;B)V");
    if(onOffStateCb == NULL) Log("onOffStateCb is null");

    levelStateCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientLevelStateCb", "(Ljava/lang/String;S)V");
    if(levelStateCb == NULL) Log("levelStateCb is null");

    meshClientHslStateCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientHslStateCb", "(Ljava/lang/String;III)V");
    if(meshClientHslStateCb == NULL) Log("meshClientHslStateCb is null");

    meshClientLightnessStateCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientLightnessStateCb", "(Ljava/lang/String;III)V");
    if(meshClientLightnessStateCb == NULL) Log("meshClientLightnessStateCb is null");

    meshClientCtlStateCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientCtlStateCb", "(Ljava/lang/String;ISISI)V");
    if(meshClientCtlStateCb == NULL) Log("meshClientCtlStateCb is null");

    meshClientNodeConnectStateCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientNodeConnectStateCb", "(BLjava/lang/String;)V");
    if(meshClientNodeConnectStateCb == NULL) Log("meshClientNodeConnectStateCb is null");

    meshClientDbStateCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientDbStateCb", "(Ljava/lang/String;)V");
    if(meshClientDbStateCb == NULL) Log("meshClientDbStateCb is null");

    meshClientLinkStatusCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientLinkStatusCb", "(BISB)V");
    if(meshClientLinkStatusCb == NULL) Log("meshClientLinkStatusCb is null");

    meshClientDfuStatusCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientDfuStatusCb", "(BII)V");
    if(meshClientDfuStatusCb == NULL) Log("meshClientDfuStatusCb is null");

    meshClientNetworkOpenedCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientNetworkOpenedCb", "(B)V");
    if(meshClientNetworkOpenedCb == NULL) Log("meshClientNetworkOpenedCb is null");

    meshClientComponentInfoCb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientComponentInfoCallback", "(BLjava/lang/String;Ljava/lang/String;)V");
    if(meshClientComponentInfoCb == NULL) Log("meshClientComponentInfoCallback is null");

    startTimercb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "startTimercb", "(II)V");
    if(startTimercb == NULL) Log("startTimercb is null");

    stopTimercb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "stopTimercb", "(I)V");
    if(startTimercb == NULL) Log("stopTimercb is null");

    sensorStatuscb = (*sCallbackEnv)->GetStaticMethodID(sCallbackEnv, jniWrapperClass, "meshClientSensorStatusCb", "(Ljava/lang/String;I[B)V");
    if(startTimercb == NULL) Log("sensorStatuscb is null");


    //cdToExtStorage();


    //setting seed for random number generation
    srand ( time(NULL) );



    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientInit(JNIEnv *env, jclass type) {
    Log("meshClientInit");
    pthread_mutex_lock(&cs);
    mesh_client_init(&mesh_client_init_callbacks);
    pthread_mutex_unlock(&cs);
}

JNIEXPORT jobjectArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetComponentGroupList(JNIEnv *env,
                                                                                   jclass type,
                                                                                   jstring componentName_) {
    Log("GetComponentGroupList");

    char* ret_groups;
    int val =0,i;

    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    pthread_mutex_lock(&cs);
    ret_groups = mesh_client_get_component_group_list(componentName);
    pthread_mutex_unlock(&cs);

    for (char *p_group_name = ret_groups; p_group_name != NULL && *p_group_name != 0; p_group_name += (strlen(p_group_name) + 1), i++)
    {
        Log("group :%s", p_group_name);
        val++;
    }
    Log("num groups :%d", val);

    jobjectArray group_array;
    const jint group_array_size = val;
    jclass stringObject = (*env)->FindClass(env, "java/lang/String");

    group_array = (*env)->NewObjectArray(env, group_array_size, stringObject, (*env)->NewStringUTF(env, NULL));
    for (char *p_group_name = ret_groups, i=0; p_group_name != NULL && *p_group_name != 0; p_group_name += (strlen(p_group_name) + 1),i++)
    {
        (*env)->SetObjectArrayElement(env, group_array, i, (*env)->NewStringUTF(env, p_group_name));
    }

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    free(ret_groups);
    return group_array;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientRemoveComponentFromGroup(JNIEnv *env,
                                                                                      jclass type,
                                                                                      jstring componentName_,
                                                                                      jstring groupName_) {
    Log("RemoveComponentFromGroup");
    jint return_val,i=0;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    const char *groupName = (*env)->GetStringUTFChars(env, groupName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_remove_component_from_group(componentName,groupName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseStringUTFChars(env, groupName_, groupName);

    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientAddComponentToGroup(JNIEnv *env,
                                                                                 jclass type,
                                                                                 jstring componentName_,
                                                                                 jstring groupName_) {
    Log("AddComponentToGroup");
    jint return_val;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    const char *groupName = (*env)->GetStringUTFChars(env, groupName_, 0);

    pthread_mutex_lock(&cs);
    return_val = mesh_client_add_component_to_group(componentName,groupName);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseStringUTFChars(env, groupName_, groupName);
    return return_val;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientDfuGetStatus(JNIEnv *env,
                                                                          jclass type,
                                                                          jstring componentName_) {
    mesh_dfu_fw_id_t fw_id;
    int res;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    Log("meshClientDfuGetStatus");
    pthread_mutex_lock(&cs);
    if (!get_firmware_id_from_file(fw_metadata_file_name, &fw_id))
        return MESH_CLIENT_ERR_NOT_FOUND;

    res = mesh_client_dfu_get_status(componentName, fw_id.company_id, fw_id.fw_id, fw_id.fw_id_len, &mesh_client_dfu_status);
    pthread_mutex_unlock(&cs);
    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    return  res;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientDfuStart(JNIEnv *env, jclass type,
                                                                      jbyte dfuMethod,
                                                                      jstring componentName_) {
    int res;
    mesh_dfu_fw_id_t fw_id;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    Log("meshClientDfuStart %s",fw_metadata_file_name);
    pthread_mutex_lock(&cs);

    if (!get_firmware_id_from_file(fw_metadata_file_name, &fw_id))
        return MESH_CLIENT_ERR_NOT_FOUND;
    res = mesh_client_dfu_start(dfuMethod, componentName, fw_id.company_id, fw_id.fw_id, fw_id.fw_id_len);
    pthread_mutex_unlock(&cs);
    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    return res;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientDfuStop(JNIEnv *env, jclass type) {
    int res;
    pthread_mutex_lock(&cs);
    res = mesh_client_dfu_stop();
    pthread_mutex_unlock(&cs);
    return res;
}

JNIEXPORT void JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSetDfuFiles(JNIEnv *env, jclass type,
                                                                         jstring fw_fileName_,
                                                                         jstring metadata_file_)
{
    const char *fw_fileName = (*env)->GetStringUTFChars(env, fw_fileName_, 0);
    const char *metadata_file = (*env)->GetStringUTFChars(env, metadata_file_, 0);

    strcpy(fw_file_name, fw_fileName);
    strcpy(fw_metadata_file_name, metadata_file);

    Log("fw file :%s",fw_fileName);
    Log("metadata file :%s",metadata_file);

    (*env)->ReleaseStringUTFChars(env, fw_fileName_, fw_fileName);
    (*env)->ReleaseStringUTFChars(env, metadata_file_, metadata_file);
}

wiced_bool_t get_firmware_id_from_file(char* sFilePath, mesh_dfu_fw_id_t *fwID)
{
    FILE *pFile;
    if (!(pFile = fopen(sFilePath, L"r")))
        return FALSE;

    char line[100];
    DWORD cid = 0;
    long fwid = 0;

    while (fgets(line, 100, pFile))
    {
        if (strstr(line, "CID=") == line)
            sscanf(&line[4], "0x%x", &cid);
        else if (strstr(line, "FWID=") == line)
            sscanf(&line[5], "0x%llx", &fwid);
    }

    fwID->company_id = (uint16_t)cid;
    fwID->fw_id_len = 8;
    fwID->fw_id[0] = (uint8_t)((fwid >> 56) & 0xFF);
    fwID->fw_id[1] = (uint8_t)((fwid >> 48) & 0xFF);
    fwID->fw_id[2] = (uint8_t)((fwid >> 40) & 0xFF);
    fwID->fw_id[3] = (uint8_t)((fwid >> 32) & 0xFF);
    fwID->fw_id[4] = (uint8_t)((fwid >> 24) & 0xFF);
    fwID->fw_id[5] = (uint8_t)((fwid >> 16) & 0xFF);
    fwID->fw_id[6] = (uint8_t)((fwid >> 8) & 0xFF);
    fwID->fw_id[7] = (uint8_t)(fwid & 0xFF);

    fclose(pFile);
    return TRUE;
}
JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientNetworkConnectionChanged(JNIEnv *env,
                                                                                      jclass type,
                                                                                      jint connId) {
    int res = 0;
    pthread_mutex_lock(&cs);
    mesh_client_connection_state_changed(connId, 75);
    pthread_mutex_unlock(&cs);
    return res;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSensorSettingSet(JNIEnv *env,
                                                                              jclass type,
                                                                              jstring componentName_,
                                                                              jint propertyId,
                                                                              jshort settingPropertyId,
                                                                              jbyteArray val_)
{
    Log("SensorSettingSet\n");
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    jbyte *val = (*env)->GetByteArrayElements(env, val_, NULL);
    int ret;

    pthread_mutex_lock(&cs);
    ret = mesh_client_sensor_setting_set(componentName, propertyId, settingPropertyId, val);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseByteArrayElements(env, val_, val, 0);
    return  ret;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSensorCadenceSet(JNIEnv *env,
                                                                              jclass type,
                                                                              jstring deviceName_,
                                                                              jint propertyId,
                                                                              jshort fastCadencePeriodDivisor,
                                                                              jboolean triggerType,
                                                                              jint triggerDeltaDown,
                                                                              jint triggerDeltaUp,
                                                                              jint minInterval,
                                                                              jint fastCadenceLow,
                                                                              jint fastCadenceHigh) {
    int ret;
    const char *deviceName = (*env)->GetStringUTFChars(env, deviceName_, 0);

    pthread_mutex_lock(&cs);
    ret = mesh_client_sensor_cadence_set(deviceName, propertyId, (const uint16_t)fastCadencePeriodDivisor, triggerType,
                                         (const uint32_t)triggerDeltaDown, (const uint32_t)triggerDeltaUp, (const uint32_t)minInterval, (const uint32_t)fastCadenceLow, (const uint32_t)fastCadenceHigh);
    pthread_mutex_unlock(&cs);
    (*env)->ReleaseStringUTFChars(env, deviceName_, deviceName);
    return ret;
}

JNIEXPORT jshortArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSensorSettingsGetPropIds(JNIEnv *env,
                                                                                      jclass type,
                                                                                      jstring componentName_,
                                                                                      jint propertyId) {
    uint16_t *ret_setting_prop_ids;
    jint prop_id_array_size = 0;
    Log("SettingsGetPropIds\n");
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);

    pthread_mutex_lock(&cs);
    ret_setting_prop_ids = mesh_client_sensor_setting_property_ids_get(componentName, propertyId);
    pthread_mutex_unlock(&cs);

    if (ret_setting_prop_ids == NULL)
        return NULL;

    uint16_t *setting_property_id_ptr = ret_setting_prop_ids;
    while(setting_property_id_ptr[prop_id_array_size])
    {
        prop_id_array_size++;
    }

    jshortArray  data = (*env)->NewShortArray(env ,prop_id_array_size);

    (*env)->SetShortArrayRegion(env,data, 0 ,prop_id_array_size, ret_setting_prop_ids);
    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    free(ret_setting_prop_ids);

    return data;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSensorGet(JNIEnv *env, jclass type,
                                                                       jstring componentName_,
                                                                       jint propertyId)
{
    int ret;

    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);

    pthread_mutex_lock(&cs);
    ret = mesh_client_sensor_get(componentName, propertyId);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    return ret;
}

JNIEXPORT jintArray JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientSensorPropertyListGet(JNIEnv *env,
                                                                                   jclass type,
                                                                                   jstring componentName_)
{
    int *ret_prop_ids;
    jint prop_id_array_size = 0;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    Log("SensorPropertyListGet ");
    pthread_mutex_lock(&cs);
    ret_prop_ids = mesh_client_sensor_property_list_get(componentName);
    pthread_mutex_unlock(&cs);

    if (ret_prop_ids == NULL)
        return NULL;

    int *property_id_ptr = ret_prop_ids;
    while (property_id_ptr[prop_id_array_size])
    {
        Log("SensorPropertyListGet %x",property_id_ptr[prop_id_array_size]);
        prop_id_array_size++;
    }
    Log("SensorPropertyListGet size :%d\n",prop_id_array_size);
    jintArray  data = (*env)->NewIntArray(env ,prop_id_array_size);
    (*env)->SetIntArrayRegion(env, data, 0 ,prop_id_array_size, ret_prop_ids);
    free(ret_prop_ids);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    return data;
}

JNIEXPORT jstring JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetPublicationTarget(JNIEnv *env,
                                                                                  jclass type,
                                                                                  jstring componentName_,
                                                                                  jbyte isClient,
                                                                                  jstring method_)
{
    char* returnValue;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    const char *method = (*env)->GetStringUTFChars(env, method_, 0);

    pthread_mutex_lock(&cs);
    returnValue = mesh_client_get_publication_target(componentName, isClient, method);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseStringUTFChars(env, method_, method);

    return (*env)->NewStringUTF(env, returnValue);
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientGetPublicationPeriod(JNIEnv *env,
                                                                                  jclass type,
                                                                                  jstring componentName_,
                                                                                  jbyte isClient,
                                                                                  jstring method_)
{
    int res;
    const char *componentName = (*env)->GetStringUTFChars(env, componentName_, 0);
    const char *method = (*env)->GetStringUTFChars(env, method_, 0);

    pthread_mutex_lock(&cs);
    res = mesh_client_get_publication_period(componentName, isClient, method);
    pthread_mutex_unlock(&cs);

    (*env)->ReleaseStringUTFChars(env, componentName_, componentName);
    (*env)->ReleaseStringUTFChars(env, method_, method);

    return res;
}

JNIEXPORT jint JNICALL
Java_com_cypress_le_mesh_meshcore_MeshNativeHelper_meshClientListenForAppGroupBroadcasts(
        JNIEnv *env, jclass type, jstring controlMethod_, jstring groupName_, jboolean startListen)
{
    int res = 0;
    const char *controlMethod = (*env)->GetStringUTFChars(env, controlMethod_, 0);
    const char *groupName = (*env)->GetStringUTFChars(env, groupName_, 0);


    res = mesh_client_listen_for_app_group_broadcasts(strcmp("",controlMethod) == 0 ? NULL:controlMethod,
                                                      strcmp("",groupName) == 0 ? NULL:groupName,
                                                      startListen);
    (*env)->ReleaseStringUTFChars(env, controlMethod_, controlMethod);
    (*env)->ReleaseStringUTFChars(env, groupName_, groupName);
    return res;
}