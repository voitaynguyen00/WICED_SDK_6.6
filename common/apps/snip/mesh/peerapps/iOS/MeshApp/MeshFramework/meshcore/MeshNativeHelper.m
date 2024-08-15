/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * This file implements the MeshNativeHelper class which wraps all mesh libraries and fucntions.
 */

#import "stdio.h"
#import "stdlib.h"
#import "time.h"
#import "MeshNativeHelper.h"
#import "IMeshNativeCallback.h"
#import "wiced_bt_mesh_model_defs.h"
#import "wiced_bt_mesh_models.h"
#import "wiced_bt_mesh_event.h"
#import "wiced_bt_mesh_core.h"
#import "wiced_bt_mesh_provision.h"
#import "wiced_mesh_client.h"
#import "mesh_main.h"
#import "wiced_timer.h"
#import "wiced_bt_ble.h"
#import <CommonCrypto/CommonDigest.h>

extern void mesh_application_init(void);
extern void mesh_client_advert_report(uint8_t *bd_addr, uint8_t addr_type, int8_t rssi, uint8_t *adv_data);
extern wiced_bool_t initTimer(void);


@implementation MeshNativeHelper
{
    // define instance variables.
}


// define class variables.
static id nativeCallbackDelegate;   // Object instance that obeys IMeshNativeHelper protocol
static MeshNativeHelper *_instance;
static char provisioner_uuid[40];  // Used to store provisioner UUID string.
static dispatch_once_t onceToken;
static dispatch_once_t zoneOnceToken;
static char dfuFwImageFileName[256];            // DFU file name.
static char dfuFwMetadataFileName[256];    // DFU metadata file name.

/*
 * Implementation of APIs that required by wiced mesh core stack library for event and data callback.
 */

void meshClientUnprovisionedDeviceFoundCb(uint8_t *uuid, uint16_t oob, uint8_t *name, uint8_t name_len)
{
    NSString *deviceName = nil;
    if (uuid == NULL) {
        NSLog(@"[MeshNativeHelper, meshClientFoundUnprovisionedDeviceCb] error: invalid parameters, uuid=0x%p, name_len=%d", uuid, name_len);
        return;
    }
    if (name != NULL && name_len > 0) {
        deviceName = [[NSString alloc] initWithBytes:name length:name_len encoding:NSUTF8StringEncoding];
    }
    NSLog(@"[MeshNativeHelper meshClientFoundUnprovisionedDeviceCb] found device name: %@, oob: 0x%04x, uuid: ", deviceName, oob); dumpHexBytes(uuid, MAX_UUID_SIZE);
    [nativeCallbackDelegate onDeviceFound:[[NSUUID alloc] initWithUUIDBytes:uuid]
                                      oob:oob
                                  uriHash:0
                                     name:deviceName];
}

void meshClientProvisionCompleted(uint8_t status, uint8_t *p_uuid)
{
    if (p_uuid == NULL) {
        NSLog(@"[MeshNativeHelper meshClientProvisionCompleted] error: invalid parameters, uuid=0x%p", p_uuid);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientProvisionCompleted] status: %u", status);
    [nativeCallbackDelegate meshClientProvisionCompletedCb:status uuid:[[NSUUID alloc] initWithUUIDBytes:p_uuid]];
}

// called when mesh network connection status changed.
void linkStatus(uint8_t is_connected, uint32_t connId, uint16_t addr, uint8_t is_over_gatt)
{
    NSLog(@"[MeshNativeHelper linkStatus] is_connected: %u, connId: 0x%08x, addr: 0x%04x, is_over_gatt: %u", is_connected, connId, addr, is_over_gatt);
    [nativeCallbackDelegate onLinkStatus:is_connected connId:connId addr:addr isOverGatt:is_over_gatt];
}

// callback for meshClientConnect API.
void meshClientNodeConnectionState(uint8_t status, char *p_name)
{
    if (p_name == NULL || *p_name == '\0') {
        NSLog(@"[MeshNativeHelper meshClientNodeConnectionState] error: invalid parameters, p_name=0x%p", p_name);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientNodeConnectionState] status: 0x%02x, device_name: %s", status, p_name);
    [nativeCallbackDelegate meshClientNodeConnectStateCb:status componentName:[NSString stringWithUTF8String:(const char *)p_name]];
}

void resetStatus(uint8_t status, char *device_name)
{
    if (device_name == NULL || *device_name == '\0') {
        NSLog(@"[MeshNativeHelper resetStatus] error: invalid parameters, device_name=0x%p", device_name);
        return;
    }
    NSLog(@"[MeshNativeHelper resetStatus] status: 0x%02x, device_name: %s", status, device_name);
    [nativeCallbackDelegate onResetStatus:status devName:[NSString stringWithUTF8String:(const char *)device_name]];
}

void meshClientOnOffState(const char *device_name, uint8_t target, uint8_t present, uint32_t remaining_time)
{
    if (device_name == NULL || *device_name == '\0') {
        NSLog(@"[MeshNativeHelper meshClientOnOffState] error: invalid parameters, device_name=0x%p", device_name);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientOnOffState] device_name: %s, target: %u, present: %u, remaining_time: %u", device_name, target, present, remaining_time);
    [nativeCallbackDelegate meshClientOnOffStateCb:[NSString stringWithUTF8String:(const char *)device_name] target:target present:present remainingTime:remaining_time];
}

void meshClientLevelState(const char *device_name, int16_t target, int16_t present, uint32_t remaining_time)
{
    if (device_name == NULL || *device_name == '\0') {
        NSLog(@"[MeshNativeHelper meshClientLevelState] error: invalid parameters, device_name=0x%p", device_name);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientLevelState] device_name: %s, target: %u, present: %u, remaining_time: %u", device_name, target, present, remaining_time);
    [nativeCallbackDelegate meshClientLevelStateCb:[NSString stringWithUTF8String:(const char *)device_name]
                                            target:target
                                           present:present
                                    remainingTime:remaining_time];
}

void meshClientLightnessState(const char *device_name, uint16_t target, uint16_t present, uint32_t remaining_time)
{
    if (device_name == NULL || *device_name == '\0') {
        NSLog(@"[MeshNativeHelper meshClientLightnessState] error: invalid parameters, device_name=0x%p", device_name);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientLightnessState] device_name: %s, target: %u, present: %u, remaining_time: %u", device_name, target, present, remaining_time);
    [nativeCallbackDelegate meshClientLightnessStateCb:[NSString stringWithUTF8String:(const char *)device_name]
                                                target:target
                                               present:present
                                         remainingTime:remaining_time];
}

void meshClientHslState(const char *device_name, uint16_t lightness, uint16_t hue, uint16_t saturation, uint32_t remaining_time)
{
    if (device_name == NULL || *device_name == '\0') {
        NSLog(@"[MeshNativeHelper meshClientHslState] error: invalid parameters, device_name=0x%p", device_name);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientHslState] device_name: %s, lightness: %u, hue: %u, saturation: %u, remaining_time: %u",
          device_name, lightness, hue, saturation, remaining_time);
    [nativeCallbackDelegate meshClientHslStateCb:[NSString stringWithUTF8String:(const char *)device_name]
                                       lightness:lightness hue:hue saturation:saturation];
}

void meshClientCtlState(const char *device_name, uint16_t present_lightness, uint16_t present_temperature, uint16_t target_lightness, uint16_t target_temperature, uint32_t remaining_time)
{
    if (device_name == NULL || *device_name == '\0') {
        NSLog(@"[MeshNativeHelper meshClientCtlState] error: invalid parameters, device_name=0x%p", device_name);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientCtlState] device_name: %s, present_lightness: %u, present_temperature: %u, target_lightness: %u, target_temperature: %u, remaining_time: %u", device_name, present_lightness, present_temperature, target_lightness, target_temperature, remaining_time);
    [nativeCallbackDelegate meshClientCtlStateCb:[NSString stringWithUTF8String:(const char *)device_name]
                                presentLightness:present_lightness
                              presentTemperature:present_temperature
                                 targetLightness:target_lightness
                               targetTemperature:target_temperature
                                   remainingTime:remaining_time];
}

void meshClientDbChangedState(char *mesh_name)
{
    if (mesh_name == NULL || *mesh_name == '\0') {
        NSLog(@"[MeshNativeHelper meshClientDbChangedState] error: invalid parameters, mesh_name=0x%p", mesh_name);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientDbChangedState] mesh_name: %s", mesh_name);
    [nativeCallbackDelegate onDatabaseChangedCb:[NSString stringWithUTF8String:mesh_name]];
}

void meshClientSensorStatusChangedCb(const char *device_name, int property_id, uint8_t length, uint8_t *value)
{
    if (device_name == NULL || *device_name == '\0') {
        NSLog(@"[MeshNativeHelper meshClientSensorStatusChangedCb] error: invalid device_name:%s or property_id:%d, length=%d", device_name, property_id, length);
        return;
    }
    NSLog(@"[MeshNativeHelper meshClientSensorStatusChangedCb] device_name:%s, property_id:%d, value lenght:%d", device_name, property_id, length);
    [nativeCallbackDelegate onMeshClientSensorStatusChanged:[NSString stringWithUTF8String:device_name]
                                                 propertyId:(uint32_t)property_id
                                                       data:[NSData dataWithBytes:value length:length]];
}

mesh_client_init_t mesh_client_init_callback = {
    .unprovisioned_device_callback = meshClientUnprovisionedDeviceFoundCb,
    .provision_status_callback = meshClientProvisionCompleted,
    .connect_status_callback = linkStatus,
    .node_connect_status_callback = meshClientNodeConnectionState,
    .database_changed_callback = meshClientDbChangedState,
    .on_off_changed_callback = meshClientOnOffState,
    .level_changed_callback = meshClientLevelState,
    .lightness_changed_callback = meshClientLightnessState,
    .hsl_changed_callback = meshClientHslState,
    .ctl_changed_callback = meshClientCtlState,
    .sensor_changed_callback = meshClientSensorStatusChangedCb,
};

// timer based iOS platform.
static uint32_t gMeshTimerId = 1;       // always > 0; 1 is the first and the app whole life second pediodic timer; other values can be reused.
static NSMutableDictionary *gMeshTimersDict = nil;

-(void) meshTimerInit
{
    if (gMeshTimersDict == nil) {
        EnterCriticalSection();
        if (gMeshTimersDict == nil) {
            gMeshTimersDict = [[NSMutableDictionary alloc] initWithCapacity:5];
        }
        LeaveCriticalSection();
    }
}

-(uint32_t) allocateMeshTimerId
{
    uint32_t timerId = 0;   // default set to invalid timerId
    EnterCriticalSection();
    /*
     * valid timerId must always > 0;
     * and 1 is the first and the one through the whole app life second pediodic timer;
     * other values can be reused, and almost have short life.
     * so, when the value of gMeshTimerId round back, the value of 1 should be bypass,
     * here, the first round back value is reset to 100 for furture compatible, if can be changed when required.
     */
    timerId = gMeshTimerId++;
    if (timerId == 0) { // the gMeshTimerid must be round back.
        gMeshTimerId = 100;
        timerId = gMeshTimerId;
    }
    LeaveCriticalSection();
    return timerId;
}

-(void) timerFiredMethod:(NSTimer *)timer
{
    NSString * timerKey = timer.userInfo;
    uint32_t timerId = 0;
    if (timerKey != nil) {
        timerId = (uint32_t)timerKey.longLongValue;
    }

    //NSLog(@"[MeshNativeHelper timerFiredMethod] timerId:%u", timerId);
    MeshTimerFunc((long)timerId);
}

/*
 * @param timeout   Timer trigger interval, uint: milliseconds.
 * @param type      The timer type.
 *                  When the timer type is WICED_SECONDS_TIMER or WICED_MILLI_SECONDS_TIMER,
 *                      the timer will be invalidated after it fires.
 *                  When the timer type is WICED_SECONDS_PERIODIC_TIMER or WICED_MILLI_SECONDS_PERIODIC_TIMER,
 *                      the timer will repeatedly reschedule itself until stopped.
 * @return          A non-zero timerId will be returned when started on success. Otherwize, 0 will be return on failure.
 */
-(uint32_t) meshStartTimer:(uint32_t)timeout type:(uint16_t)type
{
    [MeshNativeHelper.getSharedInstance meshTimerInit];
    uint32_t timerId = [MeshNativeHelper.getSharedInstance allocateMeshTimerId];
    Boolean repeats = (type == WICED_SECONDS_PERIODIC_TIMER || type == WICED_MILLI_SECONDS_PERIODIC_TIMER) ? true : false;
    NSTimeInterval interval = (NSTimeInterval)timeout;
    interval /= (NSTimeInterval)1000;
    NSString * timerKey = [NSString stringWithFormat:@"%u", timerId];
    NSTimer *timer = [NSTimer scheduledTimerWithTimeInterval:interval
                                                      target:MeshNativeHelper.getSharedInstance
                                                    selector:@selector(timerFiredMethod:)
                                                    userInfo:timerKey
                                                     repeats:repeats];
    if (timer == nil) {
        NSLog(@"[MeshNativeHelper meshStartTimer] error: failed to create and init the timer");
        return 0;
    }
    
    NSArray *timerInfo = [[NSArray alloc] initWithObjects:[NSNumber numberWithUnsignedInt:timerId], [NSNumber numberWithUnsignedShort:type], timer, nil];
    [gMeshTimersDict setObject:timerInfo forKey:timerKey];
    //NSLog(@"[MeshNativeHelper meshStartTimer] timerId:%u started, type=%u, interval=%f", timerId, type, interval);
    return timerId;
}
uint32_t start_timer(uint32_t timeout, uint16_t type) {
    return [MeshNativeHelper.getSharedInstance meshStartTimer:timeout type:type];
}

-(void) meshStopTimer:(uint32_t)timerId
{
    [MeshNativeHelper.getSharedInstance meshTimerInit];
    NSString *timerKey = [NSString stringWithFormat:@"%u", timerId];
    NSArray *timerInfo = [gMeshTimersDict valueForKey:timerKey];
    if (timerInfo != nil && [timerInfo count] == 3) {
        NSTimer *timer = timerInfo[2];
        if (timer != nil) {
            [timer invalidate];
        }
    }
    [gMeshTimersDict removeObjectForKey:timerKey];
    //NSLog(@"[MeshNativeHelper meshStopTimer] timerId:%u stopped", timerId);
}
void stop_timer(uint32_t timerId)
{
    [MeshNativeHelper.getSharedInstance meshStopTimer:timerId];
}

/*
 * @param timeout   Timer trigger interval, uint: milliseconds.
 * @param type      The timer type.
 *                  When the timer type is WICED_SECONDS_TIMER or WICED_MILLI_SECONDS_TIMER,
 *                      the timer will be invalidated after it fires.
 *                  When the timer type is WICED_SECONDS_PERIODIC_TIMER or WICED_MILLI_SECONDS_PERIODIC_TIMER,
 *                      the timer will repeatedly reschedule itself until stopped.
 * @return          The same non-zero timerId will be returned when restarted on success. Otherwize, 0 will be return on failure.
 */
-(uint32_t) meshRestartTimer:(uint32_t)timeout timerId:(uint32_t)timerId
{
    [MeshNativeHelper.getSharedInstance meshTimerInit];
    NSString *timerKey = [NSString stringWithFormat:@"%u", timerId];
    NSArray *timerInfo = [gMeshTimersDict valueForKey:timerKey];
    NSNumber *numType = timerInfo[1];
    NSTimer *timer = timerInfo[2];
    uint16_t type;
    
    if (timerInfo == nil || [timerInfo count] != 3 || timer == nil || numType == nil) {
        NSLog(@"[MeshNativeHelper meshRestartTimer] error: failed to fetch the timer with timerId=%u", timerId);
        return 0;
    }
    
    type = [numType unsignedShortValue];
    [timer invalidate];
    
    Boolean repeats = (type == WICED_SECONDS_PERIODIC_TIMER || type == WICED_MILLI_SECONDS_PERIODIC_TIMER) ? true : false;
    NSTimeInterval interval = (NSTimeInterval)timeout;
    interval /= (NSTimeInterval)1000;
    timer = [NSTimer scheduledTimerWithTimeInterval:interval
                                             target:MeshNativeHelper.getSharedInstance
                                           selector:@selector(timerFiredMethod:)
                                           userInfo:timerKey
                                            repeats:repeats];
    if (timer == nil) {
        NSLog(@"[MeshNativeHelper meshRestartTimer] error: failed to create and init the timer");
        return 0;
    }
    
    timerInfo = [[NSArray alloc] initWithObjects:[NSNumber numberWithUnsignedInt:timerId], [NSNumber numberWithUnsignedShort:type], timer, nil];
    [gMeshTimersDict setObject:timerInfo forKey:timerKey];
    //NSLog(@"%s timerId:%u restarted, type=%u, interval=%f", __FUNCTION__, timerId, type, interval);
    return timerId;
}
uint32_t restart_timer(uint32_t timeout, uint32_t timerId ) {
    return [MeshNativeHelper.getSharedInstance meshRestartTimer:timeout timerId:timerId];
}

void mesh_provision_gatt_send(uint16_t connId, uint8_t *packet, uint32_t packet_len)
{
    if (packet == NULL || packet_len == 0) {
        NSLog(@"[MeshNativeHelper mesh_provision_gatt_send] error: connId=%d packet=0x%p, packet_len=%u", connId, packet, packet_len);
        return;
    }
    NSData *data = [[NSData alloc] initWithBytes:packet length:packet_len];
    [nativeCallbackDelegate onProvGattPktReceivedCallback:connId data:data];
}

void proxy_gatt_send_cb(uint32_t connId, uint32_t ref_data, const uint8_t *packet, uint32_t packet_len)
{
    if (packet == NULL || packet_len == 0) {
        NSLog(@"[MeshNativeHelper proxy_gatt_send_cb] error: invalid parameters, packet=0x%p, packet_len=%u", packet, packet_len);
        return;
    }
    NSData *data = [[NSData alloc] initWithBytes:packet length:packet_len];
    [nativeCallbackDelegate onProxyGattPktReceivedCallback:connId data:data];
}

wiced_bool_t mesh_bt_gatt_le_disconnect(uint32_t connId)
{
    return [nativeCallbackDelegate meshClientDisconnect:(uint16_t)connId];
}

wiced_bool_t mesh_bt_gatt_le_connect(wiced_bt_device_address_t bd_addr, wiced_bt_ble_address_type_t bd_addr_type,
                                     wiced_bt_ble_conn_mode_t conn_mode, wiced_bool_t is_direct)
{
    if (bd_addr == NULL) {
        NSLog(@"[MeshNativeHelper mesh_bt_gatt_le_connect] invalid parameters, bd_addr=0x%p", bd_addr);
        return false;
    }
    NSData *bdAddr = [[NSData alloc] initWithBytes:bd_addr length:BD_ADDR_LEN];
    return [nativeCallbackDelegate meshClientConnect:bdAddr];
}

wiced_bool_t mesh_set_scan_type(uint8_t is_active)
{
    return [nativeCallbackDelegate meshClientSetScanTypeCb:is_active];
}

wiced_bool_t mesh_adv_scan_start(void)
{
    return [nativeCallbackDelegate meshClientAdvScanStartCb];
}

void mesh_adv_scan_stop(void)
{
    [nativeCallbackDelegate meshClientAdvScanStopCb];
}

/*
 * Only one instance of the MeshNativeHelper class can be created all the time.
 */
+(MeshNativeHelper *) getSharedInstance
{
    dispatch_once(&onceToken, ^{
        _instance = [[self alloc] init];
    });
    return _instance;
}

+(instancetype) allocWithZone:(struct _NSZone *)zone
{
    //static dispatch_once_t onceToken;
    dispatch_once(&zoneOnceToken, ^{
        _instance = [super allocWithZone:zone];
        [_instance instanceInit];
    });
    return _instance;
}

-(id)copyWithZone:(NSZone *)zone {
    return _instance;
}

/* Do all necessory initializations for the shared class instance. */
-(void) instanceInit
{
    [self meshTimerInit];
    [self meshBdAddrDictInit];
}

+(NSString *) getProvisionerUuidFileName
{
    return @"prov_uuid.bin";
}

+(int) setFileStorageAtPath:(NSString *)path
{
    return [MeshNativeHelper setFileStorageAtPath:path provisionerUuid:nil];
}

+(int) setFileStorageAtPath:(NSString *)path provisionerUuid: (NSUUID *)provisionerUuid
{
    Boolean bRet = true;
    NSFileManager *fileManager = [NSFileManager defaultManager];
    
    // check if the provisioner_uuid has been read from or has been created and written to the storage file.
    if (strlen(provisioner_uuid) == 32) {
        return 0;
    }
    
    if (![fileManager fileExistsAtPath:path]) {
        bRet = [fileManager createDirectoryAtPath:path withIntermediateDirectories:true attributes:nil error:nil];
        NSLog(@"[MeshNativeHelper setFileStorageAtPath] create direcotry \"%@\" %s", path, bRet ? "success" : "failed");
    }
    if (!bRet || ![fileManager isWritableFileAtPath:path]) {
        NSLog(@"[MeshNativeHelper setFileStorageAtPath] error: cannot wirte at path:\"%@\", bRet=%u", path, bRet);
        return -1;
    }
    
    // set this file directory to be current working directory
    const char *cwd = [path cStringUsingEncoding:NSASCIIStringEncoding];
    int cwdStatus = chdir(cwd);
    if (cwdStatus != 0) {
        NSLog(@"[MeshNativeHelper setFileStorageAtPath] error: unable to change current working directory to \"%s\" ", cwd);
        return -2;
    } else {
        NSLog(@"[MeshNativeHelper setFileStorageAtPath] Done, change current working directory to \"%@\"", path);
    }
    
    // create the prov_uuid.bin to stote the UUID string value or read the stored UUID string if existing.
    NSFileHandle *handle;
    NSData *data;
    NSString *filePath = [path stringByAppendingPathComponent: [MeshNativeHelper getProvisionerUuidFileName]];
    if ([fileManager fileExistsAtPath:filePath]) {
        handle = [NSFileHandle fileHandleForReadingAtPath:filePath];
        if (handle == nil) {
            NSLog(@"[MeshNativeHelper setFileStorageAtPath] error: unable to open file \"%@\" for reading", filePath);
            return -3;
        }
        
        // read back the stored provisoner uuid string from prov_uuid.bin file.
        data = [handle readDataOfLength:32];
        [data getBytes:provisioner_uuid length:(NSUInteger)32];
        provisioner_uuid[32] = '\0';  // always set the terminate character for the provisioner uuid string.
        NSLog(@"[MeshNativeHelper setFileStorageAtPath] read provisioner_uuid: %s from prov_uuid.bin", provisioner_uuid);
    } else {
        bRet = [fileManager createFileAtPath:filePath contents:nil attributes:nil];
        handle = [NSFileHandle fileHandleForWritingAtPath:filePath];
        if (!bRet || handle == nil) {
            if (handle) {
                [handle closeFile];
            }
            NSLog(@"[MeshNativeHelper setFileStorageAtPath] error: unable to create file \"%@\" for writing", filePath);
            return -4;
        }
        
        // Create a new UUID string for the provisoner and stored to the prov_uuid.bin file.
        // Based on UUID with RFC 4122 version 4, the format is XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX. It's 36 characters.
        // but the UUID format required in the provision_uuid should not including the '-' character,
        // so the UUID string stored in provision_uuid should be 32 characters, it must be conveted here.
        int j = 0;
        char *rfcuuid = NULL;
        if (provisionerUuid == nil) {
            rfcuuid = (char *)NSUUID.UUID.UUIDString.UTF8String;
        } else {
            rfcuuid = (char *)provisionerUuid.UUIDString.UTF8String;
        }
        for (int i = 0; i < strlen(rfcuuid); i++) {
            if (rfcuuid[i] == '-') {
                continue;
            }
            provisioner_uuid[j++] = rfcuuid[i];
        }
        provisioner_uuid[j] = '\0';
        data = [NSData dataWithBytes:provisioner_uuid length:strlen(provisioner_uuid)];
        [handle writeData:data];    // write 32 bytes.
        NSLog(@"[MeshNativeHelper setFileStorageAtPath] create provisioner_uuid: %s, and stored to prov_uuid.bin", provisioner_uuid);
    }
    [handle closeFile];
    return 0;
}

/*
 * MeshNativeHelper class functions.
 */

-(void) registerNativeCallback: (id)delegate
{
    NSLog(@"%s", __FUNCTION__);
    nativeCallbackDelegate = delegate;
}

+(int) meshClientNetworkExists:(NSString *) meshName
{
    NSLog(@"%s, meshName: %@", __FUNCTION__, meshName);
    return mesh_client_network_exists((char *)[meshName UTF8String]);
}

+(int) meshClientNetworkCreate:(NSString *)provisionerName meshName:(NSString *)meshName
{
    NSLog(@"%s, provisionerName: %@, meshName: %@", __FUNCTION__, provisionerName, meshName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_network_create(provisionerName.UTF8String, provisioner_uuid, (char *)meshName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

void mesh_client_network_opened(uint8_t status) {
    NSLog(@"%s, status: %u", __FUNCTION__, status);
    [nativeCallbackDelegate meshClientNetworkOpenCb:status];
}
+(int) meshClientNetworkOpen:(NSString *)provisionerName meshName:(NSString *)meshName
{
    NSLog(@"%s, provisionerName: %@, meshName: %@", __FUNCTION__, provisionerName, meshName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_network_open(provisionerName.UTF8String, provisioner_uuid, (char *)meshName.UTF8String, mesh_client_network_opened);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientNetworkDelete:(NSString*)provisionerName meshName:(NSString *)meshName
{
    NSLog(@"%s, provisionerName: %@, meshName: %@", __FUNCTION__, provisionerName, meshName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_network_delete(provisionerName.UTF8String, provisioner_uuid, (char *)meshName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(void) meshClientNetworkClose
{
    NSLog(@"%s", __FUNCTION__);
    EnterCriticalSection();
    mesh_client_network_close();
    LeaveCriticalSection();
}

+(NSString *) meshClientNetworkExport:(NSString *)meshName
{
    NSLog(@"%s, meshName=%@", __FUNCTION__, meshName);
    char *jsonString = NULL;
    EnterCriticalSection();
    jsonString = mesh_client_network_export((char *)meshName.UTF8String);
    LeaveCriticalSection();
    if (jsonString == NULL) {
        return nil;
    }
    return [[NSString alloc] initWithUTF8String:jsonString];
}

+(NSString *) meshClientNetworkImport:(NSString *)provisionerName jsonString:(NSString *)jsonString
{
    NSLog(@"%s, provisionerName: %@, jsonString: %@", __FUNCTION__, provisionerName, jsonString);
    char *networkName = NULL;
    EnterCriticalSection();
    networkName = mesh_client_network_import(provisionerName.UTF8String, provisioner_uuid, (char *)jsonString.UTF8String, mesh_client_network_opened);
    LeaveCriticalSection();
    
    if (networkName == NULL) {
        return nil;
    }
    return [[NSString alloc] initWithUTF8String:networkName];
}


+(int) meshClientGroupCreate:(NSString *)groupName parentGroupName:(NSString *)parentGroupName
{
    NSLog(@"%s, groupName: %@, parentGroupName: %@", __FUNCTION__, groupName, parentGroupName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_group_create((char *)groupName.UTF8String, (char *)parentGroupName.UTF8String);
    LeaveCriticalSection();
    return ret;
}
+(int) meshClientGroupDelete:(NSString *)groupName
{
    NSLog(@"%s, groupName: %@", __FUNCTION__, groupName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_group_delete((char *)groupName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

/*
 * This help function will convert the C strings from the input buffer to a NSArray<NSString *> array data,
 * and free C String if required.
 */
NSArray<NSString *> * meshCStringToOCStringArray(const char *cstrings, BOOL freeCString)
{
    NSMutableArray<NSString *> *stringArray = [[NSMutableArray<NSString *> alloc] init];
    char *p_str = (char *)cstrings;
    
    if (p_str == NULL || *p_str == '\0') {
        return NULL;
    }
    
    for (int i = 0; p_str != NULL && *p_str != '\0'; p_str += (strlen(p_str) + 1), i++) {
        stringArray[i] = [NSString stringWithUTF8String:p_str];
    }
    
    if (freeCString) {
        free((void *)cstrings);
    }
    return [NSArray<NSString *> arrayWithArray:stringArray];
}

+(NSArray<NSString *> *) meshClientGetAllNetworks
{
    NSLog(@"%s", __FUNCTION__);
    char *networks = mesh_client_get_all_networks();
    return meshCStringToOCStringArray(networks, TRUE);
}

+(NSArray<NSString *> *) meshClientGetAllGroups:(NSString *)inGroup
{
    NSLog(@"%s, inGroup: %@", __FUNCTION__, inGroup);
    char *groups = NULL;
    EnterCriticalSection();
    groups = mesh_client_get_all_groups((char *)inGroup.UTF8String);
    LeaveCriticalSection();
    return meshCStringToOCStringArray(groups, TRUE);
}

+(NSArray<NSString *> *) meshClientGetAllProvisioners
{
    NSLog(@"%s", __FUNCTION__);
    char *provisioners = NULL;
    EnterCriticalSection();
    provisioners = mesh_client_get_all_provisioners();
    LeaveCriticalSection();
    return meshCStringToOCStringArray(provisioners, TRUE);
}

+(NSArray<NSString *> *) meshClientGetDeviceComponents:(NSUUID *)uuid
{
    char *components = NULL;
    uint8_t p_uuid[16];
    [uuid getUUIDBytes:p_uuid];
    NSLog(@"%s device uuid: %s", __FUNCTION__, uuid.UUIDString.UTF8String);
    EnterCriticalSection();
    components = mesh_client_get_device_components(p_uuid);
    LeaveCriticalSection();
    return meshCStringToOCStringArray(components, TRUE);
}

+(NSArray<NSString *> *) meshClientGetGroupComponents:(NSString *)groupName
{
    NSLog(@"%s, groupName: %@", __FUNCTION__, groupName);
    char *componetNames = NULL;
    EnterCriticalSection();
    componetNames = mesh_client_get_group_components((char *)groupName.UTF8String);
    LeaveCriticalSection();
    return meshCStringToOCStringArray(componetNames, TRUE);
}

+(NSArray<NSString *> *) meshClientGetTargetMethods:(NSString *)componentName
{
    NSLog(@"%s, componentName: %@", __FUNCTION__, componentName);
    char *targetMethods = NULL;
    EnterCriticalSection();
    targetMethods = mesh_client_get_target_methods(componentName.UTF8String);
    LeaveCriticalSection();
    return meshCStringToOCStringArray(targetMethods, TRUE);
}

+(NSArray<NSString *> *) meshClientGetControlMethods:(NSString *)componentName
{
    NSLog(@"%s, componentName: %@", __FUNCTION__, componentName);
    char *controlMethods = NULL;
    EnterCriticalSection();
    controlMethods = mesh_client_get_control_methods(componentName.UTF8String);
    LeaveCriticalSection();
    return meshCStringToOCStringArray(controlMethods, TRUE);
}

+(uint8_t) meshClientGetComponentType:(NSString *)componentName
{
    NSLog(@"%s, componentName: %@", __FUNCTION__, componentName);
    uint8_t type;
    EnterCriticalSection();
    type = mesh_client_get_component_type((char *)componentName.UTF8String);
    LeaveCriticalSection();
    return type;
}

void meshClientComponentInfoStatusCallback(uint8_t status, char *component_name, char *component_info)
{
    NSLog(@"%s, status:0x%x", __FUNCTION__, status);
    NSString *componentName = nil;
    NSString *componentInfo = nil;
    if (component_name == NULL) {
        NSLog(@"%s, error, invalid parameters, component_name: is NULL", __FUNCTION__);
        return;
    }
    componentName = [NSString stringWithUTF8String:(const char *)component_name];
    if (component_info != NULL) {
        componentInfo = [NSString stringWithUTF8String:(const char *)component_info];
        NSLog(@"%s, componentInfo:%@", __FUNCTION__, componentInfo);
    } else {
        NSLog(@"%s, component_info string is NULL", __FUNCTION__);
    }
    [nativeCallbackDelegate meshClientComponentInfoStatusCb:status componentName:componentName componentInfo:componentInfo];
}

+(uint8_t) meshClientGetComponentInfo:(NSString *)componentName
{
    NSLog(@"%s, componentName:%@", __FUNCTION__, componentName);
    uint8_t ret;
    EnterCriticalSection();
    ret = mesh_client_get_component_info((char *)componentName.UTF8String, meshClientComponentInfoStatusCallback);
    LeaveCriticalSection();
    return ret;
}

/*
 * When the controlMethod is nil or empty, the library will register to receive messages sent to all type of messages.
 * When the groupName is nil or empty, the library will register to receive messages sent to all the groups.
 */
+(int) meshClientListenForAppGroupBroadcasts:(NSString *)controlMethod groupName:(NSString *)groupName startListen:(BOOL)startListen
{
    NSLog(@"%s, controlMethod:%@, groupName:%@, startListen:%d", __FUNCTION__, controlMethod, groupName, (int)startListen);
    char *listonControlMethod = (controlMethod != nil && controlMethod.length > 0) ? (char *)controlMethod.UTF8String : NULL;
    char *listonGroupName = (groupName != nil && groupName.length > 0) ? (char *)groupName.UTF8String : NULL;
    int ret;
    
    EnterCriticalSection();
    ret = mesh_client_listen_for_app_group_broadcasts(listonControlMethod, listonGroupName, (wiced_bool_t)startListen);
    LeaveCriticalSection();
    return ret;
}

+(NSString *) meshClientGetPublicationTarget:(NSString *)componentName isClient:(BOOL)isClient method:(NSString *)method
{
    NSLog(@"%s, componentName:%@, isClient:%d, method:%@", __FUNCTION__, componentName, (int)isClient, method);
    const char *targetName;
    EnterCriticalSection();
    targetName = mesh_client_get_publication_target(componentName.UTF8String, (uint8_t)isClient, method.UTF8String);
    LeaveCriticalSection();
    return (targetName == NULL) ? nil : [NSString stringWithUTF8String:targetName];
}

/*
 * Return 0 on failed to to get publication period or encountered any error.
 * Otherwise, return the publish period value on success.
 */
+(int) meshClientGetPublicationPeriod:(NSString *)componentName isClient:(BOOL)isClient method:(NSString *)method
{
    NSLog(@"%s, componentName:%@, isClient:%d, method:%@", __FUNCTION__, componentName, (int)isClient, method);
    int publishPeriod;
    EnterCriticalSection();
    publishPeriod = mesh_client_get_publication_period((char *)componentName.UTF8String, (uint8_t)isClient, method.UTF8String);
    LeaveCriticalSection();
    return publishPeriod;
}



+(int) meshClientRename:(NSString *)oldName newName:(NSString *)newName
{
    NSLog(@"%s oldName:%@, newName:%@", __FUNCTION__, oldName, newName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_rename((char *)oldName.UTF8String, (char *)newName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientMoveComponentToGroup:(NSString *)componentName from:(NSString *)fromGroupName to:(NSString *)toGroupName
{
    NSLog(@"[MeshNativehelper meshClientMoveComponentToGroup] componentName:%@, fromGroupName:%@, toGroupName:%@",
          componentName, fromGroupName, toGroupName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_move_component_to_group(componentName.UTF8String, fromGroupName.UTF8String, toGroupName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientConfigurePublication:(NSString *)componentName isClient:(uint8_t)isClient method:(NSString *)method targetName:(NSString *)targetName publishPeriod:(int)publishPeriod
{
    NSLog(@"[MeshNativehelper meshClientConfigurePublication] componentName:%@, isClient:%d, method:%@, targetName:%@ publishPeriod:%d",
          componentName, isClient, method, targetName, publishPeriod);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_configure_publication(componentName.UTF8String, isClient, method.UTF8String, targetName.UTF8String, publishPeriod);
    LeaveCriticalSection();
    return ret;
}

+(uint8_t) meshClientProvision:(NSString *)deviceName groupName:(NSString *)groupName uuid:(NSUUID *)uuid identifyDuration:(uint8_t)identifyDuration
{
    NSLog(@"[MeshNativehelper meshClientProvision] deviceName:%@, groupName:%@, identifyDuration:%d", deviceName, groupName, identifyDuration);
    uint8_t ret;
    uint8_t p_uuid[16];
    [uuid getUUIDBytes:p_uuid];
    EnterCriticalSection();
    ret = mesh_client_provision(deviceName.UTF8String, groupName.UTF8String, p_uuid, identifyDuration);
    LeaveCriticalSection();
    return ret;
}

+(uint8_t) meshClientConnectNetwork:(uint8_t)useGattProxy scanDuration:(uint8_t)scanDuration
{
    NSLog(@"%s useGattProxy:%d, scanDuration:%d", __FUNCTION__, useGattProxy, scanDuration);
    uint8_t ret;
    EnterCriticalSection();
    ret = mesh_client_connect_network(useGattProxy, scanDuration);
    LeaveCriticalSection();
    return ret;
}

+(uint8_t) meshClientDisconnectNetwork:(uint8_t)useGattProxy
{
    NSLog(@"%s useGattProxy:%d", __FUNCTION__, useGattProxy);
    uint8_t ret;
    EnterCriticalSection();
    ret = mesh_client_disconnect_network();
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientOnOffGet:(NSString *)deviceName
{
    NSLog(@"%s deviceName:%@", __FUNCTION__, deviceName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_on_off_get(deviceName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientOnOffSet:(NSString *)deviceName onoff:(uint8_t)onoff reliable:(Boolean)reliable transitionTime:(uint32_t)transitionTime delay:(uint16_t)delay
{
    NSLog(@"[MeshNativehelper meshClientOnOffSet] deviceName:%@, onoff:%d, reliable:%d, transitionTime:%d, delay:%d",
          deviceName, onoff, reliable, transitionTime, delay);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_on_off_set(deviceName.UTF8String, onoff, reliable, transitionTime, delay);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientLevelGet:(NSString *)deviceName
{
    NSLog(@"%s deviceName:%@", __FUNCTION__, deviceName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_level_get(deviceName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientLevelSet:(NSString *)deviceName level:(int16_t)level reliable:(Boolean)reliable transitionTime:(uint32_t)transitionTime delay:(uint16_t)delay
{
    NSLog(@"[MeshNativehelper meshClientLevelSet] deviceName:%@, level:%d, reliable:%d, transitionTime:%d, delay:%d",
          deviceName, level, reliable, transitionTime, delay);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_level_set(deviceName.UTF8String, (int16_t)level, reliable, transitionTime, delay);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientHslGet:(NSString *)deviceName
{
    NSLog(@"%s deviceName:%@", __FUNCTION__, deviceName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_hsl_get(deviceName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientHslSet:(NSString *)deviceName lightness:(uint16_t)lightness hue:(uint16_t)hue saturation:(uint16_t)saturation reliable:(Boolean)reliable transitionTime:(uint32_t)transitionTime delay:(uint16_t)delay
{
    NSLog(@"[MeshNativehelper meshClientHslSet] deviceName:%@, lightness:%d, hue:%d, saturation:%d, reliable:%d, transitionTime:%d, delay:%d",
          deviceName, lightness, hue, saturation, reliable, transitionTime, delay);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_hsl_set(deviceName.UTF8String, lightness, hue, saturation, reliable, transitionTime, delay);
    LeaveCriticalSection();
    return ret;
}

+(void) meshClientInit
{
    NSLog(@"%s", __FUNCTION__);
    @synchronized (MeshNativeHelper.getSharedInstance) {
        srand((unsigned int)time(NULL));    // Set the seed value to avoid same pseudo-random intergers are generated.
        initTimer();                        // The the global shared recurive mutex lock 'cs' befer using it.
        mesh_client_init(&mesh_client_init_callback);
    }
}

+(int) meshClientSetDeviceConfig:(NSString *)deviceName
                     isGattProxy:(int)isGattProxy
                        isFriend:(int)isFriend
                         isRelay:(int)isRelay
                          beacon:(int)beacon
                  relayXmitCount:(int)relayXmitCount
               relayXmitInterval:(int)relayXmitInterval
                      defaultTtl:(int)defaultTtl
                    netXmitCount:(int)netXmitCount
                 netXmitInterval:(int)netXmitInterval
{
    NSLog(@"[MeshNativehelper meshClientSetDeviceConfig] deviceName:%@, isGattProxy:%d, isFriend:%d, isRelay:%d, beacon:%d, relayXmitCount:%d, relayXmitInterval:%d, defaultTtl:%d, netXmitCount:%d, netXmitInterval%d",
          deviceName, isGattProxy, isFriend, isRelay, beacon, relayXmitCount, relayXmitInterval, defaultTtl, netXmitCount, netXmitInterval);
    int ret;
    char *device_name = NULL;
    if (deviceName != NULL && deviceName.length > 0) {
        device_name = (char *)deviceName.UTF8String;
    }
    EnterCriticalSection();
    ret = mesh_client_set_device_config(device_name,
                                        isGattProxy,
                                        isFriend,
                                        isRelay,
                                        beacon,
                                        relayXmitCount,
                                        relayXmitInterval,
                                        defaultTtl,
                                        netXmitCount,
                                        netXmitInterval);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientSetPublicationConfig:(int)publishCredentialFlag
               publishRetransmitCount:(int)publishRetransmitCount
            publishRetransmitInterval:(int)publishRetransmitInterval
                           publishTtl:(int)publishTtl
{
    NSLog(@"[MeshNativehelper meshClientSetPublicationConfig] publishCredentialFlag:%d, publishRetransmitCount:%d, publishRetransmitInterval:%d, publishTtl:%d",
          publishCredentialFlag, publishRetransmitCount, publishRetransmitInterval, publishTtl);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_set_publication_config(publishCredentialFlag,
                                             publishRetransmitCount,
                                             publishRetransmitInterval,
                                             publishTtl);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientResetDevice:(NSString *)componentName
{
    NSLog(@"%s componentName:%@", __FUNCTION__, componentName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_reset_device((char *)componentName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientVendorDataSet:(NSString *)deviceName data:(NSData *)data
{
    NSLog(@"%s deviceName:%@", __FUNCTION__, deviceName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_vendor_data_set(deviceName.UTF8String, (uint8_t *)data.bytes, data.length);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientIdentify:(NSString *)name duration:(uint8_t)duration
{
    NSLog(@"%s name:%@, duration:%d", __FUNCTION__, name, duration);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_identify(name.UTF8String, duration);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientLightnessGet:(NSString *)deviceName
{
    NSLog(@"%s deviceName:%@", __FUNCTION__, deviceName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_lightness_get(deviceName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientLightnessSet:(NSString *)deviceName lightness:(uint16_t)lightness reliable:(Boolean)reliable transitionTime:(uint32_t)transitionTime delay:(uint16_t)delay
{
    NSLog(@"[MeshNativehelper meshClientLightnessSet] deviceName:%@, lightness:%d, reliable:%d, transitionTime:%d, delay:%d",
          deviceName, lightness, reliable, transitionTime, delay);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_lightness_set(deviceName.UTF8String, lightness, reliable, transitionTime, delay);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientCtlGet:(NSString *)deviceName
{
    NSLog(@"%s deviceName:%@", __FUNCTION__, deviceName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_ctl_get(deviceName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientCtlSet:(NSString *)deviceName lightness:(uint16_t)lightness temperature:(uint16_t)temperature deltaUv:(uint16_t)deltaUv
               reliable:(Boolean)reliable transitionTime:(uint32_t)transitionTime delay:(uint16_t)delay
{
    NSLog(@"[MeshNativeHelper meshClientCtlSet] deviceName: %@, lightness: %d, temperature: %d, deltaUv: %d, reliable: %d, transitionTime: %d, delay: %d",
          deviceName, lightness, temperature, deltaUv, reliable, transitionTime, delay);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_ctl_set(deviceName.UTF8String, lightness, temperature, deltaUv, reliable, transitionTime, delay);
    LeaveCriticalSection();
    return ret;
}

//MESH CLIENT GATT APIS
+(void) meshClientScanUnprovisioned:(int)start
{
    NSLog(@"%s start:%d", __FUNCTION__, start);
    EnterCriticalSection();
    mesh_client_scan_unprovisioned(start);
    LeaveCriticalSection();
}

+(Boolean) meshClientIsConnectingProvisioning
{
    NSLog(@"%s", __FUNCTION__);
    Boolean is_connecting_provisioning;
    EnterCriticalSection();
    is_connecting_provisioning = mesh_client_is_connecting_provisioning();
    LeaveCriticalSection();
    return is_connecting_provisioning;
}

+(void) meshClientConnectionStateChanged:(uint16_t)connId mtu:(uint16_t)mtu
{
    NSLog(@"[MeshNativeHelper meshClientConnectionStateChanged] connId:0x%04x, mtu:%d", connId, mtu);
    mesh_client_connection_state_changed(connId, mtu);
}

+(void) meshClientAdvertReport:(NSData *)bdaddr addrType:(uint8_t)addrType rssi:(int8_t)rssi advData:(NSData *) advData
{
    NSLog(@"[MeshNativeHelper meshClientAdvertReport] advData.length:%lu, rssi:%d", (unsigned long)advData.length, rssi);
    if (bdaddr.length == 6 && advData.length > 0) {
        mesh_client_advert_report((uint8_t *)bdaddr.bytes, addrType, rssi, (uint8_t *)advData.bytes);
    } else {
        NSLog(@"[MeshNativeHelper meshClientAdvertReport] error: invalid bdaddr or advdata, bdaddr.length=%lu, advData.length=%lu",
              (unsigned long)bdaddr.length, (unsigned long)advData.length);
    }
}

+(uint8_t) meshConnectComponent:(NSString *)componentName useProxy:(uint8_t)useProxy scanDuration:(uint8_t)scanDuration
{
    NSLog(@"[MeshNativehelper meshConnectComponent] componentName: %@, useProxy: %d, scanDuration: %d", componentName, useProxy, scanDuration);
    uint8_t ret;
    EnterCriticalSection();
    ret = mesh_client_connect_component((char *)componentName.UTF8String, useProxy, scanDuration);
    if (ret != MESH_CLIENT_SUCCESS) {
        meshClientNodeConnectionState(MESH_CLIENT_NODE_WARNING_UNREACHABLE, (char *)componentName.UTF8String);
    }
    LeaveCriticalSection();
    return ret;
}

+(void) sendRxProxyPktToCore:(NSData *)data
{
    NSLog(@"%s data.length: %lu", __FUNCTION__, (unsigned long)data.length);
    EnterCriticalSection();
    mesh_client_proxy_data((uint8_t *)data.bytes, data.length);
    LeaveCriticalSection();
}

+(void) sendRxProvisPktToCore:(NSData *)data
{
    NSLog(@"%s data.length: %lu", __FUNCTION__, (unsigned long)data.length);
    EnterCriticalSection();
    mesh_client_provisioning_data(WICED_TRUE, (uint8_t *)data.bytes, data.length);
    LeaveCriticalSection();
}

+(Boolean) isMeshProvisioningServiceAdvertisementData:(NSDictionary<NSString *,id> *)advertisementData
{
    CBUUID *provUuid = [CBUUID UUIDWithString:@"1827"];
    
    NSNumber *conntable = advertisementData[CBAdvertisementDataIsConnectable];
    if (conntable == nil || [conntable isEqual: [NSNumber numberWithUnsignedInteger:0]]) {
        return false;
    }
    
    NSArray *srvUuids = advertisementData[CBAdvertisementDataServiceUUIDsKey];
    if (srvUuids == nil || srvUuids.count == 0) {
        return false;
    }
    
    for (CBUUID *uuid in srvUuids) {
        if ([uuid isEqual: provUuid]) {
            NSDictionary *srvData = advertisementData[CBAdvertisementDataServiceDataKey];
            if (srvData != nil && srvData.count > 0) {
                NSArray *allKeys = [srvData allKeys];
                for (CBUUID *key in allKeys) {
                    if (![key isEqual:provUuid]) {
                        continue;
                    }
                    NSData *data = srvData[key];
                    if (data != nil && data.length > 0) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

+(Boolean) isMeshProxyServiceAdvertisementData:(NSDictionary<NSString *,id> *)advertisementData
{
    CBUUID *proxyUuid = [CBUUID UUIDWithString:@"1828"];
    
    NSNumber *conntable = advertisementData[CBAdvertisementDataIsConnectable];
    if (conntable == nil || [conntable isEqual: [NSNumber numberWithUnsignedInteger:0]]) {
        return false;
    }
    
    NSArray *srvUuids = advertisementData[CBAdvertisementDataServiceUUIDsKey];
    if (srvUuids == nil || srvUuids.count == 0) {
        return false;
    }
    
    for (CBUUID *uuid in srvUuids) {
        if ([uuid isEqual: proxyUuid]) {
            NSDictionary *srvData = advertisementData[CBAdvertisementDataServiceDataKey];
            if (srvData != nil && srvData.count > 0) {
                NSArray *allKeys = [srvData allKeys];
                for (CBUUID *key in allKeys) {
                    if (![key isEqual:proxyUuid]) {
                        continue;
                    }
                    NSData *data = srvData[key];
                    if (data != nil && data.length > 0) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

+(Boolean) isMeshAdvertisementData:(NSDictionary<NSString *,id> *)advertisementData
{
    CBUUID *provUuid = [CBUUID UUIDWithString:@"1827"];
    CBUUID *proxyUuid = [CBUUID UUIDWithString:@"1828"];
    
    NSNumber *conntable = advertisementData[CBAdvertisementDataIsConnectable];
    if (conntable == nil || [conntable isEqual: [NSNumber numberWithUnsignedInteger:0]]) {
        return false;
    }
    
    NSArray *srvUuids = advertisementData[CBAdvertisementDataServiceUUIDsKey];
    if (srvUuids == nil || srvUuids.count == 0) {
        return false;
    }
    
    for (CBUUID *uuid in srvUuids) {
        if ([uuid isEqual: provUuid] || [uuid isEqual: proxyUuid]) {
            NSDictionary *srvData = advertisementData[CBAdvertisementDataServiceDataKey];
            if (srvData != nil && srvData.count > 0) {
                NSArray *allKeys = [srvData allKeys];
                for (CBUUID *key in allKeys) {
                    if (![key isEqual:provUuid] && ![key isEqual:proxyUuid]) {
                        continue;
                    }
                    NSData *data = srvData[key];
                    if (data != nil && data.length > 0) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

static NSMutableDictionary *gMeshBdAddrDict = nil;
-(void) meshBdAddrDictInit
{
    if (gMeshBdAddrDict == nil) {
        EnterCriticalSection();
        if (gMeshBdAddrDict == nil) {
            gMeshBdAddrDict = [[NSMutableDictionary alloc] init];
        }
        LeaveCriticalSection();
    }
}

- (void)destoryMeshClient
{
    [self deleteAllFiles];
    onceToken = 0;
    zoneOnceToken = 0;
    [gMeshBdAddrDict removeAllObjects];
    gMeshBdAddrDict = nil;
    for (NSString *key in [gMeshTimersDict allKeys]) {
        NSArray *timerInfo = [gMeshTimersDict valueForKey:key];
        if (timerInfo != nil && [timerInfo count] == 3) {
            NSTimer *timer = timerInfo[2];
            if (timer != nil) {
                [timer invalidate];
            }
        }
    }
    [gMeshTimersDict removeAllObjects];
    gMeshTimersDict = nil;
    strcpy(provisioner_uuid, "");
    _instance = nil;
}

+(void) meshBdAddrDictAppend:(NSData *)bdAddr peripheral:(CBPeripheral *)peripheral
{
    if (bdAddr == nil || bdAddr.length != BD_ADDR_LEN || peripheral == nil) {
        return;
    }
    [gMeshBdAddrDict setObject:peripheral forKey:bdAddr.description];
}

+(CBPeripheral *) meshBdAddrDictGetCBPeripheral:(NSData *)bdAddr
{
    if (bdAddr == nil || bdAddr.length != BD_ADDR_LEN) {
        return nil;
    }
    return [gMeshBdAddrDict valueForKey:bdAddr.description];
}

+(void) meshBdAddrDictDelete:(NSData *)bdAddr
{
    if (bdAddr == nil || bdAddr.length != BD_ADDR_LEN) {
        return;
    }
    [gMeshBdAddrDict removeObjectForKey:bdAddr.description];
}

+(void) meshBdAddrDictDeleteByCBPeripheral:(CBPeripheral *)peripheral
{
    if (peripheral == nil) {
        return;
    }
    for (NSData *bdAddr in [gMeshBdAddrDict allKeys]) {
        CBPeripheral *cachedPeripheral = [gMeshBdAddrDict valueForKey:bdAddr.description];
        if (peripheral == cachedPeripheral) {
            [gMeshBdAddrDict removeObjectForKey:bdAddr.description];
            break;
        }
    }
}

+(void) meshBdAddrDictClear
{
    [gMeshBdAddrDict removeAllObjects];
}

+(NSData *) MD5:(NSData *)data
{
    unsigned char md5Data[CC_MD5_DIGEST_LENGTH];
    memset(md5Data, 0, CC_MD5_DIGEST_LENGTH);
    CC_MD5(data.bytes, (unsigned int)data.length, md5Data);
    return [[NSData alloc] initWithBytes:md5Data length:CC_MD5_DIGEST_LENGTH];
}

+(NSData *)peripheralIdentifyToBdAddr:(CBPeripheral *)peripheral
{
    const char *uuid = peripheral.identifier.UUIDString.UTF8String;
    unsigned char md5Data[CC_MD5_DIGEST_LENGTH];
    CC_MD5(uuid, (unsigned int)strlen(uuid), md5Data);
    return [[NSData alloc] initWithBytes:md5Data length:BD_ADDR_LEN];
}

+(NSData *) getMeshPeripheralMappedBdAddr:(CBPeripheral *)peripheral
{
    const char *uuid = peripheral.identifier.UUIDString.UTF8String;
    unsigned char md5Data[CC_MD5_DIGEST_LENGTH];
    CC_MD5(uuid, (unsigned int)strlen(uuid), md5Data);
    NSData * bdAddr = [[NSData alloc] initWithBytes:md5Data length:BD_ADDR_LEN];
    [MeshNativeHelper meshBdAddrDictAppend:bdAddr peripheral:peripheral];
    return bdAddr;
}

#define MESH_MAX_RAW_ADVERTISEMENT_DATA_SIZE    62          /* max is 31 advertisement data combined with 31 scan response data */
+(NSData *) getConvertedRawMeshAdvertisementData:(CBPeripheral *)peripheral advertisementData:(NSDictionary<NSString *,id> *)advertisementData rssi:(NSNumber *)rssi
{
    BOOL isConntable = false;
    BOOL isMeshServiceFound = false;
    CBUUID *provUuid = [CBUUID UUIDWithString:@"1827"];
    CBUUID *proxyUuid = [CBUUID UUIDWithString:@"1828"];
    /* assume the advertisementData was combined with max 31 bytes advertisement data and max 31 bytes scan response data. */
    unsigned char rawAdvData[MESH_MAX_RAW_ADVERTISEMENT_DATA_SIZE];
    int rawAdvDataSize = 0;
    unsigned char *p;
    
    NSNumber *conntable = advertisementData[CBAdvertisementDataIsConnectable];
    if (conntable != nil && [conntable isEqual: [NSNumber numberWithUnsignedInteger:1]]) {
        isConntable = true;
        if ((rawAdvDataSize + 1 + 2) <= MESH_MAX_RAW_ADVERTISEMENT_DATA_SIZE) {
            rawAdvData[rawAdvDataSize++] = 2;                           // length
            rawAdvData[rawAdvDataSize++] = BTM_BLE_ADVERT_TYPE_FLAG;    // flag type
            rawAdvData[rawAdvDataSize++] = 0x06;                        // Flags value
        }
    }
    
    NSArray *srvUuids = advertisementData[CBAdvertisementDataServiceUUIDsKey];
    if (srvUuids != nil && srvUuids.count > 0) {
        for (CBUUID *uuid in srvUuids) {
            if ([uuid isEqual: provUuid] || [uuid isEqual: proxyUuid]) {
                isMeshServiceFound = true;
                if ((rawAdvDataSize + uuid.data.length + 2) <= MESH_MAX_RAW_ADVERTISEMENT_DATA_SIZE) {
                    rawAdvData[rawAdvDataSize++] = uuid.data.length + 1;                    // length
                    rawAdvData[rawAdvDataSize++] = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;      // flag type
                    /* UUID data bytes is in big-endia format, but raw adv data wants in little-endian format. */
                    p = (unsigned char *)uuid.data.bytes;
                    p += (uuid.data.length - 1);
                    for (int i = 0; i < uuid.data.length; i++) {
                        rawAdvData[rawAdvDataSize++] = *p--;                                // little-endian UUID data
                    }
                }
            }
        }
    }
    
    /* Not mesh device/proxy advertisement data, return nil. */
    if (!isConntable || !isMeshServiceFound) {
        return nil;
    }
    
    NSDictionary *srvData = advertisementData[CBAdvertisementDataServiceDataKey];
    if (srvData != nil && srvData.count > 0) {
        NSArray *allKeys = [srvData allKeys];
        for (CBUUID *key in allKeys) {
            if (![key isEqual:provUuid] && ![key isEqual:proxyUuid]) {
                continue;
            }
            
            NSData *data = srvData[key];
            if (data != nil && data.length > 0) {
                if ((rawAdvDataSize + key.data.length + data.length + 2) <= MESH_MAX_RAW_ADVERTISEMENT_DATA_SIZE) {
                    rawAdvData[rawAdvDataSize++] = key.data.length + data.length + 1;   // length
                    rawAdvData[rawAdvDataSize++] = BTM_BLE_ADVERT_TYPE_SERVICE_DATA;    // flag type
                    p = (unsigned char *)key.data.bytes;                                // data: service UUID.
                    p += (key.data.length - 1);
                    for (int i = 0; i < key.data.length; i++) {
                        rawAdvData[rawAdvDataSize++] = *p--;
                    }
                    memcpy(&rawAdvData[rawAdvDataSize], data.bytes, data.length);       // data: service UUID data.
                    rawAdvDataSize += data.length;
                }
            }
        }
    }
    
    /* Add local name of the peripheral if exsiting. */
    NSString *localName = advertisementData[CBAdvertisementDataLocalNameKey];
    if (localName == nil || strlen(localName.UTF8String) == 0) {
        localName = peripheral.name;
    }
    if (localName != nil && strlen(localName.UTF8String) > 0) {
        unsigned long nameLen = strlen(localName.UTF8String);
        if ((rawAdvDataSize + nameLen + 2) <= MESH_MAX_RAW_ADVERTISEMENT_DATA_SIZE) {
            rawAdvData[rawAdvDataSize++] = nameLen + 1;                            // length
            rawAdvData[rawAdvDataSize++] = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;      // flag type
            memcpy(&rawAdvData[rawAdvDataSize], localName.UTF8String, nameLen);    // Name data
            rawAdvDataSize += nameLen;
        }
    }
    
    /* Add power level into raw adv data. */
    if ((rawAdvDataSize + 1 + 2) <= MESH_MAX_RAW_ADVERTISEMENT_DATA_SIZE) {
        rawAdvData[rawAdvDataSize++] = 2;                               // length
        rawAdvData[rawAdvDataSize++] = BTM_BLE_ADVERT_TYPE_TX_POWER;    // flag type
        rawAdvData[rawAdvDataSize++] = rssi.unsignedCharValue;          // TX Power Level
    }
    
    return [[NSData alloc] initWithBytes:rawAdvData length:rawAdvDataSize];
}

void dumpHexBytes(const void *data, unsigned long size)
{
    const unsigned char *p = data;
    for (int i = 0; i < size; i++) {
        printf("%02X ", p[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}


+(void) meshClientSetGattMtu:(int)mtu
{
    NSLog(@"%s, mtu: %d", __FUNCTION__, mtu);
    EnterCriticalSection();
    wiced_bt_mesh_core_set_gatt_mtu((uint16_t)mtu);
    LeaveCriticalSection();
}
+(Boolean) meshClientIsConnectedToNetwork
{
    NSLog(@"%s", __FUNCTION__);
    Boolean is_proxy_connected;
    EnterCriticalSection();
    is_proxy_connected = mesh_client_is_proxy_connected();
    LeaveCriticalSection();
    return is_proxy_connected;
}

+(int) meshClientAddComponent:(NSString *)componentName toGorup:(NSString *)groupName
{
    NSLog(@"[MeshNativeHelper meshClientAddComponent] componentName: %@, groupName: %@", componentName, groupName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_add_component_to_group(componentName.UTF8String, groupName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

+(NSData *) meshClientOTADataEncrypt:(NSString *)componentName data:(NSData *)data
{
    //NSLog(@"[MeshNativeHelper meshClientOTADataEncrypt] componentName: %@, length: %lu", componentName, (unsigned long)[data length]);
    /* The output buffer should be at least 17 bytes larger than input buffer */
    uint8_t *pOutBuffer = (uint8_t *)malloc(data.length + 17);
    uint16_t outBufferLen = 0;
    NSData *outData = nil;
    
    if (pOutBuffer) {
        EnterCriticalSection();
        outBufferLen = mesh_client_ota_data_encrypt(componentName.UTF8String, data.bytes, data.length, pOutBuffer, data.length + 17);
        LeaveCriticalSection();
        if (outBufferLen > 0) {
            outData = [[NSData alloc] initWithBytes:pOutBuffer length:outBufferLen];
        }
    }
    free(pOutBuffer);
    return outData;
}

+(NSData *) meshClientOTADataDecrypt:(NSString *)componentName data:(NSData *)data
{
    //NSLog(@"[MeshNativeHelper meshClientOTADataDecrypt] componentName: %@, length: %lu", componentName, (unsigned long)[data length]);
    /* The output buffer should be at least 17 bytes larger than input buffer */
    uint8_t *pOutBuffer = (uint8_t *)malloc(data.length + 17);
    uint16_t outBufferLen = 0;
    NSData *outData = nil;
    
    if (pOutBuffer) {
        EnterCriticalSection();
        outBufferLen = mesh_client_ota_data_decrypt(componentName.UTF8String, data.bytes, data.length, pOutBuffer, data.length + 17);
        LeaveCriticalSection();
        if (outBufferLen > 0) {
            outData = [[NSData alloc] initWithBytes:pOutBuffer length:outBufferLen];
        }
    }
    free(pOutBuffer);
    return outData;
}

+(NSArray<NSString *> *) meshClientGetComponentGroupList:(NSString *)componentName
{
    NSLog(@"[MeshNativeHelper meshClientGetComponentGroupList] componentName: %@", componentName);
    char *groupList = NULL;
    EnterCriticalSection();
    groupList = mesh_client_get_component_group_list((char *)componentName.UTF8String);
    LeaveCriticalSection();
    return meshCStringToOCStringArray(groupList, TRUE);
}

+(int) meshClientRemoveComponent:(NSString *)componentName from:(NSString *)groupName
{
    NSLog(@"[MeshNativeHelper meshClientRemoveComponent] componentName:%@, groupName:%@", componentName, groupName);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_remove_component_from_group(componentName.UTF8String, groupName.UTF8String);
    LeaveCriticalSection();
    return ret;
}

-(NSString *) getJsonFilePath {
    
    NSFileManager *fileManager = [NSFileManager defaultManager];
    NSArray *list = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, true);
    NSString *homeDirectory = list[0];
    NSString *fileDirectory = [homeDirectory stringByAppendingPathComponent:@"mesh"];
    NSLog(@"%s current fileDirectory \"%@\"", __FUNCTION__, fileDirectory);
    
    NSString *fileContent;
    NSArray  *contents = [fileManager contentsOfDirectoryAtPath:fileDirectory error:nil];
    NSString *filePathString;
    
    for (fileContent in contents){
        if([[fileContent pathExtension]isEqualToString:@"json"]){
            NSLog(@"%@ file Name",fileContent);
            filePathString = fileContent;
            break;
        }
    }
    
    NSString *filePath = [fileDirectory stringByAppendingPathComponent:filePathString];
    return filePath;
}

-(void) deleteAllFiles {
    
    NSFileManager *fileManager = [NSFileManager defaultManager];
    NSArray *list = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, true);
    NSString *homeDirectory = list[0];
    NSString *fileDirectory = [homeDirectory stringByAppendingPathComponent:@"mesh"];
    NSLog(@"%s current fileDirectory \"%@\"", __FUNCTION__, fileDirectory);
    
    NSString *fileContent;
    NSArray  *contents = [fileManager contentsOfDirectoryAtPath:fileDirectory error:nil];
    
    for (fileContent in contents){
        NSString *fullFilePath = [fileDirectory stringByAppendingPathComponent:fileContent];
        NSLog(@"%@", fullFilePath);
        [fileManager removeItemAtPath:fullFilePath error: NULL];
    }
}


static CBPeripheral *currentConnectedPeripheral = nil;

+(void) setCurrentConnectedPeripheral:(CBPeripheral *)peripheral
{
    @synchronized (MeshNativeHelper.getSharedInstance) {
        currentConnectedPeripheral = peripheral;
    }
}

+(CBPeripheral *) getCurrentConnectedPeripheral
{
    CBPeripheral *peripheral = nil;
    @synchronized (MeshNativeHelper.getSharedInstance) {
        peripheral = currentConnectedPeripheral;
    }
    return peripheral;
}

// DFU APIs

// This API must be called firslty to set DFU FW images before calling any other DFU APIs.
+(void) meshClienSetDfuFiles:(NSString *)fwFileName metadataFile:(NSString *)metadataFile
{
    NSLog(@"[MeshNativeHelper meshClienSetDfuFiles] fwFileName:%@, metadataFile:%@", fwFileName, metadataFile);
    strcpy(dfuFwImageFileName, fwFileName.UTF8String);
    strcpy(dfuFwMetadataFileName, metadataFile.UTF8String);
}

bool getFirmwareIdFromFile(char *dfuFwMetadataFile, mesh_dfu_fw_id_t *fwId)
{
    FILE *file = fopen(dfuFwMetadataFile, "r");
    if (file == NULL) {
        NSLog(@"[MeshNativeHelper getFirmwareIdFromFile] error: failed to open file:%s, errno:%d", dfuFwMetadataFile, errno);
        return FALSE;
    }
    
    char line[100];
    DWORD cid = 0;
    unsigned long long fw_id = 0;
    while (fgets(line, 100, file)) {
        if (strstr(line, "CID=") == line) {
            sscanf(&line[4], "0x%x", &cid);
        } else {
            sscanf(&line[5], "0x%llx", &fw_id);
        }
    }
    
    fwId->company_id = (uint16_t)cid;
    fwId->fw_id_len = 8;
    fwId->fw_id[0] = (uint8_t)((fw_id >> 56) & 0xFF);
    fwId->fw_id[1] = (uint8_t)((fw_id >> 48) & 0xFF);
    fwId->fw_id[2] = (uint8_t)((fw_id >> 40) & 0xFF);
    fwId->fw_id[3] = (uint8_t)((fw_id >> 32) & 0xFF);
    fwId->fw_id[4] = (uint8_t)((fw_id >> 24) & 0xFF);
    fwId->fw_id[5] = (uint8_t)((fw_id >> 16) & 0xFF);
    fwId->fw_id[6] = (uint8_t)((fw_id >> 8) & 0xFF);
    fwId->fw_id[7] = (uint8_t)(fw_id & 0xFF);
    
    fclose(file);
    return TRUE;
}

void mesh_client_dfu_status_cb(uint8_t status, int cur_block_number, int total_blocks)
{
    NSLog(@"[MeshNativeHelper mesh_client_dfu_status_cb] status:%d, cur_block_number:%d, total_blocks:%d", status, cur_block_number, total_blocks);
    [nativeCallbackDelegate meshClientDfuStatusCb:status currentBlockNumber:cur_block_number totalBlocks:total_blocks];
}

+(int) meshClientDfuGetStatus:(NSString *)componentName
{
    NSLog(@"[MeshNativeHelper meshClientDfuGetStatus] componentName:%@", componentName);
    mesh_dfu_fw_id_t fwId;
    int ret = MESH_CLIENT_ERR_NOT_FOUND;    // failed to get firmwareId from metadata file.
    
    EnterCriticalSection();
    if (getFirmwareIdFromFile(dfuFwMetadataFileName, &fwId)) {
        ret = mesh_client_dfu_get_status((char *)componentName.UTF8String, fwId.company_id, fwId.fw_id, fwId.fw_id_len, mesh_client_dfu_status_cb);
    }
    
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientDfuStart:(int)dfuMethod  componentName:(NSString *)componentName
{
    NSLog(@"[MeshNativeHelper meshClientDfuStop] dfuMethod:%d, componentName:%@", dfuMethod, componentName);
    mesh_dfu_fw_id_t fwId;
    int ret = MESH_CLIENT_ERR_NOT_FOUND;    // failed to get firmwareId from metadata file.
    
    EnterCriticalSection();
    if (getFirmwareIdFromFile(dfuFwMetadataFileName, &fwId)) {
        ret = mesh_client_dfu_start((uint8_t)dfuMethod, (char *)componentName.UTF8String, fwId.company_id, fwId.fw_id, fwId.fw_id_len);
    }
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientDfuStop
{
    NSLog(@"[MeshNativeHelper meshClientDfuStop]");
    int ret;
    
    EnterCriticalSection();
    ret = mesh_client_dfu_stop();
    LeaveCriticalSection();
    return ret;
}

// Sensor APIs
+(int) meshClientSensorCadenceGet:(NSString *)deviceName
                       propertyId:(int)propertyId
         fastCadencePeriodDivisor:(int *)fastCadencePeriodDivisor
                      triggerType:(int *)triggerType
                 triggerDeltaDown:(int *)triggerDeltaDown
                   triggerDeltaUp:(int *)triggerDeltaUp
                      minInterval:(int *)minInterval
                   fastCadenceLow:(int *)fastCadenceLow
                  fastCadenceHigh:(int *)fastCadenceHigh
{
    NSLog(@"[MeshNativeHelper meshClientSensorCadenceGet] deviceName:%@, propertyId:0x%04x", deviceName, propertyId);
    uint16_t fast_cadence_period_divisor = 0;
    wiced_bool_t trigger_type = 0;
    uint32_t trigger_delta_down = 0;
    uint32_t trigger_delta_up = 0;
    uint32_t min_interval = 0;
    uint32_t fast_cadence_low = 0;
    uint32_t fast_cadence_high = 0;
    int ret;
    
    EnterCriticalSection();
    ret = mesh_client_sensor_cadence_get(deviceName.UTF8String, propertyId,
                                         &fast_cadence_period_divisor,
                                         &trigger_type,
                                         &trigger_delta_down,
                                         &trigger_delta_up,
                                         &min_interval,
                                         &fast_cadence_low,
                                         &fast_cadence_high);
    LeaveCriticalSection();
    
    if (ret == MESH_CLIENT_SUCCESS) {
        *fastCadencePeriodDivisor = (int)fast_cadence_period_divisor;
        *triggerType = (int)trigger_type;
        *triggerDeltaDown = (int)trigger_delta_down;
        *triggerDeltaUp = (int)trigger_delta_up;
        *minInterval = (int)min_interval;
        *fastCadenceLow = (int)fast_cadence_low;
        *fastCadenceHigh = (int)fast_cadence_high;
    }
    return ret;
}

+(int) meshClientSensorCadenceSet:(NSString *)deviceName
                       propertyId:(int)propertyId
         fastCadencePeriodDivisor:(int)fastCadencePeriodDivisor
                      triggerType:(int)triggerType
                 triggerDeltaDown:(int)triggerDeltaDown
                   triggerDeltaUp:(int)triggerDeltaUp
                      minInterval:(int)minInterval
                   fastCadenceLow:(int)fastCadenceLow
                  fastCadenceHigh:(int)fastCadenceHigh
{
    NSLog(@"[MeshNativeHelper meshClientSensorCadenceSet] deviceName:%@, propertyId:0x%04x, %u, %u, %u, %u, %u, %u, %u",
          deviceName, propertyId, fastCadencePeriodDivisor, triggerType, triggerDeltaDown, triggerDeltaUp, minInterval, fastCadenceLow, fastCadenceHigh);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_sensor_cadence_set(deviceName.UTF8String, propertyId,
                                         (uint16_t)fastCadencePeriodDivisor,
                                         (wiced_bool_t)triggerType,
                                         (uint32_t)triggerDeltaDown,
                                         (uint32_t)triggerDeltaUp,
                                         (uint32_t)minInterval,
                                         (uint32_t)fastCadenceLow,
                                         (uint32_t)fastCadenceHigh);
    LeaveCriticalSection();
    return ret;
}

+(NSData *) meshClientSensorSettingGetPropertyIds:(NSString *)componentName
                                               propertyId:(int)propertyId
{
    NSLog(@"[MeshNativeHelper meshClientSensorSettingGetPropertyIds] componentName:%@, propertyId:0x%04x", componentName, propertyId);
    NSData *settingPropertyIdsData = nil;
    int *settingPropertyIds = NULL;
    int count = 0;
    
    EnterCriticalSection();
    settingPropertyIds = mesh_client_sensor_setting_property_ids_get(componentName.UTF8String, propertyId);
    LeaveCriticalSection();
    
    if (settingPropertyIds != NULL) {
        while (settingPropertyIds[count] != WICED_BT_MESH_PROPERTY_UNKNOWN) {
            count += 1;
        }
        settingPropertyIdsData = [NSData dataWithBytes:(void *)settingPropertyIds length:(count * sizeof(int))];
        free(settingPropertyIds);
    }
    return settingPropertyIdsData;
}

+(NSData *) meshClientSensorPropertyListGet:(NSString *)componentName
{
    NSLog(@"[MeshNativeHelper meshClientSensorPropertyListGet] componentName:%@", componentName);
    NSData *propertyListData = nil;
    int *propertyList = NULL;
    int count = 0;
    
    EnterCriticalSection();
    propertyList = mesh_client_sensor_property_list_get(componentName.UTF8String);
    LeaveCriticalSection();
    
    if (propertyList != NULL) {
        while (propertyList[count] != WICED_BT_MESH_PROPERTY_UNKNOWN) {
            count += 1;
        }
        propertyListData = [NSData dataWithBytes:(void *)propertyList length:(count * sizeof(int))];
        free(propertyList);
    }
    return propertyListData;
}

+(int) meshClientSensorSettingSet:(NSString *)componentName
                       propertyId:(int)propertyId
                settingPropertyId:(int)settingPropertyId
                            value:(NSData *)value
{
    NSLog(@"[MeshNativeHelper meshClientSensorSettingSet] componentName:%@, propertyId:0x%04x, settingPropertyId:0x%04x", componentName, propertyId, settingPropertyId);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_sensor_setting_set(componentName.UTF8String, propertyId, settingPropertyId, (uint8_t *)value.bytes);
    LeaveCriticalSection();
    return ret;
}

+(int) meshClientSensorGet:(NSString *)componentName propertyId:(int)propertyId
{
    NSLog(@"[MeshNativeHelper meshClientSensorGet] componentName:%@, propertyId:0x%04x", componentName, propertyId);
    int ret;
    EnterCriticalSection();
    ret = mesh_client_sensor_get(componentName.UTF8String, propertyId);
    LeaveCriticalSection();
    return ret;
}

@end
