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

/** @file
 *
 * This file defines all callback APIs should be registered to the MeshNativeHelper.
 * These callback APIs will be invoked by mesh libraries when triggered.
 */

#ifndef IMeshNativeCallback_h
#define IMeshNativeCallback_h

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@protocol IMeshNativeCallback
/**
 * Demonstrates some basic types that you can use as parameters
 * and return values in AIDL.
 */
-(void) onProvGattPktReceivedCallback:(uint16_t)connId data:(NSData *)data;
-(void) onProxyGattPktReceivedCallback:(uint16_t)connId data:(NSData *)data;
-(void) onDeviceFound:(NSUUID *)uuid oob:(uint16_t)oob uriHash:(uint32_t)uriHash name:(NSString * __nullable)name;
-(void) onResetStatus:(uint8_t)status devName:(NSString *)devName;
-(void) onLinkStatus:(uint8_t)isConnected connId:(uint32_t)connId addr:(uint16_t)addr isOverGatt:(uint8_t)isOverGatt;

-(void) onDatabaseChangedCb:(NSString *)meshName;
/* Async result callback function of meshClientGetComponentInfo API. */
-(void) meshClientComponentInfoStatusCb:(uint8_t)status componentName:(NSString *)componentName componentInfo:(NSString * __nullable)componentInfo;
-(void) onMeshClientSensorStatusChanged:(NSString *)deviceName propertyId:(uint32_t)propertyId data:(NSData *)data;

-(void) meshClientNetworkOpenCb:(uint8_t)status;
-(void) meshClientProvisionCompletedCb:(uint8_t)status uuid:(NSUUID *)uuid;
-(void) meshClientOnOffStateCb:(NSString *)deviceName target:(uint8_t)target present:(uint8_t)present remainingTime:(uint32_t)remainingTime;
-(void) meshClientLevelStateCb:(NSString *)deviceName target:(int16_t)target present:(int16_t)present remainingTime:(uint32_t)remainingTime;
-(void) meshClientHslStateCb:(NSString *)deviceName lightness:(uint16_t)lightness hue:(uint16_t)hue saturation:(uint16_t)saturation;
-(void) meshClientCtlStateCb:(NSString *)deviceName
            presentLightness:(uint16_t)presentLightness
          presentTemperature:(uint16_t)presentTemperature
             targetLightness:(uint16_t)targetLightness
           targetTemperature:(uint16_t)targetTemperature
               remainingTime:(uint32_t)remainingTime;
-(void) meshClientLightnessStateCb:(NSString *)deviceName target:(uint16_t)target present:(uint16_t)present remainingTime:(uint32_t)remainingTime;
-(void) meshClientDfuStatusCb:(uint8_t)status currentBlockNumber:(uint32_t)currentBlockNumber totalBlocks:(uint32_t)totalBlocks;

//GATT APIS
-(Boolean) meshClientAdvScanStartCb;
-(void) meshClientAdvScanStopCb;
-(Boolean) meshClientConnect:(NSData *)bdaddr;
-(Boolean) meshClientDisconnect:(uint16_t)connId;
-(void) meshClientNodeConnectStateCb:(uint8_t)status componentName:(NSString *)componentName;
-(Boolean) meshClientSetScanTypeCb:(uint8_t)scanType;

@end

NS_ASSUME_NONNULL_END

#endif /* IMeshNativeCallback_h */
