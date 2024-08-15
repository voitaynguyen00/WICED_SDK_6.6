/*
 * Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
#import "SSIDNetwork.h"
#import <SystemConfiguration/CaptiveNetwork.h>
#import <CoreLocation/CoreLocation.h>

@interface NetworkInfo()
@end
@implementation NetworkInfo
+ (NSString *)getNetworkInfo:(CFStringRef)key {
    for (;;) {
        if (@available(iOS 13.0, *)) {
            if ([CLLocationManager authorizationStatus] == kCLAuthorizationStatusDenied) {
                NSLog(@"User has explicitly denied authorization for this application, or location services are disabled in Settings.");
                // [[UIApplication sharedApplication] openURL:[NSURL URLWithString:UIApplicationOpenSettingsURLString]];
                return nil;
        }
            if(![CLLocationManager locationServicesEnabled] || [CLLocationManager authorizationStatus] == kCLAuthorizationStatusNotDetermined){
                // Popup a dialogue prompt user to enable or disable the location access rights.
                CLLocationManager* locationMgr = [[CLLocationManager alloc] init];
                [locationMgr requestWhenInUseAuthorization];
                sleep(1);
                // Loop to wait user to make a select on the UI.
                // User may need to touch the UI to make the selection dialogue popup.
                continue;
    }
    }
        NSString *wifiName = nil;
        CFArrayRef wifiInterfaces = CNCopySupportedInterfaces();
        if (!wifiInterfaces) {
        return nil;
        }
        NSArray *interfaces = (__bridge NSArray *)wifiInterfaces;
        for (NSString *interfaceName in interfaces) {
            CFDictionaryRef dictRef = CNCopyCurrentNetworkInfo((__bridge CFStringRef)(interfaceName));

            if (dictRef) {
                NSDictionary *networkInfo = (__bridge NSDictionary *)dictRef;
                wifiName = [networkInfo objectForKey:(__bridge NSString *)key];
                CFRelease(dictRef);
        }
    }

        CFRelease(wifiInterfaces);
        return wifiName;
    }
        return nil;
}
@end

@interface SSIDNetwork ()
@end
@implementation SSIDNetwork
- (NSString*)getNetworkSSID {
    return [NetworkInfo getNetworkInfo:kCNNetworkInfoKeySSID];
}
@end

@interface BSSIDNetwork ()
@end
@implementation BSSIDNetwork
- (NSString*)getNetworkBSSID {
    return [NetworkInfo getNetworkInfo:kCNNetworkInfoKeyBSSID];
}
@end
