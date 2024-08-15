//
// Package: HomeKit-ADK
// Version: 1.1 (15E10)
//
// Disclaimer: IMPORTANT: This Apple software is supplied to you, by Apple Inc. ("Apple"), in your
// capacity as a current, and in good standing, Licensee in the MFi Licensing Program. Use of this
// Apple software is governed by and subject to the terms and conditions of your MFi License,
// including, but not limited to, the restrictions specified in the provision entitled "Public
// Software", and is further subject to your agreement to the following additional terms, and your
// agreement that the use, installation, modification or redistribution of this Apple software
// constitutes acceptance of these additional terms. If you do not agree with these additional terms,
// you may not use, install, modify or redistribute this Apple software.
//
// Subject to all of these terms and in consideration of your agreement to abide by them, Apple grants
// you, for as long as you are a current and in good-standing MFi Licensee, a personal, non-exclusive
// license, under Apple's copyrights in this Apple software (the "Apple Software"), to use,
// reproduce, and modify the Apple Software in source form, and to use, reproduce, modify, and
// redistribute the Apple Software, with or without modifications, in binary form, in each of the
// foregoing cases to the extent necessary to develop and/or manufacture "Proposed Products" and
// "Licensed Products" in accordance with the terms of your MFi License. While you may not
// redistribute the Apple Software in source form, should you redistribute the Apple Software in binary
// form, you must retain this notice and the following text and disclaimers in all such redistributions
// of the Apple Software. Neither the name, trademarks, service marks, or logos of Apple Inc. may be
// used to endorse or promote products derived from the Apple Software without specific prior written
// permission from Apple. Except as expressly stated in this notice, no other rights or licenses,
// express or implied, are granted by Apple herein, including but not limited to any patent rights that
// may be infringed by your derivative works or by other works in which the Apple Software may be
// incorporated. Apple may terminate this license to the Apple Software by removing it from the list
// of Licensed Technology in the MFi License, or otherwise in accordance with the terms of such MFi License.
//
// Unless you explicitly state otherwise, if you provide any ideas, suggestions, recommendations, bug
// fixes or enhancements to Apple in connection with this software ("Feedback"), you hereby grant to
// Apple a non-exclusive, fully paid-up, perpetual, irrevocable, worldwide license to make, use,
// reproduce, incorporate, modify, display, perform, sell, make or have made derivative works of,
// distribute (directly or indirectly) and sublicense, such Feedback in connection with Apple products
// and services. Providing this Feedback is voluntary, but if you do provide Feedback to Apple, you
// acknowledge and agree that Apple may exercise the license granted above without the payment of
// royalties or further consideration to Participant.
//
// The Apple Software is provided by Apple on an "AS IS" basis. APPLE MAKES NO WARRANTIES, EXPRESS OR
// IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY
// AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE OR ITS USE AND OPERATION ALONE OR
// IN COMBINATION WITH YOUR PRODUCTS.
//
// IN NO EVENT SHALL APPLE BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION, MODIFICATION
// AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT
// (INCLUDING NEGLIGENCE), STRICT LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (C) 2018 Apple Inc. All Rights Reserved.
//

#ifndef HAP_PLATFORM_BLE_PERIPHERAL_MANAGER_INIT_H
#define HAP_PLATFORM_BLE_PERIPHERAL_MANAGER_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "HAPPlatform.h"

#if __has_feature(nullability)
#pragma clang assume_nonnull begin
#endif

/**
 * BLE peripheral manager initialization options.
 */
typedef struct {
    /**
     * Key-value store.
     * If critical data is pending, BLE reads will be delayed until the data is persisted.
     */
    HAPPlatformKeyValueStoreRef keyValueStore;
    
    /**
     * GATT base DB.
     * This includes GATT/GAP services and any non-HAP service
     */
    uint8_t *gattBaseDB;
    size_t gattBaseDBSize;

    /**
     * GATT DB HAP services handle range.
     */
    uint16_t gattDBHapHandleStart;
    uint16_t gattDBHapHandleEnd;
    
    /**
     * GATT extended DB.
     * This includes services with handles start at 0xff00
     */
    uint8_t *gattExtDB;
    size_t gattExtDBSize;

} HAPPlatformBLEPeripheralManagerOptions;

/**
 * BLE peripheral manager.
 */
struct HAPPlatformBLEPeripheralManager {
    // Opaque type. Do not access the instance fields directly.
    /**@cond */
    HAPPlatformBLEPeripheralManagerOptions options;
    
    HAPPlatformBLEPeripheralManagerDelegate delegate;

    uint16_t connectionHandle;

    uint32_t appMode;

    uint16_t mtu;
    /**@endcond */
};

/**
 * Initializes the BLE peripheral manager.
 *
 * @param[out] blePeripheralManager Pointer to an allocated but uninitialized HAPPlatformBLEPeripheralManager structure.
 * @param      options              Initialization options.
 */
void HAPPlatformBLEPeripheralManagerCreate(
    HAPPlatformBLEPeripheralManagerRef blePeripheralManager,
    const HAPPlatformBLEPeripheralManagerOptions *options);

/**
 * Handles a GATT event from the Bluetooth stack.
 *
 * - If custom non-HomeKit characteristics are added, the related events must be filtered before calling this function.
 *
 * @param      blePeripheralManager BLE peripheral manager.
 * @param      event                GATT event.
 * @param      eventData            GATT event data.
 */
wiced_bt_gatt_status_t HAPPlatformBLEPeripheralManagerHandleGATTEvent(
    HAPPlatformBLEPeripheralManagerRef blePeripheralManager,
    wiced_bt_gatt_evt_t event,
    wiced_bt_gatt_event_data_t *eventData);

/**
 * App calls this function to inform library which mode it is in:
 *  HAP mode: In this mode HAP timeouts apply. This is the default mode.
 *  PRIVATE mode: App is in a private (non-HAP) connection. HAP timeouts do not apply.
 */
void HAPPlatformBLEPeripheralManagerSetAppConnMode(
    HAPPlatformBLEPeripheralManagerRef blePeripheralManager,
    uint32_t mode);

#if __has_feature(nullability)
#pragma clang assume_nonnull end
#endif

#ifdef __cplusplus
}
#endif

#endif
