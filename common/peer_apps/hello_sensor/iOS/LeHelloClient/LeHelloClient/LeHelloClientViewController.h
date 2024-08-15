//
//  LeHelloClientViewController.h
//  LeHelloClient
//
//  Created by fredc on 2/20/14.
//  Copyright (c) 2014 Cypress Semiconductor. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "LeDevicePickerCallback.h"

@interface LeHelloClientViewController : UIViewController<LeDevicePickerCallback, CBCentralManagerDelegate, CBPeripheralDelegate >

@end
