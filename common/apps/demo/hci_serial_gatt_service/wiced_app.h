/*
 * File: wiced_app.h
 * Copyright (c) Cypress Semiconductor 2016
 *
 * Description: This contains the Application definitions
 *
 *
 */


#ifndef _WICED_APP_H_
#define _WICED_APP_H_

#include "wiced_bt_trace.h"
#include "wiced_hci.h"

#define WICED_PIN_CODE_LEN                  4
extern const uint8_t pincode[WICED_PIN_CODE_LEN];

/* BR/EDR Profiles/Applications */
#define WICED_APP_TEST_INCLUDED             TRUE

/* BLE Profiles/Applications */
#define WICED_APP_LE_INCLUDED               TRUE
#define WICED_APP_LE_SERIAL_GATT_INCLUDED   TRUE

#endif /* _WICED_APP_H_ */

