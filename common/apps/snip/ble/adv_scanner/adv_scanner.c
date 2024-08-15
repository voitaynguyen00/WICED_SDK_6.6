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
 * BLE Advertisement Scanner
 */
#include <stdio.h>
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_app_common.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_bt_trace.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"

// This application is used for scanning and traces BLE advertisement reports. 
// To filter for specific list of BDA(s), set the count in 'filter_dev_num' and BDA(s) in 'filter_dev_list' array.

// device address list to filter for adv from these devices only.
// If filter_dev_num is 0, no filtering at all
uint16_t filter_dev_num = 0; // 3
uint8_t  filter_dev_list[3][6] = 
{
    // {0xCE, 0xA7, 0xAC, 0xD3, 0x5E, 0x76},
    {0xE5, 0x4A, 0x37, 0xD4, 0x6E, 0x61},
    {0xE7, 0xE2, 0x08, 0xBE, 0xA2, 0xF0},
    {0x20, 0x73, 0x5b, 0x09, 0x3e, 0x41}
};


/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define EXCLUDE_BEACONS     // Exclude beacons from the report

#define APP_TIMER_IN_SECONDS             1       // App Timer Timeout in seconds

// GPIO pins
#define ADV_SCANNER_GPIO_PIN_BUTTON      WICED_GPIO_PIN_BUTTON  // pin for button interrupts
#define ADV_SCANNER_GPIO_PIN_LED         WICED_GPIO_PIN_LED1    // pin for LED

// GPIO Settings
#ifndef WICED_BUTTON_PRESSED_VALUE
#define WICED_BUTTON_PRESSED_VALUE                 1
#endif
#define ADV_SCANNER_GPIO_BUTTON_SETTINGS (GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE)
#define ADV_SCANNER_GPIO_LED_SETTINGS    WICED_GPIO_LED_SETTINGS
#define SCAN_BUTTON_PRESSED              WICED_BUTTON_PRESSED_VALUE

enum
{
    HANDLE_ADV_SCANNER_GATT_SERVICE = 0x1,                      // GATT service handle

    HANDLE_ADV_SCANNER_GAP_SERVICE = 0x14,                      // GAP service handle
        HANDLE_ADV_SCANNER_GAP_SERVICE_CHAR_DEV_NAME,           // device name characteristic handle
        HANDLE_ADV_SCANNER_GAP_SERVICE_CHAR_DEV_NAME_VAL,       // char value handle

        HANDLE_ADV_SCANNER_GAP_SERVICE_CHAR_DEV_APPEARANCE,     // appearance characteristic handle
        HANDLE_ADV_SCANNER_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, // char value handle

    HANDLE_ADV_SCANNER_SERVICE =  0x28,                         // Adv Scanner Service
        HANDLE_ADV_SCANNER_SERVICE_CHAR_NOTIFY,                 // notify characteristic handle
        HANDLE_ADV_SCANNER_SERVICE_CHAR_NOTIFY_VAL,             // characteristic value handle
            HANDLE_ADV_SCANNER_SERVICE_CHAR_CFG_DESC,           // characteristic client configuration descriptor handle

    HANDLE_ADV_SCANNER_DEV_INFO_SERVICE = 0x40,                 // Device Information Service
        HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_MFR_NAME,      // manufacturer name characteristic handle
        HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  // characteristic value handle

        HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_MODEL_NUM,     // model number characteristic handle
        HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, // characteristic value handle

        HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_SYSTEM_ID,     // system ID characteristic handle
        HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, // characteristic value handle
};

/* UUID value of the Adv Scanner Service */
#define UUID_ADV_SCANNER_SERVICE   0xef, 0x48, 0xa2, 0x32, 0x17, 0xc6, 0xa6, 0xbc, 0xfa, 0x44, 0x54, 0x7c, 0x0d, 0x90, 0x03, 0xdc

/* UUID value of the Adv Scanner Data Characteristic */
#define UUID_ADV_SCANNER_DATA      0xc5, 0x42, 0x45, 0x3b, 0xd0, 0x74, 0x5b, 0x81, 0xf6, 0x4a, 0x26, 0x8f, 0xa5, 0xcf, 0x7a, 0xb7

/******************************************************************************
 *                          Type  Definitions
 ******************************************************************************/
/* Adv Scanner application info */
typedef struct
{
    uint32_t  app_timer_count;  // App timer count
    uint32_t  adv_count;        // Received advertisement count
    uint64_t  init_tick;
} adv_scanner_app_t;

/******************************************************************************
 *                            Variables Definitions
 ******************************************************************************/
/*
 * This is the GATT database for the Adv Scanner application. The database
 * defines services, characteristics and descriptors supported by the application.
 * Each attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by the
 * peer to access attributes, and can be used locally by application, for example
 * to retrieve data written by the peer.  Definition of characteristics and
 * descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if peer application is allowed to read or write
 * into it. Handles do not need to be sequential, but need to be in order.
 */
const uint8_t adv_scanner_gatt_database[]=
{
    // Handle 0x01: GATT service
    PRIMARY_SERVICE_UUID16(HANDLE_ADV_SCANNER_GATT_SERVICE, UUID_SERVICE_GATT),

    // Handle 0x14: GAP service
    PRIMARY_SERVICE_UUID16(HANDLE_ADV_SCANNER_GAP_SERVICE, UUID_SERVICE_GAP),

        CHARACTERISTIC_UUID16(HANDLE_ADV_SCANNER_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_ADV_SCANNER_GAP_SERVICE_CHAR_DEV_NAME_VAL,
             UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

        // Handle 0x17: characteristic Appearance, handle 0x18 characteristic value.
        // List of approved appearances is available at bluetooth.org.  Current
        // value is set to 0x200 - Generic Tag
        CHARACTERISTIC_UUID16(HANDLE_ADV_SCANNER_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_ADV_SCANNER_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
             UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

    // Handle 0x28: Adv Scanner Service.
    // This is the main proprietary service of Adv Scanner application.  It has
    // a single characteristic which allows peer to write to and can be configured
    // to send indications to the peer.  Note that UUID of the vendor specific
    // service is 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.
    // _UUID128 version of the macro should be used.
    PRIMARY_SERVICE_UUID128(HANDLE_ADV_SCANNER_SERVICE, UUID_ADV_SCANNER_SERVICE),

        // Handle 0x29: characteristic Adv Scanner Notification, handle 0x2a characteristic value
        // we support both notification and indication.  Peer need to allow notifications
        // or indications by writing in the Characteristic Client Configuration Descriptor
        // (see handle 2b below).  Note that UUID of the vendor specific characteristic is
        // 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.  _UUID128 version
        // of the macro should be used.
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_ADV_SCANNER_SERVICE_CHAR_NOTIFY, HANDLE_ADV_SCANNER_SERVICE_CHAR_NOTIFY_VAL,
             UUID_ADV_SCANNER_DATA, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE |
             LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
             LEGATTDB_PERM_READABLE  | LEGATTDB_PERM_WRITE_CMD  | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_VARIABLE_LENGTH),

            // Handle 0x2b: Characteristic Client Configuration Descriptor.
            // This is standard GATT characteristic descriptor.  2 byte value 0 means that
            // message to the client is disabled.  Peer can write value 1 or 2 to enable
            // notifications or indications respectively.  Not _WRITABLE in the macro.  This
            // means that attribute can be written by the peer.
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_ADV_SCANNER_SERVICE_CHAR_CFG_DESC,
                 UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE),

    // Handle 0x40: Device Info service
    // Device Information service helps peer to identify manufacture or vendor
    // of the device.  It is required for some types of the devices (for example HID,
    // and medical, and optional for others.  There are a bunch of characteristics
    // available, out of which Hello Sensor implements 3.
    PRIMARY_SERVICE_UUID16(HANDLE_ADV_SCANNER_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION),

        // Handle 0x41: characteristic Manufacturer Name, handle 0x42 characteristic value
        CHARACTERISTIC_UUID16(HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
             UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

        // Handle 0x43: characteristic Model Number, handle 0x4 characteristic value
        CHARACTERISTIC_UUID16(HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
             UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

        // Handle 0x45: characteristic System ID, handle 0x46 characteristic value
        CHARACTERISTIC_UUID16(HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_ADV_SCANNER_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
             UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),
};

/* Holds the Adv Scanner app info */
adv_scanner_app_t app_state;

char    adv_scanner_local_name[]           = "Adv Scanner";
uint8_t adv_scanner_appearance[2]          = {BIT16_TO_8(APPEARANCE_GENERIC_TAG)};
uint8_t adv_scanner_notify_value[]         = "Adv Scanner";
char    adv_scanner_char_mfr_name_value[]  = {'C', 'y', 'p', 'r', 'e', 's', 's', 0};
char    adv_scanner_char_model_num_value[] = {'4', '3', '2', '1', 0, 0, 0, 0};
uint8_t adv_scanner_char_system_id_value[] = {0xef, 0x48, 0xa2, 0x32, 0x17, 0xc6, 0xa6, 0xbc};

/* transport configuration */
const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    {WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD},
    {0, 0},
    NULL,
    NULL,
    NULL
};

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
static void            adv_scanner_app_init(void);
static wiced_result_t  adv_scanner_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void            adv_scanner_interrupt_handler(void *user_data, uint8_t value);
static void            adv_scanner_app_timer(uint32_t arg);
static void            adv_report(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);
extern UINT64          clock_SystemTimeMicroseconds64(void);

static uint64_t adv_scanner_get_tick_count(void)
{
    return clock_SystemTimeMicroseconds64() / 1000 - app_state.init_tick;
}


/* Adv Scanner application starts, ie, entry point to the application.
 * It is mandatory for all the applications to define this
 */
void application_start(void)
{
    wiced_transport_init(&transport_cfg);

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    wiced_hal_puart_select_uart_pads(WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
    wiced_hal_puart_set_baudrate(921600);

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_HCI_UART);

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must 
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    // WICED BT Stack initialization and registering the managment callback
    wiced_bt_stack_init(adv_scanner_management_cback,
                        &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}


/*
 * Adv Scanner bt/ble device and link management callbacks
 */
static wiced_result_t adv_scanner_management_cback(wiced_bt_management_evt_t event,  wiced_bt_management_evt_data_t *p_event_data)
{
    WICED_BT_TRACE("adv_scanner_management_cback: %x\n", event);
    switch (event) {
    case BTM_ENABLED_EVT:  // Bluetooth  stack enabled
        adv_scanner_app_init();
        break;
    case BTM_DISABLED_EVT:
        break;
    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        WICED_BT_TRACE("Scan State Change: %d\n", p_event_data->ble_scan_state_changed);
        break;
    default:
        break;
    }
    return WICED_BT_SUCCESS;
}

/*
 * WICED BT Init Complete.  This function is called when device initialization
 * has been completed.  Perform the App Initializations & Callback Registrations
 */
static void adv_scanner_app_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_bt_dev_status_t  status;

    WICED_BT_TRACE("\n##############\nAdv Scanner\n##############\n\n");

    memset(&app_state, 0, sizeof(app_state));

    // Configure buttons available on the platform
    wiced_hal_gpio_register_pin_for_interrupt(ADV_SCANNER_GPIO_PIN_BUTTON, adv_scanner_interrupt_handler, NULL);
    wiced_hal_gpio_configure_pin(ADV_SCANNER_GPIO_PIN_BUTTON, ADV_SCANNER_GPIO_BUTTON_SETTINGS, GPIO_PIN_OUTPUT_LOW);

    // GATT DB Initialization
    gatt_status = wiced_bt_gatt_db_init(adv_scanner_gatt_database, sizeof(adv_scanner_gatt_database));
    WICED_BT_TRACE("wiced_bt_gatt_db_init: status = %d\n", gatt_status);

    // Starting the app timers, seconds timer and milliseconds timer
    wiced_bt_app_start_timer(APP_TIMER_IN_SECONDS, 0, adv_scanner_app_timer, NULL);

    status = wiced_bt_ble_observe(WICED_TRUE, 0, adv_report);
    WICED_BT_TRACE("%s: observe status = %d\n", __func__, status);

    app_state.init_tick = clock_SystemTimeMicroseconds64() / 1000;
}

/* The function invoked on timeout of app seconds timer. */
void adv_scanner_app_timer(uint32_t arg)
{
    if ((app_state.app_timer_count & 255) == 0)
    {
        WICED_BT_TRACE("\n%s: %d\n\n", __func__, app_state.app_timer_count);
    }
    app_state.app_timer_count++;
}

/* This function is invoked on button interrupt events */
void adv_scanner_interrupt_handler(void* user_data, uint8_t value)
{
    static uint32_t button_pushed_time = 0;

    WICED_BT_TRACE("adv_scanner_interrupt_handler, app timer :%d\n", app_state.app_timer_count);

    if (wiced_hal_gpio_get_pin_input_status(WICED_GPIO_PIN_BUTTON) == SCAN_BUTTON_PRESSED)
    {
        WICED_BT_TRACE("Button pressed\n");
        button_pushed_time = app_state.app_timer_count;
    }
    else if (button_pushed_time != 0)
    {
        WICED_BT_TRACE("Button released\n");
    }
}

static void uint8_to_str(char* buffer, uint8_t val)
{
    uint8_t dig;
    buffer[0] = ' ';

    dig = val >> 4;
    if (dig < 10)
    {
        buffer[1] = '0' + dig;
    }
    else
    {
        buffer[1] = 'A' + (dig - 10);
    }

    dig = val & 0x0F;
    if (dig < 10)
    {
        buffer[2] = '0' + dig;
    }
    else
    {
        buffer[2] = 'A' + (dig - 10);
    }
}


/*
 * This function handles the scan results
 */
static void adv_report(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    static uint32_t count = 0;

    uint16_t       len;
    wiced_bool_t   dev_found;
    uint32_t       time;
    uint32_t       sec;
    uint32_t       ms;
    int            i;
    char           str[96];
    char*          p = str;
    char           txt[50];

    if (p_scan_result == NULL)
    {
        WICED_BT_TRACE("%s: Scan completed:\n", __func__);
        return;
    }

    if (filter_dev_num == 0)
    {
        dev_found = WICED_TRUE;
    }
    else
    {
        dev_found = WICED_FALSE;
        for (i = 0; i < filter_dev_num && dev_found == WICED_FALSE; i++)
        {
            if (memcmp(p_scan_result->remote_bd_addr, filter_dev_list[i], 6) == 0)
            {
                dev_found = WICED_TRUE;
            }
        }
    }

    if (!dev_found)
    {
        return;
    }

    len = p_adv_data[0] + 1;
    while (p_adv_data[len] != 0 && (len + p_adv_data[len] + 1) < 32)
    {
        len += p_adv_data[len] + 1;
    }

    if (len < 3)
    {
        WICED_BT_TRACE("%s: Adv len = %d\n", __func__, len);
        return;
    }

#ifdef EXCLUDE_BEACONS
    if (p_adv_data[1] == BTM_BLE_ADVERT_TYPE_MESH_BEACON)
    {
        return;
    }

    if (len == 20 &&
        p_adv_data[1] == BTM_BLE_ADVERT_TYPE_FLAG &&
        p_adv_data[4] == BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE &&
        p_adv_data[7] == BTM_BLE_ADVERT_TYPE_SERVICE_DATA)
    {
        // Ignore 16-byte UUID service adv too whihc is likely provisioning or proxy service
        return;
    }
#endif

    time = (uint32_t)adv_scanner_get_tick_count();
    sec  = (time / 1000) % 1000;
    ms   = time % 1000;

    *p++ = ' ';
    for (i = 0; i < len; i++)
    {
        uint8_to_str(p, p_adv_data[i]);
        p += 3;
    }
    *p = '\0';

    count++;

    switch(p_scan_result->ble_evt_type)
    {
    case BTM_BLE_EVT_CONNECTABLE_ADVERTISEMENT:
        strcpy(txt, "Connectable Undirected");
        break;
    case BTM_BLE_EVT_CONNECTABLE_DIRECTED_ADVERTISEMENT:
        strcpy(txt, "Connectable Directed");
        break;
    case BTM_BLE_EVT_SCANNABLE_ADVERTISEMENT:
        strcpy(txt, "Scannable advertisement");
        break;
    case BTM_BLE_EVT_NON_CONNECTABLE_ADVERTISEMENT:
        strcpy(txt, "Non connectable Undirected");
        break;
    case BTM_BLE_EVT_SCAN_RSP:
        strcpy(txt, "Scan response");
        break;
    default:
        strcpy(txt, "Unknown");
    };
    
    WICED_BT_TRACE("\nTime (%03d.%03d), Adv (%d), BDA (%B), len = %d,\n%s\n",
                   sec, ms, count, p_scan_result->remote_bd_addr, len, str);
    WICED_BT_TRACE("Adress type: %s, event type %s, flag %d, RSSI %d", 
        p_scan_result->ble_addr_type ? "Random" : "Public", 
        txt, p_scan_result->flag, p_scan_result->rssi);
}
