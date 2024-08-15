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
 *  thermistor_app.c
 *
 *  @brief
 *
 *  Thermistor(Environment Sensing Profile) application
 *
 * This app implements BLE Environmental Sensing Service, temperature characteristic
 * During initialization the app registers with LE stack to receive various
 * notifications and connection status change. Paired device can read temperature value.
 *
 * Features demonstrated
 *  - Functionality of ADC for Temperature measurement
 *  - GATT database and Device configuration initialization
 *  - Registration with LE stack for various events
 *  - Sending data to the client by GATT notifications.
 *  - LED D2(WICED_GPIO_PIN_LED_2) will be turned ON when a client device is connected and turned OFF when a client device is disconnected
 *  - Bluetooth device address of the client device will be sent over PUART during connection and disconnection.
 *  - LED D11 blinks when data is sent through PUART
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED CYW920719Q40EVB_01 board to your computer.
 * 2. Build and download the application.(See Wiced Quick Start Guide)
 * 3. Use Terminal emulation tools like Teraterm or Putty to view the trace messages(See Kit User Guide).
 * 4. Pair from a client device(Android/iOS App that supports Environment Sensing Profile).
 * 5. Receive notification(Temperature Values) from client device.
 *
 */

/*******************************************************************************
 *                               Includes
 *******************************************************************************/

#include "thermistor_gatt_db.h"
#include "thermistor_temp_db.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_adc.h"
#include "wiced_platform.h"
#include "wiced_bt_stack.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif


/******************************************************************************
 *                                Constants
 ******************************************************************************/

#define BALANCE_RESISTANCE              100000              /* Value of Reference Resistance connected in series with the thermistor */
#define POLL_TIMER_IN_MS                5000               /* Milliseconds timer*/

/******************************************************************************
 *                                Variable/Structure/type Definitions
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;    /* Manages runtime configuration of Bluetooth stack*/

extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[]; /* Buffer for RF, HCI, ACL packets */

uint16_t thermistor_conn_id = 0;                                /* Manages connection IDs of client devices*/

/*******************************************************************
 *                              Function Declarations/Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t thermistor_management_callback(
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);

static void seconds_timer_temperature_cb(uint32_t arg);

static void thermistor_app_init(void);

static void thermistor_set_advertisement_data(void);

static wiced_bt_gatt_status_t thermistor_get_value(uint16_t attr_handle,
        uint16_t conn_id, uint8_t *p_val, uint16_t len, uint16_t *p_len);

static wiced_bt_gatt_status_t thermistor_set_value(uint16_t attr_handle,
        uint16_t conn_id, uint8_t *p_val, uint16_t len);

static int16_t get_temp_in_celsius(uint32_t vref, uint32_t vadc);

/* *******************************************************************
 *                              GATT Registration Callbacks
 * *******************************************************************/
static wiced_bt_gatt_status_t thermistor_write_handler(
        wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id);

static wiced_bt_gatt_status_t thermistor_read_handler(
        wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id);

static wiced_bt_gatt_status_t thermistor_connect_callback(
        wiced_bt_gatt_connection_status_t *p_conn_status);

static wiced_bt_gatt_status_t thermistor_server_callback(uint16_t conn_id,
        wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data);

static wiced_bt_gatt_status_t thermistor_event_handler(
        wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

/*******************************************************************
 *                              Function Definitions
 ******************************************************************/
/*
 *                      Entry point to the application. Set device configuration and start BT
 *                      stack initialization.  The actual application initialization will happen
 *                      when stack reports that BT device is ready
 */

/*
 Function name:
 application_start

 Function Description:
 @brief    Starting point of your application

 @param void

 @return void
 */
void application_start(void) {

#if defined WICED_BT_TRACE_ENABLE || defined TRACE_TO_WICED_HCI /*WICED_BT_TRACE_ENABLE*/
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#endif

    WICED_BT_TRACE( "\n\r--------------------------------------------------------- \r\n\n"
            "              Thermistor Sample Application \n\r\n\r"
            "---------------------------------------------------------\n\r"
            "This application measures voltage on the selected DC channel\r\n"
            "(A0/ADC_CHANNEL_P10) every 5 seconds(configurable) and displays\r\n"
            "the Thermistor's resistance values and temperature via PUART. \n\r"
            "---------------------------------------------------------\n\n\r" );
    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init(thermistor_management_callback, &wiced_app_cfg_settings,
            wiced_app_cfg_buf_pools);

}

/*
 Function Name:
 thermistor_management_callback

 Function Description:
 @brief  Callback function that will be invoked by application_start()

 @param  event           Bluetooth management event type
 @param  p_event_data    Pointer to the the bluetooth management event data

 @return        BT status of the callback function
 */
static wiced_bt_dev_status_t thermistor_management_callback(
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data) {

    wiced_bt_dev_status_t status = WICED_NOTUP;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t *p_ble_info = NULL;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    WICED_BT_TRACE("Received Event : %d\n\n\r", event);

    switch (event) {
    case BTM_ENABLED_EVT:
        /* Perform application-specific initialization */
        thermistor_app_init();
        break;

    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        WICED_BT_TRACE("Bluetooth Disabled\r\n");
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\r\n", *p_adv_mode);
        if (BTM_BLE_ADVERT_OFF == *p_adv_mode) {
            /*Advertisements stopped*/
            wiced_result_t result;
            result = wiced_bt_start_advertisements(
                    BTM_BLE_ADVERT_UNDIRECTED_LOW, 0,
                    NULL);
            WICED_BT_TRACE("wiced_bt_start_advertisements: %d\r\n", result);
        }
        break;

    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: %d\r\n", event);
        break;
    }

    return status;
}

/*
 Function name:
 seconds_timer_temperature_cb

 Function Description:
 @brief  This callback function is invoked on timeout of app. seconds timer.

 @param  arg

 @return void
 */
static void seconds_timer_temperature_cb(uint32_t arg) {
    volatile uint16_t voltage_val_adc_in_mv = 0;
    volatile uint16_t vddio_mv = 0;
    volatile int16_t temperature = 0;

    /*
     * Measure the voltage(in milli volts) on the channel being passed as an argument
     * To measure the voltage across the thermistor input channel - ADC_INPUT_P10
     * To measure vref - ADC_INPUT_VDDIO
     */
    /*The wiced_hal_adc automatically powers up ADC block before reading ADC voltage registers and powers down after reading which reduces power consumption. By multiple single shot sampling, it also improves the accuracy of the reading*/
    vddio_mv = wiced_hal_adc_read_voltage(ADC_INPUT_VDDIO); /* Input channel to measure Reference voltage for Voltage divider calculation for Thermistor */
    WICED_BT_TRACE("VDDIO_MV\t%d \r\n", vddio_mv);
    if(vddio_mv < 1850){                                    /*1.85V is used instead of 1.8V because of +/-3% inaccuracy */
           wiced_hal_adc_set_input_range(ADC_RANGE_0_1P8V);
           voltage_val_adc_in_mv = wiced_hal_adc_read_voltage(ADC_INPUT_P10); /* Input channel to measure DC voltage(temperature)-> GPIO 10 -> J12.1, J14.1 */
       }else{
           wiced_hal_adc_set_input_range(ADC_RANGE_0_3P6V);
           voltage_val_adc_in_mv = wiced_hal_adc_read_voltage(ADC_INPUT_P10); /* Input channel to measure DC voltage(temperature)-> GPIO 10 -> J12.1, J14.1 */
       }

    /* Hint: check that connection is up and client is registered to receive notifications for the below block of code */
    temperature = get_temp_in_celsius(vddio_mv, voltage_val_adc_in_mv);

    thermistor_last_reading[0] = (uint8_t) (temperature & 0xff);
    thermistor_last_reading[1] = (uint8_t) ((temperature >> 8) & 0xff);

    if (0 != thermistor_conn_id)
    {
        if(0 != (thermistor_client_configuration[0] & (GATT_CLIENT_CONFIG_NOTIFICATION != 0) ))
        {

            WICED_BT_TRACE("Device is connected and GATT Notification is enabled\r\n");
            wiced_bt_gatt_send_notification(thermistor_conn_id,
                    HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE_VALUE,
                    sizeof(thermistor_last_reading), thermistor_last_reading);

        }
        else
        {
            WICED_BT_TRACE("Device is connected and GATT Notification is not enabled\r\n");
        }
    }
    else
    {
        WICED_BT_TRACE("This device is not connected to any BLE central device\r\n");
    }
}

/*
 Function name:
 get_temp_in_celsius

 Function Description:
 @brief     This function takes in ADC output from VDDIO and voltage divider to calculate the temperature in celsius. The function returns temperature value with 2 decimal points of accuracy as expected by Environmental Sensing Profile.

 @param  vref    voltage in millivolts measured from the VDDIO
 @param  vadc    voltage in millivolts measured from the DC Channel

 @return    temperature in celsius multiplied by 100. This to provide 2 fractional positions of resolution.
 */
static int16_t get_temp_in_celsius(uint32_t vref, uint32_t vadc) {
    volatile int16_t temp_in_celsius = 0;
    volatile int32_t r_thermistor = 0;
    int8_t dec = 0;
    r_thermistor = ((vref - vadc) * BALANCE_RESISTANCE) / vadc;
    temp_in_celsius = r_t_look_up(r_t_centre, r_thermistor) * 100;
    dec = temp_in_celsius % 100;
    if (dec > 9) {
        WICED_BT_TRACE("Thermistor's Resistance(in ohms) %d Temperature(in celsius) : %d.%d \r\n\n",r_thermistor, temp_in_celsius / 100, dec );
    } else {
        WICED_BT_TRACE("Thermistor's Resistance(in ohms) %d Temperature(in celsius) : %d.0%d \r\n\n",r_thermistor, temp_in_celsius / 100, dec );
    }

    return temp_in_celsius;
}

/*
 Function Description:
 @brief    This function is executed if BTM_ENABLED_EVT event occurs  in thermistor management callback.

 @param    void

 @return    void
 */
static void thermistor_app_init(void) {

    /*ADC Initialization*/
    wiced_hal_adc_init();
    /* Starting the app 5 second timer */
    wiced_bt_app_start_timer(0, POLL_TIMER_IN_MS, NULL,
            seconds_timer_temperature_cb);

    /* Set Advertisement Data */
    thermistor_set_advertisement_data();

    /* Register with stack to receive GATT callback */
    wiced_bt_gatt_register(thermistor_event_handler);

    /* Initialize GATT Database */
    wiced_bt_gatt_db_init(gatt_database, gatt_database_len);

    /* Do not allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_FALSE, FALSE);

    /* Start Undirected LE Advertisements on device startup.*/
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL);

}

/*
 Function Name:
 thermistor_set_advertisement_data

 Function Description:
 @brief  Set Advertisement Data

 @param void

 @return void
 */
static void thermistor_set_advertisement_data(void) {

    wiced_bt_ble_advert_elem_t adv_elem[4];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG
            | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t env_sensing_uuid[2] = {BIT16_TO_8(UUID_SERVICE_ENVIRONMENTAL_SENSING)};
    uint8_t gatt_appearance[2] = {BIT16_TO_8(APPEARANCE_GENERIC_THERMOMETER)};

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof(uint8_t);
    adv_elem[num_elem].p_data = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    adv_elem[num_elem].len = sizeof(gatt_appearance);
    adv_elem[num_elem].p_data = (uint8_t *)gatt_appearance;
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    adv_elem[num_elem].len = sizeof(env_sensing_uuid);
    adv_elem[num_elem].p_data = (uint8_t *)env_sensing_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = strlen(
            (const char *) wiced_app_cfg_settings.device_name);
    adv_elem[num_elem].p_data = (uint8_t *) wiced_app_cfg_settings.device_name;
    num_elem++;

    if(WICED_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem)){
        WICED_BT_TRACE("Raw advertisement failed\r\n");
    }
}

/*
 Function Name:
 thermistor_event_handler

 Function Description:
 @brief  This Function handles the GATT connection events - GATT Event Handler

 @param event            BLE GATT event type
 @param p_event_data     Pointer to BLE GATT event data

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t thermistor_event_handler(
        wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data) {
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_connection_status_t *p_conn_status = NULL;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch (event) {
    case GATT_CONNECTION_STATUS_EVT:
        status = thermistor_connect_callback(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        status = thermistor_server_callback(p_attr_req->conn_id,
                p_attr_req->request_type, &p_attr_req->data);
        break;

    default:
        WICED_BT_TRACE("Other than GATT_CONNECTION_STATUS_EVT and GATT_ATTRIBUTE_REQUEST_EVT\r\n");
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}

/*
 Function Name:
 thermistor_connect_callback

 Function Description:
 @brief  The callback function is invoked when GATT_CONNECTION_STATUS_EVT occurs in GATT Event handler function

 @param p_conn_status     Pointer to BLE GATT connection status

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t thermistor_connect_callback(
        wiced_bt_gatt_connection_status_t *p_conn_status) {
    wiced_result_t gatt_status = WICED_BT_GATT_ERROR;

    if (p_conn_status->connected) {
        /* Device has connected */
        WICED_BT_TRACE("Connected : BDA '%B', Connection ID '%d'\r\n", p_conn_status->bd_addr, p_conn_status->conn_id);
        wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_LOW);
        thermistor_conn_id = p_conn_status->conn_id;
        gatt_status = wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, BLE_ADDR_PUBLIC, NULL);
    } else {
        /* Device has disconnected */
        WICED_BT_TRACE("Disconnected : BDA '%B', Connection ID '%d', Reason '%d'\r\n", p_conn_status->bd_addr, p_conn_status->conn_id, p_conn_status->reason);
        thermistor_conn_id = 0;
        wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_HIGH);
        gatt_status = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL);
    }

    return gatt_status;
}

/*
 Function Name:
 thermistor_server_callback

 Function Description:
 @brief  The callback function is invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs in GATT Event handler function. GATT Server Event Callback function.

 @param conn_id  Connection ID from GATT Connection event
 @param type     GATT Request type
 @param p_data   Pointer to BLE GATT request data

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t thermistor_server_callback(uint16_t conn_id,
        wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data) {
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch (type) {
    case GATTS_REQ_TYPE_READ:
        status = thermistor_read_handler(&p_data->read_req, conn_id);
        break;

    case GATTS_REQ_TYPE_WRITE:
        status = thermistor_write_handler(&p_data->write_req, conn_id);
        break;
    }

    return status;
}

/*
 Function Name:
 thermistor_write_handler

 Function Description:
 @brief  The function is invoked when GATTS_REQ_TYPE_WRITE is received from the client device and is invoked by GATT Server Event Callback function. This handles "Write Requests" received from Client device

 @param p_write_req   Pointer to BLE GATT write request
 @param conn_id  Connection ID from GATT Connection event

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t thermistor_write_handler(
        wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id) {
    /* Attempt to perform the Write Request */
    return thermistor_set_value(p_write_req->handle, conn_id,
            p_write_req->p_val, p_write_req->val_len);
}

/*
 Function Name:
 thermistor_read_handler

 Function Description:
 @brief  The function is invoked when GATTS_REQ_TYPE_READ is received from the client device and is invoked by GATT Server Event Callback function. This handles "Read Requests" received from Client device

 @param p_write_req   Pointer to BLE GATT read request
 @param conn_id  Connection ID from GATT Connection event

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t thermistor_read_handler(
        wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id) {
    /* Attempt to perform the Read Request */
    return thermistor_get_value(p_read_req->handle, conn_id, p_read_req->p_val,
            *p_read_req->p_val_len, p_read_req->p_val_len);
}

/*
 Function Name:
 thermistor_get_value

 Function Description:
 @brief  The function is invoked by thermistor_read_handler to get a Value from GATT DB.

 @param attr_handle  GATT attribute handle
 @param conn_id      Connection ID from GATT Connection event
 @param p_val        Pointer to BLE GATT read request value
 @param len      Maximum length of GATT read request
 @param p_len        Pointer to BLE GATT read request length

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t thermistor_get_value(uint16_t attr_handle,
        uint16_t conn_id, uint8_t *p_val, uint16_t len, uint16_t *p_len) {
    int i = 0;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (i = 0; i < thermistor_gatt_db_ext_attr_tbl_size; i++) {
        if (thermistor_gatt_db_ext_attr_tbl[i].handle == attr_handle) {
            /* Detected a matching handle in the external lookup table */
            if (thermistor_gatt_db_ext_attr_tbl[i].offset <= len) {
                /* Value fits within the supplied buffer; copy over the value */
                *p_len = thermistor_gatt_db_ext_attr_tbl[i].offset;
                memcpy(p_val, thermistor_gatt_db_ext_attr_tbl[i].p_data,
                        thermistor_gatt_db_ext_attr_tbl[i].offset);
                res = WICED_BT_GATT_SUCCESS;
            } else {
                /* Value to read will not fit within the buffer */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }
    return res;
}

/*
 Function Name:
 thermistor_set_value

 Function Description:
 @brief  The function is invoked by thermistor_write_handler to set a Value to GATT DB.

 @param attr_handle  GATT attribute handle
 @param conn_id      Connection ID from GATT Connection event
 @param p_val        Pointer to BLE GATT write request value
 @param len          length of GATT write request

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t thermistor_set_value(uint16_t attr_handle,
        uint16_t conn_id, uint8_t *p_val, uint16_t len) {
    int i = 0;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (i = 0; i < thermistor_gatt_db_ext_attr_tbl_size; i++) {
        if (thermistor_gatt_db_ext_attr_tbl[i].handle == attr_handle) {
            /* Verify that size constraints have been met */
            if (thermistor_gatt_db_ext_attr_tbl[i].len >= len) {
                /* Value fits within the supplied buffer; copy over the value */
                thermistor_gatt_db_ext_attr_tbl[i].offset = len;
                memcpy(thermistor_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;
            } else {
                /* Value to write does not meet size constraints */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }
    return res;
}

