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
* wiced_mesh_client.h : Header file
*
*/
#ifndef WICED_MESH_CLIENT__H
#define WICED_MESH_CLIENT__H

#include <stdint.h>
#include "wiced.h"
#include "wiced_bt_mesh_models.h"

#define MESH_CLIENT_SUCCESS                     0
#define MESH_CLIENT_ERR_INVALID_STATE           1
#define MESH_CLIENT_ERR_NOT_CONNECTED           2
#define MESH_CLIENT_ERR_NOT_FOUND               3
#define MESH_CLIENT_ERR_NETWORK_CLOSED          4
#define MESH_CLIENT_ERR_NO_MEMORY               5
#define MESH_CLIENT_ERR_METHOD_NOT_AVAIL        6
#define MESH_CLIENT_ERR_NETWORK_DB              7
#define MESH_CLIENT_ERR_INVALID_ARGS            8
#define MESH_CLIENT_ERR_DUPLICATE_NAME          9
#define MESH_CLIENT_ERR_PROCEDURE_NOT_COMPLETE  10

#define DEVICE_TYPE_UNKNOWN                 0
#define DEVICE_TYPE_GENERIC_ON_OFF_CLIENT   1
#define DEVICE_TYPE_GENERIC_LEVEL_CLIENT    2
#define DEVICE_TYPE_GENERIC_ON_OFF_SERVER   3
#define DEVICE_TYPE_GENERIC_LEVEL_SERVER    4
#define DEVICE_TYPE_LIGHT_DIMMABLE          5
#define DEVICE_TYPE_POWER_OUTLET            6
#define DEVICE_TYPE_LIGHT_HSL               7
#define DEVICE_TYPE_LIGHT_CTL               8
#define DEVICE_TYPE_LIGHT_XYL               9
#define DEVICE_TYPE_SENSOR_SERVER           10
#define DEVICE_TYPE_SENSOR_CLIENT           11
#define DEVICE_TYPE_VENDOR_SPECIFIC         12


#define MESH_ERROR_BAD_COMPOSITION_DATA     1
#define MESH_ERROR_NO_MEM                   2

#define DEFAULT_TRANSITION_TIME             0xFFFFFFFF

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Network opened callback is used to indicate back to the application that the local device and stack have been initialized and ready to use.
 */
typedef void(*mesh_client_network_opened_t)(uint8_t status);

/*
 * Unprovisioned device callback is used by the mesh library to indicate an unprovisioned device found during the 
 * search for unprovisioned devices.  The application can use UUUD of the device to start the provisioning
 * process.
 */
typedef void(*mesh_client_unprovisioned_device_t)(uint8_t *uuid, uint16_t oob, uint8_t *name, uint8_t name_len);

/*
 * Provisioned status callback is used by the mesh library to indicate the status of the provisioning process.
 * After the provisioning has been completed successfully, the application can use 
 * the mesh_client_get_device_components to find the list of the components of the device.
 * For example, a single device may be connected to a light and a fan. The app may use the 
 * mesh_client_get_target_methods function to find out how the comeonent can be 
 * controlled. For example the device can be controlled using on/off or hsl clients.  
 * The application may also call mesh_client_get_control_methods function to find
 * if the new device can be a client for any of the scenarios. For example, if the device
 * is a switch it will report on/off as a control methods.
 */
#define MESH_CLIENT_PROVISION_STATUS_FAILED         0
#define MESH_CLIENT_PROVISION_STATUS_SCANNING       1
#define MESH_CLIENT_PROVISION_STATUS_CONNECTING     2
#define MESH_CLIENT_PROVISION_STATUS_PROVISIONING   3
#define MESH_CLIENT_PROVISION_STATUS_CONFIGURING    4
#define MESH_CLIENT_PROVISION_STATUS_SUCCESS        5
#define MESH_CLIENT_PROVISION_STATUS_END            6
typedef void(*mesh_client_provision_status_t)(uint8_t status, uint8_t *p_uuid);

typedef void(*mesh_client_connect_status_t)(uint8_t is_connected, uint32_t conn_id, uint16_t addr, uint8_t is_over_gatt);

#define MESH_CLIENT_NODE_WARNING_UNREACHABLE        0
#define MESH_CLIENT_NODE_CONNECTED                  1
#define MESH_CLIENT_NODE_ERROR_UNREACHABLE          2
typedef void(*mesh_client_node_connect_status_t)(uint8_t status, char *p_name);

#define DEVICE_RESET_STATUS_SUCCESS         0
#define DEVICE_RESET_STATUS_NOT_FOUND       1
#define DEVICE_RESET_STATUS_NOT_REACHABLE   2
typedef void(*mesh_client_node_reset_status_t)(uint8_t status, char *device_name);

typedef void(*mesh_client_database_changed_t)(char *mesh_name);

/*
 * Check if a network with specified name already exists.
 */
int mesh_client_network_exists(char *mesh_name);

/*
 * Create new network
 */
int mesh_client_network_create(const char *provisioner_name, const char *provisioner_uuid, char *mesh_name);

/*
 * Delete network database
 */
int mesh_client_network_delete(const char *provisioner_name, const char *provisioner_uuid, char *mesh_name);

/*
 * open existing network
 */
int mesh_client_network_open(const char *provisioner_name, const char *provisioner_uuid, char *mesh_name, mesh_client_network_opened_t p_opened_callback);

/*
 * create a network or update existing network.  The whole database is passed as a string.  
 * Import is not possible while a device is being provisioned or reconfigured.
 * Returns the name of the database opened, or NULL if import is not successfull.
 */
char* mesh_client_network_import(const char *provisioner_name, const char *provisioner_uuid, char *json_string, mesh_client_network_opened_t p_opened_callback);

/*
 * Export existing network.  The whole database is returned as a string. The caller is responsible to free the string.
 * Export is not possible while a device is being provisioned or reconfigured.
 */
char *mesh_client_network_export(char *mesh_name);

/*
 * Close current network
 */
void mesh_client_network_close(void);

/*
 * Create a new group in the parent group.
 * Devices in a mesh network can be organized in groups. If a controlling device (for example, a light switch) is a part of the group, it
 * sends on/off commands to a grou. If target device (for example, a bulb is a part of a group, it processes the commands addressed to
 * the group, as well as commands addressed to the specific device.  
 *
 * The group is empty when it is created. A new device (both control and a target type) can be provisioned to be a part of the group. 
 * Devices and components can be moved in and out of the group.
 */
int mesh_client_group_create(char *group_name, char *parent_group_name);

/*
 * Delete a group with the specified name
 */
int mesh_client_group_delete(char *group_name);

/*
 * Get all networks. 
 * The function allocates a buffer and return a concatenation of all network names. Caller is responsible for freeing the buffer.
 */
char *mesh_client_get_all_networks(void);

/*
 * Get all groups. 
 * The function allocates a buffer and return a concatenation of all group names in the current database. Caller is responsible for freeing the buffer.
 */
char *mesh_client_get_all_groups(char *in_group);

/*
 * Get all provisioners. 
 * The function allocates a buffer and return a concatenation of all provisioner names in the current database. Caller is responsible for freeing the buffer.
 */
char *mesh_client_get_all_provisioners(void);

/*
 * Get All Components of the Device. 
 * The function allocates a buffer and returns a 0 terminated array of names of components of the device. 
 * The function is typically called at the end of the provisioning. Caller is responsible for freeing the buffer. 
 */
char *mesh_client_get_device_components(uint8_t *p_uuid);

/*
 * Get Target Methods of a component. 
 * The function allocates a buffer and returns a 0 terminated array of names of methods that can be used
 * to control the component. For example, if a component contains TARGET_METHOD_ONOFF, it will accept 
 * OnOff commands from the application.
 * The function is typically called at the end of the provisioning. Caller is responsible for freeing the buffer. 
 */
#define MESH_CONTROL_METHOD_ONOFF       "ONOFF"
#define MESH_CONTROL_METHOD_LEVEL       "LEVEL"
#define MESH_CONTROL_METHOD_LIGHTNESS   "LIGHTNESS"
#define MESH_CONTROL_METHOD_POWER       "POWER"
#define MESH_CONTROL_METHOD_HSL         "HSL"
#define MESH_CONTROL_METHOD_CTL         "CTL"
#define MESH_CONTROL_METHOD_XYL         "XYL"
#define MESH_CONTROL_METHOD_VENDOR      "VENDOR_"
#define MESH_CONTROL_METHOD_SENSOR      "SENSOR"

char *mesh_client_get_target_methods(const char *component_name);

/*
 * Return a concatenated list of methods that can be used the device can control
 */
char *mesh_client_get_control_methods(const char *component_name);

/*
 * Get all Components of a group. 
 * The function allocates a buffer and returns a 0 concatenated array of all component names which belong to a group in the current database. 
 * If p_group_name is NULL, the function returns names of all components in the database. Caller is responsible for freeing the buffer. 
 */
char *mesh_client_get_group_components(char *p_group_name);

/*
 * Get Type of the Component.
 * The function returns the type of the component with specified name.
 */
uint8_t mesh_client_get_component_type(char *component_name);

/*
 * Set New Name for a group or a Components.
 */
int mesh_client_rename(char *old_name, char *new_name);

/*
 * Move Component to Group.
 * The function sets the name of the component with the specified address.
 */
int mesh_client_move_component_to_group(const char *component_name, const char *from_group_name, const char *to_group_name);

/*
 * Configure Publication.
 * The function sets a device to publish commands or status information to a specific device, group of devices,
 * or one of predefined types including "all-nodes", "all-proxies", "all_friends", "all-relays", or "this-device".
 * The method should be one of the methods returned by mesh_client_get_target_methods or 
 * mesh_client_get_control_methods including "ONOFF", "LEVEL", "LIGHTNESS", "POWER", "HSL", "CTL", "XYL", "SENSOR", or 
 * "VENDOR_<company_id><model_id>", for example VENDOR_01310001.
 */
int mesh_client_configure_publication(const char *component_name, uint8_t is_command, const char *method, const char *target_name, int publish_period);

const char *mesh_client_get_publication_target(const char *component_name, uint8_t is_client, const char *method);

int mesh_client_get_publication_period(const char *component_name, uint8_t is_client, const char *method);

/*
 * Set Device Configuration.
 * The function sets up configuration for the new devices, or reconfigures existing device
 * If the device_name parameter is NULL, the configuration parameters apply to the devices that a newly provisioned.
 */
int mesh_client_set_device_config(const char *device_name, int is_gatt_proxy, int is_friend, int is_relay, int send_net_beacon, int relay_xmit_count, int relay_xmit_interval, int default_ttl, int net_xmit_count, int net_xmit_interval);

/*
 * Set Publication Configuration.
 * The function sets up publication configuration for the new devices, or reconfigures existing device.
 * If the device_name parameter is NULL, the configuration parameters apply to the devices that a newly provisioned.  In this case publication
 * information is ued for the main function of the device.  For example if a dimmer is being provisioned, the publication will be configured
 * for the Generic Level Client model of the device.
 * If the device_name parameter is not NULL, the application can select which it applies to..
 */
int mesh_client_set_publication_config(int publish_credential_flag, int publish_retransmit_count, int publish_retransmit_interval, int publish_ttl);

/*
 * Scan unprovisioned devices. 
 */
int mesh_client_scan_unprovisioned(int start);

/*
 * Start provisioning of the device with specified UUID. 
 */
uint8_t mesh_client_provision(const char *device_name, const char *group_name, uint8_t *uuid, uint8_t identify_duration);

/*
 * Perform factory reset of the device. 
 */
uint8_t mesh_client_reset_device(char *component_name);

/*
 * Connect to currently opened network. 
 * Scan timeout indicates how long the library should scan for a proxy device.
 */
uint8_t mesh_client_connect_network(uint8_t use_gatt_proxy, uint8_t scan_duration);

/*
 * Disconnect currently connected network. 
 */
uint8_t mesh_client_disconnect_network(void);

/*
 * Connect to a specific node in the currently opened network. This is typically used 
 * to perform DFU over GATT.
 */
uint8_t mesh_client_connect_component(char *component_name, uint8_t use_proxy, uint8_t scan_duration);

/*
 * Application should notify when a connection has been established or dropped.
 * Connection up implies that that the GATT connection, client configuration descriptor set and MTU exchanged.
 * Zero conn_id indicates that connection has been dropped.
 */
void mesh_client_connection_state_changed(uint16_t conn_id, uint16_t mtu);

/*
 * Return TRUE if connecting to a provisiong service, FALSE if to proxy.
 */
uint8_t mesh_client_is_connecting_provisioning(void);

/*
 * Returns TRUE if GATT proxy connection is established. 
 */
uint8_t mesh_client_is_proxy_connected(void);

/*
 * This function should be called whenever IP connection is established with a proxy device
 */
void mesh_client_extern_proxy_connected(uint16_t addr);

/*
 * Packet received by the application from the proxy connection
 */
void mesh_client_proxy_data(uint8_t *p_data, uint16_t len);

/*
 * Packet received by the application during the provisioning over GATT connection
 */
void mesh_client_provisioning_data(uint8_t is_notification, uint8_t *p_data, uint16_t len);

/*
 * Component info callback is used to indicate the status of the get component info operation and return the retrieved information.
 */
typedef void(*mesh_client_component_info_status_t)(uint8_t status, char *component_name, char *component_info);

/*
 * Identify method sends a message to a device or a group of device to identify itself for a certain duration.
 */
int mesh_client_identify(const char *p_name, uint8_t duration);

/*
 * This function can be called to encrypt OTA FW upgrade commands and data. Note that output buffer should be at least 17 bytes larger
 * than input data length.  Function returns the size of the data to be transfered over the air using OTA_FW_UPDATE_COMMAND or _DATA handle
 */
uint16_t mesh_client_ota_data_encrypt(const char *component_name, const uint8_t *p_in_data, uint16_t in_data_len, uint8_t *p_out_buf, uint16_t out_buf_len);

/*
 * This function can be called to decrypt OTA FW upgrade event received from the peer device. Function returns the size of the data to be 
 * processed (typically 1 byte).
 */
uint16_t mesh_client_ota_data_decrypt(const char *component_name, const uint8_t *p_in_data, uint16_t in_data_len, uint8_t *p_out_buf, uint16_t out_buf_len);

/*
 * The function can be called to read CID/PID/VID and FW version information of the device.
 */
uint8_t mesh_client_get_component_info(char *component_name, mesh_client_component_info_status_t p_component_info_status_callback);

/*
 * The function can be called to start DFU process
 */
#define DFU_METHOD_PROXY_TO_ALL                     0
#define DFU_METHOD_PROXY_TO_DEVICE                  1
#define DFU_METHOD_APP_TO_ALL                       2
#define DFU_METHOD_APP_TO_DEVICE                    3

int mesh_client_dfu_start(uint8_t dfu_method, char *component_name, uint16_t company_id, uint8_t *fw_id, uint8_t fw_id_len);

/*
 * The function can be called to stop DFU process
 */
int mesh_client_dfu_stop(void);

/*
 * Component info callback is used to indicate the status of the get component info operation and return the retrieved information.
 */
typedef void(*mesh_client_dfu_status_t)(uint8_t status, int cur_block_number, int total_blocks);

/*
* The function can be called to get status of a DFU process
*/
int mesh_client_dfu_get_status(char *distributor_name, uint16_t company_id, uint8_t *fw_id, uint8_t fw_id_len, mesh_client_dfu_status_t p_dfu_status_callback);

/*
 * On/Off state callback is executed as a result of the Get/Set operation or when state of the device is changed locally 
 */
typedef void (*mesh_client_on_off_status_t)(const char *device_name, uint8_t target, uint8_t present, uint32_t remaining_time);

/*
 * Get On/Off state of a device or all devices in a group
 */
int mesh_client_on_off_get(const char *device_name);

/*
 * Set On/Off state of a device or a group
 */
int mesh_client_on_off_set(const char *device_name, uint8_t on_off, wiced_bool_t reliable, uint32_t transition_time, uint16_t delay);

/*
 * Dimmable light state callback is executed as a result of the Get/Set operation or when state of the light is changed locally 
 */
typedef void(*mesh_client_level_status_t)(const char *device_name, int16_t target, int16_t present, uint32_t remaining_time);

/*
 * Get current level of the device
 */
int mesh_client_level_get(const char *device_name);

/*
 * Set state level of the device
 */
int mesh_client_level_set(const char *device_name, int16_t level, wiced_bool_t reliable, uint32_t transition_time, uint16_t delay);

/*
 * Lightness state callback is executed as a result of the Get/Set Lightness operation or when state of the lightness is changed locally 
 */
typedef void(*mesh_client_lightness_status_t)(const char *device_name, uint16_t target, uint16_t present, uint32_t remaining_time);

/*
 * Get current Status of Dimmable light
 */
int mesh_client_lightness_get(const char *device_name);

/*
 * Set state of Dimmable light
 */
int mesh_client_lightness_set(const char *p_name, uint16_t lightness, wiced_bool_t reliable, uint32_t transition_time, uint16_t delay);


/*
 * HSL light state callback is executed as a result of the Get/Set operation or when state of the light is changed locally 
 */
typedef void(*mesh_client_hsl_status_t)(const char *device_name, uint16_t lightness, uint16_t hue, uint16_t saturation, uint32_t remaining_time);

/*
 * CTL light state callback is executed as a result of the Get/Set operation or when state of the light is changed locally 
 */
typedef void(*mesh_client_ctl_status_t)(const char *device_name, uint16_t present_lightness, uint16_t present_temperature, uint16_t target_lightness, uint16_t target_temperature, uint32_t remaining_time);

/*
 * Sensor state callback is executed as a result of the Get/Set operation or when state of the sensor is changed locally
 */
typedef void(*mesh_client_sensor_status_t)(const char *device_name, int property_id, uint8_t length, uint8_t *value);

/*
 * Get current state of HSL light
 */
int mesh_client_hsl_get(const char *device_name);

/*
 * Set state of HSL light
 */
int mesh_client_hsl_set(const char *device_name, uint16_t lightness, uint16_t hue, uint16_t saturation, wiced_bool_t reliable, uint32_t transition_time, uint16_t delay);

/*
 * Get current state of CTL light
 */
int mesh_client_ctl_get(const char *device_name);

/*
 * Set state of CTL light
 */
int mesh_client_ctl_set(const char *device_name, uint16_t lightness, uint16_t temperature, uint16_t delta_uv, wiced_bool_t reliable, uint32_t transition_time, uint16_t delay);

/*
 * Send vendor specific data
 */
int mesh_client_vendor_data_set(const char *device_name, uint8_t *buffer, uint16_t len);

/*
 * Return a concatenated list of groups to which the component belongs
 */
char *mesh_client_get_component_group_list(char *component_name);

/*
 * Add Component to Group.
 * The function sets the name of the component with the specified address of group.
 */
int mesh_client_add_component_to_group(const char *component_name, const char *group_name);

/*
 * Remove Component from Group.
 * The function removes the component with the specified name from the group.
 */
int mesh_client_remove_component_from_group(const char *component_name, const char *group_name);

/*
 * Sets the cadence values of the sensor.
 * The function checks if the values available in JSON are same as new values,
 * if different, sends the cadence set message to sensor
 */
int mesh_client_sensor_cadence_set(const char *device_name, int property_id,
                                   uint16_t fast_cadence_period_divisor, wiced_bool_t trigger_type,
                                   uint32_t trigger_delta_down, uint32_t trigger_delta_up,
                                   uint32_t min_interval, uint32_t fast_cadence_low,
                                   uint32_t fast_cadence_high);

int mesh_client_sensor_cadence_get(const char *device_name, int property_id,
    uint16_t *fast_cadence_period_divisor, wiced_bool_t *trigger_type,
    uint32_t *trigger_delta_down, uint32_t *trigger_delta_up,
    uint32_t *min_interval, uint32_t *fast_cadence_low,
    uint32_t *fast_cadence_high);

/*
 * Gets the setting property ids from the database for the specific sensor property id.
 * The function returns NULL if Sensor model is not found, or if Sensor with specified property ID is not 
 * found on the device.  Or the 0 terminated array of uint16_t with Settings Property IDs.
 */
int* mesh_client_sensor_setting_property_ids_get(const char *device_name, int property_id);

/*
 * Sets the sensor setting value for the specific sensor property id
 */
int mesh_client_sensor_setting_set(const char *device_name, int property_id, int setting_property_id, uint8_t *val);

/*
 * Gets sensor property id list
 */
int *mesh_client_sensor_property_list_get(const char *device_name);

/*
 * Gets the sensor value for the specific sensor property id
 */
int mesh_client_sensor_get(const char *device_name, int property_id);

/*
 * If start_listen is true, this function configures mesh library to receive and pass to the application messages that are sent to the group. 
 * For example. This method can be used to register to receive, for example, �SENSOR� messages sent in a specific group.
 * If the control_method parameter is NULL, the library will register to receive messages for all types of messages.  
 * Alternatively the control_message can be set to one of the predefined strings: ONOFF, LEVEL, POWER, HSL, CTL, XYL, VENDOR_XXXX or SENSOR.
 * If the group_name is NULL, the library will register to receive messages sent to all the groups. Otherwise the group_name shall be set to the name of the existing group.
 * If start_listen is false the library un-subscribes to receive messages from the specified group or all the groups.
 * If start_listen is true the library subscribes to receive messages from the specified group or all the groups.
 */
int mesh_client_listen_for_app_group_broadcasts(char *control_method, char *group_name, wiced_bool_t start_listen);

typedef struct
{
    mesh_client_unprovisioned_device_t unprovisioned_device_callback;
    mesh_client_provision_status_t provision_status_callback;
    mesh_client_connect_status_t connect_status_callback;
    mesh_client_node_connect_status_t node_connect_status_callback;
    mesh_client_database_changed_t database_changed_callback;
    mesh_client_on_off_status_t on_off_changed_callback;
    mesh_client_level_status_t level_changed_callback;
    mesh_client_lightness_status_t lightness_changed_callback;
    mesh_client_hsl_status_t hsl_changed_callback;
    mesh_client_ctl_status_t ctl_changed_callback;
    mesh_client_sensor_status_t sensor_changed_callback;
} mesh_client_init_t;

void mesh_client_init(mesh_client_init_t *p_callbacks);

#ifdef __cplusplus
}
#endif

#endif
