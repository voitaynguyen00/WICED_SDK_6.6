-------------------------------------------------------------------------------
Sensor Hub app
-------------------------------------------------------------------------------

Overview
--------
This demo application shows a  implementation of a sensor hub.
The app is based on the snip/mesh/mesh_sensor_server sample which
implements BLE Mesh Sensor Server model.

Features demonstrated
 - Temperature measurement using the on board Thermistor on the EVK
 - Ambient light measurement using the on board sensor
 - Configuring and receiving interrupts from the PIR motion sensor
 - Publishing motion data over BLE mesh

See 20819_readme.txt for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application (to the WICED board)
2. Use Android MeshController or Windows Mesh Client and provision the sensor hub
3. After successful provisioning, user can use the Android MeshController/ Windows Mesh 
   Client to configure the below parameters of the sensor
     a> configure sensor to publish the sensor data to a group(all-nodes, all-relays).
     b> configure publish period : publish period defines how often the user wants the 
        sensor to publish the data.
     c> set cadence of the sensor :
        set minimum interval in which sensor data has to be published.
        set the range in which the fast cadence has to be observed.
        set the fast cadence period (how fast the data has to be published with respect 
        to publish period).
        set the unit in which if the values change the data should be published and 
        trigger type (Native or percentage).
            example : publish data if the data changes by 2 units/10%
4. To change the temperature on the thermistor, you can keep your finger on the onboard 
   sensor and see the changes.
5. Wave your hand in front of the CYBT-213043-MESH board to show some motion.
6. Keep your hand/ hover light over CYBT-213043-MESH board to change Ambient Light.

Notes
-----
The application GATT database is located in -
bt_20819A1-1.0\components\BT-SDK\common\libraries\mesh_app_lib\mesh_app_gatt.c
If you create a GATT database using Bluetooth Configurator, update the
GATT database in the location mentioned above.

Project Settings
----------------
Apllication specific project settings are as below -

MESH_MODELS_DEBUG_TRACES
	Turn on debug trace from Mesh Models library
MESH_CORE_DEBUG_TRACES
	Turn on debug trace from Mesh Core library
MESH_PROVISIONER_DEBUG_TRACES
	Turn on debug trace from Mesh Provisioner library
REMOTE_PROVISION_SRV
	Enable device as Remote Provisioning Server
LOW_POWER_NODE
	Enable device as Low Power Node

-------------------------------------------------------------------------------