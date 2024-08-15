To build app for DF PTS tests do following steps:
1. Use the special build of the core library with DF support:
   Copy attached mesh_core_lib-d.a to the C:\Users\<user>\Documents\WICED-Studio-6.2\20719-B1_Bluetooth\WICED\libraries overwriting existing one.
2. Copy DF test app to the SDK:
   Copy attached folder mesh_direct_forward_test with two files inside to the C:\Users\<user>\Documents\WICED-Studio-6.2\common\apps\snip\mesh
3. Configure to use hardcoded node-UUID and BD-ADDRESS instead of random:
   Open file makefile.mk in the C:\Users\<user>\Documents\WICED-Studio-6.2\common\libraries\mesh_app_lib and uncomment following line:
C_FLAGS += -DPTS
4. Make sure you selected platform 20719-B1_Bluetooth in the menu "WICED Platform"
5. Create Make Target for DF test app with hardcoded node BD-ADDRESS and COM port:
   Right click in the Make Target window (right window) and select New and paste this string into Target Name and press OK.
   snip.mesh.mesh_direct_forward_test-CYW920719Q40EVB_01 BT_DEVICE_ADDRESS=207350000005 UART=COM3 download
   When you attach device to USB it creates two COM ports. For Example, COM3 and COM4. Then UART field in the above Make Target should be COM3,
   and you can use COM4 to collect device log in case you want to send it to us for issue investigation.
6. Build/download app:
   Double click on Make Target created on step 5
7. To enable friendship feature open file makefile.mk in the C:\Users\<user>\Documents\WICED-Studio-6.2\common\apps\snip\mesh\mesh_direct_forward_test
   change line
   C_FLAGS += -DMESH_FRIENDSHIP=0
   to
   C_FLAGS += -DMESH_FRIENDSHIP=1
   Build and download application
8. We can’t just initiate PTREQ - spec doesn’t have such procedure. But per spec “When a Path Originator creates a Forwarding Table entry the node may start a Path Tracing Delay timer”.
   To enable that set non-0 value (for example 10 seconds) to ptreq_delay_timer_sec:
   Open file mesh_direct_forward_test.c in the the C:\Users\<user>\Documents\WICED-Studio-6.2\common\apps\snip\mesh\mesh_direct_forward_test
   Find the place of the member ptreq_delay_timer_sec initialization and set 10
        .ptreq_delay_timer_sec = 10,                                // Value of Path Tracing Delay timer. Path originator start path tracing with that delay after path is established. Default value is 0 - no path tracing.
   

To reset device keeping it provisioned state press Reset button SW2 - middle button
For DF PTS tests actions do N short button press (shorter than 1 sec) and one long button press (longer than 1 sec).
Use button SW3 - right most button.
Where N can be:
0 - (meaning don't do short press) initialize discovery for unicast PD (1)
1 - initialize discovery for group PD (0xc110)
2 - initialize discovery for virtual PD (0x8110)
3 - factory reset - puts node to un-provisioned state
