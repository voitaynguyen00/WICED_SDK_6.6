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

#include "typedefs.h"
#include "bcmdevs.h"
#include "wiced_osl.h"
#include "wiced.h"
#include "wiced_utilities.h"
#include "wiced_management.h"
#include "wiced_resource.h"
#include "wiced_framework.h"
#include "wiced_filesystem.h"
#include "command_console.h"
#include "command_console_fs.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "platform_dct.h"
#include "platform_usb.h"
#include "ux_api.h"
#include "ux_dcd_bcm4390x.h"
#include "ux_device_stack.h"
#include "ux_device_class_storage.h"
#include "usb_device_storage.h"


/******************************************************
 *                      Macros
 ******************************************************/
#define USB_DEVICE_POOL_SIZE                (128*1024) //Use 80KB or above for application
#define USB_DEVICE_DMA_POOL_SIZE            (128*1024) //Use 80KB or above for application
#define USB_DEVICE_DMA_POOL_ALIGN           5

/*
 * RAM_DISK_USE_FILEX_DRIVEE=1: A local memory array is used as ramdisk and the FILEX ramdisk driver is used.
 *     In this mode, the ramdisk is not integrated with Wiced filesystem and only native FILEX API can be used to
 *     access the files in ramdisk. All the file commands in this APP don't work in this mode.
 * RAM_DISK_USE_FILEX_DRIVER=0: A Wiced block device must be used. It only works on board with DDR now. Wiced
 *     filesystem is integrated in this mode and the file commands can be used.
 */
#define RAM_DISK_USE_FILEX_DRIVER           1

/******************************************************
 *                    Constants
 ******************************************************/
/* Parameters to init Console Command */
#define MAX_LINE_LENGTH                     (128)
#define MAX_HISTORY_LENGTH                  (20)

/* Parameters to init USB Device. Usually keep using default values, adjustable for user scenario */
#define USB_DEVICE_THREAD_STACK_SIZE        (4*1024)
#define MAX_DEVICE_CLASS_DRIVER             (3)

/* Parameters for USB Device Descriptors */
#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED  (50)
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED  (60)
#define STRING_FRAMEWORK_LENGTH             (38)
#define LANGUAGE_ID_FRAMEWORK_LENGTH        (2)

/* Default sector size */
#define DEFAULT_SECTOR_SIZE                 (512)

#define DEFAULE_LOGICAL_BLOCK_SIZE          DEFAULT_SECTOR_SIZE

#define DISK_MBR_SECTOR                     0

/* RAM disk storage size for test */
#define RAM_DISK_STORAGE_SIZE               (512*1024) //(1024*1024)

#define RAM_DISK_DEVICE_NAME                "DDR"

/* RAM disk storage name for filesystem mount */
#define RAM_DISK_STORAGE_MOUNT_NAME         "UsbDeviceRamDisk"

#define FS_MEDIA_BUFFER_SIZE                (2*1024) //(2048)


/* USB Device console command sets */
#define USB_DEVICE_COMMANDS \
    { (char*) "disk_start",            usb_device_storage_run,         0, NULL, NULL, (char*) "",  (char*) "Start USB mass storage device" },



/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    void        *memory_pool;
    uint32_t    memory_pool_size;
    void        *dma_memory_pool;
    uint32_t    dma_memory_pool_size;
} wiced_usb_device_memory_t;

/******************************************************
 *               Variable Definitions
 ******************************************************/
/* USB Device command sets  */
static char line_buffer[ MAX_LINE_LENGTH ];
static char history_buffer_storage[ MAX_LINE_LENGTH * MAX_HISTORY_LENGTH ];

static const command_t commands[] =
{
    USB_DEVICE_COMMANDS
    FS_COMMANDS
    CMD_TABLE_END
};

/* USB Device initialization  */
wiced_usb_device_memory_t               usb20d_memcfg = { 0 };

platform_usb_device_dci_resource_t      usb20d_dci_info[ USB_DEVICE_CONTROLLER_INTERFACE_MAX ];
uint32_t                                usb20d_dci_num = 0;

static wiced_bool_t                     usb20d_init_completed = WICED_FALSE;

/* USB Storage Device class  */
static UX_SLAVE_CLASS_STORAGE_PARAMETER storage_parameter = { 0 };

#if (RAM_DISK_USE_FILEX_DRIVER)

static FX_MEDIA                         ram_disk = { 0 };
static CHAR                             ram_disk_memory[ RAM_DISK_STORAGE_SIZE ] = { 0 };
static UCHAR                            buffer[FS_MEDIA_BUFFER_SIZE] = { 0 }; //1024

extern VOID _fx_ram_driver( FX_MEDIA *media_ptr );
#else
static FX_MEDIA                         *ram_disk_media = NULL;
static wiced_filesystem_t               ram_disk_fs_handle = { 0 };
static filesystem_list_t                *ram_disk_filesystem = NULL;
#endif


/*
 *
 * [USB Descriptors]
 * http://www.beyondlogic.org/usbnutshell/usb5.shtml#DeviceDescriptors
 *
 *
 */
unsigned char device_framework_full_speed[] = {

    /* Device descriptor */
    0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08,
    0xb4, 0x04, 0x16, 0x46, 0x00, 0x00, 0x01, 0x02,
    0x03, 0x01,

    /* Configuration descriptor */
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xc0,
    0x32,

    /* Interface descriptor */
    0x09, 0x04, 0x00, 0x00, 0x02, 0x08, 0x06, 0x50,
    0x00,

    /* Endpoint descriptor (Bulk Out) */
    0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,

    /* Endpoint descriptor (Bulk In) */
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00

};

unsigned char device_framework_high_speed[] = {

    /* Device descriptor */
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0xb4, 0x04, 0x16, 0x46, 0x01, 0x00, 0x01, 0x02,
    0x03, 0x01,

    /* Device qualifier descriptor */
    0x0a, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0x01, 0x00,

    /* Configuration descriptor */
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xc0,
    0x32,

    /* Interface descriptor */
    0x09, 0x04, 0x00, 0x00, 0x02, 0x08, 0x06, 0x50,
    0x00,

    /* Endpoint descriptor (Bulk Out) */
    0x07, 0x05, 0x01, 0x02, 0x00, 0x02, 0x00, //512
    //0x07, 0x05, 0x01, 0x02, 0x00, 0x01, 0x00, //256
    //0x07, 0x05, 0x01, 0x02, 0x40, 0x02, 0x00, //576
    //0x07, 0x05, 0x01, 0x02, 0xf4, 0x01, 0x00, //500

    /* Endpoint descriptor (Bulk In) */
    0x07, 0x05, 0x82, 0x02, 0x00, 0x02, 0x00 //512
    //0x07, 0x05, 0x82, 0x02, 0x00, 0x01, 0x00 //256
    //0x07, 0x05, 0x82, 0x02, 0x40, 0x02, 0x00 //576
    //0x07, 0x05, 0x82, 0x02, 0xf4, 0x01, 0x00 //500

};

unsigned char string_framework[] = {

    /* Manufacturer string descriptor : Index 1 */
    0x09, 0x04, 0x01, 0x0c,
    0x45, 0x78, 0x70, 0x72,0x65, 0x73, 0x20, 0x4c,
    0x6f, 0x67, 0x69, 0x63,

    /* Product string descriptor : Index 2 */
    0x09, 0x04, 0x02, 0x0a,
    0x46, 0x6c, 0x61, 0x73, 0x68, 0x20, 0x44, 0x69,
    0x73, 0x6b,

    /* Serial Number string descriptor : Index 3 */
    0x09, 0x04, 0x03, 0x04,
    0x30, 0x30, 0x30, 0x31
};


    /* Multiple languages are supported on the device, to add
       a language besides english, the unicode language code must
       be appended to the language_id_framework array and the length
       adjusted accordingly. */
unsigned char language_id_framework[] = {

    /* English. */
    0x09, 0x04
};

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t wiced_usb_device_storage_init( void );

static void wiced_usb_device_usbx_isr( void );
static UINT wiced_usb_device_usbx_evt_callback( ULONG event );

static wiced_result_t ram_disk_init( void );

static UINT storage_media_read( VOID *storage, ULONG lun, UCHAR * data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status );
static UINT storage_media_write( VOID *storage, ULONG lun, UCHAR * data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status );
static UINT storage_media_status( VOID *storage, ULONG lun, ULONG media_id, ULONG *media_status );

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    UINT status;

    UNUSED_PARAMETER( status );

    printf( "\n***USB20 Device mass storage application!\n" );

    /* Initialize the device */
    wiced_init();

    /* Init Filesystem */
    status = wiced_filesystem_init();
    if(status != WICED_SUCCESS)
    {
        printf( "Filesystem init failed. status=%d\n", status );
        return;
    }

    /* Run the main application function  */
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( commands );
}


int usb_device_storage_run( int argc, char* argv[] )
{
    UINT status;

    printf( "START: USB20D Storage device connect\n" );

    /* Init RAM Disk storage with filesystem */
    status = ram_disk_init();
    if( status != WICED_SUCCESS )
    {
        printf( "RAM Disk filesystem init failed. status=%d\n", status );
        return ERR_UNKNOWN;
    }

    /* Init USB Storage Device SW & USB20 Device HW */
    status = wiced_usb_device_storage_init();
    if( status != WICED_SUCCESS )
    {
        printf( "USB Storage Device init failed. status=%d\n", status );
        return ERR_UNKNOWN;
    }

    printf( "USB Storage Device init ok!\n" );
    return ERR_CMD_OK;
}

static wiced_result_t wiced_usb_device_storage_init( void )
{
    uint pool_size;
    dmaaddr_t pool_base;
    platform_usb_device_dci_resource_t *brcm_resource = NULL;
    UX_USER_CONFIG_DEVICE ux_user_obj = { 0 };
    UINT status;
    UINT i;

    WPRINT_APP_INFO( ( "USB Storage Device init starting...\n" ) );

    /* Sanity check  */
    if ( usb20d_init_completed == WICED_TRUE )
    {
        WPRINT_PLATFORM_ERROR( ( "Already init!\n" ) );
        goto exit;
    }

    /* Allocate memory pool  */
    if ( ( usb20d_memcfg.memory_pool = malloc( USB_DEVICE_POOL_SIZE ) ) == NULL )
    {
        WPRINT_PLATFORM_ERROR( ( "Alloc usb memory pool failed!\n" ) );
        goto exit;
    }
    usb20d_memcfg.memory_pool_size = USB_DEVICE_POOL_SIZE;

    if ( ( usb20d_memcfg.dma_memory_pool = osl_dma_alloc_consistent( USB_DEVICE_DMA_POOL_SIZE, USB_DEVICE_DMA_POOL_ALIGN, &pool_size, &pool_base ) ) == NULL )
    {
        WPRINT_PLATFORM_ERROR( ( "Alloc usb dma memory pool failed!\n" ) );
        goto exit;
    }
    usb20d_memcfg.dma_memory_pool_size = pool_size;

    /* Initialize USBX Memory */
    status = ux_system_initialize( usb20d_memcfg.memory_pool, usb20d_memcfg.memory_pool_size, usb20d_memcfg.dma_memory_pool, usb20d_memcfg.dma_memory_pool_size );
    WPRINT_PLATFORM_DEBUG( ( "Initialized USBX. status=%d\n", status ) );

    /* The code below is required for installing the device portion of USBX.  */
    /* Initialize USBX Device Stack (Use USBX system default values if given UX_USER_CONFIG_DEVICE as NULL) */
    ux_user_obj.ux_user_config_device_max_class         = MAX_DEVICE_CLASS_DRIVER;
    ux_user_obj.ux_user_config_device_thread_stack_size = USB_DEVICE_THREAD_STACK_SIZE;

    status = ux_device_stack_initialize( device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                         device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                         string_framework, STRING_FRAMEWORK_LENGTH,
                                         language_id_framework, LANGUAGE_ID_FRAMEWORK_LENGTH,
                                         wiced_usb_device_usbx_evt_callback,
                                         &ux_user_obj );

    if( status != UX_SUCCESS )
    {
        WPRINT_PLATFORM_ERROR( ( "USBX device stack init failed. status=%d\n", status ) );
        goto exit;
    }

    /* Store the number of LUN in this device storage instance.  */
    storage_parameter.ux_slave_class_storage_parameter_number_lun = 1;

    /* Initialize the storage class parameters for reading/writing to the Flash Disk.  */
    #if (RAM_DISK_USE_FILEX_DRIVER)
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_last_lba        =  ( ( RAM_DISK_STORAGE_SIZE ) / DEFAULE_LOGICAL_BLOCK_SIZE ) - 1 ;
    #else
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_last_lba        =  ( ( ram_disk_filesystem->device->device_size ) / DEFAULE_LOGICAL_BLOCK_SIZE ) - 1 ;
    #endif
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_block_length    =  DEFAULE_LOGICAL_BLOCK_SIZE;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_type            =  0;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_removable_flag  =  0x80;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_read            =  storage_media_read;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_write           =  storage_media_write;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_status          =  storage_media_status;

    /* Initilize the device storage class. The class is connected with interface 0 */
    status = ux_device_stack_class_register( _ux_system_slave_class_storage_name, ux_device_class_storage_entry, 1, 0, ( VOID * ) &storage_parameter );
    if( status != UX_SUCCESS )
    {
        WPRINT_PLATFORM_ERROR( ( "USBX device storage class register failed. status=%d\n", status ) );
        goto exit;
    }

    /* Init USB20 Device HW.  */
    WPRINT_PLATFORM_DEBUG( ( "Init USB20 Device HW\n" ) );
    status = platform_usb_device_init();
    if ( status != PLATFORM_SUCCESS )
    {
        WPRINT_PLATFORM_ERROR( ( "USB20 Device HW init failed. status=%d\n", status ) );
        goto exit;
    }
    status = platform_usb_device_init_irq( wiced_usb_device_usbx_isr );
    if ( status != PLATFORM_SUCCESS )
    {
        WPRINT_PLATFORM_ERROR( ( "USB20 Device irq init failed. status=%d\n", status ) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ( "USB20 Device HW init ok!\n" ) );

    /* Obtain USB20 Device DCI resources for USB device controller driver  */
    status = platform_usb_device_get_dci_resource( ( platform_usb_device_dci_resource_t * ) usb20d_dci_info, sizeof( usb20d_dci_info ), &usb20d_dci_num );
    if ( status != PLATFORM_SUCCESS )
    {
        WPRINT_PLATFORM_ERROR( ( "USB20 Device DCI resources get failed. status=%d\n", status ) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ( "USB20 Device HW supports %lu DCI resources\n", usb20d_dci_num ) );

    for ( i = 0; i < usb20d_dci_num; i ++ )
    {
        if ( usb20d_dci_info[i].usb_device_dci_type == USB_DEVICE_CONTROLLER_INTERFACE_BRCM )
        {
            brcm_resource = &usb20d_dci_info[i];
        }
    }
    if ( brcm_resource == NULL )
    {
        WPRINT_PLATFORM_ERROR( ( "No USB20 Device resources for USBX\n" ) );
        goto exit;
    }

    /* Init USBX Device controller  */
    WPRINT_APP_INFO( ( "USB20 Device io=0x%08lx, irq=%lu\n", brcm_resource->usb_device_dci_ioaddress, brcm_resource->usb_device_dci_irq_number ) );

    status = ux_dcd_bcm4390x_initialize( brcm_resource->usb_device_dci_ioaddress, (ULONG)( brcm_resource->usb_device_dci_private_data ) );
    if( status != UX_SUCCESS )
    {
        WPRINT_PLATFORM_ERROR( ( "USBX BCM4390x device controller init failed. status=%d\n", status ) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ( "USBX BCM4390x device controller init ok!\n" ) );

    /* Enable USB20 Device interrupt  */
    status = platform_usb_device_enable_irq();
    if ( status != PLATFORM_SUCCESS )
    {
        WPRINT_PLATFORM_ERROR( ( "USB20 Device irq enable failed. status=%d\n", status ) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ( "USB20 IRQ enabled!\n" ) );

    /* Set init completed flag  */
    usb20d_init_completed = WICED_TRUE;
    WPRINT_APP_INFO( ( "USB20 Device init completed!!!\n" ) );

    return WICED_SUCCESS;

exit:
    if ( usb20d_memcfg.memory_pool )
        free( usb20d_memcfg.memory_pool );
    if ( usb20d_memcfg.dma_memory_pool )
        osl_dma_free_consistent( usb20d_memcfg.dma_memory_pool );

    return WICED_ERROR;
}

static void wiced_usb_device_usbx_isr( void )
{
    _ux_dcd_bcm4390x_interrupt_handler();
}

static UINT wiced_usb_device_usbx_evt_callback( ULONG event )
{
    WPRINT_APP_DEBUG( ( "USB20 Device: event (%ld)\n", event ) );

    switch ( event )
    {
        case UX_DEVICE_RESET:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_RESET\n" ) );
            break;

        case UX_DEVICE_ATTACHED:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_ATTACHED\n" ) );
            break;

        case UX_DEVICE_ADDRESSED:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_ADDRESSED\n" ) );
            break;

        case UX_DEVICE_CONFIGURED:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_CONFIGURED\n" ) );
            break;

        case UX_DEVICE_SUSPENDED:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_SUSPENDED\n" ) );
            break;

        case UX_DEVICE_RESUMED:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_RESUMED\n" ) );
            break;

        case UX_DEVICE_SELF_POWERED_STATE:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_SELF_POWERED_STATE\n" ) );
            break;

        case UX_DEVICE_BUS_POWERED_STATE:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_BUS_POWERED_STATE\n" ) );
            break;

        case UX_DEVICE_REMOTE_WAKEUP:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_REMOTE_WAKEUP\n" ) );
            break;

        case UX_DEVICE_BUS_RESET_COMPLETED:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_BUS_RESET_COMPLETED\n" ) );
            break;

        case UX_DEVICE_REMOVED:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_REMOVED\n" ) );
            break;

        case UX_DEVICE_FORCE_DISCONNECT:
            WPRINT_APP_DEBUG( ( "UX_DEVICE_FORCE_DISCONNECT\n" ) );
            break;

        default :
            WPRINT_APP_DEBUG( ( "USB20 Device: unknown event (%ld)\n", event ) );
            break;
    }

    return UX_SUCCESS;
}

static wiced_result_t ram_disk_init( void )
{
    wiced_result_t result;
    UINT status;

    UNUSED_PARAMETER( result );
    UNUSED_PARAMETER( status );

#if (RAM_DISK_USE_FILEX_DRIVER)
    /* Init RAM Disk storage with filesystem */
    /* Format the media.  */
    status =  fx_media_format( &ram_disk, _fx_ram_driver, ram_disk_memory, buffer, FS_MEDIA_BUFFER_SIZE, "4390x_MS", 2, 512, 0, ( RAM_DISK_STORAGE_SIZE / DEFAULT_SECTOR_SIZE ) , DEFAULT_SECTOR_SIZE, 1, 1, 1 );

    /* Check the media format status.  */
    if ( status != FX_SUCCESS )
    {
        /* Error opening media.  Return to caller.  */
        return WICED_ERROR;
    }

    /* Open the ram_disk.  */
    status =  fx_media_open( &ram_disk, "", _fx_ram_driver, ram_disk_memory, buffer, FS_MEDIA_BUFFER_SIZE );

    /* Check the media open status.  */
    if ( status != FX_SUCCESS )
    {
        /* Error opening media.  Return to caller.  */
        return WICED_ERROR;
    }
    WPRINT_APP_INFO( ( "USB20 Device: FileX RAM Disk mount ok!!!\n" ) );

#else

    /* Lookup Ramdisk device entry from all_filesystem_devices */
    if ( ram_disk_filesystem == NULL )
    {
        const filesystem_list_t* curr_item = all_filesystem_devices;

        while ( curr_item->device != NULL )
        {
            if ( strcmp ( RAM_DISK_DEVICE_NAME, curr_item->name ) == 0 )
            {
                /* Found requested device */
                ram_disk_filesystem = (filesystem_list_t*) curr_item;
                break;
            }
            curr_item++;
        }

        if ( curr_item->device == NULL )
        {
            /* Not found - print options */
            WPRINT_APP_INFO( ( "Filesystem 'DDR' not found\n" ) );
            return WICED_ERROR;
        }
    }

    WPRINT_APP_INFO( ( "Found 'DDR' Filesystem\n" ) );

    /* Format the filesystem */
    result = wiced_filesystem_format( ram_disk_filesystem->device, ram_disk_filesystem->type );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Error: Failed to format filesystem\n" ) );
        return result;
    }

    /* Mount the filesystem */
    result = wiced_filesystem_mount( ram_disk_filesystem->device, ram_disk_filesystem->type, &ram_disk_fs_handle, RAM_DISK_STORAGE_MOUNT_NAME );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Error: Failed to mount filesystem\n" ) );
        return result;
    }

    ram_disk_media = &ram_disk_fs_handle.data.filex.handle;

    WPRINT_APP_INFO( ( "USB20 Device: Wiced RAM Disk mount ok!!!\n" ) );

#endif

    return WICED_SUCCESS;
}

static UINT storage_media_status( VOID *storage, ULONG lun, ULONG media_id, ULONG *media_status )
{
    /* The ATA drive never fails. This is just for demo only !!!! */
    /* Uncomment below line to see the message. Windows/Linux calls this every few seconds. */
    // WPRINT_APP_INFO( ("USB20D Storage device status\n") );
    return( UX_SUCCESS );
}

static UINT storage_media_read( VOID *storage, ULONG lun, UCHAR * data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status )
{
    UINT status = UX_SUCCESS;

    WPRINT_APP_DEBUG( ("USB20D Storage device read: lun=%lu, data_pointer=0x%lx, number_blocks=%lu, lba=%lu\n",
        lun, (ULONG) data_pointer, number_blocks, lba ) );

#if (RAM_DISK_USE_FILEX_DRIVER)
    while( number_blocks-- )
    {
        if( lba == DISK_MBR_SECTOR )
        {
            ram_disk.fx_media_driver_logical_sector = DISK_MBR_SECTOR;
            ram_disk.fx_media_driver_sectors = 1;
            ram_disk.fx_media_driver_request = FX_DRIVER_BOOT_READ;
            ram_disk.fx_media_driver_buffer = data_pointer;

            _fx_ram_driver( &ram_disk );

            status = ram_disk.fx_media_driver_status;
            WPRINT_APP_DEBUG( ( "FX_DRIVER_BOOT_READ status=%d\n", status ) );
        }
        else
        {
            status =  fx_media_read( &ram_disk, lba, data_pointer );
        }
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ( "usb20d: read lba=%lu failed. Status=%d ", lba, status ) );
            return status;
        }
        data_pointer += DEFAULE_LOGICAL_BLOCK_SIZE;
        lba ++;
    }
#else  /* (RAM_DISK_USE_FILEX_DRIVER) */
    while(number_blocks--)
    {
        if ( lba == DISK_MBR_SECTOR )
        {
            ram_disk_media->fx_media_driver_logical_sector = DISK_MBR_SECTOR;
            ram_disk_media->fx_media_driver_sectors = 1;
            ram_disk_media->fx_media_driver_request = FX_DRIVER_BOOT_READ;
            ram_disk_media->fx_media_driver_buffer = data_pointer;

            wiced_filex_driver(ram_disk_media);

            status = ram_disk_media->fx_media_driver_status;
            WPRINT_APP_DEBUG( ( "FX_DRIVER_BOOT_READ status=%d\n", status ) );
        }
        else
        {
            status =  fx_media_read( ram_disk_media, lba, data_pointer );
        }
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ( "usb20d: read lba=%lu failed. Status=%d ", lba, status ) );
            return status;
        }
        data_pointer += DEFAULE_LOGICAL_BLOCK_SIZE;
        lba ++;
    }
#endif  /* (RAM_DISK_USE_FILEX_DRIVER) */

    return(status);
}

static UINT storage_media_write( VOID *storage, ULONG lun, UCHAR * data_pointer, ULONG number_blocks, ULONG lba, ULONG *media_status )
{
    UINT status = UX_SUCCESS;

    WPRINT_APP_DEBUG( ("USB20D Storage device write: lun=%lu, data_pointer=0x%lx, number_blocks=%lu, lba=%lu\n",
        lun, (ULONG) data_pointer, number_blocks, lba) );

#if (RAM_DISK_USE_FILEX_DRIVER)
    while(number_blocks--)
    {
        if( lba == DISK_MBR_SECTOR )
        {
            ram_disk.fx_media_driver_logical_sector = DISK_MBR_SECTOR;
            ram_disk.fx_media_driver_sectors = 1;
            ram_disk.fx_media_driver_request = FX_DRIVER_BOOT_WRITE;
            ram_disk.fx_media_driver_buffer = data_pointer;
            _fx_ram_driver( &ram_disk );

            status = ram_disk.fx_media_driver_status;
            WPRINT_APP_DEBUG( ( "FX_DRIVER_BOOT_WRITE status=%d\n", status ) );
        }
        else
        {
            status =  fx_media_write( &ram_disk,lba, data_pointer );
            WPRINT_APP_DEBUG( ( "fx_media_write status=%d\n", status ) );
        }
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ( "usb20d: write lba=%lu failed. Status=%d ", lba, status ) );
            return status;
        }
        data_pointer += DEFAULE_LOGICAL_BLOCK_SIZE;
        lba ++;
    }
#else  /* (RAM_DISK_USE_FILEX_DRIVER) */
    while( number_blocks-- )
    {
        if( lba == DISK_MBR_SECTOR )
        {
            ram_disk_media->fx_media_driver_logical_sector = DISK_MBR_SECTOR;
            ram_disk_media->fx_media_driver_sectors = 1;
            ram_disk_media->fx_media_driver_request = FX_DRIVER_BOOT_WRITE;
            ram_disk_media->fx_media_driver_buffer = data_pointer;

            wiced_filex_driver( ram_disk_media );

            status = ram_disk_media->fx_media_driver_status;
            WPRINT_APP_DEBUG( ( "FX_DRIVER_BOOT_WRITE status=%d\n", status ) );
        }
        else
        {
            status = fx_media_write( ram_disk_media, lba, data_pointer );
        }
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ( "usb20d: write lba=%lu failed. Status=%d ", lba, status ) );
            return status;
        }
        data_pointer += DEFAULE_LOGICAL_BLOCK_SIZE;
        lba ++;
    }
#endif  /* (RAM_DISK_USE_FILEX_DRIVER) */

    return(status);
}



