#include "wiced.h"
#include "arrow/gateway.h"
#include "arrow/device.h"

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/
#define DEFAULT_DEVICE_NAME   DEVICE_NAME

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct acn_config_dct_s
{
    wiced_bool_t  is_configured;
    char          device_name[32];
    char          gateway_hid[64];
    char          device_hid[64];
    char          device_eid[64];
} acn_config_dct_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t acn_configure_device(void);


#ifdef __cplusplus
} /* extern "C" */
#endif
