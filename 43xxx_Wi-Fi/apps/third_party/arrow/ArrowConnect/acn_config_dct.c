/** @file
 *
 */

#include "acn_config.h"
#include "wiced_framework.h"


/******************************************************
 *               Variable Definitions
 ******************************************************/
DEFINE_APP_DCT(acn_config_dct_t)
{
    .is_configured = WICED_FALSE,
    .device_name = DEFAULT_DEVICE_NAME
};
