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
 * WICED HID Host SDP function
 *
 */

#include "wiced_memory.h"
#include "wiced_bt_hidh_sdp.h"
#include "wiced_bt_hidh_core.h"
#include "wiced_bt_hidh_utils.h"
#include "wiced_bt_types.h"
#include "wiced_bt_sdp_defs.h"
/*
 * Definitions
 */
#define WICED_BT_HIDH_DISC_BUF_SIZE     512
#define WICED_BT_HIDH_DESC_BUF_SIZE     2048

/*
 * Local functions
 */
static void wiced_bt_hidh_sdp_cback (uint16_t sdp_result);
static wiced_bt_hidh_status_t wiced_bt_hidh_sdp_check_record(wiced_bt_sdp_discovery_db_t *p_sdp_db);
static void wiced_bt_hidh_sdp_descriptor_cback (uint16_t sdp_result);

/*
 * wiced_bt_hidh_sdp_discover
 */
wiced_bt_hidh_status_t wiced_bt_hidh_sdp_discover(wiced_bt_hidh_dev_t *p_dev)
{
    wiced_bool_t result;
    wiced_bt_sdp_discovery_db_t *p_sdp_db;
    wiced_bt_uuid_t  wiced_bt_hidh_uuid[] =
        {{.len = LEN_UUID_16, .uu.uuid16 = UUID_SERVCLASS_HUMAN_INTERFACE}};
    uint16_t attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST/*, ATTR_ID_HID_BOOT_DEVICE*/};

    HIDH_TRACE_DBG("wiced_bt_hidh_sdp_discover\n");

    if (wiced_bt_hidh_cb.p_sdp_db != NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_sdp_discover already ongoing\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    /* Allocate a buffer for the SDP Response */
    p_sdp_db = (wiced_bt_sdp_discovery_db_t *)wiced_bt_get_buffer(WICED_BT_HIDH_DISC_BUF_SIZE);
    if (p_sdp_db == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_sdp_discover Mem full\n");
        return WICED_BT_HIDH_STATUS_MEM_FULL;
    }

    /* Initialize the SDP Request */
    result = wiced_bt_sdp_init_discovery_db(p_sdp_db, WICED_BT_HIDH_DISC_BUF_SIZE,
            HIDH_SIZEOF_ARRAY(wiced_bt_hidh_uuid), wiced_bt_hidh_uuid,
            HIDH_SIZEOF_ARRAY(attr_list), attr_list);
    if (result == WICED_FALSE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_sdp_discover sdp_init_discovery_db failed\n");
        wiced_bt_free_buffer(p_sdp_db);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    wiced_bt_hidh_cb.p_sdp_db = p_sdp_db;
    wiced_bt_hidh_cb.p_dev_sdp = p_dev;

    /* perform service search */
    result = wiced_bt_sdp_service_search_attribute_request(p_dev->bdaddr, p_sdp_db,
            wiced_bt_hidh_sdp_cback);
    if (result == WICED_FALSE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_sdp_discover service_search_attribute_request failed\n");
        wiced_bt_free_buffer(p_sdp_db);
        wiced_bt_hidh_cb.p_sdp_db = NULL;
        wiced_bt_hidh_cb.p_dev_sdp = NULL;
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_sdp_cback
 */
static void wiced_bt_hidh_sdp_cback (uint16_t sdp_status)
{
    wiced_bt_hidh_dev_t *p_dev = wiced_bt_hidh_cb.p_dev_sdp;
    wiced_bt_hidh_status_t status;

    HIDH_TRACE_DBG( "wiced_bt_hidh_sdp_cback sdp_status:0x%x\n", sdp_status);

    if (sdp_status != WICED_BT_SDP_SUCCESS)
    {
        wiced_bt_free_buffer(wiced_bt_hidh_cb.p_sdp_db);
        wiced_bt_hidh_cb.p_sdp_db = NULL;
        wiced_bt_hidh_cb.p_dev_sdp = NULL;

        switch(sdp_status)
        {
        case WICED_BT_SDP_CONN_FAILED:
             status = WICED_BT_HIDH_STATUS_PAGE_TIMEOUT;
             break;
        case WICED_BT_SDP_NO_DI_RECORD_FOUND:
        case WICED_BT_SDP_ERR_ATTR_NOT_PRESENT:
            status = WICED_BT_HIDH_STATUS_NOT_SUPPORTED;
            break;
        default:
            status = WICED_BT_HIDH_STATUS_ERROR;
            break;
        }
        /* Update HIDH State Machine */
        wiced_bt_hidh_core_discovery_done(p_dev, status);
        return;
    }

    /* Check if the SDP Record contains an HID Service */
    status = wiced_bt_hidh_sdp_check_record(wiced_bt_hidh_cb.p_sdp_db);

    wiced_bt_free_buffer(wiced_bt_hidh_cb.p_sdp_db);
    wiced_bt_hidh_cb.p_sdp_db = NULL;
    wiced_bt_hidh_cb.p_dev_sdp = NULL;

    /* Update HIDH State Machine */
    wiced_bt_hidh_core_discovery_done(p_dev, status);
}

/*
 * wiced_bt_hidh_sdp_check_record
 */
static wiced_bt_hidh_status_t wiced_bt_hidh_sdp_check_record(wiced_bt_sdp_discovery_db_t *p_sdp_db)
{
    wiced_bt_uuid_t  wiced_bt_hidh_uuid[] =
        {{.len = LEN_UUID_16, .uu.uuid16 = UUID_SERVCLASS_HUMAN_INTERFACE}};
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_discovery_attribute_t  *p_attr;

    HIDH_TRACE_DBG("wiced_bt_hidh_sdp_check_record\n");

    p_rec = wiced_bt_sdp_find_service_uuid_in_db(p_sdp_db, wiced_bt_hidh_uuid, p_rec);
    if (p_rec == NULL)
    {
        HIDH_TRACE_DBG("HID service Not found\n");
        return WICED_BT_HIDH_STATUS_NOT_SUPPORTED;
    }

    HIDH_TRACE_DBG("HID service found\n");

    /* Search for the Boot attribute */
    p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_HID_BOOT_DEVICE);
    if (p_attr != NULL)
    {
        HIDH_TRACE_DBG("HID Boot attribute found. Value:%d\n", p_attr->attr_value.v.u16);
        wiced_bt_hidh_cb.p_dev_sdp->boot_supported = p_attr->attr_value.v.u16;
    }

    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_sdp_get_descriptor
 */
wiced_bt_hidh_status_t wiced_bt_hidh_sdp_get_descriptor(wiced_bt_hidh_dev_t *p_dev)
{
    wiced_bool_t result;
    wiced_bt_sdp_discovery_db_t *p_sdp_db;
    wiced_bt_uuid_t  wiced_bt_hidh_uuid[] =
        {{.len = LEN_UUID_16, .uu.uuid16 = UUID_SERVCLASS_HUMAN_INTERFACE}};
    uint16_t attr_list[] = {ATTR_ID_SERVICE_CLASS_ID_LIST, ATTR_ID_HID_DESCRIPTOR_LIST};

    HIDH_TRACE_DBG("wiced_bt_hidh_sdp_get_descriptor for [%B]\n", p_dev->bdaddr);

    if (wiced_bt_hidh_cb.p_sdp_db != NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_sdp_get_descriptor SDP already ongoing\n");
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    p_sdp_db = (wiced_bt_sdp_discovery_db_t *)wiced_bt_get_buffer(WICED_BT_HIDH_DESC_BUF_SIZE);
    if (p_sdp_db == NULL)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_sdp_get_descriptor Mem full\n");
        return WICED_BT_HIDH_STATUS_MEM_FULL;
    }

    /* Initialize SDP Search */
    result = wiced_bt_sdp_init_discovery_db(p_sdp_db, WICED_BT_HIDH_DESC_BUF_SIZE,
            HIDH_SIZEOF_ARRAY(wiced_bt_hidh_uuid), wiced_bt_hidh_uuid,
            HIDH_SIZEOF_ARRAY(attr_list), attr_list);
    if (result == WICED_FALSE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_sdp_get_descriptor sdp_init_discovery_db failed\n");
        wiced_bt_free_buffer(p_sdp_db);
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    wiced_bt_hidh_cb.p_sdp_db = p_sdp_db;
    wiced_bt_hidh_cb.p_dev_sdp = p_dev;

    /* perform service search */
    result = wiced_bt_sdp_service_search_attribute_request(p_dev->bdaddr, p_sdp_db,
            wiced_bt_hidh_sdp_descriptor_cback);
    if (result == WICED_FALSE)
    {
        HIDH_TRACE_DBG("Err: wiced_bt_hidh_sdp_get_descriptor sdp_service_search_attribute_request failed\n");
        wiced_bt_free_buffer(p_sdp_db);
        wiced_bt_hidh_cb.p_sdp_db = NULL;
        wiced_bt_hidh_cb.p_dev_sdp = NULL;
        return WICED_BT_HIDH_STATUS_ERROR;
    }

    return WICED_BT_HIDH_STATUS_OK;
}

/*
 * wiced_bt_hidh_sdp_descriptor_cback
 */
static void wiced_bt_hidh_sdp_descriptor_cback (uint16_t sdp_result)
{
    wiced_bt_hidh_dev_t *p_dev = wiced_bt_hidh_cb.p_dev_sdp;
    wiced_bt_hidh_status_t status = WICED_BT_HIDH_STATUS_OK;
    wiced_bt_uuid_t  wiced_bt_hidh_uuid[] =
        {{.len = LEN_UUID_16, .uu.uuid16 = UUID_SERVCLASS_HUMAN_INTERFACE}};
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    uint8_t *p_hid_desc = NULL;
    uint16_t hid_desc_len = 0;
    wiced_bt_sdp_discovery_attribute_t *p_attr, *p_subattr1, *p_subattr2, *p_repdesc;

    HIDH_TRACE_DBG( "wiced_bt_hidh_sdp_descriptor_cback sdp_result:0x%x\n", sdp_result);

    if (sdp_result != WICED_BT_SDP_SUCCESS)
    {
        wiced_bt_free_buffer(wiced_bt_hidh_cb.p_sdp_db);
        wiced_bt_hidh_cb.p_sdp_db = NULL;
        wiced_bt_hidh_cb.p_dev_sdp = NULL;

        switch(sdp_result)
        {
        case WICED_BT_SDP_CONN_FAILED:
             status = WICED_BT_HIDH_STATUS_PAGE_TIMEOUT;
             break;
        case WICED_BT_SDP_NO_DI_RECORD_FOUND:
        case WICED_BT_SDP_ERR_ATTR_NOT_PRESENT:
            status = WICED_BT_HIDH_STATUS_NOT_SUPPORTED;
            break;
        default:
            status = WICED_BT_HIDH_STATUS_ERROR;
            break;
        }
        /* HID Descriptor done */
        wiced_bt_hidh_core_descriptor_done(p_dev, status, p_hid_desc, hid_desc_len);
        return;
    }

    /* Search the HID Service */
    p_rec = wiced_bt_sdp_find_service_uuid_in_db(wiced_bt_hidh_cb.p_sdp_db, wiced_bt_hidh_uuid,
            p_rec);
    if (p_rec == NULL)
    {
        HIDH_TRACE_DBG("HID Service Not found\n");
        status = WICED_BT_HIDH_STATUS_NOT_SUPPORTED;
    }
    else
    {
        /* Search the HID DESCRIPTOR attribute in the record */
        p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_HID_DESCRIPTOR_LIST);
        if ((p_attr == NULL)                                                                    ||
            (SDP_DISC_ATTR_TYPE(p_attr->attr_len_type) != DATA_ELE_SEQ_DESC_TYPE)               ||
            ((p_subattr1 = p_attr->attr_value.v.p_sub_attr) == NULL)                            ||
            (SDP_DISC_ATTR_TYPE(p_subattr1->attr_len_type) != DATA_ELE_SEQ_DESC_TYPE)           ||
            ((p_subattr2 = p_subattr1->attr_value.v.p_sub_attr) == NULL)                        ||
            ((p_repdesc=(wiced_bt_sdp_discovery_attribute_t *)p_subattr2->p_next_attr) == NULL) ||
            (SDP_DISC_ATTR_TYPE(p_repdesc->attr_len_type) != TEXT_STR_DESC_TYPE))
        {
            HIDH_TRACE_DBG("HID Descriptor Not found\n");
            status = WICED_BT_HIDH_STATUS_NOT_SUPPORTED;
        }
        else
        {
           if (SDP_DISC_ATTR_LEN(p_repdesc->attr_len_type) != 0)
           {
               status = WICED_BT_HIDH_STATUS_OK;
               hid_desc_len = SDP_DISC_ATTR_LEN(p_repdesc->attr_len_type);
               p_hid_desc = (uint8_t *)&p_repdesc->attr_value;
               HIDH_TRACE_DBG("HID Descriptor length:%d found\n", hid_desc_len);
           }
        }
    }

    /* HID Descriptor done */
    wiced_bt_hidh_core_descriptor_done(p_dev, status, p_hid_desc, hid_desc_len);

    /* Free SDP Database */
    wiced_bt_free_buffer(wiced_bt_hidh_cb.p_sdp_db);
    wiced_bt_hidh_cb.p_sdp_db = NULL;
    wiced_bt_hidh_cb.p_dev_sdp = NULL;
}

