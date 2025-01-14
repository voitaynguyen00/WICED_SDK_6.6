/**************************************************************************/
/*                                                                        */
/*            Copyright (c) 1996-2019 by Express Logic Inc.               */
/*                                                                        */
/*  This software is copyrighted by and is the sole property of Express   */
/*  Logic, Inc.  All rights, title, ownership, or other interests         */
/*  in the software remain the property of Express Logic, Inc.  This      */
/*  software may only be used in accordance with the corresponding        */
/*  license agreement.  Any unauthorized use, duplication, transmission,  */
/*  distribution, or disclosure of this software is expressly forbidden.  */
/*                                                                        */
/*  This Copyright notice may not be removed or modified without prior    */
/*  written consent of Express Logic, Inc.                                */
/*                                                                        */
/*  Express Logic, Inc. reserves the right to modify this software        */
/*  without notice.                                                       */
/*                                                                        */
/*  Express Logic, Inc.                     info@expresslogic.com         */
/*  11423 West Bernardo Court               http://www.expresslogic.com   */
/*  San Diego, CA  92127                                                  */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** NetX Component                                                        */
/**                                                                       */
/**   Reverse Address Resolution Protocol (RARP)                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/*                                                                        */
/*  COMPONENT DEFINITION                                   RELEASE        */
/*                                                                        */
/*    nx_rarp.h                                           PORTABLE C      */
/*                                                           5.12         */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Express Logic, Inc.                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file defines the NetX Reverse Address Resolution Protocol      */
/*    component, including all data types and external references.  It    */
/*    is assumed that nx_api.h and nx_port.h have already been included.  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  12-12-2005     William E. Lamie         Initial Version 5.0           */
/*  08-09-2007     William E. Lamie         Modified comment(s), and      */
/*                                            changed UL to ULONG cast,   */
/*                                            resulting in version 5.1    */
/*  12-30-2007     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.2    */
/*  08-03-2009     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.3    */
/*  11-23-2009     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.4    */
/*  06-01-2010     Yuxin Zhou               Removed internal debug logic, */
/*                                            resulting in version 5.5    */
/*  10-10-2011     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.6    */
/*  01-31-2013     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.7    */
/*  01-12-2015     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.8    */
/*  02-22-2016     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.9    */
/*  05-10-2016     Yuxin Zhou               Modified comment(s), and      */
/*                                            removed unused code,        */
/*                                            resulting in version 5.10   */
/*  07-15-2018     Yuxin Zhou               Modified comment(s), added    */
/*                                            support for disabling IPv4, */
/*                                            resulting in version 5.11   */
/*  08-15-2019     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.12   */
/*                                                                        */
/**************************************************************************/

#ifndef NX_RARP_H
#define NX_RARP_H


#ifndef NX_DISABLE_IPV4
/* Define RARP Message format.  This will get encapsulated by an Ethernet frame
   as well.  The Ethernet frame will typically have a 6-byte Ethernet destination
   address, a 6-byte Ethernet source address, and a 2-byte Ethernet Frame type,
   which is 0x8035.  Regular IP frames have a frame type of 0x0800.

    Byte offset     Size            Meaning

        0           2           Hardware type (1 for Ethernet)
        2           2           Protocol type (0x0800 for IP)
        4           1           Number of bytes for hardware address (6 for Ethernet)
        5           1           Number of bytes for IP address (4 for IP)
        6           2           Operation, ARP request is 1, ARP reply is 2
        8           6           Sender's Ethernet Address
        14          4           Sender's IP Address
        18          6           Target Ethernet Address
        24          4           Target IP Address
 */

#define NX_RARP_HARDWARE_TYPE   ((ULONG)0x0001)
#define NX_RARP_PROTOCOL_TYPE   ((ULONG)0x0800)
#define NX_RARP_HARDWARE_SIZE   ((ULONG)0x06)
#define NX_RARP_PROTOCOL_SIZE   ((ULONG)0x04)
#define NX_RARP_OPTION_REQUEST  ((ULONG)0x0003)
#define NX_RARP_OPTION_RESPONSE ((ULONG)0x0004)
#define NX_RARP_MESSAGE_SIZE    28

/* Define RARP internal function prototypes.  */
VOID _nx_rarp_packet_send(NX_IP *ip_ptr);
VOID _nx_rarp_packet_receive(NX_IP *ip_ptr, NX_PACKET *packet_ptr);
VOID _nx_rarp_packet_deferred_receive(NX_IP *ip_ptr, NX_PACKET *packet_ptr);
VOID _nx_rarp_periodic_update(NX_IP *ip_ptr);
VOID _nx_rarp_queue_process(NX_IP *ip_ptr);
#endif /* NX_DISABLE_IPV4 */

/* Define RARP function prototypes.  */
UINT _nx_rarp_enable(NX_IP *ip_ptr);
UINT _nx_rarp_disable(NX_IP *ip_ptr);
UINT _nx_rarp_info_get(NX_IP *ip_ptr, ULONG *rarp_requests_sent, ULONG *rarp_responses_received,
                       ULONG *rarp_invalid_messages);

/* Define error checking shells for RARP services.  These are only referenced by the
   application.  */

UINT _nxe_rarp_enable(NX_IP *ip_ptr);
UINT _nxe_rarp_disable(NX_IP *ip_ptr);
UINT _nxe_rarp_info_get(NX_IP *ip_ptr, ULONG *rarp_requests_sent, ULONG *rarp_responses_received,
                        ULONG *rarp_invalid_messages);
#endif

