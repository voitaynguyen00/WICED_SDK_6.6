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
/*  11423 West Bernardo Court               www.expresslogic.com          */
/*  San Diego, CA  92127                                                  */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** ThreadX Component                                                     */
/**                                                                       */
/**   Mutex                                                               */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    tx_mutex.h                                          PORTABLE C      */ 
/*                                                           5.9          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    William E. Lamie, Express Logic, Inc.                               */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file defines the ThreadX mutex management component,           */ 
/*    including all data types and external references.  It is assumed    */ 
/*    that tx_api.h and tx_port.h have already been included.             */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  12-12-2005     William E. Lamie         Initial Version 5.0           */ 
/*  04-02-2007     William E. Lamie         Modified comment(s), and      */ 
/*                                            replaced UL constant        */ 
/*                                            modifier with ULONG cast,   */ 
/*                                            resulting in version 5.1    */ 
/*  12-12-2008     William E. Lamie         Modified comment(s),          */ 
/*                                            resulting in version 5.2    */ 
/*  07-04-2009     William E. Lamie         Modified comment(s),          */ 
/*                                            resulting in version 5.3    */ 
/*  12-12-2009     William E. Lamie         Modified comment(s),          */ 
/*                                            resulting in version 5.4    */ 
/*  07-15-2011     William E. Lamie         Modified comment(s), and      */ 
/*                                            removed unnecessary param   */ 
/*                                            in mutex priority change,   */ 
/*                                            resulting in version 5.5    */ 
/*  07-15-2011     William E. Lamie         Modified comment(s),          */ 
/*                                            resulting in version 5.5    */ 
/*  11-01-2012     William E. Lamie         Modified comment(s), and      */ 
/*                                            added new prototype,        */ 
/*                                            resulting in version 5.6    */ 
/*  05-01-2015     William E. Lamie         Modified comment(s), and      */ 
/*                                            modified code for MISRA     */ 
/*                                            compliance, resulting in    */ 
/*                                            version 5.7                 */ 
/*  06-01-2017     William E. Lamie         Modified comment(s), added    */ 
/*                                            suspension sequence to      */ 
/*                                            verify cleanup is still     */ 
/*                                            necessary, made MISRA       */ 
/*                                            compatibility changes,      */ 
/*                                            and added macro for         */ 
/*                                            extending mutex delete,     */ 
/*                                            resulting in version 5.8    */ 
/*  02-01-2019     William E. Lamie         Modified comment(s),          */ 
/*                                            resulting in version 5.9    */ 
/*                                                                        */ 
/**************************************************************************/ 

#ifndef TX_MUTEX_H
#define TX_MUTEX_H


/* Define mutex control specific data definitions.  */

#define TX_MUTEX_ID                             ((ULONG) 0x4D555445)


/* Determine if in-line component initialization is supported by the 
   caller.  */

#ifdef TX_INVOKE_INLINE_INITIALIZATION

/* Yes, in-line initialization is supported, remap the mutex initialization 
   function.  */

#ifndef TX_MUTEX_ENABLE_PERFORMANCE_INFO
#define _tx_mutex_initialize() \
                    _tx_mutex_created_ptr =                             TX_NULL;      \
                    _tx_mutex_created_count =                           TX_EMPTY                          
#else
#define _tx_mutex_initialize() \
                    _tx_mutex_created_ptr =                             TX_NULL;      \
                    _tx_mutex_created_count =                           TX_EMPTY;     \
                    _tx_mutex_performance_put_count =                   ((ULONG) 0);  \
                    _tx_mutex_performance_get_count =                   ((ULONG) 0);  \
                    _tx_mutex_performance_suspension_count =            ((ULONG) 0);  \
                    _tx_mutex_performance_timeout_count =               ((ULONG) 0);  \
                    _tx_mutex_performance_priority_inversion_count =    ((ULONG) 0);  \
                    _tx_mutex_performance__priority_inheritance_count =  ((ULONG) 0)
#endif
#define TX_MUTEX_INIT
#else

/* No in-line initialization is supported, use standard function call.  */
VOID        _tx_mutex_initialize(VOID);
#endif


/* Define internal mutex management function prototypes.  */

VOID        _tx_mutex_cleanup(TX_THREAD *thread_ptr, ULONG suspension_sequence);
VOID        _tx_mutex_thread_release(TX_THREAD *thread_ptr);
VOID        _tx_mutex_priority_change(TX_THREAD *thread_ptr, UINT new_priority);


/* Mutex management component data declarations follow.  */

/* Determine if the initialization function of this component is including
   this file.  If so, make the data definitions really happen.  Otherwise,
   make them extern so other functions in the component can access them.  */

#ifdef TX_MUTEX_INIT
#define MUTEX_DECLARE 
#else
#define MUTEX_DECLARE extern
#endif


/* Define the head pointer of the created mutex list.  */

MUTEX_DECLARE  TX_MUTEX *   _tx_mutex_created_ptr;


/* Define the variable that holds the number of created mutexes. */

MUTEX_DECLARE  ULONG        _tx_mutex_created_count;


#ifdef TX_MUTEX_ENABLE_PERFORMANCE_INFO

/* Define the total number of mutex puts.  */

MUTEX_DECLARE  ULONG        _tx_mutex_performance_put_count;


/* Define the total number of mutex gets.  */

MUTEX_DECLARE  ULONG        _tx_mutex_performance_get_count;


/* Define the total number of mutex suspensions.  */

MUTEX_DECLARE  ULONG        _tx_mutex_performance_suspension_count;


/* Define the total number of mutex timeouts.  */

MUTEX_DECLARE  ULONG        _tx_mutex_performance_timeout_count;


/* Define the total number of priority inversions.  */

MUTEX_DECLARE  ULONG        _tx_mutex_performance_priority_inversion_count;


/* Define the total number of priority inheritance conditions.  */

MUTEX_DECLARE  ULONG        _tx_mutex_performance__priority_inheritance_count;


#endif


/* Define default post mutex delete macro to whitespace, if it hasn't been defined previously (typically in tx_port.h).  */

#ifndef TX_MUTEX_DELETE_PORT_COMPLETION
#define TX_MUTEX_DELETE_PORT_COMPLETION(m)
#endif


#endif
