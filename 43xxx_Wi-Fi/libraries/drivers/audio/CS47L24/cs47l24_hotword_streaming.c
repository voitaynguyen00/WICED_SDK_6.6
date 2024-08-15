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

/**
 * @file CS47L24 DSP data buffer extraction/streaming over SPI
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wiced_log.h"
#include "wiced_rtos.h"
#include "wiced_defaults.h"

#include "cs47l24.h"
#include "cs47l24_private.h"
#include "cs47l24_register_map.h"
#include "unshorten.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define DSP_WORD_SIGN_MASK                  (0x00800000)                       /* DSP words are SIGNED 24-bit; number is negative if bit 23 is set            */

/******************************************************
 *                    Constants
 ******************************************************/

#define DSP_HOST_BUFFER_HEADER_MAGIC        (0x0049AEC7)                       /* Start of header marker                                                      */

#define MAX_RDMA_CHANNEL_COUNT              (6)                                /* Maximum number of output DMA channels                                       */
#define MAX_WDMA_CHANNEL_COUNT              (8)                                /* Maximum number of input DMA channels                                        */
#define JOB_NAME_SIZE                       (3)                                /* Maximum number of input DMA channels                                        */

#define DSP_STREAM_START_OFFSET_MIN         (-70)                              /* Min number of trigger offset periods is -70                                 */
#define DSP_STREAM_START_OFFSET_DEFAULT     (+34)                               /* 34 periods of 15 msecs each = 510 msecs before hotword                     */
#define DSP_STREAM_START_OFFSET_MAX         (+70)                               /* Max number of trigger offset periods is +70                                */

#define DSP_BUFFERTRIGGERONLY_DEFAULT       (WICED_FALSE)                      /* DSP will buffer hotword and the rest of utterance                           */
#define DSP_CMD_TIMEOUT_DEFAULT             (0)                                /* DSP will NOT wait for a command word right after the trigger word           */
#define DSP_STREAMING_TIMEOUT_DEFAULT       (0)                                /* DSP will stream out indefinitely                                            */
#define DSP_NO_STREAMING_TIMEOUT_DEFAULT    (2)                                /* DSP will not actually stream when combined with interrupt_and_stream at 0   */
#define DSP_INTERRUPT_AND_STREAM_DEFAULT    (WICED_TRUE)                       /* DSP will signal with interrupt and will stream out                          */
#define DSP_GPIO_TRIGGER_INDICATOR_DEFAULT  (2)                                /* DSP will signal first interrupt through GP2                                 */

#define DSP_HOST_HIGH_WATER_MARK_DEFAULT    (80)                               /* high water mark in number of 24-bit words                                   */
#define DSP_HOST_COMPRESSED_BUFFER_SIZE     (0x4000)                           /* Max continuous DSP buffer size                                              */
#define DSP_HOST_PCM_BUFFER_SIZE            (16000 + 1)                        /* up to 500 msecs of audio at 16KHz and 16-bit per frame                      */
#define DSP_HOST_PCM_BUFFER_THRESHOLD       (DSP_PCM_BLOCK_SIZE_DEFAULT << 1)  /* signal consumer when amount of available PCM data goes above this threshold */

#define DSP_PCM_BLOCK_SIZE_DEFAULT          (480)                              /* DSP uses 15-msec blocks, 480 bytes at 16KHz with 16-bit per sample          */
#define DSP_PCM_HOTWORD_START_IN_SAMPLES    (8000)                             /* 500 msecs, 8000 16-bit samples, at 16KHz                                    */
#define DSP_PCM_HOTWORD_END_OFFSET_ADJ      (8000)                             /* 500 msecs, 8000 16-bit samples, at 16KHz                                    */

#define DSP_HOST_NEW_DATA_FLAG              (0x1)
#define DSP_HOST_NEW_OFFSET_FLAG            (0x1)

#define SCC_THREAD_PRIORITY                 (4)
#define SCC_THREAD_STACK_SIZE               (4096)

#define DSP_PCM_RING_WRITE_RETRY_COUNT      (50)

#ifdef CS47L24_ENABLE_DSP_WATCHDOG
#define DSP_IRQ_WORKER_WAIT_TIMEOUT_MSECS   (1000)
#define CS47L24_MONITOR_WATCHDOG()          cs47l24_monitor_watchdog_counter(cs47l24)
#else /* !CS47L24_ENABLE_DSP_WATCHDOG */
#define DSP_IRQ_WORKER_WAIT_TIMEOUT_MSECS   (WICED_WAIT_FOREVER)
#define CS47L24_MONITOR_WATCHDOG()
#endif /* CS47L24_ENABLE_DSP_WATCHDOG */


/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    ADSP2_FIRMWARE_STATE_DOWNLOAD = 0x01,
    ADSP2_FIRMWARE_STATE_STARTED  = 0x02,
    ADSP2_FIRMWARE_STATE_INIT_OK  = 0x03,
} adsp2_firmware_state_t;

typedef enum
{
    DSP_SCC_FIRMWARE_VER_010700   = 0x00010700,
    DSP_SCC_FIRMWARE_VER_010701   = 0x00010701,
    DSP_SCC_FIRMWARE_VER_010702   = 0x00010702,
    DSP_SCC_FIRMWARE_VER_010800   = 0x00010800,
    DSP_SCC_FIRMWARE_VER_010801   = 0x00010801,
    DSP_SCC_FIRMWARE_VER_010803   = 0x00010803,
} dsp_scc_firmware_version_t;

typedef enum
{
    DSP_SCC_BUFFER_NO_ERROR              = 0,      /* No error                                                                             */
    DSP_SCC_BUFFER_ERROR_OVERFLOW        = 1 << 0, /* Buffer is full; write would cause an overflow                                        */
    DSP_SCC_BUFFER_ERROR_REWIND_ARG      = 1 << 1, /* Rewind argument is zero or less than zero                                            */
    DSP_SCC_BUFFER_ERROR_REWIND_TWICE    = 1 << 2, /* Buffer already rewound and is not locked                                             */
    DSP_SCC_BUFFER_ERROR_RESERVE_TWICE   = 1 << 3, /* Space has already been reserved                                                      */
    DSP_SCC_BUFFER_ERROR_INVALID_RESERVE = 1 << 4, /* Requested reservation space is less than zero or greater than the maximum block size */
    DSP_SCC_BUFFER_ERROR_INCONSISTENT    = 1 << 5, /* Previously reserved space doesn't match space required for write                     */
    DSP_SCC_BUFFER_ERROR_CORRUPT         = 1 << 6, /* Record size is invalid or space reservation error                                    */
    DSP_SCC_BUFFER_ERROR_FASTFORWARD_ARG = 1 << 7, /* Buffer hasn't been read or no read pointer to be moved or buffer is empty            */
} dsp_scc_buffer_error_t;

/******************************************************
 *                    Structures
 ******************************************************/

/*
 * Structures used in communication between ADSP2 core and host
 * The content of these structures needs to remain in NETWORK BYTE ORDER (big endian)
 */

#pragma pack(2)

typedef struct
{
    uint32_t    register_address;                        /* Place holder for register address                                          */
    uint16_t    padding;                                 /* Padding                                                                    */
    uint32_t    buffer[DSP_HOST_COMPRESSED_BUFFER_SIZE]; /* DSP word buffer                                                            */
} dsp_word_buffer_t;

typedef struct
{
    uint32_t    magic;              /* Magic data to identify start of buffer                                                                          */
    uint32_t    unused1;            /* Redundant, backwards compatibility only                                                                         */
    uint32_t    unused2;            /* Redundant, backwards compatibility only                                                                         */
    uint32_t    host_buf;           /* Pointer to buffer management structure dsp_host_buf_t                                                           */
    uint32_t    unused3;            /* Redundant, backwards compatibility only                                                                         */
    uint32_t    high_water_mark;    /* Point in the buffer at which IRQ is asserted (used to initialize the buffer manager's high_water_mark value)    */
    uint32_t    unused4;            /* Redundant, backwards compatibility only                                                                         */
} dsp_host_buffer_hdr_t;

typedef struct dsp_host_buf
{
    uint32_t    register_address;   /* Place holder for register address                                          */
    uint16_t    padding;            /* Padding                                                                    */

    uint32_t    X_buf_base;         /* pointer to base of X memory area used for buffer                           */
    uint32_t    X_buf_size;         /* number of words of X memory used for buffer                                */
    uint32_t    X_buf_base2;        /* pointer to base of 2nd X memory area used for buffer                       */
    uint32_t    X_buf_brk;          /* total X + X2 size                                                          */
    uint32_t    Y_buf_base;         /* pointer to base of Y memory area used for buffer                           */
    uint32_t    wrap;               /* total of X + X2 + Y buffer sizes                                           */
    uint32_t    high_water_mark;    /* send IRQ if more than this amount present, if requested by host            */
    uint32_t    irq_count;          /* count of IRQs sent                                                         */
    uint32_t    irq_ack;            /* used to acknowledge IRQs and indicate when another IRQ is required         */
    uint32_t    next_write_index;   /* next sample index to be written (only updated by writer)                   */
    uint32_t    next_read_index;    /* next sample index to be read (only updated by reader                       */
                                    /* if negative, equals minus the size of the last block written to the buffer */
                                    /* and indicates that the stream to the host has not yet started              */
    uint32_t    error;              /* flag indicating overflow                                                   */

    uint32_t    oldest_block_index; /* index in buffer of oldest surviving block                                  */
    uint32_t    requested_rewind;   /* rewind value passed when stream started                                    */
    uint32_t    reserved_space;     /* set up by reserve_space() call                                             */
    uint32_t    min_free;           /* minimum free space after stream started                                    */
    uint32_t    blocks_written_1;   /* total number of blocks written since reset                                 */
    uint32_t    blocks_written_0;
    uint32_t    words_written_1;    /* total number of words written since reset                                  */
    uint32_t    words_written_0;    /* total number of words written since reset                                  */
    uint32_t    iwrap;              /* ADSP2 only view of buffer memory excluding YM                              */
    uint32_t    support_ym_overwrite;
    uint32_t    generate_host_IRQ;
} dsp_host_buf_t;

typedef struct
{
    uint32_t              register_address;                            /* Place holder for register address                                    */
    uint16_t              padding;                                     /* Padding                                                              */

    uint32_t              sys_enable;                                  /* system enable flag (can be used to bypass the whole system)          */

    uint32_t              plugin_fmw_id;                               /* An easy to locate identity of the firware id picked up by WISCE      */
    uint32_t              plugin_fmw_vrs;                              /* An easy to locate identity of the firware version picked up by WISCE */

    uint32_t              adsp2_state;                                 /* State of the ADSP2 (boot success/failure)                            */
    uint32_t              adsp2_watchdog;                              /* Watch dog timer                                                      */

    uint32_t              dma_buffer_size;
    uint32_t              rdma_control_table[MAX_RDMA_CHANNEL_COUNT];  /* Contains the addresses for output dma buffers                        */
    uint32_t              wdma_control_table[MAX_WDMA_CHANNEL_COUNT];  /* Contains the addresses for input dma buffers                         */

    uint32_t              build_job_name[JOB_NAME_SIZE];
    uint32_t              build_job_number;

    dsp_host_buffer_hdr_t compression_buf_hdr;

    uint32_t              Trigger_Phrase_Found_Counter;
    uint32_t              Trigger_id;                                  /* Id of the heard word                                                 */
    uint32_t              Trigger_score;                               /* score for detected hit */

    uint32_t              UseSADTrigger;
    uint32_t              Streaming_Timeout;
    uint32_t              LowDspClkSpeed;
    uint32_t              HighDspClkSpeed;
    uint32_t              Interrupt_and_Stream;
    uint32_t              SadDebounceValue;
    uint32_t              GPIO_Trigger_Indicator;
    uint32_t              volym;
} system_config_XM_struct_t;

typedef struct
{
    uint32_t              register_address;                            /* Place holder for register address                                    */
    uint16_t              padding;                                     /* Padding                                                              */

    uint32_t              max_system_gain;                             /* Max system gain that can be applied                                  */
    uint32_t              bits_per_sample;                             /* 16 implies no data loss                                              */
    uint32_t              apply_compression;                           /* do we apply compression buffer                                       */
    uint32_t              StreamBufferStartOffset;                     /* additional offset to the start of the buffering data passed to the host after an interrupt detect */
    uint32_t              BufferTriggerOnly;                           /* Added to permit trigger capture only                                 */
    uint32_t              Cmd_TimeOut;                                 /* Initial timeout set for 3 seconds                                    */
    uint32_t              Dflt_SharedMem_Usage;                        /* kWords of shared memory used between cores 2 and 3 values can be in the range 0..4 inclusive */
} system_config_ZM_struct_t;

typedef struct
{
    uint32_t    register_address;   /* Place holder for register address    */
    uint16_t    padding;            /* Padding                              */

    uint32_t    core_id;
    uint32_t    core_rev;
    uint32_t    firmware_id;
    uint32_t    firmware_ver;
    uint32_t    zm;                 /* pointer to system_config_ZM_struct_t */
    uint32_t    xm;                 /* pointer to system_config_XM_struct_t */
    uint32_t    ym;                 /* pointer to system_config_YM_struct_t */
    uint32_t    n_algs;
} dsp_id_hdr_t;

#pragma pack()

typedef struct
{
    const cs47l24_dsp_core_details_t* dsp_core;
    dsp_id_hdr_t                      id_header;
    system_config_XM_struct_t         xm_struct;
    system_config_ZM_struct_t         zm_struct;
    dsp_host_buf_t                    host_buf;
    uint32_t                          xm_struct_reg_addr;
    uint32_t                          zm_struct_reg_addr;
    uint32_t                          host_buf_reg_addr;
    struct unshorten                  unshorten_ctx;
    dsp_word_t                        dsp_word;
    wiced_event_flags_t               offset_events;
    wiced_event_flags_t               pcm_events;
    wiced_semaphore_t                 thread_events;
    wiced_ring_buffer_t               pcm_ring;
    uint32_t                          pcm_bytecount;
    uint32_t                          pcm_hotword_offset;
    uint32_t                          pcm_drop_count;
#ifdef CS47L24_ENABLE_DSP_IRQ_TIMING_STATS
    wiced_time_t                      process_irq_max;
    wiced_time_t                      process_irq_min;
    wiced_time_t                      irq_interval_max;
    wiced_time_t                      irq_interval_min;
    wiced_time_t                      irq_timestamp;
#endif /* CS47L24_ENABLE_DSP_IRQ_TIMING_STATS */
#ifdef CS47L24_ENABLE_DSP_WATCHDOG
    uint32_t                          dsp_watchdog_counter;
#endif /* CS47L24_ENABLE_DSP_WATCHDOG */
    wiced_thread_t                    thread;
    wiced_thread_t*                   thread_ptr;
    volatile wiced_bool_t             thread_stop;
    volatile wiced_bool_t             thread_quit;
    uint8_t                           thread_stack[SCC_THREAD_STACK_SIZE];
    uint8_t*                          pcm_buf;
    dsp_word_buffer_t*                compressed_buf;
    wiced_audio_wupd_streaming_mode_t streaming_mode;
    wiced_bool_t                      dsp_reset_pending;
} dsp_scc_context_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

static dsp_scc_context_t g_scc_context;

/******************************************************
 *            Static Function Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static inline void display_host_buf( void )
{
    /*
     * Let's print host_buf out
     */
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "\n");
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "start                0x%08lX\n", g_scc_context.host_buf_reg_addr);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "X_buf_base           0x%08lX\n", ntohl(g_scc_context.host_buf.X_buf_base));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "X_buf_size           0x%08lX\n", ntohl(g_scc_context.host_buf.X_buf_size));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "X_buf_base2          0x%08lX\n", ntohl(g_scc_context.host_buf.X_buf_base2));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "X_buf_brk            0x%08lX\n", ntohl(g_scc_context.host_buf.X_buf_brk));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "Y_buf_base           0x%08lX\n", ntohl(g_scc_context.host_buf.Y_buf_base));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "wrap                 0x%08lX\n", ntohl(g_scc_context.host_buf.wrap));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "high_water_mark      0x%08lX\n", ntohl(g_scc_context.host_buf.high_water_mark));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "irq_count            0x%08lX\n", ntohl(g_scc_context.host_buf.irq_count));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "irq_ack              0x%08lX\n", ntohl(g_scc_context.host_buf.irq_ack));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "next_write_index     0x%08lX\n", ntohl(g_scc_context.host_buf.next_write_index));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "next_read_index      0x%08lX\n", ntohl(g_scc_context.host_buf.next_read_index));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "error                0x%08lX\n", ntohl(g_scc_context.host_buf.error));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "words_written_1      0x%08lX\n", ntohl(g_scc_context.host_buf.words_written_1));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "words_written_0      0x%08lX\n", ntohl(g_scc_context.host_buf.words_written_0));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "support_ym_overwrite 0x%08lX\n", ntohl(g_scc_context.host_buf.support_ym_overwrite));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "generate_host_IRQ    0x%08lX\n", ntohl(g_scc_context.host_buf.generate_host_IRQ));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "\n");
}


static inline void display_xm_struct( void )
{
    /*
     * Let's print XM config struct
     */
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "\n");
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "start                  0x%08lX\n", g_scc_context.xm_struct_reg_addr);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "plugin_fmw_id          0x%08lX\n", ntohl(g_scc_context.xm_struct.plugin_fmw_id));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "plugin_fmw_vrs         0x%08lX\n", ntohl(g_scc_context.xm_struct.plugin_fmw_vrs));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "adsp2_state            0x%08lX\n", ntohl(g_scc_context.xm_struct.adsp2_state));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "magic                  0x%08lX\n", ntohl(g_scc_context.xm_struct.compression_buf_hdr.magic));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "host_buf               0x%08lX\n", ntohl(g_scc_context.xm_struct.compression_buf_hdr.host_buf));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "high_water_mark        0x%08lX\n", ntohl(g_scc_context.xm_struct.compression_buf_hdr.high_water_mark));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "Streaming_Timeout      0x%08lX\n", ntohl(g_scc_context.xm_struct.Streaming_Timeout));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "Interrupt_and_Stream   0x%08lX\n", ntohl(g_scc_context.xm_struct.Interrupt_and_Stream));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "GPIO_Trigger_Indicator 0x%08lX\n", ntohl(g_scc_context.xm_struct.GPIO_Trigger_Indicator));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "\n");
}


static inline void display_zm_struct( void )
{
    /*
     * Let's print ZM config struct
     */
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "\n");
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "start                   0x%08lX\n", g_scc_context.zm_struct_reg_addr);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "bits_per_sample         0x%08lX\n", ntohl(g_scc_context.zm_struct.bits_per_sample));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "apply_compression       0x%08lX\n", ntohl(g_scc_context.zm_struct.apply_compression));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "StreamBufferStartOffset 0x%08lX\n", ntohl(g_scc_context.zm_struct.StreamBufferStartOffset));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "BufferTriggerOnly       0x%08lX\n", ntohl(g_scc_context.zm_struct.BufferTriggerOnly));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "Cmd_TimeOut             0x%08lX\n", ntohl(g_scc_context.zm_struct.Cmd_TimeOut));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "Dflt_SharedMem_Usage    0x%08lX\n", ntohl(g_scc_context.zm_struct.Dflt_SharedMem_Usage));
    wiced_log_msg(WLF_DRIVER, WICED_LOG_INFO, "\n");
}


static inline wiced_result_t write_dsp_host_buffer_item( cs47l24_device_cmn_data_t* cs47l24, uint32_t item_offset, uint32_t item_value )
{
    wiced_result_t result = WICED_SUCCESS;

    g_scc_context.dsp_word.register_address = htonl(g_scc_context.host_buf_reg_addr + ((item_offset - offsetof(dsp_host_buf_t, X_buf_base)) >> 1));
    g_scc_context.dsp_word.padding          = 0;
    g_scc_context.dsp_word.word             = item_value;

    CS47L24_VERIFY_GOTO( cs47l24_block_write(cs47l24, (uint8_t*)&g_scc_context.dsp_word, sizeof(g_scc_context.dsp_word)), result, _exit );

 _exit:
    return result;
}


static inline wiced_result_t write_xm_struct_item( cs47l24_device_cmn_data_t* cs47l24, uint32_t item_offset, uint32_t item_value )
{
    wiced_result_t result = WICED_SUCCESS;

    g_scc_context.dsp_word.register_address = htonl(g_scc_context.xm_struct_reg_addr + ((item_offset - offsetof(system_config_XM_struct_t, sys_enable)) >> 1));
    g_scc_context.dsp_word.padding          = 0;
    g_scc_context.dsp_word.word             = item_value;

    CS47L24_VERIFY_GOTO( cs47l24_block_write(cs47l24, (uint8_t*)&g_scc_context.dsp_word, sizeof(g_scc_context.dsp_word)), result, _exit );

 _exit:
    return result;
}


static inline wiced_result_t write_zm_struct_item( cs47l24_device_cmn_data_t* cs47l24, uint32_t item_offset, uint32_t item_value )
{
    wiced_result_t result = WICED_SUCCESS;

    g_scc_context.dsp_word.register_address = htonl(g_scc_context.zm_struct_reg_addr + ((item_offset - offsetof(system_config_ZM_struct_t, max_system_gain)) >> 1));
    g_scc_context.dsp_word.padding          = 0;
    g_scc_context.dsp_word.word             = item_value;

    CS47L24_VERIFY_GOTO( cs47l24_block_write(cs47l24, (uint8_t*)&g_scc_context.dsp_word, sizeof(g_scc_context.dsp_word)), result, _exit );

 _exit:
    return result;
}


static inline wiced_result_t read_dsp_host_buffer_item( cs47l24_device_cmn_data_t* cs47l24, uint32_t item_offset, uint32_t* item_value )
{
    wiced_result_t result = WICED_SUCCESS;

    g_scc_context.dsp_word.register_address = htonl(g_scc_context.host_buf_reg_addr + ((item_offset - offsetof(dsp_host_buf_t, X_buf_base)) >> 1));
    g_scc_context.dsp_word.padding          = 0;
    g_scc_context.dsp_word.word             = 0UL;

    CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)&g_scc_context.dsp_word, sizeof(g_scc_context.dsp_word)), result, _exit );
    *item_value = g_scc_context.dsp_word.word;

 _exit:
    return result;
}


static inline wiced_result_t read_xm_struct_item( cs47l24_device_cmn_data_t* cs47l24, uint32_t item_offset, uint32_t* item_value )
{
    wiced_result_t result = WICED_SUCCESS;

    g_scc_context.dsp_word.register_address = htonl(g_scc_context.xm_struct_reg_addr + ((item_offset - offsetof(system_config_XM_struct_t, sys_enable)) >> 1));
    g_scc_context.dsp_word.padding          = 0;
    g_scc_context.dsp_word.word             = 0UL;

    CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)&g_scc_context.dsp_word, sizeof(g_scc_context.dsp_word)), result, _exit );
    *item_value = g_scc_context.dsp_word.word;

 _exit:
    return result;
}


static inline wiced_result_t get_dsp_address( uint32_t read_index, uint32_t* dsp_address, uint32_t* size )
{
    wiced_result_t result     = WICED_SUCCESS;
    uint32_t       X_buf_size = ntohl(g_scc_context.host_buf.X_buf_size);
    uint32_t       X_buf_brk  = ntohl(g_scc_context.host_buf.X_buf_brk);
    uint32_t       wrap       = ntohl(g_scc_context.host_buf.wrap);

    if (read_index < X_buf_size)
    {
        *dsp_address = g_scc_context.dsp_core->xm_start + (ntohl(g_scc_context.host_buf.X_buf_base) * 2) + (read_index * 2);
        *size        = X_buf_size - read_index;
    }
    else if ((read_index >= X_buf_size) && (read_index < X_buf_brk))
    {
        *dsp_address = g_scc_context.dsp_core->xm_start + (ntohl(g_scc_context.host_buf.X_buf_base2) * 2) + ((read_index - X_buf_size) * 2);
        *size        = X_buf_brk - read_index;
    }
    else if ((read_index >= X_buf_brk) && (read_index < wrap))
    {
        *dsp_address = g_scc_context.dsp_core->ym_start + (ntohl(g_scc_context.host_buf.Y_buf_base) * 2) + ((read_index - X_buf_brk) * 2);
        *size        = wrap - read_index;
    }
    else
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "get_dsp_address: requested read_index 0x%08lXx is not in defined buffers\n", read_index);
        result = WICED_ERROR;
    }

    return result;
}


inline static wiced_result_t cs47l24_dsp_scc_clear_interrupt( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result;
    uint16_t       irq_status;

    /*
     * Clear DSP interrupt
     */
    CS47L24_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_STATUS_2, &irq_status), result, _exit);
    CS47L24_VERIFY_GOTO( cs47l24_reg_write(cs47l24, CS47L24_INTERRUPT_STATUS_2, (irq_status & ~CS47L24_DSP_IRQ1_EINT1_MASK) | CS47L24_DSP_IRQ1_EINT1), result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_clear_interrupt: failed with %d\n", result);
    }
    return result;
}


static wiced_result_t cs47l24_dsp_scc_interrupt_mask_and_clear( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result     = WICED_SUCCESS;
    uint16_t       irq_status;

    /*
     * Mask & Clear 'Boot done' interrupt
     */
    CS47L24_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_INTERRUPT_STATUS_5_MASK, CS47L24_IM_BOOT_DONE_EINT1_MASK, CS47L24_IM_BOOT_DONE_EINT1), result, _exit);
    CS47L24_VERIFY_GOTO( cs47l24_reg_read(cs47l24, CS47L24_INTERRUPT_STATUS_5, &irq_status), result, _exit);
    CS47L24_VERIFY_GOTO( cs47l24_reg_write(cs47l24, CS47L24_INTERRUPT_STATUS_5, (irq_status & ~CS47L24_BOOT_DONE_EINT1_MASK) | CS47L24_BOOT_DONE_EINT1), result, _exit);

    /*
     * Mask DSP IRQ
     */
    CS47L24_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_INTERRUPT_STATUS_2_MASK, CS47L24_IM_DSP_IRQ1_EINT1_MASK, CS47L24_IM_DSP_IRQ1_EINT1), result, _exit);
    /*
     * Clear DSP IRQ
     */
    CS47L24_VERIFY_GOTO( cs47l24_dsp_scc_clear_interrupt(cs47l24), result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_interrupt_mask_and_clear: failed with %d\n", result);
    }
    return result;
}


static wiced_result_t cs47l24_dsp_scc_interrupt_unmask( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result = WICED_SUCCESS;

    /*
     * Unmask IRQ
     */
    CS47L24_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, CS47L24_INTERRUPT_STATUS_2_MASK, CS47L24_IM_DSP_IRQ1_EINT1_MASK, 0x0), result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_interrupt_unmask: failed with %d\n", result);
    }
    return result;
}


#ifdef CS47L24_ENABLE_DSP_WATCHDOG
static wiced_result_t cs47l24_monitor_watchdog_counter( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result  = WICED_SUCCESS;
    uint32_t       counter;

    if (g_scc_context.host_buf_reg_addr != 0 && (g_scc_context.dsp_reset_pending = WICED_FALSE))
    {
        CS47L24_VERIFY_GOTO( read_xm_struct_item(cs47l24, offsetof(system_config_XM_struct_t, adsp2_watchdog), &counter), result, _exit );
        if (counter == g_scc_context.dsp_watchdog_counter)
        {
            CS47L24_VERIFY_GOTO( cs47l24_dsp_core_reset(cs47l24, CS47L24_FIRMWARE_DSP2), result, _exit );
            wiced_log_msg(WLF_DRIVER, WICED_LOG_WARNING, "\ncs47l24_monitor_watchdog_counter: ******** DSP2 core was reset (0x%08lx)! *********\n\n", counter);
        }
        else
        {
            g_scc_context.dsp_watchdog_counter = counter;
        }
    }

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_monitor_watchdog_counter: failed with %d\n", result);
    }
    return result;
}
#endif /* CS47L24_ENABLE_DSP_WATCHDOG */


static void cs47l24_irq_worker( wiced_thread_arg_t arg )
{
    cs47l24_device_cmn_data_t* cs47l24 = (cs47l24_device_cmn_data_t*)arg;
    wiced_result_t             result;

    do
    {
        result = wiced_rtos_get_semaphore(&g_scc_context.thread_events, DSP_IRQ_WORKER_WAIT_TIMEOUT_MSECS);
        if (((result != WICED_SUCCESS) && (result != WICED_TIMEOUT)) || (g_scc_context.thread_quit == WICED_TRUE))
        {
            break;
        }
        if (result == WICED_TIMEOUT)
        {
            CS47L24_MONITOR_WATCHDOG();
            continue;
        }
        cs47l24_dsp_scc_process_interrupt(cs47l24);
    } while (g_scc_context.thread_quit == WICED_FALSE);

    wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_irq_worker: done.\n", result);

    WICED_END_OF_CURRENT_THREAD();
}


static void cs47l24_gpio_irq_handler(void* arg)
{
#ifdef CS47L24_ENABLE_DSP_IRQ_TIMING_STATS
    wiced_time_t tick;
    wiced_time_t tack;

    wiced_time_get_time(&tick);

    if (g_scc_context.irq_timestamp == 0)
    {
        g_scc_context.irq_timestamp = tick;
    }
    else
    {
        tack = tick - g_scc_context.irq_timestamp;
        g_scc_context.irq_timestamp = tick;

        if (tack > g_scc_context.irq_interval_max)
        {
            g_scc_context.irq_interval_max = tack;
        }
        if (tack < g_scc_context.irq_interval_min)
        {
            g_scc_context.irq_interval_min = tack;
        }
    }
#endif /* CS47L24_ENABLE_DSP_IRQ_TIMING_STATS */

    if (g_scc_context.thread_ptr != NULL)
    {
        wiced_rtos_set_semaphore(&g_scc_context.thread_events);
    }
}


wiced_result_t cs47l24_dsp_scc_init( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result = WICED_SUCCESS;

    CS47L24_VERIFY_GOTO( cs47l24_dsp_scc_interrupt_mask_and_clear(cs47l24), result, _exit );
    CS47L24_VERIFY_GOTO( cs47l24_dsp_scc_interrupt_unmask(cs47l24), result, _exit );

    if (cs47l24->irq != WICED_GPIO_NONE)
    {
        CS47L24_VERIFY_GOTO( wiced_gpio_init( cs47l24->irq, INPUT_HIGH_IMPEDANCE ), result, _exit );
        CS47L24_VERIFY_GOTO( wiced_gpio_input_irq_enable( cs47l24->irq, IRQ_TRIGGER_FALLING_EDGE, cs47l24_gpio_irq_handler, cs47l24 ), result, _exit );
    }

    memset(&g_scc_context, 0, sizeof(g_scc_context));
    g_scc_context.dsp_core = &g_cs47l24_dsp[CS47L24_FIRMWARE_DSP2];
    /*
     * Set default driver behavior (SPI streaming details)
     */
    g_scc_context.streaming_mode.interrupt_and_stream       = DSP_INTERRUPT_AND_STREAM_DEFAULT;
    g_scc_context.streaming_mode.streaming_timeout          = DSP_STREAMING_TIMEOUT_DEFAULT;
    g_scc_context.streaming_mode.stream_buffer_start_offset = DSP_STREAM_START_OFFSET_DEFAULT;
    g_scc_context.streaming_mode.buffer_trigger_only        = DSP_BUFFERTRIGGERONLY_DEFAULT;

#ifdef CS47L24_ENABLE_DSP_IRQ_TIMING_STATS
    g_scc_context.process_irq_min  = (wiced_time_t)-1;
    g_scc_context.irq_interval_min = (wiced_time_t)-1;
#endif /* CS47L24_ENABLE_DSP_IRQ_TIMING_STATS */
    CS47L24_VERIFY_GOTO( wiced_rtos_init_event_flags(&g_scc_context.offset_events), result, _exit );
    CS47L24_VERIFY_GOTO( wiced_rtos_init_event_flags(&g_scc_context.pcm_events), result, _deinit_offset_flags );
    CS47L24_VERIFY_GOTO( wiced_rtos_init_semaphore(&g_scc_context.thread_events), result, _deinit_pcm_flags );
    CS47L24_VERIFY_GOTO( init_unshorten(&g_scc_context.unshorten_ctx), result, _deinit_thread_flags );
    CS47L24_VERIFY_GOTO( wiced_rtos_create_thread_with_stack(&g_scc_context.thread, SCC_THREAD_PRIORITY, "cs47l24_thread", cs47l24_irq_worker,
                         g_scc_context.thread_stack, SCC_THREAD_STACK_SIZE, cs47l24), result, _deinit_unshorten);
    g_scc_context.thread_ptr = &g_scc_context.thread;

 _deinit_unshorten:
    if (result != WICED_SUCCESS)
    {
        free_unshorten(&g_scc_context.unshorten_ctx);
    }

 _deinit_thread_flags:
    if (result != WICED_SUCCESS)
    {
        wiced_rtos_deinit_semaphore(&g_scc_context.thread_events);
    }

 _deinit_pcm_flags:
    if (result != WICED_SUCCESS)
    {
        wiced_rtos_deinit_event_flags(&g_scc_context.pcm_events);
    }

 _deinit_offset_flags:
    if (result != WICED_SUCCESS)
    {
        wiced_rtos_deinit_event_flags(&g_scc_context.offset_events);
    }

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_init: failed with %d\n", result);
    }
    return result;
}


wiced_result_t cs47l24_dsp_scc_deinit( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result = WICED_SUCCESS;

    if (cs47l24->irq != WICED_GPIO_NONE)
    {
        wiced_gpio_input_irq_disable(cs47l24->irq);
        wiced_gpio_deinit(cs47l24->irq);
    }

    if (g_scc_context.thread_ptr != NULL)
    {
        g_scc_context.thread_quit = WICED_TRUE;
        wiced_rtos_set_semaphore(&g_scc_context.thread_events);
        wiced_rtos_thread_force_awake(g_scc_context.thread_ptr);
        wiced_rtos_thread_join(g_scc_context.thread_ptr);
        wiced_rtos_delete_thread(g_scc_context.thread_ptr);
        g_scc_context.thread_ptr = NULL;
    }

    free_unshorten(&g_scc_context.unshorten_ctx);
    ring_buffer_deinit(&g_scc_context.pcm_ring);
    wiced_rtos_deinit_event_flags(&g_scc_context.offset_events);
    wiced_rtos_deinit_event_flags(&g_scc_context.pcm_events);
    wiced_rtos_deinit_semaphore(&g_scc_context.thread_events);
    free(g_scc_context.pcm_buf);
    free(g_scc_context.compressed_buf);

    memset(&g_scc_context, 0, sizeof(g_scc_context));

    return result;
}


wiced_result_t cs47l24_dsp_scc_setup_pre( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result     = WICED_SUCCESS;
    uint32_t       loop_count;

    /*
     * Read DSP core and firmware info at the beginning of XM
     */

    loop_count = 0;
    do
    {
        g_scc_context.id_header.register_address = htonl(g_scc_context.dsp_core->xm_start);
        g_scc_context.id_header.padding          = 0;
        CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)&g_scc_context.id_header, sizeof(g_scc_context.id_header)), result, _exit );
        if (g_scc_context.id_header.firmware_ver == 0UL)
        {
            loop_count++;
            wiced_rtos_delay_milliseconds(1);
        }
    } while ((g_scc_context.id_header.firmware_ver == 0UL) && (loop_count < 20));

    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_dsp_scc_setup_pre: FWID=0x%08lX, FWVER=0x%08lX, XM=0x%08lX ZM=0x%08lX, retries=%lu\n",
                  ntohl(g_scc_context.id_header.firmware_id), ntohl(g_scc_context.id_header.firmware_ver),
                  ntohl(g_scc_context.id_header.xm), ntohl(g_scc_context.id_header.zm), loop_count);

    switch (ntohl(g_scc_context.id_header.firmware_ver))
    {
        case DSP_SCC_FIRMWARE_VER_010700:
        case DSP_SCC_FIRMWARE_VER_010701:
        case DSP_SCC_FIRMWARE_VER_010702:
        case DSP_SCC_FIRMWARE_VER_010800:
        case DSP_SCC_FIRMWARE_VER_010801:
        case DSP_SCC_FIRMWARE_VER_010803:
            break;

        default:
            result = WICED_ERROR;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_pre: unsupported firmware version 0x%08lX, retries=%lu\n",
                          ntohl(g_scc_context.id_header.firmware_ver), loop_count);
            goto _exit;
            break;
    }

    g_scc_context.xm_struct_reg_addr         = g_scc_context.dsp_core->xm_start + (2 * ntohl(g_scc_context.id_header.xm));
    g_scc_context.xm_struct.register_address = htonl(g_scc_context.xm_struct_reg_addr);
    g_scc_context.xm_struct.padding          = 0;
    CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)&g_scc_context.xm_struct, sizeof(g_scc_context.xm_struct)), result, _exit );

    if (ntohl(g_scc_context.xm_struct.compression_buf_hdr.magic) != DSP_HOST_BUFFER_HEADER_MAGIC)
    {
        result = WICED_ERROR;
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_pre: incorrect host_buf_hdr magic 0x%08lX\n",
                      ntohl(g_scc_context.xm_struct.compression_buf_hdr.magic));
        goto _exit;
    }

    /*
     * Keep record of system_config_ZM_struct_t address in HOST byte order;
     * It's needed by write_zm_struct_item() call
     */
    g_scc_context.zm_struct_reg_addr = g_scc_context.dsp_core->zm_start + (2 * ntohl(g_scc_context.id_header.zm));

    if (g_scc_context.streaming_mode.interrupt_and_stream > 0)
    {
        if (g_scc_context.streaming_mode.stream_buffer_start_offset >= 0)
        {
            if (g_scc_context.streaming_mode.stream_buffer_start_offset > DSP_STREAM_START_OFFSET_MAX)
            {
                g_scc_context.streaming_mode.stream_buffer_start_offset = DSP_STREAM_START_OFFSET_MAX;
            }
        }
        else
        {
            if (g_scc_context.streaming_mode.stream_buffer_start_offset < DSP_STREAM_START_OFFSET_MIN)
            {
                g_scc_context.streaming_mode.stream_buffer_start_offset = DSP_STREAM_START_OFFSET_MIN;
            }
            g_scc_context.streaming_mode.stream_buffer_start_offset = labs(g_scc_context.streaming_mode.stream_buffer_start_offset);
            g_scc_context.streaming_mode.stream_buffer_start_offset = (uint32_t)(1 << 24) - g_scc_context.streaming_mode.stream_buffer_start_offset;
        }

        /*
         * We're going to be streaming from SPI
         */
        if (g_scc_context.pcm_buf == NULL)
        {
            g_scc_context.pcm_buf = malloc(DSP_HOST_PCM_BUFFER_SIZE * sizeof(uint8_t));
            if (g_scc_context.pcm_buf == NULL)
            {
                result = WICED_OUT_OF_HEAP_SPACE;
                wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_pre: malloc(pcm_buf) failed\n");
                goto _exit;
            }
            g_scc_context.compressed_buf = malloc(sizeof(dsp_word_buffer_t));
            if (g_scc_context.compressed_buf == NULL)
            {
                result = WICED_OUT_OF_HEAP_SPACE;
                wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_pre: malloc(g_scc_context.compressed_buf) failed\n");
                free(g_scc_context.pcm_buf);
                g_scc_context.pcm_buf = NULL;
                goto _exit;
            }
            CS47L24_VERIFY_GOTO( ring_buffer_init(&g_scc_context.pcm_ring, g_scc_context.pcm_buf, (uint32_t)DSP_HOST_PCM_BUFFER_SIZE), result, _exit );
        }
        wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_dsp_scc_setup_pre: SPI streaming is on\n", result);
    }
    else
    {
        /*
         * Make sure streaming timeout is very small but NOT zero
         */
        g_scc_context.streaming_mode.streaming_timeout = DSP_NO_STREAMING_TIMEOUT_DEFAULT;

        /*
         * We're not streaming; set start offset to 0 anyway
         */
        g_scc_context.streaming_mode.stream_buffer_start_offset = 0;

        /*
         * Not streaming from SPI; get rid of PCM storage
         */
        if (g_scc_context.pcm_buf != NULL)
        {
            CS47L24_VERIFY_GOTO( ring_buffer_deinit(&g_scc_context.pcm_ring), result, _exit );
            free(g_scc_context.pcm_buf);
            g_scc_context.pcm_buf = NULL;
        }
        if (g_scc_context.compressed_buf != NULL)
        {
            free(g_scc_context.compressed_buf);
            g_scc_context.compressed_buf = NULL;
        }
        wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_dsp_scc_setup_pre: SPI streaming is off\n", result);
    }

    g_scc_context.streaming_mode.streaming_timeout &= SC_FF_DUET_REG_MASK;

    /*
     * Update XM and ZM parameters with our own values
     */

    CS47L24_VERIFY_GOTO( write_xm_struct_item(cs47l24, offsetof(system_config_XM_struct_t, compression_buf_hdr.high_water_mark), htonl(DSP_HOST_HIGH_WATER_MARK_DEFAULT)), result, _exit);
    CS47L24_VERIFY_GOTO( write_xm_struct_item(cs47l24, offsetof(system_config_XM_struct_t, Streaming_Timeout), htonl(g_scc_context.streaming_mode.streaming_timeout)), result, _exit);
    CS47L24_VERIFY_GOTO( write_xm_struct_item(cs47l24, offsetof(system_config_XM_struct_t, Interrupt_and_Stream), htonl((uint32_t)g_scc_context.streaming_mode.interrupt_and_stream)), result, _exit);
    CS47L24_VERIFY_GOTO( write_xm_struct_item(cs47l24, offsetof(system_config_XM_struct_t, GPIO_Trigger_Indicator), htonl(DSP_GPIO_TRIGGER_INDICATOR_DEFAULT)), result, _exit);
    CS47L24_VERIFY_GOTO( write_zm_struct_item(cs47l24, offsetof(system_config_ZM_struct_t, StreamBufferStartOffset), htonl(g_scc_context.streaming_mode.stream_buffer_start_offset)), result, _exit);
    CS47L24_VERIFY_GOTO( write_zm_struct_item(cs47l24, offsetof(system_config_ZM_struct_t, BufferTriggerOnly), htonl((uint32_t)g_scc_context.streaming_mode.buffer_trigger_only)), result, _exit);
    CS47L24_VERIFY_GOTO( write_zm_struct_item(cs47l24, offsetof(system_config_ZM_struct_t, Cmd_TimeOut), htonl(DSP_CMD_TIMEOUT_DEFAULT)), result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_pre: failed with %d\n", result);
    }
    return result;
}


wiced_result_t cs47l24_dsp_scc_setup_post( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result     = WICED_SUCCESS;
    uint32_t       loop_count;

    /*
     * Get system_config_XM_struct_t address from "xm" field of dsp_id_hdr_t struct
     */

    loop_count = 0;
    while ((g_scc_context.xm_struct.compression_buf_hdr.host_buf == 0UL) && (loop_count < 5))
    {
        g_scc_context.xm_struct.register_address = htonl(g_scc_context.xm_struct_reg_addr);
        g_scc_context.xm_struct.padding          = 0;
        CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)&g_scc_context.xm_struct, sizeof(g_scc_context.xm_struct)), result, _exit );
        if (ntohl(g_scc_context.xm_struct.compression_buf_hdr.magic) != DSP_HOST_BUFFER_HEADER_MAGIC)
        {
            result = WICED_ERROR;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_post: incorrect host_buf_hdr magic 0x%08lX\n",
                          ntohl(g_scc_context.xm_struct.compression_buf_hdr.magic));
            goto _exit;
        }
        if (g_scc_context.xm_struct.compression_buf_hdr.host_buf == 0UL)
        {
            loop_count++;
            wiced_rtos_delay_milliseconds(1);
        }
    }

    if (loop_count == 5)
    {
        result = WICED_ERROR;
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_post: host_buf is invalid\n");
        goto _exit;
    }

    if (ntohl(g_scc_context.xm_struct.adsp2_state) != ADSP2_FIRMWARE_STATE_INIT_OK)
    {
        result = WICED_ERROR;
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_post: dsp is not running STATE=0x%08lX\n", ntohl(g_scc_context.xm_struct.adsp2_state));
        goto _exit;
    }

    /*
     * Get dsp_host_buf_t structure
     * Keep a record of the address for that structure in network byte order
     */

    g_scc_context.host_buf_reg_addr         = g_scc_context.dsp_core->xm_start + (2 * ntohl(g_scc_context.xm_struct.compression_buf_hdr.host_buf));

 _exit:
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_setup_post: failed with %d\n", result);
    }
    else
    {
#ifdef CS47L24_ENABLE_DSP_DEBUG_TRACE
        g_scc_context.host_buf.register_address = htonl(g_scc_context.host_buf_reg_addr);
        CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)&g_scc_context.host_buf, sizeof(g_scc_context.host_buf)), result, _exit );

        g_scc_context.zm_struct.register_address = htonl(g_scc_context.zm_struct_reg_addr);
        g_scc_context.zm_struct.padding      = 0;
        CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)&g_scc_context.zm_struct, sizeof(g_scc_context.zm_struct)), result, _exit );

        display_xm_struct();
        display_zm_struct();
        display_host_buf();
#endif /* CS47L24_ENABLE_DSP_DEBUG_TRACE */
    }

    return result;
}


wiced_result_t cs47l24_dsp_scc_process_interrupt( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result          = WICED_SUCCESS;
    uint32_t       words_available;
    uint32_t       read_index;
    uint32_t       write_index;
    uint32_t       words_to_read;
    uint32_t       buffer_max;
    uint32_t       read_address;
    uint8_t*       ptr_in;
    uint8_t*       ptr_out;
    uint32_t       words_xfered;
    uint32_t       xfer_size;
    uint32_t       block_read_size;
    uint32_t       i;
    int            unshorten_rc;
    uint32_t       dsp_error;
#ifdef CS47L24_ENABLE_DSP_IRQ_TIMING_STATS
    wiced_time_t   tick;
    wiced_time_t   tack;

    wiced_time_get_time(&tick);
#endif /* CS47L24_ENABLE_DSP_IRQ_TIMING_STATS */

    /*
     * Update dsp_host_buffer struct
     */
    g_scc_context.host_buf.register_address = htonl(g_scc_context.host_buf_reg_addr);
    g_scc_context.host_buf.padding          = 0;
    CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)&g_scc_context.host_buf, sizeof(g_scc_context.host_buf)), result, _exit );

    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_dsp_scc_process_interrupt: irq_count=%lu\n", (ntohl(g_scc_context.host_buf.irq_count) >> 1));

    dsp_error = ntohl(g_scc_context.host_buf.error);

    if (dsp_error != DSP_SCC_BUFFER_NO_ERROR)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_process_interrupt: ERROR=0x%08lX\n", dsp_error);
        /*
         * There was an error; bail out
         */
        result = WICED_ERROR;
        goto _exit;
    }

    if (((ntohl(g_scc_context.host_buf.irq_count) >> 1) == 1) && g_scc_context.thread_stop)
    {
        /*
         * First trigger interrupt; reset thread_stop flag
         */
        g_scc_context.thread_stop = WICED_FALSE;
        wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4,"cs47l24_dsp_scc_process_interrupt: >>> first interrupt <<<\n");
    }

    read_index = ntohl(g_scc_context.host_buf.next_read_index);

    if ((read_index & DSP_WORD_SIGN_MASK) != 0)
    {
        /*
         * next_read_index has a negative value; bail out
         */
        if (g_scc_context.streaming_mode.interrupt_and_stream > 0)
        {
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_process_interrupt: next_read_index is negative (0x%08lX)\n", read_index);
        }
        result = WICED_ERROR;
        goto _exit;

    }

    write_index = ntohl(g_scc_context.host_buf.next_write_index);

    /*
     * Work out words available
     */

    if (read_index <= write_index)
    {
        words_available = write_index - read_index;
    }
    else
    {
        words_available = ntohl(g_scc_context.host_buf.wrap) - (read_index - write_index);
    }

    wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_dsp_scc_process_interrupt: read=0x%08lX, write=0x%08lX, avail=%lu\n", read_index, write_index, words_available);

    if (words_available == 0)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR, "cs47l24_dsp_scc_process_interrupt: spurious interrupt\n");
        goto _update_indexes;
    }

    words_to_read = words_available;

    while (words_to_read > 0)
    {
        if (g_scc_context.thread_stop || g_scc_context.thread_quit)
        {
            wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_dsp_scc_process_interrupt: thread stopping/quitting\n");
            goto _exit;
        }
        /*
         * Get codec address for memory location and max available words in segment
         */
        read_address = 0;
        buffer_max   = 0;
        CS47L24_VERIFY_GOTO( get_dsp_address(read_index, &read_address, &buffer_max), result, _exit );

        words_xfered = MIN(words_to_read, buffer_max);
        xfer_size    = (words_xfered * sizeof(uint32_t));
        wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4,"cs47l24_dsp_scc_process_interrupt: read_index=0x%08lX, DSP_addr=0x%08lX\n", read_index, read_address);
        wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4,"cs47l24_dsp_scc_process_interrupt: segment=%lu, words_to_read=%lu,words_xfered=%lu(%lu bytes)\n",
                      buffer_max, words_to_read, words_xfered, xfer_size);

        /*
         * Read words from the DSP
         */
        if ((g_scc_context.compressed_buf == NULL) || (xfer_size > sizeof(g_scc_context.compressed_buf->buffer)))
        {
            result = WICED_ERROR;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR,"cs47l24_dsp_scc_process_interrupt: compressed buffer is too small %lu\n", buffer_max);
            goto _exit;
        }
        block_read_size = xfer_size + sizeof(g_scc_context.compressed_buf->register_address) + sizeof(g_scc_context.compressed_buf->padding);
        g_scc_context.compressed_buf->register_address = htonl(read_address);
        g_scc_context.compressed_buf->padding          = 0;
        CS47L24_VERIFY_GOTO( cs47l24_block_read(cs47l24, (uint8_t*)g_scc_context.compressed_buf, block_read_size), result, _exit );

        words_to_read -= words_xfered;
        read_index    += words_xfered;

        /*
         * Check if we reached the end of the buffer and need to go back to 0
         */
        if (read_index == ntohl(g_scc_context.host_buf.wrap))
        {
            read_index = 0;
        }
        else if (read_index > ntohl(g_scc_context.host_buf.wrap))
        {
            result = WICED_ERROR;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR,"cs47l24_dsp_scc_process_interrupt: pointer error, read_index=0x%08lX\n", read_index);
            goto _exit;
        }

        ptr_in  = (uint8_t*)g_scc_context.compressed_buf->buffer;
        ptr_out = (uint8_t*)g_scc_context.compressed_buf->buffer;

        for (i = 0; i < words_xfered; i++)
        {
            /*
             * Go to host byte order and drop 4th byte since DSP word is only 24-bit
             */
            ptr_out[0]  = ptr_in[3];              /* Store 4th byte into first byte                */
            ptr_in[3]   = ptr_in[1];              /* Now use 4th byte as temp storage for 2nd byte */
            ptr_out[1]  = ptr_in[2];              /* Store 3rd byte into 2nd byte                  */
            ptr_out[2]  = ptr_in[3];              /* Store 4th byte content into 3rd byte          */
            ptr_out    += (3 * sizeof(uint8_t));  /* Point to 4th byte                             */
            ptr_in     += sizeof(uint32_t);       /* Point to beginning of next 4-byte word        */
        }

        unshorten_rc = unshorten_supply_input(&g_scc_context.unshorten_ctx, g_scc_context.compressed_buf->buffer, words_xfered * 3 * sizeof(uint8_t));
        if (unshorten_rc != 0)
        {
            result = WICED_ERROR;
            wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR,"cs47l24_dsp_scc_process_interrupt: unshorten_supply_input() failed\n");
            goto _exit;
        }

        do
        {
            if (g_scc_context.thread_stop || g_scc_context.thread_quit)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_dsp_scc_process_interrupt: thread stopping/quitting\n");
                goto _exit;
            }

            unshorten_rc = unshorten_process(&g_scc_context.unshorten_ctx);
            if (unshorten_rc & UNSHORTEN_CORRUPT)
            {
                result = WICED_ERROR;
                wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR,"cs47l24_dsp_scc_process_interrupt: unshorten_process() corrupt\n");
                reset_unshorten(&g_scc_context.unshorten_ctx);
                break;
            }
            else if (unshorten_rc & UNSHORTEN_INPUT_REQUIRED)
            {
                wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4,"cs47l24_dsp_scc_process_interrupt: unshorten_process() need input\n");
                break;
            }
            else if (unshorten_rc & UNSHORTEN_OUTPUT_AVAILABLE)
            {
                uint32_t loop_count;
                uint32_t sample_count;
                uint8_t* sample_buf;
                uint32_t bytes_written;

                sample_buf                   = (uint8_t*)unshorten_extract_output(&g_scc_context.unshorten_ctx);
                sample_count                 = UNSHORTEN_OUTPUT_SIZE(unshorten_rc) * sizeof(Sample);
                bytes_written                = 0;
                g_scc_context.pcm_bytecount += sample_count;
                loop_count                   = 0;

                do
                {
                    if (g_scc_context.thread_stop || g_scc_context.thread_quit)
                    {
                        wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4, "cs47l24_dsp_scc_process_interrupt: thread stopping/quitting\n");
                        goto _exit;
                    }
                    bytes_written += ring_buffer_write(&g_scc_context.pcm_ring, (const uint8_t*)&sample_buf[bytes_written], sample_count - bytes_written);
                    if (ring_buffer_used_space(&g_scc_context.pcm_ring) >= DSP_HOST_PCM_BUFFER_THRESHOLD)
                    {
                        wiced_rtos_set_event_flags(&g_scc_context.pcm_events, DSP_HOST_NEW_DATA_FLAG);
                    }
                    if (bytes_written < sample_count)
                    {
                        loop_count++;
                        wiced_rtos_delay_milliseconds(1);
                    }
                } while ((bytes_written < sample_count) && (loop_count < DSP_PCM_RING_WRITE_RETRY_COUNT));

                if (bytes_written < sample_count)
                {
                    g_scc_context.pcm_drop_count += (sample_count - bytes_written);
                }
            }
        } while (unshorten_rc & UNSHORTEN_OUTPUT_AVAILABLE);
    }

    if (((ntohl(g_scc_context.host_buf.irq_count) >> 1) == 1) && (g_scc_context.pcm_hotword_offset == 0))
    {
        /*
         * Done with very first interrupt containing hotword; record PCM byte count
         */
        g_scc_context.pcm_hotword_offset = g_scc_context.pcm_bytecount;
        wiced_rtos_set_event_flags(&g_scc_context.offset_events, DSP_HOST_NEW_OFFSET_FLAG);
        /*
         * Forcibly set semaphore in anticipation of second interrupt
         */
        wiced_rtos_set_semaphore(&g_scc_context.thread_events);
        wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4,"cs47l24_dsp_scc_process_interrupt: off=%lu\n", g_scc_context.pcm_hotword_offset);
    }

 _update_indexes:
    /*
     * Clear DSP interrupt
     */
    CS47L24_VERIFY_GOTO( cs47l24_dsp_scc_clear_interrupt(cs47l24), result, _exit);

    /*
     * Update read index
     */
    g_scc_context.host_buf.next_read_index = g_scc_context.host_buf.next_write_index;
    CS47L24_VERIFY_GOTO( write_dsp_host_buffer_item(cs47l24, offsetof(dsp_host_buf_t, next_read_index), g_scc_context.host_buf.next_read_index), result, _exit);

    /*
     * Acknowledge interrupt
     */
    g_scc_context.host_buf.irq_ack = htonl((ntohl(g_scc_context.host_buf.irq_count) | 1));
    CS47L24_VERIFY_GOTO( write_dsp_host_buffer_item(cs47l24, offsetof(dsp_host_buf_t, irq_ack), g_scc_context.host_buf.irq_ack), result, _exit);

 _exit:
    if (result != WICED_SUCCESS)
    {
        /*
         * Clear DSP interrupt
         */
        cs47l24_dsp_scc_clear_interrupt(cs47l24);

        wiced_log_msg(WLF_DRIVER, WICED_LOG_DEBUG4,"cs47l24_dsp_scc_process_interrupt: failed with %d\n", result);
    }
#ifdef CS47L24_ENABLE_DSP_IRQ_TIMING_STATS
    wiced_time_get_time(&tack);
    tack -= tick;
    if (tack > g_scc_context.process_irq_max)
    {
        g_scc_context.process_irq_max = tack;
    }
    if (tack < g_scc_context.process_irq_min)
    {
        g_scc_context.process_irq_min = tack;
    }
#endif /* CS47L24_ENABLE_DSP_IRQ_TIMING_STATS */
    return result;
}


static wiced_result_t cs47l24_dsp_scc_reset( cs47l24_device_cmn_data_t* cs47l24 )
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t       events;

    UNUSED_PARAMETER(events);

    /*
     * We're about to reset the DSP
     */
    g_scc_context.dsp_reset_pending = WICED_TRUE;

    /*
     * Mask and Clear interrupt
     */
    CS47L24_VERIFY_GOTO( cs47l24_dsp_scc_interrupt_mask_and_clear(cs47l24), result, _exit );

    /*
     * Flag irq worker thread down
     */
    g_scc_context.thread_stop = WICED_TRUE;

    /*
     * Disable DSP
     */
    CS47L24_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_scc_context.dsp_core->dsp_control1, CS47L24_DSP_CORE_ENA_MASK, 0), result, _exit);

    /*
     * Clear IRQ worker events
     */
    while (wiced_rtos_get_semaphore(&g_scc_context.thread_events, WICED_NO_WAIT) == WICED_SUCCESS);

    /*
     * Go through pre-setup again
     */
    cs47l24_dsp_scc_setup_pre(cs47l24);

    if (g_scc_context.pcm_buf != NULL)
    {
        /*
         * Reset ring buffer, reset unshorten, reset PCM byte count
         */
        CS47L24_VERIFY_GOTO( ring_buffer_deinit(&g_scc_context.pcm_ring), result, _exit );
        CS47L24_VERIFY_GOTO( ring_buffer_init(&g_scc_context.pcm_ring, g_scc_context.pcm_buf, (uint32_t)DSP_HOST_PCM_BUFFER_SIZE), result, _exit );
    }
    /*
     * Reset unshorten, reset PCM byte count
     */
    reset_unshorten(&g_scc_context.unshorten_ctx);

    if (g_scc_context.pcm_drop_count > 0)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR,"cs47l24_dsp_scc_reset: dropped %lu bytes out of PCM ring buffer\n", g_scc_context.pcm_drop_count);
    }
    g_scc_context.pcm_drop_count        = 0;
    g_scc_context.pcm_bytecount         = 0;
    g_scc_context.pcm_hotword_offset    = 0;
#ifdef CS47L24_ENABLE_DSP_IRQ_TIMING_STATS
    wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR,"cs47l24_dsp_scc_reset: irq processing times  min=%lu,max=%lu\n",
                  g_scc_context.process_irq_min, g_scc_context.process_irq_max);
    wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR,"cs47l24_dsp_scc_reset: irq trigger intervals min=%lu,max=%lu\n",
                  g_scc_context.irq_interval_min, g_scc_context.irq_interval_max);
    g_scc_context.process_irq_min       = (wiced_time_t)-1;
    g_scc_context.process_irq_max       = (wiced_time_t)0;
    g_scc_context.irq_interval_min      = (wiced_time_t)-1;
    g_scc_context.irq_interval_max      = (wiced_time_t)0;
    g_scc_context.irq_timestamp         = (wiced_time_t)0;
#endif /* CS47L24_ENABLE_DSP_IRQ_TIMING_STATS */

    /*
     * Clear PCM and offset events
     */
    wiced_rtos_wait_for_event_flags(&g_scc_context.pcm_events, DSP_HOST_NEW_DATA_FLAG, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
    wiced_rtos_wait_for_event_flags(&g_scc_context.offset_events, DSP_HOST_NEW_OFFSET_FLAG, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);

    /*
     * Unmask interrupt
     */
    CS47L24_VERIFY_GOTO( cs47l24_dsp_scc_interrupt_unmask(cs47l24), result, _exit );

    /*
     * Re-enable core
     */
    CS47L24_VERIFY_GOTO( cs47l24_upd_bits(cs47l24, g_scc_context.dsp_core->dsp_control1, CS47L24_DSP_CORE_ENA_MASK, CS47L24_DSP_CORE_ENA), result, _exit);

    /*
     * Go through post-setup again
     */
    result = cs47l24_dsp_scc_setup_post(cs47l24);

 _exit:
    /*
     * We're done with DSP reset
     */
    g_scc_context.dsp_reset_pending = WICED_FALSE;

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_DRIVER, WICED_LOG_ERR,"cs47l24_dsp_scc_reset: failed with %d\n", result);
    }
    return result;
}


wiced_result_t cs47l24_dsp_scc_get_offset( cs47l24_device_cmn_data_t* cs47l24, wiced_audio_data_offset_t* offset )
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t       events;

    UNUSED_PARAMETER(events);

    result = wiced_rtos_wait_for_event_flags(&g_scc_context.offset_events, DSP_HOST_NEW_OFFSET_FLAG, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, offset->timeout_msec);

    if (g_scc_context.pcm_hotword_offset > 0)
    {
        offset->start = DSP_PCM_HOTWORD_START_IN_SAMPLES;
        offset->end   = (g_scc_context.pcm_hotword_offset / sizeof(Sample)) - DSP_PCM_HOTWORD_END_OFFSET_ADJ;
    }
    else
    {
        offset->start = 0;
        offset->end = 0;
    }

    return result;
}


wiced_result_t cs47l24_dsp_scc_get_audio( cs47l24_device_cmn_data_t* cs47l24, wiced_audio_data_buffer_t* buffer )
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t       events;

    UNUSED_PARAMETER(events);

    if ((buffer->buffer != NULL) && (buffer->buffer_size > 0))
    {
        result = wiced_rtos_wait_for_event_flags(&g_scc_context.pcm_events, DSP_HOST_NEW_DATA_FLAG, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, buffer->timeout_msec);
        ring_buffer_read(&g_scc_context.pcm_ring, &buffer->buffer[buffer->data_offset], buffer->buffer_size - buffer->data_offset, &buffer->data_length);
    }
    else
    {
        /*
         * Assuming here that we're all done until the next hotword trigger
         */
        result = cs47l24_dsp_scc_reset(cs47l24);
    }

    return result;
}


wiced_result_t cs47l24_dsp_scc_set_streaming_mode( cs47l24_device_cmn_data_t* cs47l24, wiced_audio_wupd_streaming_mode_t* streaming_mode )
{
    wiced_result_t result = WICED_SUCCESS;

    memcpy(&g_scc_context.streaming_mode, streaming_mode, sizeof(g_scc_context.streaming_mode));
    result = cs47l24_dsp_scc_reset(cs47l24);

    return result;
}


wiced_result_t cs47l24_dsp_scc_get_streaming_mode( cs47l24_device_cmn_data_t* cs47l24, wiced_audio_wupd_streaming_mode_t* streaming_mode )
{
    wiced_result_t result = WICED_SUCCESS;

    memcpy(streaming_mode, &g_scc_context.streaming_mode, sizeof(g_scc_context.streaming_mode));

    return result;
}
