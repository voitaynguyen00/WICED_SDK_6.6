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

/** @file
 *
 * SD card filesystem application
 *
 * This application snippet demonstrates filesystem API usage and AES XTS encryption/decryption APIs usage.
 *
 * By default AES_XTS_ENABLED flag is disabled and application functions as below:
 * - Manually create a src folder in SD card and create any txt file and write content to it.
 * - Plug SD card to WICED device (ex. CYW954907AEVAL1F) and download the app.
 * - Once downloaded, application creates tgt folder and copies file from src folder to tgt folder and list
 *   all the files in folder.
 *
 * If AES_XTS_ENABLED flag is enabled then application functions as below:
 * - Plug SD card to WICED device (ex. CYW954907AEVAL1F) and download the app with AES_XTS_ENBALED flag.
 * - Once downloaded, application creates tgt folder and file name test.txt in it. It loads Keys, IV from
 *   aes_xts_config.h file and does AES XTS encryption on plaintext data given in same file. It writes
 *   encrypted data to test.txt file and compare encrypted data with aes_test_xts_ct32 given in aes_xts_config.h
 *   file. Similarly it reads encrypted data from SD card, does AES XTS decryption and compares the plaintext with
 *   aes_test_xts_pt32 given in aes_xts_config.h file. Also ITERATION flag can be modified in aes_xts_config.h file
 *   to run it for multiple iteration.
 */

#include "wiced.h"
#include "wiced_filesystem.h"
#include "sdiovar.h"

#include "aes_xts_config.h"
#include "wiced_osl.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define PATH_BUFFER_SIZE        FX_MAXIMUM_PATH
#define DIRECTORY_SEPARATOR_STR "/"
#define ROOT_DIRECTORY          "/"
#define DIRECTORY_SEPARATOR_CHR '/'
/* reduce FILE_BUFFER_SIZE or increse APPLICATION_STACK_SIZE in mk file,
   if you meet stack overflow */
#define FILE_BUFFER_SIZE        (65536)
#define SRC_DIR                 "/src"
#define TGT_DIR                 "/tgt"
#define PROGRESS_UNIT           (50)
#define LOCAL_BUFFER_SIZE       (64*1024)

/* Enable AES XTS encryption-decryption test */
//#define AES_XTS_ENABLED
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

#ifndef AES_XTS_ENABLED
static int list_files( wiced_filesystem_t* fs_handle, const char* dir_name );
static int mirror_files( wiced_filesystem_t* fs_handle, const char* src_dir_name, const char* tgt_dir_name );
static int copy_file( wiced_filesystem_t* fs_handle, const char* src_path, const char* tgt_path );
#endif

static int delete_files( wiced_filesystem_t* fs_handle, const char* dir_name );
/******************************************************
 *               Variable Definitions
 ******************************************************/
extern uint sd_msglevel;
extern wiced_block_device_t block_device_sdmmc;

/******************************************************
 *               Function Definitions

 ******************************************************/
#ifdef AES_XTS_ENABLED
static void dump_data(unsigned char* buf, unsigned long data_length)
{

    unsigned char* ptr = NULL;
    int i = 1;

    for (ptr = buf; ptr < (buf + data_length); ptr++, i++)
    {
        printf ("0x%02x,", *ptr);
        if( i % 4 == 0)
        {
           printf (" ");
        }

        if( i % 16 == 0 )
        {
        printf ("\n");
        }
    }
}

static int readfrom_decrypt_file( mbedtls_aes_xts_context* ctx_xts, wiced_filesystem_t* fs_handle, const char* tgt_path, unsigned char* plain_text, int* plain_text_length )
{
    wiced_file_t f_tgt;
    int          result = 0;
    uint64_t     read_count = 0;
    char*        local_buffer = NULL;

    if ( WICED_SUCCESS != wiced_filesystem_file_open( fs_handle, &f_tgt, tgt_path, WICED_FILESYSTEM_OPEN_FOR_READ ))
    {
        printf( "Error opening target file %s\n", tgt_path );
        return -1;
    }

    result = mbedtls_aes_xts_setkey_dec( ctx_xts, aes_test_xts_key, ((sizeof(aes_test_xts_key)) * 8));
    if( result != 0)
    {
        printf ("failed to set AES XTS decryption key \n");
        goto cleanup;
    }

    /* Read data from source file */
    local_buffer = (char*) osl_malloc_align( (uint)LOCAL_BUFFER_SIZE, (uint)PLATFORM_L1_CACHE_SHIFT );
    if(local_buffer == NULL)
    {
        printf( "Failed allocate temporary buffer\n" );
        result = -1;
        goto cleanup;
    }

    do
    {
        /* Read the file in blocks.  */
        result = wiced_filesystem_file_read( &f_tgt, local_buffer, LOCAL_BUFFER_SIZE, &read_count );

    } while ( ( result == WICED_SUCCESS ) && ( read_count == LOCAL_BUFFER_SIZE ) ); /* Check if end of file.  */

    printf ("Read [%d] bytes from SD card \n", (int) read_count);

    if( read_count > 0 )
    {
        /* AES XTS encryption */
        result = mbedtls_aes_crypt_xts( ctx_xts, MBEDTLS_AES_DECRYPT, read_count, aes_test_xts_data_unit, (unsigned char*) local_buffer, plain_text );
        if( result != 0 )
        {
            printf ("AES XTS decryption failed \n");
            goto cleanup;
        }

        printf ("AES XTS decryption successful \n");
    }

    *plain_text_length = read_count;

cleanup:
wiced_filesystem_file_close(&f_tgt);

if(local_buffer != NULL)
{
    free(local_buffer);
}

return result;

}

static int encrypt_writeto_file( mbedtls_aes_xts_context* ctx, wiced_filesystem_t* fs_handle, const char* tgt_path, unsigned char* cipher_text, int* cipher_length )
{
    wiced_file_t f_tgt;
    int          result = 0;
    uint64_t     write_count = 1;
    char*        local_buffer = NULL;
    int          length;
    int          plain_txt_length=0;

    plain_txt_length = sizeof(aes_test_xts_pt32);
    unsigned char plain_text[plain_txt_length];

    if ( WICED_SUCCESS != wiced_filesystem_file_open( fs_handle, &f_tgt, tgt_path, WICED_FILESYSTEM_OPEN_ZERO_LENGTH ))
    {
        printf( "Error opening target file %s\n", tgt_path );
        return -1;
    }

    result = mbedtls_aes_xts_setkey_enc( ctx, aes_test_xts_key, ((sizeof(aes_test_xts_key)) * 8));
    if( result != 0)
    {
        printf ("Failed to set AES XTS encryption key \n");
        goto cleanup;
    }

    memcpy( plain_text, aes_test_xts_pt32, sizeof(aes_test_xts_pt32));
    length = sizeof(aes_test_xts_pt32);

    /* AES XTS encryption */
    result = mbedtls_aes_crypt_xts( ctx, MBEDTLS_AES_ENCRYPT, length, aes_test_xts_data_unit, plain_text, cipher_text );
    if( result != 0 )
    {
         printf ("AES XTS encryption failed \n");
         goto cleanup;
    }

    local_buffer = (char*) osl_malloc_align( (uint)length, (uint)PLATFORM_L1_CACHE_SHIFT );
    if ( local_buffer == NULL )
    {
        printf( "Failed allocate temporary buffer\n" );
        result = -1;
        goto cleanup;
    }

    memcpy(local_buffer, cipher_text, length);

    if( wiced_filesystem_file_write( &f_tgt, (const char *) local_buffer, length, &write_count ) != WICED_SUCCESS )
    {
        printf("Failed to write encrypted data to file \n");
        result = -1;
        goto cleanup;
    }

    *cipher_length = write_count;

    printf ("Wrote [%d] bytes to SD card successfully \n", (int) write_count);

cleanup:
wiced_filesystem_file_close( &f_tgt );

if(local_buffer != NULL)
{
    free(local_buffer);
}

return result;

}

int aes_xts_encryption_decryption_test( wiced_filesystem_t* fs_handle )
{
    char tgt_pathbuf[PATH_BUFFER_SIZE];
    int i = 0, result = 0;
    int cipher_text_size = 0;
    int plain_text_size  = 0;

    int actual_cipher_txt_size = sizeof(aes_test_xts_ct32);
    int actual_plain_txt_size  = sizeof(aes_test_xts_pt32);

    unsigned char cipher_text[actual_cipher_txt_size];
    unsigned char plain_text [actual_plain_txt_size];

    /* Initialize AES XTS */
    mbedtls_aes_xts_context ctx;

    mbedtls_aes_xts_init( &ctx );

    sprintf( tgt_pathbuf, "%s%s%s", TGT_DIR, DIRECTORY_SEPARATOR_STR, "test.txt" );
    for (i = 0; i < ITERATION; i++ )
    {
        printf("\r\nAES XTS Encryption-Decryption test started for iteration [%d] \n", (i+1));

        result = encrypt_writeto_file( &ctx, fs_handle, tgt_pathbuf, cipher_text, &cipher_text_size);
        if(result != 0)
        {
            printf ("AES Encryption or writing to file is failing \n");
            goto cleanup;
        }

        printf("================================================================ \n");
        printf("After Encryption data written to file : [%d] \n", cipher_text_size);
        dump_data(cipher_text, cipher_text_size);
        printf("================================================================ \n");

        if( memcmp(cipher_text, aes_test_xts_ct32, actual_cipher_txt_size) != 0)
        {
            printf("Data after encryption not mactching with given cipher text \n");
            result = -1;
            goto cleanup;
        }

        printf("AES Encryption passed for [%d] \r\n", (i+1));

        readfrom_decrypt_file( &ctx, fs_handle, tgt_pathbuf, plain_text, &plain_text_size );
        if(result != 0)
        {
            printf ("AES decryption or writing to file is failing \n");
            goto cleanup;
        }

        printf("================================================================ \n");
        printf("Read from SD card, After decryption data length : [%d] \n", plain_text_size);
        dump_data(plain_text, plain_text_size);
        printf("================================================================ \n");

        if( memcmp(plain_text, aes_test_xts_pt32, actual_plain_txt_size) != 0)
        {
            printf("Data after decryption not matching with given plain text \n");
            result = -1;
            goto cleanup;
        }

        printf("AES Decryption passed for [%d] \r\n", (i+1));

        printf("AES XTS Encryption-Decryption test completed for iteration [%d] \n", (i+1));
    }

cleanup:
mbedtls_aes_xts_free(&ctx);

return result;

}
#endif

void application_start( )
{
    wiced_filesystem_t fs_handle;
    wiced_result_t result;
    wiced_dir_t dir;

    sd_msglevel = SDH_ERROR_VAL;

    printf( "Testing SD/MMC fs\n\n");

    if ( !PLATFORM_FEATURE_ENAB(SDIO) )
    {
        printf( "\nSD card NOT supported\n" );
        return;
    }

    result = wiced_filesystem_init();
    if ( result != WICED_SUCCESS  )
    {
        printf( "Error initing filesystems\n");
        return;
    }

    /* TEST FileX */
    printf( "\n\nTesting SD/MMC on FileX\n\n");

    result = wiced_filesystem_mount( &block_device_sdmmc, WICED_FILESYSTEM_HANDLE_FILEX, &fs_handle, "TestWicedFS" );

    if ( result != WICED_SUCCESS )
    {
        printf( "Error mounting filesystem\n" );
    }

#ifndef AES_XTS_ENABLED
    if ( WICED_SUCCESS != wiced_filesystem_dir_open( &fs_handle, &dir, SRC_DIR ) )
    {
        wiced_filesystem_dir_close( &dir );
        printf( "source dir %s is not existing.\n", SRC_DIR );
        goto finish;
    }
#endif

    if ( WICED_SUCCESS == wiced_filesystem_dir_open( &fs_handle, &dir, TGT_DIR ) )
    {
        printf( "target dir %s is already existing.\n", TGT_DIR );
        wiced_filesystem_dir_close( &dir );

        printf( "deleting %s first...\n", TGT_DIR );

        if ( 0 != delete_files( &fs_handle, TGT_DIR ) )
        {
            printf( "fail to deleting %s.\n", TGT_DIR );
            goto finish;
        }
    }

    printf( "creating target directory %s.\n", TGT_DIR );

    if ( WICED_SUCCESS != wiced_filesystem_dir_create( &fs_handle, TGT_DIR ) )
    {
        printf( "Error creating target directory %s\n", TGT_DIR );
        goto finish;
    }

#ifdef AES_XTS_ENABLED
    int ret = 0;
    ret = aes_xts_encryption_decryption_test( &fs_handle );
    if( ret != 0 )
    {
        printf("AES XTS encryption-decryption test failed \n");
        wiced_filesystem_unmount( &fs_handle );
        return;
    }
#else

    if ( 0 != mirror_files( &fs_handle, SRC_DIR, TGT_DIR ) )
    {
        printf( "fail to mirroring.\n" );
        goto finish;
    }

    list_files( &fs_handle, ROOT_DIRECTORY );

#endif

finish:
    /* Clean up */
    if ( WICED_SUCCESS != wiced_filesystem_unmount( &fs_handle ) )
    {
        printf( "Error unmounting filesystem\n" );
        return;
    }

    printf( "\nSD Card driver test done successfully.\n\n" );

}

#ifndef AES_XTS_ENABLED
static int list_files( wiced_filesystem_t* fs_handle, const char* dir_name )
{
    wiced_dir_t   dir;
    int len;
    char namebuf[PATH_BUFFER_SIZE];
    char pathbuf[PATH_BUFFER_SIZE];
    wiced_dir_entry_type_t    entry_type;
    wiced_dir_entry_details_t details;

    printf( "list_files: %s\n", dir_name );

    strcpy(namebuf, dir_name);
    len = strlen(namebuf);
    if ( len > 1 && namebuf[ len - 1 ] == DIRECTORY_SEPARATOR_CHR )
    {
        namebuf[ len - 1 ] = '\0';
    }

    /* Open the Wiced directory */
    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &dir, namebuf ) )
    {
        printf( "%d: Error opening dir %s\n", __LINE__, namebuf );
        return -1;
    }

    while ( 1 != wiced_filesystem_dir_end_reached( &dir ) )
    {
        if ( WICED_SUCCESS == wiced_filesystem_dir_read( &dir, namebuf, sizeof(namebuf), &entry_type, &details ) )
        {
            // printf( "%d: %s%s\n", entry_type, dir_name, namebuf );
            printf( "%s%s\n", dir_name, namebuf );
            if ( entry_type == WICED_FILESYSTEM_DIR && 0 != strcmp( ".", namebuf ) && 0 != strcmp( "..", namebuf ) )
            {
                sprintf( pathbuf, "%s%s%s", dir_name, namebuf, DIRECTORY_SEPARATOR_STR );
                list_files( fs_handle, pathbuf );
            }
        }
        else
        {
            break;
        }
    }

    wiced_filesystem_dir_close( &dir );

    return 0;
}

static int mirror_files( wiced_filesystem_t* fs_handle, const char* src_dir_name, const char* tgt_dir_name )
{
    wiced_dir_t   src_dir, tgt_dir;
    char namebuf[PATH_BUFFER_SIZE];
    char src_pathbuf[PATH_BUFFER_SIZE];
    char tgt_pathbuf[PATH_BUFFER_SIZE];
    wiced_dir_entry_type_t    entry_type;
    wiced_dir_entry_details_t details;

    printf( "mirror_files: %s to %s\n", src_dir_name, tgt_dir_name );

    /* Open the Wiced directory */
    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &src_dir, src_dir_name ) )
    {
        printf( "%d: Error opening dir %s\n", __LINE__, src_dir_name );
        return -1;
    }

    printf( "mirror_files: source dir %s opened\n", src_dir_name );

    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &tgt_dir, tgt_dir_name ) )
    {
        if ( WICED_SUCCESS != wiced_filesystem_dir_create( fs_handle, tgt_dir_name ) )
        {
            wiced_filesystem_dir_close( &src_dir );
            printf( "Error creating dir %s\n", tgt_dir_name );
            return -1;
        }
    }

    printf( "mirror_files: target dir %s opened\n", tgt_dir_name );

    while ( 1 != wiced_filesystem_dir_end_reached( &src_dir ) )
    {
        if ( WICED_SUCCESS == wiced_filesystem_dir_read( &src_dir, namebuf, sizeof(namebuf), &entry_type, &details ) )
        {
            if ( entry_type == WICED_FILESYSTEM_DIR )
            {
                if ( 0 == strcmp( ".", namebuf ) || 0 == strcmp( "..", namebuf ) )
                {
                    continue;
                }
                sprintf( src_pathbuf, "%s%s%s", src_dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                sprintf( tgt_pathbuf, "%s%s%s", tgt_dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                mirror_files( fs_handle, src_pathbuf, tgt_pathbuf );
            }
            else if ( entry_type == WICED_FILESYSTEM_FILE )
            {
                sprintf( src_pathbuf, "%s%s%s", src_dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                sprintf( tgt_pathbuf, "%s%s%s", tgt_dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                copy_file( fs_handle, src_pathbuf, tgt_pathbuf );

            }
            else if ( entry_type == WICED_FILESYSTEM_LINK )
            {
                printf( "mirroring file type 'WICED_FILESYSTEM_LINK' not supported.\n");
            }
            else
            {
                printf( "fail to mirroring unknown file type.\n");
            }
        }
        else
        {
            break;
        }
    }

    wiced_filesystem_dir_close( &src_dir );
    wiced_filesystem_dir_close( &tgt_dir );

    return 0;
}
#endif

static int delete_files( wiced_filesystem_t* fs_handle, const char* dir_name )
{
    wiced_dir_t   dir;
    char namebuf[PATH_BUFFER_SIZE];
    char pathbuf[PATH_BUFFER_SIZE];
    wiced_dir_entry_type_t    entry_type;
    wiced_dir_entry_details_t details;

    printf( "delete_files: %s\n", dir_name );

    /* Open the Wiced directory */
    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &dir, dir_name ) )
    {
        printf( "%d: Error opening dir %s\n", __LINE__, dir_name );
        return -1;
    }

    while ( 1 != wiced_filesystem_dir_end_reached( &dir ) )
    {
        if ( WICED_SUCCESS == wiced_filesystem_dir_read( &dir, namebuf, sizeof(namebuf), &entry_type, &details ) )
        {
            if ( entry_type == WICED_FILESYSTEM_DIR )
            {
                if ( 0 == strcmp( ".", namebuf ) || 0 == strcmp( "..", namebuf ) )
                {
                    continue;
                }
                sprintf( pathbuf, "%s%s%s", dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                if ( 0 == delete_files( fs_handle, pathbuf ) )
                {
                    printf( "deleting %s...\n", pathbuf );
                    if ( WICED_SUCCESS != wiced_filesystem_dir_delete( fs_handle, pathbuf ) )
                    {
                        printf( "Error deleting %s\n", pathbuf );
                        wiced_filesystem_dir_close( &dir );
                        return -1;
                    }
                }
            }
            else
            {
                sprintf( pathbuf, "%s%s%s", dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                printf( "deleting %s...\n", pathbuf );
                if ( WICED_SUCCESS != wiced_filesystem_file_delete( fs_handle, pathbuf ) )
                {
                    printf( "Error deleting %s\n", pathbuf );
                    wiced_filesystem_dir_close( &dir );
                    return -1;
                }
            }
        }
        else
        {
            break;
        }
    }

    wiced_filesystem_dir_close( &dir );

    return 0;
}

#ifndef AES_XTS_ENABLED
static int copy_file( wiced_filesystem_t* fs_handle, const char* src_path, const char* tgt_path )
{
    wiced_file_t f_src;
    wiced_file_t f_tgt;
    DEFINE_CACHE_LINE_ALIGNED_ARRAY(char, filebuf, FILE_BUFFER_SIZE);
    int result = 0;
    uint64_t read_count = 1;
    uint64_t write_count = 1;
    int progress = 0;

    printf( "copy_file: %s to %s\n", src_path, tgt_path );

    if ( WICED_SUCCESS != wiced_filesystem_file_open( fs_handle, &f_src, src_path, WICED_FILESYSTEM_OPEN_FOR_READ ))
    {
        printf( "Error opening src file %s\n", src_path );
        result = -1;
        goto finish;
    }

    if ( WICED_SUCCESS != wiced_filesystem_file_open( fs_handle, &f_tgt, tgt_path, WICED_FILESYSTEM_OPEN_ZERO_LENGTH ))
    {
        printf( "Error opening target file %s\n", tgt_path );
        result = -1;
        goto finish;
    }

    while ( read_count && write_count )
    {
        wiced_filesystem_file_read( &f_src, filebuf, ( uint64_t ) FILE_BUFFER_SIZE, &read_count );

        if ( 0 == ( progress % PROGRESS_UNIT ) )
        {
           // printf("R");
        }

        if ( read_count )
        {
            wiced_filesystem_file_write( &f_tgt, (const char *) filebuf, read_count, &write_count );

            if ( 0 == ( progress % PROGRESS_UNIT ) )
            {
                // printf("W");
            }

            if ( read_count != write_count )
            {
                printf( "wiced_filesystem_file_write: try = %lu, write = %lu\n", ( uint32_t ) read_count, ( uint32_t ) write_count );
                result = -1;
                break;
            }
        }
        progress++;
    }

    printf("\n");


finish:
    /*
       Note:
       Because there is bug in original FileX 5.5 exFAT 'fx_media_flush' and 'fx_media_close',
       close f_tgt file which is opened for WRITE mode first before closing f_src.
       fx_media_flush and fx_media_close don't call _fx_directory_exFAT_entry_write for exFAT filesystem,
       which corrupts directory entries.
    */
    wiced_filesystem_file_close( &f_tgt );
    wiced_filesystem_file_close( &f_src );

    return result;
}
#endif
