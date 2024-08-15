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
 * AES128-CBC Encrypt/Decrypt input file
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <aes.h>
#include <sha256.h>
#include <limits.h>
#include <time.h>

#define BCM_KEY_SIZE        (16)
#define AES_BLOCK_SZ        (16)
#define AES_KEY_BITLEN(x)   (x*8)
#define MIN(x,y) ((x) < (y) ? (x) : (y))
/* fail = 0, success = 1 */
#define PRINT_RESULT_AND_INCREMENT_FAILURES(x,y) {do{ if(x == 1){ printf("pass\n"); } else{ printf("fail\n"); y++;} }while(0);}

#define IS_NON_ZERO(x) ((x)?1:0)
#define NUM_BYTES_RAND_MAX ( IS_NON_ZERO( RAND_MAX & 0xff000000 ) + IS_NON_ZERO( RAND_MAX & 0x00ff0000 ) + IS_NON_ZERO( RAND_MAX & 0x0000ff00 ) + IS_NON_ZERO( RAND_MAX & 0x000000ff ) )

int aes_cbc_128_enc( const char *keyfile, const char *infile, const char *outfile, uint32_t blocksize, int sector_num );
int aes_cbc_128_dec( const char *keyfile, const char *infile, const char *outfile, uint32_t blocksize, int sector_num );


int readfile( const char *infile, char *buf, int maxlen )
{
    FILE *infp;
    int filelen = 0;

    infp = fopen( infile, "rb" );
    if ( !infp )
    {
        printf( "* Failed to open input file '%s' - %s\n", infile, strerror( errno ) );
        return -1;
    }

    filelen = fread( buf, 1, maxlen, infp );
    if ( filelen < 0 )
    {
        printf( "* Failed to read data from file '%s'\n", infile );
    }

    fclose( infp );

    return filelen;
}

int generate_random_file( const char *filename, int generated_file_size )
{
    FILE *outfp = NULL;
    uint8_t random_data_buffer[4096];
    uint8_t bytes_in_randmax = NUM_BYTES_RAND_MAX;
    uint16_t written_data_counter = 0, generated_data_counter = 0;

    srand( time(NULL) );

    outfp = fopen( filename, "wb"); /* this will overwrite the previous file */

    if( !outfp )
    {
        printf( "* Failed to generate file '%s' - %s\n", filename, strerror( errno ) );
        return -1;
    }

    written_data_counter = 0;
    while( written_data_counter < generated_file_size )
    {
        /* calculate the number of bytes to generate */
        int bytes_to_generate = ( generated_file_size - written_data_counter <= sizeof(random_data_buffer) ) ? generated_file_size - written_data_counter : sizeof( random_data_buffer );

        /* generate data to fill the buffer, or until the end of file */
        generated_data_counter = 0;
        while( generated_data_counter < bytes_to_generate )
        {
            int rand_i = rand();
            memcpy( &random_data_buffer[generated_data_counter], &rand_i, ( bytes_in_randmax <= sizeof(random_data_buffer) - generated_data_counter) ? bytes_in_randmax : sizeof(random_data_buffer) - generated_data_counter );
            generated_data_counter += bytes_in_randmax;
        }
        /* write the buffer to the file */
        fwrite( random_data_buffer, 1, bytes_to_generate, outfp );
        written_data_counter += bytes_to_generate;
    }
    fclose( outfp );

    return 0;
}

int compare_files( const char *file1, const char *file2 )
{
    /* returns 1 if the files are the same, returns 0 if data mismatch, returns -1 if file open error */
    FILE *fp1 = NULL;
    FILE *fp2 = NULL;
    const uint16_t buffer_size = 2 * 1024;
    uint32_t filesize1, filesize2, read_counter = 0;
    uint8_t readbuf1[buffer_size], readbuf2[buffer_size]; /* use defines */

    fp1 = fopen( file1, "rb" );
    fp2 = fopen( file2, "rb" );

    if( !fp1 )
    {
        printf( "* Failed to open file '%s' - %s\n", file1, strerror( errno ) );
        return -1;
    }
    if( !fp2 )
    {
        printf( "* Failed to open file '%s' - %s\n", file2, strerror( errno ) );
        return -1;
    }

    /* set pointer to EOF to determine file size */
    fseek( fp1, 0L, SEEK_END );
    filesize1 = ftell( fp1 );

    fseek( fp2, 0L, SEEK_END );
    filesize2 = ftell( fp2 );

    /* compare file size */
    if( filesize1 != filesize2 )
    {
        return 0;
    }

    /* set stream pointer to start of file */
    fseek( fp1, 0L, SEEK_SET );
    fseek( fp2, 0L, SEEK_SET );

    /* at this point filesize1 == filesize2 */
    while( read_counter < filesize1 )
    {
        uint16_t bytes_to_read = ( filesize1 - read_counter < sizeof(readbuf1) ? filesize1 - read_counter : sizeof(readbuf1) );
        fread( readbuf1, 1, bytes_to_read, fp1 );
        fread( readbuf2, 1, bytes_to_read, fp2 );
        if( memcmp(readbuf1, readbuf2, bytes_to_read) != 0 )
        {
            /* files are different! */
            fclose( fp1 );
            fclose( fp2 );
            return 0;
        }
        read_counter += bytes_to_read;
    }
    fclose( fp1 );
    fclose( fp2 );
    return 1;
}

int aes_cbc_128_enc( const char *keyfile, const char *infile, const char *outfile, uint32_t blocksize, int sector_num )
{
    uint8_t key[ BCM_KEY_SIZE ];
    uint8_t iv[ BCM_KEY_SIZE * 2];
    aes_context_t aes_ctx;
    uint8_t *filebuffer = NULL;
    uint8_t *encbuffer = NULL;
    uint32_t filesize, padded_filesize;
    int32_t encsize;
    uint32_t keysize;
    FILE *infp = NULL;
    FILE *outfp = NULL;
    int32_t success = 1;
    int32_t remaining;
    char zero_char = '\0';

    keysize = readfile( keyfile, key, sizeof( key ) );

    if ( keysize != BCM_KEY_SIZE )
    {
        printf("Incorrect key size read\n");
        success = 0;
        goto finalize;
    }

    infp = fopen( infile, "rb" );
    if ( !infp )
    {
        printf( "* Failed to open input file '%s' - %s\n", infile, strerror( errno ) );
        success = 0;
        goto finalize;
    }

    fseek( infp, 0L, SEEK_END );
    filesize = ftell( infp );
    fclose( infp );
    padded_filesize = ( ( ( filesize - 1 ) >> 4 ) + 1 ) << 4;

    if ( padded_filesize != filesize )
    {
        uint32_t padding = padded_filesize - filesize;

        printf( "[aes_cbc_128] padding %s [%d -> %d Bytes] to make its size AES_BLOCK_SIZE (16) aligned\n", infile, filesize, padded_filesize );
        infp = fopen( infile, "ab" );
        if ( !infp )
        {
            printf( "* Failed to open input file '%s' - %s\n", infile, strerror( errno ) );
            success = 0;
            goto finalize;
        }

        while ( padding != 0 )
        {
            fwrite( &zero_char, sizeof(char), 1, infp );
            padding-- ;
        }

        fclose( infp );
    }
    infp = fopen( infile, "rb" );
    if ( !infp )
    {
        printf( "* Failed to open input file '%s' - %s\n", infile, strerror( errno ) );
        success = 0;
        goto finalize;
    }

    fseek( infp, 0L, SEEK_SET );

    if ( blocksize == 0 )
        blocksize = padded_filesize;

    filebuffer = malloc( blocksize );
    if ( filebuffer == NULL )
    {
        printf( "* Failed to allocate filebuffer of file '%s'\n", infile );
        success = 0;
        goto finalize;
    }
    encbuffer = malloc( blocksize );
    if ( encbuffer == NULL )
    {
        printf( "* Failed to allocate encrypt buffer of file '%s'\n", infile );
        success = 0;
        goto finalize;
    }

    outfp = fopen( outfile, "wb" );
    remaining = padded_filesize;
    while ( remaining > 0 )
    {

        uint32_t read_size;
        int i;

        memset( filebuffer, 0, blocksize );
        memset( encbuffer, 0, blocksize );
        memset( iv, 0, AES_BLOCK_SZ );
        if ( sector_num > 0 )
        {
            /* iv = SHA2( sector_number XOR key ) */
            for(i = 0; i < AES_BLOCK_SZ; i++)
            {
                iv[i] = ( (unsigned char)key[i] ^ (unsigned char)(sector_num % UCHAR_MAX));
            }
            mbedtls_sha256( iv, AES_BLOCK_SZ, iv, 0);
            sector_num +=1;
        }

        mbedtls_aes_setkey_enc( &aes_ctx, key, AES_KEY_BITLEN(keysize) );
        read_size = MIN(blocksize, remaining);

        if ( fread( filebuffer, 1, read_size, infp ) != read_size )
        {
            printf( "* File '%s' read error\n", infile );
            success = 0;
            goto finalize;
        }

        mbedtls_aes_crypt_cbc( &aes_ctx, MBEDTLS_AES_ENCRYPT, read_size, iv, filebuffer, encbuffer );

        if ( fwrite( encbuffer, 1, read_size, outfp ) != read_size )
        {
            success = 0;
            printf( "* Failed to write output file '%s' - %s\n", outfile, strerror( errno ) );
            goto finalize;
        }

        remaining -= read_size;
    }

    finalize:

    if ( infp )
        fclose( infp );
    if ( outfp )
        fclose( outfp );
    if ( filebuffer )
        free( filebuffer );
    if ( encbuffer )
        free( encbuffer );
    if ( !success )
        remove( outfile );

    return success;
}

int aes_cbc_128_dec( const char *keyfile, const char *infile, const char *outfile, uint32_t blocksize, int sector_num )
{
    uint8_t key[ BCM_KEY_SIZE ];
    uint8_t iv[ BCM_KEY_SIZE * 2];
    aes_context_t aes_ctx;
    uint8_t *filebuffer = NULL;
    uint8_t *decbuffer = NULL;
    uint32_t filesize, padded_filesize;
    int32_t decsize;
    uint32_t keysize;
    FILE *infp = NULL;
    FILE *outfp = NULL;
    int32_t success = 1;
    int32_t remaining;

    keysize = readfile( keyfile, key, sizeof( key ) );

    if ( keysize != BCM_KEY_SIZE )
    {
        printf("Incorrect key size read\n");
        success = 0;
        goto finalize;
    }

    infp = fopen( infile, "rb" );
    if ( !infp )
    {
        printf( "* Failed to open input file '%s' - %s\n", infile, strerror( errno ) );
        success = 0;
        goto finalize;
    }

    fseek( infp, 0L, SEEK_END );
    filesize = ftell( infp );
    padded_filesize = ( ( ( filesize - 1 ) >> 4 ) + 1 ) << 4;
    fseek( infp, 0L, SEEK_SET );

    if ( blocksize == 0 )
        blocksize = padded_filesize;

    remaining = filesize;
    filebuffer = malloc( blocksize );
    if ( filebuffer == NULL )
    {
        printf( "* Failed to allocate filebuffer of file '%s'\n", infile );
        success = 0;
        goto finalize;
    }
    decbuffer = malloc( blocksize );
    if ( decbuffer == NULL )
    {
        printf( "* Failed to allocate decrypt buffer of file '%s'\n", infile );
        success = 0;
        goto finalize;
    }

    outfp = fopen( outfile, "wb" );

    while ( remaining > 0 )
    {
        uint32_t read_size;
        int i;

        read_size = MIN(blocksize, remaining);
        memset( filebuffer, 0, blocksize );
        memset( decbuffer, 0, blocksize );
        memset( iv, 0, AES_BLOCK_SZ );

        if ( sector_num > 0 )
        {
            /* iv = SHA2( sector_number XOR key ) */
            for(i = 0; i < AES_BLOCK_SZ; i++)
            {
                iv[i] = ( (unsigned char)key[i] ^ (unsigned char)(sector_num % UCHAR_MAX));
            }
            mbedtls_sha256( iv, AES_BLOCK_SZ, iv, 0);
            sector_num +=1;
        }

        mbedtls_aes_setkey_dec( &aes_ctx, key, AES_KEY_BITLEN( keysize) );

        if ( ( success = fread( filebuffer, 1, read_size, infp ) ) != read_size )
        {
            printf( "* File '%s' read error\n", infile );
            printf( "expected = %d, got = %d\n", read_size, success );
            success = 0;
            goto finalize;
        }

        mbedtls_aes_crypt_cbc( &aes_ctx, MBEDTLS_AES_DECRYPT, read_size, iv, filebuffer, decbuffer );

        if ( fwrite( decbuffer, 1, read_size, outfp ) != read_size )
        {
            success = 0;
            printf( "* Failed to write output file '%s' - %s\n", outfile, strerror( errno ) );
            goto finalize;
        }

        remaining -= read_size;
    }
    finalize:

    if ( infp )
        fclose( infp );
    if ( outfp )
        fclose( outfp );
    if ( filebuffer )
        free( filebuffer );
    if ( decbuffer )
        free( decbuffer );
    if ( !success )
        remove( outfile );

    return success;
}

int aes_cbc_128_test_iteration( const char *keyfile, const char *infile, const char *encfile, const char *decfile, uint32_t blocksize, int sector_num )
{
    /* Encrypt the file, decrypt the encrypted file, and return the result of the comparison of the input file and the decrypted file */
    aes_cbc_128_enc( keyfile, infile, encfile, blocksize, sector_num );
    aes_cbc_128_dec( keyfile, encfile, decfile, blocksize, sector_num );
    return compare_files( infile, decfile);
}

int aes_cbc_128_test( void )
{
    const char* key_file_name = "aes_test_key.key";
    const char* test_file_name = "aes_test_file.bin";
    const char* enc_test_file_name = "aes_test_file.bin.enc";
    const char* dec_test_file_name = "aes_test_file.bin.dec";
    const int const_file_size = 16 * 1024;

    uint16_t random_file_size; /* limit to 64k-1 */
    uint16_t random_block_size; /* limit to 8k-1 */
    uint8_t random_sector_number; /* limit to 1k-1 */
    uint8_t number_random_test_iteration; /* limit to 100-1 */
    uint8_t random_test_iteration;

    int result = 0;
    uint8_t num_failures = 0;

    srand( time(NULL) );

    number_random_test_iteration = rand( ) % 100; /* limit to 100-1 */
    random_test_iteration = number_random_test_iteration;

    /* Constant file size test cases */
    printf( "Positive test case: file size 16k, block size 0, sector number 0: ");
    generate_random_file( key_file_name, BCM_KEY_SIZE );
    generate_random_file( test_file_name, const_file_size );
    aes_cbc_128_enc( key_file_name, test_file_name, enc_test_file_name, 0, 0 );
    aes_cbc_128_dec( key_file_name, enc_test_file_name, dec_test_file_name, 0, 0 );
    PRINT_RESULT_AND_INCREMENT_FAILURES( compare_files(test_file_name, dec_test_file_name), num_failures );

    printf( "Positive test case: file size 16k, block size 4k, sector number 0: ");
    aes_cbc_128_enc( key_file_name, test_file_name, enc_test_file_name, 4096, 0 );
    aes_cbc_128_dec( key_file_name, enc_test_file_name, dec_test_file_name, 4096, 0 );
    PRINT_RESULT_AND_INCREMENT_FAILURES( compare_files(test_file_name, dec_test_file_name), num_failures );

    printf( "Positive test case: file size 16k, block size 4k, sector number != 0: ");
    aes_cbc_128_enc( key_file_name, test_file_name, enc_test_file_name, 4096, 8 );
    aes_cbc_128_dec( key_file_name, enc_test_file_name, dec_test_file_name, 4096, 8 );
    PRINT_RESULT_AND_INCREMENT_FAILURES( compare_files(test_file_name, dec_test_file_name), num_failures );

    printf( "Negative test case: block size mismatch: ");
    aes_cbc_128_dec( key_file_name, enc_test_file_name, dec_test_file_name, 2048, 8 );
    PRINT_RESULT_AND_INCREMENT_FAILURES( !compare_files(test_file_name, dec_test_file_name), num_failures ); /* if compare_files fails, the test case passes */

    printf( "Negative test case: sector number mismatch: ");
    aes_cbc_128_dec( key_file_name, enc_test_file_name, dec_test_file_name, 4096, 9 );
    PRINT_RESULT_AND_INCREMENT_FAILURES( !compare_files(test_file_name, dec_test_file_name), num_failures ); /* if compare_files fails, the test case passes */

    /* Random test cases */
    printf( "\nRandom test cases: %d iterations\n", number_random_test_iteration );
    while( random_test_iteration )
    {
        random_file_size = rand() % USHRT_MAX; /* limit to 64k-1 */
        random_block_size = rand() % 8192; /* limit to 8k-1 */
        random_block_size = ( ( ( random_block_size - 1 ) >> 4 ) + 1 ) << 4; /* block size must be a multiple of 16 */
        random_sector_number = rand() % 1024; /* limit to 1k-1 */

        printf( "Iteration %d: filesize %d, blocksize %d, sector number %d\n", number_random_test_iteration - random_test_iteration + 1, random_file_size, random_block_size, random_sector_number );
        generate_random_file( key_file_name, BCM_KEY_SIZE );
        generate_random_file( test_file_name, random_file_size );
        PRINT_RESULT_AND_INCREMENT_FAILURES( aes_cbc_128_test_iteration( key_file_name, test_file_name, enc_test_file_name, dec_test_file_name, random_block_size, random_sector_number ), num_failures );
        random_test_iteration--;
    }

    printf( "Finished %d random test cases with %d failures (%.1f%%)\n", number_random_test_iteration, num_failures, (float)num_failures/(float)number_random_test_iteration );

    remove( key_file_name );
    remove( test_file_name );
    remove( enc_test_file_name );
    remove( dec_test_file_name );

    return num_failures;
}

void usage( void )
{
    printf( "Usage: aes_cbc_128 <mode>\n"
        "where <mode> is one of:\n"
        "  test                                                                   Run the self test\n"
        "  enc <keyfile> <infile> <encryptedfile> <blocksz> <starting sector no>  Encrypt the supplied file\n"
        "  dec <keyfile> <infile> <decryptedfile> <blocksz> <starting sector no>  Decrypt the supplied file\n" );
    exit( 0 );
}

int main( int argc, const char **argv )
{
    int argn;
    const char *mode;
    uint32_t blocksize = 0;
    int sector_num = -1;
    for ( argn = 1; argn < argc; argn++ )
    {
        const char *arg = argv[ argn ];
        if ( arg[ 0 ] != '-' )
            break;

        printf( "* Unknown option '%s'\n", arg );
        usage( );
    }

    if ( argn == argc )
        usage( );

    mode = argv[ argn++ ];

    if ( strcmp( mode, "test" ) == 0 )
    {
        return aes_cbc_128_test( );
    }
    else if ( strcmp( mode, "enc" ) == 0 )
    {
        if ( ( argn + 3 ) == argc )
        {
            blocksize = 0;
        }
        else if ( ( argn + 4 ) == argc )
        {
            blocksize = atoi( argv[ argn + 3 ] );
        }
        else if ( ( argn + 5 ) == argc )
        {
            blocksize  = atoi( argv[ argn + 3 ] );
            sector_num = atoi( argv[ argn + 4 ] );
        }
        else
        {
            printf( "* Incorrect parameters\n" );
            usage( );
        }

        if ( aes_cbc_128_enc( argv[ argn ], argv[ argn + 1 ], argv[ argn + 2 ], blocksize, sector_num ) == 0 )
        {
            return -1;
        }
    }
    else if ( strcmp( mode, "dec" ) == 0 )
    {
        if ( ( argn + 3 ) == argc )
        {
            blocksize = 0;
        }
        else if ( ( argn + 4 ) == argc )
        {
            blocksize = atoi( argv[ argn + 3 ] );
        }
        else if ( ( argn + 5 ) == argc )
        {
            blocksize  = atoi( argv[ argn + 3 ] );
            sector_num = atoi( argv[ argn + 4 ] );
        }
        else
        {
            printf( "* Incorrect parameters\n" );
            usage( );
        }

        if ( aes_cbc_128_dec( argv[ argn ], argv[ argn + 1 ], argv[ argn + 2 ], blocksize, sector_num) == 0 )
        {
            return -1;
        }
    }
    else
    {
        printf( "* Unknown mode '%s'\n", mode );
        usage( );
    }

    return 0;
}
