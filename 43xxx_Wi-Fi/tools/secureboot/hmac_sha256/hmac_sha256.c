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
 * HMAC SHA256 Sign/Verify input file
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <sha256.h>
#include <limits.h>
#include <time.h>

#define SHA256_DIGEST_LENGTH    (32)
#define BCM_KEY_SIZE            (32)
/* fail = 0, success = 1 */
#define PRINT_RESULT_AND_INCREMENT_FAILURES(x,y) {do{ if(x == 1){ printf("pass\n"); } else{ printf("fail\n"); y++;} }while(0);}

#define IS_NON_ZERO(x) ((x)?1:0)
#define NUM_BYTES_RAND_MAX ( IS_NON_ZERO( RAND_MAX & 0xff000000 ) + IS_NON_ZERO( RAND_MAX & 0x00ff0000 ) + IS_NON_ZERO( RAND_MAX & 0x0000ff00 ) + IS_NON_ZERO( RAND_MAX & 0x000000ff ) )

int readfile( const char *infile, char *buf, int maxlen )
{
    FILE *infp;
    int filelen = 0;

    infp = fopen( infile, "rb" );
    if ( infp == NULL )
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

int hmacsign( const char *keyfile, const char *infile, const char *outfile, unsigned int blocksize, unsigned int no_padding, int signature_only )
{
    unsigned char key[ BCM_KEY_SIZE ] =
    { '\0' };
    unsigned char hash[ SHA256_DIGEST_LENGTH ];
    unsigned char *filebuffer = NULL;
    unsigned int filesize, padded_filesize;
    unsigned int keysize;
    unsigned int digest_len;
    FILE *infp = NULL;
    FILE *outfp = NULL;
    int remaining;
    unsigned int padding;
    int success = 1;
    char zero_char = '0';

    keysize = readfile( keyfile, key, sizeof( key ) );

    if ( keysize > BCM_KEY_SIZE )
    {
        success = 0;
        printf( "keysize = %d > BCM_KEY_SIZE = %d Exit\n", keysize, BCM_KEY_SIZE );
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
    fseek( infp, 0L, SEEK_SET );
    fclose( infp );

    /* If file has to be signed block by block, make sure that
     * the size of file is aligned to blocksize by padding it
     */
    if ( blocksize != 0 )
    {
        blocksize = blocksize - SHA256_DIGEST_LENGTH;
        if ( ( no_padding == 1 ) || ( ( filesize % blocksize ) == 0 ) )
            padding = 0;
        else
            padding = blocksize - ( filesize % blocksize );

    }
    else
    {
        if ( signature_only )
            padded_filesize = filesize;
        else
            padded_filesize = ( ( ( filesize - 1 ) >> 4 ) + 1 ) << 4;

        padding = padded_filesize - filesize;
    }

    if ( padding != 0 )
    {
        printf( "[sha256_hmac] : Padding file %s [%d -> %d bytes] \n", infile, filesize, filesize + padding );

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

    fseek( infp, 0L, SEEK_END );
    padded_filesize = ftell( infp );
    fseek( infp, 0L, SEEK_SET );

    remaining = padded_filesize;
    if ( blocksize == 0 )
        blocksize = padded_filesize;

    filebuffer = malloc( blocksize );
    if ( filebuffer == NULL )
    {
        printf( "* Failed to allocation filebuffer of file '%s'\n", infile );
        success = 0;
        goto finalize;
    }
    memset( filebuffer, 0, blocksize );
    outfp = fopen( outfile, "wb" );

    if ( !infp )
    {
        printf( "* Failed to open output file '%s' - %s\n", outfile, strerror( errno ) );
        fclose( infp );
        success = 0;
        goto finalize;
    }

    while ( remaining >= blocksize )
    {

        if ( fread( filebuffer, 1, blocksize, infp ) != blocksize )
        {
            printf( "* File '%s' read error\n", infile );
            success = 0;
            goto finalize;
        }

        sha2_hmac( key, keysize, filebuffer, blocksize, hash, 0 );
        if ( !signature_only && (blocksize > filesize) && no_padding )
        {
            fwrite( filebuffer, 1, filesize, outfp );
        }
        else if ( !signature_only )
        {
            fwrite( filebuffer, 1, blocksize, outfp );
        }

        fwrite( hash, 1, SHA256_DIGEST_LENGTH, outfp );

        remaining -= blocksize;
    }

    if( remaining )
    {
        printf( "The filesize is not a multiple of (blocksize - SHA256_DIGEST_LENGTH) and the file was not padded\nFile is truncated or empty\n");
    }

    finalize:

    if ( infp != NULL )
        fclose( infp );
    if ( outfp != NULL )
        fclose( outfp );
    if ( filebuffer != NULL )
        free( filebuffer );
    if ( !success )
        remove( outfile );
    return success;
}

int hmacverify( const char *keyfile, const char *infile, const char *signaturefile, unsigned int blocksize, const char *outfile )
{
    unsigned char key[ BCM_KEY_SIZE ] =
    { '\0' };
    unsigned char hash[ SHA256_DIGEST_LENGTH ];
    unsigned char signature[ SHA256_DIGEST_LENGTH ];
    unsigned char *filebuffer = NULL;
    unsigned int filesize;
    unsigned int keysize;
    unsigned int digest_len;
    FILE *infp = NULL;
    FILE *outfp = NULL;
    int success = 1;
    unsigned int remaining;
    int block_num = 0;

    keysize = readfile( keyfile, key, sizeof( key ) );

    if ( keysize > BCM_KEY_SIZE )
    {
        printf( "keysize > %d, Exit \n", BCM_KEY_SIZE );
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

    if ( outfile )
    {
        outfp = fopen( outfile, "wb" );
        if( !outfp )
        {
            printf( "* Failed to open output file '%s' - %s\n", outfile, strerror( errno ) );
            success = 0;
            goto finalize;
        }
    }

    fseek( infp, 0L, SEEK_END );
    filesize = ftell( infp );
    fseek( infp, 0L, SEEK_SET );

    if ( blocksize == 0 )
        blocksize = filesize;

    if ( filesize > 0 )
    {
        filebuffer = malloc( blocksize );
        if ( filebuffer == NULL )
        {
            printf( "* Failed to allocation filebuffer of file '%s'\n", infile );
            success = 0;
            goto finalize;
        }
    }
    else
    {
        printf( "* File '%s' is not valid\n", infile );
        success = 0;
        goto finalize;
    }

    remaining = filesize;

    while ( remaining != 0 )
    {
        if ( fread( filebuffer, 1, blocksize, infp ) != blocksize )
        {
            printf( "* File '%s' read error\n", infile );
            success = 0;
            goto finalize;
        }

        if ( signaturefile )
        {
            if ( readfile( signaturefile, signature, SHA256_DIGEST_LENGTH ) != SHA256_DIGEST_LENGTH )
            {
                printf( "* Failed to read signature file\n" );
                success = 0;
                goto finalize;
            }
        }
        else
        {
            memcpy( signature, filebuffer + blocksize - SHA256_DIGEST_LENGTH, SHA256_DIGEST_LENGTH );
        }

        sha2_hmac( key, keysize, filebuffer, blocksize - SHA256_DIGEST_LENGTH, hash, 0 );

        if ( memcmp( hash, signature, SHA256_DIGEST_LENGTH ) == 0 )
        {
            if( outfp )
            {
                fwrite( filebuffer, sizeof(char), blocksize - SHA256_DIGEST_LENGTH, outfp );
            }
        }
        else
        {
            printf( "* Block %d - Hash doesn't match\n", block_num );
            success = 0;
        }

        remaining -= blocksize;
        block_num++;
    }

    finalize:
    if ( infp )
        fclose( infp );
    if( outfp )
        fclose ( outfp );
    if( !success )
        remove( outfile );
    if ( filebuffer )
        free( filebuffer );

    return success;
}

int hmac_test_iteration( const char *keyfile, const char *infile, const char *signedfile, const char *reconstructed_file, uint32_t blocksize, int pad )
{
    /* Sign the file, verify the signed file, reconstruct the file without signature, and return the result of the comparison of the input file and the decrypted file */
    int result;

    FILE *fp1 = NULL;
    int filesize;

    /* Get the input file size */
    fp1 = fopen( infile, "rb" );
    if( !fp1 )
    {
        printf( "* Failed to open file '%s' - %s\n", infile, strerror( errno ) );
        return -1;
    }
    fseek( fp1, 0L, SEEK_END );
    filesize = ftell( fp1 );
    fclose( fp1 );

    hmacsign( keyfile, infile, signedfile, blocksize, pad, 0 ); /* sign */
    hmacverify( keyfile, signedfile, NULL, blocksize, reconstructed_file ); /* vrfy */
    result = compare_files( infile, reconstructed_file );
    remove( reconstructed_file );

    /* If the filesize is not a multiple of (blocksize - SHA256_DIGEST_LENGTH) and there was no padding, the file compare should fail */
    if( (filesize % (blocksize - SHA256_DIGEST_LENGTH)) && pad )
    {
        if( result == -1 )
        {
            return 1;
        }
        result = !result;
    }

    return result;
}

int hmactest( void )
{
    const char* key_file_name = "hmac_test_key.key";
    const char* test_file_name = "hmac_test_file.bin";
    const char* signed_test_file_name = "hmac_test_file.bin.signed";
    const char* reconstructed_test_file_name = "hmac_test_file.bin.signed.reconst";
    const char* signature_file_name = "hmac_signature.sig";

    const uint8_t no_blocks = 0;
    const uint16_t even_block_size = 4096;
    const uint16_t odd_block_size = 4095;
    const uint8_t no_padding = 1;
    const uint8_t padding = 0;
    const uint16_t const_file_size = 4 * even_block_size - (SHA256_DIGEST_LENGTH * 4);

    uint16_t random_file_size; /* limit to 64k-1 */
    uint16_t random_block_size; /* limit to 8k-1 */
    uint8_t number_random_test_iteration; /* limit to 100-1 */
    uint8_t random_test_iteration;
    uint8_t random_padding;

    int result = 0;
    uint8_t num_failures = 0;

    srand( time(NULL) );

    number_random_test_iteration = rand( ) % 100; /* limit to 100-1 */
    random_test_iteration = number_random_test_iteration;

    /* Constant file size test cases */
    printf( "Positive test case: file size 16k, block size 0, no padding:\n");
    generate_random_file( key_file_name, BCM_KEY_SIZE );
    generate_random_file( test_file_name, const_file_size );
    hmacsign( key_file_name, test_file_name, signed_test_file_name, no_blocks, no_padding, 0 ); /* sign */
    hmacsign( key_file_name, test_file_name, signature_file_name, no_blocks, no_padding, 1 ); /* sig */
    hmacverify( key_file_name, signed_test_file_name, NULL, no_blocks, reconstructed_test_file_name ); /* vrfy */
    hmacverify( key_file_name, signed_test_file_name, signature_file_name, no_blocks, NULL ); /* vsig */
    PRINT_RESULT_AND_INCREMENT_FAILURES( compare_files(test_file_name, reconstructed_test_file_name), num_failures );

    printf( "Positive test case: file size 16k, block size 4096 (even), no padding:\n");
    hmacsign( key_file_name, test_file_name, signed_test_file_name, even_block_size, no_padding, 0 ); /* sign */
    hmacverify( key_file_name, signed_test_file_name, NULL, even_block_size, reconstructed_test_file_name ); /* vrfy */
    PRINT_RESULT_AND_INCREMENT_FAILURES( compare_files(test_file_name, reconstructed_test_file_name), num_failures );

    printf( "Negative test case: file size 16k, block size 4095 (odd), no padding:\n");
    hmacsign( key_file_name, test_file_name, signed_test_file_name, odd_block_size, no_padding, 0 ); /* sign */
    hmacverify( key_file_name, signed_test_file_name, NULL, odd_block_size, reconstructed_test_file_name ); /* vrfy */
    PRINT_RESULT_AND_INCREMENT_FAILURES( !compare_files(test_file_name, reconstructed_test_file_name), num_failures ); /* if compare_files fails, the test case passes */

    printf( "Negative test case: file size 16k, block size 4095/4096 (mismatch), no padding:\n");
    hmacsign( key_file_name, test_file_name, signed_test_file_name, even_block_size, no_padding, 0 ); /* sign */
    hmacverify( key_file_name, signed_test_file_name, NULL, odd_block_size, reconstructed_test_file_name ); /* vrfy */
    result = compare_files(test_file_name, reconstructed_test_file_name); /* in linux this returns -1 for file not found, but windows returns 0 for file compare failure */
    /* if compare_files fails, the test case passes */
    /* the mismatch block size will cause an error in verify and as a result, remove the output file. The file comparison will return -1 */
    if( result == -1 )
    {
        result = 0;
    }
    PRINT_RESULT_AND_INCREMENT_FAILURES( !result, num_failures );

    printf( "Positive test case: file size 16k, block size 4096 (even), w/ padding:\n");
    hmacsign( key_file_name, test_file_name, signed_test_file_name, even_block_size, padding, 0 ); /* sign */
    hmacverify( key_file_name, signed_test_file_name, NULL, even_block_size, reconstructed_test_file_name ); /* vrfy */
    PRINT_RESULT_AND_INCREMENT_FAILURES( compare_files(test_file_name, reconstructed_test_file_name), num_failures );

    printf( "Positive test case: file size 16k, block size 4095 (odd), w/ padding:\n");
    hmacsign( key_file_name, test_file_name, signed_test_file_name, odd_block_size, padding, 0 ); /* sign */
    hmacverify( key_file_name, signed_test_file_name, NULL, odd_block_size, reconstructed_test_file_name ); /* vrfy */
    PRINT_RESULT_AND_INCREMENT_FAILURES( compare_files(test_file_name, reconstructed_test_file_name), num_failures );

    /* Random test cases */
    printf( "\nRandom test cases: %d iterations\n", number_random_test_iteration );
    while( random_test_iteration )
    {
        random_padding = rand() % 2;
        random_file_size = rand() % USHRT_MAX; /* limit to 64k-1 */
        random_block_size = rand() % 8192; /* limit to 8k-1 */
        while( random_block_size <= 32 )
        {
            random_block_size = rand() % 8192; /* make sure the block size is larger than 32 bytes to allow space for the signature */
        }

        printf( "Iteration %d: filesize %d, blocksize %d, padding %d (0 means pad)\n", number_random_test_iteration - random_test_iteration + 1, random_file_size, random_block_size, random_padding );
        generate_random_file( key_file_name, BCM_KEY_SIZE );
        generate_random_file( test_file_name, random_file_size );
        PRINT_RESULT_AND_INCREMENT_FAILURES( hmac_test_iteration( key_file_name, test_file_name, signed_test_file_name, reconstructed_test_file_name, random_block_size, random_padding ), num_failures );
        random_test_iteration--;
    }

    printf( "Finished %d random test cases with %d failures (%.1f%%)\n", number_random_test_iteration, num_failures, (float)num_failures/(float)number_random_test_iteration );

    remove( key_file_name );
    remove( test_file_name );
    remove( signed_test_file_name );
    remove( reconstructed_test_file_name );
    remove( signature_file_name );

    return num_failures;
}

void usage( void )
{
    printf( "Usage: hmac_sha256 <mode>\n"
        "  where <mode> is one of:\n"
        "  sign <keyfile> <infile> <signedfile> [blocksize] [padding]\n"
        "                                                     Sign the supplied file\n"
        "                                                     blocksize : Divide file into blocks of\n"
        "                                                     (blocksize - SHA256_DIGEST_LEN) and sign\n"
        "                                                     each block individually\n"
        "                                                     padding : 0 [Default] Pad file to SHA256 input alignment (448 mod 512)\n"
        "                                                     padding : 1 Do not pad the input file\n"
        "                                                         warning: if filesize is not a multiple of (blocksize-SHA256_DIGEST_LENGTH), output file will be truncated\n"
        "                                                         if the filesize is less than (blocksize-SHA256_DIGEST_LENGTH), output file will be empty\n"
        "  sig <keyfile> <infile> <signature>                 Generate the signature for <infile>\n"
        "  vrfy <keyfile> <signedfile> [blocksize] [outfile]  Verify the signature in <signedfile>\n"
        "                                                     blocksize : Divide file into blocks of\n"
        "                                                     (blocksize) and verify each block\n"
        "                                                     outfile : output file with signature removed\n"
        "  test                                               Run the self test\n"
        "  vsig <keyfile> <infile> <signature>                Verify <signature> against <infile>\n"
        "                                                     This is identical to vrfy <keyfile> <infile>\n");
    exit( 0 );
}

int main( int argc, const char **argv )
{
    int argn;
    const char *mode;

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
        return hmactest( );
    }
    else if ( strcmp( mode, "sig" ) == 0 )
    {
        if ( ( argn + 3 ) != argc )
        {
            printf( "* Incorrect parameters for sign\n" );
            usage( );
        }

        if ( hmacsign( argv[ argn ], argv[ argn + 1 ], argv[ argn + 2 ], 0, 0, 1 ) == 0 )
        {
            return -1;
        }
    }
    else if ( strcmp( mode, "sign" ) == 0 )
    {
        unsigned int blocksize;
        unsigned int no_padding;
        if ( ( argn + 3 ) == argc )
        {
            blocksize = 0;
        }
        else if ( ( argn + 5 ) == argc )
        {
            blocksize = atoi( argv[ argn + 3 ] );
            no_padding = atoi( argv [argn + 4 ] );
        }
        else
        {
            printf( "* Incorrect parameters for sig\n" );
            usage( );
        }
        if ( hmacsign( argv[ argn ], argv[ argn + 1 ], argv[ argn + 2 ], blocksize, no_padding, 0 ) == 0 )
        {
            return -1;
        }
    }
    else if ( strcmp( mode, "vrfy" ) == 0 )
    {
        unsigned int blocksize = 0;
        char *filename = NULL;

        if ( ( argn + 2 ) == argc )
        {
            blocksize = 0;
        }
        else if ( ( argn + 3 ) == argc )
        {
            blocksize = atoi( argv[ argn + 2 ] );
            filename = NULL;
        }
        else if ( ( argn + 4 ) == argc )
        {
            blocksize = atoi( argv[ argn + 2 ] );
            filename = (char *)argv[ argn + 3 ];
        }
        else
        {
            printf( "* Incorrect parameters for vrfy\n" );
            usage( );
        }

        if ( hmacverify( argv[ argn ], argv[ argn + 1 ], NULL, blocksize, filename ) == 0 )
        {
            return -1;
        }
    }
    else if ( strcmp( mode, "vsig" ) == 0 )
    {
        if ( ( argn + 3 ) != argc )
        {
            printf( "* Incorrect parameters for vsig\n" );
            usage( );
        }
        if ( hmacverify( argv[ argn ], argv[ argn + 1 ], argv[ argn + 2 ], 0, NULL ) == 0 )
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
