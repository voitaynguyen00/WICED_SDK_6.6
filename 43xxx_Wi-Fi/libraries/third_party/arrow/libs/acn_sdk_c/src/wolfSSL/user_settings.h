/* Copyright (c) 2017 Arrow Electronics, Inc.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Apache License 2.0
 * which accompanies this distribution, and is available at
 * http://apache.org/licenses/LICENSE-2.0
 * Contributors: Arrow Electronics, Inc.
 */

#if !defined(NO_DEV_RANDOM)
# define USE_RAND_DEFAULT
#endif

#if defined(ARCH_SSL)
# include <sys/arch/ssl.h>
#else

#if defined(__senseability__)
#define WOLFSSL_USER_IO
#define NO_WRITEV
#define NO_DEV_RANDOM
#define NO_WOLFSSL_DIR
#define NO_SHA512
#define NO_DH
#define NO_DSA
#define NO_RSA
#define NO_RC4
#define NO_DES
#define NO_DES3
#define NO_RABBIT
#define NO_AES
#define NO_ECC256
#define NO_ECC_DHE
#define NO_HC128
#define NO_PSK
#define NO_MD2
#define NO_MD4
#define NO_MD5
#define NO_OLD_TLS
#define NO_PWDBASED
#define NO_SKID
//#define NO_CURVE25519
//#define NO_ED25519
#define NO_WOLFSSL_SERVER
//#define NO_CERTS
#define SINGLE_THREADED
#define NO_STDIO_FILESYSTEM
//#define CTYPE_USER
//#define XMALLOC_USER
//#define SHA256_DIGEST_SIZE 32
#define WOLFSSL_BASE64_ENCODE
//#define DEBUG_WOLFSSL
#define WOLFSSL_STATIC_RSA
//#define HAVE_SUPPORTED_CURVES
//#define HAVE_TLS_EXTENSIONS
//#define SIZEOF_LONG_LONG  8
/* Options for Sample program */
#define NO_SESSION_CACHE // For Small RAM
//#define USE_CYASSL_MEMORY
//#define NO_WOLFSSL_MEMORY
#define WOLFSSL_NO_VERIFYSERVER
#define NO_FILESYSTEM
#define NO_CERT
#define HAVE_TM_TYPE

#ifndef WOLFSSL_NO_VERIFYSERVER
    #define TIME_OVERRIDES
    #define XTIME time
    #define XGMTIME localtime
#endif

#elif defined(__nrf52832__)
#define WOLFSSL_USER_IO
#define NO_WRITEV
#define NO_DEV_RANDOM
#define NO_WOLFSSL_DIR
#define NO_SHA512
#define NO_DH
#define NO_DSA
//#define NO_RSA
#define NO_RC4
#define NO_DES
#define NO_DES3
#define NO_RABBIT
//#define NO_AES
#define NO_ECC256
#define HAVE_ECC
//#define NO_ECC_DHE
#define NO_HC128
#define NO_PSK
#define NO_MD2
#define NO_MD4
//#define NO_MD5
#define NO_OLD_TLS
#define NO_PWDBASED
#define NO_SKID
//#define NO_CURVE25519
//#define NO_ED25519

#define WOLFSSL_STATIC_RSA
#define NO_WOLFSSL_SERVER
//#define NO_CERTS
#define SINGLE_THREADED
#define NO_STDIO_FILESYSTEM
//#define CTYPE_USER
//#define XMALLOC_USER
//#define SHA256_DIGEST_SIZE 32
#define WOLFSSL_BASE64_ENCODE
//#define DEBUG_WOLFSSL
//#define HAVE_SUPPORTED_CURVES
//#define HAVE_TLS_EXTENSIONS
//#define SIZEOF_LONG_LONG  8
/* Options for Sample program */
#define NO_SESSION_CACHE // For Small RAM
#define WOLFSSL_LOW_MEMORY
//#define WOLFSSL_SMALL_STACK
#define TFM_TIMING_RESISTANT
#define RSA_LOW_MEM
//#define USE_CYASSL_MEMORY
#define NO_WOLFSSL_MEMORY
//#define STATIC_CHUNKS_ONLY
//#define LARGE_STATIC_BUFFERS
#define WOLFSSL_NO_VERIFYSERVER
#define NO_FILESYSTEM
#define NO_CERT
#define HAVE_TM_TYPE
#ifndef WOLFSSL_NO_VERIFYSERVER
    #define TIME_OVERRIDES
    #define XTIME time
    #define XGMTIME localtime
#endif

#endif

#endif
