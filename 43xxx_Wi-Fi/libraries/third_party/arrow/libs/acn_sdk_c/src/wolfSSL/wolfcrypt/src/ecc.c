/* ecc.c
 *
 * Copyright (C) 2006-2016 wolfSSL Inc.
 *
 * This file is part of wolfSSL.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */



#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

/* in case user set HAVE_ECC there */
#include <wolfssl/wolfcrypt/settings.h>
typedef int __pedantic_dummy;

/*
Possible ECC enable options:
 * HAVE_ECC:            Overall control of ECC                  default: on
 * HAVE_ECC_ENCRYPT:    ECC encrypt/decrypt w/AES and HKDF      default: off
 * HAVE_ECC_SIGN:       ECC sign                                default: on
 * HAVE_ECC_VERIFY:     ECC verify                              default: on
 * HAVE_ECC_DHE:        ECC build shared secret                 default: on
 * HAVE_ECC_KEY_IMPORT: ECC Key import                          default: on
 * HAVE_ECC_KEY_EXPORT: ECC Key export                          default: on
 * ECC_SHAMIR:          Enables Shamir calc method              default: on
 * HAVE_COMP_KEY:       Enables compressed key                  default: off
 * WOLFSSL_VALIDATE_ECC_IMPORT: Validate ECC key on import      default: off
*/

/*
ECC Curves:
 * ECC_USER_CURVES: Allows custom combination of key sizes below
 * HAVE_ALL_CURVES: Enable all key sizes (on unless ECC_USER_CURVES is defined)
 * HAVE_ECC112: 112 bit key
 * HAVE_ECC128: 128 bit key
 * HAVE_ECC160: 160 bit key
 * HAVE_ECC192: 192 bit key
 * HAVE_ECC224: 224 bit key
 * NO_ECC256: Disables 256 bit key (on by default)
 * HAVE_ECC384: 384 bit key
 * HAVE_ECC521: 521 bit key
*/

#ifdef HAVE_ECC

#if (defined(HAVE_ECC_SIGN) || defined(HAVE_ECC_VERIFY)) && defined(NO_ASN)
    #error ASN must be enabled for ECC sign/verify
#endif

#include <wolfssl/wolfcrypt/ecc.h>
#include <wolfssl/openssl/ec.h>
#include <wolfssl/wolfcrypt/asn.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#include <wolfssl/wolfcrypt/logging.h>

#ifdef HAVE_ECC_ENCRYPT
    #include <wolfssl/wolfcrypt/hmac.h>
    #include <wolfssl/wolfcrypt/aes.h>
#endif

#ifdef NO_INLINE
    #include <wolfssl/wolfcrypt/misc.h>
#else
    #include <wolfcrypt/src/misc.c>
#endif

/* map

   ptmul -> mulmod

*/


/* p256 curve on by default whether user curves or not */
#if defined(HAVE_ECC112) || defined(HAVE_ALL_CURVES)
    #define ECC112
#endif
#if defined(HAVE_ECC128) || defined(HAVE_ALL_CURVES)
    #define ECC128
#endif
#if defined(HAVE_ECC160) || defined(HAVE_ALL_CURVES)
    #define ECC160
#endif
#if defined(HAVE_ECC192) || defined(HAVE_ALL_CURVES)
    #define ECC192
#endif
#if defined(HAVE_ECC224) || defined(HAVE_ALL_CURVES)
    #define ECC224
#endif
#if !defined(NO_ECC256)  || defined(HAVE_ALL_CURVES)
    #define ECC256
#endif
#if defined(HAVE_ECC384) || defined(HAVE_ALL_CURVES)
    #define ECC384
#endif
#if defined(HAVE_ECC521) || defined(HAVE_ALL_CURVES)
    #define ECC521
#endif



/* This holds the key settings.  ***MUST*** be organized by size from
   smallest to largest. */

const ecc_set_type ecc_sets[] = {
#ifdef ECC112
{
        14,
        NID_secp111r1,
        "SECP112R1",
        "DB7C2ABF62E35E668076BEAD208B",
        "DB7C2ABF62E35E668076BEAD2088",
        "659EF8BA043916EEDE8911702B22",
        "DB7C2ABF62E35E7628DFAC6561C5",
        "09487239995A5EE76B55F9C2F098",
        "A89CE5AF8724C0A23E0E0FF77500"
},
#endif
#ifdef ECC128
{
        16,
        NID_secp128r1,
        "SECP128R1",
        "FFFFFFFDFFFFFFFFFFFFFFFFFFFFFFFF",
        "FFFFFFFDFFFFFFFFFFFFFFFFFFFFFFFC",
        "E87579C11079F43DD824993C2CEE5ED3",
        "FFFFFFFE0000000075A30D1B9038A115",
        "161FF7528B899B2D0C28607CA52C5B86",
        "CF5AC8395BAFEB13C02DA292DDED7A83",
},
#endif
#ifdef ECC160
{
        20,
        NID_secp160r1,
        "SECP160R1",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF7FFFFFFF",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF7FFFFFFC",
        "1C97BEFC54BD7A8B65ACF89F81D4D4ADC565FA45",
        "0100000000000000000001F4C8F927AED3CA752257",
        "4A96B5688EF573284664698968C38BB913CBFC82",
        "23A628553168947D59DCC912042351377AC5FB32",
},
#endif
#ifdef ECC192
{
        24,
        NID_cert192,
        "ECC-192",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFF",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFC",
        "64210519E59C80E70FA7E9AB72243049FEB8DEECC146B9B1",
        "FFFFFFFFFFFFFFFFFFFFFFFF99DEF836146BC9B1B4D22831",
        "188DA80EB03090F67CBF20EB43A18800F4FF0AFD82FF1012",
        "7192B95FFC8DA78631011ED6B24CDD573F977A11E794811",
},
#endif
#ifdef ECC224
{
        28,
        NID_cert224,
        "ECC-224",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFFFFFFFFFE",
        "B4050A850C04B3ABF54132565044B0B7D7BFD8BA270B39432355FFB4",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFF16A2E0B8F03E13DD29455C5C2A3D",
        "B70E0CBD6BB4BF7F321390B94A03C1D356C21122343280D6115C1D21",
        "BD376388B5F723FB4C22DFE6CD4375A05A07476444D5819985007E34",
},
#endif
#ifdef ECC256
{
        32,
        NID_X9_62_prime256v1,
        "nistp256",
        "FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF",
        "FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFC",
        "5AC635D8AA3A93E7B3EBBD55769886BC651D06B0CC53B0F63BCE3C3E27D2604B",
        "FFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551",
        "6B17D1F2E12C4247F8BCE6E563A440F277037D812DEB33A0F4A13945D898C296",
        "4FE342E2FE1A7F9B8EE7EB4A7C0F9E162BCE33576B315ECECBB6406837BF51F5",
},
#endif
#ifdef ECC384
{
        48,
        NID_secp384r1,
        "nistp384",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFF0000000000000000FFFFFFFF",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFF0000000000000000FFFFFFFC",
        "B3312FA7E23EE7E4988E056BE3F82D19181D9C6EFE8141120314088F5013875AC656398D8A2ED19D2A85C8EDD3EC2AEF",
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC7634D81F4372DDF581A0DB248B0A77AECEC196ACCC52973",
        "AA87CA22BE8B05378EB1C71EF320AD746E1D3B628BA79B9859F741E082542A385502F25DBF55296C3A545E3872760AB7",
        "3617DE4A96262C6F5D9E98BF9292DC29F8F41DBD289A147CE9DA3113B5F0B8C00A60B1CE1D7E819D7A431D7C90EA0E5F",
},
#endif
#ifdef ECC521
{
        66,
        NID_secp521r1,
        "nistp521",
        "1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        "1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC",
        "51953EB9618E1C9A1F929A21A0B68540EEA2DA725B99B315F3B8B489918EF109E156193951EC7E937B1652C0BD3BB1BF073573DF883D2C34F1EF451FD46B503F00",
        "1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFA51868783BF2F966B7FCC0148F709A5D03BB5C9B8899C47AEBB6FB71E91386409",
        "C6858E06B70404E9CD9E3ECB662395B4429C648139053FB521F828AF606B4D3DBAA14B5E77EFE75928FE1DC127A2FFA8DE3348B3C1856A429BF97E7E31C2E5BD66",
        "11839296A789A3BC0045C8A5FB42C7D1BD998F54449579B446817AFBD17273E662C97EE72995EF42640C550B9013FAD0761353C7086A272C24088BE94769FD16650",
},
#endif
{
   0, -1,
   NULL, NULL, NULL, NULL, NULL, NULL, NULL
}
};


int  ecc_map(ecc_point*, mp_int*, mp_digit*);
int  ecc_projective_add_point(ecc_point* P, ecc_point* Q, ecc_point* R,
                              mp_int* modulus, mp_digit* mp);
int  ecc_projective_dbl_point(ecc_point* P, ecc_point* R, mp_int* modulus,
                              mp_digit* mp);
static int ecc_check_pubkey_order(ecc_key* key, mp_int* prime, mp_int* order);
#ifdef ECC_SHAMIR
static int ecc_mul2add(ecc_point* A, mp_int* kA, ecc_point* B, mp_int* kB,
                       ecc_point* C, mp_int* modulus);
#endif

int mp_jacobi(mp_int* a, mp_int* p, int* c);
int mp_sqrtmod_prime(mp_int* n, mp_int* prime, mp_int* ret);
int mp_submod(mp_int* a, mp_int* b, mp_int* c, mp_int* d);

#ifdef HAVE_COMP_KEY
static int wc_ecc_export_x963_compressed(ecc_key*, byte* out, word32* outLen);
#endif

/* helper for either lib */
static int get_digit_count(mp_int* a)
{
    if (a == NULL)
        return 0;

    return a->used;
}

/* helper for either lib */
static mp_digit get_digit(mp_int* a, int n)
{
    if (a == NULL)
        return 0;

    return (n >= a->used || n < 0) ? 0 : a->dp[n];
}


#if defined(USE_FAST_MATH)

/* fast math accelerated version, but not for fp ecc yet */

/**
   Add two ECC points
   P        The point to add
   Q        The point to add
   R        [out] The destination of the double
   modulus  The modulus of the field the ECC curve is in
   mp       The "b" value from montgomery_setup()
   return   MP_OKAY on success
*/
int ecc_projective_add_point(ecc_point *P, ecc_point *Q, ecc_point *R,
                             mp_int* modulus, mp_digit* mp)
{
   fp_int t1, t2, x, y, z;
   int    err;

   if (P == NULL || Q == NULL || R == NULL || modulus == NULL || mp == NULL)
       return ECC_BAD_ARG_E;

   if ((err = mp_init_multi(&t1, &t2, &x, &y, &z, NULL)) != MP_OKAY) {
      return err;
   }

   /* should we dbl instead? */
   fp_sub(modulus, Q->y, &t1);
   if ( (fp_cmp(P->x, Q->x) == FP_EQ) &&
        (get_digit_count(Q->z) && fp_cmp(P->z, Q->z) == FP_EQ) &&
        (fp_cmp(P->y, Q->y) == FP_EQ || fp_cmp(P->y, &t1) == FP_EQ)) {
        return ecc_projective_dbl_point(P, R, modulus, mp);
   }

   fp_copy(P->x, &x);
   fp_copy(P->y, &y);
   fp_copy(P->z, &z);

   /* if Z is one then these are no-operations */
   if (get_digit_count(Q->z)) {
      /* T1 = Z' * Z' */
      fp_sqr(Q->z, &t1);
      fp_montgomery_reduce(&t1, modulus, *mp);
      /* X = X * T1 */
      fp_mul(&t1, &x, &x);
      fp_montgomery_reduce(&x, modulus, *mp);
      /* T1 = Z' * T1 */
      fp_mul(Q->z, &t1, &t1);
      fp_montgomery_reduce(&t1, modulus, *mp);
      /* Y = Y * T1 */
      fp_mul(&t1, &y, &y);
      fp_montgomery_reduce(&y, modulus, *mp);
   }

   /* T1 = Z*Z */
   fp_sqr(&z, &t1);
   fp_montgomery_reduce(&t1, modulus, *mp);
   /* T2 = X' * T1 */
   fp_mul(Q->x, &t1, &t2);
   fp_montgomery_reduce(&t2, modulus, *mp);
   /* T1 = Z * T1 */
   fp_mul(&z, &t1, &t1);
   fp_montgomery_reduce(&t1, modulus, *mp);
   /* T1 = Y' * T1 */
   fp_mul(Q->y, &t1, &t1);
   fp_montgomery_reduce(&t1, modulus, *mp);

   /* Y = Y - T1 */
   fp_sub(&y, &t1, &y);
   if (fp_cmp_d(&y, 0) == FP_LT) {
      fp_add(&y, modulus, &y);
   }
   /* T1 = 2T1 */
   fp_add(&t1, &t1, &t1);
   if (fp_cmp(&t1, modulus) != FP_LT) {
      fp_sub(&t1, modulus, &t1);
   }
   /* T1 = Y + T1 */
   fp_add(&t1, &y, &t1);
   if (fp_cmp(&t1, modulus) != FP_LT) {
      fp_sub(&t1, modulus, &t1);
   }
   /* X = X - T2 */
   fp_sub(&x, &t2, &x);
   if (fp_cmp_d(&x, 0) == FP_LT) {
      fp_add(&x, modulus, &x);
   }
   /* T2 = 2T2 */
   fp_add(&t2, &t2, &t2);
   if (fp_cmp(&t2, modulus) != FP_LT) {
      fp_sub(&t2, modulus, &t2);
   }
   /* T2 = X + T2 */
   fp_add(&t2, &x, &t2);
   if (fp_cmp(&t2, modulus) != FP_LT) {
      fp_sub(&t2, modulus, &t2);
   }

   /* if Z' != 1 */
   if (get_digit_count(Q->z)) {
      /* Z = Z * Z' */
      fp_mul(&z, Q->z, &z);
      fp_montgomery_reduce(&z, modulus, *mp);
   }

   /* Z = Z * X */
   fp_mul(&z, &x, &z);
   fp_montgomery_reduce(&z, modulus, *mp);

   /* T1 = T1 * X  */
   fp_mul(&t1, &x, &t1);
   fp_montgomery_reduce(&t1, modulus, *mp);
   /* X = X * X */
   fp_sqr(&x, &x);
   fp_montgomery_reduce(&x, modulus, *mp);
   /* T2 = T2 * x */
   fp_mul(&t2, &x, &t2);
   fp_montgomery_reduce(&t2, modulus, *mp);
   /* T1 = T1 * X  */
   fp_mul(&t1, &x, &t1);
   fp_montgomery_reduce(&t1, modulus, *mp);

   /* X = Y*Y */
   fp_sqr(&y, &x);
   fp_montgomery_reduce(&x, modulus, *mp);
   /* X = X - T2 */
   fp_sub(&x, &t2, &x);
   if (fp_cmp_d(&x, 0) == FP_LT) {
      fp_add(&x, modulus, &x);
   }

   /* T2 = T2 - X */
   fp_sub(&t2, &x, &t2);
   if (fp_cmp_d(&t2, 0) == FP_LT) {
      fp_add(&t2, modulus, &t2);
   }
   /* T2 = T2 - X */
   fp_sub(&t2, &x, &t2);
   if (fp_cmp_d(&t2, 0) == FP_LT) {
      fp_add(&t2, modulus, &t2);
   }
   /* T2 = T2 * Y */
   fp_mul(&t2, &y, &t2);
   fp_montgomery_reduce(&t2, modulus, *mp);
   /* Y = T2 - T1 */
   fp_sub(&t2, &t1, &y);
   if (fp_cmp_d(&y, 0) == FP_LT) {
      fp_add(&y, modulus, &y);
   }
   /* Y = Y/2 */
   if (fp_isodd(&y)) {
      fp_add(&y, modulus, &y);
   }
   fp_div_2(&y, &y);

   fp_copy(&x, R->x);
   fp_copy(&y, R->y);
   fp_copy(&z, R->z);

   return MP_OKAY;
}


/**
   Double an ECC point
   P   The point to double
   R   [out] The destination of the double
   modulus  The modulus of the field the ECC curve is in
   mp       The "b" value from montgomery_setup()
   return   MP_OKAY on success
*/
int ecc_projective_dbl_point(ecc_point *P, ecc_point *R, mp_int* modulus,
                             mp_digit* mp)
{
   fp_int   t1, t2;
   int      err;

   if (P == NULL || R == NULL || modulus == NULL || mp == NULL)
       return ECC_BAD_ARG_E;

   if (P != R) {
      fp_copy(P->x, R->x);
      fp_copy(P->y, R->y);
      fp_copy(P->z, R->z);
   }

   if ((err = mp_init_multi(&t1, &t2, NULL, NULL, NULL, NULL)) != MP_OKAY) {
      return err;
   }

   /* t1 = Z * Z */
   fp_sqr(R->z, &t1);
   fp_montgomery_reduce(&t1, modulus, *mp);
   /* Z = Y * Z */
   fp_mul(R->z, R->y, R->z);
   fp_montgomery_reduce(R->z, modulus, *mp);
   /* Z = 2Z */
   fp_add(R->z, R->z, R->z);
   if (fp_cmp(R->z, modulus) != FP_LT) {
      fp_sub(R->z, modulus, R->z);
   }

   /* &t2 = X - T1 */
   fp_sub(R->x, &t1, &t2);
   if (fp_cmp_d(&t2, 0) == FP_LT) {
      fp_add(&t2, modulus, &t2);
   }
   /* T1 = X + T1 */
   fp_add(&t1, R->x, &t1);
   if (fp_cmp(&t1, modulus) != FP_LT) {
      fp_sub(&t1, modulus, &t1);
   }
   /* T2 = T1 * T2 */
   fp_mul(&t1, &t2, &t2);
   fp_montgomery_reduce(&t2, modulus, *mp);
   /* T1 = 2T2 */
   fp_add(&t2, &t2, &t1);
   if (fp_cmp(&t1, modulus) != FP_LT) {
      fp_sub(&t1, modulus, &t1);
   }
   /* T1 = T1 + T2 */
   fp_add(&t1, &t2, &t1);
   if (fp_cmp(&t1, modulus) != FP_LT) {
      fp_sub(&t1, modulus, &t1);
   }

   /* Y = 2Y */
   fp_add(R->y, R->y, R->y);
   if (fp_cmp(R->y, modulus) != FP_LT) {
      fp_sub(R->y, modulus, R->y);
   }
   /* Y = Y * Y */
   fp_sqr(R->y, R->y);
   fp_montgomery_reduce(R->y, modulus, *mp);
   /* T2 = Y * Y */
   fp_sqr(R->y, &t2);
   fp_montgomery_reduce(&t2, modulus, *mp);
   /* T2 = T2/2 */
   if (fp_isodd(&t2)) {
      fp_add(&t2, modulus, &t2);
   }
   fp_div_2(&t2, &t2);
   /* Y = Y * X */
   fp_mul(R->y, R->x, R->y);
   fp_montgomery_reduce(R->y, modulus, *mp);

   /* X  = T1 * T1 */
   fp_sqr(&t1, R->x);
   fp_montgomery_reduce(R->x, modulus, *mp);
   /* X = X - Y */
   fp_sub(R->x, R->y, R->x);
   if (fp_cmp_d(R->x, 0) == FP_LT) {
      fp_add(R->x, modulus, R->x);
   }
   /* X = X - Y */
   fp_sub(R->x, R->y, R->x);
   if (fp_cmp_d(R->x, 0) == FP_LT) {
      fp_add(R->x, modulus, R->x);
   }

   /* Y = Y - X */
   fp_sub(R->y, R->x, R->y);
   if (fp_cmp_d(R->y, 0) == FP_LT) {
      fp_add(R->y, modulus, R->y);
   }
   /* Y = Y * T1 */
   fp_mul(R->y, &t1, R->y);
   fp_montgomery_reduce(R->y, modulus, *mp);
   /* Y = Y - T2 */
   fp_sub(R->y, &t2, R->y);
   if (fp_cmp_d(R->y, 0) == FP_LT) {
      fp_add(R->y, modulus, R->y);
   }

   return MP_OKAY;
}

#else /* USE_FAST_MATH */

/**
   Add two ECC points
   P        The point to add
   Q        The point to add
   R        [out] The destination of the double
   modulus  The modulus of the field the ECC curve is in
   mp       The "b" value from montgomery_setup()
   return   MP_OKAY on success
*/
int ecc_projective_add_point(ecc_point* P, ecc_point* Q, ecc_point* R,
                             mp_int* modulus, mp_digit* mp)
{
   mp_int t1;
   mp_int t2;
   mp_int x;
   mp_int y;
   mp_int z;
   int    err;

   if (P == NULL || Q == NULL || R == NULL || modulus == NULL || mp == NULL)
       return ECC_BAD_ARG_E;

   if ((err = mp_init_multi(&t1, &t2, &x, &y, &z, NULL)) != MP_OKAY) {
      return err;
   }

   /* should we dbl instead? */
   err = mp_sub(modulus, Q->y, &t1);

   if (err == MP_OKAY) {
       if ( (mp_cmp(P->x, Q->x) == MP_EQ) &&
            (get_digit_count(Q->z) && mp_cmp(P->z, Q->z) == MP_EQ) &&
            (mp_cmp(P->y, Q->y) == MP_EQ || mp_cmp(P->y, &t1) == MP_EQ)) {
                mp_clear(&t1);
                mp_clear(&t2);
                mp_clear(&x);
                mp_clear(&y);
                mp_clear(&z);

                return ecc_projective_dbl_point(P, R, modulus, mp);
       }
   }

   if (err == MP_OKAY)
       err = mp_copy(P->x, &x);
   if (err == MP_OKAY)
       err = mp_copy(P->y, &y);
   if (err == MP_OKAY)
       err = mp_copy(P->z, &z);

   /* if Z is one then these are no-operations */
   if (err == MP_OKAY) {
       if (get_digit_count(Q->z)) {
           /* T1 = Z' * Z' */
           err = mp_sqr(Q->z, &t1);
           if (err == MP_OKAY)
               err = mp_montgomery_reduce(&t1, modulus, *mp);

           /* X = X * T1 */
           if (err == MP_OKAY)
               err = mp_mul(&t1, &x, &x);
           if (err == MP_OKAY)
               err = mp_montgomery_reduce(&x, modulus, *mp);

           /* T1 = Z' * T1 */
           if (err == MP_OKAY)
               err = mp_mul(Q->z, &t1, &t1);
           if (err == MP_OKAY)
               err = mp_montgomery_reduce(&t1, modulus, *mp);

           /* Y = Y * T1 */
           if (err == MP_OKAY)
               err = mp_mul(&t1, &y, &y);
           if (err == MP_OKAY)
               err = mp_montgomery_reduce(&y, modulus, *mp);
       }
   }

   /* T1 = Z*Z */
   if (err == MP_OKAY)
       err = mp_sqr(&z, &t1);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t1, modulus, *mp);

   /* T2 = X' * T1 */
   if (err == MP_OKAY)
       err = mp_mul(Q->x, &t1, &t2);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t2, modulus, *mp);

   /* T1 = Z * T1 */
   if (err == MP_OKAY)
       err = mp_mul(&z, &t1, &t1);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t1, modulus, *mp);

   /* T1 = Y' * T1 */
   if (err == MP_OKAY)
       err = mp_mul(Q->y, &t1, &t1);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t1, modulus, *mp);

   /* Y = Y - T1 */
   if (err == MP_OKAY)
       err = mp_sub(&y, &t1, &y);
   if (err == MP_OKAY) {
       if (mp_cmp_d(&y, 0) == MP_LT)
           err = mp_add(&y, modulus, &y);
   }
   /* T1 = 2T1 */
   if (err == MP_OKAY)
       err = mp_add(&t1, &t1, &t1);
   if (err == MP_OKAY) {
       if (mp_cmp(&t1, modulus) != MP_LT)
           err = mp_sub(&t1, modulus, &t1);
   }
   /* T1 = Y + T1 */
   if (err == MP_OKAY)
       err = mp_add(&t1, &y, &t1);
   if (err == MP_OKAY) {
       if (mp_cmp(&t1, modulus) != MP_LT)
           err = mp_sub(&t1, modulus, &t1);
   }
   /* X = X - T2 */
   if (err == MP_OKAY)
       err = mp_sub(&x, &t2, &x);
   if (err == MP_OKAY) {
       if (mp_cmp_d(&x, 0) == MP_LT)
           err = mp_add(&x, modulus, &x);
   }
   /* T2 = 2T2 */
   if (err == MP_OKAY)
       err = mp_add(&t2, &t2, &t2);
   if (err == MP_OKAY) {
       if (mp_cmp(&t2, modulus) != MP_LT)
           err = mp_sub(&t2, modulus, &t2);
   }
   /* T2 = X + T2 */
   if (err == MP_OKAY)
       err = mp_add(&t2, &x, &t2);
   if (err == MP_OKAY) {
       if (mp_cmp(&t2, modulus) != MP_LT)
           err = mp_sub(&t2, modulus, &t2);
   }

   if (err == MP_OKAY) {
       if (get_digit_count(Q->z)) {
           /* Z = Z * Z' */
           err = mp_mul(&z, Q->z, &z);
           if (err == MP_OKAY)
               err = mp_montgomery_reduce(&z, modulus, *mp);
       }
   }

   /* Z = Z * X */
   if (err == MP_OKAY)
       err = mp_mul(&z, &x, &z);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&z, modulus, *mp);

   /* T1 = T1 * X  */
   if (err == MP_OKAY)
       err = mp_mul(&t1, &x, &t1);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t1, modulus, *mp);

   /* X = X * X */
   if (err == MP_OKAY)
       err = mp_sqr(&x, &x);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&x, modulus, *mp);

   /* T2 = T2 * x */
   if (err == MP_OKAY)
       err = mp_mul(&t2, &x, &t2);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t2, modulus, *mp);

   /* T1 = T1 * X  */
   if (err == MP_OKAY)
       err = mp_mul(&t1, &x, &t1);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t1, modulus, *mp);

   /* X = Y*Y */
   if (err == MP_OKAY)
       err = mp_sqr(&y, &x);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&x, modulus, *mp);

   /* X = X - T2 */
   if (err == MP_OKAY)
       err = mp_sub(&x, &t2, &x);
   if (err == MP_OKAY) {
       if (mp_cmp_d(&x, 0) == MP_LT)
           err = mp_add(&x, modulus, &x);
   }
   /* T2 = T2 - X */
   if (err == MP_OKAY)
       err = mp_sub(&t2, &x, &t2);
   if (err == MP_OKAY) {
       if (mp_cmp_d(&t2, 0) == MP_LT)
           err = mp_add(&t2, modulus, &t2);
   }
   /* T2 = T2 - X */
   if (err == MP_OKAY)
       err = mp_sub(&t2, &x, &t2);
   if (err == MP_OKAY) {
       if (mp_cmp_d(&t2, 0) == MP_LT)
           err = mp_add(&t2, modulus, &t2);
   }
   /* T2 = T2 * Y */
   if (err == MP_OKAY)
       err = mp_mul(&t2, &y, &t2);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t2, modulus, *mp);

   /* Y = T2 - T1 */
   if (err == MP_OKAY)
       err = mp_sub(&t2, &t1, &y);
   if (err == MP_OKAY) {
       if (mp_cmp_d(&y, 0) == MP_LT)
           err = mp_add(&y, modulus, &y);
   }
   /* Y = Y/2 */
   if (err == MP_OKAY) {
       if (mp_isodd(&y))
           err = mp_add(&y, modulus, &y);
   }
   if (err == MP_OKAY)
       err = mp_div_2(&y, &y);

   if (err == MP_OKAY)
       err = mp_copy(&x, R->x);
   if (err == MP_OKAY)
       err = mp_copy(&y, R->y);
   if (err == MP_OKAY)
       err = mp_copy(&z, R->z);

   /* clean up */
   mp_clear(&t1);
   mp_clear(&t2);
   mp_clear(&x);
   mp_clear(&y);
   mp_clear(&z);

   return err;
}


/**
   Double an ECC point
   P   The point to double
   R   [out] The destination of the double
   modulus  The modulus of the field the ECC curve is in
   mp       The "b" value from montgomery_setup()
   return   MP_OKAY on success
*/
int ecc_projective_dbl_point(ecc_point *P, ecc_point *R, mp_int* modulus,
                             mp_digit* mp)
{
   mp_int t1;
   mp_int t2;
   int    err;

   if (P == NULL || R == NULL || modulus == NULL || mp == NULL)
       return ECC_BAD_ARG_E;

   if ((err = mp_init_multi(&t1, &t2, NULL, NULL, NULL, NULL)) != MP_OKAY) {
      return err;
   }

   if (P != R) {
      err = mp_copy(P->x, R->x);
      if (err == MP_OKAY)
          err = mp_copy(P->y, R->y);
      if (err == MP_OKAY)
          err = mp_copy(P->z, R->z);
   }

   /* t1 = Z * Z */
   if (err == MP_OKAY)
       err = mp_sqr(R->z, &t1);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t1, modulus, *mp);

   /* Z = Y * Z */
   if (err == MP_OKAY)
       err = mp_mul(R->z, R->y, R->z);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(R->z, modulus, *mp);

   /* Z = 2Z */
   if (err == MP_OKAY)
       err = mp_add(R->z, R->z, R->z);
   if (err == MP_OKAY) {
       if (mp_cmp(R->z, modulus) != MP_LT)
           err = mp_sub(R->z, modulus, R->z);
   }

   /* T2 = X - T1 */
   if (err == MP_OKAY)
       err = mp_sub(R->x, &t1, &t2);
   if (err == MP_OKAY) {
       if (mp_cmp_d(&t2, 0) == MP_LT)
           err = mp_add(&t2, modulus, &t2);
   }
   /* T1 = X + T1 */
   if (err == MP_OKAY)
       err = mp_add(&t1, R->x, &t1);
   if (err == MP_OKAY) {
       if (mp_cmp(&t1, modulus) != MP_LT)
           err = mp_sub(&t1, modulus, &t1);
   }
   /* T2 = T1 * T2 */
   if (err == MP_OKAY)
       err = mp_mul(&t1, &t2, &t2);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t2, modulus, *mp);

   /* T1 = 2T2 */
   if (err == MP_OKAY)
       err = mp_add(&t2, &t2, &t1);
   if (err == MP_OKAY) {
       if (mp_cmp(&t1, modulus) != MP_LT)
           err = mp_sub(&t1, modulus, &t1);
   }
   /* T1 = T1 + T2 */
   if (err == MP_OKAY)
       err = mp_add(&t1, &t2, &t1);
   if (err == MP_OKAY) {
       if (mp_cmp(&t1, modulus) != MP_LT)
           err = mp_sub(&t1, modulus, &t1);
   }
   /* Y = 2Y */
   if (err == MP_OKAY)
       err = mp_add(R->y, R->y, R->y);
   if (err == MP_OKAY) {
       if (mp_cmp(R->y, modulus) != MP_LT)
           err = mp_sub(R->y, modulus, R->y);
   }
   /* Y = Y * Y */
   if (err == MP_OKAY)
       err = mp_sqr(R->y, R->y);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(R->y, modulus, *mp);

   /* T2 = Y * Y */
   if (err == MP_OKAY)
       err = mp_sqr(R->y, &t2);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(&t2, modulus, *mp);

   /* T2 = T2/2 */
   if (err == MP_OKAY) {
       if (mp_isodd(&t2))
           err = mp_add(&t2, modulus, &t2);
   }
   if (err == MP_OKAY)
       err = mp_div_2(&t2, &t2);

   /* Y = Y * X */
   if (err == MP_OKAY)
       err = mp_mul(R->y, R->x, R->y);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(R->y, modulus, *mp);

   /* X  = T1 * T1 */
   if (err == MP_OKAY)
       err = mp_sqr(&t1, R->x);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(R->x, modulus, *mp);

   /* X = X - Y */
   if (err == MP_OKAY)
       err = mp_sub(R->x, R->y, R->x);
   if (err == MP_OKAY) {
       if (mp_cmp_d(R->x, 0) == MP_LT)
           err = mp_add(R->x, modulus, R->x);
   }
   /* X = X - Y */
   if (err == MP_OKAY)
       err = mp_sub(R->x, R->y, R->x);
   if (err == MP_OKAY) {
       if (mp_cmp_d(R->x, 0) == MP_LT)
           err = mp_add(R->x, modulus, R->x);
   }
   /* Y = Y - X */
   if (err == MP_OKAY)
       err = mp_sub(R->y, R->x, R->y);
   if (err == MP_OKAY) {
       if (mp_cmp_d(R->y, 0) == MP_LT)
           err = mp_add(R->y, modulus, R->y);
   }
   /* Y = Y * T1 */
   if (err == MP_OKAY)
       err = mp_mul(R->y, &t1, R->y);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(R->y, modulus, *mp);

   /* Y = Y - T2 */
   if (err == MP_OKAY)
       err = mp_sub(R->y, &t2, R->y);
   if (err == MP_OKAY) {
       if (mp_cmp_d(R->y, 0) == MP_LT)
           err = mp_add(R->y, modulus, R->y);
   }

   /* clean up */
   mp_clear(&t1);
   mp_clear(&t2);

   return err;
}

#endif /* USE_FAST_MATH */

/**
  Map a projective jacbobian point back to affine space
  P        [in/out] The point to map
  modulus  The modulus of the field the ECC curve is in
  mp       The "b" value from montgomery_setup()
  return   MP_OKAY on success
*/
int ecc_map(ecc_point* P, mp_int* modulus, mp_digit* mp)
{
   mp_int t1;
   mp_int t2;
   int    err;

   if (P == NULL || mp == NULL || modulus == NULL)
       return ECC_BAD_ARG_E;

   /* special case for point at infinity */
   if (mp_cmp_d(P->z, 0) == MP_EQ) {
       mp_set(P->x, 0);
       mp_set(P->y, 0);
       mp_set(P->z, 1);
       return MP_OKAY;
   }

   if ((err = mp_init_multi(&t1, &t2, NULL, NULL, NULL, NULL)) != MP_OKAY) {
      return MEMORY_E;
   }

   /* first map z back to normal */
   err = mp_montgomery_reduce(P->z, modulus, *mp);

   /* get 1/z */
   if (err == MP_OKAY)
       err = mp_invmod(P->z, modulus, &t1);

   /* get 1/z^2 and 1/z^3 */
   if (err == MP_OKAY)
       err = mp_sqr(&t1, &t2);
   if (err == MP_OKAY)
       err = mp_mod(&t2, modulus, &t2);
   if (err == MP_OKAY)
       err = mp_mul(&t1, &t2, &t1);
   if (err == MP_OKAY)
       err = mp_mod(&t1, modulus, &t1);

   /* multiply against x/y */
   if (err == MP_OKAY)
       err = mp_mul(P->x, &t2, P->x);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(P->x, modulus, *mp);
   if (err == MP_OKAY)
       err = mp_mul(P->y, &t1, P->y);
   if (err == MP_OKAY)
       err = mp_montgomery_reduce(P->y, modulus, *mp);

   if (err == MP_OKAY)
       mp_set(P->z, 1);

   /* clean up */
   mp_clear(&t1);
   mp_clear(&t2);

   return err;
}


#ifndef ECC_TIMING_RESISTANT

/* size of sliding window, don't change this! */
#define WINSIZE 4

/**
   Perform a point multiplication
   k    The scalar to multiply by
   G    The base point
   R    [out] Destination for kG
   modulus  The modulus of the field the ECC curve is in
   map      Boolean whether to map back to affine or not
                (1==map, 0 == leave in projective)
   return MP_OKAY on success
*/
#ifdef FP_ECC
static int normal_ecc_mulmod(mp_int* k, ecc_point *G, ecc_point *R,
                      mp_int* modulus, int map)
#else
int wc_ecc_mulmod(mp_int* k, ecc_point *G, ecc_point *R, mp_int* modulus,
               int map)
#endif
{
   ecc_point *tG, *M[8];
   int           i, j, err;
   mp_int        mu;
   mp_digit      mp;
   mp_digit      buf;
   int           first = 1, bitbuf = 0, bitcpy = 0, bitcnt = 0, mode = 0,
                 digidx = 0;

   if (k == NULL || G == NULL || R == NULL || modulus == NULL)
       return ECC_BAD_ARG_E;

   /* init montgomery reduction */
   if ((err = mp_montgomery_setup(modulus, &mp)) != MP_OKAY) {
      return err;
   }
   if ((err = mp_init(&mu)) != MP_OKAY) {
      return err;
   }
   if ((err = mp_montgomery_calc_normalization(&mu, modulus)) != MP_OKAY) {
      mp_clear(&mu);
      return err;
   }

  /* alloc ram for window temps */
  for (i = 0; i < 8; i++) {
      M[i] = wc_ecc_new_point();
      if (M[i] == NULL) {
         for (j = 0; j < i; j++) {
             wc_ecc_del_point(M[j]);
         }
         mp_clear(&mu);
         return MEMORY_E;
      }
  }

   /* make a copy of G in case R==G */
   tG = wc_ecc_new_point();
   if (tG == NULL)
       err = MEMORY_E;

   /* tG = G  and convert to montgomery */
   if (err == MP_OKAY) {
       if (mp_cmp_d(&mu, 1) == MP_EQ) {
           err = mp_copy(G->x, tG->x);
           if (err == MP_OKAY)
               err = mp_copy(G->y, tG->y);
           if (err == MP_OKAY)
               err = mp_copy(G->z, tG->z);
       } else {
           err = mp_mulmod(G->x, &mu, modulus, tG->x);
           if (err == MP_OKAY)
               err = mp_mulmod(G->y, &mu, modulus, tG->y);
           if (err == MP_OKAY)
               err = mp_mulmod(G->z, &mu, modulus, tG->z);
       }
   }
   mp_clear(&mu);

   /* calc the M tab, which holds kG for k==8..15 */
   /* M[0] == 8G */
   if (err == MP_OKAY)
       err = ecc_projective_dbl_point(tG, M[0], modulus, &mp);
   if (err == MP_OKAY)
       err = ecc_projective_dbl_point(M[0], M[0], modulus, &mp);
   if (err == MP_OKAY)
       err = ecc_projective_dbl_point(M[0], M[0], modulus, &mp);

   /* now find (8+k)G for k=1..7 */
   if (err == MP_OKAY)
       for (j = 9; j < 16; j++) {
           err = ecc_projective_add_point(M[j-9], tG, M[j-8], modulus, &mp);
           if (err != MP_OKAY) break;
       }

   /* setup sliding window */
   if (err == MP_OKAY) {
       mode   = 0;
       bitcnt = 1;
       buf    = 0;
       digidx = get_digit_count(k) - 1;
       bitcpy = bitbuf = 0;
       first  = 1;

       /* perform ops */
       for (;;) {
           /* grab next digit as required */
           if (--bitcnt == 0) {
               if (digidx == -1) {
                   break;
               }
               buf    = get_digit(k, digidx);
               bitcnt = (int) DIGIT_BIT;
               --digidx;
           }

           /* grab the next msb from the ltiplicand */
           i = (int)(buf >> (DIGIT_BIT - 1)) & 1;
           buf <<= 1;

           /* skip leading zero bits */
           if (mode == 0 && i == 0)
               continue;

           /* if the bit is zero and mode == 1 then we double */
           if (mode == 1 && i == 0) {
               err = ecc_projective_dbl_point(R, R, modulus, &mp);
               if (err != MP_OKAY) break;
               continue;
           }

           /* else we add it to the window */
           bitbuf |= (i << (WINSIZE - ++bitcpy));
           mode = 2;

           if (bitcpy == WINSIZE) {
               /* if this is the first window we do a simple copy */
               if (first == 1) {
                   /* R = kG [k = first window] */
                   err = mp_copy(M[bitbuf-8]->x, R->x);
                   if (err != MP_OKAY) break;

                   err = mp_copy(M[bitbuf-8]->y, R->y);
                   if (err != MP_OKAY) break;

                   err = mp_copy(M[bitbuf-8]->z, R->z);
                   first = 0;
               } else {
                   /* normal window */
                   /* ok window is filled so double as required and add  */
                   /* double first */
                   for (j = 0; j < WINSIZE; j++) {
                       err = ecc_projective_dbl_point(R, R, modulus, &mp);
                       if (err != MP_OKAY) break;
                   }
                   if (err != MP_OKAY) break;  /* out of first for(;;) */

                   /* then add, bitbuf will be 8..15 [8..2^WINSIZE] guaranteed */
                   err = ecc_projective_add_point(R,M[bitbuf-8],R,modulus,&mp);
               }
               if (err != MP_OKAY) break;
               /* empty window and reset */
               bitcpy = bitbuf = 0;
               mode = 1;
           }
       }
   }

   /* if bits remain then double/add */
   if (err == MP_OKAY) {
       if (mode == 2 && bitcpy > 0) {
           /* double then add */
           for (j = 0; j < bitcpy; j++) {
               /* only double if we have had at least one add first */
               if (first == 0) {
                   err = ecc_projective_dbl_point(R, R, modulus, &mp);
                   if (err != MP_OKAY) break;
               }

               bitbuf <<= 1;
               if ((bitbuf & (1 << WINSIZE)) != 0) {
                   if (first == 1) {
                       /* first add, so copy */
                       err = mp_copy(tG->x, R->x);
                       if (err != MP_OKAY) break;

                       err = mp_copy(tG->y, R->y);
                       if (err != MP_OKAY) break;

                       err = mp_copy(tG->z, R->z);
                       if (err != MP_OKAY) break;
                       first = 0;
                   } else {
                       /* then add */
                       err = ecc_projective_add_point(R, tG, R, modulus, &mp);
                       if (err != MP_OKAY) break;
                   }
               }
           }
       }
   }

   /* map R back from projective space */
   if (err == MP_OKAY && map)
       err = ecc_map(R, modulus, &mp);

   mp_clear(&mu);
   wc_ecc_del_point(tG);
   for (i = 0; i < 8; i++) {
       wc_ecc_del_point(M[i]);
   }
   return err;
}

#undef WINSIZE

#else /* ECC_TIMING_RESISTANT */

/**
   Perform a point multiplication  (timing resistant)
   k    The scalar to multiply by
   G    The base point
   R    [out] Destination for kG
   modulus  The modulus of the field the ECC curve is in
   map      Boolean whether to map back to affine or not
            (1==map, 0 == leave in projective)
   return MP_OKAY on success
*/
#ifdef FP_ECC
static int normal_ecc_mulmod(mp_int* k, ecc_point *G, ecc_point *R,
                      mp_int* modulus, int map)
#else
int wc_ecc_mulmod(mp_int* k, ecc_point *G, ecc_point *R, mp_int* modulus,
              int map)
#endif
{
   ecc_point    *tG, *M[3];
   int           i, j, err;
   mp_int        mu;
   mp_digit      mp;
   mp_digit      buf;
   int           bitcnt = 0, mode = 0, digidx = 0;

   if (k == NULL || G == NULL || R == NULL || modulus == NULL)
       return ECC_BAD_ARG_E;

   /* init montgomery reduction */
   if ((err = mp_montgomery_setup(modulus, &mp)) != MP_OKAY) {
      return err;
   }
   if ((err = mp_init(&mu)) != MP_OKAY) {
      return err;
   }
   if ((err = mp_montgomery_calc_normalization(&mu, modulus)) != MP_OKAY) {
      mp_clear(&mu);
      return err;
   }

  /* alloc ram for window temps */
  for (i = 0; i < 3; i++) {
      M[i] = wc_ecc_new_point();
      if (M[i] == NULL) {
         for (j = 0; j < i; j++) {
             wc_ecc_del_point(M[j]);
         }
         mp_clear(&mu);
         return MEMORY_E;
      }
  }

   /* make a copy of G in case R==G */
   tG = wc_ecc_new_point();
   if (tG == NULL)
       err = MEMORY_E;

   /* tG = G  and convert to montgomery */
   if (err == MP_OKAY) {
       err = mp_mulmod(G->x, &mu, modulus, tG->x);
       if (err == MP_OKAY)
           err = mp_mulmod(G->y, &mu, modulus, tG->y);
       if (err == MP_OKAY)
           err = mp_mulmod(G->z, &mu, modulus, tG->z);
   }
   mp_clear(&mu);

   /* calc the M tab */
   /* M[0] == G */
   if (err == MP_OKAY)
       err = mp_copy(tG->x, M[0]->x);
   if (err == MP_OKAY)
       err = mp_copy(tG->y, M[0]->y);
   if (err == MP_OKAY)
       err = mp_copy(tG->z, M[0]->z);

   /* M[1] == 2G */
   if (err == MP_OKAY)
       err = ecc_projective_dbl_point(tG, M[1], modulus, &mp);

   /* setup sliding window */
   mode   = 0;
   bitcnt = 1;
   buf    = 0;
   digidx = get_digit_count(k) - 1;

   /* perform ops */
   if (err == MP_OKAY) {
       for (;;) {
           /* grab next digit as required */
           if (--bitcnt == 0) {
               if (digidx == -1) {
                   break;
               }
               buf = get_digit(k, digidx);
               bitcnt = (int) DIGIT_BIT;
               --digidx;
           }

           /* grab the next msb from the ltiplicand */
           i = (buf >> (DIGIT_BIT - 1)) & 1;
           buf <<= 1;

           if (mode == 0 && i == 0) {
               /* dummy operations */
               if (err == MP_OKAY)
                   err = ecc_projective_add_point(M[0], M[1], M[2], modulus,
                                                  &mp);
               if (err == MP_OKAY)
                   err = ecc_projective_dbl_point(M[1], M[2], modulus, &mp);
               if (err == MP_OKAY)
                   continue;
           }

           if (mode == 0 && i == 1) {
               mode = 1;
               /* dummy operations */
               if (err == MP_OKAY)
                   err = ecc_projective_add_point(M[0], M[1], M[2], modulus,
                                                  &mp);
               if (err == MP_OKAY)
                   err = ecc_projective_dbl_point(M[1], M[2], modulus, &mp);
               if (err == MP_OKAY)
                   continue;
           }

           if (err == MP_OKAY)
               err = ecc_projective_add_point(M[0], M[1], M[i^1], modulus, &mp);
           if (err == MP_OKAY)
               err = ecc_projective_dbl_point(M[i], M[i], modulus, &mp);
           if (err != MP_OKAY)
               break;
       } /* end for */
   }

   /* copy result out */
   if (err == MP_OKAY)
       err = mp_copy(M[0]->x, R->x);
   if (err == MP_OKAY)
       err = mp_copy(M[0]->y, R->y);
   if (err == MP_OKAY)
       err = mp_copy(M[0]->z, R->z);

   /* map R back from projective space */
   if (err == MP_OKAY && map)
      err = ecc_map(R, modulus, &mp);

   /* done */
   mp_clear(&mu);
   wc_ecc_del_point(tG);
   for (i = 0; i < 3; i++) {
       wc_ecc_del_point(M[i]);
   }
   return err;
}

#endif /* ECC_TIMING_RESISTANT */


#ifdef ALT_ECC_SIZE

static void alt_fp_init(fp_int* a)
{
    a->size = FP_SIZE_ECC;
    fp_zero(a);
}

#endif /* ALT_ECC_SIZE */


/**
   Allocate a new ECC point
   return A newly allocated point or NULL on error
*/
ecc_point* wc_ecc_new_point(void)
{
   ecc_point* p;

   p = (ecc_point*)XMALLOC(sizeof(ecc_point), 0, DYNAMIC_TYPE_ECC);
   if (p == NULL) {
      return NULL;
   }
   XMEMSET(p, 0, sizeof(ecc_point));

#ifndef USE_FAST_MATH
   p->x->dp = NULL;
   p->y->dp = NULL;
   p->z->dp = NULL;
#endif

#ifndef ALT_ECC_SIZE
   if (mp_init_multi(p->x, p->y, p->z, NULL, NULL, NULL) != MP_OKAY) {
      XFREE(p, 0, DYNAMIC_TYPE_ECC);
      return NULL;
   }
#else
   p->x = (mp_int*)&p->xyz[0];
   p->y = (mp_int*)&p->xyz[1];
   p->z = (mp_int*)&p->xyz[2];
   alt_fp_init(p->x);
   alt_fp_init(p->y);
   alt_fp_init(p->z);
#endif

   return p;
}

/** Free an ECC point from memory
  p   The point to free
*/
void wc_ecc_del_point(ecc_point* p)
{
   /* prevents free'ing null arguments */
   if (p != NULL) {
      mp_clear(p->x);
      mp_clear(p->y);
      mp_clear(p->z);
      XFREE(p, 0, DYNAMIC_TYPE_ECC);
   }
}

/** Copy the value of a point to an other one
  p    The point to copy
  r    The created point
*/
int wc_ecc_copy_point(ecc_point* p, ecc_point *r)
{
    int ret;

    /* prevents null arguments */
    if (p == NULL || r == NULL)
        return ECC_BAD_ARG_E;

    ret = mp_copy(p->x, r->x);
    if (ret != MP_OKAY)
        return ret;
    ret = mp_copy(p->y, r->y);
    if (ret != MP_OKAY)
        return ret;
    ret = mp_copy(p->z, r->z);
    if (ret != MP_OKAY)
        return ret;

    return MP_OKAY;
}

/** Compare the value of a point with an other one
 a    The point to compare
 b    The other point to compare

 return MP_EQ if equal, MP_LT/MP_GT if not, < 0 in case of error
 */
int wc_ecc_cmp_point(ecc_point* a, ecc_point *b)
{
    int ret;

    /* prevents null arguments */
    if (a == NULL || b == NULL)
        return BAD_FUNC_ARG;

    ret = mp_cmp(a->x, b->x);
    if (ret != MP_EQ)
        return ret;
    ret = mp_cmp(a->y, b->y);
    if (ret != MP_EQ)
        return ret;
    ret = mp_cmp(a->z, b->z);
    if (ret != MP_EQ)
        return ret;

    return MP_EQ;
}

/** Returns whether an ECC idx is valid or not
  n      The idx number to check
  return 1 if valid, 0 if not
*/
int wc_ecc_is_valid_idx(int n)
{
   int x;

   for (x = 0; ecc_sets[x].size != 0; x++)
       ;
   /* -1 is a valid index --- indicating that the domain params
      were supplied by the user */
   if ((n >= -1) && (n < x)) {
      return 1;
   }
   return 0;
}

#ifdef HAVE_ECC_DHE
/**
  Create an ECC shared secret between two keys
  private_key      The private ECC key
  public_key       The public key
  out              [out] Destination of the shared secret
                         Conforms to EC-DH from ANSI X9.63
  outlen           [in/out] The max size and resulting size of the shared secret
  return           MP_OKAY if successful
*/
int wc_ecc_shared_secret(ecc_key* private_key, ecc_key* public_key, byte* out,
                      word32* outlen)
{
   word32         x = 0;
   ecc_point*     result;
   mp_int         prime;
   int            err;

   if (private_key == NULL || public_key == NULL || out == NULL ||
                                                    outlen == NULL)
       return BAD_FUNC_ARG;

   /* type valid? */
   if (private_key->type != ECC_PRIVATEKEY) {
      return ECC_BAD_ARG_E;
   }

   /* Verify domain params supplied */
   if (wc_ecc_is_valid_idx(private_key->idx) == 0 ||
       wc_ecc_is_valid_idx(public_key->idx)  == 0)
      return ECC_BAD_ARG_E;

   /* Verify curve name matches */
   if (XSTRNCMP(private_key->dp->name, public_key->dp->name, ECC_MAXNAME) != 0)
      return ECC_BAD_ARG_E;

   /* make new point */
   result = wc_ecc_new_point();
   if (result == NULL) {
      return MEMORY_E;
   }

   if ((err = mp_init(&prime)) != MP_OKAY) {
      wc_ecc_del_point(result);
      return err;
   }

   err = mp_read_radix(&prime, (char *)private_key->dp->prime, 16);

   if (err == MP_OKAY)
       err = wc_ecc_mulmod(&private_key->k, &public_key->pubkey, result, &prime,1);

   if (err == MP_OKAY) {
       x = mp_unsigned_bin_size(&prime);
       if (*outlen < x)
          err = BUFFER_E;
   }

   if (err == MP_OKAY) {
       XMEMSET(out, 0, x);
       err = mp_to_unsigned_bin(result->x,out + (x -
                                              mp_unsigned_bin_size(result->x)));
       *outlen = x;
   }

   mp_clear(&prime);
   wc_ecc_del_point(result);

   return err;
}

/**
 Create an ECC shared secret between private key and public point
 private_key      The private ECC key
 point            The point to use (public key)
 out              [out] Destination of the shared secret
                        Conforms to EC-DH from ANSI X9.63
 outlen           [in/out] The max size and resulting size of the shared secret
 return           MP_OKAY if successful
*/
int wc_ecc_shared_secret_ssh(ecc_key* private_key, ecc_point* point,
                             byte* out, word32 *outlen)
{
    word32         x = 0;
    ecc_point*     result;
    mp_int         prime;
    int            err;

    if (private_key == NULL || point == NULL || out == NULL || outlen == NULL)
        return BAD_FUNC_ARG;

    /* type valid? */
    if (private_key->type != ECC_PRIVATEKEY) {
        return ECC_BAD_ARG_E;
    }

    /* Verify domain params supplied */
    if (wc_ecc_is_valid_idx(private_key->idx) == 0)
        return ECC_BAD_ARG_E;

    /* make new point */
    result = wc_ecc_new_point();
    if (result == NULL) {
        return MEMORY_E;
    }

    if ((err = mp_init(&prime)) != MP_OKAY) {
        wc_ecc_del_point(result);
        return err;
    }

    err = mp_read_radix(&prime, (char *)private_key->dp->prime, 16);

    if (err == MP_OKAY)
        err = wc_ecc_mulmod(&private_key->k, point, result, &prime, 1);

    if (err == MP_OKAY) {
        x = mp_unsigned_bin_size(&prime);
        if (*outlen < x)
            err = BUFFER_E;
    }

    if (err == MP_OKAY) {
        XMEMSET(out, 0, x);
        err = mp_to_unsigned_bin(result->x,out +
                                 (x - mp_unsigned_bin_size(result->x)));
        *outlen = x;
    }

    mp_clear(&prime);
    wc_ecc_del_point(result);

    return err;
}
#endif /* HAVE_ECC_DHE */

/* return 1 if point is at infinity, 0 if not, < 0 on error */
int wc_ecc_point_is_at_infinity(ecc_point* p)
{
    if (p == NULL)
        return BAD_FUNC_ARG;

    if (get_digit_count(p->x) == 0 && get_digit_count(p->y) == 0)
        return 1;

    return 0;
}


static int wc_ecc_make_key_ex(WC_RNG* rng, ecc_key* key, const ecc_set_type* dp)
{
   int            err;
   ecc_point*     base;
   mp_int         prime;
   mp_int         order;
#ifdef WOLFSSL_SMALL_STACK
   byte*          buf;
#else
   byte           buf[ECC_MAXSIZE_GEN];
#endif
   int            keysize;
   int            po_init = 0;  /* prime order Init flag for clear */

   if (key == NULL || rng == NULL || dp == NULL)
       return ECC_BAD_ARG_E;

#ifdef WOLFSSL_SMALL_STACK
   buf = (byte*)XMALLOC(ECC_MAXSIZE_GEN, NULL, DYNAMIC_TYPE_TMP_BUFFER);
   if (buf == NULL)
       return MEMORY_E;
#endif

   key->idx = -1;
   key->dp  = dp;

   /*generate 8 extra bytes to mitigate bias from the modulo operation below*/
   /*see section A.1.2 in 'Suite B Implementor's Guide to FIPS 186-3 (ECDSA)'*/
   keysize  = dp->size + 8;

   /* allocate ram */
   base = NULL;

   /* make up random string */
   err = wc_RNG_GenerateBlock(rng, buf, keysize);

   /* setup the key variables */
   if (err == 0) {
#ifndef ALT_ECC_SIZE
       err = mp_init_multi(key->pubkey.x, key->pubkey.y, key->pubkey.z,
                            &key->k, &prime, &order);
#else
       key->pubkey.x = (mp_int*)&key->pubkey.xyz[0];
       key->pubkey.y = (mp_int*)&key->pubkey.xyz[1];
       key->pubkey.z = (mp_int*)&key->pubkey.xyz[2];
       alt_fp_init(key->pubkey.x);
       alt_fp_init(key->pubkey.y);
       alt_fp_init(key->pubkey.z);
       err = mp_init_multi(&key->k, &prime, &order, NULL, NULL, NULL);
#endif
       if (err != MP_OKAY)
           err = MEMORY_E;
       else
           po_init = 1;
   }

   if (err == MP_OKAY) {
       base = wc_ecc_new_point();
       if (base == NULL)
           err = MEMORY_E;
   }

   /* read in the specs for this key */
   if (err == MP_OKAY)
       err = mp_read_radix(&prime,   (char *)key->dp->prime, 16);
   if (err == MP_OKAY)
       err = mp_read_radix(&order,   (char *)key->dp->order, 16);
   if (err == MP_OKAY)
       err = mp_read_radix(base->x, (char *)key->dp->Gx, 16);
   if (err == MP_OKAY)
       err = mp_read_radix(base->y, (char *)key->dp->Gy, 16);

   if (err == MP_OKAY)
       mp_set(base->z, 1);
   if (err == MP_OKAY)
       err = mp_read_unsigned_bin(&key->k, (byte*)buf, keysize);

   /* quick sanity check to make sure we're not dealing with a 0 key */
   if (err == MP_OKAY) {
       if (MP_YES == mp_iszero(&key->k))
           err = MP_ZERO_E;
   }

   /* the key should be smaller than the order of base point */
   if (err == MP_OKAY) {
       if (mp_cmp(&key->k, &order) != MP_LT)
           err = mp_mod(&key->k, &order, &key->k);
   }
   /* make the public key */
   if (err == MP_OKAY)
       err = wc_ecc_mulmod(&key->k, base, &key->pubkey, &prime, 1);

#ifdef WOLFSSL_VALIDATE_ECC_KEYGEN
   /* validate the public key, order * pubkey = point at infinity */
   if (err == MP_OKAY)
       err = ecc_check_pubkey_order(key, &prime, &order);
#endif /* WOLFSSL_VALIDATE_KEYGEN */

   if (err == MP_OKAY)
       key->type = ECC_PRIVATEKEY;

   if (err != MP_OKAY) {
       /* clean up */
       mp_clear(key->pubkey.x);
       mp_clear(key->pubkey.y);
       mp_clear(key->pubkey.z);
       mp_clear(&key->k);
   }
   wc_ecc_del_point(base);
   if (po_init) {
       mp_clear(&prime);
       mp_clear(&order);
   }

   ForceZero(buf, ECC_MAXSIZE);
#ifdef WOLFSSL_SMALL_STACK
   XFREE(buf, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

   return err;
}

/**
 Make a new ECC key
 rng          An active RNG state
 keysize      The keysize for the new key (in octets from 20 to 65 bytes)
 key          [out] Destination of the newly created key
 return       MP_OKAY if successful,
 upon error all allocated memory will be freed
 */
int wc_ecc_make_key(WC_RNG* rng, int keysize, ecc_key* key)
{
    int x, err;

    if (key == NULL || rng == NULL)
        return ECC_BAD_ARG_E;

    /* find key size */
    for (x = 0; (keysize > ecc_sets[x].size) && (ecc_sets[x].size != 0); x++)
        ;
    keysize = ecc_sets[x].size;

    if (keysize > ECC_MAXSIZE || ecc_sets[x].size == 0) {
        return BAD_FUNC_ARG;
    }
    err = wc_ecc_make_key_ex(rng, key, &ecc_sets[x]);
    key->idx = x;

    return err;
}

/* Setup dynamic pointers is using normal math for proper freeing */
int wc_ecc_init(ecc_key* key)
{
    (void)key;

#ifndef USE_FAST_MATH
    key->pubkey.x->dp = NULL;
    key->pubkey.y->dp = NULL;
    key->pubkey.z->dp = NULL;

    key->k.dp = NULL;
#endif

#ifdef ALT_ECC_SIZE
    if (mp_init(&key->k) != MP_OKAY)
        return MEMORY_E;

    key->pubkey.x = (mp_int*)&key->pubkey.xyz[0];
    key->pubkey.y = (mp_int*)&key->pubkey.xyz[1];
    key->pubkey.z = (mp_int*)&key->pubkey.xyz[2];
    alt_fp_init(key->pubkey.x);
    alt_fp_init(key->pubkey.y);
    alt_fp_init(key->pubkey.z);
#endif

    return MP_OKAY;
}


#ifdef HAVE_ECC_SIGN

#ifndef NO_ASN
/**
 Sign a message digest
 in        The message digest to sign
 inlen     The length of the digest
 out       [out] The destination for the signature
 outlen    [in/out] The max size and resulting size of the signature
 key       A private ECC key
 return    MP_OKAY if successful
 */
int wc_ecc_sign_hash(const byte* in, word32 inlen, byte* out, word32 *outlen,
                     WC_RNG* rng, ecc_key* key)
{
    mp_int    r;
    mp_int    s;
    int        err;

    if (in == NULL || out == NULL || outlen == NULL ||
        key == NULL || rng == NULL)
        return ECC_BAD_ARG_E;

    if ((err = mp_init_multi(&r, &s, NULL, NULL, NULL, NULL)) != MP_OKAY) {
        return err;
    }

    err = wc_ecc_sign_hash_ex(in, inlen, rng, key, &r, &s);
    if (err == MP_OKAY)
        err = StoreECC_DSA_Sig(out, outlen, &r, &s);

    mp_clear(&r);
    mp_clear(&s);

    return err;
}
#endif /* !NO_ASN */

/**
  Sign a message digest
  in        The message digest to sign
  inlen     The length of the digest
  out       [out] The destination for the signature
  outlen    [in/out] The max size and resulting size of the signature
  key       A private ECC key
  r         [out] The destination for r component of the signature
  s            [out] The destination for s component of the signature
  return    MP_OKAY if successful
*/
int wc_ecc_sign_hash_ex(const byte* in, word32 inlen, WC_RNG* rng,
                     ecc_key* key, mp_int *r, mp_int *s)
{
   mp_int        e;
   mp_int        p;
   int           err;

   if (in == NULL || r == NULL || s == NULL || key == NULL || rng == NULL)
       return ECC_BAD_ARG_E;

   /* is this a private key? */
   if (key->type != ECC_PRIVATEKEY) {
      return ECC_BAD_ARG_E;
   }

   /* is the IDX valid ?  */
   if (wc_ecc_is_valid_idx(key->idx) != 1) {
      return ECC_BAD_ARG_E;
   }

   /* get the hash and load it as a bignum into 'e' */
   /* init the bignums */
   if ((err = mp_init_multi(&p, &e, NULL, NULL, NULL, NULL)) != MP_OKAY) {
      return err;
   }
   err = mp_read_radix(&p, (char *)key->dp->order, 16);

   if (err == MP_OKAY) {
       /* we may need to truncate if hash is longer than key size */
       word32 orderBits = mp_count_bits(&p);

       /* truncate down to byte size, may be all that's needed */
       if ( (WOLFSSL_BIT_SIZE * inlen) > orderBits)
           inlen = (orderBits + WOLFSSL_BIT_SIZE - 1)/WOLFSSL_BIT_SIZE;
       err = mp_read_unsigned_bin(&e, (byte*)in, inlen);

       /* may still need bit truncation too */
       if (err == MP_OKAY && (WOLFSSL_BIT_SIZE * inlen) > orderBits)
           mp_rshb(&e, WOLFSSL_BIT_SIZE - (orderBits & 0x7));
   }

   /* make up a key and export the public copy */
   if (err == MP_OKAY) {
       int loop_check = 0;
       ecc_key pubkey;
       if (wc_ecc_init(&pubkey) == MP_OKAY) {
           for (;;) {
               if (++loop_check > 64) {
                    err = RNG_FAILURE_E;
                    break;
               }
               err = wc_ecc_make_key_ex(rng, &pubkey, key->dp);
               if (err != MP_OKAY) break;

               /* find r = x1 mod n */
               err = mp_mod(pubkey.pubkey.x, &p, r);
               if (err != MP_OKAY) break;

               if (mp_iszero(r) == MP_YES) {
                   mp_clear(pubkey.pubkey.x);
                   mp_clear(pubkey.pubkey.y);
                   mp_clear(pubkey.pubkey.z);
                   mp_clear(&pubkey.k);
               }
               else {
                   /* find s = (e + xr)/k */
                   err = mp_invmod(&pubkey.k, &p, &pubkey.k);
                   if (err != MP_OKAY) break;

                   err = mp_mulmod(&key->k, r, &p, s);   /* s = xr */
                   if (err != MP_OKAY) break;

                   err = mp_add(&e, s, s);               /* s = e +  xr */
                   if (err != MP_OKAY) break;

                   err = mp_mod(s, &p, s);               /* s = e +  xr */
                   if (err != MP_OKAY) break;

                   err = mp_mulmod(s, &pubkey.k, &p, s); /* s = (e + xr)/k */
                   if (err != MP_OKAY) break;

                   if (mp_iszero(s) == MP_NO)
                       break;
                }
           }
           wc_ecc_free(&pubkey);
       }
   }

   mp_clear(&p);
   mp_clear(&e);

   return err;
}
#endif /* HAVE_ECC_SIGN */

/**
  Free an ECC key from memory
  key   The key you wish to free
*/
void wc_ecc_free(ecc_key* key)
{
   if (key == NULL)
       return;

   mp_clear(key->pubkey.x);
   mp_clear(key->pubkey.y);
   mp_clear(key->pubkey.z);
   mp_clear(&key->k);
}


#ifdef USE_FAST_MATH
    #define GEN_MEM_ERR FP_MEM
#else
    #define GEN_MEM_ERR MP_MEM
#endif

#ifdef ECC_SHAMIR

/** Computes kA*A + kB*B = C using Shamir's Trick
  A        First point to multiply
  kA       What to multiple A by
  B        Second point to multiply
  kB       What to multiple B by
  C        [out] Destination point (can overlap with A or B)
  modulus  Modulus for curve
  return MP_OKAY on success
*/
#ifdef FP_ECC
static int normal_ecc_mul2add(ecc_point* A, mp_int* kA,
                             ecc_point* B, mp_int* kB,
                             ecc_point* C, mp_int* modulus)
#else
static int ecc_mul2add(ecc_point* A, mp_int* kA,
                    ecc_point* B, mp_int* kB,
                    ecc_point* C, mp_int* modulus)
#endif
{
  ecc_point*     precomp[16];
  unsigned       bitbufA, bitbufB, lenA, lenB, len, x, y, nA, nB, nibble;
  unsigned char* tA;
  unsigned char* tB;
  int            err = MP_OKAY, first;
  int            muInit    = 0;
  int            tableInit = 0;
  mp_digit mp;
  mp_int   mu;

  /* argchks */
  if (A == NULL || kA == NULL || B == NULL || kB == NULL || C == NULL ||
                   modulus == NULL)
    return ECC_BAD_ARG_E;


  /* allocate memory */
  tA = (unsigned char*)XMALLOC(ECC_BUFSIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
  if (tA == NULL) {
     return GEN_MEM_ERR;
  }
  tB = (unsigned char*)XMALLOC(ECC_BUFSIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
  if (tB == NULL) {
     XFREE(tA, NULL, DYNAMIC_TYPE_TMP_BUFFER);
     return GEN_MEM_ERR;
  }
  XMEMSET(tA, 0, ECC_BUFSIZE);
  XMEMSET(tB, 0, ECC_BUFSIZE);

  /* get sizes */
  lenA = mp_unsigned_bin_size(kA);
  lenB = mp_unsigned_bin_size(kB);
  len  = MAX(lenA, lenB);

  /* sanity check */
  if ((lenA > ECC_BUFSIZE) || (lenB > ECC_BUFSIZE)) {
     err = BAD_FUNC_ARG;
  }

  if (err == MP_OKAY) {
    /* extract and justify kA */
    err = mp_to_unsigned_bin(kA, (len - lenA) + tA);

    /* extract and justify kB */
    if (err == MP_OKAY)
        err = mp_to_unsigned_bin(kB, (len - lenB) + tB);

    /* allocate the table */
    if (err == MP_OKAY) {
        for (x = 0; x < 16; x++) {
            precomp[x] = wc_ecc_new_point();
            if (precomp[x] == NULL) {
                for (y = 0; y < x; ++y) {
                    wc_ecc_del_point(precomp[y]);
                }
                err = GEN_MEM_ERR;
                break;
            }
        }
    }
  }

  if (err == MP_OKAY)
    tableInit = 1;

  if (err == MP_OKAY)
   /* init montgomery reduction */
   err = mp_montgomery_setup(modulus, &mp);

  if (err == MP_OKAY)
    err = mp_init(&mu);
  if (err == MP_OKAY)
    muInit = 1;

  if (err == MP_OKAY)
    err = mp_montgomery_calc_normalization(&mu, modulus);

  if (err == MP_OKAY)
    /* copy ones ... */
    err = mp_mulmod(A->x, &mu, modulus, precomp[1]->x);

  if (err == MP_OKAY)
    err = mp_mulmod(A->y, &mu, modulus, precomp[1]->y);
  if (err == MP_OKAY)
    err = mp_mulmod(A->z, &mu, modulus, precomp[1]->z);

  if (err == MP_OKAY)
    err = mp_mulmod(B->x, &mu, modulus, precomp[1<<2]->x);
  if (err == MP_OKAY)
    err = mp_mulmod(B->y, &mu, modulus, precomp[1<<2]->y);
  if (err == MP_OKAY)
    err = mp_mulmod(B->z, &mu, modulus, precomp[1<<2]->z);

  if (err == MP_OKAY)
    /* precomp [i,0](A + B) table */
    err = ecc_projective_dbl_point(precomp[1], precomp[2], modulus, &mp);

  if (err == MP_OKAY)
    err = ecc_projective_add_point(precomp[1], precomp[2], precomp[3],
                                   modulus, &mp);
  if (err == MP_OKAY)
    /* precomp [0,i](A + B) table */
    err = ecc_projective_dbl_point(precomp[1<<2], precomp[2<<2], modulus, &mp);

  if (err == MP_OKAY)
    err = ecc_projective_add_point(precomp[1<<2], precomp[2<<2], precomp[3<<2],
                                   modulus, &mp);

  if (err == MP_OKAY) {
    /* precomp [i,j](A + B) table (i != 0, j != 0) */
    for (x = 1; x < 4; x++) {
        for (y = 1; y < 4; y++) {
            if (err == MP_OKAY)
                err = ecc_projective_add_point(precomp[x], precomp[(y<<2)],
                                               precomp[x+(y<<2)], modulus, &mp);
        }
    }
  }

  if (err == MP_OKAY) {
    nibble  = 3;
    first   = 1;
    bitbufA = tA[0];
    bitbufB = tB[0];

    /* for every byte of the multiplicands */
    for (x = (unsigned)-1;; ) {
        /* grab a nibble */
        if (++nibble == 4) {
            ++x; if (x == len) break;
            bitbufA = tA[x];
            bitbufB = tB[x];
            nibble  = 0;
        }

        /* extract two bits from both, shift/update */
        nA = (bitbufA >> 6) & 0x03;
        nB = (bitbufB >> 6) & 0x03;
        bitbufA = (bitbufA << 2) & 0xFF;
        bitbufB = (bitbufB << 2) & 0xFF;

        /* if both zero, if first, continue */
        if ((nA == 0) && (nB == 0) && (first == 1)) {
            continue;
        }

        /* double twice, only if this isn't the first */
        if (first == 0) {
            /* double twice */
            if (err == MP_OKAY)
                err = ecc_projective_dbl_point(C, C, modulus, &mp);
            if (err == MP_OKAY)
                err = ecc_projective_dbl_point(C, C, modulus, &mp);
            else
                break;
        }

        /* if not both zero */
        if ((nA != 0) || (nB != 0)) {
            if (first == 1) {
                /* if first, copy from table */
                first = 0;
                if (err == MP_OKAY)
                    err = mp_copy(precomp[nA + (nB<<2)]->x, C->x);

                if (err == MP_OKAY)
                    err = mp_copy(precomp[nA + (nB<<2)]->y, C->y);

                if (err == MP_OKAY)
                    err = mp_copy(precomp[nA + (nB<<2)]->z, C->z);
                else
                    break;
            } else {
                /* if not first, add from table */
                if (err == MP_OKAY)
                    err = ecc_projective_add_point(C, precomp[nA + (nB<<2)], C,
                                                   modulus, &mp);
                else
                    break;
            }
        }
    }
  }

  if (err == MP_OKAY)
    /* reduce to affine */
    err = ecc_map(C, modulus, &mp);

  /* clean up */
  if (muInit)
    mp_clear(&mu);

  if (tableInit) {
    for (x = 0; x < 16; x++) {
       wc_ecc_del_point(precomp[x]);
    }
  }
   ForceZero(tA, ECC_BUFSIZE);
   ForceZero(tB, ECC_BUFSIZE);
   XFREE(tA, NULL, DYNAMIC_TYPE_TMP_BUFFER);
   XFREE(tB, NULL, DYNAMIC_TYPE_TMP_BUFFER);

   return err;
}


#endif /* ECC_SHAMIR */


#ifdef HAVE_ECC_VERIFY
#ifndef NO_ASN
/* verify
 *
 * w  = s^-1 mod n
 * u1 = xw
 * u2 = rw
 * X = u1*G + u2*Q
 * v = X_x1 mod n
 * accept if v == r
 */

/**
 Verify an ECC signature
 sig         The signature to verify
 siglen      The length of the signature (octets)
 hash        The hash (message digest) that was signed
 hashlen     The length of the hash (octets)
 stat        Result of signature, 1==valid, 0==invalid
 key         The corresponding public ECC key
 return      MP_OKAY if successful (even if the signature is not valid)
 */
int wc_ecc_verify_hash(const byte* sig, word32 siglen, const byte* hash,
                       word32 hashlen, int* stat, ecc_key* key)
{
    mp_int    r;
    mp_int    s;
    int        err;

    if (sig == NULL || hash == NULL || stat == NULL || key == NULL)
        return ECC_BAD_ARG_E;

    /* default to invalid signature */
    *stat = 0;

    /* Note, DecodeECC_DSA_Sig() calls mp_init() on r and s.
     * If either of those don't allocate correctly, none of
     * the rest of this function will execute, and everything
     * gets cleaned up at the end. */
    XMEMSET(&r, 0, sizeof(r));
    XMEMSET(&s, 0, sizeof(s));

    err = DecodeECC_DSA_Sig(sig, siglen, &r, &s);

    if (err == MP_OKAY)
        err = wc_ecc_verify_hash_ex(&r, &s, hash, hashlen, stat, key);

    mp_clear(&r);
    mp_clear(&s);

    return err;
}
#endif /* !NO_ASN */

/**
   Verify an ECC signature
   r           The signature R component to verify
   s           The signature S component to verify
   hash        The hash (message digest) that was signed
   hashlen     The length of the hash (octets)
   stat        Result of signature, 1==valid, 0==invalid
   key         The corresponding public ECC key
   return      MP_OKAY if successful (even if the signature is not valid)
*/
int wc_ecc_verify_hash_ex(mp_int *r, mp_int *s, const byte* hash,
                    word32 hashlen, int* stat, ecc_key* key)
{
   ecc_point    *mG, *mQ;
   mp_int        v;
   mp_int        w;
   mp_int        u1;
   mp_int        u2;
   mp_int        e;
   mp_int        p;
   mp_int        m;
   int           err;

   if (r == NULL || s == NULL || hash == NULL || stat == NULL || key == NULL)
       return ECC_BAD_ARG_E;

   /* default to invalid signature */
   *stat = 0;

   /* is the IDX valid ?  */
   if (wc_ecc_is_valid_idx(key->idx) != 1) {
      return ECC_BAD_ARG_E;
   }

   /* allocate ints */
   if ((err = mp_init_multi(&v, &w, &u1, &u2, &p, &e)) != MP_OKAY) {
      return MEMORY_E;
   }

   if ((err = mp_init(&m)) != MP_OKAY) {
      mp_clear(&v);
      mp_clear(&w);
      mp_clear(&u1);
      mp_clear(&u2);
      mp_clear(&p);
      mp_clear(&e);
      return MEMORY_E;
   }

   /* allocate points */
   mG = wc_ecc_new_point();
   mQ = wc_ecc_new_point();
   if (mQ  == NULL || mG == NULL)
      err = MEMORY_E;

   /* get the order */
   if (err == MP_OKAY)
       err = mp_read_radix(&p, (char *)key->dp->order, 16);

   /* get the modulus */
   if (err == MP_OKAY)
       err = mp_read_radix(&m, (char *)key->dp->prime, 16);

   /* check for zero */
   if (err == MP_OKAY) {
       if (mp_iszero(r) || mp_iszero(s) || mp_cmp(r, &p) != MP_LT ||
           mp_cmp(s, &p) != MP_LT)
           err = MP_ZERO_E;
   }
   /* read hash */
   if (err == MP_OKAY) {
       /* we may need to truncate if hash is longer than key size */
       unsigned int orderBits = mp_count_bits(&p);

       /* truncate down to byte size, may be all that's needed */
       if ( (WOLFSSL_BIT_SIZE * hashlen) > orderBits)
           hashlen = (orderBits + WOLFSSL_BIT_SIZE - 1)/WOLFSSL_BIT_SIZE;
       err = mp_read_unsigned_bin(&e, hash, hashlen);

       /* may still need bit truncation too */
       if (err == MP_OKAY && (WOLFSSL_BIT_SIZE * hashlen) > orderBits)
           mp_rshb(&e, WOLFSSL_BIT_SIZE - (orderBits & 0x7));
   }

   /*  w  = s^-1 mod n */
   if (err == MP_OKAY)
       err = mp_invmod(s, &p, &w);

   /* u1 = ew */
   if (err == MP_OKAY)
       err = mp_mulmod(&e, &w, &p, &u1);

   /* u2 = rw */
   if (err == MP_OKAY)
       err = mp_mulmod(r, &w, &p, &u2);

   /* find mG and mQ */
   if (err == MP_OKAY)
       err = mp_read_radix(mG->x, (char *)key->dp->Gx, 16);

   if (err == MP_OKAY)
       err = mp_read_radix(mG->y, (char *)key->dp->Gy, 16);
   if (err == MP_OKAY)
       mp_set(mG->z, 1);

   if (err == MP_OKAY)
       err = mp_copy(key->pubkey.x, mQ->x);
   if (err == MP_OKAY)
       err = mp_copy(key->pubkey.y, mQ->y);
   if (err == MP_OKAY)
       err = mp_copy(key->pubkey.z, mQ->z);

#ifndef ECC_SHAMIR
    {
       mp_digit      mp;

       /* compute u1*mG + u2*mQ = mG */
       if (err == MP_OKAY)
           err = wc_ecc_mulmod(&u1, mG, mG, &m, 0);
       if (err == MP_OKAY)
           err = wc_ecc_mulmod(&u2, mQ, mQ, &m, 0);

       /* find the montgomery mp */
       if (err == MP_OKAY)
           err = mp_montgomery_setup(&m, &mp);

       /* add them */
       if (err == MP_OKAY)
           err = ecc_projective_add_point(mQ, mG, mG, &m, &mp);

       /* reduce */
       if (err == MP_OKAY)
           err = ecc_map(mG, &m, &mp);
    }
#else
       /* use Shamir's trick to compute u1*mG + u2*mQ using half the doubles */
       if (err == MP_OKAY)
           err = ecc_mul2add(mG, &u1, mQ, &u2, mG, &m);
#endif /* ECC_SHAMIR */

   /* v = X_x1 mod n */
   if (err == MP_OKAY)
       err = mp_mod(mG->x, &p, &v);

   /* does v == r */
   if (err == MP_OKAY) {
       if (mp_cmp(&v, r) == MP_EQ)
           *stat = 1;
   }

   wc_ecc_del_point(mG);
   wc_ecc_del_point(mQ);

   mp_clear(&v);
   mp_clear(&w);
   mp_clear(&u1);
   mp_clear(&u2);
   mp_clear(&p);
   mp_clear(&e);
   mp_clear(&m);

   return err;
}
#endif /* HAVE_ECC_VERIFY */

#ifdef HAVE_ECC_KEY_IMPORT
/* import point from der */
int wc_ecc_import_point_der(byte* in, word32 inLen, const int curve_idx,
                            ecc_point* point)
{
    int err = 0;
    int compressed = 0;

    if (in == NULL || point == NULL || (curve_idx < 0) ||
        (wc_ecc_is_valid_idx(curve_idx) == 0))
        return ECC_BAD_ARG_E;

    /* must be odd */
    if ((inLen & 1) == 0) {
        return ECC_BAD_ARG_E;
    }

    /* init point */
#ifdef ALT_ECC_SIZE
    point->x = (mp_int*)&point->xyz[0];
    point->y = (mp_int*)&point->xyz[1];
    point->z = (mp_int*)&point->xyz[2];
    alt_fp_init(point->x);
    alt_fp_init(point->y);
    alt_fp_init(point->z);
#else
    err = mp_init_multi(point->x, point->y, point->z, NULL, NULL, NULL);
#endif
    if (err != MP_OKAY)
        return MEMORY_E;

    /* check for 4, 2, or 3 */
    if (in[0] != 0x04 && in[0] != 0x02 && in[0] != 0x03) {
        err = ASN_PARSE_E;
    }

    if (in[0] == 0x02 || in[0] == 0x03) {
#ifdef HAVE_COMP_KEY
        compressed = 1;
#else
        err = NOT_COMPILED_IN;
#endif
    }

    /* read data */
    if (err == MP_OKAY)
        err = mp_read_unsigned_bin(point->x, (byte*)in+1, (inLen-1)>>1);

#ifdef HAVE_COMP_KEY
    if (err == MP_OKAY && compressed == 1) {   /* build y */
        mp_int t1, t2, prime, a, b;

        if (mp_init_multi(&t1, &t2, &prime, &a, &b, NULL) != MP_OKAY)
            err = MEMORY_E;

        /* load prime */
        if (err == MP_OKAY)
            err = mp_read_radix(&prime, (char *)ecc_sets[curve_idx].prime, 16);

        /* load a */
        if (err == MP_OKAY)
            err = mp_read_radix(&a, (char *)ecc_sets[curve_idx].Af, 16);

        /* load b */
        if (err == MP_OKAY)
            err = mp_read_radix(&b, (char *)ecc_sets[curve_idx].Bf, 16);

        /* compute x^3 */
        if (err == MP_OKAY)
            err = mp_sqr(point->x, &t1);

        if (err == MP_OKAY)
            err = mp_mulmod(&t1, point->x, &prime, &t1);

        /* compute x^3 + a*x */
        if (err == MP_OKAY)
            err = mp_mulmod(&a, point->x, &prime, &t2);

        if (err == MP_OKAY)
            err = mp_add(&t1, &t2, &t1);

        /* compute x^3 + a*x + b */
        if (err == MP_OKAY)
            err = mp_add(&t1, &b, &t1);

        /* compute sqrt(x^3 + a*x + b) */
        if (err == MP_OKAY)
            err = mp_sqrtmod_prime(&t1, &prime, &t2);

        /* adjust y */
        if (err == MP_OKAY) {
            if ((mp_isodd(&t2) && in[0] == 0x03) ||
                (!mp_isodd(&t2) && in[0] == 0x02)) {
                err = mp_mod(&t2, &prime, point->y);
            }
            else {
                err = mp_submod(&prime, &t2, &prime, point->y);
            }
        }

        mp_clear(&a);
        mp_clear(&b);
        mp_clear(&prime);
        mp_clear(&t2);
        mp_clear(&t1);
    }
#endif

    if (err == MP_OKAY && compressed == 0)
        err = mp_read_unsigned_bin(point->y,
                                   (byte*)in+1+((inLen-1)>>1), (inLen-1)>>1);
    if (err == MP_OKAY)
        mp_set(point->z, 1);

    if (err != MP_OKAY) {
        mp_clear(point->x);
        mp_clear(point->y);
        mp_clear(point->z);
    }

    return err;
}
#endif /* HAVE_ECC_KEY_IMPORT */

#ifdef HAVE_ECC_KEY_EXPORT
/* export point to der */
int wc_ecc_export_point_der(const int curve_idx, ecc_point* point, byte* out,
                            word32* outLen)
{
#ifdef WOLFSSL_SMALL_STACK
    byte*  buf;
#else
    byte   buf[ECC_BUFSIZE];
#endif
    word32 numlen;
    int    ret = MP_OKAY;

    if ((curve_idx < 0) || (wc_ecc_is_valid_idx(curve_idx) == 0))
        return ECC_BAD_ARG_E;

    /* return length needed only */
    if (point != NULL && out == NULL && outLen != NULL) {
        numlen = ecc_sets[curve_idx].size;
        *outLen = 1 + 2*numlen;
        return LENGTH_ONLY_E;
    }

    if (point == NULL || out == NULL || outLen == NULL)
        return ECC_BAD_ARG_E;

    numlen = ecc_sets[curve_idx].size;

    if (*outLen < (1 + 2*numlen)) {
        *outLen = 1 + 2*numlen;
        return BUFFER_E;
    }

    /* store byte 0x04 */
    out[0] = 0x04;

#ifdef WOLFSSL_SMALL_STACK
    buf = (byte*)XMALLOC(ECC_BUFSIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (buf == NULL)
        return MEMORY_E;
#endif

    /* pad and store x */
    XMEMSET(buf, 0, ECC_BUFSIZE);
    ret = mp_to_unsigned_bin(point->x, buf +
                                 (numlen - mp_unsigned_bin_size(point->x)));
    if (ret != MP_OKAY)
        goto done;
    XMEMCPY(out+1, buf, numlen);

    /* pad and store y */
    XMEMSET(buf, 0, ECC_BUFSIZE);
    ret = mp_to_unsigned_bin(point->y, buf +
                                 (numlen - mp_unsigned_bin_size(point->y)));
    if (ret != MP_OKAY)
        goto done;
    XMEMCPY(out+1+numlen, buf, numlen);

    *outLen = 1 + 2*numlen;

done:
#ifdef WOLFSSL_SMALL_STACK
    XFREE(buf, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;
}


/* export public ECC key in ANSI X9.63 format */
int wc_ecc_export_x963(ecc_key* key, byte* out, word32* outLen)
{
#ifdef WOLFSSL_SMALL_STACK
   byte*  buf;
#else
   byte   buf[ECC_BUFSIZE];
#endif
   word32 numlen;
   int    ret = MP_OKAY;

   /* return length needed only */
   if (key != NULL && out == NULL && outLen != NULL) {
      numlen = key->dp->size;
      *outLen = 1 + 2*numlen;
      return LENGTH_ONLY_E;
   }

   if (key == NULL || out == NULL || outLen == NULL)
      return ECC_BAD_ARG_E;

   if (wc_ecc_is_valid_idx(key->idx) == 0) {
      return ECC_BAD_ARG_E;
   }
   numlen = key->dp->size;

   if (*outLen < (1 + 2*numlen)) {
      *outLen = 1 + 2*numlen;
      return BUFFER_E;
   }

   /* store byte 0x04 */
   out[0] = 0x04;

#ifdef WOLFSSL_SMALL_STACK
   buf = (byte*)XMALLOC(ECC_BUFSIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
   if (buf == NULL)
      return MEMORY_E;
#endif

    /* pad and store x */
    XMEMSET(buf, 0, ECC_BUFSIZE);
    ret = mp_to_unsigned_bin(key->pubkey.x,
                         buf + (numlen - mp_unsigned_bin_size(key->pubkey.x)));
    if (ret != MP_OKAY)
        goto done;
    XMEMCPY(out+1, buf, numlen);

    /* pad and store y */
    XMEMSET(buf, 0, ECC_BUFSIZE);
    ret = mp_to_unsigned_bin(key->pubkey.y,
                         buf + (numlen - mp_unsigned_bin_size(key->pubkey.y)));
    if (ret != MP_OKAY)
        goto done;
    XMEMCPY(out+1+numlen, buf, numlen);

    *outLen = 1 + 2*numlen;

done:
#ifdef WOLFSSL_SMALL_STACK
   XFREE(buf, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

   return ret;
}


/* export public ECC key in ANSI X9.63 format, extended with
 * compression option */
int wc_ecc_export_x963_ex(ecc_key* key, byte* out, word32* outLen,
                          int compressed)
{
    if (compressed == 0)
        return wc_ecc_export_x963(key, out, outLen);
#ifdef HAVE_COMP_KEY
    else
        return wc_ecc_export_x963_compressed(key, out, outLen);
#endif

    return NOT_COMPILED_IN;
}
#endif /* HAVE_ECC_KEY_EXPORT */

/* is ec point on curve described by dp ? */
static int ecc_is_point(const ecc_set_type* dp, ecc_point* ecp, mp_int* prime)
{
   mp_int b, t1, t2;
   int err;

   if ((err = mp_init_multi(&b, &t1, &t2, NULL, NULL, NULL)) != MP_OKAY) {
      return err;
   }

   /* load  b */
   err = mp_read_radix(&b, dp->Bf, 16);

   /* compute y^2 */
   if (err == MP_OKAY)
       err = mp_sqr(ecp->y, &t1);

   /* compute x^3 */
   if (err == MP_OKAY)
       err = mp_sqr(ecp->x, &t2);
   if (err == MP_OKAY)
       err = mp_mod(&t2, prime, &t2);
   if (err == MP_OKAY)
       err = mp_mul(ecp->x, &t2, &t2);

   /* compute y^2 - x^3 */
   if (err == MP_OKAY)
       err = mp_sub(&t1, &t2, &t1);

   /* compute y^2 - x^3 + 3x */
   if (err == MP_OKAY)
       err = mp_add(&t1, ecp->x, &t1);
   if (err == MP_OKAY)
       err = mp_add(&t1, ecp->x, &t1);
   if (err == MP_OKAY)
       err = mp_add(&t1, ecp->x, &t1);
   if (err == MP_OKAY)
       err = mp_mod(&t1, prime, &t1);

   while (err == MP_OKAY && mp_cmp_d(&t1, 0) == MP_LT) {
      err = mp_add(&t1, prime, &t1);
   }
   while (err == MP_OKAY && mp_cmp(&t1, prime) != MP_LT) {
      err = mp_sub(&t1, prime, &t1);
   }

   /* compare to b */
   if (err == MP_OKAY) {
       if (mp_cmp(&t1, &b) != MP_EQ) {
          err = MP_VAL;
       } else {
          err = MP_OKAY;
       }
   }

   mp_clear(&b);
   mp_clear(&t1);
   mp_clear(&t2);

   return err;
}


/* validate privkey * generator == pubkey, 0 on success */
static int ecc_check_privkey_gen(ecc_key* key, mp_int* prime)
{
    ecc_point* base = NULL;
    ecc_point* res  = NULL;
    int        err;

    if (key == NULL)
        return BAD_FUNC_ARG;

    base = wc_ecc_new_point();
    if (base == NULL)
        return MEMORY_E;

    /* set up base generator */
    err = mp_read_radix(base->x, (char*)key->dp->Gx, 16);
    if (err == MP_OKAY)
        err = mp_read_radix(base->y, (char*)key->dp->Gy, 16);
    if (err == MP_OKAY)
        mp_set(base->z, 1);

    if (err == MP_OKAY) {
        res = wc_ecc_new_point();
        if (res == NULL)
            err = MEMORY_E;
        else {
            err = wc_ecc_mulmod(&key->k, base, res, prime, 1);
            if (err == MP_OKAY) {
                /* compare result to public key */
                if (mp_cmp(res->x, key->pubkey.x) != MP_EQ ||
                    mp_cmp(res->y, key->pubkey.y) != MP_EQ ||
                    mp_cmp(res->z, key->pubkey.z) != MP_EQ) {
                    /* didn't match */
                    err = ECC_PRIV_KEY_E;
                }
            }
        }
    }

    wc_ecc_del_point(res);
    wc_ecc_del_point(base);

    return err;
}


#ifdef WOLFSSL_VALIDATE_ECC_IMPORT

/* check privkey generator helper, creates prime needed */
static int ecc_check_privkey_gen_helper(ecc_key* key)
{
    mp_int prime;
    int    err;

    if (key == NULL)
        return BAD_FUNC_ARG;

    err = mp_init(&prime);
    if (err != MP_OKAY)
        return err;

    err = mp_read_radix(&prime, (char*)key->dp->prime, 16);

    if (err == MP_OKAY);
        err = ecc_check_privkey_gen(key, &prime);

    mp_clear(&prime);

    return err;
}

#endif /* WOLFSSL_VALIDATE_ECC_IMPORT */


/* validate order * pubkey = point at infinity, 0 on success */
static int ecc_check_pubkey_order(ecc_key* key, mp_int* prime, mp_int* order)
{
    ecc_point* inf = NULL;
    int        err;

    if (key == NULL)
        return BAD_FUNC_ARG;

    inf = wc_ecc_new_point();
    if (inf == NULL)
        err = MEMORY_E;
    else {
        err = wc_ecc_mulmod(order, &key->pubkey, inf, prime, 1);
        if (err == MP_OKAY && !wc_ecc_point_is_at_infinity(inf))
            err = ECC_INF_E;
    }

    wc_ecc_del_point(inf);

    return err;
}


/* perform sanity checks on ec key validity, 0 on success */
int wc_ecc_check_key(ecc_key* key)
{
    mp_int prime;  /* used by multiple calls so let's cache */
    mp_int order;  /* other callers have, so let's gen here */
    int    err;

    if (key == NULL)
        return BAD_FUNC_ARG;

    /* pubkey point cannot be at infinity */
    if (wc_ecc_point_is_at_infinity(&key->pubkey))
        return ECC_INF_E;

    err = mp_init_multi(&prime, &order, NULL, NULL, NULL, NULL);
    if (err != MP_OKAY)
        return err;

    err = mp_read_radix(&prime, (char*)key->dp->prime, 16);

    /* make sure point is actually on curve */
    if (err == MP_OKAY)
        err = ecc_is_point(key->dp, &key->pubkey, &prime);

    if (err == MP_OKAY)
        err = mp_read_radix(&order, (char*)key->dp->order, 16);

    /* pubkey * order must be at infinity */
    if (err == MP_OKAY)
        err = ecc_check_pubkey_order(key, &prime, &order);

    /* private * base generator must equal pubkey */
    if (err == MP_OKAY && key->type == ECC_PRIVATEKEY)
        err = ecc_check_privkey_gen(key, &prime);

    mp_clear(&order);
    mp_clear(&prime);

    return err;
}

#ifdef HAVE_ECC_KEY_IMPORT
/* import public ECC key in ANSI X9.63 format */
int wc_ecc_import_x963(const byte* in, word32 inLen, ecc_key* key)
{
   int x, err;
   int compressed = 0;

   if (in == NULL || key == NULL)
       return ECC_BAD_ARG_E;

   /* must be odd */
   if ((inLen & 1) == 0) {
      return ECC_BAD_ARG_E;
   }

   /* init key */
#ifdef ALT_ECC_SIZE
   key->pubkey.x = (mp_int*)&key->pubkey.xyz[0];
   key->pubkey.y = (mp_int*)&key->pubkey.xyz[1];
   key->pubkey.z = (mp_int*)&key->pubkey.xyz[2];
   alt_fp_init(key->pubkey.x);
   alt_fp_init(key->pubkey.y);
   alt_fp_init(key->pubkey.z);
   err = mp_init(&key->k);
#else
   err = mp_init_multi(key->pubkey.x, key->pubkey.y, key->pubkey.z, &key->k,
                     NULL, NULL);
#endif
   if (err != MP_OKAY)
      return MEMORY_E;

   /* check for 4, 2, or 3 */
   if (in[0] != 0x04 && in[0] != 0x02 && in[0] != 0x03) {
      err = ASN_PARSE_E;
   }

   if (in[0] == 0x02 || in[0] == 0x03) {
#ifdef HAVE_COMP_KEY
       compressed = 1;
#else
       err = NOT_COMPILED_IN;
#endif
   }

   if (err == MP_OKAY) {
      /* determine the idx */

      if (compressed)
          inLen = (inLen-1)*2 + 1;  /* used uncompressed len */

      for (x = 0; ecc_sets[x].size != 0; x++) {
         if ((unsigned)ecc_sets[x].size >= ((inLen-1)>>1)) {
            break;
         }
      }
      if (ecc_sets[x].size == 0) {
          WOLFSSL_MSG("ecc_set size not found");
          err = ASN_PARSE_E;
      } else {
          /* set the idx */
          key->idx  = x;
          key->dp = &ecc_sets[x];
          key->type = ECC_PUBLICKEY;
      }
   }

   /* read data */
   if (err == MP_OKAY)
       err = mp_read_unsigned_bin(key->pubkey.x, (byte*)in+1, (inLen-1)>>1);

#ifdef HAVE_COMP_KEY
   if (err == MP_OKAY && compressed == 1) {   /* build y */
        mp_int t1, t2, prime, a, b;

        if (mp_init_multi(&t1, &t2, &prime, &a, &b, NULL) != MP_OKAY)
            err = MEMORY_E;

        /* load prime */
        if (err == MP_OKAY)
            err = mp_read_radix(&prime, (char *)key->dp->prime, 16);

        /* load a */
        if (err == MP_OKAY)
            err = mp_read_radix(&a, (char *)key->dp->Af, 16);

        /* load b */
        if (err == MP_OKAY)
            err = mp_read_radix(&b, (char *)key->dp->Bf, 16);

        /* compute x^3 */
        if (err == MP_OKAY)
            err = mp_sqr(key->pubkey.x, &t1);

        if (err == MP_OKAY)
            err = mp_mulmod(&t1, key->pubkey.x, &prime, &t1);

        /* compute x^3 + a*x */
        if (err == MP_OKAY)
            err = mp_mulmod(&a, key->pubkey.x, &prime, &t2);

        if (err == MP_OKAY)
            err = mp_add(&t1, &t2, &t1);

        /* compute x^3 + a*x + b */
        if (err == MP_OKAY)
            err = mp_add(&t1, &b, &t1);

        /* compute sqrt(x^3 + a*x + b) */
        if (err == MP_OKAY)
            err = mp_sqrtmod_prime(&t1, &prime, &t2);

        /* adjust y */
        if (err == MP_OKAY) {
            if ((mp_isodd(&t2) && in[0] == 0x03) ||
               (!mp_isodd(&t2) && in[0] == 0x02)) {
                err = mp_mod(&t2, &prime, key->pubkey.y);
            }
            else {
                err = mp_submod(&prime, &t2, &prime, key->pubkey.y);
            }
        }

        mp_clear(&a);
        mp_clear(&b);
        mp_clear(&prime);
        mp_clear(&t2);
        mp_clear(&t1);
   }
#endif

   if (err == MP_OKAY && compressed == 0)
       err = mp_read_unsigned_bin(key->pubkey.y, (byte*)in+1+((inLen-1)>>1),
                                  (inLen-1)>>1);
   if (err == MP_OKAY)
       mp_set(key->pubkey.z, 1);

#ifdef WOLFSSL_VALIDATE_ECC_IMPORT
   if (err == MP_OKAY)
       err = wc_ecc_check_key(key);
#endif

   if (err != MP_OKAY) {
       mp_clear(key->pubkey.x);
       mp_clear(key->pubkey.y);
       mp_clear(key->pubkey.z);
       mp_clear(&key->k);
   }

   return err;
}
#endif /* HAVE_ECC_KEY_IMPORT */

#ifdef HAVE_ECC_KEY_EXPORT
/* export ecc private key only raw, outLen is in/out size
   return MP_OKAY on success */
int wc_ecc_export_private_only(ecc_key* key, byte* out, word32* outLen)
{
   word32 numlen;

   if (key == NULL || out == NULL || outLen == NULL)
       return ECC_BAD_ARG_E;

   if (wc_ecc_is_valid_idx(key->idx) == 0) {
      return ECC_BAD_ARG_E;
   }
   numlen = key->dp->size;

   if (*outLen < numlen) {
      *outLen = numlen;
      return BUFFER_E;
   }
   *outLen = numlen;
   XMEMSET(out, 0, *outLen);
   return mp_to_unsigned_bin(&key->k, out + (numlen -
                                             mp_unsigned_bin_size(&key->k)));
}
#endif /* HAVE_ECC_KEY_EXPORT */

#ifdef HAVE_ECC_KEY_IMPORT
/* ecc private key import, public key in ANSI X9.63 format, private raw */
int wc_ecc_import_private_key(const byte* priv, word32 privSz, const byte* pub,
                           word32 pubSz, ecc_key* key)
{
    int ret = wc_ecc_import_x963(pub, pubSz, key);
    if (ret != 0)
        return ret;

    key->type = ECC_PRIVATEKEY;

    ret = mp_read_unsigned_bin(&key->k, priv, privSz);

#ifdef WOLFSSL_VALIDATE_ECC_IMPORT
    if (ret == MP_OKAY)
        ret = ecc_check_privkey_gen_helper(key);
#endif

    return ret;
}
#endif /* HAVE_ECC_KEY_IMPORT */

#ifndef NO_ASN
/**
   Convert ECC R,S to signature
   r       R component of signature
   s       S component of signature
   out     DER-encoded ECDSA signature
   outlen  [in/out] output buffer size, output signature size
   return  MP_OKAY on success
*/
int wc_ecc_rs_to_sig(const char* r, const char* s, byte* out, word32* outlen)
{
    int err;
    mp_int rtmp;
    mp_int stmp;

    if (r == NULL || s == NULL || out == NULL || outlen == NULL)
        return ECC_BAD_ARG_E;

    err = mp_init_multi(&rtmp, &stmp, NULL, NULL, NULL, NULL);
    if (err != MP_OKAY)
        return err;

    err = mp_read_radix(&rtmp, r, 16);
    if (err == MP_OKAY)
        err = mp_read_radix(&stmp, s, 16);

    /* convert mp_ints to ECDSA sig, initializes rtmp and stmp internally */
    if (err == MP_OKAY)
        err = StoreECC_DSA_Sig(out, outlen, &rtmp, &stmp);

    if (err == MP_OKAY) {
        if (mp_iszero(&rtmp) || mp_iszero(&stmp))
            err = MP_ZERO_E;
    }

    mp_clear(&rtmp);
    mp_clear(&stmp);

    return err;
}
#endif /* !NO_ASN */

#ifdef HAVE_ECC_KEY_IMPORT
/**
   Import raw ECC key
   key       The destination ecc_key structure
   qx        x component of base point, as ASCII hex string
   qy        y component of base point, as ASCII hex string
   d         private key, as ASCII hex string
   curveName ECC curve name, from ecc_sets[]
   return    MP_OKAY on success
*/
int wc_ecc_import_raw(ecc_key* key, const char* qx, const char* qy,
                   const char* d, const char* curveName)
{
    int err, x;

    if (key == NULL || qx == NULL || qy == NULL || d == NULL ||
        curveName == NULL)
        return ECC_BAD_ARG_E;

    /* init key */
#ifdef ALT_ECC_SIZE
    key->pubkey.x = (mp_int*)&key->pubkey.xyz[0];
    key->pubkey.y = (mp_int*)&key->pubkey.xyz[1];
    key->pubkey.z = (mp_int*)&key->pubkey.xyz[2];
    alt_fp_init(key->pubkey.x);
    alt_fp_init(key->pubkey.y);
    alt_fp_init(key->pubkey.z);
    err = mp_init(&key->k);
#else
    err = mp_init_multi(key->pubkey.x, key->pubkey.y, key->pubkey.z, &key->k,
                      NULL, NULL);
#endif
    if (err != MP_OKAY)
        return MEMORY_E;

    /* read Qx */
    if (err == MP_OKAY)
        err = mp_read_radix(key->pubkey.x, qx, 16);

    /* read Qy */
    if (err == MP_OKAY)
        err = mp_read_radix(key->pubkey.y, qy, 16);

    if (err == MP_OKAY)
        mp_set(key->pubkey.z, 1);

    /* read and set the curve */
    if (err == MP_OKAY) {
        for (x = 0; ecc_sets[x].size != 0; x++) {
            if (XSTRNCMP(ecc_sets[x].name, curveName,
                         XSTRLEN(curveName)) == 0) {
                break;
            }
        }
        if (ecc_sets[x].size == 0) {
            WOLFSSL_MSG("ecc_set curve name not found");
            err = ASN_PARSE_E;
        } else {
            /* set the curve */
            key->idx = x;
            key->dp = &ecc_sets[x];
            key->type = ECC_PUBLICKEY;
        }
    }

    /* import private key */
    if (err == MP_OKAY) {
        key->type = ECC_PRIVATEKEY;
        err = mp_read_radix(&key->k, d, 16);
    }

#ifdef WOLFSSL_VALIDATE_ECC_IMPORT
    if (err == MP_OKAY)
        err = wc_ecc_check_key(key);
#endif

    if (err != MP_OKAY) {
        mp_clear(key->pubkey.x);
        mp_clear(key->pubkey.y);
        mp_clear(key->pubkey.z);
        mp_clear(&key->k);
    }

    return err;
}
#endif /* HAVE_ECC_KEY_IMPORT */

/* key size in octets */
int wc_ecc_size(ecc_key* key)
{
    if (key == NULL) return 0;

    return key->dp->size;
}


/* worst case estimate, check actual return from wc_ecc_sign_hash for actual
   value of signature size in octets */
int wc_ecc_sig_size(ecc_key* key)
{
    int sz = wc_ecc_size(key);
    if (sz <= 0)
        return sz;

    return (sz * 2) + SIG_HEADER_SZ + ECC_MAX_PAD_SZ;
}


#ifdef FP_ECC

/* fixed point ECC cache */
/* number of entries in the cache */
#ifndef FP_ENTRIES
    #define FP_ENTRIES 16
#endif

/* number of bits in LUT */
#ifndef FP_LUT
    #define FP_LUT     8U
#endif

#ifdef ECC_SHAMIR
    /* Sharmir requires a bigger LUT, TAO */
    #if (FP_LUT > 12) || (FP_LUT < 4)
        #error FP_LUT must be between 4 and 12 inclusively
    #endif
#else
    #if (FP_LUT > 12) || (FP_LUT < 2)
        #error FP_LUT must be between 2 and 12 inclusively
    #endif
#endif


/** Our FP cache */
typedef struct {
   ecc_point* g;               /* cached COPY of base point */
   ecc_point* LUT[1U<<FP_LUT]; /* fixed point lookup */
   mp_int     mu;              /* copy of the montgomery constant */
   int        lru_count;       /* amount of times this entry has been used */
   int        lock;            /* flag to indicate cache eviction */
                               /* permitted (0) or not (1) */
} fp_cache_t;

/* if HAVE_THREAD_LS this cache is per thread, no locking needed */
static THREAD_LS_T fp_cache_t fp_cache[FP_ENTRIES];

#ifndef HAVE_THREAD_LS
    static volatile int initMutex = 0;  /* prevent multiple mutex inits */
    static wolfSSL_Mutex ecc_fp_lock;
#endif /* HAVE_THREAD_LS */

/* simple table to help direct the generation of the LUT */
static const struct {
   int ham, terma, termb;
} lut_orders[] = {
   { 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 }, { 2, 1, 2 }, { 1, 0, 0 }, { 2, 1, 4 }, { 2, 2, 4 }, { 3, 3, 4 },
   { 1, 0, 0 }, { 2, 1, 8 }, { 2, 2, 8 }, { 3, 3, 8 }, { 2, 4, 8 }, { 3, 5, 8 }, { 3, 6, 8 }, { 4, 7, 8 },
   { 1, 0, 0 }, { 2, 1, 16 }, { 2, 2, 16 }, { 3, 3, 16 }, { 2, 4, 16 }, { 3, 5, 16 }, { 3, 6, 16 }, { 4, 7, 16 },
   { 2, 8, 16 }, { 3, 9, 16 }, { 3, 10, 16 }, { 4, 11, 16 }, { 3, 12, 16 }, { 4, 13, 16 }, { 4, 14, 16 }, { 5, 15, 16 },
   { 1, 0, 0 }, { 2, 1, 32 }, { 2, 2, 32 }, { 3, 3, 32 }, { 2, 4, 32 }, { 3, 5, 32 }, { 3, 6, 32 }, { 4, 7, 32 },
   { 2, 8, 32 }, { 3, 9, 32 }, { 3, 10, 32 }, { 4, 11, 32 }, { 3, 12, 32 }, { 4, 13, 32 }, { 4, 14, 32 }, { 5, 15, 32 },
   { 2, 16, 32 }, { 3, 17, 32 }, { 3, 18, 32 }, { 4, 19, 32 }, { 3, 20, 32 }, { 4, 21, 32 }, { 4, 22, 32 }, { 5, 23, 32 },
   { 3, 24, 32 }, { 4, 25, 32 }, { 4, 26, 32 }, { 5, 27, 32 }, { 4, 28, 32 }, { 5, 29, 32 }, { 5, 30, 32 }, { 6, 31, 32 },
#if FP_LUT > 6
   { 1, 0, 0 }, { 2, 1, 64 }, { 2, 2, 64 }, { 3, 3, 64 }, { 2, 4, 64 }, { 3, 5, 64 }, { 3, 6, 64 }, { 4, 7, 64 },
   { 2, 8, 64 }, { 3, 9, 64 }, { 3, 10, 64 }, { 4, 11, 64 }, { 3, 12, 64 }, { 4, 13, 64 }, { 4, 14, 64 }, { 5, 15, 64 },
   { 2, 16, 64 }, { 3, 17, 64 }, { 3, 18, 64 }, { 4, 19, 64 }, { 3, 20, 64 }, { 4, 21, 64 }, { 4, 22, 64 }, { 5, 23, 64 },
   { 3, 24, 64 }, { 4, 25, 64 }, { 4, 26, 64 }, { 5, 27, 64 }, { 4, 28, 64 }, { 5, 29, 64 }, { 5, 30, 64 }, { 6, 31, 64 },
   { 2, 32, 64 }, { 3, 33, 64 }, { 3, 34, 64 }, { 4, 35, 64 }, { 3, 36, 64 }, { 4, 37, 64 }, { 4, 38, 64 }, { 5, 39, 64 },
   { 3, 40, 64 }, { 4, 41, 64 }, { 4, 42, 64 }, { 5, 43, 64 }, { 4, 44, 64 }, { 5, 45, 64 }, { 5, 46, 64 }, { 6, 47, 64 },
   { 3, 48, 64 }, { 4, 49, 64 }, { 4, 50, 64 }, { 5, 51, 64 }, { 4, 52, 64 }, { 5, 53, 64 }, { 5, 54, 64 }, { 6, 55, 64 },
   { 4, 56, 64 }, { 5, 57, 64 }, { 5, 58, 64 }, { 6, 59, 64 }, { 5, 60, 64 }, { 6, 61, 64 }, { 6, 62, 64 }, { 7, 63, 64 },
#if FP_LUT > 7
   { 1, 0, 0 }, { 2, 1, 128 }, { 2, 2, 128 }, { 3, 3, 128 }, { 2, 4, 128 }, { 3, 5, 128 }, { 3, 6, 128 }, { 4, 7, 128 },
   { 2, 8, 128 }, { 3, 9, 128 }, { 3, 10, 128 }, { 4, 11, 128 }, { 3, 12, 128 }, { 4, 13, 128 }, { 4, 14, 128 }, { 5, 15, 128 },
   { 2, 16, 128 }, { 3, 17, 128 }, { 3, 18, 128 }, { 4, 19, 128 }, { 3, 20, 128 }, { 4, 21, 128 }, { 4, 22, 128 }, { 5, 23, 128 },
   { 3, 24, 128 }, { 4, 25, 128 }, { 4, 26, 128 }, { 5, 27, 128 }, { 4, 28, 128 }, { 5, 29, 128 }, { 5, 30, 128 }, { 6, 31, 128 },
   { 2, 32, 128 }, { 3, 33, 128 }, { 3, 34, 128 }, { 4, 35, 128 }, { 3, 36, 128 }, { 4, 37, 128 }, { 4, 38, 128 }, { 5, 39, 128 },
   { 3, 40, 128 }, { 4, 41, 128 }, { 4, 42, 128 }, { 5, 43, 128 }, { 4, 44, 128 }, { 5, 45, 128 }, { 5, 46, 128 }, { 6, 47, 128 },
   { 3, 48, 128 }, { 4, 49, 128 }, { 4, 50, 128 }, { 5, 51, 128 }, { 4, 52, 128 }, { 5, 53, 128 }, { 5, 54, 128 }, { 6, 55, 128 },
   { 4, 56, 128 }, { 5, 57, 128 }, { 5, 58, 128 }, { 6, 59, 128 }, { 5, 60, 128 }, { 6, 61, 128 }, { 6, 62, 128 }, { 7, 63, 128 },
   { 2, 64, 128 }, { 3, 65, 128 }, { 3, 66, 128 }, { 4, 67, 128 }, { 3, 68, 128 }, { 4, 69, 128 }, { 4, 70, 128 }, { 5, 71, 128 },
   { 3, 72, 128 }, { 4, 73, 128 }, { 4, 74, 128 }, { 5, 75, 128 }, { 4, 76, 128 }, { 5, 77, 128 }, { 5, 78, 128 }, { 6, 79, 128 },
   { 3, 80, 128 }, { 4, 81, 128 }, { 4, 82, 128 }, { 5, 83, 128 }, { 4, 84, 128 }, { 5, 85, 128 }, { 5, 86, 128 }, { 6, 87, 128 },
   { 4, 88, 128 }, { 5, 89, 128 }, { 5, 90, 128 }, { 6, 91, 128 }, { 5, 92, 128 }, { 6, 93, 128 }, { 6, 94, 128 }, { 7, 95, 128 },
   { 3, 96, 128 }, { 4, 97, 128 }, { 4, 98, 128 }, { 5, 99, 128 }, { 4, 100, 128 }, { 5, 101, 128 }, { 5, 102, 128 }, { 6, 103, 128 },
   { 4, 104, 128 }, { 5, 105, 128 }, { 5, 106, 128 }, { 6, 107, 128 }, { 5, 108, 128 }, { 6, 109, 128 }, { 6, 110, 128 }, { 7, 111, 128 },
   { 4, 112, 128 }, { 5, 113, 128 }, { 5, 114, 128 }, { 6, 115, 128 }, { 5, 116, 128 }, { 6, 117, 128 }, { 6, 118, 128 }, { 7, 119, 128 },
   { 5, 120, 128 }, { 6, 121, 128 }, { 6, 122, 128 }, { 7, 123, 128 }, { 6, 124, 128 }, { 7, 125, 128 }, { 7, 126, 128 }, { 8, 127, 128 },
#if FP_LUT > 8
   { 1, 0, 0 }, { 2, 1, 256 }, { 2, 2, 256 }, { 3, 3, 256 }, { 2, 4, 256 }, { 3, 5, 256 }, { 3, 6, 256 }, { 4, 7, 256 },
   { 2, 8, 256 }, { 3, 9, 256 }, { 3, 10, 256 }, { 4, 11, 256 }, { 3, 12, 256 }, { 4, 13, 256 }, { 4, 14, 256 }, { 5, 15, 256 },
   { 2, 16, 256 }, { 3, 17, 256 }, { 3, 18, 256 }, { 4, 19, 256 }, { 3, 20, 256 }, { 4, 21, 256 }, { 4, 22, 256 }, { 5, 23, 256 },
   { 3, 24, 256 }, { 4, 25, 256 }, { 4, 26, 256 }, { 5, 27, 256 }, { 4, 28, 256 }, { 5, 29, 256 }, { 5, 30, 256 }, { 6, 31, 256 },
   { 2, 32, 256 }, { 3, 33, 256 }, { 3, 34, 256 }, { 4, 35, 256 }, { 3, 36, 256 }, { 4, 37, 256 }, { 4, 38, 256 }, { 5, 39, 256 },
   { 3, 40, 256 }, { 4, 41, 256 }, { 4, 42, 256 }, { 5, 43, 256 }, { 4, 44, 256 }, { 5, 45, 256 }, { 5, 46, 256 }, { 6, 47, 256 },
   { 3, 48, 256 }, { 4, 49, 256 }, { 4, 50, 256 }, { 5, 51, 256 }, { 4, 52, 256 }, { 5, 53, 256 }, { 5, 54, 256 }, { 6, 55, 256 },
   { 4, 56, 256 }, { 5, 57, 256 }, { 5, 58, 256 }, { 6, 59, 256 }, { 5, 60, 256 }, { 6, 61, 256 }, { 6, 62, 256 }, { 7, 63, 256 },
   { 2, 64, 256 }, { 3, 65, 256 }, { 3, 66, 256 }, { 4, 67, 256 }, { 3, 68, 256 }, { 4, 69, 256 }, { 4, 70, 256 }, { 5, 71, 256 },
   { 3, 72, 256 }, { 4, 73, 256 }, { 4, 74, 256 }, { 5, 75, 256 }, { 4, 76, 256 }, { 5, 77, 256 }, { 5, 78, 256 }, { 6, 79, 256 },
   { 3, 80, 256 }, { 4, 81, 256 }, { 4, 82, 256 }, { 5, 83, 256 }, { 4, 84, 256 }, { 5, 85, 256 }, { 5, 86, 256 }, { 6, 87, 256 },
   { 4, 88, 256 }, { 5, 89, 256 }, { 5, 90, 256 }, { 6, 91, 256 }, { 5, 92, 256 }, { 6, 93, 256 }, { 6, 94, 256 }, { 7, 95, 256 },
   { 3, 96, 256 }, { 4, 97, 256 }, { 4, 98, 256 }, { 5, 99, 256 }, { 4, 100, 256 }, { 5, 101, 256 }, { 5, 102, 256 }, { 6, 103, 256 },
   { 4, 104, 256 }, { 5, 105, 256 }, { 5, 106, 256 }, { 6, 107, 256 }, { 5, 108, 256 }, { 6, 109, 256 }, { 6, 110, 256 }, { 7, 111, 256 },
   { 4, 112, 256 }, { 5, 113, 256 }, { 5, 114, 256 }, { 6, 115, 256 }, { 5, 116, 256 }, { 6, 117, 256 }, { 6, 118, 256 }, { 7, 119, 256 },
   { 5, 120, 256 }, { 6, 121, 256 }, { 6, 122, 256 }, { 7, 123, 256 }, { 6, 124, 256 }, { 7, 125, 256 }, { 7, 126, 256 }, { 8, 127, 256 },
   { 2, 128, 256 }, { 3, 129, 256 }, { 3, 130, 256 }, { 4, 131, 256 }, { 3, 132, 256 }, { 4, 133, 256 }, { 4, 134, 256 }, { 5, 135, 256 },
   { 3, 136, 256 }, { 4, 137, 256 }, { 4, 138, 256 }, { 5, 139, 256 }, { 4, 140, 256 }, { 5, 141, 256 }, { 5, 142, 256 }, { 6, 143, 256 },
   { 3, 144, 256 }, { 4, 145, 256 }, { 4, 146, 256 }, { 5, 147, 256 }, { 4, 148, 256 }, { 5, 149, 256 }, { 5, 150, 256 }, { 6, 151, 256 },
   { 4, 152, 256 }, { 5, 153, 256 }, { 5, 154, 256 }, { 6, 155, 256 }, { 5, 156, 256 }, { 6, 157, 256 }, { 6, 158, 256 }, { 7, 159, 256 },
   { 3, 160, 256 }, { 4, 161, 256 }, { 4, 162, 256 }, { 5, 163, 256 }, { 4, 164, 256 }, { 5, 165, 256 }, { 5, 166, 256 }, { 6, 167, 256 },
   { 4, 168, 256 }, { 5, 169, 256 }, { 5, 170, 256 }, { 6, 171, 256 }, { 5, 172, 256 }, { 6, 173, 256 }, { 6, 174, 256 }, { 7, 175, 256 },
   { 4, 176, 256 }, { 5, 177, 256 }, { 5, 178, 256 }, { 6, 179, 256 }, { 5, 180, 256 }, { 6, 181, 256 }, { 6, 182, 256 }, { 7, 183, 256 },
   { 5, 184, 256 }, { 6, 185, 256 }, { 6, 186, 256 }, { 7, 187, 256 }, { 6, 188, 256 }, { 7, 189, 256 }, { 7, 190, 256 }, { 8, 191, 256 },
   { 3, 192, 256 }, { 4, 193, 256 }, { 4, 194, 256 }, { 5, 195, 256 }, { 4, 196, 256 }, { 5, 197, 256 }, { 5, 198, 256 }, { 6, 199, 256 },
   { 4, 200, 256 }, { 5, 201, 256 }, { 5, 202, 256 }, { 6, 203, 256 }, { 5, 204, 256 }, { 6, 205, 256 }, { 6, 206, 256 }, { 7, 207, 256 },
   { 4, 208, 256 }, { 5, 209, 256 }, { 5, 210, 256 }, { 6, 211, 256 }, { 5, 212, 256 }, { 6, 213, 256 }, { 6, 214, 256 }, { 7, 215, 256 },
   { 5, 216, 256 }, { 6, 217, 256 }, { 6, 218, 256 }, { 7, 219, 256 }, { 6, 220, 256 }, { 7, 221, 256 }, { 7, 222, 256 }, { 8, 223, 256 },
   { 4, 224, 256 }, { 5, 225, 256 }, { 5, 226, 256 }, { 6, 227, 256 }, { 5, 228, 256 }, { 6, 229, 256 }, { 6, 230, 256 }, { 7, 231, 256 },
   { 5, 232, 256 }, { 6, 233, 256 }, { 6, 234, 256 }, { 7, 235, 256 }, { 6, 236, 256 }, { 7, 237, 256 }, { 7, 238, 256 }, { 8, 239, 256 },
   { 5, 240, 256 }, { 6, 241, 256 }, { 6, 242, 256 }, { 7, 243, 256 }, { 6, 244, 256 }, { 7, 245, 256 }, { 7, 246, 256 }, { 8, 247, 256 },
   { 6, 248, 256 }, { 7, 249, 256 }, { 7, 250, 256 }, { 8, 251, 256 }, { 7, 252, 256 }, { 8, 253, 256 }, { 8, 254, 256 }, { 9, 255, 256 },
#if FP_LUT > 9
   { 1, 0, 0 }, { 2, 1, 512 }, { 2, 2, 512 }, { 3, 3, 512 }, { 2, 4, 512 }, { 3, 5, 512 }, { 3, 6, 512 }, { 4, 7, 512 },
   { 2, 8, 512 }, { 3, 9, 512 }, { 3, 10, 512 }, { 4, 11, 512 }, { 3, 12, 512 }, { 4, 13, 512 }, { 4, 14, 512 }, { 5, 15, 512 },
   { 2, 16, 512 }, { 3, 17, 512 }, { 3, 18, 512 }, { 4, 19, 512 }, { 3, 20, 512 }, { 4, 21, 512 }, { 4, 22, 512 }, { 5, 23, 512 },
   { 3, 24, 512 }, { 4, 25, 512 }, { 4, 26, 512 }, { 5, 27, 512 }, { 4, 28, 512 }, { 5, 29, 512 }, { 5, 30, 512 }, { 6, 31, 512 },
   { 2, 32, 512 }, { 3, 33, 512 }, { 3, 34, 512 }, { 4, 35, 512 }, { 3, 36, 512 }, { 4, 37, 512 }, { 4, 38, 512 }, { 5, 39, 512 },
   { 3, 40, 512 }, { 4, 41, 512 }, { 4, 42, 512 }, { 5, 43, 512 }, { 4, 44, 512 }, { 5, 45, 512 }, { 5, 46, 512 }, { 6, 47, 512 },
   { 3, 48, 512 }, { 4, 49, 512 }, { 4, 50, 512 }, { 5, 51, 512 }, { 4, 52, 512 }, { 5, 53, 512 }, { 5, 54, 512 }, { 6, 55, 512 },
   { 4, 56, 512 }, { 5, 57, 512 }, { 5, 58, 512 }, { 6, 59, 512 }, { 5, 60, 512 }, { 6, 61, 512 }, { 6, 62, 512 }, { 7, 63, 512 },
   { 2, 64, 512 }, { 3, 65, 512 }, { 3, 66, 512 }, { 4, 67, 512 }, { 3, 68, 512 }, { 4, 69, 512 }, { 4, 70, 512 }, { 5, 71, 512 },
   { 3, 72, 512 }, { 4, 73, 512 }, { 4, 74, 512 }, { 5, 75, 512 }, { 4, 76, 512 }, { 5, 77, 512 }, { 5, 78, 512 }, { 6, 79, 512 },
   { 3, 80, 512 }, { 4, 81, 512 }, { 4, 82, 512 }, { 5, 83, 512 }, { 4, 84, 512 }, { 5, 85, 512 }, { 5, 86, 512 }, { 6, 87, 512 },
   { 4, 88, 512 }, { 5, 89, 512 }, { 5, 90, 512 }, { 6, 91, 512 }, { 5, 92, 512 }, { 6, 93, 512 }, { 6, 94, 512 }, { 7, 95, 512 },
   { 3, 96, 512 }, { 4, 97, 512 }, { 4, 98, 512 }, { 5, 99, 512 }, { 4, 100, 512 }, { 5, 101, 512 }, { 5, 102, 512 }, { 6, 103, 512 },
   { 4, 104, 512 }, { 5, 105, 512 }, { 5, 106, 512 }, { 6, 107, 512 }, { 5, 108, 512 }, { 6, 109, 512 }, { 6, 110, 512 }, { 7, 111, 512 },
   { 4, 112, 512 }, { 5, 113, 512 }, { 5, 114, 512 }, { 6, 115, 512 }, { 5, 116, 512 }, { 6, 117, 512 }, { 6, 118, 512 }, { 7, 119, 512 },
   { 5, 120, 512 }, { 6, 121, 512 }, { 6, 122, 512 }, { 7, 123, 512 }, { 6, 124, 512 }, { 7, 125, 512 }, { 7, 126, 512 }, { 8, 127, 512 },
   { 2, 128, 512 }, { 3, 129, 512 }, { 3, 130, 512 }, { 4, 131, 512 }, { 3, 132, 512 }, { 4, 133, 512 }, { 4, 134, 512 }, { 5, 135, 512 },
   { 3, 136, 512 }, { 4, 137, 512 }, { 4, 138, 512 }, { 5, 139, 512 }, { 4, 140, 512 }, { 5, 141, 512 }, { 5, 142, 512 }, { 6, 143, 512 },
   { 3, 144, 512 }, { 4, 145, 512 }, { 4, 146, 512 }, { 5, 147, 512 }, { 4, 148, 512 }, { 5, 149, 512 }, { 5, 150, 512 }, { 6, 151, 512 },
   { 4, 152, 512 }, { 5, 153, 512 }, { 5, 154, 512 }, { 6, 155, 512 }, { 5, 156, 512 }, { 6, 157, 512 }, { 6, 158, 512 }, { 7, 159, 512 },
   { 3, 160, 512 }, { 4, 161, 512 }, { 4, 162, 512 }, { 5, 163, 512 }, { 4, 164, 512 }, { 5, 165, 512 }, { 5, 166, 512 }, { 6, 167, 512 },
   { 4, 168, 512 }, { 5, 169, 512 }, { 5, 170, 512 }, { 6, 171, 512 }, { 5, 172, 512 }, { 6, 173, 512 }, { 6, 174, 512 }, { 7, 175, 512 },
   { 4, 176, 512 }, { 5, 177, 512 }, { 5, 178, 512 }, { 6, 179, 512 }, { 5, 180, 512 }, { 6, 181, 512 }, { 6, 182, 512 }, { 7, 183, 512 },
   { 5, 184, 512 }, { 6, 185, 512 }, { 6, 186, 512 }, { 7, 187, 512 }, { 6, 188, 512 }, { 7, 189, 512 }, { 7, 190, 512 }, { 8, 191, 512 },
   { 3, 192, 512 }, { 4, 193, 512 }, { 4, 194, 512 }, { 5, 195, 512 }, { 4, 196, 512 }, { 5, 197, 512 }, { 5, 198, 512 }, { 6, 199, 512 },
   { 4, 200, 512 }, { 5, 201, 512 }, { 5, 202, 512 }, { 6, 203, 512 }, { 5, 204, 512 }, { 6, 205, 512 }, { 6, 206, 512 }, { 7, 207, 512 },
   { 4, 208, 512 }, { 5, 209, 512 }, { 5, 210, 512 }, { 6, 211, 512 }, { 5, 212, 512 }, { 6, 213, 512 }, { 6, 214, 512 }, { 7, 215, 512 },
   { 5, 216, 512 }, { 6, 217, 512 }, { 6, 218, 512 }, { 7, 219, 512 }, { 6, 220, 512 }, { 7, 221, 512 }, { 7, 222, 512 }, { 8, 223, 512 },
   { 4, 224, 512 }, { 5, 225, 512 }, { 5, 226, 512 }, { 6, 227, 512 }, { 5, 228, 512 }, { 6, 229, 512 }, { 6, 230, 512 }, { 7, 231, 512 },
   { 5, 232, 512 }, { 6, 233, 512 }, { 6, 234, 512 }, { 7, 235, 512 }, { 6, 236, 512 }, { 7, 237, 512 }, { 7, 238, 512 }, { 8, 239, 512 },
   { 5, 240, 512 }, { 6, 241, 512 }, { 6, 242, 512 }, { 7, 243, 512 }, { 6, 244, 512 }, { 7, 245, 512 }, { 7, 246, 512 }, { 8, 247, 512 },
   { 6, 248, 512 }, { 7, 249, 512 }, { 7, 250, 512 }, { 8, 251, 512 }, { 7, 252, 512 }, { 8, 253, 512 }, { 8, 254, 512 }, { 9, 255, 512 },
   { 2, 256, 512 }, { 3, 257, 512 }, { 3, 258, 512 }, { 4, 259, 512 }, { 3, 260, 512 }, { 4, 261, 512 }, { 4, 262, 512 }, { 5, 263, 512 },
   { 3, 264, 512 }, { 4, 265, 512 }, { 4, 266, 512 }, { 5, 267, 512 }, { 4, 268, 512 }, { 5, 269, 512 }, { 5, 270, 512 }, { 6, 271, 512 },
   { 3, 272, 512 }, { 4, 273, 512 }, { 4, 274, 512 }, { 5, 275, 512 }, { 4, 276, 512 }, { 5, 277, 512 }, { 5, 278, 512 }, { 6, 279, 512 },
   { 4, 280, 512 }, { 5, 281, 512 }, { 5, 282, 512 }, { 6, 283, 512 }, { 5, 284, 512 }, { 6, 285, 512 }, { 6, 286, 512 }, { 7, 287, 512 },
   { 3, 288, 512 }, { 4, 289, 512 }, { 4, 290, 512 }, { 5, 291, 512 }, { 4, 292, 512 }, { 5, 293, 512 }, { 5, 294, 512 }, { 6, 295, 512 },
   { 4, 296, 512 }, { 5, 297, 512 }, { 5, 298, 512 }, { 6, 299, 512 }, { 5, 300, 512 }, { 6, 301, 512 }, { 6, 302, 512 }, { 7, 303, 512 },
   { 4, 304, 512 }, { 5, 305, 512 }, { 5, 306, 512 }, { 6, 307, 512 }, { 5, 308, 512 }, { 6, 309, 512 }, { 6, 310, 512 }, { 7, 311, 512 },
   { 5, 312, 512 }, { 6, 313, 512 }, { 6, 314, 512 }, { 7, 315, 512 }, { 6, 316, 512 }, { 7, 317, 512 }, { 7, 318, 512 }, { 8, 319, 512 },
   { 3, 320, 512 }, { 4, 321, 512 }, { 4, 322, 512 }, { 5, 323, 512 }, { 4, 324, 512 }, { 5, 325, 512 }, { 5, 326, 512 }, { 6, 327, 512 },
   { 4, 328, 512 }, { 5, 329, 512 }, { 5, 330, 512 }, { 6, 331, 512 }, { 5, 332, 512 }, { 6, 333, 512 }, { 6, 334, 512 }, { 7, 335, 512 },
   { 4, 336, 512 }, { 5, 337, 512 }, { 5, 338, 512 }, { 6, 339, 512 }, { 5, 340, 512 }, { 6, 341, 512 }, { 6, 342, 512 }, { 7, 343, 512 },
   { 5, 344, 512 }, { 6, 345, 512 }, { 6, 346, 512 }, { 7, 347, 512 }, { 6, 348, 512 }, { 7, 349, 512 }, { 7, 350, 512 }, { 8, 351, 512 },
   { 4, 352, 512 }, { 5, 353, 512 }, { 5, 354, 512 }, { 6, 355, 512 }, { 5, 356, 512 }, { 6, 357, 512 }, { 6, 358, 512 }, { 7, 359, 512 },
   { 5, 360, 512 }, { 6, 361, 512 }, { 6, 362, 512 }, { 7, 363, 512 }, { 6, 364, 512 }, { 7, 365, 512 }, { 7, 366, 512 }, { 8, 367, 512 },
   { 5, 368, 512 }, { 6, 369, 512 }, { 6, 370, 512 }, { 7, 371, 512 }, { 6, 372, 512 }, { 7, 373, 512 }, { 7, 374, 512 }, { 8, 375, 512 },
   { 6, 376, 512 }, { 7, 377, 512 }, { 7, 378, 512 }, { 8, 379, 512 }, { 7, 380, 512 }, { 8, 381, 512 }, { 8, 382, 512 }, { 9, 383, 512 },
   { 3, 384, 512 }, { 4, 385, 512 }, { 4, 386, 512 }, { 5, 387, 512 }, { 4, 388, 512 }, { 5, 389, 512 }, { 5, 390, 512 }, { 6, 391, 512 },
   { 4, 392, 512 }, { 5, 393, 512 }, { 5, 394, 512 }, { 6, 395, 512 }, { 5, 396, 512 }, { 6, 397, 512 }, { 6, 398, 512 }, { 7, 399, 512 },
   { 4, 400, 512 }, { 5, 401, 512 }, { 5, 402, 512 }, { 6, 403, 512 }, { 5, 404, 512 }, { 6, 405, 512 }, { 6, 406, 512 }, { 7, 407, 512 },
   { 5, 408, 512 }, { 6, 409, 512 }, { 6, 410, 512 }, { 7, 411, 512 }, { 6, 412, 512 }, { 7, 413, 512 }, { 7, 414, 512 }, { 8, 415, 512 },
   { 4, 416, 512 }, { 5, 417, 512 }, { 5, 418, 512 }, { 6, 419, 512 }, { 5, 420, 512 }, { 6, 421, 512 }, { 6, 422, 512 }, { 7, 423, 512 },
   { 5, 424, 512 }, { 6, 425, 512 }, { 6, 426, 512 }, { 7, 427, 512 }, { 6, 428, 512 }, { 7, 429, 512 }, { 7, 430, 512 }, { 8, 431, 512 },
   { 5, 432, 512 }, { 6, 433, 512 }, { 6, 434, 512 }, { 7, 435, 512 }, { 6, 436, 512 }, { 7, 437, 512 }, { 7, 438, 512 }, { 8, 439, 512 },
   { 6, 440, 512 }, { 7, 441, 512 }, { 7, 442, 512 }, { 8, 443, 512 }, { 7, 444, 512 }, { 8, 445, 512 }, { 8, 446, 512 }, { 9, 447, 512 },
   { 4, 448, 512 }, { 5, 449, 512 }, { 5, 450, 512 }, { 6, 451, 512 }, { 5, 452, 512 }, { 6, 453, 512 }, { 6, 454, 512 }, { 7, 455, 512 },
   { 5, 456, 512 }, { 6, 457, 512 }, { 6, 458, 512 }, { 7, 459, 512 }, { 6, 460, 512 }, { 7, 461, 512 }, { 7, 462, 512 }, { 8, 463, 512 },
   { 5, 464, 512 }, { 6, 465, 512 }, { 6, 466, 512 }, { 7, 467, 512 }, { 6, 468, 512 }, { 7, 469, 512 }, { 7, 470, 512 }, { 8, 471, 512 },
   { 6, 472, 512 }, { 7, 473, 512 }, { 7, 474, 512 }, { 8, 475, 512 }, { 7, 476, 512 }, { 8, 477, 512 }, { 8, 478, 512 }, { 9, 479, 512 },
   { 5, 480, 512 }, { 6, 481, 512 }, { 6, 482, 512 }, { 7, 483, 512 }, { 6, 484, 512 }, { 7, 485, 512 }, { 7, 486, 512 }, { 8, 487, 512 },
   { 6, 488, 512 }, { 7, 489, 512 }, { 7, 490, 512 }, { 8, 491, 512 }, { 7, 492, 512 }, { 8, 493, 512 }, { 8, 494, 512 }, { 9, 495, 512 },
   { 6, 496, 512 }, { 7, 497, 512 }, { 7, 498, 512 }, { 8, 499, 512 }, { 7, 500, 512 }, { 8, 501, 512 }, { 8, 502, 512 }, { 9, 503, 512 },
   { 7, 504, 512 }, { 8, 505, 512 }, { 8, 506, 512 }, { 9, 507, 512 }, { 8, 508, 512 }, { 9, 509, 512 }, { 9, 510, 512 }, { 10, 511, 512 },
#if FP_LUT > 10
   { 1, 0, 0 }, { 2, 1, 1024 }, { 2, 2, 1024 }, { 3, 3, 1024 }, { 2, 4, 1024 }, { 3, 5, 1024 }, { 3, 6, 1024 }, { 4, 7, 1024 },
   { 2, 8, 1024 }, { 3, 9, 1024 }, { 3, 10, 1024 }, { 4, 11, 1024 }, { 3, 12, 1024 }, { 4, 13, 1024 }, { 4, 14, 1024 }, { 5, 15, 1024 },
   { 2, 16, 1024 }, { 3, 17, 1024 }, { 3, 18, 1024 }, { 4, 19, 1024 }, { 3, 20, 1024 }, { 4, 21, 1024 }, { 4, 22, 1024 }, { 5, 23, 1024 },
   { 3, 24, 1024 }, { 4, 25, 1024 }, { 4, 26, 1024 }, { 5, 27, 1024 }, { 4, 28, 1024 }, { 5, 29, 1024 }, { 5, 30, 1024 }, { 6, 31, 1024 },
   { 2, 32, 1024 }, { 3, 33, 1024 }, { 3, 34, 1024 }, { 4, 35, 1024 }, { 3, 36, 1024 }, { 4, 37, 1024 }, { 4, 38, 1024 }, { 5, 39, 1024 },
   { 3, 40, 1024 }, { 4, 41, 1024 }, { 4, 42, 1024 }, { 5, 43, 1024 }, { 4, 44, 1024 }, { 5, 45, 1024 }, { 5, 46, 1024 }, { 6, 47, 1024 },
   { 3, 48, 1024 }, { 4, 49, 1024 }, { 4, 50, 1024 }, { 5, 51, 1024 }, { 4, 52, 1024 }, { 5, 53, 1024 }, { 5, 54, 1024 }, { 6, 55, 1024 },
   { 4, 56, 1024 }, { 5, 57, 1024 }, { 5, 58, 1024 }, { 6, 59, 1024 }, { 5, 60, 1024 }, { 6, 61, 1024 }, { 6, 62, 1024 }, { 7, 63, 1024 },
   { 2, 64, 1024 }, { 3, 65, 1024 }, { 3, 66, 1024 }, { 4, 67, 1024 }, { 3, 68, 1024 }, { 4, 69, 1024 }, { 4, 70, 1024 }, { 5, 71, 1024 },
   { 3, 72, 1024 }, { 4, 73, 1024 }, { 4, 74, 1024 }, { 5, 75, 1024 }, { 4, 76, 1024 }, { 5, 77, 1024 }, { 5, 78, 1024 }, { 6, 79, 1024 },
   { 3, 80, 1024 }, { 4, 81, 1024 }, { 4, 82, 1024 }, { 5, 83, 1024 }, { 4, 84, 1024 }, { 5, 85, 1024 }, { 5, 86, 1024 }, { 6, 87, 1024 },
   { 4, 88, 1024 }, { 5, 89, 1024 }, { 5, 90, 1024 }, { 6, 91, 1024 }, { 5, 92, 1024 }, { 6, 93, 1024 }, { 6, 94, 1024 }, { 7, 95, 1024 },
   { 3, 96, 1024 }, { 4, 97, 1024 }, { 4, 98, 1024 }, { 5, 99, 1024 }, { 4, 100, 1024 }, { 5, 101, 1024 }, { 5, 102, 1024 }, { 6, 103, 1024 },
   { 4, 104, 1024 }, { 5, 105, 1024 }, { 5, 106, 1024 }, { 6, 107, 1024 }, { 5, 108, 1024 }, { 6, 109, 1024 }, { 6, 110, 1024 }, { 7, 111, 1024 },
   { 4, 112, 1024 }, { 5, 113, 1024 }, { 5, 114, 1024 }, { 6, 115, 1024 }, { 5, 116, 1024 }, { 6, 117, 1024 }, { 6, 118, 1024 }, { 7, 119, 1024 },
   { 5, 120, 1024 }, { 6, 121, 1024 }, { 6, 122, 1024 }, { 7, 123, 1024 }, { 6, 124, 1024 }, { 7, 125, 1024 }, { 7, 126, 1024 }, { 8, 127, 1024 },
   { 2, 128, 1024 }, { 3, 129, 1024 }, { 3, 130, 1024 }, { 4, 131, 1024 }, { 3, 132, 1024 }, { 4, 133, 1024 }, { 4, 134, 1024 }, { 5, 135, 1024 },
   { 3, 136, 1024 }, { 4, 137, 1024 }, { 4, 138, 1024 }, { 5, 139, 1024 }, { 4, 140, 1024 }, { 5, 141, 1024 }, { 5, 142, 1024 }, { 6, 143, 1024 },
   { 3, 144, 1024 }, { 4, 145, 1024 }, { 4, 146, 1024 }, { 5, 147, 1024 }, { 4, 148, 1024 }, { 5, 149, 1024 }, { 5, 150, 1024 }, { 6, 151, 1024 },
   { 4, 152, 1024 }, { 5, 153, 1024 }, { 5, 154, 1024 }, { 6, 155, 1024 }, { 5, 156, 1024 }, { 6, 157, 1024 }, { 6, 158, 1024 }, { 7, 159, 1024 },
   { 3, 160, 1024 }, { 4, 161, 1024 }, { 4, 162, 1024 }, { 5, 163, 1024 }, { 4, 164, 1024 }, { 5, 165, 1024 }, { 5, 166, 1024 }, { 6, 167, 1024 },
   { 4, 168, 1024 }, { 5, 169, 1024 }, { 5, 170, 1024 }, { 6, 171, 1024 }, { 5, 172, 1024 }, { 6, 173, 1024 }, { 6, 174, 1024 }, { 7, 175, 1024 },
   { 4, 176, 1024 }, { 5, 177, 1024 }, { 5, 178, 1024 }, { 6, 179, 1024 }, { 5, 180, 1024 }, { 6, 181, 1024 }, { 6, 182, 1024 }, { 7, 183, 1024 },
   { 5, 184, 1024 }, { 6, 185, 1024 }, { 6, 186, 1024 }, { 7, 187, 1024 }, { 6, 188, 1024 }, { 7, 189, 1024 }, { 7, 190, 1024 }, { 8, 191, 1024 },
   { 3, 192, 1024 }, { 4, 193, 1024 }, { 4, 194, 1024 }, { 5, 195, 1024 }, { 4, 196, 1024 }, { 5, 197, 1024 }, { 5, 198, 1024 }, { 6, 199, 1024 },
   { 4, 200, 1024 }, { 5, 201, 1024 }, { 5, 202, 1024 }, { 6, 203, 1024 }, { 5, 204, 1024 }, { 6, 205, 1024 }, { 6, 206, 1024 }, { 7, 207, 1024 },
   { 4, 208, 1024 }, { 5, 209, 1024 }, { 5, 210, 1024 }, { 6, 211, 1024 }, { 5, 212, 1024 }, { 6, 213, 1024 }, { 6, 214, 1024 }, { 7, 215, 1024 },
   { 5, 216, 1024 }, { 6, 217, 1024 }, { 6, 218, 1024 }, { 7, 219, 1024 }, { 6, 220, 1024 }, { 7, 221, 1024 }, { 7, 222, 1024 }, { 8, 223, 1024 },
   { 4, 224, 1024 }, { 5, 225, 1024 }, { 5, 226, 1024 }, { 6, 227, 1024 }, { 5, 228, 1024 }, { 6, 229, 1024 }, { 6, 230, 1024 }, { 7, 231, 1024 },
   { 5, 232, 1024 }, { 6, 233, 1024 }, { 6, 234, 1024 }, { 7, 235, 1024 }, { 6, 236, 1024 }, { 7, 237, 1024 }, { 7, 238, 1024 }, { 8, 239, 1024 },
   { 5, 240, 1024 }, { 6, 241, 1024 }, { 6, 242, 1024 }, { 7, 243, 1024 }, { 6, 244, 1024 }, { 7, 245, 1024 }, { 7, 246, 1024 }, { 8, 247, 1024 },
   { 6, 248, 1024 }, { 7, 249, 1024 }, { 7, 250, 1024 }, { 8, 251, 1024 }, { 7, 252, 1024 }, { 8, 253, 1024 }, { 8, 254, 1024 }, { 9, 255, 1024 },
   { 2, 256, 1024 }, { 3, 257, 1024 }, { 3, 258, 1024 }, { 4, 259, 1024 }, { 3, 260, 1024 }, { 4, 261, 1024 }, { 4, 262, 1024 }, { 5, 263, 1024 },
   { 3, 264, 1024 }, { 4, 265, 1024 }, { 4, 266, 1024 }, { 5, 267, 1024 }, { 4, 268, 1024 }, { 5, 269, 1024 }, { 5, 270, 1024 }, { 6, 271, 1024 },
   { 3, 272, 1024 }, { 4, 273, 1024 }, { 4, 274, 1024 }, { 5, 275, 1024 }, { 4, 276, 1024 }, { 5, 277, 1024 }, { 5, 278, 1024 }, { 6, 279, 1024 },
   { 4, 280, 1024 }, { 5, 281, 1024 }, { 5, 282, 1024 }, { 6, 283, 1024 }, { 5, 284, 1024 }, { 6, 285, 1024 }, { 6, 286, 1024 }, { 7, 287, 1024 },
   { 3, 288, 1024 }, { 4, 289, 1024 }, { 4, 290, 1024 }, { 5, 291, 1024 }, { 4, 292, 1024 }, { 5, 293, 1024 }, { 5, 294, 1024 }, { 6, 295, 1024 },
   { 4, 296, 1024 }, { 5, 297, 1024 }, { 5, 298, 1024 }, { 6, 299, 1024 }, { 5, 300, 1024 }, { 6, 301, 1024 }, { 6, 302, 1024 }, { 7, 303, 1024 },
   { 4, 304, 1024 }, { 5, 305, 1024 }, { 5, 306, 1024 }, { 6, 307, 1024 }, { 5, 308, 1024 }, { 6, 309, 1024 }, { 6, 310, 1024 }, { 7, 311, 1024 },
   { 5, 312, 1024 }, { 6, 313, 1024 }, { 6, 314, 1024 }, { 7, 315, 1024 }, { 6, 316, 1024 }, { 7, 317, 1024 }, { 7, 318, 1024 }, { 8, 319, 1024 },
   { 3, 320, 1024 }, { 4, 321, 1024 }, { 4, 322, 1024 }, { 5, 323, 1024 }, { 4, 324, 1024 }, { 5, 325, 1024 }, { 5, 326, 1024 }, { 6, 327, 1024 },
   { 4, 328, 1024 }, { 5, 329, 1024 }, { 5, 330, 1024 }, { 6, 331, 1024 }, { 5, 332, 1024 }, { 6, 333, 1024 }, { 6, 334, 1024 }, { 7, 335, 1024 },
   { 4, 336, 1024 }, { 5, 337, 1024 }, { 5, 338, 1024 }, { 6, 339, 1024 }, { 5, 340, 1024 }, { 6, 341, 1024 }, { 6, 342, 1024 }, { 7, 343, 1024 },
   { 5, 344, 1024 }, { 6, 345, 1024 }, { 6, 346, 1024 }, { 7, 347, 1024 }, { 6, 348, 1024 }, { 7, 349, 1024 }, { 7, 350, 1024 }, { 8, 351, 1024 },
   { 4, 352, 1024 }, { 5, 353, 1024 }, { 5, 354, 1024 }, { 6, 355, 1024 }, { 5, 356, 1024 }, { 6, 357, 1024 }, { 6, 358, 1024 }, { 7, 359, 1024 },
   { 5, 360, 1024 }, { 6, 361, 1024 }, { 6, 362, 1024 }, { 7, 363, 1024 }, { 6, 364, 1024 }, { 7, 365, 1024 }, { 7, 366, 1024 }, { 8, 367, 1024 },
   { 5, 368, 1024 }, { 6, 369, 1024 }, { 6, 370, 1024 }, { 7, 371, 1024 }, { 6, 372, 1024 }, { 7, 373, 1024 }, { 7, 374, 1024 }, { 8, 375, 1024 },
   { 6, 376, 1024 }, { 7, 377, 1024 }, { 7, 378, 1024 }, { 8, 379, 1024 }, { 7, 380, 1024 }, { 8, 381, 1024 }, { 8, 382, 1024 }, { 9, 383, 1024 },
   { 3, 384, 1024 }, { 4, 385, 1024 }, { 4, 386, 1024 }, { 5, 387, 1024 }, { 4, 388, 1024 }, { 5, 389, 1024 }, { 5, 390, 1024 }, { 6, 391, 1024 },
   { 4, 392, 1024 }, { 5, 393, 1024 }, { 5, 394, 1024 }, { 6, 395, 1024 }, { 5, 396, 1024 }, { 6, 397, 1024 }, { 6, 398, 1024 }, { 7, 399, 1024 },
   { 4, 400, 1024 }, { 5, 401, 1024 }, { 5, 402, 1024 }, { 6, 403, 1024 }, { 5, 404, 1024 }, { 6, 405, 1024 }, { 6, 406, 1024 }, { 7, 407, 1024 },
   { 5, 408, 1024 }, { 6, 409, 1024 }, { 6, 410, 1024 }, { 7, 411, 1024 }, { 6, 412, 1024 }, { 7, 413, 1024 }, { 7, 414, 1024 }, { 8, 415, 1024 },
   { 4, 416, 1024 }, { 5, 417, 1024 }, { 5, 418, 1024 }, { 6, 419, 1024 }, { 5, 420, 1024 }, { 6, 421, 1024 }, { 6, 422, 1024 }, { 7, 423, 1024 },
   { 5, 424, 1024 }, { 6, 425, 1024 }, { 6, 426, 1024 }, { 7, 427, 1024 }, { 6, 428, 1024 }, { 7, 429, 1024 }, { 7, 430, 1024 }, { 8, 431, 1024 },
   { 5, 432, 1024 }, { 6, 433, 1024 }, { 6, 434, 1024 }, { 7, 435, 1024 }, { 6, 436, 1024 }, { 7, 437, 1024 }, { 7, 438, 1024 }, { 8, 439, 1024 },
   { 6, 440, 1024 }, { 7, 441, 1024 }, { 7, 442, 1024 }, { 8, 443, 1024 }, { 7, 444, 1024 }, { 8, 445, 1024 }, { 8, 446, 1024 }, { 9, 447, 1024 },
   { 4, 448, 1024 }, { 5, 449, 1024 }, { 5, 450, 1024 }, { 6, 451, 1024 }, { 5, 452, 1024 }, { 6, 453, 1024 }, { 6, 454, 1024 }, { 7, 455, 1024 },
   { 5, 456, 1024 }, { 6, 457, 1024 }, { 6, 458, 1024 }, { 7, 459, 1024 }, { 6, 460, 1024 }, { 7, 461, 1024 }, { 7, 462, 1024 }, { 8, 463, 1024 },
   { 5, 464, 1024 }, { 6, 465, 1024 }, { 6, 466, 1024 }, { 7, 467, 1024 }, { 6, 468, 1024 }, { 7, 469, 1024 }, { 7, 470, 1024 }, { 8, 471, 1024 },
   { 6, 472, 1024 }, { 7, 473, 1024 }, { 7, 474, 1024 }, { 8, 475, 1024 }, { 7, 476, 1024 }, { 8, 477, 1024 }, { 8, 478, 1024 }, { 9, 479, 1024 },
   { 5, 480, 1024 }, { 6, 481, 1024 }, { 6, 482, 1024 }, { 7, 483, 1024 }, { 6, 484, 1024 }, { 7, 485, 1024 }, { 7, 486, 1024 }, { 8, 487, 1024 },
   { 6, 488, 1024 }, { 7, 489, 1024 }, { 7, 490, 1024 }, { 8, 491, 1024 }, { 7, 492, 1024 }, { 8, 493, 1024 }, { 8, 494, 1024 }, { 9, 495, 1024 },
   { 6, 496, 1024 }, { 7, 497, 1024 }, { 7, 498, 1024 }, { 8, 499, 1024 }, { 7, 500, 1024 }, { 8, 501, 1024 }, { 8, 502, 1024 }, { 9, 503, 1024 },
   { 7, 504, 1024 }, { 8, 505, 1024 }, { 8, 506, 1024 }, { 9, 507, 1024 }, { 8, 508, 1024 }, { 9, 509, 1024 }, { 9, 510, 1024 }, { 10, 511, 1024 },
   { 2, 512, 1024 }, { 3, 513, 1024 }, { 3, 514, 1024 }, { 4, 515, 1024 }, { 3, 516, 1024 }, { 4, 517, 1024 }, { 4, 518, 1024 }, { 5, 519, 1024 },
   { 3, 520, 1024 }, { 4, 521, 1024 }, { 4, 522, 1024 }, { 5, 523, 1024 }, { 4, 524, 1024 }, { 5, 525, 1024 }, { 5, 526, 1024 }, { 6, 527, 1024 },
   { 3, 528, 1024 }, { 4, 529, 1024 }, { 4, 530, 1024 }, { 5, 531, 1024 }, { 4, 532, 1024 }, { 5, 533, 1024 }, { 5, 534, 1024 }, { 6, 535, 1024 },
   { 4, 536, 1024 }, { 5, 537, 1024 }, { 5, 538, 1024 }, { 6, 539, 1024 }, { 5, 540, 1024 }, { 6, 541, 1024 }, { 6, 542, 1024 }, { 7, 543, 1024 },
   { 3, 544, 1024 }, { 4, 545, 1024 }, { 4, 546, 1024 }, { 5, 547, 1024 }, { 4, 548, 1024 }, { 5, 549, 1024 }, { 5, 550, 1024 }, { 6, 551, 1024 },
   { 4, 552, 1024 }, { 5, 553, 1024 }, { 5, 554, 1024 }, { 6, 555, 1024 }, { 5, 556, 1024 }, { 6, 557, 1024 }, { 6, 558, 1024 }, { 7, 559, 1024 },
   { 4, 560, 1024 }, { 5, 561, 1024 }, { 5, 562, 1024 }, { 6, 563, 1024 }, { 5, 564, 1024 }, { 6, 565, 1024 }, { 6, 566, 1024 }, { 7, 567, 1024 },
   { 5, 568, 1024 }, { 6, 569, 1024 }, { 6, 570, 1024 }, { 7, 571, 1024 }, { 6, 572, 1024 }, { 7, 573, 1024 }, { 7, 574, 1024 }, { 8, 575, 1024 },
   { 3, 576, 1024 }, { 4, 577, 1024 }, { 4, 578, 1024 }, { 5, 579, 1024 }, { 4, 580, 1024 }, { 5, 581, 1024 }, { 5, 582, 1024 }, { 6, 583, 1024 },
   { 4, 584, 1024 }, { 5, 585, 1024 }, { 5, 586, 1024 }, { 6, 587, 1024 }, { 5, 588, 1024 }, { 6, 589, 1024 }, { 6, 590, 1024 }, { 7, 591, 1024 },
   { 4, 592, 1024 }, { 5, 593, 1024 }, { 5, 594, 1024 }, { 6, 595, 1024 }, { 5, 596, 1024 }, { 6, 597, 1024 }, { 6, 598, 1024 }, { 7, 599, 1024 },
   { 5, 600, 1024 }, { 6, 601, 1024 }, { 6, 602, 1024 }, { 7, 603, 1024 }, { 6, 604, 1024 }, { 7, 605, 1024 }, { 7, 606, 1024 }, { 8, 607, 1024 },
   { 4, 608, 1024 }, { 5, 609, 1024 }, { 5, 610, 1024 }, { 6, 611, 1024 }, { 5, 612, 1024 }, { 6, 613, 1024 }, { 6, 614, 1024 }, { 7, 615, 1024 },
   { 5, 616, 1024 }, { 6, 617, 1024 }, { 6, 618, 1024 }, { 7, 619, 1024 }, { 6, 620, 1024 }, { 7, 621, 1024 }, { 7, 622, 1024 }, { 8, 623, 1024 },
   { 5, 624, 1024 }, { 6, 625, 1024 }, { 6, 626, 1024 }, { 7, 627, 1024 }, { 6, 628, 1024 }, { 7, 629, 1024 }, { 7, 630, 1024 }, { 8, 631, 1024 },
   { 6, 632, 1024 }, { 7, 633, 1024 }, { 7, 634, 1024 }, { 8, 635, 1024 }, { 7, 636, 1024 }, { 8, 637, 1024 }, { 8, 638, 1024 }, { 9, 639, 1024 },
   { 3, 640, 1024 }, { 4, 641, 1024 }, { 4, 642, 1024 }, { 5, 643, 1024 }, { 4, 644, 1024 }, { 5, 645, 1024 }, { 5, 646, 1024 }, { 6, 647, 1024 },
   { 4, 648, 1024 }, { 5, 649, 1024 }, { 5, 650, 1024 }, { 6, 651, 1024 }, { 5, 652, 1024 }, { 6, 653, 1024 }, { 6, 654, 1024 }, { 7, 655, 1024 },
   { 4, 656, 1024 }, { 5, 657, 1024 }, { 5, 658, 1024 }, { 6, 659, 1024 }, { 5, 660, 1024 }, { 6, 661, 1024 }, { 6, 662, 1024 }, { 7, 663, 1024 },
   { 5, 664, 1024 }, { 6, 665, 1024 }, { 6, 666, 1024 }, { 7, 667, 1024 }, { 6, 668, 1024 }, { 7, 669, 1024 }, { 7, 670, 1024 }, { 8, 671, 1024 },
   { 4, 672, 1024 }, { 5, 673, 1024 }, { 5, 674, 1024 }, { 6, 675, 1024 }, { 5, 676, 1024 }, { 6, 677, 1024 }, { 6, 678, 1024 }, { 7, 679, 1024 },
   { 5, 680, 1024 }, { 6, 681, 1024 }, { 6, 682, 1024 }, { 7, 683, 1024 }, { 6, 684, 1024 }, { 7, 685, 1024 }, { 7, 686, 1024 }, { 8, 687, 1024 },
   { 5, 688, 1024 }, { 6, 689, 1024 }, { 6, 690, 1024 }, { 7, 691, 1024 }, { 6, 692, 1024 }, { 7, 693, 1024 }, { 7, 694, 1024 }, { 8, 695, 1024 },
   { 6, 696, 1024 }, { 7, 697, 1024 }, { 7, 698, 1024 }, { 8, 699, 1024 }, { 7, 700, 1024 }, { 8, 701, 1024 }, { 8, 702, 1024 }, { 9, 703, 1024 },
   { 4, 704, 1024 }, { 5, 705, 1024 }, { 5, 706, 1024 }, { 6, 707, 1024 }, { 5, 708, 1024 }, { 6, 709, 1024 }, { 6, 710, 1024 }, { 7, 711, 1024 },
   { 5, 712, 1024 }, { 6, 713, 1024 }, { 6, 714, 1024 }, { 7, 715, 1024 }, { 6, 716, 1024 }, { 7, 717, 1024 }, { 7, 718, 1024 }, { 8, 719, 1024 },
   { 5, 720, 1024 }, { 6, 721, 1024 }, { 6, 722, 1024 }, { 7, 723, 1024 }, { 6, 724, 1024 }, { 7, 725, 1024 }, { 7, 726, 1024 }, { 8, 727, 1024 },
   { 6, 728, 1024 }, { 7, 729, 1024 }, { 7, 730, 1024 }, { 8, 731, 1024 }, { 7, 732, 1024 }, { 8, 733, 1024 }, { 8, 734, 1024 }, { 9, 735, 1024 },
   { 5, 736, 1024 }, { 6, 737, 1024 }, { 6, 738, 1024 }, { 7, 739, 1024 }, { 6, 740, 1024 }, { 7, 741, 1024 }, { 7, 742, 1024 }, { 8, 743, 1024 },
   { 6, 744, 1024 }, { 7, 745, 1024 }, { 7, 746, 1024 }, { 8, 747, 1024 }, { 7, 748, 1024 }, { 8, 749, 1024 }, { 8, 750, 1024 }, { 9, 751, 1024 },
   { 6, 752, 1024 }, { 7, 753, 1024 }, { 7, 754, 1024 }, { 8, 755, 1024 }, { 7, 756, 1024 }, { 8, 757, 1024 }, { 8, 758, 1024 }, { 9, 759, 1024 },
   { 7, 760, 1024 }, { 8, 761, 1024 }, { 8, 762, 1024 }, { 9, 763, 1024 }, { 8, 764, 1024 }, { 9, 765, 1024 }, { 9, 766, 1024 }, { 10, 767, 1024 },
   { 3, 768, 1024 }, { 4, 769, 1024 }, { 4, 770, 1024 }, { 5, 771, 1024 }, { 4, 772, 1024 }, { 5, 773, 1024 }, { 5, 774, 1024 }, { 6, 775, 1024 },
   { 4, 776, 1024 }, { 5, 777, 1024 }, { 5, 778, 1024 }, { 6, 779, 1024 }, { 5, 780, 1024 }, { 6, 781, 1024 }, { 6, 782, 1024 }, { 7, 783, 1024 },
   { 4, 784, 1024 }, { 5, 785, 1024 }, { 5, 786, 1024 }, { 6, 787, 1024 }, { 5, 788, 1024 }, { 6, 789, 1024 }, { 6, 790, 1024 }, { 7, 791, 1024 },
   { 5, 792, 1024 }, { 6, 793, 1024 }, { 6, 794, 1024 }, { 7, 795, 1024 }, { 6, 796, 1024 }, { 7, 797, 1024 }, { 7, 798, 1024 }, { 8, 799, 1024 },
   { 4, 800, 1024 }, { 5, 801, 1024 }, { 5, 802, 1024 }, { 6, 803, 1024 }, { 5, 804, 1024 }, { 6, 805, 1024 }, { 6, 806, 1024 }, { 7, 807, 1024 },
   { 5, 808, 1024 }, { 6, 809, 1024 }, { 6, 810, 1024 }, { 7, 811, 1024 }, { 6, 812, 1024 }, { 7, 813, 1024 }, { 7, 814, 1024 }, { 8, 815, 1024 },
   { 5, 816, 1024 }, { 6, 817, 1024 }, { 6, 818, 1024 }, { 7, 819, 1024 }, { 6, 820, 1024 }, { 7, 821, 1024 }, { 7, 822, 1024 }, { 8, 823, 1024 },
   { 6, 824, 1024 }, { 7, 825, 1024 }, { 7, 826, 1024 }, { 8, 827, 1024 }, { 7, 828, 1024 }, { 8, 829, 1024 }, { 8, 830, 1024 }, { 9, 831, 1024 },
   { 4, 832, 1024 }, { 5, 833, 1024 }, { 5, 834, 1024 }, { 6, 835, 1024 }, { 5, 836, 1024 }, { 6, 837, 1024 }, { 6, 838, 1024 }, { 7, 839, 1024 },
   { 5, 840, 1024 }, { 6, 841, 1024 }, { 6, 842, 1024 }, { 7, 843, 1024 }, { 6, 844, 1024 }, { 7, 845, 1024 }, { 7, 846, 1024 }, { 8, 847, 1024 },
   { 5, 848, 1024 }, { 6, 849, 1024 }, { 6, 850, 1024 }, { 7, 851, 1024 }, { 6, 852, 1024 }, { 7, 853, 1024 }, { 7, 854, 1024 }, { 8, 855, 1024 },
   { 6, 856, 1024 }, { 7, 857, 1024 }, { 7, 858, 1024 }, { 8, 859, 1024 }, { 7, 860, 1024 }, { 8, 861, 1024 }, { 8, 862, 1024 }, { 9, 863, 1024 },
   { 5, 864, 1024 }, { 6, 865, 1024 }, { 6, 866, 1024 }, { 7, 867, 1024 }, { 6, 868, 1024 }, { 7, 869, 1024 }, { 7, 870, 1024 }, { 8, 871, 1024 },
   { 6, 872, 1024 }, { 7, 873, 1024 }, { 7, 874, 1024 }, { 8, 875, 1024 }, { 7, 876, 1024 }, { 8, 877, 1024 }, { 8, 878, 1024 }, { 9, 879, 1024 },
   { 6, 880, 1024 }, { 7, 881, 1024 }, { 7, 882, 1024 }, { 8, 883, 1024 }, { 7, 884, 1024 }, { 8, 885, 1024 }, { 8, 886, 1024 }, { 9, 887, 1024 },
   { 7, 888, 1024 }, { 8, 889, 1024 }, { 8, 890, 1024 }, { 9, 891, 1024 }, { 8, 892, 1024 }, { 9, 893, 1024 }, { 9, 894, 1024 }, { 10, 895, 1024 },
   { 4, 896, 1024 }, { 5, 897, 1024 }, { 5, 898, 1024 }, { 6, 899, 1024 }, { 5, 900, 1024 }, { 6, 901, 1024 }, { 6, 902, 1024 }, { 7, 903, 1024 },
   { 5, 904, 1024 }, { 6, 905, 1024 }, { 6, 906, 1024 }, { 7, 907, 1024 }, { 6, 908, 1024 }, { 7, 909, 1024 }, { 7, 910, 1024 }, { 8, 911, 1024 },
   { 5, 912, 1024 }, { 6, 913, 1024 }, { 6, 914, 1024 }, { 7, 915, 1024 }, { 6, 916, 1024 }, { 7, 917, 1024 }, { 7, 918, 1024 }, { 8, 919, 1024 },
   { 6, 920, 1024 }, { 7, 921, 1024 }, { 7, 922, 1024 }, { 8, 923, 1024 }, { 7, 924, 1024 }, { 8, 925, 1024 }, { 8, 926, 1024 }, { 9, 927, 1024 },
   { 5, 928, 1024 }, { 6, 929, 1024 }, { 6, 930, 1024 }, { 7, 931, 1024 }, { 6, 932, 1024 }, { 7, 933, 1024 }, { 7, 934, 1024 }, { 8, 935, 1024 },
   { 6, 936, 1024 }, { 7, 937, 1024 }, { 7, 938, 1024 }, { 8, 939, 1024 }, { 7, 940, 1024 }, { 8, 941, 1024 }, { 8, 942, 1024 }, { 9, 943, 1024 },
   { 6, 944, 1024 }, { 7, 945, 1024 }, { 7, 946, 1024 }, { 8, 947, 1024 }, { 7, 948, 1024 }, { 8, 949, 1024 }, { 8, 950, 1024 }, { 9, 951, 1024 },
   { 7, 952, 1024 }, { 8, 953, 1024 }, { 8, 954, 1024 }, { 9, 955, 1024 }, { 8, 956, 1024 }, { 9, 957, 1024 }, { 9, 958, 1024 }, { 10, 959, 1024 },
   { 5, 960, 1024 }, { 6, 961, 1024 }, { 6, 962, 1024 }, { 7, 963, 1024 }, { 6, 964, 1024 }, { 7, 965, 1024 }, { 7, 966, 1024 }, { 8, 967, 1024 },
   { 6, 968, 1024 }, { 7, 969, 1024 }, { 7, 970, 1024 }, { 8, 971, 1024 }, { 7, 972, 1024 }, { 8, 973, 1024 }, { 8, 974, 1024 }, { 9, 975, 1024 },
   { 6, 976, 1024 }, { 7, 977, 1024 }, { 7, 978, 1024 }, { 8, 979, 1024 }, { 7, 980, 1024 }, { 8, 981, 1024 }, { 8, 982, 1024 }, { 9, 983, 1024 },
   { 7, 984, 1024 }, { 8, 985, 1024 }, { 8, 986, 1024 }, { 9, 987, 1024 }, { 8, 988, 1024 }, { 9, 989, 1024 }, { 9, 990, 1024 }, { 10, 991, 1024 },
   { 6, 992, 1024 }, { 7, 993, 1024 }, { 7, 994, 1024 }, { 8, 995, 1024 }, { 7, 996, 1024 }, { 8, 997, 1024 }, { 8, 998, 1024 }, { 9, 999, 1024 },
   { 7, 1000, 1024 }, { 8, 1001, 1024 }, { 8, 1002, 1024 }, { 9, 1003, 1024 }, { 8, 1004, 1024 }, { 9, 1005, 1024 }, { 9, 1006, 1024 }, { 10, 1007, 1024 },
   { 7, 1008, 1024 }, { 8, 1009, 1024 }, { 8, 1010, 1024 }, { 9, 1011, 1024 }, { 8, 1012, 1024 }, { 9, 1013, 1024 }, { 9, 1014, 1024 }, { 10, 1015, 1024 },
   { 8, 1016, 1024 }, { 9, 1017, 1024 }, { 9, 1018, 1024 }, { 10, 1019, 1024 }, { 9, 1020, 1024 }, { 10, 1021, 1024 }, { 10, 1022, 1024 }, { 11, 1023, 1024 },
#if FP_LUT > 11
   { 1, 0, 0 }, { 2, 1, 2048 }, { 2, 2, 2048 }, { 3, 3, 2048 }, { 2, 4, 2048 }, { 3, 5, 2048 }, { 3, 6, 2048 }, { 4, 7, 2048 },
   { 2, 8, 2048 }, { 3, 9, 2048 }, { 3, 10, 2048 }, { 4, 11, 2048 }, { 3, 12, 2048 }, { 4, 13, 2048 }, { 4, 14, 2048 }, { 5, 15, 2048 },
   { 2, 16, 2048 }, { 3, 17, 2048 }, { 3, 18, 2048 }, { 4, 19, 2048 }, { 3, 20, 2048 }, { 4, 21, 2048 }, { 4, 22, 2048 }, { 5, 23, 2048 },
   { 3, 24, 2048 }, { 4, 25, 2048 }, { 4, 26, 2048 }, { 5, 27, 2048 }, { 4, 28, 2048 }, { 5, 29, 2048 }, { 5, 30, 2048 }, { 6, 31, 2048 },
   { 2, 32, 2048 }, { 3, 33, 2048 }, { 3, 34, 2048 }, { 4, 35, 2048 }, { 3, 36, 2048 }, { 4, 37, 2048 }, { 4, 38, 2048 }, { 5, 39, 2048 },
   { 3, 40, 2048 }, { 4, 41, 2048 }, { 4, 42, 2048 }, { 5, 43, 2048 }, { 4, 44, 2048 }, { 5, 45, 2048 }, { 5, 46, 2048 }, { 6, 47, 2048 },
   { 3, 48, 2048 }, { 4, 49, 2048 }, { 4, 50, 2048 }, { 5, 51, 2048 }, { 4, 52, 2048 }, { 5, 53, 2048 }, { 5, 54, 2048 }, { 6, 55, 2048 },
   { 4, 56, 2048 }, { 5, 57, 2048 }, { 5, 58, 2048 }, { 6, 59, 2048 }, { 5, 60, 2048 }, { 6, 61, 2048 }, { 6, 62, 2048 }, { 7, 63, 2048 },
   { 2, 64, 2048 }, { 3, 65, 2048 }, { 3, 66, 2048 }, { 4, 67, 2048 }, { 3, 68, 2048 }, { 4, 69, 2048 }, { 4, 70, 2048 }, { 5, 71, 2048 },
   { 3, 72, 2048 }, { 4, 73, 2048 }, { 4, 74, 2048 }, { 5, 75, 2048 }, { 4, 76, 2048 }, { 5, 77, 2048 }, { 5, 78, 2048 }, { 6, 79, 2048 },
   { 3, 80, 2048 }, { 4, 81, 2048 }, { 4, 82, 2048 }, { 5, 83, 2048 }, { 4, 84, 2048 }, { 5, 85, 2048 }, { 5, 86, 2048 }, { 6, 87, 2048 },
   { 4, 88, 2048 }, { 5, 89, 2048 }, { 5, 90, 2048 }, { 6, 91, 2048 }, { 5, 92, 2048 }, { 6, 93, 2048 }, { 6, 94, 2048 }, { 7, 95, 2048 },
   { 3, 96, 2048 }, { 4, 97, 2048 }, { 4, 98, 2048 }, { 5, 99, 2048 }, { 4, 100, 2048 }, { 5, 101, 2048 }, { 5, 102, 2048 }, { 6, 103, 2048 },
   { 4, 104, 2048 }, { 5, 105, 2048 }, { 5, 106, 2048 }, { 6, 107, 2048 }, { 5, 108, 2048 }, { 6, 109, 2048 }, { 6, 110, 2048 }, { 7, 111, 2048 },
   { 4, 112, 2048 }, { 5, 113, 2048 }, { 5, 114, 2048 }, { 6, 115, 2048 }, { 5, 116, 2048 }, { 6, 117, 2048 }, { 6, 118, 2048 }, { 7, 119, 2048 },
   { 5, 120, 2048 }, { 6, 121, 2048 }, { 6, 122, 2048 }, { 7, 123, 2048 }, { 6, 124, 2048 }, { 7, 125, 2048 }, { 7, 126, 2048 }, { 8, 127, 2048 },
   { 2, 128, 2048 }, { 3, 129, 2048 }, { 3, 130, 2048 }, { 4, 131, 2048 }, { 3, 132, 2048 }, { 4, 133, 2048 }, { 4, 134, 2048 }, { 5, 135, 2048 },
   { 3, 136, 2048 }, { 4, 137, 2048 }, { 4, 138, 2048 }, { 5, 139, 2048 }, { 4, 140, 2048 }, { 5, 141, 2048 }, { 5, 142, 2048 }, { 6, 143, 2048 },
   { 3, 144, 2048 }, { 4, 145, 2048 }, { 4, 146, 2048 }, { 5, 147, 2048 }, { 4, 148, 2048 }, { 5, 149, 2048 }, { 5, 150, 2048 }, { 6, 151, 2048 },
   { 4, 152, 2048 }, { 5, 153, 2048 }, { 5, 154, 2048 }, { 6, 155, 2048 }, { 5, 156, 2048 }, { 6, 157, 2048 }, { 6, 158, 2048 }, { 7, 159, 2048 },
   { 3, 160, 2048 }, { 4, 161, 2048 }, { 4, 162, 2048 }, { 5, 163, 2048 }, { 4, 164, 2048 }, { 5, 165, 2048 }, { 5, 166, 2048 }, { 6, 167, 2048 },
   { 4, 168, 2048 }, { 5, 169, 2048 }, { 5, 170, 2048 }, { 6, 171, 2048 }, { 5, 172, 2048 }, { 6, 173, 2048 }, { 6, 174, 2048 }, { 7, 175, 2048 },
   { 4, 176, 2048 }, { 5, 177, 2048 }, { 5, 178, 2048 }, { 6, 179, 2048 }, { 5, 180, 2048 }, { 6, 181, 2048 }, { 6, 182, 2048 }, { 7, 183, 2048 },
   { 5, 184, 2048 }, { 6, 185, 2048 }, { 6, 186, 2048 }, { 7, 187, 2048 }, { 6, 188, 2048 }, { 7, 189, 2048 }, { 7, 190, 2048 }, { 8, 191, 2048 },
   { 3, 192, 2048 }, { 4, 193, 2048 }, { 4, 194, 2048 }, { 5, 195, 2048 }, { 4, 196, 2048 }, { 5, 197, 2048 }, { 5, 198, 2048 }, { 6, 199, 2048 },
   { 4, 200, 2048 }, { 5, 201, 2048 }, { 5, 202, 2048 }, { 6, 203, 2048 }, { 5, 204, 2048 }, { 6, 205, 2048 }, { 6, 206, 2048 }, { 7, 207, 2048 },
   { 4, 208, 2048 }, { 5, 209, 2048 }, { 5, 210, 2048 }, { 6, 211, 2048 }, { 5, 212, 2048 }, { 6, 213, 2048 }, { 6, 214, 2048 }, { 7, 215, 2048 },
   { 5, 216, 2048 }, { 6, 217, 2048 }, { 6, 218, 2048 }, { 7, 219, 2048 }, { 6, 220, 2048 }, { 7, 221, 2048 }, { 7, 222, 2048 }, { 8, 223, 2048 },
   { 4, 224, 2048 }, { 5, 225, 2048 }, { 5, 226, 2048 }, { 6, 227, 2048 }, { 5, 228, 2048 }, { 6, 229, 2048 }, { 6, 230, 2048 }, { 7, 231, 2048 },
   { 5, 232, 2048 }, { 6, 233, 2048 }, { 6, 234, 2048 }, { 7, 235, 2048 }, { 6, 236, 2048 }, { 7, 237, 2048 }, { 7, 238, 2048 }, { 8, 239, 2048 },
   { 5, 240, 2048 }, { 6, 241, 2048 }, { 6, 242, 2048 }, { 7, 243, 2048 }, { 6, 244, 2048 }, { 7, 245, 2048 }, { 7, 246, 2048 }, { 8, 247, 2048 },
   { 6, 248, 2048 }, { 7, 249, 2048 }, { 7, 250, 2048 }, { 8, 251, 2048 }, { 7, 252, 2048 }, { 8, 253, 2048 }, { 8, 254, 2048 }, { 9, 255, 2048 },
   { 2, 256, 2048 }, { 3, 257, 2048 }, { 3, 258, 2048 }, { 4, 259, 2048 }, { 3, 260, 2048 }, { 4, 261, 2048 }, { 4, 262, 2048 }, { 5, 263, 2048 },
   { 3, 264, 2048 }, { 4, 265, 2048 }, { 4, 266, 2048 }, { 5, 267, 2048 }, { 4, 268, 2048 }, { 5, 269, 2048 }, { 5, 270, 2048 }, { 6, 271, 2048 },
   { 3, 272, 2048 }, { 4, 273, 2048 }, { 4, 274, 2048 }, { 5, 275, 2048 }, { 4, 276, 2048 }, { 5, 277, 2048 }, { 5, 278, 2048 }, { 6, 279, 2048 },
   { 4, 280, 2048 }, { 5, 281, 2048 }, { 5, 282, 2048 }, { 6, 283, 2048 }, { 5, 284, 2048 }, { 6, 285, 2048 }, { 6, 286, 2048 }, { 7, 287, 2048 },
   { 3, 288, 2048 }, { 4, 289, 2048 }, { 4, 290, 2048 }, { 5, 291, 2048 }, { 4, 292, 2048 }, { 5, 293, 2048 }, { 5, 294, 2048 }, { 6, 295, 2048 },
   { 4, 296, 2048 }, { 5, 297, 2048 }, { 5, 298, 2048 }, { 6, 299, 2048 }, { 5, 300, 2048 }, { 6, 301, 2048 }, { 6, 302, 2048 }, { 7, 303, 2048 },
   { 4, 304, 2048 }, { 5, 305, 2048 }, { 5, 306, 2048 }, { 6, 307, 2048 }, { 5, 308, 2048 }, { 6, 309, 2048 }, { 6, 310, 2048 }, { 7, 311, 2048 },
   { 5, 312, 2048 }, { 6, 313, 2048 }, { 6, 314, 2048 }, { 7, 315, 2048 }, { 6, 316, 2048 }, { 7, 317, 2048 }, { 7, 318, 2048 }, { 8, 319, 2048 },
   { 3, 320, 2048 }, { 4, 321, 2048 }, { 4, 322, 2048 }, { 5, 323, 2048 }, { 4, 324, 2048 }, { 5, 325, 2048 }, { 5, 326, 2048 }, { 6, 327, 2048 },
   { 4, 328, 2048 }, { 5, 329, 2048 }, { 5, 330, 2048 }, { 6, 331, 2048 }, { 5, 332, 2048 }, { 6, 333, 2048 }, { 6, 334, 2048 }, { 7, 335, 2048 },
   { 4, 336, 2048 }, { 5, 337, 2048 }, { 5, 338, 2048 }, { 6, 339, 2048 }, { 5, 340, 2048 }, { 6, 341, 2048 }, { 6, 342, 2048 }, { 7, 343, 2048 },
   { 5, 344, 2048 }, { 6, 345, 2048 }, { 6, 346, 2048 }, { 7, 347, 2048 }, { 6, 348, 2048 }, { 7, 349, 2048 }, { 7, 350, 2048 }, { 8, 351, 2048 },
   { 4, 352, 2048 }, { 5, 353, 2048 }, { 5, 354, 2048 }, { 6, 355, 2048 }, { 5, 356, 2048 }, { 6, 357, 2048 }, { 6, 358, 2048 }, { 7, 359, 2048 },
   { 5, 360, 2048 }, { 6, 361, 2048 }, { 6, 362, 2048 }, { 7, 363, 2048 }, { 6, 364, 2048 }, { 7, 365, 2048 }, { 7, 366, 2048 }, { 8, 367, 2048 },
   { 5, 368, 2048 }, { 6, 369, 2048 }, { 6, 370, 2048 }, { 7, 371, 2048 }, { 6, 372, 2048 }, { 7, 373, 2048 }, { 7, 374, 2048 }, { 8, 375, 2048 },
   { 6, 376, 2048 }, { 7, 377, 2048 }, { 7, 378, 2048 }, { 8, 379, 2048 }, { 7, 380, 2048 }, { 8, 381, 2048 }, { 8, 382, 2048 }, { 9, 383, 2048 },
   { 3, 384, 2048 }, { 4, 385, 2048 }, { 4, 386, 2048 }, { 5, 387, 2048 }, { 4, 388, 2048 }, { 5, 389, 2048 }, { 5, 390, 2048 }, { 6, 391, 2048 },
   { 4, 392, 2048 }, { 5, 393, 2048 }, { 5, 394, 2048 }, { 6, 395, 2048 }, { 5, 396, 2048 }, { 6, 397, 2048 }, { 6, 398, 2048 }, { 7, 399, 2048 },
   { 4, 400, 2048 }, { 5, 401, 2048 }, { 5, 402, 2048 }, { 6, 403, 2048 }, { 5, 404, 2048 }, { 6, 405, 2048 }, { 6, 406, 2048 }, { 7, 407, 2048 },
   { 5, 408, 2048 }, { 6, 409, 2048 }, { 6, 410, 2048 }, { 7, 411, 2048 }, { 6, 412, 2048 }, { 7, 413, 2048 }, { 7, 414, 2048 }, { 8, 415, 2048 },
   { 4, 416, 2048 }, { 5, 417, 2048 }, { 5, 418, 2048 }, { 6, 419, 2048 }, { 5, 420, 2048 }, { 6, 421, 2048 }, { 6, 422, 2048 }, { 7, 423, 2048 },
   { 5, 424, 2048 }, { 6, 425, 2048 }, { 6, 426, 2048 }, { 7, 427, 2048 }, { 6, 428, 2048 }, { 7, 429, 2048 }, { 7, 430, 2048 }, { 8, 431, 2048 },
   { 5, 432, 2048 }, { 6, 433, 2048 }, { 6, 434, 2048 }, { 7, 435, 2048 }, { 6, 436, 2048 }, { 7, 437, 2048 }, { 7, 438, 2048 }, { 8, 439, 2048 },
   { 6, 440, 2048 }, { 7, 441, 2048 }, { 7, 442, 2048 }, { 8, 443, 2048 }, { 7, 444, 2048 }, { 8, 445, 2048 }, { 8, 446, 2048 }, { 9, 447, 2048 },
   { 4, 448, 2048 }, { 5, 449, 2048 }, { 5, 450, 2048 }, { 6, 451, 2048 }, { 5, 452, 2048 }, { 6, 453, 2048 }, { 6, 454, 2048 }, { 7, 455, 2048 },
   { 5, 456, 2048 }, { 6, 457, 2048 }, { 6, 458, 2048 }, { 7, 459, 2048 }, { 6, 460, 2048 }, { 7, 461, 2048 }, { 7, 462, 2048 }, { 8, 463, 2048 },
   { 5, 464, 2048 }, { 6, 465, 2048 }, { 6, 466, 2048 }, { 7, 467, 2048 }, { 6, 468, 2048 }, { 7, 469, 2048 }, { 7, 470, 2048 }, { 8, 471, 2048 },
   { 6, 472, 2048 }, { 7, 473, 2048 }, { 7, 474, 2048 }, { 8, 475, 2048 }, { 7, 476, 2048 }, { 8, 477, 2048 }, { 8, 478, 2048 }, { 9, 479, 2048 },
   { 5, 480, 2048 }, { 6, 481, 2048 }, { 6, 482, 2048 }, { 7, 483, 2048 }, { 6, 484, 2048 }, { 7, 485, 2048 }, { 7, 486, 2048 }, { 8, 487, 2048 },
   { 6, 488, 2048 }, { 7, 489, 2048 }, { 7, 490, 2048 }, { 8, 491, 2048 }, { 7, 492, 2048 }, { 8, 493, 2048 }, { 8, 494, 2048 }, { 9, 495, 2048 },
   { 6, 496, 2048 }, { 7, 497, 2048 }, { 7, 498, 2048 }, { 8, 499, 2048 }, { 7, 500, 2048 }, { 8, 501, 2048 }, { 8, 502, 2048 }, { 9, 503, 2048 },
   { 7, 504, 2048 }, { 8, 505, 2048 }, { 8, 506, 2048 }, { 9, 507, 2048 }, { 8, 508, 2048 }, { 9, 509, 2048 }, { 9, 510, 2048 }, { 10, 511, 2048 },
   { 2, 512, 2048 }, { 3, 513, 2048 }, { 3, 514, 2048 }, { 4, 515, 2048 }, { 3, 516, 2048 }, { 4, 517, 2048 }, { 4, 518, 2048 }, { 5, 519, 2048 },
   { 3, 520, 2048 }, { 4, 521, 2048 }, { 4, 522, 2048 }, { 5, 523, 2048 }, { 4, 524, 2048 }, { 5, 525, 2048 }, { 5, 526, 2048 }, { 6, 527, 2048 },
   { 3, 528, 2048 }, { 4, 529, 2048 }, { 4, 530, 2048 }, { 5, 531, 2048 }, { 4, 532, 2048 }, { 5, 533, 2048 }, { 5, 534, 2048 }, { 6, 535, 2048 },
   { 4, 536, 2048 }, { 5, 537, 2048 }, { 5, 538, 2048 }, { 6, 539, 2048 }, { 5, 540, 2048 }, { 6, 541, 2048 }, { 6, 542, 2048 }, { 7, 543, 2048 },
   { 3, 544, 2048 }, { 4, 545, 2048 }, { 4, 546, 2048 }, { 5, 547, 2048 }, { 4, 548, 2048 }, { 5, 549, 2048 }, { 5, 550, 2048 }, { 6, 551, 2048 },
   { 4, 552, 2048 }, { 5, 553, 2048 }, { 5, 554, 2048 }, { 6, 555, 2048 }, { 5, 556, 2048 }, { 6, 557, 2048 }, { 6, 558, 2048 }, { 7, 559, 2048 },
   { 4, 560, 2048 }, { 5, 561, 2048 }, { 5, 562, 2048 }, { 6, 563, 2048 }, { 5, 564, 2048 }, { 6, 565, 2048 }, { 6, 566, 2048 }, { 7, 567, 2048 },
   { 5, 568, 2048 }, { 6, 569, 2048 }, { 6, 570, 2048 }, { 7, 571, 2048 }, { 6, 572, 2048 }, { 7, 573, 2048 }, { 7, 574, 2048 }, { 8, 575, 2048 },
   { 3, 576, 2048 }, { 4, 577, 2048 }, { 4, 578, 2048 }, { 5, 579, 2048 }, { 4, 580, 2048 }, { 5, 581, 2048 }, { 5, 582, 2048 }, { 6, 583, 2048 },
   { 4, 584, 2048 }, { 5, 585, 2048 }, { 5, 586, 2048 }, { 6, 587, 2048 }, { 5, 588, 2048 }, { 6, 589, 2048 }, { 6, 590, 2048 }, { 7, 591, 2048 },
   { 4, 592, 2048 }, { 5, 593, 2048 }, { 5, 594, 2048 }, { 6, 595, 2048 }, { 5, 596, 2048 }, { 6, 597, 2048 }, { 6, 598, 2048 }, { 7, 599, 2048 },
   { 5, 600, 2048 }, { 6, 601, 2048 }, { 6, 602, 2048 }, { 7, 603, 2048 }, { 6, 604, 2048 }, { 7, 605, 2048 }, { 7, 606, 2048 }, { 8, 607, 2048 },
   { 4, 608, 2048 }, { 5, 609, 2048 }, { 5, 610, 2048 }, { 6, 611, 2048 }, { 5, 612, 2048 }, { 6, 613, 2048 }, { 6, 614, 2048 }, { 7, 615, 2048 },
   { 5, 616, 2048 }, { 6, 617, 2048 }, { 6, 618, 2048 }, { 7, 619, 2048 }, { 6, 620, 2048 }, { 7, 621, 2048 }, { 7, 622, 2048 }, { 8, 623, 2048 },
   { 5, 624, 2048 }, { 6, 625, 2048 }, { 6, 626, 2048 }, { 7, 627, 2048 }, { 6, 628, 2048 }, { 7, 629, 2048 }, { 7, 630, 2048 }, { 8, 631, 2048 },
   { 6, 632, 2048 }, { 7, 633, 2048 }, { 7, 634, 2048 }, { 8, 635, 2048 }, { 7, 636, 2048 }, { 8, 637, 2048 }, { 8, 638, 2048 }, { 9, 639, 2048 },
   { 3, 640, 2048 }, { 4, 641, 2048 }, { 4, 642, 2048 }, { 5, 643, 2048 }, { 4, 644, 2048 }, { 5, 645, 2048 }, { 5, 646, 2048 }, { 6, 647, 2048 },
   { 4, 648, 2048 }, { 5, 649, 2048 }, { 5, 650, 2048 }, { 6, 651, 2048 }, { 5, 652, 2048 }, { 6, 653, 2048 }, { 6, 654, 2048 }, { 7, 655, 2048 },
   { 4, 656, 2048 }, { 5, 657, 2048 }, { 5, 658, 2048 }, { 6, 659, 2048 }, { 5, 660, 2048 }, { 6, 661, 2048 }, { 6, 662, 2048 }, { 7, 663, 2048 },
   { 5, 664, 2048 }, { 6, 665, 2048 }, { 6, 666, 2048 }, { 7, 667, 2048 }, { 6, 668, 2048 }, { 7, 669, 2048 }, { 7, 670, 2048 }, { 8, 671, 2048 },
   { 4, 672, 2048 }, { 5, 673, 2048 }, { 5, 674, 2048 }, { 6, 675, 2048 }, { 5, 676, 2048 }, { 6, 677, 2048 }, { 6, 678, 2048 }, { 7, 679, 2048 },
   { 5, 680, 2048 }, { 6, 681, 2048 }, { 6, 682, 2048 }, { 7, 683, 2048 }, { 6, 684, 2048 }, { 7, 685, 2048 }, { 7, 686, 2048 }, { 8, 687, 2048 },
   { 5, 688, 2048 }, { 6, 689, 2048 }, { 6, 690, 2048 }, { 7, 691, 2048 }, { 6, 692, 2048 }, { 7, 693, 2048 }, { 7, 694, 2048 }, { 8, 695, 2048 },
   { 6, 696, 2048 }, { 7, 697, 2048 }, { 7, 698, 2048 }, { 8, 699, 2048 }, { 7, 700, 2048 }, { 8, 701, 2048 }, { 8, 702, 2048 }, { 9, 703, 2048 },
   { 4, 704, 2048 }, { 5, 705, 2048 }, { 5, 706, 2048 }, { 6, 707, 2048 }, { 5, 708, 2048 }, { 6, 709, 2048 }, { 6, 710, 2048 }, { 7, 711, 2048 },
   { 5, 712, 2048 }, { 6, 713, 2048 }, { 6, 714, 2048 }, { 7, 715, 2048 }, { 6, 716, 2048 }, { 7, 717, 2048 }, { 7, 718, 2048 }, { 8, 719, 2048 },
   { 5, 720, 2048 }, { 6, 721, 2048 }, { 6, 722, 2048 }, { 7, 723, 2048 }, { 6, 724, 2048 }, { 7, 725, 2048 }, { 7, 726, 2048 }, { 8, 727, 2048 },
   { 6, 728, 2048 }, { 7, 729, 2048 }, { 7, 730, 2048 }, { 8, 731, 2048 }, { 7, 732, 2048 }, { 8, 733, 2048 }, { 8, 734, 2048 }, { 9, 735, 2048 },
   { 5, 736, 2048 }, { 6, 737, 2048 }, { 6, 738, 2048 }, { 7, 739, 2048 }, { 6, 740, 2048 }, { 7, 741, 2048 }, { 7, 742, 2048 }, { 8, 743, 2048 },
   { 6, 744, 2048 }, { 7, 745, 2048 }, { 7, 746, 2048 }, { 8, 747, 2048 }, { 7, 748, 2048 }, { 8, 749, 2048 }, { 8, 750, 2048 }, { 9, 751, 2048 },
   { 6, 752, 2048 }, { 7, 753, 2048 }, { 7, 754, 2048 }, { 8, 755, 2048 }, { 7, 756, 2048 }, { 8, 757, 2048 }, { 8, 758, 2048 }, { 9, 759, 2048 },
   { 7, 760, 2048 }, { 8, 761, 2048 }, { 8, 762, 2048 }, { 9, 763, 2048 }, { 8, 764, 2048 }, { 9, 765, 2048 }, { 9, 766, 2048 }, { 10, 767, 2048 },
   { 3, 768, 2048 }, { 4, 769, 2048 }, { 4, 770, 2048 }, { 5, 771, 2048 }, { 4, 772, 2048 }, { 5, 773, 2048 }, { 5, 774, 2048 }, { 6, 775, 2048 },
   { 4, 776, 2048 }, { 5, 777, 2048 }, { 5, 778, 2048 }, { 6, 779, 2048 }, { 5, 780, 2048 }, { 6, 781, 2048 }, { 6, 782, 2048 }, { 7, 783, 2048 },
   { 4, 784, 2048 }, { 5, 785, 2048 }, { 5, 786, 2048 }, { 6, 787, 2048 }, { 5, 788, 2048 }, { 6, 789, 2048 }, { 6, 790, 2048 }, { 7, 791, 2048 },
   { 5, 792, 2048 }, { 6, 793, 2048 }, { 6, 794, 2048 }, { 7, 795, 2048 }, { 6, 796, 2048 }, { 7, 797, 2048 }, { 7, 798, 2048 }, { 8, 799, 2048 },
   { 4, 800, 2048 }, { 5, 801, 2048 }, { 5, 802, 2048 }, { 6, 803, 2048 }, { 5, 804, 2048 }, { 6, 805, 2048 }, { 6, 806, 2048 }, { 7, 807, 2048 },
   { 5, 808, 2048 }, { 6, 809, 2048 }, { 6, 810, 2048 }, { 7, 811, 2048 }, { 6, 812, 2048 }, { 7, 813, 2048 }, { 7, 814, 2048 }, { 8, 815, 2048 },
   { 5, 816, 2048 }, { 6, 817, 2048 }, { 6, 818, 2048 }, { 7, 819, 2048 }, { 6, 820, 2048 }, { 7, 821, 2048 }, { 7, 822, 2048 }, { 8, 823, 2048 },
   { 6, 824, 2048 }, { 7, 825, 2048 }, { 7, 826, 2048 }, { 8, 827, 2048 }, { 7, 828, 2048 }, { 8, 829, 2048 }, { 8, 830, 2048 }, { 9, 831, 2048 },
   { 4, 832, 2048 }, { 5, 833, 2048 }, { 5, 834, 2048 }, { 6, 835, 2048 }, { 5, 836, 2048 }, { 6, 837, 2048 }, { 6, 838, 2048 }, { 7, 839, 2048 },
   { 5, 840, 2048 }, { 6, 841, 2048 }, { 6, 842, 2048 }, { 7, 843, 2048 }, { 6, 844, 2048 }, { 7, 845, 2048 }, { 7, 846, 2048 }, { 8, 847, 2048 },
   { 5, 848, 2048 }, { 6, 849, 2048 }, { 6, 850, 2048 }, { 7, 851, 2048 }, { 6, 852, 2048 }, { 7, 853, 2048 }, { 7, 854, 2048 }, { 8, 855, 2048 },
   { 6, 856, 2048 }, { 7, 857, 2048 }, { 7, 858, 2048 }, { 8, 859, 2048 }, { 7, 860, 2048 }, { 8, 861, 2048 }, { 8, 862, 2048 }, { 9, 863, 2048 },
   { 5, 864, 2048 }, { 6, 865, 2048 }, { 6, 866, 2048 }, { 7, 867, 2048 }, { 6, 868, 2048 }, { 7, 869, 2048 }, { 7, 870, 2048 }, { 8, 871, 2048 },
   { 6, 872, 2048 }, { 7, 873, 2048 }, { 7, 874, 2048 }, { 8, 875, 2048 }, { 7, 876, 2048 }, { 8, 877, 2048 }, { 8, 878, 2048 }, { 9, 879, 2048 },
   { 6, 880, 2048 }, { 7, 881, 2048 }, { 7, 882, 2048 }, { 8, 883, 2048 }, { 7, 884, 2048 }, { 8, 885, 2048 }, { 8, 886, 2048 }, { 9, 887, 2048 },
   { 7, 888, 2048 }, { 8, 889, 2048 }, { 8, 890, 2048 }, { 9, 891, 2048 }, { 8, 892, 2048 }, { 9, 893, 2048 }, { 9, 894, 2048 }, { 10, 895, 2048 },
   { 4, 896, 2048 }, { 5, 897, 2048 }, { 5, 898, 2048 }, { 6, 899, 2048 }, { 5, 900, 2048 }, { 6, 901, 2048 }, { 6, 902, 2048 }, { 7, 903, 2048 },
   { 5, 904, 2048 }, { 6, 905, 2048 }, { 6, 906, 2048 }, { 7, 907, 2048 }, { 6, 908, 2048 }, { 7, 909, 2048 }, { 7, 910, 2048 }, { 8, 911, 2048 },
   { 5, 912, 2048 }, { 6, 913, 2048 }, { 6, 914, 2048 }, { 7, 915, 2048 }, { 6, 916, 2048 }, { 7, 917, 2048 }, { 7, 918, 2048 }, { 8, 919, 2048 },
   { 6, 920, 2048 }, { 7, 921, 2048 }, { 7, 922, 2048 }, { 8, 923, 2048 }, { 7, 924, 2048 }, { 8, 925, 2048 }, { 8, 926, 2048 }, { 9, 927, 2048 },
   { 5, 928, 2048 }, { 6, 929, 2048 }, { 6, 930, 2048 }, { 7, 931, 2048 }, { 6, 932, 2048 }, { 7, 933, 2048 }, { 7, 934, 2048 }, { 8, 935, 2048 },
   { 6, 936, 2048 }, { 7, 937, 2048 }, { 7, 938, 2048 }, { 8, 939, 2048 }, { 7, 940, 2048 }, { 8, 941, 2048 }, { 8, 942, 2048 }, { 9, 943, 2048 },
   { 6, 944, 2048 }, { 7, 945, 2048 }, { 7, 946, 2048 }, { 8, 947, 2048 }, { 7, 948, 2048 }, { 8, 949, 2048 }, { 8, 950, 2048 }, { 9, 951, 2048 },
   { 7, 952, 2048 }, { 8, 953, 2048 }, { 8, 954, 2048 }, { 9, 955, 2048 }, { 8, 956, 2048 }, { 9, 957, 2048 }, { 9, 958, 2048 }, { 10, 959, 2048 },
   { 5, 960, 2048 }, { 6, 961, 2048 }, { 6, 962, 2048 }, { 7, 963, 2048 }, { 6, 964, 2048 }, { 7, 965, 2048 }, { 7, 966, 2048 }, { 8, 967, 2048 },
   { 6, 968, 2048 }, { 7, 969, 2048 }, { 7, 970, 2048 }, { 8, 971, 2048 }, { 7, 972, 2048 }, { 8, 973, 2048 }, { 8, 974, 2048 }, { 9, 975, 2048 },
   { 6, 976, 2048 }, { 7, 977, 2048 }, { 7, 978, 2048 }, { 8, 979, 2048 }, { 7, 980, 2048 }, { 8, 981, 2048 }, { 8, 982, 2048 }, { 9, 983, 2048 },
   { 7, 984, 2048 }, { 8, 985, 2048 }, { 8, 986, 2048 }, { 9, 987, 2048 }, { 8, 988, 2048 }, { 9, 989, 2048 }, { 9, 990, 2048 }, { 10, 991, 2048 },
   { 6, 992, 2048 }, { 7, 993, 2048 }, { 7, 994, 2048 }, { 8, 995, 2048 }, { 7, 996, 2048 }, { 8, 997, 2048 }, { 8, 998, 2048 }, { 9, 999, 2048 },
   { 7, 1000, 2048 }, { 8, 1001, 2048 }, { 8, 1002, 2048 }, { 9, 1003, 2048 }, { 8, 1004, 2048 }, { 9, 1005, 2048 }, { 9, 1006, 2048 }, { 10, 1007, 2048 },
   { 7, 1008, 2048 }, { 8, 1009, 2048 }, { 8, 1010, 2048 }, { 9, 1011, 2048 }, { 8, 1012, 2048 }, { 9, 1013, 2048 }, { 9, 1014, 2048 }, { 10, 1015, 2048 },
   { 8, 1016, 2048 }, { 9, 1017, 2048 }, { 9, 1018, 2048 }, { 10, 1019, 2048 }, { 9, 1020, 2048 }, { 10, 1021, 2048 }, { 10, 1022, 2048 }, { 11, 1023, 2048 },
   { 2, 1024, 2048 }, { 3, 1025, 2048 }, { 3, 1026, 2048 }, { 4, 1027, 2048 }, { 3, 1028, 2048 }, { 4, 1029, 2048 }, { 4, 1030, 2048 }, { 5, 1031, 2048 },
   { 3, 1032, 2048 }, { 4, 1033, 2048 }, { 4, 1034, 2048 }, { 5, 1035, 2048 }, { 4, 1036, 2048 }, { 5, 1037, 2048 }, { 5, 1038, 2048 }, { 6, 1039, 2048 },
   { 3, 1040, 2048 }, { 4, 1041, 2048 }, { 4, 1042, 2048 }, { 5, 1043, 2048 }, { 4, 1044, 2048 }, { 5, 1045, 2048 }, { 5, 1046, 2048 }, { 6, 1047, 2048 },
   { 4, 1048, 2048 }, { 5, 1049, 2048 }, { 5, 1050, 2048 }, { 6, 1051, 2048 }, { 5, 1052, 2048 }, { 6, 1053, 2048 }, { 6, 1054, 2048 }, { 7, 1055, 2048 },
   { 3, 1056, 2048 }, { 4, 1057, 2048 }, { 4, 1058, 2048 }, { 5, 1059, 2048 }, { 4, 1060, 2048 }, { 5, 1061, 2048 }, { 5, 1062, 2048 }, { 6, 1063, 2048 },
   { 4, 1064, 2048 }, { 5, 1065, 2048 }, { 5, 1066, 2048 }, { 6, 1067, 2048 }, { 5, 1068, 2048 }, { 6, 1069, 2048 }, { 6, 1070, 2048 }, { 7, 1071, 2048 },
   { 4, 1072, 2048 }, { 5, 1073, 2048 }, { 5, 1074, 2048 }, { 6, 1075, 2048 }, { 5, 1076, 2048 }, { 6, 1077, 2048 }, { 6, 1078, 2048 }, { 7, 1079, 2048 },
   { 5, 1080, 2048 }, { 6, 1081, 2048 }, { 6, 1082, 2048 }, { 7, 1083, 2048 }, { 6, 1084, 2048 }, { 7, 1085, 2048 }, { 7, 1086, 2048 }, { 8, 1087, 2048 },
   { 3, 1088, 2048 }, { 4, 1089, 2048 }, { 4, 1090, 2048 }, { 5, 1091, 2048 }, { 4, 1092, 2048 }, { 5, 1093, 2048 }, { 5, 1094, 2048 }, { 6, 1095, 2048 },
   { 4, 1096, 2048 }, { 5, 1097, 2048 }, { 5, 1098, 2048 }, { 6, 1099, 2048 }, { 5, 1100, 2048 }, { 6, 1101, 2048 }, { 6, 1102, 2048 }, { 7, 1103, 2048 },
   { 4, 1104, 2048 }, { 5, 1105, 2048 }, { 5, 1106, 2048 }, { 6, 1107, 2048 }, { 5, 1108, 2048 }, { 6, 1109, 2048 }, { 6, 1110, 2048 }, { 7, 1111, 2048 },
   { 5, 1112, 2048 }, { 6, 1113, 2048 }, { 6, 1114, 2048 }, { 7, 1115, 2048 }, { 6, 1116, 2048 }, { 7, 1117, 2048 }, { 7, 1118, 2048 }, { 8, 1119, 2048 },
   { 4, 1120, 2048 }, { 5, 1121, 2048 }, { 5, 1122, 2048 }, { 6, 1123, 2048 }, { 5, 1124, 2048 }, { 6, 1125, 2048 }, { 6, 1126, 2048 }, { 7, 1127, 2048 },
   { 5, 1128, 2048 }, { 6, 1129, 2048 }, { 6, 1130, 2048 }, { 7, 1131, 2048 }, { 6, 1132, 2048 }, { 7, 1133, 2048 }, { 7, 1134, 2048 }, { 8, 1135, 2048 },
   { 5, 1136, 2048 }, { 6, 1137, 2048 }, { 6, 1138, 2048 }, { 7, 1139, 2048 }, { 6, 1140, 2048 }, { 7, 1141, 2048 }, { 7, 1142, 2048 }, { 8, 1143, 2048 },
   { 6, 1144, 2048 }, { 7, 1145, 2048 }, { 7, 1146, 2048 }, { 8, 1147, 2048 }, { 7, 1148, 2048 }, { 8, 1149, 2048 }, { 8, 1150, 2048 }, { 9, 1151, 2048 },
   { 3, 1152, 2048 }, { 4, 1153, 2048 }, { 4, 1154, 2048 }, { 5, 1155, 2048 }, { 4, 1156, 2048 }, { 5, 1157, 2048 }, { 5, 1158, 2048 }, { 6, 1159, 2048 },
   { 4, 1160, 2048 }, { 5, 1161, 2048 }, { 5, 1162, 2048 }, { 6, 1163, 2048 }, { 5, 1164, 2048 }, { 6, 1165, 2048 }, { 6, 1166, 2048 }, { 7, 1167, 2048 },
   { 4, 1168, 2048 }, { 5, 1169, 2048 }, { 5, 1170, 2048 }, { 6, 1171, 2048 }, { 5, 1172, 2048 }, { 6, 1173, 2048 }, { 6, 1174, 2048 }, { 7, 1175, 2048 },
   { 5, 1176, 2048 }, { 6, 1177, 2048 }, { 6, 1178, 2048 }, { 7, 1179, 2048 }, { 6, 1180, 2048 }, { 7, 1181, 2048 }, { 7, 1182, 2048 }, { 8, 1183, 2048 },
   { 4, 1184, 2048 }, { 5, 1185, 2048 }, { 5, 1186, 2048 }, { 6, 1187, 2048 }, { 5, 1188, 2048 }, { 6, 1189, 2048 }, { 6, 1190, 2048 }, { 7, 1191, 2048 },
   { 5, 1192, 2048 }, { 6, 1193, 2048 }, { 6, 1194, 2048 }, { 7, 1195, 2048 }, { 6, 1196, 2048 }, { 7, 1197, 2048 }, { 7, 1198, 2048 }, { 8, 1199, 2048 },
   { 5, 1200, 2048 }, { 6, 1201, 2048 }, { 6, 1202, 2048 }, { 7, 1203, 2048 }, { 6, 1204, 2048 }, { 7, 1205, 2048 }, { 7, 1206, 2048 }, { 8, 1207, 2048 },
   { 6, 1208, 2048 }, { 7, 1209, 2048 }, { 7, 1210, 2048 }, { 8, 1211, 2048 }, { 7, 1212, 2048 }, { 8, 1213, 2048 }, { 8, 1214, 2048 }, { 9, 1215, 2048 },
   { 4, 1216, 2048 }, { 5, 1217, 2048 }, { 5, 1218, 2048 }, { 6, 1219, 2048 }, { 5, 1220, 2048 }, { 6, 1221, 2048 }, { 6, 1222, 2048 }, { 7, 1223, 2048 },
   { 5, 1224, 2048 }, { 6, 1225, 2048 }, { 6, 1226, 2048 }, { 7, 1227, 2048 }, { 6, 1228, 2048 }, { 7, 1229, 2048 }, { 7, 1230, 2048 }, { 8, 1231, 2048 },
   { 5, 1232, 2048 }, { 6, 1233, 2048 }, { 6, 1234, 2048 }, { 7, 1235, 2048 }, { 6, 1236, 2048 }, { 7, 1237, 2048 }, { 7, 1238, 2048 }, { 8, 1239, 2048 },
   { 6, 1240, 2048 }, { 7, 1241, 2048 }, { 7, 1242, 2048 }, { 8, 1243, 2048 }, { 7, 1244, 2048 }, { 8, 1245, 2048 }, { 8, 1246, 2048 }, { 9, 1247, 2048 },
   { 5, 1248, 2048 }, { 6, 1249, 2048 }, { 6, 1250, 2048 }, { 7, 1251, 2048 }, { 6, 1252, 2048 }, { 7, 1253, 2048 }, { 7, 1254, 2048 }, { 8, 1255, 2048 },
   { 6, 1256, 2048 }, { 7, 1257, 2048 }, { 7, 1258, 2048 }, { 8, 1259, 2048 }, { 7, 1260, 2048 }, { 8, 1261, 2048 }, { 8, 1262, 2048 }, { 9, 1263, 2048 },
   { 6, 1264, 2048 }, { 7, 1265, 2048 }, { 7, 1266, 2048 }, { 8, 1267, 2048 }, { 7, 1268, 2048 }, { 8, 1269, 2048 }, { 8, 1270, 2048 }, { 9, 1271, 2048 },
   { 7, 1272, 2048 }, { 8, 1273, 2048 }, { 8, 1274, 2048 }, { 9, 1275, 2048 }, { 8, 1276, 2048 }, { 9, 1277, 2048 }, { 9, 1278, 2048 }, { 10, 1279, 2048 },
   { 3, 1280, 2048 }, { 4, 1281, 2048 }, { 4, 1282, 2048 }, { 5, 1283, 2048 }, { 4, 1284, 2048 }, { 5, 1285, 2048 }, { 5, 1286, 2048 }, { 6, 1287, 2048 },
   { 4, 1288, 2048 }, { 5, 1289, 2048 }, { 5, 1290, 2048 }, { 6, 1291, 2048 }, { 5, 1292, 2048 }, { 6, 1293, 2048 }, { 6, 1294, 2048 }, { 7, 1295, 2048 },
   { 4, 1296, 2048 }, { 5, 1297, 2048 }, { 5, 1298, 2048 }, { 6, 1299, 2048 }, { 5, 1300, 2048 }, { 6, 1301, 2048 }, { 6, 1302, 2048 }, { 7, 1303, 2048 },
   { 5, 1304, 2048 }, { 6, 1305, 2048 }, { 6, 1306, 2048 }, { 7, 1307, 2048 }, { 6, 1308, 2048 }, { 7, 1309, 2048 }, { 7, 1310, 2048 }, { 8, 1311, 2048 },
   { 4, 1312, 2048 }, { 5, 1313, 2048 }, { 5, 1314, 2048 }, { 6, 1315, 2048 }, { 5, 1316, 2048 }, { 6, 1317, 2048 }, { 6, 1318, 2048 }, { 7, 1319, 2048 },
   { 5, 1320, 2048 }, { 6, 1321, 2048 }, { 6, 1322, 2048 }, { 7, 1323, 2048 }, { 6, 1324, 2048 }, { 7, 1325, 2048 }, { 7, 1326, 2048 }, { 8, 1327, 2048 },
   { 5, 1328, 2048 }, { 6, 1329, 2048 }, { 6, 1330, 2048 }, { 7, 1331, 2048 }, { 6, 1332, 2048 }, { 7, 1333, 2048 }, { 7, 1334, 2048 }, { 8, 1335, 2048 },
   { 6, 1336, 2048 }, { 7, 1337, 2048 }, { 7, 1338, 2048 }, { 8, 1339, 2048 }, { 7, 1340, 2048 }, { 8, 1341, 2048 }, { 8, 1342, 2048 }, { 9, 1343, 2048 },
   { 4, 1344, 2048 }, { 5, 1345, 2048 }, { 5, 1346, 2048 }, { 6, 1347, 2048 }, { 5, 1348, 2048 }, { 6, 1349, 2048 }, { 6, 1350, 2048 }, { 7, 1351, 2048 },
   { 5, 1352, 2048 }, { 6, 1353, 2048 }, { 6, 1354, 2048 }, { 7, 1355, 2048 }, { 6, 1356, 2048 }, { 7, 1357, 2048 }, { 7, 1358, 2048 }, { 8, 1359, 2048 },
   { 5, 1360, 2048 }, { 6, 1361, 2048 }, { 6, 1362, 2048 }, { 7, 1363, 2048 }, { 6, 1364, 2048 }, { 7, 1365, 2048 }, { 7, 1366, 2048 }, { 8, 1367, 2048 },
   { 6, 1368, 2048 }, { 7, 1369, 2048 }, { 7, 1370, 2048 }, { 8, 1371, 2048 }, { 7, 1372, 2048 }, { 8, 1373, 2048 }, { 8, 1374, 2048 }, { 9, 1375, 2048 },
   { 5, 1376, 2048 }, { 6, 1377, 2048 }, { 6, 1378, 2048 }, { 7, 1379, 2048 }, { 6, 1380, 2048 }, { 7, 1381, 2048 }, { 7, 1382, 2048 }, { 8, 1383, 2048 },
   { 6, 1384, 2048 }, { 7, 1385, 2048 }, { 7, 1386, 2048 }, { 8, 1387, 2048 }, { 7, 1388, 2048 }, { 8, 1389, 2048 }, { 8, 1390, 2048 }, { 9, 1391, 2048 },
   { 6, 1392, 2048 }, { 7, 1393, 2048 }, { 7, 1394, 2048 }, { 8, 1395, 2048 }, { 7, 1396, 2048 }, { 8, 1397, 2048 }, { 8, 1398, 2048 }, { 9, 1399, 2048 },
   { 7, 1400, 2048 }, { 8, 1401, 2048 }, { 8, 1402, 2048 }, { 9, 1403, 2048 }, { 8, 1404, 2048 }, { 9, 1405, 2048 }, { 9, 1406, 2048 }, { 10, 1407, 2048 },
   { 4, 1408, 2048 }, { 5, 1409, 2048 }, { 5, 1410, 2048 }, { 6, 1411, 2048 }, { 5, 1412, 2048 }, { 6, 1413, 2048 }, { 6, 1414, 2048 }, { 7, 1415, 2048 },
   { 5, 1416, 2048 }, { 6, 1417, 2048 }, { 6, 1418, 2048 }, { 7, 1419, 2048 }, { 6, 1420, 2048 }, { 7, 1421, 2048 }, { 7, 1422, 2048 }, { 8, 1423, 2048 },
   { 5, 1424, 2048 }, { 6, 1425, 2048 }, { 6, 1426, 2048 }, { 7, 1427, 2048 }, { 6, 1428, 2048 }, { 7, 1429, 2048 }, { 7, 1430, 2048 }, { 8, 1431, 2048 },
   { 6, 1432, 2048 }, { 7, 1433, 2048 }, { 7, 1434, 2048 }, { 8, 1435, 2048 }, { 7, 1436, 2048 }, { 8, 1437, 2048 }, { 8, 1438, 2048 }, { 9, 1439, 2048 },
   { 5, 1440, 2048 }, { 6, 1441, 2048 }, { 6, 1442, 2048 }, { 7, 1443, 2048 }, { 6, 1444, 2048 }, { 7, 1445, 2048 }, { 7, 1446, 2048 }, { 8, 1447, 2048 },
   { 6, 1448, 2048 }, { 7, 1449, 2048 }, { 7, 1450, 2048 }, { 8, 1451, 2048 }, { 7, 1452, 2048 }, { 8, 1453, 2048 }, { 8, 1454, 2048 }, { 9, 1455, 2048 },
   { 6, 1456, 2048 }, { 7, 1457, 2048 }, { 7, 1458, 2048 }, { 8, 1459, 2048 }, { 7, 1460, 2048 }, { 8, 1461, 2048 }, { 8, 1462, 2048 }, { 9, 1463, 2048 },
   { 7, 1464, 2048 }, { 8, 1465, 2048 }, { 8, 1466, 2048 }, { 9, 1467, 2048 }, { 8, 1468, 2048 }, { 9, 1469, 2048 }, { 9, 1470, 2048 }, { 10, 1471, 2048 },
   { 5, 1472, 2048 }, { 6, 1473, 2048 }, { 6, 1474, 2048 }, { 7, 1475, 2048 }, { 6, 1476, 2048 }, { 7, 1477, 2048 }, { 7, 1478, 2048 }, { 8, 1479, 2048 },
   { 6, 1480, 2048 }, { 7, 1481, 2048 }, { 7, 1482, 2048 }, { 8, 1483, 2048 }, { 7, 1484, 2048 }, { 8, 1485, 2048 }, { 8, 1486, 2048 }, { 9, 1487, 2048 },
   { 6, 1488, 2048 }, { 7, 1489, 2048 }, { 7, 1490, 2048 }, { 8, 1491, 2048 }, { 7, 1492, 2048 }, { 8, 1493, 2048 }, { 8, 1494, 2048 }, { 9, 1495, 2048 },
   { 7, 1496, 2048 }, { 8, 1497, 2048 }, { 8, 1498, 2048 }, { 9, 1499, 2048 }, { 8, 1500, 2048 }, { 9, 1501, 2048 }, { 9, 1502, 2048 }, { 10, 1503, 2048 },
   { 6, 1504, 2048 }, { 7, 1505, 2048 }, { 7, 1506, 2048 }, { 8, 1507, 2048 }, { 7, 1508, 2048 }, { 8, 1509, 2048 }, { 8, 1510, 2048 }, { 9, 1511, 2048 },
   { 7, 1512, 2048 }, { 8, 1513, 2048 }, { 8, 1514, 2048 }, { 9, 1515, 2048 }, { 8, 1516, 2048 }, { 9, 1517, 2048 }, { 9, 1518, 2048 }, { 10, 1519, 2048 },
   { 7, 1520, 2048 }, { 8, 1521, 2048 }, { 8, 1522, 2048 }, { 9, 1523, 2048 }, { 8, 1524, 2048 }, { 9, 1525, 2048 }, { 9, 1526, 2048 }, { 10, 1527, 2048 },
   { 8, 1528, 2048 }, { 9, 1529, 2048 }, { 9, 1530, 2048 }, { 10, 1531, 2048 }, { 9, 1532, 2048 }, { 10, 1533, 2048 }, { 10, 1534, 2048 }, { 11, 1535, 2048 },
   { 3, 1536, 2048 }, { 4, 1537, 2048 }, { 4, 1538, 2048 }, { 5, 1539, 2048 }, { 4, 1540, 2048 }, { 5, 1541, 2048 }, { 5, 1542, 2048 }, { 6, 1543, 2048 },
   { 4, 1544, 2048 }, { 5, 1545, 2048 }, { 5, 1546, 2048 }, { 6, 1547, 2048 }, { 5, 1548, 2048 }, { 6, 1549, 2048 }, { 6, 1550, 2048 }, { 7, 1551, 2048 },
   { 4, 1552, 2048 }, { 5, 1553, 2048 }, { 5, 1554, 2048 }, { 6, 1555, 2048 }, { 5, 1556, 2048 }, { 6, 1557, 2048 }, { 6, 1558, 2048 }, { 7, 1559, 2048 },
   { 5, 1560, 2048 }, { 6, 1561, 2048 }, { 6, 1562, 2048 }, { 7, 1563, 2048 }, { 6, 1564, 2048 }, { 7, 1565, 2048 }, { 7, 1566, 2048 }, { 8, 1567, 2048 },
   { 4, 1568, 2048 }, { 5, 1569, 2048 }, { 5, 1570, 2048 }, { 6, 1571, 2048 }, { 5, 1572, 2048 }, { 6, 1573, 2048 }, { 6, 1574, 2048 }, { 7, 1575, 2048 },
   { 5, 1576, 2048 }, { 6, 1577, 2048 }, { 6, 1578, 2048 }, { 7, 1579, 2048 }, { 6, 1580, 2048 }, { 7, 1581, 2048 }, { 7, 1582, 2048 }, { 8, 1583, 2048 },
   { 5, 1584, 2048 }, { 6, 1585, 2048 }, { 6, 1586, 2048 }, { 7, 1587, 2048 }, { 6, 1588, 2048 }, { 7, 1589, 2048 }, { 7, 1590, 2048 }, { 8, 1591, 2048 },
   { 6, 1592, 2048 }, { 7, 1593, 2048 }, { 7, 1594, 2048 }, { 8, 1595, 2048 }, { 7, 1596, 2048 }, { 8, 1597, 2048 }, { 8, 1598, 2048 }, { 9, 1599, 2048 },
   { 4, 1600, 2048 }, { 5, 1601, 2048 }, { 5, 1602, 2048 }, { 6, 1603, 2048 }, { 5, 1604, 2048 }, { 6, 1605, 2048 }, { 6, 1606, 2048 }, { 7, 1607, 2048 },
   { 5, 1608, 2048 }, { 6, 1609, 2048 }, { 6, 1610, 2048 }, { 7, 1611, 2048 }, { 6, 1612, 2048 }, { 7, 1613, 2048 }, { 7, 1614, 2048 }, { 8, 1615, 2048 },
   { 5, 1616, 2048 }, { 6, 1617, 2048 }, { 6, 1618, 2048 }, { 7, 1619, 2048 }, { 6, 1620, 2048 }, { 7, 1621, 2048 }, { 7, 1622, 2048 }, { 8, 1623, 2048 },
   { 6, 1624, 2048 }, { 7, 1625, 2048 }, { 7, 1626, 2048 }, { 8, 1627, 2048 }, { 7, 1628, 2048 }, { 8, 1629, 2048 }, { 8, 1630, 2048 }, { 9, 1631, 2048 },
   { 5, 1632, 2048 }, { 6, 1633, 2048 }, { 6, 1634, 2048 }, { 7, 1635, 2048 }, { 6, 1636, 2048 }, { 7, 1637, 2048 }, { 7, 1638, 2048 }, { 8, 1639, 2048 },
   { 6, 1640, 2048 }, { 7, 1641, 2048 }, { 7, 1642, 2048 }, { 8, 1643, 2048 }, { 7, 1644, 2048 }, { 8, 1645, 2048 }, { 8, 1646, 2048 }, { 9, 1647, 2048 },
   { 6, 1648, 2048 }, { 7, 1649, 2048 }, { 7, 1650, 2048 }, { 8, 1651, 2048 }, { 7, 1652, 2048 }, { 8, 1653, 2048 }, { 8, 1654, 2048 }, { 9, 1655, 2048 },
   { 7, 1656, 2048 }, { 8, 1657, 2048 }, { 8, 1658, 2048 }, { 9, 1659, 2048 }, { 8, 1660, 2048 }, { 9, 1661, 2048 }, { 9, 1662, 2048 }, { 10, 1663, 2048 },
   { 4, 1664, 2048 }, { 5, 1665, 2048 }, { 5, 1666, 2048 }, { 6, 1667, 2048 }, { 5, 1668, 2048 }, { 6, 1669, 2048 }, { 6, 1670, 2048 }, { 7, 1671, 2048 },
   { 5, 1672, 2048 }, { 6, 1673, 2048 }, { 6, 1674, 2048 }, { 7, 1675, 2048 }, { 6, 1676, 2048 }, { 7, 1677, 2048 }, { 7, 1678, 2048 }, { 8, 1679, 2048 },
   { 5, 1680, 2048 }, { 6, 1681, 2048 }, { 6, 1682, 2048 }, { 7, 1683, 2048 }, { 6, 1684, 2048 }, { 7, 1685, 2048 }, { 7, 1686, 2048 }, { 8, 1687, 2048 },
   { 6, 1688, 2048 }, { 7, 1689, 2048 }, { 7, 1690, 2048 }, { 8, 1691, 2048 }, { 7, 1692, 2048 }, { 8, 1693, 2048 }, { 8, 1694, 2048 }, { 9, 1695, 2048 },
   { 5, 1696, 2048 }, { 6, 1697, 2048 }, { 6, 1698, 2048 }, { 7, 1699, 2048 }, { 6, 1700, 2048 }, { 7, 1701, 2048 }, { 7, 1702, 2048 }, { 8, 1703, 2048 },
   { 6, 1704, 2048 }, { 7, 1705, 2048 }, { 7, 1706, 2048 }, { 8, 1707, 2048 }, { 7, 1708, 2048 }, { 8, 1709, 2048 }, { 8, 1710, 2048 }, { 9, 1711, 2048 },
   { 6, 1712, 2048 }, { 7, 1713, 2048 }, { 7, 1714, 2048 }, { 8, 1715, 2048 }, { 7, 1716, 2048 }, { 8, 1717, 2048 }, { 8, 1718, 2048 }, { 9, 1719, 2048 },
   { 7, 1720, 2048 }, { 8, 1721, 2048 }, { 8, 1722, 2048 }, { 9, 1723, 2048 }, { 8, 1724, 2048 }, { 9, 1725, 2048 }, { 9, 1726, 2048 }, { 10, 1727, 2048 },
   { 5, 1728, 2048 }, { 6, 1729, 2048 }, { 6, 1730, 2048 }, { 7, 1731, 2048 }, { 6, 1732, 2048 }, { 7, 1733, 2048 }, { 7, 1734, 2048 }, { 8, 1735, 2048 },
   { 6, 1736, 2048 }, { 7, 1737, 2048 }, { 7, 1738, 2048 }, { 8, 1739, 2048 }, { 7, 1740, 2048 }, { 8, 1741, 2048 }, { 8, 1742, 2048 }, { 9, 1743, 2048 },
   { 6, 1744, 2048 }, { 7, 1745, 2048 }, { 7, 1746, 2048 }, { 8, 1747, 2048 }, { 7, 1748, 2048 }, { 8, 1749, 2048 }, { 8, 1750, 2048 }, { 9, 1751, 2048 },
   { 7, 1752, 2048 }, { 8, 1753, 2048 }, { 8, 1754, 2048 }, { 9, 1755, 2048 }, { 8, 1756, 2048 }, { 9, 1757, 2048 }, { 9, 1758, 2048 }, { 10, 1759, 2048 },
   { 6, 1760, 2048 }, { 7, 1761, 2048 }, { 7, 1762, 2048 }, { 8, 1763, 2048 }, { 7, 1764, 2048 }, { 8, 1765, 2048 }, { 8, 1766, 2048 }, { 9, 1767, 2048 },
   { 7, 1768, 2048 }, { 8, 1769, 2048 }, { 8, 1770, 2048 }, { 9, 1771, 2048 }, { 8, 1772, 2048 }, { 9, 1773, 2048 }, { 9, 1774, 2048 }, { 10, 1775, 2048 },
   { 7, 1776, 2048 }, { 8, 1777, 2048 }, { 8, 1778, 2048 }, { 9, 1779, 2048 }, { 8, 1780, 2048 }, { 9, 1781, 2048 }, { 9, 1782, 2048 }, { 10, 1783, 2048 },
   { 8, 1784, 2048 }, { 9, 1785, 2048 }, { 9, 1786, 2048 }, { 10, 1787, 2048 }, { 9, 1788, 2048 }, { 10, 1789, 2048 }, { 10, 1790, 2048 }, { 11, 1791, 2048 },
   { 4, 1792, 2048 }, { 5, 1793, 2048 }, { 5, 1794, 2048 }, { 6, 1795, 2048 }, { 5, 1796, 2048 }, { 6, 1797, 2048 }, { 6, 1798, 2048 }, { 7, 1799, 2048 },
   { 5, 1800, 2048 }, { 6, 1801, 2048 }, { 6, 1802, 2048 }, { 7, 1803, 2048 }, { 6, 1804, 2048 }, { 7, 1805, 2048 }, { 7, 1806, 2048 }, { 8, 1807, 2048 },
   { 5, 1808, 2048 }, { 6, 1809, 2048 }, { 6, 1810, 2048 }, { 7, 1811, 2048 }, { 6, 1812, 2048 }, { 7, 1813, 2048 }, { 7, 1814, 2048 }, { 8, 1815, 2048 },
   { 6, 1816, 2048 }, { 7, 1817, 2048 }, { 7, 1818, 2048 }, { 8, 1819, 2048 }, { 7, 1820, 2048 }, { 8, 1821, 2048 }, { 8, 1822, 2048 }, { 9, 1823, 2048 },
   { 5, 1824, 2048 }, { 6, 1825, 2048 }, { 6, 1826, 2048 }, { 7, 1827, 2048 }, { 6, 1828, 2048 }, { 7, 1829, 2048 }, { 7, 1830, 2048 }, { 8, 1831, 2048 },
   { 6, 1832, 2048 }, { 7, 1833, 2048 }, { 7, 1834, 2048 }, { 8, 1835, 2048 }, { 7, 1836, 2048 }, { 8, 1837, 2048 }, { 8, 1838, 2048 }, { 9, 1839, 2048 },
   { 6, 1840, 2048 }, { 7, 1841, 2048 }, { 7, 1842, 2048 }, { 8, 1843, 2048 }, { 7, 1844, 2048 }, { 8, 1845, 2048 }, { 8, 1846, 2048 }, { 9, 1847, 2048 },
   { 7, 1848, 2048 }, { 8, 1849, 2048 }, { 8, 1850, 2048 }, { 9, 1851, 2048 }, { 8, 1852, 2048 }, { 9, 1853, 2048 }, { 9, 1854, 2048 }, { 10, 1855, 2048 },
   { 5, 1856, 2048 }, { 6, 1857, 2048 }, { 6, 1858, 2048 }, { 7, 1859, 2048 }, { 6, 1860, 2048 }, { 7, 1861, 2048 }, { 7, 1862, 2048 }, { 8, 1863, 2048 },
   { 6, 1864, 2048 }, { 7, 1865, 2048 }, { 7, 1866, 2048 }, { 8, 1867, 2048 }, { 7, 1868, 2048 }, { 8, 1869, 2048 }, { 8, 1870, 2048 }, { 9, 1871, 2048 },
   { 6, 1872, 2048 }, { 7, 1873, 2048 }, { 7, 1874, 2048 }, { 8, 1875, 2048 }, { 7, 1876, 2048 }, { 8, 1877, 2048 }, { 8, 1878, 2048 }, { 9, 1879, 2048 },
   { 7, 1880, 2048 }, { 8, 1881, 2048 }, { 8, 1882, 2048 }, { 9, 1883, 2048 }, { 8, 1884, 2048 }, { 9, 1885, 2048 }, { 9, 1886, 2048 }, { 10, 1887, 2048 },
   { 6, 1888, 2048 }, { 7, 1889, 2048 }, { 7, 1890, 2048 }, { 8, 1891, 2048 }, { 7, 1892, 2048 }, { 8, 1893, 2048 }, { 8, 1894, 2048 }, { 9, 1895, 2048 },
   { 7, 1896, 2048 }, { 8, 1897, 2048 }, { 8, 1898, 2048 }, { 9, 1899, 2048 }, { 8, 1900, 2048 }, { 9, 1901, 2048 }, { 9, 1902, 2048 }, { 10, 1903, 2048 },
   { 7, 1904, 2048 }, { 8, 1905, 2048 }, { 8, 1906, 2048 }, { 9, 1907, 2048 }, { 8, 1908, 2048 }, { 9, 1909, 2048 }, { 9, 1910, 2048 }, { 10, 1911, 2048 },
   { 8, 1912, 2048 }, { 9, 1913, 2048 }, { 9, 1914, 2048 }, { 10, 1915, 2048 }, { 9, 1916, 2048 }, { 10, 1917, 2048 }, { 10, 1918, 2048 }, { 11, 1919, 2048 },
   { 5, 1920, 2048 }, { 6, 1921, 2048 }, { 6, 1922, 2048 }, { 7, 1923, 2048 }, { 6, 1924, 2048 }, { 7, 1925, 2048 }, { 7, 1926, 2048 }, { 8, 1927, 2048 },
   { 6, 1928, 2048 }, { 7, 1929, 2048 }, { 7, 1930, 2048 }, { 8, 1931, 2048 }, { 7, 1932, 2048 }, { 8, 1933, 2048 }, { 8, 1934, 2048 }, { 9, 1935, 2048 },
   { 6, 1936, 2048 }, { 7, 1937, 2048 }, { 7, 1938, 2048 }, { 8, 1939, 2048 }, { 7, 1940, 2048 }, { 8, 1941, 2048 }, { 8, 1942, 2048 }, { 9, 1943, 2048 },
   { 7, 1944, 2048 }, { 8, 1945, 2048 }, { 8, 1946, 2048 }, { 9, 1947, 2048 }, { 8, 1948, 2048 }, { 9, 1949, 2048 }, { 9, 1950, 2048 }, { 10, 1951, 2048 },
   { 6, 1952, 2048 }, { 7, 1953, 2048 }, { 7, 1954, 2048 }, { 8, 1955, 2048 }, { 7, 1956, 2048 }, { 8, 1957, 2048 }, { 8, 1958, 2048 }, { 9, 1959, 2048 },
   { 7, 1960, 2048 }, { 8, 1961, 2048 }, { 8, 1962, 2048 }, { 9, 1963, 2048 }, { 8, 1964, 2048 }, { 9, 1965, 2048 }, { 9, 1966, 2048 }, { 10, 1967, 2048 },
   { 7, 1968, 2048 }, { 8, 1969, 2048 }, { 8, 1970, 2048 }, { 9, 1971, 2048 }, { 8, 1972, 2048 }, { 9, 1973, 2048 }, { 9, 1974, 2048 }, { 10, 1975, 2048 },
   { 8, 1976, 2048 }, { 9, 1977, 2048 }, { 9, 1978, 2048 }, { 10, 1979, 2048 }, { 9, 1980, 2048 }, { 10, 1981, 2048 }, { 10, 1982, 2048 }, { 11, 1983, 2048 },
   { 6, 1984, 2048 }, { 7, 1985, 2048 }, { 7, 1986, 2048 }, { 8, 1987, 2048 }, { 7, 1988, 2048 }, { 8, 1989, 2048 }, { 8, 1990, 2048 }, { 9, 1991, 2048 },
   { 7, 1992, 2048 }, { 8, 1993, 2048 }, { 8, 1994, 2048 }, { 9, 1995, 2048 }, { 8, 1996, 2048 }, { 9, 1997, 2048 }, { 9, 1998, 2048 }, { 10, 1999, 2048 },
   { 7, 2000, 2048 }, { 8, 2001, 2048 }, { 8, 2002, 2048 }, { 9, 2003, 2048 }, { 8, 2004, 2048 }, { 9, 2005, 2048 }, { 9, 2006, 2048 }, { 10, 2007, 2048 },
   { 8, 2008, 2048 }, { 9, 2009, 2048 }, { 9, 2010, 2048 }, { 10, 2011, 2048 }, { 9, 2012, 2048 }, { 10, 2013, 2048 }, { 10, 2014, 2048 }, { 11, 2015, 2048 },
   { 7, 2016, 2048 }, { 8, 2017, 2048 }, { 8, 2018, 2048 }, { 9, 2019, 2048 }, { 8, 2020, 2048 }, { 9, 2021, 2048 }, { 9, 2022, 2048 }, { 10, 2023, 2048 },
   { 8, 2024, 2048 }, { 9, 2025, 2048 }, { 9, 2026, 2048 }, { 10, 2027, 2048 }, { 9, 2028, 2048 }, { 10, 2029, 2048 }, { 10, 2030, 2048 }, { 11, 2031, 2048 },
   { 8, 2032, 2048 }, { 9, 2033, 2048 }, { 9, 2034, 2048 }, { 10, 2035, 2048 }, { 9, 2036, 2048 }, { 10, 2037, 2048 }, { 10, 2038, 2048 }, { 11, 2039, 2048 },
   { 9, 2040, 2048 }, { 10, 2041, 2048 }, { 10, 2042, 2048 }, { 11, 2043, 2048 }, { 10, 2044, 2048 }, { 11, 2045, 2048 }, { 11, 2046, 2048 }, { 12, 2047, 2048 },
#endif
#endif
#endif
#endif
#endif
#endif
};

/* find a hole and free as required, return -1 if no hole found */
static int find_hole(void)
{
   unsigned x;
   int      y, z;
   for (z = -1, y = INT_MAX, x = 0; x < FP_ENTRIES; x++) {
       if (fp_cache[x].lru_count < y && fp_cache[x].lock == 0) {
          z = x;
          y = fp_cache[x].lru_count;
       }
   }

   /* decrease all */
   for (x = 0; x < FP_ENTRIES; x++) {
      if (fp_cache[x].lru_count > 3) {
         --(fp_cache[x].lru_count);
      }
   }

   /* free entry z */
   if (z >= 0 && fp_cache[z].g) {
      mp_clear(&fp_cache[z].mu);
      wc_ecc_del_point(fp_cache[z].g);
      fp_cache[z].g  = NULL;
      for (x = 0; x < (1U<<FP_LUT); x++) {
         wc_ecc_del_point(fp_cache[z].LUT[x]);
         fp_cache[z].LUT[x] = NULL;
      }
      fp_cache[z].lru_count = 0;
   }
   return z;
}

/* determine if a base is already in the cache and if so, where */
static int find_base(ecc_point* g)
{
   int x;
   for (x = 0; x < FP_ENTRIES; x++) {
      if (fp_cache[x].g != NULL &&
          mp_cmp(fp_cache[x].g->x, g->x) == MP_EQ &&
          mp_cmp(fp_cache[x].g->y, g->y) == MP_EQ &&
          mp_cmp(fp_cache[x].g->z, g->z) == MP_EQ) {
         break;
      }
   }
   if (x == FP_ENTRIES) {
      x = -1;
   }
   return x;
}

/* add a new base to the cache */
static int add_entry(int idx, ecc_point *g)
{
   unsigned x, y;

   /* allocate base and LUT */
   fp_cache[idx].g = wc_ecc_new_point();
   if (fp_cache[idx].g == NULL) {
      return GEN_MEM_ERR;
   }

   /* copy x and y */
   if ((mp_copy(g->x, fp_cache[idx].g->x) != MP_OKAY) ||
       (mp_copy(g->y, fp_cache[idx].g->y) != MP_OKAY) ||
       (mp_copy(g->z, fp_cache[idx].g->z) != MP_OKAY)) {
      wc_ecc_del_point(fp_cache[idx].g);
      fp_cache[idx].g = NULL;
      return GEN_MEM_ERR;
   }

   for (x = 0; x < (1U<<FP_LUT); x++) {
      fp_cache[idx].LUT[x] = wc_ecc_new_point();
      if (fp_cache[idx].LUT[x] == NULL) {
         for (y = 0; y < x; y++) {
            wc_ecc_del_point(fp_cache[idx].LUT[y]);
            fp_cache[idx].LUT[y] = NULL;
         }
         wc_ecc_del_point(fp_cache[idx].g);
         fp_cache[idx].g         = NULL;
         fp_cache[idx].lru_count = 0;
         return GEN_MEM_ERR;
      }
   }

   fp_cache[idx].lru_count = 0;

   return MP_OKAY;
}

/* build the LUT by spacing the bits of the input by #modulus/FP_LUT bits apart
 *
 * The algorithm builds patterns in increasing bit order by first making all
 * single bit input patterns, then all two bit input patterns and so on
 */
static int build_lut(int idx, mp_int* modulus, mp_digit* mp, mp_int* mu)
{
   unsigned x, y, err, bitlen, lut_gap;
   mp_int tmp;

   if (mp_init(&tmp) != MP_OKAY)
       return GEN_MEM_ERR;

   /* sanity check to make sure lut_order table is of correct size,
      should compile out to a NOP if true */
   if ((sizeof(lut_orders) / sizeof(lut_orders[0])) < (1U<<FP_LUT)) {
       err = BAD_FUNC_ARG;
   }
   else {
    /* get bitlen and round up to next multiple of FP_LUT */
    bitlen  = mp_unsigned_bin_size(modulus) << 3;
    x       = bitlen % FP_LUT;
    if (x) {
      bitlen += FP_LUT - x;
    }
    lut_gap = bitlen / FP_LUT;

    /* init the mu */
    err = mp_init_copy(&fp_cache[idx].mu, mu);
   }

   /* copy base */
   if (err == MP_OKAY) {
     if ((mp_mulmod(fp_cache[idx].g->x, mu, modulus,
                  fp_cache[idx].LUT[1]->x) != MP_OKAY) ||
         (mp_mulmod(fp_cache[idx].g->y, mu, modulus,
                  fp_cache[idx].LUT[1]->y) != MP_OKAY) ||
         (mp_mulmod(fp_cache[idx].g->z, mu, modulus,
                  fp_cache[idx].LUT[1]->z) != MP_OKAY)) {
       err = MP_MULMOD_E;
     }
   }

   /* make all single bit entries */
   for (x = 1; x < FP_LUT; x++) {
      if (err != MP_OKAY)
          break;
      if ((mp_copy(fp_cache[idx].LUT[1<<(x-1)]->x,
                   fp_cache[idx].LUT[1<<x]->x) != MP_OKAY) ||
          (mp_copy(fp_cache[idx].LUT[1<<(x-1)]->y,
                   fp_cache[idx].LUT[1<<x]->y) != MP_OKAY) ||
          (mp_copy(fp_cache[idx].LUT[1<<(x-1)]->z,
                   fp_cache[idx].LUT[1<<x]->z) != MP_OKAY)){
          err = MP_INIT_E;
          break;
      } else {

         /* now double it bitlen/FP_LUT times */
         for (y = 0; y < lut_gap; y++) {
             if ((err = ecc_projective_dbl_point(fp_cache[idx].LUT[1<<x],
                            fp_cache[idx].LUT[1<<x], modulus, mp)) != MP_OKAY) {
                 break;
             }
         }
     }
  }

   /* now make all entries in increase order of hamming weight */
   for (x = 2; x <= FP_LUT; x++) {
       if (err != MP_OKAY)
           break;
       for (y = 0; y < (1UL<<FP_LUT); y++) {
           if (err != MP_OKAY)
             break;
           if (lut_orders[y].ham != (int)x) continue;

           /* perform the add */
           if ((err = ecc_projective_add_point(
                           fp_cache[idx].LUT[lut_orders[y].terma],
                           fp_cache[idx].LUT[lut_orders[y].termb],
                           fp_cache[idx].LUT[y], modulus, mp)) != MP_OKAY) {
              break;
           }
       }
   }

   /* now map all entries back to affine space to make point addition faster */
   for (x = 1; x < (1UL<<FP_LUT); x++) {
       if (err != MP_OKAY)
           break;

       /* convert z to normal from montgomery */
       err = mp_montgomery_reduce(fp_cache[idx].LUT[x]->z, modulus, *mp);

       /* invert it */
       if (err == MP_OKAY)
         err = mp_invmod(fp_cache[idx].LUT[x]->z, modulus,
                         fp_cache[idx].LUT[x]->z);

       if (err == MP_OKAY)
         /* now square it */
         err = mp_sqrmod(fp_cache[idx].LUT[x]->z, modulus, &tmp);

       if (err == MP_OKAY)
         /* fix x */
         err = mp_mulmod(fp_cache[idx].LUT[x]->x, &tmp, modulus,
                         fp_cache[idx].LUT[x]->x);

       if (err == MP_OKAY)
         /* get 1/z^3 */
         err = mp_mulmod(&tmp, fp_cache[idx].LUT[x]->z, modulus, &tmp);

       if (err == MP_OKAY)
         /* fix y */
         err = mp_mulmod(fp_cache[idx].LUT[x]->y, &tmp, modulus,
                         fp_cache[idx].LUT[x]->y);

       if (err == MP_OKAY)
         /* free z */
         mp_clear(fp_cache[idx].LUT[x]->z);
   }
   mp_clear(&tmp);

   if (err == MP_OKAY)
     return MP_OKAY;

   /* err cleanup */
   for (y = 0; y < (1U<<FP_LUT); y++) {
      wc_ecc_del_point(fp_cache[idx].LUT[y]);
      fp_cache[idx].LUT[y] = NULL;
   }
   wc_ecc_del_point(fp_cache[idx].g);
   fp_cache[idx].g         = NULL;
   fp_cache[idx].lru_count = 0;
   mp_clear(&fp_cache[idx].mu);
   mp_clear(&tmp);

   return err;
}

/* perform a fixed point ECC mulmod */
static int accel_fp_mul(int idx, mp_int* k, ecc_point *R, mp_int* modulus,
                        mp_digit* mp, int map)
{
#define KB_SIZE 128

#ifdef WOLFSSL_SMALL_STACK
   unsigned char* kb;
#else
   unsigned char kb[128];
#endif
   int      x;
   unsigned y, z, err, bitlen, bitpos, lut_gap, first;
   mp_int   tk;

   if (mp_init(&tk) != MP_OKAY)
       return MP_INIT_E;

   /* if it's smaller than modulus we fine */
   if (mp_unsigned_bin_size(k) > mp_unsigned_bin_size(modulus)) {
      mp_int order;
      if (mp_init(&order) != MP_OKAY) {
        mp_clear(&tk);
        return MP_INIT_E;
      }

      /* find order */
      y = mp_unsigned_bin_size(modulus);
      for (x = 0; ecc_sets[x].size; x++) {
         if (y <= (unsigned)ecc_sets[x].size) break;
      }

      /* back off if we are on the 521 bit curve */
      if (y == 66) --x;

      if ((err = mp_read_radix(&order, ecc_sets[x].order, 16)) != MP_OKAY) {
         mp_clear(&order);
         mp_clear(&tk);
         return err;
      }

      /* k must be less than modulus */
      if (mp_cmp(k, &order) != MP_LT) {
         if ((err = mp_mod(k, &order, &tk)) != MP_OKAY) {
            mp_clear(&tk);
            mp_clear(&order);
            return err;
         }
      } else {
         mp_copy(k, &tk);
      }
      mp_clear(&order);
   } else {
      mp_copy(k, &tk);
   }

   /* get bitlen and round up to next multiple of FP_LUT */
   bitlen  = mp_unsigned_bin_size(modulus) << 3;
   x       = bitlen % FP_LUT;
   if (x) {
      bitlen += FP_LUT - x;
   }
   lut_gap = bitlen / FP_LUT;

   /* get the k value */
   if (mp_unsigned_bin_size(&tk) > (int)(KB_SIZE - 2)) {
      mp_clear(&tk);
      return BUFFER_E;
   }

   /* store k */
#ifdef WOLFSSL_SMALL_STACK
   kb = (unsigned char*)XMALLOC(KB_SIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
   if (kb == NULL)
      return MEMORY_E;
#endif

   XMEMSET(kb, 0, KB_SIZE);
   if ((err = mp_to_unsigned_bin(&tk, kb)) != MP_OKAY) {
      mp_clear(&tk);
   }
   else {
      /* let's reverse kb so it's little endian */
      x = 0;
      y = mp_unsigned_bin_size(&tk);
      if (y > 0) {
          y -= 1;
      }
      mp_clear(&tk);

      while ((unsigned)x < y) {
         z = kb[x]; kb[x] = kb[y]; kb[y] = z;
         ++x; --y;
      }

      /* at this point we can start, yipee */
      first = 1;
      for (x = lut_gap-1; x >= 0; x--) {
          /* extract FP_LUT bits from kb spread out by lut_gap bits and offset
             by x bits from the start */
          bitpos = x;
          for (y = z = 0; y < FP_LUT; y++) {
             z |= ((kb[bitpos>>3] >> (bitpos&7)) & 1) << y;
             bitpos += lut_gap;  /* it's y*lut_gap + x, but here we can avoid
                                    the mult in each loop */
          }

          /* double if not first */
          if (!first) {
             if ((err = ecc_projective_dbl_point(R, R, modulus,
                                                              mp)) != MP_OKAY) {
                break;
             }
          }

          /* add if not first, otherwise copy */
          if (!first && z) {
             if ((err = ecc_projective_add_point(R, fp_cache[idx].LUT[z], R,
                                                     modulus, mp)) != MP_OKAY) {
                break;
             }
          } else if (z) {
             if ((mp_copy(fp_cache[idx].LUT[z]->x, R->x) != MP_OKAY) ||
                 (mp_copy(fp_cache[idx].LUT[z]->y, R->y) != MP_OKAY) ||
                 (mp_copy(&fp_cache[idx].mu,       R->z) != MP_OKAY)) {
                 err = GEN_MEM_ERR;
                 break;
             }
                 first = 0;
          }
      }
   }

   if (err == MP_OKAY) {
      z = 0;    /* mp_to_unsigned_bin != MP_OKAY z will be declared/not set */
      (void) z; /* Acknowledge the unused assignment */
      ForceZero(kb, KB_SIZE);
      /* map R back from projective space */
      if (map) {
         err = ecc_map(R, modulus, mp);
      } else {
         err = MP_OKAY;
      }
   }

#ifdef WOLFSSL_SMALL_STACK
   XFREE(kb, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

#undef KB_SIZE

   return err;
}

#ifdef ECC_SHAMIR
/* perform a fixed point ECC mulmod */
static int accel_fp_mul2add(int idx1, int idx2,
                            mp_int* kA, mp_int* kB,
                            ecc_point *R, mp_int* modulus, mp_digit* mp)
{
#define KB_SIZE 128

#ifdef WOLFSSL_SMALL_STACK
   unsigned char* kb[2];
#else
   unsigned char kb[2][128];
#endif
   int      x;
   unsigned y, z, err, bitlen, bitpos, lut_gap, first, zA, zB;
   mp_int tka;
   mp_int tkb;
   mp_int order;

   if (mp_init_multi(&tka, &tkb, 0, 0, 0, 0) != MP_OKAY)
       return MP_INIT_E;

   /* if it's smaller than modulus we fine */
   if (mp_unsigned_bin_size(kA) > mp_unsigned_bin_size(modulus)) {
      /* find order */
      y = mp_unsigned_bin_size(modulus);
      for (x = 0; ecc_sets[x].size; x++) {
         if (y <= (unsigned)ecc_sets[x].size) break;
      }

      /* back off if we are on the 521 bit curve */
      if (y == 66) --x;

      if ((err = mp_init(&order)) != MP_OKAY) {
         mp_clear(&tkb);
         mp_clear(&tka);
         return err;
      }
      if ((err = mp_read_radix(&order, ecc_sets[x].order, 16)) != MP_OKAY) {
         mp_clear(&tkb);
         mp_clear(&tka);
         mp_clear(&order);
         return err;
      }

      /* kA must be less than modulus */
      if (mp_cmp(kA, &order) != MP_LT) {
         if ((err = mp_mod(kA, &order, &tka)) != MP_OKAY) {
            mp_clear(&tkb);
            mp_clear(&tka);
            mp_clear(&order);
            return err;
         }
      } else {
         mp_copy(kA, &tka);
      }
      mp_clear(&order);
   } else {
      mp_copy(kA, &tka);
   }

   /* if it's smaller than modulus we fine */
   if (mp_unsigned_bin_size(kB) > mp_unsigned_bin_size(modulus)) {
      /* find order */
      y = mp_unsigned_bin_size(modulus);
      for (x = 0; ecc_sets[x].size; x++) {
         if (y <= (unsigned)ecc_sets[x].size) break;
      }

      /* back off if we are on the 521 bit curve */
      if (y == 66) --x;

      if ((err = mp_init(&order)) != MP_OKAY) {
         mp_clear(&tkb);
         mp_clear(&tka);
         return err;
      }
      if ((err = mp_read_radix(&order, ecc_sets[x].order, 16)) != MP_OKAY) {
         mp_clear(&tkb);
         mp_clear(&tka);
         mp_clear(&order);
         return err;
      }

      /* kB must be less than modulus */
      if (mp_cmp(kB, &order) != MP_LT) {
         if ((err = mp_mod(kB, &order, &tkb)) != MP_OKAY) {
            mp_clear(&tkb);
            mp_clear(&tka);
            mp_clear(&order);
            return err;
         }
      } else {
         mp_copy(kB, &tkb);
      }
      mp_clear(&order);
   } else {
      mp_copy(kB, &tkb);
   }

   /* get bitlen and round up to next multiple of FP_LUT */
   bitlen  = mp_unsigned_bin_size(modulus) << 3;
   x       = bitlen % FP_LUT;
   if (x) {
      bitlen += FP_LUT - x;
   }
   lut_gap = bitlen / FP_LUT;

   /* get the k value */
   if ((mp_unsigned_bin_size(&tka) > (int)(KB_SIZE - 2)) ||
       (mp_unsigned_bin_size(&tkb) > (int)(KB_SIZE - 2))  ) {
      mp_clear(&tka);
      mp_clear(&tkb);
      return BUFFER_E;
   }

   /* store k */
#ifdef WOLFSSL_SMALL_STACK
   kb[0] = (unsigned char*)XMALLOC(KB_SIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
   if (kb[0] == NULL)
      return MEMORY_E;
#endif

   XMEMSET(kb[0], 0, KB_SIZE);
   if ((err = mp_to_unsigned_bin(&tka, kb[0])) != MP_OKAY) {
      mp_clear(&tka);
      mp_clear(&tkb);
#ifdef WOLFSSL_SMALL_STACK
      XFREE(kb[0], NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif
      return err;
   }

   /* let's reverse kb so it's little endian */
   x = 0;
   y = mp_unsigned_bin_size(&tka);
   if (y > 0) {
       y -= 1;
   }
   mp_clear(&tka);
   while ((unsigned)x < y) {
      z = kb[0][x]; kb[0][x] = kb[0][y]; kb[0][y] = z;
      ++x; --y;
   }

   /* store b */
#ifdef WOLFSSL_SMALL_STACK
   kb[1] = (unsigned char*)XMALLOC(KB_SIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
   if (kb[1] == NULL) {
      XFREE(kb[0], NULL, DYNAMIC_TYPE_TMP_BUFFER);
      return MEMORY_E;
   }
#endif

   XMEMSET(kb[1], 0, KB_SIZE);
   if ((err = mp_to_unsigned_bin(&tkb, kb[1])) != MP_OKAY) {
      mp_clear(&tkb);
   }
   else {
      x = 0;
      y = mp_unsigned_bin_size(&tkb);
      if (y > 0) {
          y -= 1;
      }
      mp_clear(&tkb);
      while ((unsigned)x < y) {
         z = kb[1][x]; kb[1][x] = kb[1][y]; kb[1][y] = z;
         ++x; --y;
      }

      /* at this point we can start, yipee */
      first = 1;
      for (x = lut_gap-1; x >= 0; x--) {
          /* extract FP_LUT bits from kb spread out by lut_gap bits and
             offset by x bits from the start */
          bitpos = x;
          for (y = zA = zB = 0; y < FP_LUT; y++) {
             zA |= ((kb[0][bitpos>>3] >> (bitpos&7)) & 1) << y;
             zB |= ((kb[1][bitpos>>3] >> (bitpos&7)) & 1) << y;
             bitpos += lut_gap;    /* it's y*lut_gap + x, but here we can avoid
                                      the mult in each loop */
          }

          /* double if not first */
          if (!first) {
             if ((err = ecc_projective_dbl_point(R, R, modulus,
                                                              mp)) != MP_OKAY) {
                break;
             }
          }

          /* add if not first, otherwise copy */
          if (!first) {
             if (zA) {
                if ((err = ecc_projective_add_point(R, fp_cache[idx1].LUT[zA],
                                                  R, modulus, mp)) != MP_OKAY) {
                   break;
                }
             }
             if (zB) {
                if ((err = ecc_projective_add_point(R, fp_cache[idx2].LUT[zB],
                                                  R, modulus, mp)) != MP_OKAY) {
                   break;
                }
             }
          } else {
             if (zA) {
                 if ((mp_copy(fp_cache[idx1].LUT[zA]->x, R->x) != MP_OKAY) ||
                    (mp_copy(fp_cache[idx1].LUT[zA]->y,  R->y) != MP_OKAY) ||
                    (mp_copy(&fp_cache[idx1].mu,         R->z) != MP_OKAY)) {
                     err = GEN_MEM_ERR;
                     break;
                 }
                    first = 0;
             }
             if (zB && first == 0) {
                if (zB) {
                   if ((err = ecc_projective_add_point(R,
                           fp_cache[idx2].LUT[zB], R, modulus, mp)) != MP_OKAY){
                      break;
                   }
                }
             } else if (zB && first == 1) {
                 if ((mp_copy(fp_cache[idx2].LUT[zB]->x, R->x) != MP_OKAY) ||
                    (mp_copy(fp_cache[idx2].LUT[zB]->y, R->y) != MP_OKAY) ||
                    (mp_copy(&fp_cache[idx2].mu,        R->z) != MP_OKAY)) {
                     err = GEN_MEM_ERR;
                     break;
                 }
                    first = 0;
             }
          }
      }
   }

   ForceZero(kb[0], KB_SIZE);
   ForceZero(kb[1], KB_SIZE);

#ifdef WOLFSSL_SMALL_STACK
   XFREE(kb[0], NULL, DYNAMIC_TYPE_TMP_BUFFER);
   XFREE(kb[1], NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

#undef KB_SIZE

    if (err != MP_OKAY)
        return err;

   return ecc_map(R, modulus, mp);
}

/** ECC Fixed Point mulmod global
  Computes kA*A + kB*B = C using Shamir's Trick
  A        First point to multiply
  kA       What to multiple A by
  B        Second point to multiply
  kB       What to multiple B by
  C        [out] Destination point (can overlap with A or B)
  modulus  Modulus for curve
  return MP_OKAY on success
*/
int ecc_mul2add(ecc_point* A, mp_int* kA,
                ecc_point* B, mp_int* kB,
                ecc_point* C, mp_int* modulus)
{
   int  idx1 = -1, idx2 = -1, err = MP_OKAY, mpInit = 0;
   mp_digit mp;
   mp_int   mu;

   err = mp_init(&mu);
   if (err != MP_OKAY)
       return err;

#ifndef HAVE_THREAD_LS
   if (initMutex == 0) {
        InitMutex(&ecc_fp_lock);
        initMutex = 1;
   }
   if (LockMutex(&ecc_fp_lock) != 0)
      return BAD_MUTEX_E;
#endif /* HAVE_THREAD_LS */

      /* find point */
      idx1 = find_base(A);

      /* no entry? */
      if (idx1 == -1) {
         /* find hole and add it */
         if ((idx1 = find_hole()) >= 0) {
            err = add_entry(idx1, A);
         }
      }
      if (err == MP_OKAY && idx1 != -1) {
         /* increment LRU */
         ++(fp_cache[idx1].lru_count);
      }

      if (err == MP_OKAY)
        /* find point */
        idx2 = find_base(B);

      if (err == MP_OKAY) {
        /* no entry? */
        if (idx2 == -1) {
           /* find hole and add it */
           if ((idx2 = find_hole()) >= 0)
              err = add_entry(idx2, B);
         }
      }

      if (err == MP_OKAY && idx2 != -1) {
         /* increment LRU */
         ++(fp_cache[idx2].lru_count);
      }

      if (err == MP_OKAY) {
        /* if it's 2 build the LUT, if it's higher just use the LUT */
        if (idx1 >= 0 && fp_cache[idx1].lru_count == 2) {
           /* compute mp */
           err = mp_montgomery_setup(modulus, &mp);

           if (err == MP_OKAY) {
             mpInit = 1;
             err = mp_montgomery_calc_normalization(&mu, modulus);
           }

           if (err == MP_OKAY)
             /* build the LUT */
               err = build_lut(idx1, modulus, &mp, &mu);
        }
      }

      if (err == MP_OKAY) {
        /* if it's 2 build the LUT, if it's higher just use the LUT */
        if (idx2 >= 0 && fp_cache[idx2].lru_count == 2) {
           if (mpInit == 0) {
                /* compute mp */
                err = mp_montgomery_setup(modulus, &mp);
                if (err == MP_OKAY) {
                    mpInit = 1;
                    err = mp_montgomery_calc_normalization(&mu, modulus);
                }
            }

            if (err == MP_OKAY)
            /* build the LUT */
              err = build_lut(idx2, modulus, &mp, &mu);
        }
      }


      if (err == MP_OKAY) {
        if (idx1 >=0 && idx2 >= 0 && fp_cache[idx1].lru_count >= 2 &&
                                     fp_cache[idx2].lru_count >= 2) {
           if (mpInit == 0) {
              /* compute mp */
              err = mp_montgomery_setup(modulus, &mp);
           }
           if (err == MP_OKAY)
             err = accel_fp_mul2add(idx1, idx2, kA, kB, C, modulus, &mp);
        } else {
           err = normal_ecc_mul2add(A, kA, B, kB, C, modulus);
        }
    }

#ifndef HAVE_THREAD_LS
    UnLockMutex(&ecc_fp_lock);
#endif /* HAVE_THREAD_LS */
    mp_clear(&mu);

    return err;
}
#endif /* ECC_SHAMIR */

/** ECC Fixed Point mulmod global
    k        The multiplicand
    G        Base point to multiply
    R        [out] Destination of product
    modulus  The modulus for the curve
    map      [boolean] If non-zero maps the point back to affine co-ordinates,
             otherwise it's left in jacobian-montgomery form
    return MP_OKAY if successful
*/
int wc_ecc_mulmod(mp_int* k, ecc_point *G, ecc_point *R, mp_int* modulus,
               int map)
{
   int   idx, err = MP_OKAY;
   mp_digit mp;
   mp_int   mu;
   int      mpSetup = 0;

   if (mp_init(&mu) != MP_OKAY)
       return MP_INIT_E;

#ifndef HAVE_THREAD_LS
   if (initMutex == 0) {
        InitMutex(&ecc_fp_lock);
        initMutex = 1;
   }

   if (LockMutex(&ecc_fp_lock) != 0)
      return BAD_MUTEX_E;
#endif /* HAVE_THREAD_LS */

      /* find point */
      idx = find_base(G);

      /* no entry? */
      if (idx == -1) {
         /* find hole and add it */
         idx = find_hole();

         if (idx >= 0)
            err = add_entry(idx, G);
      }
      if (err == MP_OKAY && idx >= 0) {
         /* increment LRU */
         ++(fp_cache[idx].lru_count);
      }


      if (err == MP_OKAY) {
        /* if it's 2 build the LUT, if it's higher just use the LUT */
        if (idx >= 0 && fp_cache[idx].lru_count == 2) {
           /* compute mp */
           err = mp_montgomery_setup(modulus, &mp);

           if (err == MP_OKAY) {
             /* compute mu */
             mpSetup = 1;
             err = mp_montgomery_calc_normalization(&mu, modulus);
           }

           if (err == MP_OKAY)
             /* build the LUT */
             err = build_lut(idx, modulus, &mp, &mu);
        }
      }

      if (err == MP_OKAY) {
        if (idx >= 0 && fp_cache[idx].lru_count >= 2) {
           if (mpSetup == 0) {
              /* compute mp */
              err = mp_montgomery_setup(modulus, &mp);
           }
           if (err == MP_OKAY)
             err = accel_fp_mul(idx, k, R, modulus, &mp, map);
        } else {
           err = normal_ecc_mulmod(k, G, R, modulus, map);
        }
     }

#ifndef HAVE_THREAD_LS
    UnLockMutex(&ecc_fp_lock);
#endif /* HAVE_THREAD_LS */
    mp_clear(&mu);

    return err;
}

/* helper function for freeing the cache ...
   must be called with the cache mutex locked */
static void wc_ecc_fp_free_cache(void)
{
   unsigned x, y;
   for (x = 0; x < FP_ENTRIES; x++) {
      if (fp_cache[x].g != NULL) {
         for (y = 0; y < (1U<<FP_LUT); y++) {
            wc_ecc_del_point(fp_cache[x].LUT[y]);
            fp_cache[x].LUT[y] = NULL;
         }
         wc_ecc_del_point(fp_cache[x].g);
         fp_cache[x].g         = NULL;
         mp_clear(&fp_cache[x].mu);
         fp_cache[x].lru_count = 0;
         fp_cache[x].lock = 0;
      }
   }
}

/** Free the Fixed Point cache */
void wc_ecc_fp_free(void)
{
#ifndef HAVE_THREAD_LS
   if (initMutex == 0) {
        InitMutex(&ecc_fp_lock);
        initMutex = 1;
   }

   if (LockMutex(&ecc_fp_lock) == 0) {
#endif /* HAVE_THREAD_LS */

       wc_ecc_fp_free_cache();

#ifndef HAVE_THREAD_LS
       UnLockMutex(&ecc_fp_lock);
       FreeMutex(&ecc_fp_lock);
       initMutex = 0;
   }
#endif /* HAVE_THREAD_LS */
}


#endif /* FP_ECC */

#ifdef HAVE_ECC_ENCRYPT


enum ecCliState {
    ecCLI_INIT      = 1,
    ecCLI_SALT_GET  = 2,
    ecCLI_SALT_SET  = 3,
    ecCLI_SENT_REQ  = 4,
    ecCLI_RECV_RESP = 5,
    ecCLI_BAD_STATE = 99
};

enum ecSrvState {
    ecSRV_INIT      = 1,
    ecSRV_SALT_GET  = 2,
    ecSRV_SALT_SET  = 3,
    ecSRV_RECV_REQ  = 4,
    ecSRV_SENT_RESP = 5,
    ecSRV_BAD_STATE = 99
};


struct ecEncCtx {
    const byte* kdfSalt;   /* optional salt for kdf */
    const byte* kdfInfo;   /* optional info for kdf */
    const byte* macSalt;   /* optional salt for mac */
    word32    kdfSaltSz;   /* size of kdfSalt */
    word32    kdfInfoSz;   /* size of kdfInfo */
    word32    macSaltSz;   /* size of macSalt */
    byte      clientSalt[EXCHANGE_SALT_SZ];  /* for msg exchange */
    byte      serverSalt[EXCHANGE_SALT_SZ];  /* for msg exchange */
    byte      encAlgo;     /* which encryption type */
    byte      kdfAlgo;     /* which key derivation function type */
    byte      macAlgo;     /* which mac function type */
    byte      protocol;    /* are we REQ_RESP client or server ? */
    byte      cliSt;       /* protocol state, for sanity checks */
    byte      srvSt;       /* protocol state, for sanity checks */
};


const byte* wc_ecc_ctx_get_own_salt(ecEncCtx* ctx)
{
    if (ctx == NULL || ctx->protocol == 0)
        return NULL;

    if (ctx->protocol == REQ_RESP_CLIENT) {
        if (ctx->cliSt == ecCLI_INIT) {
            ctx->cliSt =  ecCLI_SALT_GET;
            return ctx->clientSalt;
        }
        else {
            ctx->cliSt = ecCLI_BAD_STATE;
            return NULL;
        }
    }
    else if (ctx->protocol == REQ_RESP_SERVER) {
        if (ctx->srvSt == ecSRV_INIT) {
            ctx->srvSt =  ecSRV_SALT_GET;
            return ctx->serverSalt;
        }
        else {
            ctx->srvSt = ecSRV_BAD_STATE;
            return NULL;
        }
    }

    return NULL;
}


/* optional set info, can be called before or after set_peer_salt */
int wc_ecc_ctx_set_info(ecEncCtx* ctx, const byte* info, int sz)
{
    if (ctx == NULL || info == 0 || sz < 0)
        return BAD_FUNC_ARG;

    ctx->kdfInfo   = info;
    ctx->kdfInfoSz = sz;

    return 0;
}


static const char* exchange_info = "Secure Message Exchange";

int wc_ecc_ctx_set_peer_salt(ecEncCtx* ctx, const byte* salt)
{
    byte tmp[EXCHANGE_SALT_SZ/2];
    int  halfSz = EXCHANGE_SALT_SZ/2;

    if (ctx == NULL || ctx->protocol == 0 || salt == NULL)
        return BAD_FUNC_ARG;

    if (ctx->protocol == REQ_RESP_CLIENT) {
        XMEMCPY(ctx->serverSalt, salt, EXCHANGE_SALT_SZ);
        if (ctx->cliSt == ecCLI_SALT_GET)
            ctx->cliSt =  ecCLI_SALT_SET;
        else {
            ctx->cliSt =  ecCLI_BAD_STATE;
            return BAD_ENC_STATE_E;
        }
    }
    else {
        XMEMCPY(ctx->clientSalt, salt, EXCHANGE_SALT_SZ);
        if (ctx->srvSt == ecSRV_SALT_GET)
            ctx->srvSt =  ecSRV_SALT_SET;
        else {
            ctx->srvSt =  ecSRV_BAD_STATE;
            return BAD_ENC_STATE_E;
        }
    }

    /* mix half and half */
    /* tmp stores 2nd half of client before overwrite */
    XMEMCPY(tmp, ctx->clientSalt + halfSz, halfSz);
    XMEMCPY(ctx->clientSalt + halfSz, ctx->serverSalt, halfSz);
    XMEMCPY(ctx->serverSalt, tmp, halfSz);

    ctx->kdfSalt   = ctx->clientSalt;
    ctx->kdfSaltSz = EXCHANGE_SALT_SZ;

    ctx->macSalt   = ctx->serverSalt;
    ctx->macSaltSz = EXCHANGE_SALT_SZ;

    if (ctx->kdfInfo == NULL) {
        /* default info */
        ctx->kdfInfo   = (const byte*)exchange_info;
        ctx->kdfInfoSz = EXCHANGE_INFO_SZ;
    }

    return 0;
}


static int ecc_ctx_set_salt(ecEncCtx* ctx, int flags, WC_RNG* rng)
{
    byte* saltBuffer = NULL;

    if (ctx == NULL || rng == NULL || flags == 0)
        return BAD_FUNC_ARG;

    saltBuffer = (flags == REQ_RESP_CLIENT) ? ctx->clientSalt : ctx->serverSalt;

    return wc_RNG_GenerateBlock(rng, saltBuffer, EXCHANGE_SALT_SZ);
}


static void ecc_ctx_init(ecEncCtx* ctx, int flags)
{
    if (ctx) {
        XMEMSET(ctx, 0, sizeof(ecEncCtx));

        ctx->encAlgo  = ecAES_128_CBC;
        ctx->kdfAlgo  = ecHKDF_SHA256;
        ctx->macAlgo  = ecHMAC_SHA256;
        ctx->protocol = (byte)flags;

        if (flags == REQ_RESP_CLIENT)
            ctx->cliSt = ecCLI_INIT;
        if (flags == REQ_RESP_SERVER)
            ctx->srvSt = ecSRV_INIT;
    }
}


/* allow ecc context reset so user doesn't have to init/free for reuse */
int wc_ecc_ctx_reset(ecEncCtx* ctx, WC_RNG* rng)
{
    if (ctx == NULL || rng == NULL)
        return BAD_FUNC_ARG;

    ecc_ctx_init(ctx, ctx->protocol);
    return ecc_ctx_set_salt(ctx, ctx->protocol, rng);
}


/* alloc/init and set defaults, return new Context  */
ecEncCtx* wc_ecc_ctx_new(int flags, WC_RNG* rng)
{
    int       ret = 0;
    ecEncCtx* ctx = (ecEncCtx*)XMALLOC(sizeof(ecEncCtx), 0, DYNAMIC_TYPE_ECC);

    if (ctx)
        ctx->protocol = (byte)flags;

    ret = wc_ecc_ctx_reset(ctx, rng);
    if (ret != 0) {
        wc_ecc_ctx_free(ctx);
        ctx = NULL;
    }

    return ctx;
}


/* free any resources, clear any keys */
void wc_ecc_ctx_free(ecEncCtx* ctx)
{
    if (ctx) {
        ForceZero(ctx, sizeof(ecEncCtx));
        XFREE(ctx, 0, DYNAMIC_TYPE_ECC);
    }
}


static int ecc_get_key_sizes(ecEncCtx* ctx, int* encKeySz, int* ivSz,
                             int* keysLen, word32* digestSz, word32* blockSz)
{
    if (ctx) {
        switch (ctx->encAlgo) {
            case ecAES_128_CBC:
                *encKeySz = KEY_SIZE_128;
                *ivSz     = IV_SIZE_128;
                *blockSz  = AES_BLOCK_SIZE;
                break;
            default:
                return BAD_FUNC_ARG;
        }

        switch (ctx->macAlgo) {
            case ecHMAC_SHA256:
                *digestSz = SHA256_DIGEST_SIZE;
                break;
            default:
                return BAD_FUNC_ARG;
        }
    } else
        return BAD_FUNC_ARG;

    *keysLen  = *encKeySz + *ivSz + *digestSz;

    return 0;
}


/* ecc encrypt with shared secret run through kdf
   ctx holds non default algos and inputs
   msgSz should be the right size for encAlgo, i.e., already padded
   return 0 on success */
int wc_ecc_encrypt(ecc_key* privKey, ecc_key* pubKey, const byte* msg,
                word32 msgSz, byte* out, word32* outSz, ecEncCtx* ctx)
{
    int          ret;
    word32       blockSz;
    word32       digestSz;
    ecEncCtx     localCtx;
#ifdef WOLFSSL_SMALL_STACK
    byte*        sharedSecret;
    byte*        keys;
#else
    byte         sharedSecret[ECC_MAXSIZE];  /* 521 max size */
    byte         keys[ECC_BUFSIZE];         /* max size */
#endif
    word32       sharedSz = ECC_MAXSIZE;
    int          keysLen;
    int          encKeySz;
    int          ivSz;
    int          offset = 0;         /* keys offset if doing msg exchange */
    byte*        encKey;
    byte*        encIv;
    byte*        macKey;

    if (privKey == NULL || pubKey == NULL || msg == NULL || out == NULL ||
                           outSz  == NULL)
        return BAD_FUNC_ARG;

    if (ctx == NULL) {  /* use defaults */
        ecc_ctx_init(&localCtx, 0);
        ctx = &localCtx;
    }

    ret = ecc_get_key_sizes(ctx, &encKeySz, &ivSz, &keysLen, &digestSz,
                            &blockSz);
    if (ret != 0)
        return ret;

    if (ctx->protocol == REQ_RESP_SERVER) {
        offset = keysLen;
        keysLen *= 2;

        if (ctx->srvSt != ecSRV_RECV_REQ)
            return BAD_ENC_STATE_E;

        ctx->srvSt = ecSRV_BAD_STATE; /* we're done no more ops allowed */
    }
    else if (ctx->protocol == REQ_RESP_CLIENT) {
        if (ctx->cliSt != ecCLI_SALT_SET)
            return BAD_ENC_STATE_E;

        ctx->cliSt = ecCLI_SENT_REQ; /* only do this once */
    }

    if (keysLen > ECC_BUFSIZE) /* keys size */
        return BUFFER_E;

    if ( (msgSz%blockSz) != 0)
        return BAD_PADDING_E;

    if (*outSz < (msgSz + digestSz))
        return BUFFER_E;

#ifdef WOLFSSL_SMALL_STACK
    sharedSecret = (byte*)XMALLOC(ECC_MAXSIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (sharedSecret == NULL)
        return MEMORY_E;

    keys = (byte*)XMALLOC(ECC_BUFSIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (keys == NULL) {
        XFREE(sharedSecret, NULL, DYNAMIC_TYPE_TMP_BUFFER);
        return MEMORY_E;
    }
#endif

    ret = wc_ecc_shared_secret(privKey, pubKey, sharedSecret, &sharedSz);

    if (ret == 0) {
       switch (ctx->kdfAlgo) {
           case ecHKDF_SHA256 :
               ret = wc_HKDF(SHA256, sharedSecret, sharedSz, ctx->kdfSalt,
                          ctx->kdfSaltSz, ctx->kdfInfo, ctx->kdfInfoSz,
                          keys, keysLen);
               break;

           default:
               ret = BAD_FUNC_ARG;
               break;
       }
    }

    if (ret == 0) {
       encKey = keys + offset;
       encIv  = encKey + encKeySz;
       macKey = encKey + encKeySz + ivSz;

       switch (ctx->encAlgo) {
           case ecAES_128_CBC:
               {
                   Aes aes;
                   ret = wc_AesSetKey(&aes, encKey, KEY_SIZE_128, encIv,
                                                                AES_ENCRYPTION);
                   if (ret != 0)
                       break;
                   ret = wc_AesCbcEncrypt(&aes, out, msg, msgSz);
               }
               break;

           default:
               ret = BAD_FUNC_ARG;
               break;
       }
    }

    if (ret == 0) {
       switch (ctx->macAlgo) {
           case ecHMAC_SHA256:
               {
                   Hmac hmac;
                   ret = wc_HmacSetKey(&hmac, SHA256, macKey, SHA256_DIGEST_SIZE);
                   if (ret != 0)
                       break;
                   ret = wc_HmacUpdate(&hmac, out, msgSz);
                   if (ret != 0)
                       break;
                   ret = wc_HmacUpdate(&hmac, ctx->macSalt, ctx->macSaltSz);
                   if (ret != 0)
                       break;
                   ret = wc_HmacFinal(&hmac, out+msgSz);
               }
               break;

           default:
               ret = BAD_FUNC_ARG;
               break;
       }
    }

    if (ret == 0)
       *outSz = msgSz + digestSz;

#ifdef WOLFSSL_SMALL_STACK
    XFREE(sharedSecret, NULL, DYNAMIC_TYPE_TMP_BUFFER);
    XFREE(keys, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;
}


/* ecc decrypt with shared secret run through kdf
   ctx holds non default algos and inputs
   return 0 on success */
int wc_ecc_decrypt(ecc_key* privKey, ecc_key* pubKey, const byte* msg,
                word32 msgSz, byte* out, word32* outSz, ecEncCtx* ctx)
{
    int          ret;
    word32       blockSz;
    word32       digestSz;
    ecEncCtx     localCtx;
#ifdef WOLFSSL_SMALL_STACK
    byte*        sharedSecret;
    byte*        keys;
#else
    byte         sharedSecret[ECC_MAXSIZE];  /* 521 max size */
    byte         keys[ECC_BUFSIZE];         /* max size */
#endif
    word32       sharedSz = ECC_MAXSIZE;
    int          keysLen;
    int          encKeySz;
    int          ivSz;
    int          offset = 0;       /* in case using msg exchange */
    byte*        encKey;
    byte*        encIv;
    byte*        macKey;

    if (privKey == NULL || pubKey == NULL || msg == NULL || out == NULL ||
                           outSz  == NULL)
        return BAD_FUNC_ARG;

    if (ctx == NULL) {  /* use defaults */
        ecc_ctx_init(&localCtx, 0);
        ctx = &localCtx;
    }

    ret = ecc_get_key_sizes(ctx, &encKeySz, &ivSz, &keysLen, &digestSz,
                            &blockSz);
    if (ret != 0)
        return ret;

    if (ctx->protocol == REQ_RESP_CLIENT) {
        offset = keysLen;
        keysLen *= 2;

        if (ctx->cliSt != ecCLI_SENT_REQ)
            return BAD_ENC_STATE_E;

        ctx->cliSt = ecSRV_BAD_STATE; /* we're done no more ops allowed */
    }
    else if (ctx->protocol == REQ_RESP_SERVER) {
        if (ctx->srvSt != ecSRV_SALT_SET)
            return BAD_ENC_STATE_E;

        ctx->srvSt = ecSRV_RECV_REQ; /* only do this once */
    }

    if (keysLen > ECC_BUFSIZE) /* keys size */
        return BUFFER_E;

    if ( ((msgSz-digestSz) % blockSz) != 0)
        return BAD_PADDING_E;

    if (*outSz < (msgSz - digestSz))
        return BUFFER_E;

#ifdef WOLFSSL_SMALL_STACK
    sharedSecret = (byte*)XMALLOC(ECC_MAXSIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (sharedSecret == NULL)
        return MEMORY_E;

    keys = (byte*)XMALLOC(ECC_BUFSIZE, NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (keys == NULL) {
        XFREE(sharedSecret, NULL, DYNAMIC_TYPE_TMP_BUFFER);
        return MEMORY_E;
    }
#endif

    ret = wc_ecc_shared_secret(privKey, pubKey, sharedSecret, &sharedSz);

    if (ret == 0) {
       switch (ctx->kdfAlgo) {
           case ecHKDF_SHA256 :
               ret = wc_HKDF(SHA256, sharedSecret, sharedSz, ctx->kdfSalt,
                          ctx->kdfSaltSz, ctx->kdfInfo, ctx->kdfInfoSz,
                          keys, keysLen);
               break;

           default:
               ret = BAD_FUNC_ARG;
               break;
       }
    }

    if (ret == 0) {
       encKey = keys + offset;
       encIv  = encKey + encKeySz;
       macKey = encKey + encKeySz + ivSz;

       switch (ctx->macAlgo) {
           case ecHMAC_SHA256:
               {
                   byte verify[SHA256_DIGEST_SIZE];
                   Hmac hmac;
                   ret = wc_HmacSetKey(&hmac, SHA256, macKey, SHA256_DIGEST_SIZE);
                   if (ret != 0)
                       break;
                   ret = wc_HmacUpdate(&hmac, msg, msgSz-digestSz);
                   if (ret != 0)
                       break;
                   ret = wc_HmacUpdate(&hmac, ctx->macSalt, ctx->macSaltSz);
                   if (ret != 0)
                       break;
                   ret = wc_HmacFinal(&hmac, verify);
                   if (ret != 0)
                       break;
                   if (memcmp(verify, msg + msgSz - digestSz, digestSz) != 0)
                       ret = -1;
               }
               break;

           default:
               ret = BAD_FUNC_ARG;
               break;
       }
    }

    if (ret == 0) {
       switch (ctx->encAlgo) {
    #ifdef HAVE_AES_CBC
           case ecAES_128_CBC:
               {
                   Aes aes;
                   ret = wc_AesSetKey(&aes, encKey, KEY_SIZE_128, encIv,
                                                                AES_DECRYPTION);
                   if (ret != 0)
                       break;
                   ret = wc_AesCbcDecrypt(&aes, out, msg, msgSz-digestSz);
               }
               break;
    #endif
           default:
               ret = BAD_FUNC_ARG;
               break;
       }
    }

    if (ret == 0)
       *outSz = msgSz - digestSz;

#ifdef WOLFSSL_SMALL_STACK
    XFREE(sharedSecret, NULL, DYNAMIC_TYPE_TMP_BUFFER);
    XFREE(keys, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;
}


#endif /* HAVE_ECC_ENCRYPT */


#ifdef HAVE_COMP_KEY

/* computes the jacobi c = (a | n) (or Legendre if n is prime)
 * HAC pp. 73 Algorithm 2.149
 */
int mp_jacobi(mp_int* a, mp_int* p, int* c)
{
  mp_int   a1, p1;
  int      k, s, r, res;
  mp_digit residue;

  /* if p <= 0 return MP_VAL */
  if (mp_cmp_d(p, 0) != MP_GT) {
     return MP_VAL;
  }

  /* step 1.  if a == 0, return 0 */
  if (mp_iszero (a) == 1) {
    *c = 0;
    return MP_OKAY;
  }

  /* step 2.  if a == 1, return 1 */
  if (mp_cmp_d (a, 1) == MP_EQ) {
    *c = 1;
    return MP_OKAY;
  }

  /* default */
  s = 0;

  /* step 3.  write a = a1 * 2**k  */
  if ((res = mp_init_copy (&a1, a)) != MP_OKAY) {
    return res;
  }

  if ((res = mp_init (&p1)) != MP_OKAY) {
    mp_clear(&a1);
    return res;
  }

  /* divide out larger power of two */
  k = mp_cnt_lsb(&a1);
  res = mp_div_2d(&a1, k, &a1, NULL);

  if (res == MP_OKAY) {
    /* step 4.  if e is even set s=1 */
    if ((k & 1) == 0) {
      s = 1;
    } else {
      /* else set s=1 if p = 1/7 (mod 8) or s=-1 if p = 3/5 (mod 8) */
      residue = p->dp[0] & 7;

      if (residue == 1 || residue == 7) {
        s = 1;
      } else if (residue == 3 || residue == 5) {
        s = -1;
      }
    }

    /* step 5.  if p == 3 (mod 4) *and* a1 == 3 (mod 4) then s = -s */
    if ( ((p->dp[0] & 3) == 3) && ((a1.dp[0] & 3) == 3)) {
      s = -s;
    }
  }

  if (res == MP_OKAY) {
    /* if a1 == 1 we're done */
    if (mp_cmp_d (&a1, 1) == MP_EQ) {
      *c = s;
    } else {
      /* n1 = n mod a1 */
      res = mp_mod (p, &a1, &p1);
      if (res == MP_OKAY)
        res = mp_jacobi (&p1, &a1, &r);

      if (res == MP_OKAY)
      *c = s * r;
    }
  }

  /* done */
  mp_clear (&p1);
  mp_clear (&a1);

  return res;
}


int mp_sqrtmod_prime(mp_int* n, mp_int* prime, mp_int* ret)
{
  int res, legendre, done = 0;
  mp_int t1, C, Q, S, Z, M, T, R, two;
  mp_digit i;

  /* first handle the simple cases */
  if (mp_cmp_d(n, 0) == MP_EQ) {
    mp_zero(ret);
    return MP_OKAY;
  }
  if (mp_cmp_d(prime, 2) == MP_EQ)       return MP_VAL; /* prime must be odd */
  /* TAO removed
  if ((res = mp_jacobi(n, prime, &legendre)) != MP_OKAY)      return res;
  if (legendre == -1)  return MP_VAL; */ /* quadratic non-residue mod prime */

  if ((res = mp_init_multi(&t1, &C, &Q, &S, &Z, &M)) != MP_OKAY)
    return res;

  if ((res = mp_init_multi(&T, &R, &two, NULL, NULL, NULL))
                          != MP_OKAY) {
    mp_clear(&t1); mp_clear(&C); mp_clear(&Q); mp_clear(&S); mp_clear(&Z);
    mp_clear(&M);
    return res;
  }

  /* SPECIAL CASE: if prime mod 4 == 3
   * compute directly: res = n^(prime+1)/4 mod prime
   * Handbook of Applied Cryptography algorithm 3.36
   */
  res = mp_mod_d(prime, 4, &i);
  if (res == MP_OKAY && i == 3) {
    res = mp_add_d(prime, 1, &t1);

    if (res == MP_OKAY)
      res = mp_div_2(&t1, &t1);
    if (res == MP_OKAY)
      res = mp_div_2(&t1, &t1);
    if (res == MP_OKAY)
      res = mp_exptmod(n, &t1, prime, ret);

    done = 1;
  }

  /* NOW: TonelliShanks algorithm */

 if (res == MP_OKAY && done == 0) {

   /* factor out powers of 2 from prime-1, defining Q and S
    *                                      as: prime-1 = Q*2^S */
    res = mp_copy(prime, &Q);
    if (res == MP_OKAY)
      res = mp_sub_d(&Q, 1, &Q);
    /* Q = prime - 1 */
    if (res == MP_OKAY)
      mp_zero(&S);
    /* S = 0 */
    while (res == MP_OKAY && mp_iseven(&Q)) {
      res = mp_div_2(&Q, &Q);
      /* Q = Q / 2 */
      if (res == MP_OKAY)
        res = mp_add_d(&S, 1, &S);
        /* S = S + 1 */
    }

    /* find a Z such that the Legendre symbol (Z|prime) == -1 */
    if (res == MP_OKAY)
      res = mp_set_int(&Z, 2);
    /* Z = 2 */
    while (res == MP_OKAY) {
      res = mp_jacobi(&Z, prime, &legendre);
      if (res == MP_OKAY && legendre == -1)
        break;
      if (res == MP_OKAY)
        res = mp_add_d(&Z, 1, &Z);
        /* Z = Z + 1 */
    }

    if (res == MP_OKAY)
      res = mp_exptmod(&Z, &Q, prime, &C);
      /* C = Z ^ Q mod prime */
    if (res == MP_OKAY)
      res = mp_add_d(&Q, 1, &t1);
    if (res == MP_OKAY)
      res = mp_div_2(&t1, &t1);
      /* t1 = (Q + 1) / 2 */
    if (res == MP_OKAY)
      res = mp_exptmod(n, &t1, prime, &R);
    /* R = n ^ ((Q + 1) / 2) mod prime */
    if (res == MP_OKAY)
      res = mp_exptmod(n, &Q, prime, &T);
    /* T = n ^ Q mod prime */
    if (res == MP_OKAY)
      res = mp_copy(&S, &M);
    /* M = S */
    if (res == MP_OKAY)
      res = mp_set_int(&two, 2);

    while (res == MP_OKAY && done == 0) {
      res = mp_copy(&T, &t1);
      i = 0;
      while (res == MP_OKAY) {
        if (mp_cmp_d(&t1, 1) == MP_EQ)
            break;
        res = mp_exptmod(&t1, &two, prime, &t1);
        if (res == MP_OKAY)
          i++;
      }
      if (res == MP_OKAY && i == 0) {
        mp_copy(&R, ret);
        res = MP_OKAY;
        done = 1;
      }

      if (done == 0) {
        if (res == MP_OKAY)
          res = mp_sub_d(&M, i, &t1);
        if (res == MP_OKAY)
          res = mp_sub_d(&t1, 1, &t1);
        if (res == MP_OKAY)
          res = mp_exptmod(&two, &t1, prime, &t1);
        /* t1 = 2 ^ (M - i - 1) */
        if (res == MP_OKAY)
          res = mp_exptmod(&C, &t1, prime, &t1);
        /* t1 = C ^ (2 ^ (M - i - 1)) mod prime */
        if (res == MP_OKAY)
          res = mp_sqrmod(&t1, prime, &C);
        /* C = (t1 * t1) mod prime */
        if (res == MP_OKAY)
          res = mp_mulmod(&R, &t1, prime, &R);
        /* R = (R * t1) mod prime */
        if (res == MP_OKAY)
          res = mp_mulmod(&T, &C, prime, &T);
        /* T = (T * C) mod prime */
        if (res == MP_OKAY)
          mp_set(&M, i);
        /* M = i */
      }
    }
  }

  /* done */
  mp_clear(&t1);
  mp_clear(&C);
  mp_clear(&Q);
  mp_clear(&S);
  mp_clear(&Z);
  mp_clear(&M);
  mp_clear(&T);
  mp_clear(&R);
  mp_clear(&two);

  return res;
}


/* export public ECC key in ANSI X9.63 format compressed */
int wc_ecc_export_x963_compressed(ecc_key* key, byte* out, word32* outLen)
{
   word32 numlen;
   int    ret = MP_OKAY;

   if (key == NULL || out == NULL || outLen == NULL)
       return ECC_BAD_ARG_E;

   if (wc_ecc_is_valid_idx(key->idx) == 0) {
      return ECC_BAD_ARG_E;
   }
   numlen = key->dp->size;

   if (*outLen < (1 + numlen)) {
      *outLen = 1 + numlen;
      return BUFFER_E;
   }

   /* store first byte */
   out[0] = mp_isodd(key->pubkey.y) ? 0x03 : 0x02;

   /* pad and store x */
   XMEMSET(out+1, 0, numlen);
   ret = mp_to_unsigned_bin(key->pubkey.x,
                       out+1 + (numlen - mp_unsigned_bin_size(key->pubkey.x)));
   *outLen = 1 + numlen;
   return ret;
}


/* d = a - b (mod c) */
int mp_submod(mp_int* a, mp_int* b, mp_int* c, mp_int* d)
{
  int     res;
  mp_int  t;

  if ((res = mp_init (&t)) != MP_OKAY) {
    return res;
  }

  if ((res = mp_sub (a, b, &t)) != MP_OKAY) {
    mp_clear (&t);
    return res;
  }
  res = mp_mod (&t, c, d);
  mp_clear (&t);

  return res;
}


#endif /* HAVE_COMP_KEY */

#endif /* HAVE_ECC */

