/* ec.h for openssl */

#ifndef WOLFSSL_EC_H_
#define WOLFSSL_EC_H_

#include <wolfssl/openssl/ssl.h>
#include <wolfssl/openssl/bn.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Map OpenSSL NID value */
enum {
    POINT_CONVERSION_UNCOMPRESSED = 4,
    NID_secp111r1 = 0,
    NID_secp128r1 = 1,
    NID_secp160r1 = 2,
    NID_cert192 = 3,
    NID_cert224 = 4,
    NID_X9_62_prime256v1 = 5,
    NID_secp384r1 = 6,
    NID_secp521r1 = 7,
    NID_X9_62_prime_field = 100,
    OPENSSL_EC_NAMED_CURVE  = 0x001
};

struct WOLFSSL_EC_POINT {
    WOLFSSL_BIGNUM *X;
    WOLFSSL_BIGNUM *Y;
    WOLFSSL_BIGNUM *Z;

    void*          internal;     /* our ECC point */
    char           inSet;        /* internal set from external ? */
    char           exSet;        /* external set from internal ? */
};

struct WOLFSSL_EC_GROUP {
    int curve_idx; /* index of curve, used by WolfSSL as reference */
    int curve_nid; /* NID of curve, used by OpenSSL/OpenSSH as reference */
};

struct WOLFSSL_EC_KEY {
    WOLFSSL_EC_GROUP *group;
    WOLFSSL_EC_POINT *pub_key;
    WOLFSSL_BIGNUM *priv_key;

    void*          internal;     /* our ECC Key */
    char           inSet;        /* internal set from external ? */
    char           exSet;        /* external set from internal ? */
};

WOLFSSL_API
int wolfSSL_ECPoint_i2d(const WOLFSSL_EC_GROUP *curve,
                        const WOLFSSL_EC_POINT *p,
                        unsigned char *out, unsigned int *len);
WOLFSSL_API
int wolfSSL_ECPoint_d2i(unsigned char *in, unsigned int len,
                        const WOLFSSL_EC_GROUP *curve, WOLFSSL_EC_POINT *p);
WOLFSSL_API
int wolfSSL_EC_KEY_LoadDer(WOLFSSL_EC_KEY* key,
                           const unsigned char* der, int derSz);
WOLFSSL_API
void wolfSSL_EC_KEY_free(WOLFSSL_EC_KEY *key);
WOLFSSL_API
WOLFSSL_EC_POINT *wolfSSL_EC_KEY_get0_public_key(const WOLFSSL_EC_KEY *key);
WOLFSSL_API
const WOLFSSL_EC_GROUP *wolfSSL_EC_KEY_get0_group(const WOLFSSL_EC_KEY *key);
WOLFSSL_API
int wolfSSL_EC_KEY_set_private_key(WOLFSSL_EC_KEY *key,
                                   const WOLFSSL_BIGNUM *priv_key);
WOLFSSL_API
WOLFSSL_BIGNUM *wolfSSL_EC_KEY_get0_private_key(const WOLFSSL_EC_KEY *key);
WOLFSSL_API
WOLFSSL_EC_KEY *wolfSSL_EC_KEY_new_by_curve_name(int nid);
WOLFSSL_API
WOLFSSL_EC_KEY *wolfSSL_EC_KEY_new(void);
WOLFSSL_API
int wolfSSL_EC_KEY_set_group(WOLFSSL_EC_KEY *key, WOLFSSL_EC_GROUP *group);
WOLFSSL_API
int wolfSSL_EC_KEY_generate_key(WOLFSSL_EC_KEY *key);
WOLFSSL_API
void wolfSSL_EC_KEY_set_asn1_flag(WOLFSSL_EC_KEY *key, int asn1_flag);
WOLFSSL_API
int wolfSSL_EC_KEY_set_public_key(WOLFSSL_EC_KEY *key,
                                  const WOLFSSL_EC_POINT *pub);
WOLFSSL_API
void wolfSSL_EC_GROUP_set_asn1_flag(WOLFSSL_EC_GROUP *group, int flag);
WOLFSSL_API
WOLFSSL_EC_GROUP *wolfSSL_EC_GROUP_new_by_curve_name(int nid);
WOLFSSL_API
int wolfSSL_EC_GROUP_cmp(const WOLFSSL_EC_GROUP *a, const WOLFSSL_EC_GROUP *b,
                         WOLFSSL_BN_CTX *ctx);
WOLFSSL_API
int wolfSSL_EC_GROUP_get_curve_name(const WOLFSSL_EC_GROUP *group);
WOLFSSL_API
int wolfSSL_EC_GROUP_get_degree(const WOLFSSL_EC_GROUP *group);
WOLFSSL_API
int wolfSSL_EC_GROUP_get_order(const WOLFSSL_EC_GROUP *group,
                               WOLFSSL_BIGNUM *order, WOLFSSL_BN_CTX *ctx);
WOLFSSL_API
void wolfSSL_EC_GROUP_free(WOLFSSL_EC_GROUP *group);
#if defined(DEBUG_WOLFSSL) && !defined(NO_FILESYSTEM)
WOLFSSL_API
void wolfssl_EC_POINT_dump(const char *msg, const WOLFSSL_EC_POINT *p);
#endif
WOLFSSL_API
WOLFSSL_EC_POINT *wolfSSL_EC_POINT_new(const WOLFSSL_EC_GROUP *group);
WOLFSSL_API
int wolfSSL_EC_POINT_get_affine_coordinates_GFp(const WOLFSSL_EC_GROUP *group,
                                                const WOLFSSL_EC_POINT *p,
                                                WOLFSSL_BIGNUM *x,
                                                WOLFSSL_BIGNUM *y,
                                                WOLFSSL_BN_CTX *ctx);
WOLFSSL_API
int wolfSSL_EC_POINT_mul(const WOLFSSL_EC_GROUP *group, WOLFSSL_EC_POINT *r,
                         const WOLFSSL_BIGNUM *n,
                         const WOLFSSL_EC_POINT *q, const WOLFSSL_BIGNUM *m,
                         WOLFSSL_BN_CTX *ctx);
WOLFSSL_API
void wolfSSL_EC_POINT_clear_free(WOLFSSL_EC_POINT *point);
WOLFSSL_API
int wolfSSL_EC_POINT_cmp(const WOLFSSL_EC_GROUP *group,
                         const WOLFSSL_EC_POINT *a, const WOLFSSL_EC_POINT *b,
                         WOLFSSL_BN_CTX *ctx);
WOLFSSL_API
void wolfSSL_EC_POINT_free(WOLFSSL_EC_POINT *point);
WOLFSSL_API
int wolfSSL_EC_POINT_is_at_infinity(const WOLFSSL_EC_GROUP *group,
                                    const WOLFSSL_EC_POINT *a);

#define EC_KEY_free wolfSSL_EC_KEY_free
#define EC_KEY_get0_public_key wolfSSL_EC_KEY_get0_public_key
#define EC_KEY_get0_group wolfSSL_EC_KEY_get0_group
#define EC_KEY_set_private_key wolfSSL_EC_KEY_set_private_key
#define EC_KEY_get0_private_key wolfSSL_EC_KEY_get0_private_key
#define EC_KEY_new_by_curve_name wolfSSL_EC_KEY_new_by_curve_name
#define EC_KEY_set_group wolfSSL_EC_KEY_set_group
#define EC_KEY_generate_key wolfSSL_EC_KEY_generate_key
#define EC_KEY_set_asn1_flag wolfSSL_EC_KEY_set_asn1_flag
#define EC_KEY_set_public_key wolfSSL_EC_KEY_set_public_key
#define EC_KEY_new wolfSSL_EC_KEY_new

#define EC_GROUP_set_asn1_flag wolfSSL_EC_GROUP_set_asn1_flag
#define EC_GROUP_new_by_curve_name wolfSSL_EC_GROUP_new_by_curve_name
#define EC_GROUP_cmp wolfSSL_EC_GROUP_cmp
#define EC_GROUP_get_curve_name wolfSSL_EC_GROUP_get_curve_name
#define EC_GROUP_get_degree wolfSSL_EC_GROUP_get_degree
#define EC_GROUP_get_order wolfSSL_EC_GROUP_get_order
#define EC_GROUP_free wolfSSL_EC_GROUP_free

#define EC_POINT_new wolfSSL_EC_POINT_new
#define EC_POINT_get_affine_coordinates_GFp \
            wolfSSL_EC_POINT_get_affine_coordinates_GFp
#define EC_POINT_mul wolfSSL_EC_POINT_mul
#define EC_POINT_clear_free wolfSSL_EC_POINT_clear_free
#define EC_POINT_cmp wolfSSL_EC_POINT_cmp
#define EC_POINT_free wolfSSL_EC_POINT_free
#define EC_POINT_is_at_infinity wolfSSL_EC_POINT_is_at_infinity

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* header */

