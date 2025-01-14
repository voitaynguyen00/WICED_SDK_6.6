#include "sha2.h"
#include "multprecision.h"
#include "ecc_pp.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define HASH_SIZE       32

extern BOOL32 ecdsa_verify(unsigned char* digest, unsigned char* signature, Point* key);


int main (int argc, char *argv[])
{
	sha2_context sha2_ctx;
    unsigned char *buf;
    unsigned char testDigest[KEY_LENGTH_BYTES * 2];
    unsigned char *pSignature;
    unsigned char signature[KEY_LENGTH_BYTES*2];
    unsigned char pubKey[KEY_LENGTH_BYTES * 2];
    FILE *fPatch;
    FILE *fPublicKey;
    int fileSize;
    int totalSize;
    int i = 0;

    if(argc != 2)
    {
        printf("usage: %s <patch.signed>\n", argv[0]);
        return 0;
    }

    // open public key file
    if ((fopen_s(&fPublicKey, "ecdsa256_key.pub.bin", "rb")) != 0)
    {
        printf("can not open output signature file %s\n", argv[2]);
        return -1;
    }
    else
    {
        // read the key
        totalSize = (int)fread(pubKey, 1, KEY_LENGTH_BYTES * 2, fPublicKey);
        fclose(fPublicKey);

        // check key length
        if (totalSize != KEY_LENGTH_BYTES * 2)
        {
            printf("public key legnth incorrect\n");
            return -1;
        }
    }

    // open patch file
    if ((fopen_s(&fPatch, argv[1], "rb")) != 0)
    {
        printf("can not open patch file %s\n", argv[1]);
        return -1;
    }

    // read patch size
    fseek(fPatch, 0, SEEK_END);
    fileSize = ftell(fPatch);
    rewind(fPatch);

    // allocate memory and read the file
    if ((buf = (unsigned char *)malloc(fileSize)) == NULL)
    {
        printf("no memory\n");
        return -1;
    }

    totalSize = (int)fread(buf, fileSize, 1,  fPatch);
    fclose(fPatch);

    // everything is ready. Buffer starts at location buf.  The length is 
    // totalSize.  The last 64 bytes is the signature.

    // initialize sha256 context
    sha2_starts(&sha2_ctx, 0);

    // generate digest, last bytes are the signature, do not include.
    sha2_update(&sha2_ctx, buf, fileSize - (KEY_LENGTH_BYTES * 2));
    sha2_finish(&sha2_ctx, testDigest);

    // signature is the last (KEY_LENGTH_BYTES * 2) bytes of the file
    pSignature = &buf[fileSize - (KEY_LENGTH_BYTES * 2)];
    for (i = 0; i < KEY_LENGTH_BYTES * 2; ++i)
        signature[i] = *pSignature++;

    if (!ecdsa_verify(testDigest, signature, (Point *)pubKey))
    {
        printf("verify failure\n");
    }
    else
    {
        printf("verify success\n");
    }
    return 0;
}


