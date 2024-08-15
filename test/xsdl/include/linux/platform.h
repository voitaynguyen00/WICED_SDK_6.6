#ifndef PLATFORM_H_DEFINED

#define PLATFORM_H_DEFINED

#define PROCESSOR_IS_LITTLE_ENDIAN


#ifndef UINT8_t
typedef unsigned    char            UINT8_t;
#endif

#ifndef INT8_t
typedef             char            INT8_t;
#endif

#ifndef UINT16_t
typedef unsigned    short           UINT16_t;
#endif

#ifndef INT16_t
typedef             short           INT16_t;
#endif

#ifndef UINT32_t
typedef unsigned    int             UINT32_t;
#endif

#ifndef INT32_t
typedef             int             INT32_t;
#endif

#ifndef UINT64_t
typedef unsigned    long long       UINT64_t;
#endif

#ifndef INT64_t
typedef             long long       INT64_t;
#endif


#define         PRINTF_INT64_MODIFIER   "ll"




inline char * STRING_TO_UPPER (char * s) /* mhwong */
{
    char* p = s;
    while (*s)
    {
        if ((*s >= 'a') && (*s <= 'z'))
        {
            *s -= ('a' - 'A');
        }
        s++;
    }
    return p;
}


#define        STRING_COMPARE_NOCASE       strcasecmp



#endif


