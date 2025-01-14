/* mem_track.h
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



#ifndef WOLFSSL_MEM_TRACK_H
#define WOLFSSL_MEM_TRACK_H

#ifdef USE_WOLFSSL_MEMORY

    #include "wolfssl/wolfcrypt/logging.h"

    typedef struct memoryStats {
        size_t totalAllocs;     /* number of allocations */
        size_t totalBytes;      /* total number of bytes allocated */
        size_t peakBytes;       /* concurrent max bytes */
        size_t currentBytes;    /* total current bytes in use */
    } memoryStats;

    typedef struct memHint {
        size_t thisSize;      /* size of this memory */
        void*  thisMemory;    /* actual memory for user */
    } memHint;

    typedef struct memoryTrack {
        union {
            memHint hint;
            byte    alignit[16];   /* make sure we have strong alignment */
        } u;
    } memoryTrack;

    #if defined(WOLFSSL_TRACK_MEMORY)
        #define DO_MEM_STATS
        static memoryStats ourMemStats;
    #endif

    static INLINE void* TrackMalloc(size_t sz)
    {
        memoryTrack* mt;

        if (sz == 0)
            return NULL;

        mt = (memoryTrack*)malloc(sizeof(memoryTrack) + sz);
        if (mt == NULL)
            return NULL;

        mt->u.hint.thisSize   = sz;
        mt->u.hint.thisMemory = (byte*)mt + sizeof(memoryTrack);

#ifdef DO_MEM_STATS
        ourMemStats.totalAllocs++;
        ourMemStats.totalBytes   += sz;
        ourMemStats.currentBytes += sz;
        if (ourMemStats.currentBytes > ourMemStats.peakBytes)
            ourMemStats.peakBytes = ourMemStats.currentBytes;
#endif

        return mt->u.hint.thisMemory;
    }


    static INLINE void TrackFree(void* ptr)
    {
        memoryTrack* mt;

        if (ptr == NULL) {
            return;
        }

        mt = (memoryTrack*)ptr;
        --mt;   /* same as minus sizeof(memoryTrack), removes header */

#ifdef DO_MEM_STATS
        ourMemStats.currentBytes -= mt->u.hint.thisSize;
#endif

        free(mt);
    }


    static INLINE void* TrackRealloc(void* ptr, size_t sz)
    {
        void* ret = TrackMalloc(sz);

        if (ptr) {
            /* if realloc is bigger, don't overread old ptr */
            memoryTrack* mt = (memoryTrack*)ptr;
            --mt;  /* same as minus sizeof(memoryTrack), removes header */

            if (mt->u.hint.thisSize < sz)
                sz = mt->u.hint.thisSize;
        }

        if (ret && ptr)
            memcpy(ret, ptr, sz);

        if (ret)
            TrackFree(ptr);

        return ret;
    }

    static INLINE int InitMemoryTracker(void)
    {
        int ret = wolfSSL_SetAllocators(TrackMalloc, TrackFree, TrackRealloc);
        if (ret < 0) {
            printf("wolfSSL SetAllocators failed for track memory\n");
            return ret;
        }

    #ifdef DO_MEM_STATS
        ourMemStats.totalAllocs  = 0;
        ourMemStats.totalBytes   = 0;
        ourMemStats.peakBytes    = 0;
        ourMemStats.currentBytes = 0;
    #endif
        
        return ret;
    }

    static INLINE void ShowMemoryTracker(void)
    {
    #ifdef DO_MEM_STATS
        printf("total   Allocs = %9lu\n",
                                       (unsigned long)ourMemStats.totalAllocs);
        printf("total   Bytes  = %9lu\n",
                                       (unsigned long)ourMemStats.totalBytes);
        printf("peak    Bytes  = %9lu\n",
                                       (unsigned long)ourMemStats.peakBytes);
        printf("current Bytes  = %9lu\n",
                                       (unsigned long)ourMemStats.currentBytes);
    #endif
    }

#endif /* USE_WOLFSSL_MEMORY */

#endif /* WOLFSSL_MEM_TRACK_H */
    
