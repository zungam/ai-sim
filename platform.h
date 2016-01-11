#ifndef _platform_h_
#define _platform_h_
#include <stdint.h>
typedef float       r32;
typedef uint64_t    u64;
typedef uint32_t    u32;
typedef uint16_t    u16;
typedef uint8_t     u08;
typedef int32_t     s32;
typedef int16_t     s16;
typedef int8_t      s08;

#define global static
#define persist static

struct VideoMode
{
    int width;
    int height;
    int gl_major;
    int gl_minor;
    int double_buffer;
    int depth_bits;
    int stencil_bits;
    int multisamples;

    // 0 for immediate updates, 1 for updates synchronized with the
    // vertical retrace. If the system supports it, you may
    // specify -1 to allow late swaps to happen immediately
    // instead of waiting for the next retrace.
    int swap_interval;
};

// The random numbers are seeded on startup.
// They are not uniform or otherwise good quality
// random numbers.

// return: Pseudo-random real number in the range [0, 1]
float frand();

// return: Pseudo-random integer in the range [0, 64]
int random_0_64();

#endif
