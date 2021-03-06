
#if !defined(_STDLINUX_)
#define _STDLINUX_

#include <cstddef>
#include <stdint.h>
#include "types.h"
#include "tchar.h"
#include <assert.h>
#include <math.h>
#include <malloc.h>


#ifndef ASSERT
#define ASSERT(x) assert(x)
#endif

typedef unsigned int uint;
typedef unsigned char byte;

#ifndef _tcsclen
#define _tcsclen(x) _tcslen(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif


#endif
