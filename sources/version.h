//! @file version.h
//! @brief Macro definitions for version number
#ifndef VERSION_H
#define VERSION_H

// Standard includes
#include <stdint.h>

#define DO_EXPAND(VAL) VAL##1
#define EXPAND(VAL)    DO_EXPAND(VAL)

// VERSION_* is taken from the ENV_VERSION_* through MAKE_VERSION_* in makefile.defs
#if !defined(VERSION_MAJOR) || (EXPAND(VERSION_MAJOR) == 1)
#undef VERSION_MAJOR
#define VERSION_MAJOR 0
#endif

#if !defined(VERSION_MINOR) || (EXPAND(VERSION_MINOR) == 1)
#undef VERSION_MINOR
#define VERSION_MINOR 0
#endif

#if !defined(VERSION_BUGFIX) || (EXPAND(VERSION_BUGFIX) == 1)
#undef VERSION_BUGFIX
#define VERSION_BUGFIX 0
#endif

#if !defined(SHORT_GIT_HASH)
#define SHORT_GIT_HASH ""
#endif

#define SHORT_GIT_HASH_STRING \
    ((SHORT_GIT_HASH != 0) && (sizeof(SHORT_GIT_HASH) == 11) ? SHORT_GIT_HASH : "debugbuild")

#ifndef IMAGE_ID
#error "define IMAGE_ID in the project properties -> Symbols"
#endif

typedef struct
{
    uint8_t  imageId;
    uint16_t major;
    uint16_t minor;
    uint16_t bugfix;
    char     gitHash[11];  // 10 bytes of hash plus terminating /0

} VersionInfo_t;
#endif
