#pragma once

#include <pdal/pdal_export.hpp>

#include <cstdint>

#define PF_MAX_DESCRIPTION_LEN 128
#define PF_MAX_LINK_LEN 128

#ifdef __cplusplus
extern "C" {
#endif

typedef enum PF_PluginType
{
    PF_PluginType_Kernel,
    PF_PluginType_Reader,
    PF_PluginType_Filter,
    PF_PluginType_Writer
} PF_PluginType;

typedef void * (*PF_CreateFunc)();
typedef int32_t (*PF_DestroyFunc)(void *);

typedef struct PF_RegisterParams
{
    PF_CreateFunc createFunc;
    PF_DestroyFunc destroyFunc;
    char description[PF_MAX_DESCRIPTION_LEN + 1];
    char link[PF_MAX_LINK_LEN + 1];
    PF_PluginType pluginType;
} PF_RegisterParams;

typedef int32_t (*PF_ExitFunc)();
typedef PF_ExitFunc (*PF_InitFunc)();

#ifndef PDAL_DLL
  #ifdef WIN32
    #define PDAL_DLL __declspec(dllimport)
  #else
    #define PDAL_DLL
  #endif
#endif

extern
#ifdef __cplusplus
"C"
#endif
PDAL_DLL PF_ExitFunc PF_initPlugin();

#ifdef __cplusplus
}
#endif

