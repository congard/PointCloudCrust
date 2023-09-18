#ifndef POINTCLOUDCRUST_PLATFORM_H
#define POINTCLOUDCRUST_PLATFORM_H

#ifdef _WIN32
    #ifdef PCC_BUILD_LIB
        #define PCC_EXPORT __declspec(dllexport)
    #else
        #define PCC_EXPORT __declspec(dllimport)
    #endif
#else
    #define PCC_EXPORT
#endif

#endif //POINTCLOUDCRUST_PLATFORM_H
