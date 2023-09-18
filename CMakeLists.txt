cmake_minimum_required(VERSION 3.10)
project(PointCloudCrust)

set(CMAKE_CXX_STANDARD 17)

set(PointCloudCrust_SOURCES
        include/PointCloudCrust/PointCloudCrust.h
        include/PointCloudCrust/Vector.h
        include/PointCloudCrust/types.h
        include/PointCloudCrust/Points.h
        src/PointCloudCrust.cpp
        include/PointCloudCrust/platform.h)

include_directories(include)

if (PCC_BUILD_SHARED)
    add_library(pcc SHARED ${PointCloudCrust_SOURCES})
elseif (PCC_BUILD_STATIC)
    add_library(pcc STATIC ${PointCloudCrust_SOURCES})
endif()

if (PCC_BUILD_SHARED OR PCC_BUILD_STATIC AND NOT PCC_DEBUG)
    if (MSVC)
        target_compile_options(pcc PRIVATE /O2)
    else()
        target_compile_options(pcc PRIVATE -O3)
    endif()
endif()

if (PCC_DOUBLE_PRECISION)
    add_compile_definitions(POINTCLOUDCRUST_DOUBLE_PRECISION)
endif()

if (WIN32)
    target_compile_definitions(pcc PRIVATE PCC_BUILD_LIB)
endif()
