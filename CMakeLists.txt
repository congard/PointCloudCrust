cmake_minimum_required(VERSION 3.10)
project(PointCloudCrust)

set(CMAKE_CXX_STANDARD 17)

set(PointCloudCrust_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/include/PointCloudCrust/PointCloudCrust.h
        ${CMAKE_CURRENT_LIST_DIR}/include/PointCloudCrust/Vector.h
        ${CMAKE_CURRENT_LIST_DIR}/include/PointCloudCrust/types.h
        ${CMAKE_CURRENT_LIST_DIR}/include/PointCloudCrust/Points.h
        ${CMAKE_CURRENT_LIST_DIR}/src/PointCloudCrust.cpp)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

if (PCC_BUILD_SHARED)
    if (MSVC)
        set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS true)
    endif()

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
