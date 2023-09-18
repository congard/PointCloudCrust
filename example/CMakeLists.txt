cmake_minimum_required(VERSION 3.10)
project(pcc_test)

set(CMAKE_CXX_STANDARD 17)

set(PCC_BUILD_SHARED ON)

add_subdirectory(../ pcc)

# add_compile_options(-O3)

if (NOT WIN32)
    set(pcc_test_link_extra pthread)
endif()

add_executable(pcc_test main.cpp)

target_link_libraries(pcc_test pcc ${pcc_test_link_extra})
target_include_directories(pcc_test PRIVATE ../include)
