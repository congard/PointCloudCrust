cmake_minimum_required(VERSION 3.10)
project(pcc_test)

set(CMAKE_CXX_STANDARD 17)

set(PCC_BUILD_SHARED ON)

include(../CMakeLists.txt)

# add_compile_options(-O3)

add_executable(pcc_test main.cpp)
target_link_libraries(pcc_test pthread pcc)
