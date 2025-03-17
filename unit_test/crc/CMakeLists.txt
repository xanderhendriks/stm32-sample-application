cmake_minimum_required(VERSION 3.7)
project(unit_test)

# Look for installed version of CppUTest
if(DEFINED ENV{CPPUTEST_HOME})
    message(STATUS "Using CppUTest home: $ENV{CPPUTEST_HOME}")
    set(CPPUTEST_INCLUDE_DIRS $ENV{CPPUTEST_HOME}/include)
    set(CPPUTEST_LIBRARIES $ENV{CPPUTEST_HOME}/lib)
    set(CPPUTEST_LDFLAGS CppUTest CppUTestExt)
else()
    find_package(PkgConfig REQUIRED)
    pkg_search_module(CPPUTEST REQUIRED cpputest>=3.8)
    message(STATUS "Found CppUTest version ${CPPUTEST_VERSION}")
endif()

add_compile_options(-Wall -Werror -Wno-format)

set(APP_NAME application)
set(APP_LIB_NAME application)

# include source, mocks and tests build instructions
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../sources sources)
add_subdirectory(mocks)
add_subdirectory(tests)
