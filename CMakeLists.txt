project(crsGA)

cmake_minimum_required(VERSION 3.19.3)

if(COMMAND CMAKE_POLICY)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND CMAKE_POLICY)

set(MAJOR_VERSION 0)
set(MINOR_VERSION 1)
set(PATCH_VERSION 0)
set(VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")
include(functions)

# setup project globals
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(CRSGA_BUILD_APPLICATIONS "Builds example applications of Genetic Algorithm framework" OFF)

find_package(Threads)

add_subdirectory(src)
add_subdirectory(examples)
if (CRSGA_BUILD_APPLICATIONS)
    # Applications depend on third party packages
    setup_conan()
    add_subdirectory(applications)
    # Install common data for apps
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/data 
            DESTINATION share/crsGA)
endif()

include(InstallRequiredSystemLibraries)

