cmake_minimum_required(VERSION 3.12)

project(QuadControl_BVP LANGUAGES CXX)

if(UNIX AND NOT APPLE)
    set(CMAKE_C_COMPILER "/usr/bin/gcc")
    set(CMAKE_CXX_COMPILER "/usr/bin/g++")
elseif(APPLE)
    set(CMAKE_C_COMPILER "clang")
    set(CMAKE_CXX_COMPILER "clang++")
else()
    message(WARNING "Only support LINUX and APPLE for now.")
endif()

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,${CMAKE_SOURCE_DIR}/build")

find_package(casadi REQUIRED)
find_package(Threads REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_LOADED)
    message(STATUS "Compiler Path: ${CMAKE_CXX_COMPILER}")
    message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
    message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
endif()

set(LIBRARY_DIRS /usr/local/lib)
if(UNIX AND NOT APPLE)
    set(IPOPT_INCLUDE_DIRS /usr/include/coin)
elseif(APPLE)
    set(IPOPT_INCLUDE_DIRS /usr/local/include/coin-or)
endif()

set(CASADI_INCLUDE_DIR /usr/local/include/casadi)

find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)

if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/Toolbox
    SYSTEM ${IPOPT_INCLUDE_DIRS}
    SYSTEM ${CASADI_INCLUDE_DIR}
)

link_directories(${LIBRARY_DIRS})

add_executable(multiple_shooting multipleShooting.cpp
        ${CMAKE_SOURCE_DIR}/Toolbox/FMU_CasadiWrapper.cpp
        ${CMAKE_SOURCE_DIR}/Toolbox/FMU_CasadiCallback.cpp)
target_link_libraries(multiple_shooting PRIVATE ${CASADI_LIBRARIES} Threads::Threads)

if(UNIX AND NOT APPLE)
    target_link_libraries(multiple_shooting PRIVATE dl)
endif()

if(MSVC)
  target_compile_options(multiple_shooting PRIVATE /W3)
else()
  target_compile_options(multiple_shooting PRIVATE -Wall -Wextra -Wno-subobject-linkage -Wno-unused-function)
endif()
