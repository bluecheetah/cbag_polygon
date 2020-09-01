# Version 3.9 required because of C++17 / LTO support.
cmake_minimum_required(VERSION 3.9)

project(cbag_polygon)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# control compiler warnings
add_compile_options(-fexceptions)
add_compile_options(-fmax-errors=2)
add_compile_options(-g)
add_compile_options(-Wall)
add_compile_options(-Wno-delete-non-virtual-dtor)
# add_compile_options(-Wno-logical-op-parentheses)
# add_compile_options(-Wno-new-returns-null)
# set optimzation level for release
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# generate compilation commands file for emacs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# prefer pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)

# make sure linker raise errors if shared library has undefined symbols
# this makes it a lot easier to debug
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
# add rpaths to the final install executable
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# check for link time optimization
include(CheckIPOSupported)

check_ipo_supported(RESULT ipo_result)
if(ipo_result AND CMAKE_BUILD_TYPE MATCHES Release)
  message("Enabling interprocedural optimization")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  message("Interprocedural optimization disabled")
endif()

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Include fmt library
find_package(fmt REQUIRED CONFIG)

# Include yaml-cpp
find_package(yaml-cpp REQUIRED CONFIG)

add_library(cbag_polygon INTERFACE)
target_include_directories(cbag_polygon INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
