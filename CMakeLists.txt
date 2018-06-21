cmake_minimum_required(VERSION 2.8)
project(winsun)

# Suppress warning about having target named test
cmake_policy(SET CMP0037 OLD)
# Enable C++11
# These lines must appear before any calls to add_library or add_executable
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# Specify include directories here
# Also add paths to NetBeans Code Assistance in project properties
include_directories(
.
)

# Specify hardware dependent source files here
#add_library(Core STATIC
#)
# Specify library dependencies here
set(LIBRARIES
    pthread
    zpp
    iw
    udev
    tar
    ${ImageMagick_LIBRARIES}
    ${ZLIB_LIBRARIES}
    drm
)

# Specify hardware dependent source files here
add_library(Hardware STATIC EXCLUDE_FROM_ALL
    I2C_Device.cpp
)
# Specify mock hardware source file here
add_library(MockHardware STATIC EXCLUDE_FROM_ALL
    mock_hardware/NamedPipeResource.cpp
    mock_hardware/NamedPipeI2C_Device.cpp
    mock_hardware/HardwareFactory.cpp
    mock_hardware/ImageWritingFrameBuffer.cpp
)

add_executable(winsun
    main.cpp
)

#list(INSERT LIBRARIES 0 Core)
# Specify compilation flags here
set(CMAKE_CXX_FLAGS_DEBUG    "-g3 -gdwarf-2")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")
set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")

# Specify compile time definitions here
# DIRECTORY scope applies definitions to all sources under the directory that
# contains this file
#set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS
#    # Define DEBUG for debug and coverage builds
#    $<$<CONFIG:Debug>:DEBUG>
#    $<$<CONFIG:Coverage>:DEBUG>
#
#    # ImageMagick configuration
#   # MAGICKCORE_HDRI_ENABLE=0
#   # MAGICKCORE_QUANTUM_DEPTH=16
#)
#set(USE_MOCK_HARDWARE FALSE CACHE BOOL "Enable to build with mock hardware")
set(USE_MOCK_HARDWARE FALSE CACHE BOOL "Enable to build with mock hardware")

if (${USE_MOCK_HARDWARE})
    message(STATUS "Configured to build with mock hardware")
    list(INSERT LIBRARIES 0 MockHardware)
else()
    message(STATUS "Configured to build with actual hardware")
    list(INSERT LIBRARIES 0 Hardware)
endif()

#list(INSERT LIBRARIES 0 Core)

target_link_libraries(winsun ${LIBRARIES})
install(TARGETS winsun RUNTIME DESTINATION /usr/local/bin)

