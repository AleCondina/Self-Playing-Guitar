cmake_minimum_required(VERSION 3.22)
include(ExternalProject)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# ----Section for rebuilding CMSIS-DSP from scratch (Might need for actual chip)----#
#setting CMSIS directory variables
set(CMSISDSP ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP)
set(CMSISCORE ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/)

# #enabling compiling the CMSIS-DSP library
# add_subdirectory(${CMSISDSP}/Source bin_dsp)
#------------------------------------------------

# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Set the project name
set(CMAKE_PROJECT_NAME NUCLEO_H755ZI)

# Core project settings
project(${CMAKE_PROJECT_NAME})



# Include external project file
include("mx-generated.cmake")
