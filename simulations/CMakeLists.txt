#=============================================================================
# CMake configuration file for Chrono-Validation Projects
#=============================================================================

cmake_minimum_required(VERSION 2.8)

project(PirProjects)

# Set location of executable
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#set(PROJECT_DATA_DIR ${PROJECT_BINARY_DIR}/bin)

#-----------------------------------------------------------------------------
# Disable some warnings
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC)
        add_definitions( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-DNOMINMAX" ) # do not use MSVC's min/max macros
        add_definitions( "-MP" ) # perform parallel builds
    endif(MSVC)
endif()

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#-----------------------------------------------------------------------------
# Invoke CMake in subdirectories
#-----------------------------------------------------------------------------

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Irrlicht Parallel Postprocess
             CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Set output data path. Return now if directory does not exist.
#--------------------------------------------------------------
set(PROJECT_DATA_DIR "" CACHE PATH "Path to data output directory")

if(NOT PROJECT_DATA_DIR)
  message(SEND_ERROR "Could not find PROJECT_DATA_DIR")
endif()

#--------------------------------------------------------------
# Add subdirectories
#--------------------------------------------------------------

add_subdirectory(shared)

# Options for configuring/building individual sets of programs
option(ENABLE_TAYLORCOUETTE "Build taylor-couette shear cell validation programs" ON)

# Propagate configuration to submodules.
if(ENABLE_TAYLORCOUETTE)
  message(STATUS "==== Validation program templates ====")
  add_subdirectory(taylor-couette)
endif()

add_DLL_copy_command("${CHRONO_DLLS}")