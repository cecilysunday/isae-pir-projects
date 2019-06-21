#=============================================================================
#
# CMake configuration file for the main ChronoValidation library.
#
#=============================================================================

message(STATUS "==== Chrono Validation Testing - Shared Library ====")

#-----------------------------------------------------------------------------
# Collect files in the main ChronoValidation library
#-----------------------------------------------------------------------------

set(ChronoValidation_SHARED_SOURCES
    ProjDataPath.cpp
    ProjLog.cpp
    ProjShape.cpp
    ProjWriteData.cpp
    )

set(ChronoValidation_SHARED_HEADERS
    ProjDataPath.h
    ProjLog.h
    ProjShape.h
    ProjWriteData.h
    )

source_group(ChronoValidation_SHARED FILES
    ${ChronoValidation_SHARED_SOURCES}
    ${ChronoValidation_SHARED_HEADERS})

# Collect all source and header files for the ChronoValidation library
set(ChronoValidation_SHARED_FILES
    ${ChronoValidation_SHARED_SOURCES}
    ${ChronoValidation_SHARED_HEADERS}
    )

#-----------------------------------------------------------------------------
# Includes for the main library
#-----------------------------------------------------------------------------

# Make PROJ_INCLUDES visible in parent scope
set(PROJ_SHARED_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Add the ChronoValidation_shared library
#-----------------------------------------------------------------------------

message(STATUS "${CHRONO_LIBRARIES}")

add_library(ChronoValidation_shared ${ChronoValidation_SHARED_FILES})

set_target_properties(ChronoValidation_shared PROPERTIES
    COMPILE_FLAGS "${COMPILE_FLAGS}"
    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"; SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"; PROJECT_DATA_DIR=\"${PROJECT_DATA_DIR}\""
    LINK_FLAGS "${COMPILE_FLAGS} ${CHRONO_LINKER_FLAGS}"
  )

target_include_directories(ChronoValidation_shared PRIVATE ${CHRONO_INCLUDE_DIRS})
target_link_libraries(ChronoValidation_shared ${CHRONO_LIBRARIES})

if(UNIX)
    target_link_libraries(ChronoValidation_shared ${CHRONO_LIBRARIES} stdc++fs)
endif()