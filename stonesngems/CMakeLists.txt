project(stonesngems)

cmake_minimum_required (VERSION 3.13)

# Set flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Testing
enable_testing()

set(STONESNGEMS_SOURCES
    src/definitions.h
    src/stonesngems_base.cpp 
    src/stonesngems_base.h 
    src/util.cpp 
    src/util.h
)

# Build library
add_library(stonesngems STATIC ${STONESNGEMS_SOURCES})
target_compile_options(stonesngems PRIVATE 
    -Wall -Wextra 
    $<$<CONFIG:RELEASE>:-O3> $<$<CONFIG:RELEASE>:-DNDEBUG>
    $<$<CONFIG:DEBUG>:-g> $<$<CONFIG:DEBUG>:-DDEBUG> $<$<CONFIG:DEBUG>:-pg> $<$<CONFIG:DEBUG>:-O1>
)

target_include_directories(stonesngems PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

# Build child targets
add_subdirectory(test)
