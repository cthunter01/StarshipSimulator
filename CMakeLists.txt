cmake_minimum_required(VERSION 3.28)   # FetchContent SYSTEM + EXCLUDE_FROM_ALL + FIND_PACKAGE_ARGS
project(StarshipSimulator VERSION 0.1.0 DESCRIPTION "Walk around inside space habitats and starships under an accurate sky" LANGUAGES C CXX)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not allowed. Use a preset: cmake --preset clang-debug")
endif()

# Presets set the build type; this covers a bare `cmake -B build`. A parent project keeps its own choice.
if(PROJECT_IS_TOP_LEVEL AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)     # no C++ modules; remove this line to use them
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

option(STARSHIPSIMULATOR_BUILD_TESTS "Build the unit tests" ${PROJECT_IS_TOP_LEVEL})
option(STARSHIPSIMULATOR_BUILD_DOCS "Add a 'docs' target when Doxygen is found" ${PROJECT_IS_TOP_LEVEL})
option(STARSHIPSIMULATOR_WARNINGS_AS_ERRORS "Treat compiler and clang-tidy warnings as errors" OFF)
option(STARSHIPSIMULATOR_ENABLE_COVERAGE "Clang source-based coverage and a 'coverage' target" OFF)
option(STARSHIPSIMULATOR_ENABLE_CLANG_TIDY "Run clang-tidy on our sources while compiling" OFF)
option(STARSHIPSIMULATOR_ENABLE_IPO "Link-time optimisation for our targets" OFF)
set(STARSHIPSIMULATOR_SANITIZERS "" CACHE STRING "Sanitizers for our targets, e.g. address;undefined or thread")

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM AND NOT CMAKE_CXX_COMPILER_LAUNCHER)   # a launcher the user chose wins
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# Point clangd (in any editor) at the most recently configured preset.
# Windows allows symlinks only in Developer Mode (or as admin), so a failure is a note, not an error.
if(PROJECT_IS_TOP_LEVEL)
    file(CREATE_LINK ${PROJECT_BINARY_DIR}/compile_commands.json
         ${PROJECT_SOURCE_DIR}/compile_commands.json RESULT link_result SYMBOLIC)
    if(NOT link_result EQUAL 0)
        message(STATUS "Could not link compile_commands.json into the source tree (${link_result}). "
                       "On Windows, turn on Developer Mode to allow symlinks.")
    endif()
endif()

# Debug builds still optimise world generation and Jolt (-O2 on those files, see src/core and
# cmake/Dependencies.cmake), which MSVC's run-time checks (/RTC1) do not allow.
if(MSVC)
    string(REPLACE "/RTC1" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
    string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
endif()

# Project modules are included by path, so a parent's CMAKE_MODULE_PATH cannot replace them.
include(${PROJECT_SOURCE_DIR}/cmake/ProjectOptions.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/Dependencies.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/Shaders.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/SkyData.cmake)

add_subdirectory(third_party)
add_subdirectory(shaders)
add_subdirectory(src)

# Runtime data (scenario presets, ...), copied next to the executable: build/<preset>/bin/data.
add_custom_target(StarshipSimulator_data ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
            ${PROJECT_SOURCE_DIR}/data ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/data
    COMMENT "Copying runtime data")

if(STARSHIPSIMULATOR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
if(STARSHIPSIMULATOR_ENABLE_COVERAGE)
    include(${PROJECT_SOURCE_DIR}/cmake/Coverage.cmake)
endif()
if(STARSHIPSIMULATOR_BUILD_DOCS)
    include(${PROJECT_SOURCE_DIR}/cmake/Docs.cmake)
endif()
