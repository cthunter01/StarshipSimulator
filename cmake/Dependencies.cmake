include(FetchContent)

# FIND_PACKAGE_ARGS: an installed package (find_package) wins; otherwise the source is downloaded.
# SYSTEM: the dependency's headers are system headers, so our warnings and clang-tidy skip them.
# EXCLUDE_FROM_ALL: only the parts of the dependency we link against get built.
# Third-party targets never call StarshipSimulator_configure_target(): our warnings are for our code only.

# SDL3: window, input and the SDL_GPU rendering API (Arch: pacman -S sdl3).
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)   # only used when SDL is built from source
FetchContent_Declare(SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG        release-3.4.14
    GIT_SHALLOW    TRUE
    SYSTEM
    EXCLUDE_FROM_ALL
    FIND_PACKAGE_ARGS 3.4 CONFIG)
FetchContent_MakeAvailable(SDL3)

# GLM: vector and matrix math (Arch: pacman -S glm). Link glm::glm-header-only.
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.3
    GIT_SHALLOW    TRUE
    SYSTEM
    EXCLUDE_FROM_ALL
    FIND_PACKAGE_ARGS CONFIG)
FetchContent_MakeAvailable(glm)

# toml++: reads habitat scenario files (Arch: pacman -S tomlplusplus). Used only inside core.
FetchContent_Declare(tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG        v3.4.0
    GIT_SHALLOW    TRUE
    SYSTEM
    EXCLUDE_FROM_ALL
    FIND_PACKAGE_ARGS CONFIG)
FetchContent_MakeAvailable(tomlplusplus)

# Dear ImGui: debug HUD and editor panels. It has no CMake build, so the repository is only downloaded
# and StarshipSimulator_imgui is built from its sources, with the SDL3 platform and SDL_GPU renderer backends.
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.9b
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(imgui)
add_library(StarshipSimulator_imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlgpu3.cpp)
target_include_directories(StarshipSimulator_imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_compile_definitions(StarshipSimulator_imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
target_link_libraries(StarshipSimulator_imgui PUBLIC SDL3::SDL3)

# Jolt Physics: rigid bodies, collisions and the walking character, in double precision so bodies
# stay precise kilometres from the origin. Always built from source (an installed Jolt would have
# its own configuration). Only StarshipSimulator_physics links it, PRIVATE: Jolt's target passes
# CPU flags (-mavx2 ...) and its configuration defines on to everything that links it.
# Its options are plain variables (policy CMP0077), so they don't clutter our cache.
set(DOUBLE_PRECISION ON)
set(OVERRIDE_CXX_FLAGS OFF)                  # keep our build type's flags
set(INTERPROCEDURAL_OPTIMIZATION OFF)
set(GENERATE_DEBUG_SYMBOLS OFF)
set(ENABLE_ALL_WARNINGS OFF)                 # its own -Wall -Werror
set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF)
set(CPP_RTTI_ENABLED ON)                     # we derive from its interfaces in RTTI-enabled code
set(CPP_EXCEPTIONS_ENABLED ON)
set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF)
set(PROFILER_IN_DEBUG_AND_RELEASE OFF)
set(ENABLE_OBJECT_STREAM OFF)
set(ENABLE_INSTALL OFF)
set(JPH_USE_DX12 OFF)                        # its GPU compute backends: not used
set(JPH_USE_VK OFF)
set(JPH_USE_MTL OFF)
set(JPH_USE_CPU_COMPUTE OFF)
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF)     # MSVC: the same (DLL) C runtime as everything else
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(USE_ASSERTS ON)
endif()
FetchContent_Declare(JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG        v5.6.0
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  Build
    SYSTEM
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(JoltPhysics)
# Unoptimized, Jolt is too slow for the character and terrain tiles even in Debug builds.
# (cl takes -O2 as well; the top-level CMakeLists.txt drops MSVC's /RTC1, which forbids it.)
target_compile_options(Jolt PRIVATE $<$<CONFIG:Debug>:-O2>)

if(STARSHIPSIMULATOR_BUILD_TESTS)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    # MSVC: link the same (DLL) C runtime as our targets instead of GoogleTest's static default.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.18.0
        GIT_SHALLOW    TRUE
        SYSTEM
        EXCLUDE_FROM_ALL
        FIND_PACKAGE_ARGS NAMES GTest)
    FetchContent_MakeAvailable(googletest)
endif()
