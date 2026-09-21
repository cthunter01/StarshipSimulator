# Script mode (cmake -DROOT=<source dir> -P CheckLayering.cmake), run as the 'layering' test.
# The core library must stay free of windowing, GPU, UI and physics-engine code so it can be unit
# tested anywhere (including CI without a GPU): it fails if a core file includes SDL, ImGui, Jolt,
# or a physics, render, audio or app header. The physics library may use core and Jolt, but not SDL,
# ImGui or the renderer; the audio library may use core and SDL's audio, but not the renderer.

if(NOT DEFINED ROOT)
    message(FATAL_ERROR "CheckLayering.cmake requires -DROOT=<project source dir>")
endif()

function(check_layer name forbidden_regex)
    set(files ${ARGN})
    set(violations "")
    foreach(file IN LISTS files)
        file(STRINGS "${file}" lines REGEX "${forbidden_regex}")
        foreach(line IN LISTS lines)
            cmake_path(RELATIVE_PATH file BASE_DIRECTORY "${ROOT}" OUTPUT_VARIABLE relative)
            string(STRIP "${line}" line)
            list(APPEND violations "  ${relative}: ${line}")
        endforeach()
    endforeach()
    if(violations)
        list(JOIN violations "\n" report)
        message(FATAL_ERROR "The ${name} library includes headers it must not use:\n${report}")
    endif()
    list(LENGTH files count)
    message(STATUS "Layering OK: ${count} ${name} files checked")
endfunction()

file(GLOB_RECURSE core_files
    "${ROOT}/include/StarshipSimulator/core/*.h"
    "${ROOT}/src/core/*.cpp"
    "${ROOT}/src/core/*.h")
check_layer(core
    "#[ \t]*include[ \t]*[<\"](SDL3/|SDL\\.h|imgui|Jolt/|StarshipSimulator/(render|app|physics|audio)/)"
    ${core_files})

file(GLOB_RECURSE physics_files
    "${ROOT}/include/StarshipSimulator/physics/*.h"
    "${ROOT}/src/physics/*.cpp"
    "${ROOT}/src/physics/*.h")
check_layer(physics
    "#[ \t]*include[ \t]*[<\"](SDL3/|SDL\\.h|imgui|StarshipSimulator/(render|app)/)"
    ${physics_files})

file(GLOB_RECURSE audio_files
    "${ROOT}/include/StarshipSimulator/audio/*.h"
    "${ROOT}/src/audio/*.cpp"
    "${ROOT}/src/audio/*.h")
check_layer(audio
    "#[ \t]*include[ \t]*[<\"](imgui|Jolt/|StarshipSimulator/(render|app|physics)/)"
    ${audio_files})
