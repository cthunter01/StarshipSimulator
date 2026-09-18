# Script mode (cmake -DROOT=<source dir> -P CheckLayering.cmake), run as the 'layering' test.
# The core library must stay free of windowing, GPU and UI code so it can be unit tested anywhere
# (including CI without a GPU). Fails if a core file includes SDL, ImGui, or a render/app header.

if(NOT DEFINED ROOT)
    message(FATAL_ERROR "CheckLayering.cmake requires -DROOT=<project source dir>")
endif()

file(GLOB_RECURSE core_files
    "${ROOT}/include/StarshipSimulator/core/*.h"
    "${ROOT}/src/core/*.cpp"
    "${ROOT}/src/core/*.h")

set(forbidden_regex "#[ \t]*include[ \t]*[<\"](SDL3/|SDL\\.h|imgui|StarshipSimulator/(render|app)/)")
set(violations "")
foreach(file IN LISTS core_files)
    file(STRINGS "${file}" lines REGEX "${forbidden_regex}")
    foreach(line IN LISTS lines)
        cmake_path(RELATIVE_PATH file BASE_DIRECTORY "${ROOT}" OUTPUT_VARIABLE relative)
        string(STRIP "${line}" line)
        list(APPEND violations "  ${relative}: ${line}")
    endforeach()
endforeach()

if(violations)
    list(JOIN violations "\n" report)
    message(FATAL_ERROR "The core library must not include SDL, ImGui, render or app headers:\n${report}")
endif()
list(LENGTH core_files count)
message(STATUS "Layering OK: ${count} core files checked")
