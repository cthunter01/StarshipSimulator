# GLSL shaders are compiled to SPIR-V at build time with glslc (Arch: pacman -S shaderc).
# SDL_GPU creates a Vulkan 1.0 instance, so shaders target Vulkan 1.0 / SPIR-V 1.0.

find_program(STARSHIPSIMULATOR_GLSLC glslc REQUIRED)

# StarshipSimulator_add_shaders(<target> OUTPUT_DIR <dir> [INCLUDE_DIRS <dirs...>] SOURCES <files...>)
# Adds a custom target <target> (part of 'all') that compiles each source (stage from its extension:
# .vert, .frag, .comp) to <OUTPUT_DIR>/<file name>.spv. #include dependencies are tracked with depfiles.
function(StarshipSimulator_add_shaders target)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "OUTPUT_DIR" "INCLUDE_DIRS;SOURCES")
    if(NOT arg_OUTPUT_DIR OR NOT arg_SOURCES)
        message(FATAL_ERROR "StarshipSimulator_add_shaders(${target}) needs OUTPUT_DIR and SOURCES")
    endif()

    set(include_args "")
    foreach(dir IN LISTS arg_INCLUDE_DIRS)
        list(APPEND include_args -I ${dir})
    endforeach()

    set(outputs "")
    foreach(source IN LISTS arg_SOURCES)
        cmake_path(ABSOLUTE_PATH source BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_VARIABLE source_path)
        cmake_path(GET source_path FILENAME name)
        set(output ${arg_OUTPUT_DIR}/${name}.spv)
        set(depfile ${CMAKE_CURRENT_BINARY_DIR}/${name}.d)
        add_custom_command(
            OUTPUT ${output}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${arg_OUTPUT_DIR}
            COMMAND ${STARSHIPSIMULATOR_GLSLC} --target-env=vulkan1.0 -Werror
                    "$<IF:$<CONFIG:Debug>,-g;-O0,-O>" ${include_args}
                    -MD -MF ${depfile} -o ${output} ${source_path}
            MAIN_DEPENDENCY ${source_path}
            DEPFILE ${depfile}
            COMMAND_EXPAND_LISTS
            VERBATIM
            COMMENT "glslc ${name}")
        list(APPEND outputs ${output})
    endforeach()
    add_custom_target(${target} ALL DEPENDS ${outputs} SOURCES ${arg_SOURCES})
endfunction()
