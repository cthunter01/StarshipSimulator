# Compiled to build/<preset>/bin/shaders/<name>.spv, next to the executable (found via SDL_GetBasePath()).
# Resource bindings must follow SDL_GPU's set layout; see CLAUDE.md. spirv_reflect_test checks every shader.
StarshipSimulator_add_shaders(StarshipSimulator_shaders
    OUTPUT_DIR   ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    SOURCES
        fullscreen.vert
        grid.frag
        mesh.vert
        marker.frag
        tonemap.frag)
