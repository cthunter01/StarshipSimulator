# Compiled to build/<preset>/bin/shaders/<name>.spv, next to the executable (found via SDL_GetBasePath()).
# Resource bindings must follow SDL_GPU's set layout; see CLAUDE.md. spirv_reflect_test checks every shader.
StarshipSimulator_add_shaders(StarshipSimulator_shaders
    OUTPUT_DIR   ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/shaders
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    SOURCES
        body.frag
        body.vert
        building.frag
        bird.frag
        bird.vert
        cloud_shell.vert
        clouds.frag
        rain.frag
        fullscreen.vert
        glass.frag
        habitat.frag
        hull.frag
        landscape.frag
        landscape.vert
        marker.frag
        mesh.vert
        milkyway.frag
        mirror.frag
        mirror.vert
        planets.vert
        person.frag
        person.vert
        prop.frag
        prop.vert
        shadow.frag
        stars.frag
        stars.vert
        tonemap.frag
        transit.frag
        transit.vert
        tree.frag
        tree.vert
        water.frag)
