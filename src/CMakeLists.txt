# core: simulation, math, procedural generation (no SDL/GPU; unit tested)
# render: SDL_GPU renderer and ImGui layer
# app: the StarshipSimulator executable
add_subdirectory(core)
add_subdirectory(render)
add_subdirectory(app)
