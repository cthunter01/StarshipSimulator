# core: simulation, math, procedural generation (no SDL/GPU; unit tested)
# physics: rigid bodies and the player's body (Jolt), in the rotating frame; unit tested
# render: SDL_GPU renderer and ImGui layer
# audio: the synthesised soundscape played through SDL
# app: the StarshipSimulator executable
add_subdirectory(core)
add_subdirectory(physics)
add_subdirectory(render)
add_subdirectory(audio)
add_subdirectory(app)
