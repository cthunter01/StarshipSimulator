# Sound: plays the core library's synthesised soundscape through SDL's audio device.
add_library(StarshipSimulator_audio STATIC
    audio_device.cpp)
add_library(StarshipSimulator::audio ALIAS StarshipSimulator_audio)
target_link_libraries(StarshipSimulator_audio PUBLIC StarshipSimulator::core PRIVATE SDL3::SDL3)
StarshipSimulator_configure_target(StarshipSimulator_audio)
