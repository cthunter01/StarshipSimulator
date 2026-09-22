# Rigid-body physics in the habitat's rotating frame (Jolt): terrain tiles, buildings, props and
# the player's body. Jolt stays private: its target passes CPU flags and configuration defines on.
add_library(StarshipSimulator_physics STATIC
    PhysicsWorld.cpp)
add_library(StarshipSimulator::physics ALIAS StarshipSimulator_physics)
target_link_libraries(StarshipSimulator_physics PUBLIC StarshipSimulator::core PRIVATE Jolt)
StarshipSimulator_configure_target(StarshipSimulator_physics)
