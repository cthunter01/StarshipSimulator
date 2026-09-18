# StarshipSimulator

Walk around inside space habitats (O'Neill cylinders, Bishop rings, starships), look out the windows or up at
the land overhead, and see an astronomically accurate sky. The goal: help people picture a positive future in
space that we could actually build, with the real physics of spin gravity (Coriolis drift, gravity that fades
toward the axis) and real stars and planets outside.

**Status: M0 (foundation).** A Vulkan renderer (SDL_GPU) with double-precision, camera-relative rendering and a
test world: walk or fly over a metric grid and teleport 1000 km away without losing precision. Next up, M1:
walking inside an O'Neill cylinder.

## Requirements
- Linux with a Vulkan GPU
- CMake 3.28+ and Ninja
- GCC 14+ or Clang 18+ (C++23)
- SDL 3.4, GLM 1.0, glslc (shaderc)

On Arch Linux:
```sh
sudo pacman -S --needed cmake ninja clang sdl3 glm shaderc
# optional: Vulkan validation layers (Debug builds), ccache, RenderDoc
sudo pacman -S --needed vulkan-validation-layers ccache renderdoc
```
Dear ImGui and GoogleTest are downloaded at configure time (installed GoogleTest, SDL3 and GLM are used if found).

## Build and run
```sh
cmake --workflow --preset dev          # configure + build + test, Clang Debug
./build/clang-debug/bin/StarshipSimulator
```

Controls: click the view to capture the mouse (Esc releases it). WASD to move, Shift to run, Space to jump
(walk) or rise (fly), Ctrl to descend, F toggles walk/fly, mouse wheel sets fly speed, F1 toggles the HUD,
F5 reloads shaders, F12 saves a screenshot to `~/.local/share/StarshipSimulator/screenshots/`.

Useful options (`--help` lists all):
```sh
StarshipSimulator --size 1920x1080 --camera 1000000,0,1.7,0,0   # start 1000 km out
StarshipSimulator --capture shot.png --capture-ui                # render, save a PNG, exit
```

| Preset | What it is |
| --- | --- |
| `clang-debug`, `clang-release`, `gcc-debug`, `gcc-release` | Everyday builds |
| `asan` | Clang Debug with AddressSanitizer + UndefinedBehaviorSanitizer |
| `tsan` | Clang RelWithDebInfo with ThreadSanitizer |
| `tidy` | Clang Debug running clang-tidy on every file; findings are errors |
| `coverage` | `cmake --workflow --preset coverage` writes `build/coverage/coverage/html/index.html` |
| `ci-gcc`, `ci-clang` | Release builds with warnings as errors, as run in CI |

Each workflow preset (`dev`, `ci-gcc`, `ci-clang`, `asan`, `tsan`, `tidy`, `coverage`) configures, builds and
tests in one command. Separate steps: `cmake --preset <p>`, `cmake --build --preset <p>`, `ctest --preset <p>`.

## Code layout
- `src/core`: simulation, math and procedural generation; no graphics dependencies, fully unit tested
- `src/render`: the SDL_GPU renderer and the ImGui layer
- `src/app`: the executable (main loop, input, HUD)
- `shaders`: GLSL, compiled to SPIR-V at build time

API docs: `cmake --build --preset clang-debug --target docs`, then open `build/clang-debug/docs/html/index.html`.
