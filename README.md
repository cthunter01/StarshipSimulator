# StarshipSimulator

Walk around inside space habitats (O'Neill cylinders, Bishop rings, starships), look out the windows or up at
the land overhead, and see an astronomically accurate sky. The goal: help people picture a positive future in
space that we could actually build, with the real physics of spin gravity (Coriolis drift, gravity that fades
toward the axis) and real stars and planets outside.

**Status: M1: first steps inside Island Three.** Walk the valleys of Gerard O'Neill's 8 km wide, 32 km long
cylinder: farmland curving up overhead through blue haze, three window strips with the stars sweeping past every
two minutes, three mirrors bringing in the sunlight, mountain ramps at one end and a dome at the other. The
physics is real: gravity comes from spin (and fades as you climb toward the axis), jumps and thrown balls drift
from the Coriolis force, and the mirror angle sets the time of day. A habitat editor lets you design your own
cylinder and save it as a small TOML file. Next up, M2: the real sky (star catalog, Sun, Earth and Moon).

## Requirements
- Linux with a Vulkan GPU
- CMake 3.28+ and Ninja
- GCC 14+ or Clang 18+ (C++23)
- SDL 3.4, GLM 1.0, toml++ 3.4, glslc (shaderc)

On Arch Linux:
```sh
sudo pacman -S --needed cmake ninja clang sdl3 glm shaderc tomlplusplus
# optional: Vulkan validation layers (Debug builds), ccache, RenderDoc
sudo pacman -S --needed vulkan-validation-layers ccache renderdoc
```
Dear ImGui is downloaded at configure time; GoogleTest, SDL3, GLM and toml++ are used from the system when
installed, otherwise downloaded too.

## Build and run
```sh
cmake --workflow --preset dev          # configure + build + test, Clang Debug
./build/clang-debug/bin/StarshipSimulator
```

Controls: click the view to capture the mouse (Esc releases it). WASD to move, Shift to run, Space to jump
(walk) or rise (fly), Ctrl to descend, F toggles walking/flying, G throws a ball (its path is compared with the
same throw on a planet), C toggles comfort mode (no Coriolis force on you), mouse wheel sets fly speed, Tab opens
the habitat editor, F1 toggles the HUD, F5 reloads shaders, F12 saves a screenshot to
`~/.local/share/StarshipSimulator/screenshots/`. Saved habitats go to `~/.local/share/StarshipSimulator/habitats/`.

Useful options (`--help` lists all):
```sh
StarshipSimulator --view lookup                       # start looking up at the far side
StarshipSimulator --mirror 30                         # morning: 45 is noon, 90 sunset, more is night
StarshipSimulator --scenario data/presets/coriolis_playground.toml   # a small, fast-spinning habitat
StarshipSimulator --capture shot.png --capture-ui     # render, save a PNG, exit
StarshipSimulator --no-vsync --benchmark              # frame times over a fixed tour (use a Release build)
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
- `src/core`: habitat geometry, spin physics, procedural generation, scenario files; no graphics dependencies,
  fully unit tested
- `src/render`: the SDL_GPU renderer and the ImGui layer
- `src/app`: the executable (main loop, input, HUD)
- `shaders`: GLSL, compiled to SPIR-V at build time
- `data/presets`: habitat scenarios (Island Three, Coriolis Playground)

API docs: `cmake --build --preset clang-debug --target docs`, then open `build/clang-debug/docs/html/index.html`.
