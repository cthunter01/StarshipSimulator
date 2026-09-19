# StarshipSimulator

Walk around inside space habitats (O'Neill cylinders, Bishop rings, starships), look out the windows or up at
the land overhead, and see an astronomically accurate sky. The goal: help people picture a positive future in
space that we could actually build, with the real physics of spin gravity (Coriolis drift, gravity that fades
toward the axis) and real stars and planets outside.

**Status: M3: a living valley.** Walk the valleys of Gerard O'Neill's Island Three, an 8 km wide, 32 km long
cylinder at the Earth-Moon L5 point: farmland curving up overhead through blue haze, three window strips, three
mirrors bringing in the sunlight, mountain ramps at one end and a dome at the other. The physics is real:
gravity comes from spin (and fades as you climb toward the axis), jumps and thrown balls drift from the Coriolis
force.

Outside the windows is the real sky of any date you choose: 25,000 stars from the HYG catalog, NASA's Milky Way,
the planets, and Earth (two degrees across, with its phases, clouds and city lights at night) and the Moon, all
at their true positions, sweeping past as the habitat turns every two minutes. The mirrors swing through a day
schedule: morning, noon, sunset, and a night with the stars. Alongside flies the counter-rotating partner
cylinder, 80 km away. Press I to name the star or planet under the crosshair, B for binoculars. A habitat
editor lets you design your own cylinder and save it as a small TOML file.

The valleys are alive: a river meanders down each one, widening into lakes, between patchwork fields, meadows
and woods of 2.7 million trees (oaks, pines and riverside poplars) that cast soft shadows, as do the hills. The
land stays sharp from your feet to the far side 8 km overhead without popping as you move, and a colour grade
gives it the warm light of the 1970s habitat paintings (the "Painted colours" slider in the HUD). Next up, M4:
towns.

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
installed, otherwise downloaded too. The first configure also downloads the sky data (about 52 MB: star catalog,
Milky Way, Earth and Moon maps) into `build/_downloads/sky`; turn that off with
`-DSTARSHIPSIMULATOR_DOWNLOAD_SKY_DATA=OFF` (the app then shows placeholder stars). See `data/CREDITS.md`.

## Build and run
```sh
cmake --workflow --preset dev          # configure + build + test, Clang Debug
./build/clang-debug/bin/StarshipSimulator
```

Controls: click the view to capture the mouse (Esc releases it). WASD to move, Shift to run, Space to jump
(walk) or rise (fly), Ctrl to descend, F toggles walking/flying, G throws a ball (its path is compared with the
same throw on a planet), C toggles comfort mode (no Coriolis force on you), mouse wheel sets fly speed, Tab opens
the habitat editor, I names the star, planet or moon under the crosshair, B toggles binoculars, P pauses time,
comma and period slow down and speed up time (up to a day per second), F1 toggles the HUD, F5 reloads shaders,
F12 saves a screenshot to
`~/.local/share/StarshipSimulator/screenshots/`. Saved habitats go to `~/.local/share/StarshipSimulator/habitats/`.

Useful options (`--help` lists all):
```sh
StarshipSimulator --view lookup                       # start looking up at the far side (or river, lake, ...)
StarshipSimulator --time 2045-06-15T23:00             # night (UTC; the habitat's day runs 06:00-20:00)
StarshipSimulator --look-at earth                     # look out of a window at Earth (or moon, jupiter, Vega, partner)
StarshipSimulator --time-scale 3600                   # an hour per second: watch the day go by
StarshipSimulator --mirror 30                         # hold the mirrors: 45 is noon, 90 sunset, more is night
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
- `src/core`: habitat geometry, spin physics, procedural generation, scenario files, astronomy (time, positions,
  star catalog) and data decoding; no graphics dependencies, fully unit tested
- `src/render`: the SDL_GPU renderer and the ImGui layer
- `src/app`: the executable (main loop, input, HUD)
- `shaders`: GLSL, compiled to SPIR-V at build time
- `data/presets`: habitat scenarios (Island Three, Coriolis Playground)
- `third_party`: Astronomy Engine and stb, vendored

API docs: `cmake --build --preset clang-debug --target docs`, then open `build/clang-debug/docs/html/index.html`.
