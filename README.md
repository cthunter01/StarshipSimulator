# StarshipSimulator

Walk around inside space habitats (O'Neill cylinders, Bishop rings, starships), look out the windows or up at
the land overhead, and see an astronomically accurate sky. The goal: help people picture a positive future in
space that we could actually build, with the real physics of spin gravity (Coriolis drift, gravity that fades
toward the axis) and real stars and planets outside.

**Status: M7: share the vision.** Walk the valleys of Gerard O'Neill's Island Three, an 8 km wide, 32 km long
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
gives it the warm light of the 1970s habitat paintings (the "Painted colours" slider in the HUD).

Towns line the rivers: a dozen villages of plastered houses under terracotta roofs, with shuttered windows,
shops with striped awnings on tree-lined main streets, a square with a fountain and a hall with a bell tower,
stone bridges and a lamp-lit river front; farmsteads stand out in the fields. At night the windows and street
lamps light up. Everything is solid (Jolt Physics, in the habitat's spinning frame): you bump into walls and
trees, climb kerbs and steps, and can kick (E) the balls, crates, barrels, hay bales and cafe chairs lying
about; thrown balls bounce and roll, float on the water, and fall off course with the Coriolis force.

The habitat has weather and a year. A deck of cloud a few hundred metres up drifts with the wind, throws its
shadow on the land, and ends at a horizon of its own two dozen degrees above yours, so there is always a ring of
clear air under it through which you see the far side. It thickens and clears over the hours, rains (the drops
slant antispinward: Coriolis again), leaves the ground dark and wet, and lies as mist over the river on still
mornings. The mirrors run a year as well as a day, longer and shorter days by season, with spring blossom in the
orchards and whole woods turning gold in autumn. Flocks of birds wheel over the fields. And you can hear it:
wind, leaves, running water, rain, birdsong (crickets after dark), the murmur of a town and your own footsteps,
all synthesised on the fly rather than played from files.

The habitat is inhabited. People walk the streets of its towns, stand about the square and the market stalls
and sit on the benches, fewer of them out after dark and in the rain; you bump into them rather than walking
through. A tramway runs the length of every valley on a ballasted track, calling at each town and at halts out
in the fields; stand on a tram and it carries you along. The lines are engineered rather than draped
over the land: each alignment is smoothed until it could be built, the ground is cut and filled to
meet it with embankments and cuttings sloped at a constant angle, and where the land falls away the
track crosses it on a braced timber trestle. At the antisunward end a funicular climbs seven
kilometres of endcap ramp to the hub at the axis, where the spin gravity has all but gone. And there you can strap on a pair of wings (V): a real aerofoil in the habitat's own
air, which glides but cannot be kept up by muscle alone down in the valley, and which a fit person can climb on
near the axis — the oldest promise of an O'Neill cylinder.

The place explains itself. Press K for the almanac: seven pages on what this habitat is, in numbers and plain
words, worked out from the one you are standing in rather than quoted from a book — the drop of a dropped ball
is flown through the same free-flight code the ball itself uses. Guided tours (in the HUD, or `--tour 1`) fly
you around and talk: the whole place in five minutes, how the mirrors make a day, and what the sky outside is
doing; moving takes the controls back. F2 is photo mode: the HUD gets out of the way, and you can hold the
shutter open for star trails or save a picture at four times the screen's size. The habitat editor (Tab) is the
whole file, not just the shape: valleys, endcaps, mirrors and the day they keep, terrain, rivers, woods, towns,
air, clouds, the length of the year, where in the solar system it flies and when the visit starts, with every
derived number updating as you drag and every problem spelled out before you can build it. The gallery lists
the habitats that came with the program and the ones you have saved, each a small TOML file; a determinism test
pins a whole generated world to a hash, so the file you send someone builds the same world on their machine,
under either compiler. Next up, M8: more revolved worlds.

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
Dear ImGui and Jolt Physics (v5.6.0, built in double precision) are downloaded at configure time; GoogleTest,
SDL3, GLM and toml++ are used from the system when installed, otherwise downloaded too. The first configure also downloads the sky data (about 52 MB: star catalog,
Milky Way, Earth and Moon maps) into `build/_downloads/sky`; turn that off with
`-DSTARSHIPSIMULATOR_DOWNLOAD_SKY_DATA=OFF` (the app then shows placeholder stars). See `data/CREDITS.md`.

## Build and run
```sh
cmake --workflow --preset dev          # configure + build + test, Clang Debug
./build/clang-debug/bin/StarshipSimulator
```

Controls: click the view to capture the mouse (Esc releases it). WASD to move, Shift to run, Space to jump
(walk), rise (fly) or flap (wings), Ctrl to descend, F toggles walking/flying, V straps on a pair of wings,
K opens the almanac (what this place is, in numbers), F2 is photo mode (long exposures and enlarged pictures),
G throws a ball (its path is compared with the
same throw on a planet), E kicks whatever is in front of you, C toggles comfort mode (no Coriolis force on you), mouse wheel sets fly speed, Tab opens
the habitat editor, I names the star, planet or moon under the crosshair, B toggles binoculars, P pauses time,
comma and period slow down and speed up time (up to a day per second), F1 toggles the HUD, F5 reloads shaders,
F12 saves a screenshot to
`~/.local/share/StarshipSimulator/screenshots/`. Saved habitats go to `~/.local/share/StarshipSimulator/habitats/`.

Useful options (`--help` lists all):
```sh
StarshipSimulator --view lookup                       # start looking up at the far side (or river, lake, ...)
StarshipSimulator --view town                         # on a town square (or street, rooftops)
StarshipSimulator --view tram                         # on a tram platform (or lift, hub)
StarshipSimulator --time 2045-06-15T23:00             # night (UTC; the habitat's day runs 06:00-20:00)
StarshipSimulator --look-at earth                     # look out of a window at Earth (or moon, jupiter, Vega, partner)
StarshipSimulator --time-scale 3600                   # an hour per second: watch the day go by
StarshipSimulator --mirror 30                         # hold the mirrors: 45 is noon, 90 sunset, more is night
StarshipSimulator --weather rain                      # hold the weather: clear, fair, cloudy, overcast, mist, rain, storm
StarshipSimulator --mute                              # no sound
StarshipSimulator --scenario data/presets/coriolis_playground.toml   # a small, fast-spinning habitat
StarshipSimulator --tour 1                            # set off on a guided tour (1, 2, 3, or part of its name)
StarshipSimulator --panel gallery                     # open a panel at startup (editor, gallery, almanac)
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
- `src/physics`: rigid bodies and the player's body (Jolt Physics) in the habitat's spinning frame
- `src/render`: the SDL_GPU renderer and the ImGui layer
- `src/audio`: plays the synthesised soundscape through SDL's audio device
- `src/app`: the executable (main loop, input, HUD)
- `shaders`: GLSL, compiled to SPIR-V at build time
- `data/presets`: habitat scenarios (Island Three, Coriolis Playground)
- `third_party`: Astronomy Engine and stb, vendored

API docs: `cmake --build --preset clang-debug --target docs`, then open `build/clang-debug/docs/html/index.html`.
